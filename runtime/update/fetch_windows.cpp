// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING). See fetch.h.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winhttp.h>

#include "fetch.h"

#include <array>
#include <algorithm>
#include <format>
#include <climits>
#include <cmath>

namespace update
{
    namespace
    {
        struct Api
        {
            HMODULE module = LoadLibraryW(L"winhttp.dll");
            decltype(&::WinHttpOpen) open = nullptr;
            decltype(&::WinHttpCloseHandle) close = nullptr;
            decltype(&::WinHttpCrackUrl) crackUrl = nullptr;
            decltype(&::WinHttpSetOption) setOption = nullptr;
            decltype(&::WinHttpSetTimeouts) setTimeouts = nullptr;
            decltype(&::WinHttpConnect) connect = nullptr;
            decltype(&::WinHttpOpenRequest) openRequest = nullptr;
            decltype(&::WinHttpAddRequestHeaders) addRequestHeaders = nullptr;
            decltype(&::WinHttpSendRequest) sendRequest = nullptr;
            decltype(&::WinHttpReceiveResponse) receiveResponse = nullptr;
            decltype(&::WinHttpQueryHeaders) queryHeaders = nullptr;
            decltype(&::WinHttpReadData) readData = nullptr;

            template <typename T>
            bool Load(T& function, const char* name)
            {
                function = module ? reinterpret_cast<T>(GetProcAddress(module, name)) : nullptr;
                return function != nullptr;
            }

            Api()
            {
                bool ready = Load(open, "WinHttpOpen") && Load(close, "WinHttpCloseHandle") &&
                    Load(crackUrl, "WinHttpCrackUrl") && Load(setOption, "WinHttpSetOption") &&
                    Load(setTimeouts, "WinHttpSetTimeouts") && Load(connect, "WinHttpConnect") &&
                    Load(openRequest, "WinHttpOpenRequest") && Load(addRequestHeaders, "WinHttpAddRequestHeaders") &&
                    Load(sendRequest, "WinHttpSendRequest") && Load(receiveResponse, "WinHttpReceiveResponse") &&
                    Load(queryHeaders, "WinHttpQueryHeaders") && Load(readData, "WinHttpReadData");
                if (!ready)
                {
                    if (module)
                        FreeLibrary(module);
                    module = nullptr;
                }
            }

            ~Api() { if (module) FreeLibrary(module); }
            explicit operator bool() const { return module != nullptr; }
        };

        bool Wide(const std::string& text, std::wstring& result)
        {
            if (text.empty())
            {
                result.clear();
                return true;
            }
            int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), int(text.size()), nullptr, 0);
            if (count <= 0)
                return false;
            result.resize(size_t(count));
            return MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(), int(text.size()),
                result.data(), count) == count;
        }

        bool Utf8(const wchar_t* text, size_t length, std::string& result)
        {
            if (length == 0)
            {
                result.clear();
                return true;
            }
            int count = WideCharToMultiByte(CP_UTF8, 0, text, int(length), nullptr, 0, nullptr, nullptr);
            if (count <= 0)
                return false;
            result.resize(size_t(count));
            return WideCharToMultiByte(CP_UTF8, 0, text, int(length), result.data(), count, nullptr, nullptr) == count;
        }

        struct Handle
        {
            Api* api = nullptr;
            HINTERNET value = nullptr;
            ~Handle() { if (value) api->close(value); }
            operator HINTERNET() const { return value; }
        };

        Transport Classify(DWORD error)
        {
            switch (error)
            {
            case ERROR_WINHTTP_NAME_NOT_RESOLVED:
            case ERROR_WINHTTP_CANNOT_CONNECT:
            case ERROR_WINHTTP_CONNECTION_ERROR:
            case ERROR_WINHTTP_CANNOT_CALL_BEFORE_OPEN:
            case ERROR_WINHTTP_OPERATION_CANCELLED:
                return Transport::Offline;
            case ERROR_WINHTTP_TIMEOUT:
                return Transport::Timeout;
            case ERROR_WINHTTP_SECURE_FAILURE:
                return Transport::Secure;
            default:
                return Transport::Failed;
            }
        }

        void SetFailure(Fetched& result, DWORD error)
        {
            result.transport = Classify(error);
            result.detail = std::format("WinHTTP error {}", error);
        }
    }

    Fetched Fetch(const Request& request)
    {
        Fetched result;
        Api api;
        if (!api)
        {
            result.transport = Transport::Unavailable;
            result.detail = "WinHTTP is not available on this Windows installation.";
            return result;
        }
        std::wstring url, userAgent;
        if (!Wide(request.url, url) || !Wide(request.userAgent.empty() ? "SpeedBreaker" : request.userAgent, userAgent))
        {
            result.transport = Transport::Failed;
            result.detail = "The URL or user agent is not valid UTF-8.";
            return result;
        }

        URL_COMPONENTS parts{};
        parts.dwStructSize = sizeof(parts);
        parts.dwHostNameLength = DWORD(-1);
        parts.dwUrlPathLength = DWORD(-1);
        parts.dwExtraInfoLength = DWORD(-1);
        if (!api.crackUrl(url.c_str(), DWORD(url.size()), 0, &parts) ||
            parts.dwHostNameLength == 0 || parts.dwUserNameLength != 0 || parts.dwPasswordLength != 0)
        {
            result.transport = Transport::Garbled;
            result.detail = "The update URL is invalid.";
            return result;
        }
        bool secure = parts.nScheme == INTERNET_SCHEME_HTTPS;
        if (!secure && (parts.nScheme != INTERNET_SCHEME_HTTP || !request.allowHttp))
        {
            result.transport = Transport::Secure;
            result.detail = "Only HTTPS update requests are allowed.";
            return result;
        }

        const double timeoutSeconds = std::isfinite(request.timeoutSeconds) ? request.timeoutSeconds : kTimeoutSeconds;
        const ULONGLONG timeoutMs = ULONGLONG(std::clamp(timeoutSeconds, 0.1,
            double(INT_MAX) / 1000.0) * 1000.0);
        const ULONGLONG deadline = GetTickCount64() + timeoutMs;
        auto setTimeout = [&](HINTERNET handle) {
            ULONGLONG now = GetTickCount64();
            if (now >= deadline)
                return false;
            int remaining = int(std::min<ULONGLONG>(deadline - now, INT_MAX));
            return api.setTimeouts(handle, remaining, remaining, remaining, remaining) != FALSE;
        };

        Handle session{ &api, api.open(userAgent.c_str(), WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
            WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0) };
        if (!session.value)
        {
            SetFailure(result, GetLastError());
            return result;
        }
        DWORD disabled = WINHTTP_DISABLE_COOKIES;
        DWORD logon = WINHTTP_AUTOLOGON_SECURITY_LEVEL_HIGH;
        if (!api.setOption(session, WINHTTP_OPTION_DISABLE_FEATURE, &disabled, sizeof(disabled)) ||
            !api.setOption(session, WINHTTP_OPTION_AUTOLOGON_POLICY, &logon, sizeof(logon)))
        {
            SetFailure(result, GetLastError());
            return result;
        }
        if (!setTimeout(session))
        {
            result.transport = Transport::Timeout;
            result.detail = "The update request timed out.";
            return result;
        }

        std::wstring host(parts.lpszHostName, parts.dwHostNameLength);
        std::wstring path(parts.lpszUrlPath, parts.dwUrlPathLength);
        if (path.empty())
            path = L"/";
        if (parts.dwExtraInfoLength)
            path.append(parts.lpszExtraInfo, parts.dwExtraInfoLength);
        Handle connection{ &api, api.connect(session, host.c_str(), parts.nPort, 0) };
        if (!connection.value)
        {
            SetFailure(result, GetLastError());
            return result;
        }
        if (!setTimeout(session))
        {
            result.transport = Transport::Timeout;
            result.detail = "The update request timed out.";
            return result;
        }
        Handle handle{ &api, api.openRequest(connection, L"GET", path.c_str(), nullptr,
            WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, secure ? WINHTTP_FLAG_SECURE : 0) };
        if (!handle.value)
        {
            SetFailure(result, GetLastError());
            return result;
        }
        DWORD redirect = WINHTTP_OPTION_REDIRECT_POLICY_NEVER;
        if (!api.setOption(handle, WINHTTP_OPTION_REDIRECT_POLICY, &redirect, sizeof(redirect)))
        {
            SetFailure(result, GetLastError());
            return result;
        }
        if (!setTimeout(session))
        {
            result.transport = Transport::Timeout;
            result.detail = "The update request timed out.";
            return result;
        }

        for (const auto& [name, value] : request.headers)
        {
            std::wstring wideName, wideValue;
            if (!Wide(name, wideName) || !Wide(value, wideValue))
            {
                result.transport = Transport::Failed;
                result.detail = "An update request header is not valid UTF-8.";
                return result;
            }
            std::wstring header = wideName + L": " + wideValue + L"\r\n";
            if (!api.addRequestHeaders(handle, header.c_str(), DWORD(header.size()),
                    WINHTTP_ADDREQ_FLAG_ADD | WINHTTP_ADDREQ_FLAG_REPLACE))
            {
                SetFailure(result, GetLastError());
                return result;
            }
        }

        if (!setTimeout(session))
        {
            result.transport = Transport::Timeout;
            result.detail = "The update request timed out.";
            return result;
        }
        if (!api.sendRequest(handle, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0))
        {
            DWORD error = GetLastError();
            SetFailure(result, error);
            return result;
        }
        if (!setTimeout(session))
        {
            result.transport = Transport::Timeout;
            result.detail = "The update request timed out.";
            return result;
        }
        if (!api.receiveResponse(handle, nullptr))
        {
            SetFailure(result, GetLastError());
            return result;
        }

        DWORD status = 0, statusSize = sizeof(status);
        if (!api.queryHeaders(handle, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                WINHTTP_HEADER_NAME_BY_INDEX, &status, &statusSize, WINHTTP_NO_HEADER_INDEX))
        {
            SetFailure(result, GetLastError());
            return result;
        }
        DWORD headerSize = 0;
        api.queryHeaders(handle, WINHTTP_QUERY_RAW_HEADERS_CRLF, WINHTTP_HEADER_NAME_BY_INDEX,
            nullptr, &headerSize, WINHTTP_NO_HEADER_INDEX);
        if (GetLastError() != ERROR_INSUFFICIENT_BUFFER || headerSize < sizeof(wchar_t))
        {
            result.transport = Transport::Garbled;
            result.detail = "The server returned invalid HTTP headers.";
            return result;
        }
        if (headerSize > (1u << 20))
        {
            result.transport = Transport::TooLarge;
            result.detail = "The update response headers exceeded their size limit.";
            return result;
        }
        std::vector<wchar_t> rawHeaders(headerSize / sizeof(wchar_t));
        if (!api.queryHeaders(handle, WINHTTP_QUERY_RAW_HEADERS_CRLF, WINHTTP_HEADER_NAME_BY_INDEX,
                rawHeaders.data(), &headerSize, WINHTTP_NO_HEADER_INDEX))
        {
            SetFailure(result, GetLastError());
            return result;
        }
        std::string raw;
        if (!Utf8(rawHeaders.data(), wcslen(rawHeaders.data()), raw) || !ParseHttpResponse(raw, result.response))
        {
            result.transport = Transport::Garbled;
            result.detail = "The server returned invalid HTTP headers.";
            return result;
        }
        result.response.status = int(status);

        std::array<char, 16384> buffer{};
        for (;;)
        {
            DWORD received = 0;
            if (!setTimeout(session))
            {
                result.transport = Transport::Timeout;
                result.detail = "The update request timed out.";
                return result;
            }
            if (!api.readData(handle, buffer.data(), DWORD(buffer.size()), &received))
            {
                SetFailure(result, GetLastError());
                return result;
            }
            if (received == 0)
                break;
            if (result.response.body.size() + received > request.maxBytes)
            {
                result.transport = Transport::TooLarge;
                result.detail = "The update response exceeded its size limit.";
                return result;
            }
            result.response.body.append(buffer.data(), received);
        }
        result.transport = Transport::Ok;
        return result;
    }
}
#endif
