// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
#include <stdafx.h>
#include "report.h"
#include "internal.h"

#include <cpu/ppc_context.h>
#include <gpu/command_processor.h>
#include <gpu/renderer.h>
#include <ui/ui.h>

#include <fstream>
#include <string_view>

namespace report
{
    namespace
    {
        std::atomic<bool> s_crashing{ false };
        std::chrono::steady_clock::time_point s_started;
        std::filesystem::path s_pendingPath;
        std::filesystem::path s_finalPath;

        const char* ExceptionName(DWORD code)
        {
            switch (code)
            {
            case EXCEPTION_ACCESS_VIOLATION: return "access violation";
            case EXCEPTION_ARRAY_BOUNDS_EXCEEDED: return "array bounds exceeded";
            case EXCEPTION_BREAKPOINT: return "breakpoint";
            case EXCEPTION_DATATYPE_MISALIGNMENT: return "datatype misalignment";
            case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "floating-point divide by zero";
            case EXCEPTION_ILLEGAL_INSTRUCTION: return "illegal instruction";
            case EXCEPTION_INT_DIVIDE_BY_ZERO: return "integer divide by zero";
            case EXCEPTION_STACK_OVERFLOW: return "stack overflow";
            default: return "unhandled exception";
            }
        }

        std::string SessionHeader()
        {
            return std::format(
                "SpeedBreaker (NFS: Most Wanted, Xbox 360): crash report\n"
                "Build: SpeedBreaker {}\n"
                "Session: {} (log: {}.log)\n\n",
                BuildString(), SessionName(), SessionName());
        }
    }

    void PrepareCrashReports()
    {
        s_started = std::chrono::steady_clock::now();
        std::error_code ec;
        const auto directory = LogDirectory();
        std::filesystem::create_directories(directory, ec);
        s_pendingPath = directory / (".crash-pending-" + std::string(SessionName()));
        s_finalPath = directory / ("crash-" + std::string(SessionName()) + ".txt");
        std::ofstream marker(s_pendingPath, std::ios::binary | std::ios::trunc);
        if (!marker)
            fprintf(stderr, "[crash] can't prepare a crash report file in %s\n", directory.string().c_str());
    }

    void WriteCrashReport(_EXCEPTION_POINTERS* exception)
    {
        if (s_crashing.exchange(true, std::memory_order_acq_rel))
            return;
        FlushLog();

        const EXCEPTION_RECORD& record = *exception->ExceptionRecord;
        const CONTEXT& context = *exception->ContextRecord;
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - s_started).count();
        std::ofstream out(s_pendingPath, std::ios::binary | std::ios::app);
        if (out)
        {
            out << SessionHeader()
                << "[crash] " << ExceptionName(record.ExceptionCode)
                << " (0x" << std::hex << record.ExceptionCode << ") at 0x"
                << reinterpret_cast<uintptr_t>(record.ExceptionAddress) << std::dec
                << " in thread " << GetCurrentThreadId() << ", " << elapsed
                << " seconds into the session, " << gpu::FrameCount() << " game frames\n"
                << "[crash] host registers: rip=0x" << std::hex << context.Rip
                << " rsp=0x" << context.Rsp << " rbp=0x" << context.Rbp << std::dec << '\n';
            if (PPCContext* guest = GetPPCContext())
                out << "[crash] guest registers: lr=" << std::hex << uint32_t(guest->lr)
                    << " r1=" << guest->r1.u32 << " r3=" << guest->r3.u32
                    << " r4=" << guest->r4.u32 << " r5=" << guest->r5.u32
                    << " r31=" << guest->r31.u32 << std::dec << '\n';
            out << "\nThe log's last lines:\n" << RecentOutput(300)
                << "\nSystem (at startup):\n" << detail::LoggedSystemSummary();
            out.flush();
            out.close();
            MoveFileExW(s_pendingPath.c_str(), s_finalPath.c_str(),
                MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
        }
        fprintf(stderr, "[crash] %s at 0x%p; report: %s\n", ExceptionName(record.ExceptionCode),
            record.ExceptionAddress, s_finalPath.string().c_str());
    }

    void ClearCrashMarker()
    {
        if (!s_crashing.load(std::memory_order_acquire) && !s_pendingPath.empty())
        {
            std::error_code ec;
            std::filesystem::remove(s_pendingPath, ec);
        }
    }

    void NoticePreviousCrash(bool lastClearsMarker)
    {
        std::filesystem::path directory = LogDirectory();
        std::error_code ec;
        for (std::filesystem::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec))
        {
            std::string name = it->path().filename().string();
            constexpr std::string_view prefix = ".crash-pending-";
            if (!name.starts_with(prefix) || name == s_pendingPath.filename().string())
                continue;
            std::error_code sizeError;
            uintmax_t size = std::filesystem::file_size(it->path(), sizeError);
            if (!sizeError && size != 0)
            {
                std::filesystem::path destination = directory / ("crash-" + name.substr(prefix.size()) + ".txt");
                std::filesystem::rename(it->path(), destination, ec);
            }
            else
            {
                std::filesystem::remove(it->path(), ec);
                if (lastClearsMarker)
                    fprintf(stderr, "[crash] an earlier session (%s) ended without quitting or a report\n",
                        name.substr(prefix.size()).c_str());
            }
            ec.clear();
        }

        detail::KeepNewest(directory.string(), "crash-", ".txt", 5);
        std::string newest;
        for (std::filesystem::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec))
        {
            std::string name = it->path().filename().string();
            if (name.starts_with("crash-") && name.ends_with(".txt") && name > newest)
                newest = std::move(name);
        }
        if (newest.empty())
            return;
        const std::filesystem::path seenPath = directory / ".crash-seen";
        std::string seen;
        {
            std::ifstream in(seenPath);
            std::getline(in, seen);
        }
        if (seen == newest)
            return;
        fprintf(stderr, "[crash] an earlier session crashed: %s\n", (directory / newest).string().c_str());
        ui::Toast(std::format("The game crashed last time and saved a crash report. To send it: {}.",
            detail::kSaveBugReportHint), 15.0);
        std::ofstream(seenPath, std::ios::trunc) << newest << '\n';
    }

    [[noreturn]] void TestCrash()
    {
        const char* value = std::getenv("NFSMW_TEST_CRASH");
        if (value && std::strstr(value, ":abort"))
        {
            fprintf(stderr, "[test] NFSMW_TEST_CRASH: abort\n");
            std::abort();
        }
        ULONG_PTR parameters[] = { 1, 0x10 };
        fprintf(stderr, "[test] NFSMW_TEST_CRASH: raising an access violation\n");
        RaiseException(EXCEPTION_ACCESS_VIOLATION, 0, 2, parameters);
        std::abort();
    }
}
