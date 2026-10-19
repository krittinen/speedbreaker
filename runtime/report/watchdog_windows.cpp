// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
#include <stdafx.h>
#include "report.h"

#include <apu/audio.h>
#include <cpu/ppc_context.h>
#include <cpu/guest_time.h>
#include <gpu/command_processor.h>
#include <gpu/renderer.h>
#include "internal.h"

#include <dbghelp.h>
#include <fstream>

namespace report
{
    namespace
    {
        struct ThreadSlot
        {
            std::atomic<bool> used{ false };
            DWORD id = 0;
            char name[32]{};
            PPCContext* guest = nullptr;
        };
        constexpr size_t kMaxThreads = 256;
        ThreadSlot s_threads[kMaxThreads];
        std::mutex s_threadsMutex;
        std::atomic<bool> s_started{ false };
        thread_local int t_slot = -1;

        struct Unregister
        {
            ~Unregister()
            {
                if (t_slot < 0)
                    return;
                std::lock_guard lock(s_threadsMutex);
                s_threads[t_slot].guest = nullptr;
                s_threads[t_slot].used.store(false, std::memory_order_release);
            }
        };
        thread_local Unregister t_unregister;

        struct ThreadInfo
        {
            DWORD id;
            std::string name;
            bool hasGuest;
            uint32_t lr;
            uint32_t r1;
        };

        std::vector<ThreadInfo> Threads()
        {
            std::vector<ThreadInfo> result;
            std::lock_guard lock(s_threadsMutex);
            for (const ThreadSlot& slot : s_threads)
            {
                if (!slot.used.load(std::memory_order_acquire))
                    continue;
                ThreadInfo info{ slot.id, slot.name, slot.guest != nullptr, 0, 0 };
                if (slot.guest)
                {
                    info.lr = uint32_t(slot.guest->lr);
                    info.r1 = slot.guest->r1.u32;
                }
                result.push_back(std::move(info));
            }
            return result;
        }

        std::string Stack(DWORD id)
        {
            HANDLE thread = OpenThread(THREAD_GET_CONTEXT | THREAD_SUSPEND_RESUME | THREAD_QUERY_INFORMATION,
                FALSE, id);
            if (!thread)
                return "    (thread is no longer available)\n";
            if (SuspendThread(thread) == DWORD(-1))
            {
                CloseHandle(thread);
                return "    (couldn't suspend thread)\n";
            }

            CONTEXT context{};
            context.ContextFlags = CONTEXT_FULL;
            bool haveContext = GetThreadContext(thread, &context) != 0;
            ResumeThread(thread);
            if (!haveContext)
            {
                CloseHandle(thread);
                return "    (couldn't capture thread context)\n";
            }

            HANDLE process = GetCurrentProcess();
            static std::once_flag symbols;
            std::call_once(symbols, [&] {
                SymSetOptions(SYMOPT_DEFERRED_LOADS | SYMOPT_UNDNAME);
                SymInitialize(process, nullptr, TRUE);
            });

            STACKFRAME64 frame{};
            frame.AddrPC.Offset = context.Rip;
            frame.AddrPC.Mode = AddrModeFlat;
            frame.AddrStack.Offset = context.Rsp;
            frame.AddrStack.Mode = AddrModeFlat;
            frame.AddrFrame.Offset = context.Rbp;
            frame.AddrFrame.Mode = AddrModeFlat;
            std::string result;
            for (unsigned index = 0; index < 48 && frame.AddrPC.Offset; index++)
            {
                if (!StackWalk64(IMAGE_FILE_MACHINE_AMD64, process, thread, &frame, &context, nullptr,
                        SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
                    break;
                char storage[sizeof(SYMBOL_INFO) + MAX_SYM_NAME]{};
                auto* symbol = reinterpret_cast<SYMBOL_INFO*>(storage);
                symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
                symbol->MaxNameLen = MAX_SYM_NAME;
                DWORD64 displacement = 0;
                result += std::format("    #{:02} 0x{:016X}", index, frame.AddrPC.Offset);
                if (SymFromAddr(process, frame.AddrPC.Offset, &displacement, symbol))
                    result += std::format(" {}+0x{:X}", symbol->Name, displacement);
                result += '\n';
            }
            CloseHandle(thread);
            return result.empty() ? "    (no stack frames captured)\n" : result;
        }

        void WriteHangReport(double seconds, uint64_t bytes)
        {
            const std::string name = std::format("hang-{}-{}.txt", SessionName(), detail::TimeStamp());
            const std::filesystem::path path = LogDirectory() / name;
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            if (!out)
            {
                fprintf(stderr, "[watchdog] can't write hang report %s\n", path.string().c_str());
                return;
            }
            out << "SpeedBreaker (NFS: Most Wanted, Xbox 360): hang report\n"
                << "Build: SpeedBreaker " << BuildString() << '\n'
                << "Session: " << SessionName() << ", " << SessionSeconds() << " s in\n"
                << "No new frame for " << seconds << " s; " << bytes
                << " bytes read from game files in the last 10 s.\n\n"
                << "Command processor:\n" << gpu::DescribeState()
                << "\nAudio:\n" << apu::DescribeState()
                << "\nThreads (captured with Windows thread contexts):\n";
            for (const ThreadInfo& thread : Threads())
            {
                out << "  " << thread.name;
                if (thread.hasGuest)
                    out << std::format(" (guest lr={:08X} r1={:08X})", thread.lr, thread.r1);
                out << ":\n" << Stack(thread.id);
            }
            out << "\nThe log's last lines:\n" << RecentOutput(300);
            out.flush();
            fprintf(stderr, "[watchdog] hang report: %s\n", path.string().c_str());
        }

        void Watch()
        {
            uint64_t lastFrame = gpu::FrameCount();
            uint64_t lastBytes = g_fileBytesRead.load();
            auto lastFrameAt = std::chrono::steady_clock::now();
            auto readWindowAt = lastFrameAt;
            uint64_t readWindowBytes = 0;
            bool reported = false;
            for (;;)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                auto now = std::chrono::steady_clock::now();
                const uint64_t frames = gpu::FrameCount();
                const uint64_t bytes = g_fileBytesRead.load();
                if (frames != lastFrame)
                {
                    lastFrame = frames;
                    lastFrameAt = now;
                    reported = false;
                }
                if (bytes != lastBytes)
                {
                    readWindowBytes += bytes - lastBytes;
                    lastBytes = bytes;
                }
                if (now - readWindowAt >= std::chrono::seconds(10))
                {
                    readWindowBytes = 0;
                    readWindowAt = now;
                }
                double stalled = std::chrono::duration<double>(now - lastFrameAt).count();
                if (!reported && stalled >= 20.0 && readWindowBytes < (4ull << 20))
                {
                    WriteHangReport(stalled, readWindowBytes);
                    reported = true;
                }
            }
        }
    }

    void RegisterThread(const char* name)
    {
        std::lock_guard lock(s_threadsMutex);
        if (t_slot < 0)
        {
            for (size_t i = 0; i < kMaxThreads; i++)
                if (!s_threads[i].used.load(std::memory_order_relaxed))
                {
                    t_slot = int(i);
                    break;
                }
            if (t_slot < 0)
                return;
        }
        ThreadSlot& slot = s_threads[t_slot];
        slot.id = GetCurrentThreadId();
        snprintf(slot.name, sizeof(slot.name), "%s", name);
        slot.used.store(true, std::memory_order_release);
    }

    void SetThreadGuestContext(PPCContext* context)
    {
        if (t_slot < 0)
            return;
        std::lock_guard lock(s_threadsMutex);
        s_threads[t_slot].guest = context;
    }

    void StartWatchdog()
    {
        bool expected = false;
        if (s_started.compare_exchange_strong(expected, true))
            std::thread(Watch).detach();
    }
}
