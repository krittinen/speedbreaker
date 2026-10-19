// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
//
// A requested termination ends the game the way its window's close button
// does: main.cpp's quit path (settings saved, the renderer, UI and window
// shut down, the session's crash marker cleared, the log flushed, exit code 0).
//   - SIGTERM: a shutdown or logout (systemd), `kill`, and the way a launcher
//     such as Steam asks a game to end (Steam's own reaper only forks, waits
//     and reaps: the stop comes from the Steam client; SIGKILL, which can't
//     be caught, is a stop that doesn't wait);
//   - SIGINT: Ctrl+C in the terminal the game was started from;
//   - SIGHUP: that terminal closed.
// Until v0.1.6 SDL turned SIGTERM and SIGINT into its quit event (SDL puts its
// handler only where a signal's action is still the default) once the window
// was up, and SIGHUP, or SIGTERM before the window, ended the game on the
// spot: no settings saved, and the crash marker left as by a crash
// (report/crash_marker.h).
//
// The handler only records the first request (async-signal-safe); the main
// thread's frame loop (video::RunFrame, which also runs the installer and the
// startup error page) reads it and leaves; with no window (NFSMW_HEADLESS=1)
// main() waits for it. Another request while quitting changes nothing, as
// SDL's didn't. A signal the game was started with set to be ignored
// (nohup's SIGHUP) stays ignored. SIGKILL can't be caught: a session it ends
// keeps its crash marker, and the next launch logs that it ended without
// quitting.
//
// Header only (tests/clean_exit_test.cpp).
#pragma once
#include <atomic>
#include <cstdio>
#ifdef _WIN32
#include <windows.h>
#else
#include <csignal>
#endif

namespace quitsignals
{
    namespace detail
    {
        inline std::atomic<int> g_requested{ 0 };
        static_assert(std::atomic<int>::is_always_lock_free, "the handler needs a lock-free atomic");

        inline void Handle(int sig)
        {
            int none = 0;
            g_requested.compare_exchange_strong(none, sig, std::memory_order_relaxed);
        }

#ifdef _WIN32
        inline BOOL WINAPI ConsoleHandler(DWORD event)
        {
            switch (event)
            {
            case CTRL_C_EVENT:
            case CTRL_BREAK_EVENT:
                Handle(SIGINT);
                return TRUE;
            case CTRL_CLOSE_EVENT:
            case CTRL_LOGOFF_EVENT:
            case CTRL_SHUTDOWN_EVENT:
                Handle(SIGTERM);
                return TRUE;
            default:
                return FALSE;
            }
        }
#endif
    }

    // The signals a player or the system ends the game with.
#ifdef _WIN32
    inline constexpr int kSignals[] = { SIGTERM, SIGINT };
#else
    inline constexpr int kSignals[] = { SIGTERM, SIGINT, SIGHUP };
#endif

    // Main thread, before SDL starts (video::Initialize), so SDL leaves these
    // signals to us.
    inline void Install()
    {
#ifdef _WIN32
        if (!SetConsoleCtrlHandler(detail::ConsoleHandler, TRUE))
            fprintf(stderr, "[runtime] SetConsoleCtrlHandler failed with error %lu\n",
                static_cast<unsigned long>(GetLastError()));
#else
        for (int sig : kSignals)
        {
            struct sigaction old{};
            sigaction(sig, nullptr, &old);
            if (!(old.sa_flags & SA_SIGINFO) && old.sa_handler == SIG_IGN)
                continue;
            struct sigaction sa{};
            sa.sa_handler = detail::Handle;
            sigemptyset(&sa.sa_mask);
            sa.sa_flags = SA_RESTART;
            sigaction(sig, &sa, nullptr);
        }
#endif
    }

    // The signal that asked the game to quit, 0 if none has. Any thread.
    inline int Requested()
    {
        return detail::g_requested.load(std::memory_order_relaxed);
    }

    // For the log line.
    inline const char* Name(int sig)
    {
        switch (sig)
        {
        case SIGTERM: return "SIGTERM";
        case SIGINT: return "SIGINT";
#ifndef _WIN32
        case SIGHUP: return "SIGHUP";
#endif
        default: return "a signal";
        }
    }
}
