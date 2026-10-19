// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING). See vblank_timer.h.
#include "vblank_timer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
#endif

#ifdef __APPLE__
#include <TargetConditionals.h>
#endif
// macOS only: iOS keeps the ordinary thread (its vblank timing hasn't been measured).
#if defined(__APPLE__) && !TARGET_OS_IOS
#define SB_REALTIME_VBLANK 1
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace video
{
    namespace
    {
#ifdef _WIN32
        thread_local HANDLE s_vblankTimer = nullptr;
        thread_local bool s_vblankTimerFailed = false;
#endif

#ifdef SB_REALTIME_VBLANK
        uint32_t ToMach(int64_t ns)
        {
            static const mach_timebase_info_data_t timebase = [] {
                mach_timebase_info_data_t tb{};
                mach_timebase_info(&tb);
                return tb;
            }();
            return uint32_t(double(std::max<int64_t>(ns, 0)) * double(timebase.denom) / double(timebase.numer));
        }
#endif

        int64_t Percentile(std::vector<int64_t> v, size_t permille)
        {
            if (v.empty())
                return 0;
            size_t k = std::min(v.size() - 1, v.size() * permille / 1000);
            std::nth_element(v.begin(), v.begin() + ptrdiff_t(k), v.end());
            return v[k];
        }
    }

    VblankTimer ChosenVblankTimer()
    {
        const char* v = std::getenv("NFSMW_VBLANK_TIMER");
        return v && std::string_view(v) == "sleep" ? VblankTimer::Ordinary : VblankTimer::RealTime;
    }

    bool PrepareVblankThread(VblankTimer timer, int64_t periodNs)
    {
#ifdef SB_REALTIME_VBLANK
        if (timer != VblankTimer::RealTime)
            return false;
        // Woken at its time, ahead of the game's own threads: it runs the
        // guest's vblank handler (microseconds) and sleeps again.
        thread_time_constraint_policy_data_t policy{};
        policy.period = ToMach(periodNs);
        policy.computation = ToMach(1'000'000);
        policy.constraint = ToMach(2'000'000);
        policy.preemptible = 1;
        return thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
                   reinterpret_cast<thread_policy_t>(&policy), THREAD_TIME_CONSTRAINT_POLICY_COUNT) == KERN_SUCCESS;
#else
#ifdef _WIN32
        if (timer == VblankTimer::RealTime)
        {
            (void)periodNs;
            s_vblankTimer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                TIMER_MODIFY_STATE | SYNCHRONIZE);
            if (!s_vblankTimer)
                fprintf(stderr, "[vblank] couldn't create high-resolution waitable timer (Windows error %lu); "
                    "falling back to sleep_until\n", static_cast<unsigned long>(GetLastError()));
            return s_vblankTimer != nullptr;
        }
#else
        (void)timer;
        (void)periodNs;
#endif
        return false;
#endif
    }

    void WaitVblankUntil(VblankTimer timer, int64_t deadlineNs)
    {
#ifdef _WIN32
        if (timer == VblankTimer::RealTime && s_vblankTimer && !s_vblankTimerFailed)
        {
            using namespace std::chrono;
            const int64_t remainingNs =
                deadlineNs - duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
            if (remainingNs <= 0)
                return;
            LARGE_INTEGER due{};
            due.QuadPart = -((remainingNs + 99) / 100);
            if (SetWaitableTimer(s_vblankTimer, &due, 0, nullptr, nullptr, FALSE))
            {
                if (WaitForSingleObject(s_vblankTimer, INFINITE) == WAIT_OBJECT_0)
                    return;
            }
            const DWORD error = GetLastError();
            s_vblankTimerFailed = true;
            fprintf(stderr, "[vblank] high-resolution waitable timer failed (Windows error %lu); falling back to sleep_until\n",
                static_cast<unsigned long>(error));
        }
#else
        (void)timer;
#endif
        std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(deadlineNs)));
    }

    const char* VblankTimerName(VblankTimer timer, bool realtime)
    {
#ifdef SB_REALTIME_VBLANK
        if (timer == VblankTimer::RealTime)
            return realtime ? "real-time" : "ordinary (real-time refused)";
        return "ordinary (NFSMW_VBLANK_TIMER=sleep)";
#elif defined(_WIN32)
        if (timer == VblankTimer::RealTime)
            return realtime ? "high-resolution waitable timer" : "ordinary (high-resolution timer unavailable)";
        return "ordinary (NFSMW_VBLANK_TIMER=sleep)";
#else
        (void)timer;
        (void)realtime;
        return "ordinary";
#endif
    }

    void VblankTiming::Add(int64_t lateNs, int64_t handlerNs)
    {
        if (m_late.size() < kCount)
        {
            m_late.push_back(lateNs);
            m_handler.push_back(handlerNs);
            return;
        }
        m_late[m_next] = lateNs;
        m_handler[m_next] = handlerNs;
        m_next = (m_next + 1) % kCount;
    }

    VblankTiming::Summary VblankTiming::Summarize() const
    {
        Summary s;
        s.count = uint32_t(m_late.size());
        if (m_late.empty())
            return s;
        s.lateMedianNs = double(Percentile(m_late, 500));
        s.late99Ns = double(Percentile(m_late, 990));
        s.lateMaxNs = double(*std::max_element(m_late.begin(), m_late.end()));
        s.handler99Ns = double(Percentile(m_handler, 990));
        return s;
    }
}
