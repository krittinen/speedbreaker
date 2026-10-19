// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
//
// The guest vblank's timer (gpu/command_processor.cpp's Vsync thread): how
// that thread is scheduled, and how late the vblanks fire.
//
// macOS: the thread slept with std::this_thread::sleep_until as an ordinary
// thread, a timer macOS lets fire late (coalescing): on a MacBook Pro its
// vblanks came 3.2 ms late at the median and up to 9 ms (8.5-25 ms apart
// instead of 16.667), idle or not, and the game's frames left for the display
// that unevenly. At the user-interactive class it was the same (3.3 ms), and
// mach_wait_until didn't help either: what does is real-time scheduling, the
// time-constraint policy Core Audio's threads run under, which the thread now
// asks for (0.01 ms late at the median, 0.02 in 99%). NFSMW_VBLANK_TIMER=sleep
// keeps the ordinary thread (an A/B switch). Windows uses a high-resolution
// waitable timer: ordinary sleep wakeups were late by several milliseconds.
// Elsewhere nothing changes: on the Steam Machine (Linux's high-resolution
// timers) the vblanks were already within 0.06 ms of their time.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace video
{
    enum class VblankTimer
    {
        Ordinary,  // the thread as created
        RealTime,  // macOS: time-constraint policy; Windows: high-resolution timer
    };

    // NFSMW_VBLANK_TIMER=sleep: Ordinary; otherwise RealTime.
    VblankTimer ChosenVblankTimer();
    // The calling thread, once, before its first sleep: its scheduling for
    // vblanks `periodNs` apart. True: the platform's preferred timer was set up.
    bool PrepareVblankThread(VblankTimer timer, int64_t periodNs);
    // Wait for an absolute steady-clock deadline in nanoseconds.
    void WaitVblankUntil(VblankTimer timer, int64_t deadlineNs);
    // For the log: "real-time", "high-resolution waitable timer",
    // "ordinary (NFSMW_VBLANK_TIMER=sleep)", or a platform fallback.
    const char* VblankTimerName(VblankTimer timer, bool realtime);

    // The last minute of vblanks: how late each fired after its time (the
    // timer's wake-up, or a guest interrupt handler before it that ran past
    // it) and how long the guest's interrupt handler ran. One thread.
    class VblankTiming
    {
    public:
        static constexpr size_t kCount = 3600;

        void Add(int64_t lateNs, int64_t handlerNs);
        struct Summary
        {
            uint32_t count = 0;
            double lateMedianNs = 0, late99Ns = 0, lateMaxNs = 0;
            double handler99Ns = 0;
        };
        Summary Summarize() const;

    private:
        std::vector<int64_t> m_late, m_handler;
        size_t m_next = 0;
    };
}
