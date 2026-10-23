// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
//
// Diagnostics for the time-of-day lighting state (nothing here changes the
// game unless NFSMW_TOD_BLEND is set). The live lighting object's pointer is
// the word at 0x82A2C4F8; its +0 / +8 / +12 are the event's update rate,
// phase and sun azimuth, +256 is the lighting blend sub_822265D0 applies
// (between the sets at +176/+216 and +196/+236) and the overcast is at
// 0x82900CCC. Each hooked routine logs its first calls and every change of
// the value it handles, with its caller (ctx.lr), prefixed [tod].
//   NFSMW_TOD_TRACE=0  turns the logging off.
//   NFSMW_TOD_BLEND=x  forces sub_822265D0's blend argument (experiment).
//   NFSMW_TOD_PHASE=x  pins the phase at +8 on every blend call (experiment).
#include <stdafx.h>

#include <kernel/function.h>

extern "C" PPC_FUNC(__imp__sub_822265D0);
extern "C" PPC_FUNC(__imp__sub_8223A0B8);
extern "C" PPC_FUNC(__imp__sub_8223A150);
extern "C" PPC_FUNC(__imp__sub_82221898);
extern "C" PPC_FUNC(__imp__sub_822218B0);
extern "C" PPC_FUNC(__imp__sub_822218C8);
extern "C" PPC_FUNC(__imp__sub_822266A8);
extern "C" PPC_FUNC(__imp__sub_8233A5D0);
extern "C" PPC_FUNC(__imp__sub_82367F18);
extern "C" PPC_FUNC(__imp__sub_8238D640);
extern "C" PPC_FUNC(__imp__sub_8238D6E8);

namespace
{
    constexpr uint32_t kTodPointer = 0x82A2C4F8;
    constexpr uint32_t kOvercast = 0x82900CCC;

    const bool s_trace = [] {
        const char* v = std::getenv("NFSMW_TOD_TRACE");
        return !(v && v[0] == '0');
    }();

    const float s_blend = [] {
        const char* v = std::getenv("NFSMW_TOD_BLEND");
        if (!v || !v[0])
            return -1.0f;
        const float f = static_cast<float>(std::atof(v));
        fprintf(stderr, "[tod] NFSMW_TOD_BLEND=%.4f: sub_822265D0's blend is forced\n", f);
        return f;
    }();

    const float s_phase = [] {
        const char* v = std::getenv("NFSMW_TOD_PHASE");
        if (!v || !v[0])
            return -1.0f;
        const float f = static_cast<float>(std::atof(v));
        fprintf(stderr, "[tod] NFSMW_TOD_PHASE=%.4f: the phase at +8 is pinned on every blend call\n", f);
        return f;
    }();

    float LoadFloat(uint8_t* base, uint32_t address)
    {
        const uint32_t bits = PPC_LOAD_U32(address);
        float f;
        memcpy(&f, &bits, sizeof(f));
        return f;
    }

    struct Site
    {
        const char* name;
        uint32_t calls = 0;
        uint32_t logged = 0;
        float last = 1e30f;
    };

    void Log(uint8_t* base, Site& site, uint32_t lr, uint32_t r3, float value)
    {
        if (!s_trace)
            return;
        static std::mutex mutex;
        std::lock_guard lock(mutex);
        site.calls++;
        if (site.logged >= 400 || (site.calls > 24 && std::fabs(value - site.last) < 1e-4f))
            return;
        site.last = value;
        site.logged++;
        const uint32_t tod = PPC_LOAD_U32(kTodPointer);
        if (tod)
            fprintf(stderr, "[tod] %s #%u lr=%08X r3=%08X value=%.5f | obj %08X rate=%.5f phase=%.5f azimuth=%.5f blend=%.5f/%.5f overcast=%.5f\n",
                site.name, site.calls, lr, r3, value, tod, LoadFloat(base, tod), LoadFloat(base, tod + 8),
                LoadFloat(base, tod + 12), LoadFloat(base, tod + 256), LoadFloat(base, tod + 260), LoadFloat(base, kOvercast));
        else
            fprintf(stderr, "[tod] %s #%u lr=%08X r3=%08X value=%.5f | no object\n", site.name, site.calls, lr, r3, value);
    }
}

PPC_FUNC(sub_822265D0)
{
    static Site site{"822265D0 blend(f1)"};
    if (s_blend >= 0.0f)
        ctx.f1.f64 = s_blend;
    if (s_phase >= 0.0f)
    {
        const uint32_t tod = PPC_LOAD_U32(kTodPointer);
        if (tod)
        {
            uint32_t bits;
            memcpy(&bits, &s_phase, sizeof(bits));
            PPC_STORE_U32(tod + 8, bits);
        }
    }
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_822265D0(ctx, base);
}

PPC_FUNC(sub_8223A0B8)
{
    static Site site{"8223A0B8 (manager->phase)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, LoadFloat(base, ctx.r3.u32 + 92 + 40));
    __imp__sub_8223A0B8(ctx, base);
}

PPC_FUNC(sub_8223A150)
{
    static Site site{"8223A150 (phase->manager)"};
    const uint32_t tod = PPC_LOAD_U32(kTodPointer);
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, tod ? LoadFloat(base, tod + 8) : 0.0f);
    __imp__sub_8223A150(ctx, base);
}

PPC_FUNC(sub_82221898)
{
    static Site site{"82221898 setPhase(f1)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_82221898(ctx, base);
}

PPC_FUNC(sub_822218B0)
{
    static Site site{"822218B0 setRate(f1)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_822218B0(ctx, base);
}

PPC_FUNC(sub_822218C8)
{
    static Site site{"822218C8 setAzimuth(f1)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_822218C8(ctx, base);
}

PPC_FUNC(sub_822266A8)
{
    static Site site{"822266A8 setPhase(f1)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_822266A8(ctx, base);
}

PPC_FUNC(sub_8233A5D0)
{
    static Site site{"8233A5D0"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_8233A5D0(ctx, base);
}

PPC_FUNC(sub_82367F18)
{
    static Site site{"82367F18"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, static_cast<float>(ctx.f1.f64));
    __imp__sub_82367F18(ctx, base);
}

PPC_FUNC(sub_8238D640)
{
    static Site site{"8238D640 (TOD state set)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, 0.0f);
    __imp__sub_8238D640(ctx, base);
}

PPC_FUNC(sub_8238D6E8)
{
    static Site site{"8238D6E8 (ENISTimeOfDay)"};
    Log(base, site, static_cast<uint32_t>(ctx.lr), ctx.r3.u32, 0.0f);
    __imp__sub_8238D6E8(ctx, base);
}
