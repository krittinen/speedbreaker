// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
//
// Camera motion for motion vectors. The game's view setup (sub_8243EAC0,
// hooked in game/hfr.cpp) builds the player's view-projection once per
// picture; this keeps each picture's reprojection matrix for the renderer.
//
// A pixel of the picture has NDC (x, y) and a depth d (the game's depth
// buffer, z/w). Its clip position is proportional to (x, y, d, 1), so the
// same point in the previous picture is
//     (x, y, d, 1) * inverse(VP_now) * VP_before
// (the game's matrices are row-major and act on row vectors). That product,
// Reprojection, is all the shader needs: it divides by w and subtracts.
//
// The pictures are matched to the renderer's by count: the guest thread
// numbers its pictures by the VdSwaps it has made, the command processor by
// the XE_SWAPs it has run (gpu::FrameCount()), so a picture's draws are
// recorded under the number the swap that ends them will have been given.
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <game/hfr_blend.h>

namespace motionvectors
{
    using Matrix = std::array<float, 16>;

    inline Matrix Identity()
    {
        return { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    }

    // m * n, row-major (row vectors: v * m * n).
    inline Matrix Multiply(const Matrix& m, const Matrix& n)
    {
        Matrix r{};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                double s = 0;
                for (int k = 0; k < 4; k++)
                    s += double(m[i * 4 + k]) * double(n[k * 4 + j]);
                r[i * 4 + j] = float(s);
            }
        return r;
    }

    // The inverse by Gauss-Jordan elimination in double; false when singular.
    inline bool Inverse(const Matrix& m, Matrix& out)
    {
        double a[4][8];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                a[i][j] = m[i * 4 + j];
                a[i][4 + j] = i == j ? 1.0 : 0.0;
            }
        for (int c = 0; c < 4; c++)
        {
            int p = c;
            for (int r = c + 1; r < 4; r++)
                if (std::fabs(a[r][c]) > std::fabs(a[p][c]))
                    p = r;
            if (std::fabs(a[p][c]) < 1e-12)
                return false;
            if (p != c)
                for (int j = 0; j < 8; j++)
                    std::swap(a[p][j], a[c][j]);
            const double inv = 1.0 / a[c][c];
            for (int j = 0; j < 8; j++)
                a[c][j] *= inv;
            for (int r = 0; r < 4; r++)
                if (r != c)
                {
                    const double f = a[r][c];
                    for (int j = 0; j < 8; j++)
                        a[r][j] -= f * a[c][j];
                }
        }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                out[i * 4 + j] = float(a[i][4 + j]);
        return true;
    }

    // The matrix that takes a point of this picture, as (NDC x, NDC y, depth,
    // 1), to its place in the picture before.
    inline Matrix Reprojection(const Matrix& viewProjection, const Matrix& previousViewProjection)
    {
        Matrix inverse;
        if (!Inverse(viewProjection, inverse))
            return Identity();
        return Multiply(inverse, previousViewProjection);
    }

    struct Vehicle
    {
        uint32_t id = 0;  // its render object: the same vehicle in every picture
        Matrix world = Identity();
        float center[3] = { 0.0f, 0.0f, 0.5f };  // its box, in its own space (metres): see BoxFromWheels
        float half[3] = { 2.4f, 1.0f, 0.8f };
        float wheels[4][3] = {};  // the hubs, in its own space
        float radius = 0.0f;
        float maxWheelDelta = 0.6f;  // the most a wheel turns in a picture (radians): the game's own clamp
        bool hasWheels = false;
        // The vertex buffer addresses (guest physical, sorted) found in the models of the parts the car is built
        // from (its CarRenderInfo's part table): a draw whose buffers are among them is a part of this car, and
        // nothing else (scenery, props) has them. Null when the table was not found.
        std::shared_ptr<const std::vector<uint32_t>> buffers;
    };

    // A vehicle's box from its wheels (four positions, x along the vehicle,
    // y across, z about the hub's height, and the wheel radius): its length
    // is the wheels' span plus an overhang each end (a trailer's, whose
    // origin is its hitch, reaches the hitch), its width the track plus a
    // tyre, its height from the ground (a radius below the hub) a multiple
    // of the radius (a car's 4.5, a truck's 6). False for a vehicle without
    // wheels (a helicopter): the caller keeps the default box.
    inline bool BoxFromWheels(const float wheels[4][3], float radius, Vehicle& v)
    {
        float lo = 1e9f, hi = -1e9f, side = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            lo = std::min(lo, wheels[i][0]);
            hi = std::max(hi, wheels[i][0]);
            side = std::max(side, std::fabs(wheels[i][1]));
        }
        if (!(hi - lo > 0.5f) || !(side > 0.3f) || !(radius > 0.1f) || radius > 1.5f || hi - lo > 20.0f)
            return false;
        constexpr float kOverhang = 0.85f;
        lo = std::min(lo - kOverhang, 0.0f);
        hi = std::max(hi + kOverhang, 0.0f);
        const float height = radius * (radius < 0.45f ? 4.5f : 6.0f);
        v.center[0] = (lo + hi) * 0.5f;
        v.half[0] = (hi - lo) * 0.5f;
        v.center[1] = 0.0f;
        v.half[1] = side + 0.12f;
        v.center[2] = (height - 2.0f * radius) * 0.5f;
        v.half[2] = height * 0.5f;
        return true;
    }

    // A rigid motion (row vectors: p' = p * R + t, as a view matrix step is) scaled to a fraction
    // of itself: the rotation's angle and the translation times s. For the small steps of a
    // camera between pictures this is the motion of a fraction of the step.
    inline Matrix ScaleRigid(const Matrix& t, float s)
    {
        double r[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r[i][j] = t[j * 4 + i];  // the column-vector form of the rotation
        const double cosine = std::clamp((r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5, -1.0, 1.0);
        const double theta = std::acos(cosine);
        double scaled[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        if (theta > 1e-7 && theta < 3.0)
        {
            const double sn = std::sin(theta);
            const double a[3] = { (r[2][1] - r[1][2]) / (2.0 * sn), (r[0][2] - r[2][0]) / (2.0 * sn), (r[1][0] - r[0][1]) / (2.0 * sn) };
            const double th = theta * double(s), cs = std::cos(th), ss = std::sin(th);
            const double cross[3][3] = { { 0, -a[2], a[1] }, { a[2], 0, -a[0] }, { -a[1], a[0], 0 } };
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    scaled[i][j] = (i == j ? cs : 0.0) + (1.0 - cs) * a[i] * a[j] + ss * cross[i][j];
        }
        Matrix out = Identity();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                out[i * 4 + j] = float(scaled[j][i]);
        for (int k = 0; k < 3; k++)
            out[12 + k] = t[12 + k] * s;
        return out;
    }

    struct Entry
    {
        uint64_t picture = ~0ull;
        Matrix reprojection = Identity();
        Matrix viewProjection = Identity();
        bool cut = true;  // no picture before, or the camera jumped: nothing moved
        std::vector<Vehicle> vehicles;  // the vehicles' world matrices as this picture draws them
        uint64_t serial = 0;  // which publish made it (a picture may be published again)
        // The camera's step to the picture before as a fraction of its last few steps (the
        // pictures' steps are uneven: 0.0006, 0.0032, 0.0006 in one second of steady driving,
        // and a near object's vector follows them): the reprojection, and the view-projection of
        // the picture before it goes to.
        Matrix smoothedReprojection = Identity();
        Matrix smoothedPrevious = Identity();
    };

    inline std::mutex g_mutex;
    inline std::array<Entry, 16> g_entries;
    inline std::atomic<uint64_t> g_guestPictures{ 0 };  // VdSwaps made by the guest
    inline std::atomic<bool> g_scanRequested{ false };  // F11: NFSMW_FIND_SMACKABLES scans the guest heap once
    // NFSMW_FIND_SMACKABLES: the guest addresses of the matrix-shaped blocks the scan found (a block's matrix is
    // at +0x48), compared with every draw's world matrix for a couple of pictures after the scan.
    inline std::vector<uint32_t> g_matrixCandidates;
    inline const uint8_t* g_guestBase = nullptr;
    inline std::atomic<bool> g_matchPending{ false };
    inline uint64_t g_matchFrom = 0, g_matchUntil = 0;

    // Guest render thread, from VdSwap: the picture that was being drawn is done.
    inline void NoteGuestSwap()
    {
        g_guestPictures.fetch_add(1, std::memory_order_relaxed);
    }

    // Guest render thread, after a view setup of the player's view: its view
    // and view-projection matrices. A jump of the camera since the last
    // picture (a cut) shows no motion.
    inline void Publish(const Matrix& view, const Matrix& viewProjection, std::vector<Vehicle> vehicles)
    {
        // The picture before is the last one's final view; a picture that sets
        // its view up more than once compares each with that same one.
        static Matrix s_current = Identity(), s_before = Identity(), s_currentView = Identity(), s_beforeView = Identity();
        static bool s_haveCurrent = false, s_haveBefore = false;
        static uint64_t s_lastPicture = ~0ull;
        const uint64_t picture = g_guestPictures.load(std::memory_order_relaxed);
        if (picture != s_lastPicture)
        {
            s_before = s_current;
            s_beforeView = s_currentView;
            s_haveBefore = s_haveCurrent;
            s_lastPicture = picture;
        }
        s_current = viewProjection;
        s_currentView = view;
        s_haveCurrent = true;
        bool cut = !s_haveBefore || hfr::blend::ViewCut(s_beforeView.data(), view.data(), 8.0f, 30.0f);
        Matrix reprojection = cut ? Identity() : Reprojection(viewProjection, s_before);
        // A camera that jumps far further than it has been moving (the finish line's cut to
        // another camera, which moves under the 8 m / 30 degree test) is a cut too: its one
        // picture of "motion" would smear the whole screen.
        if (!cut)
        {
            static float s_typicalStep = 0.003f;
            const float pw = 0.99f * reprojection[11] + reprojection[15];
            const float step = std::fabs(pw) > 1e-6f ? std::hypot((0.99f * reprojection[8] + reprojection[12]) / pw,
                (0.99f * reprojection[9] + reprojection[13]) / pw) : 0.0f;
            if (step > 0.05f && step > 10.0f * s_typicalStep)
            {
                cut = true;
                reprojection = Identity();
            }
            else
                s_typicalStep = std::max(0.0005f, 0.9f * s_typicalStep + 0.1f * step);
        }
        static const bool logMotion = std::getenv("NFSMW_LOG_MV") != nullptr;
        auto eye = [](const Matrix& m, float* e) {
            for (int k = 0; k < 3; k++)
                e[k] = -(m[12] * m[k * 4] + m[13] * m[k * 4 + 1] + m[14] * m[k * 4 + 2]);
        };
        if (logMotion)
        {
            float e[3];
            eye(view, e);
            static float s_prevEye[3];
            static int s_lines = 0;
            if (!vehicles.empty() ? s_lines++ < 1200 : (picture % 120) == 0)
            {
                std::string line = std::format("[mv] picture {} cut {} eye {:.3f} {:.3f} {:.3f}", picture, cut ? 1 : 0,
                    e[0] - s_prevEye[0], e[1] - s_prevEye[1], e[2] - s_prevEye[2]);
                fprintf(stderr, "%s\n", line.c_str());
            }
            memcpy(s_prevEye, e, sizeof(e));
            // The player's car (item 0 of the list): its speed from the last picture's position and the time between them.
            if (!vehicles.empty())
            {
                static float s_prevCar[3];
                static bool s_havePrevCar = false;
                static std::chrono::steady_clock::time_point s_prevTime;
                static uint64_t s_prevCarPicture = ~0ull;
                static uint32_t s_speedLines = 0;
                const auto now = std::chrono::steady_clock::now();
                if (s_havePrevCar && picture != s_prevCarPicture)
                {
                    const float step = std::hypot(vehicles[0].world[12] - s_prevCar[0], vehicles[0].world[13] - s_prevCar[1], vehicles[0].world[14] - s_prevCar[2]);
                    const float seconds = std::chrono::duration<float>(now - s_prevTime).count();
                    if (seconds > 0.001f && s_speedLines++ < 60000)
                        fprintf(stderr, "[spd] picture %llu km/h %.1f step %.3f m dt %.1f ms\n", (unsigned long long)picture,
                            step / seconds * 3.6f, step, seconds * 1000.0f);
                }
                if (picture != s_prevCarPicture)
                {
                    memcpy(s_prevCar, &vehicles[0].world[12], sizeof(s_prevCar));
                    s_prevTime = now;
                    s_prevCarPicture = picture;
                    s_havePrevCar = true;
                }
            }
            static int s_listLines = 0;
            if ((picture % 20) < 4 && s_listLines < 60000)
                for (const Vehicle& v : vehicles)
                {
                    s_listLines++;
                    fprintf(stderr, "[mvl] picture %llu car%08x pos %.2f %.2f %.2f\n", (unsigned long long)picture, v.id,
                        v.world[12], v.world[13], v.world[14]);
                }
        }
        // The camera's step smoothed in size, not in direction: the previous view is the current one
        // stepped back by the last picture's own step, scaled to the average size of the last few
        // pictures' steps (not across a cut). The pictures' steps are uneven in size, so their sizes
        // are averaged; their directions are kept, because an average of the motions themselves
        // cancels a shaking camera's (the back-and-forth of a few pictures nets to nothing) and
        // the blur drops out while the picture shakes.
        struct History
        {
            uint64_t picture;
            Matrix view, viewProjection;
            bool cut;
            float step;  // the step to the picture before, in NDC at the far end of the view
        };
        static std::vector<History> s_history;
        static uint64_t s_historyPicture = ~0ull;
        if (picture != s_historyPicture)
        {
            s_history.push_back({});
            if (s_history.size() > 8)
                s_history.erase(s_history.begin());
            s_historyPicture = picture;
        }
        float currentStep = 0.0f;
        if (!cut)
        {
            const float pw = 0.99f * reprojection[11] + reprojection[15];
            if (std::fabs(pw) > 1e-6f)
                currentStep = std::hypot((0.99f * reprojection[8] + reprojection[12]) / pw, (0.99f * reprojection[9] + reprojection[13]) / pw);
        }
        s_history.back() = { picture, view, viewProjection, cut, currentStep };
        Matrix smoothedPrevious = cut ? viewProjection : s_before;
        Matrix smoothedReprojection = reprojection;
        static const bool noSmooth = std::getenv("NFSMW_MV_NO_SMOOTH") != nullptr;
        static const int kSmoothPictures = [] {
            const char* v = std::getenv("NFSMW_MV_SMOOTH_PICTURES");
            return v ? std::clamp(std::atoi(v), 1, 7) : 6;
        }();
        if (!noSmooth && !cut && currentStep > 1e-6f)
        {
            const int size = int(s_history.size());
            float sum = currentStep;
            int n = 1;
            while (n < kSmoothPictures && size - 1 - n >= 0 && !s_history[size - 1 - n].cut &&
                s_history[size - 1 - n].picture + uint64_t(n) == picture)
            {
                sum += s_history[size - 1 - n].step;
                n++;
            }
            Matrix inverseView, inverseBeforeView;
            if (n >= 2 && Inverse(view, inverseView) && Inverse(s_history[size - 2].view, inverseBeforeView))
            {
                const float scale = std::clamp(sum / float(n) / currentStep, 0.25f, 4.0f);
                const Matrix step = Multiply(inverseView, s_history[size - 2].view);
                const Matrix previousView = Multiply(view, ScaleRigid(step, scale));
                const Matrix projectionBefore = Multiply(inverseBeforeView, s_history[size - 2].viewProjection);
                smoothedPrevious = Multiply(previousView, projectionBefore);
                smoothedReprojection = Reprojection(viewProjection, smoothedPrevious);
            }
        }
        else if (cut)
            smoothedReprojection = Identity();
        std::lock_guard lock(g_mutex);
        static uint64_t s_serial = 0;
        g_entries[picture % g_entries.size()] = { picture, reprojection, viewProjection, cut, std::move(vehicles), ++s_serial,
            smoothedReprojection, smoothedPrevious };
    }

    // Command processor: the reprojection of the picture whose draws are being
    // run (picture = swaps run so far); the identity when none was recorded.
    inline std::atomic<uint64_t> g_lookupHit{ 0 }, g_lookupMiss{ 0 }, g_carOffset[3];

    inline Matrix Lookup(uint64_t picture)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
        (e.picture == picture ? g_lookupHit : g_lookupMiss).fetch_add(1, std::memory_order_relaxed);
        return e.picture == picture ? e.reprojection : Identity();
    }

    // The same with the camera's step smoothed (what the blur and the velocity view show).
    inline Matrix LookupSmoothed(uint64_t picture)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
        (e.picture == picture ? g_lookupHit : g_lookupMiss).fetch_add(1, std::memory_order_relaxed);
        return e.picture == picture ? e.smoothedReprojection : Identity();
    }

    // The picture's record (false when none).
    inline bool LookupEntry(uint64_t picture, Entry& out)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
        if (e.picture != picture)
            return false;
        out = e;
        return true;
    }

    // The pixel shader constants the velocity view reads, from the slot of
    // the reprojection matrix (kSlots of them): the camera's reprojection.
    constexpr int kSlots = 4;
    inline void VelocityConstants(uint64_t picture, float* out)
    {
        const Matrix m = LookupSmoothed(picture);
        memcpy(out, m.data(), sizeof(Matrix));
    }

    // A picture's matrices as the command processor needs them.
    struct PictureMatrices
    {
        uint64_t picture = ~0ull, serial = ~0ull;
        Matrix inverseViewProjection = Identity(), reprojection = Identity();
        bool cut = true;
    };

    inline bool LookupPicture(uint64_t picture, PictureMatrices& out)
    {
        static PictureMatrices s_cache[8];
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
        if (e.picture != picture)
            return false;
        PictureMatrices& c = s_cache[picture % 8];
        if (c.picture != picture || c.serial != e.serial)
        {
            c.picture = ~0ull;
            if (!Inverse(e.viewProjection, c.inverseViewProjection))
                return false;
            c.picture = picture;
            c.serial = e.serial;
            c.reprojection = e.reprojection;
            c.cut = e.cut;
        }
        out = c;
        return true;
    }

    // Motion by what a draw draws (command processor only). The vertex shaders
    // of the world's objects hold the draw's world * view-projection as columns
    // in c0-c3 ("clip"). The draw of the same mesh (its vertex streams, index
    // range and vertex shader: the caller's `mesh` key) in the picture before
    // has its own, and
    //     inverse(clip_now) * clip_before
    // takes a pixel, as (NDC x, NDC y, depth, 1), through the mesh's own space
    // to where it was: the object's motion and the camera's together, exact,
    // for a vehicle, a wheel turning on it, or a car in traffic, with no need
    // to know which object it is. Of a mesh drawn more than once (a mirror's
    // view, two cars of a model) the earlier draw nearest in the world is the
    // one. A draw that moves with the camera alone is static: the blur pass's
    // own camera reprojection covers it.
    enum class DrawMotion { NotClip, NoPrevious, Static, Moving };

    // A draw's clip matrix with the picture whose view-projection it holds (the
    // renderer's frames and the guest's pictures are not one to one: a frame
    // may draw a neighbouring picture, or the same one twice).
    struct MeshDraw
    {
        uint64_t frame, picture;
        Matrix clip;
    };

    struct MeshHistory
    {
        uint64_t frame = ~0ull;
        std::vector<MeshDraw> draws;
    };

    inline float g_lastDeviation = 0;
    inline int g_lastOffset = 0;
    // Why a draw had no previous: 0 cut or picture missing, 1 mesh never seen, 2 none from the picture before, 3 all too far.
    inline int g_lastReason = 0;
    inline float g_lastNearest = -1, g_lastNearDeviation = -1, g_lastWorld[3];

    inline uint64_t g_looseMatches = 0;
    // A draw no car owns (a prop: a barrel, a fence, a lightpost; or a tree): matched to the same mesh's draw of the
    // picture before without any car's estimate, and only to a near, clearly nearest one.
    inline bool g_propMode = false;

    inline bool LookupViewProjection(uint64_t picture, Matrix& out);

    // The game's own vehicle list (each car's render object and world matrix, published with
    // the camera): a draw inside a car's box (a wheel, the body, a mirror) moves as that car
    // did since the picture before, found by the object's id, whatever mesh it is.
    inline uint64_t g_vehicleHits = 0;
    // For the log: the car of the last listed draw, and where the last draw's move came from
    // (0 none, 1 the game's list, 2 the draws' own table, 3 an earlier draw of the mesh) and what it was.
    inline uint32_t g_lastCarId = 0;
    inline int g_lastSource = 0;
    inline float g_lastMove[3], g_lastLocal[3];
    inline bool g_lastHub = false;

    // The wheel a draw at a car's hub belongs to: where its hub is, its axle and up (unit, world),
    // and its radius.
    struct WheelInfo
    {
        bool valid = false;
        float hub[3], axle[3], up[3], radius;
        float maxDelta = 0.6f;
    };
    inline WheelInfo g_wheel;
    inline uint64_t g_spinDraws = 0;

    // Where a point of a wheel was in the picture before, as a matrix on world positions
    // (row vectors): the wheel rolled on the ground, so it turned by the distance the car went
    // forward over its radius about its axle (turning a point back by that angle round the
    // hub, and the hub back by the car's move). A straight line between the two poses is
    // wrong once the wheel turns far in a picture; this is the circle. The angle is capped:
    // past it the blur would sweep further than the picture's exposure shows.
    inline Matrix WheelPrevious(const WheelInfo& w, const float* move, float maxAngle)
    {
        const float* n = w.axle;
        const float c[3] = { n[1] * w.up[2] - n[2] * w.up[1], n[2] * w.up[0] - n[0] * w.up[2], n[0] * w.up[1] - n[1] * w.up[0] };
        const float cc = std::max(c[0] * c[0] + c[1] * c[1] + c[2] * c[2], 1e-6f);
        float theta = (c[0] * move[0] + c[1] * move[1] + c[2] * move[2]) / (w.radius * cc);
        theta = std::clamp(theta, -maxAngle, maxAngle);
        const float cs = std::cos(-theta), sn = std::sin(-theta);
        const float cross[3][3] = { { 0, -n[2], n[1] }, { n[2], 0, -n[0] }, { -n[1], n[0], 0 } };
        Matrix a = Identity();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                a[i * 4 + j] = (i == j ? cs : 0.0f) + (1.0f - cs) * n[i] * n[j] + sn * cross[j][i];
        for (int j = 0; j < 3; j++)
            a[12 + j] = w.hub[j] - move[j] - (w.hub[0] * a[j] + w.hub[1] * a[4 + j] + w.hub[2] * a[8 + j]);
        return a;
    }

    inline bool VehicleMove(uint64_t picture, const Matrix& world, float* out)
    {
        struct Car
        {
            uint32_t id;
            float pos[3], axis[3][3], center[3], half[3], move[3], hubs[4][3], radius, maxDelta;
            bool hasWheels;
        };
        static uint64_t s_picture = ~0ull, s_serial = ~0ull, s_serialBefore = ~0ull;
        static std::vector<Car> s_cars;
        std::lock_guard lock(g_mutex);
        if (picture == 0)
            return false;
        const Entry& e = g_entries[picture % g_entries.size()];
        const Entry& b = g_entries[(picture - 1) % g_entries.size()];
        if (e.picture != picture || b.picture != picture - 1)
            return false;
        if (s_picture != picture || s_serial != e.serial || s_serialBefore != b.serial)
        {
            s_picture = picture;
            s_serial = e.serial;
            s_serialBefore = b.serial;
            s_cars.clear();
            for (const Vehicle& v : e.vehicles)
                for (const Vehicle& o : b.vehicles)
                    if (o.id == v.id)
                    {
                        Car c;
                        c.id = v.id;
                        c.hasWheels = v.hasWheels;
                        c.radius = v.radius;
                        c.maxDelta = v.maxWheelDelta;
                        for (int w = 0; w < 4; w++)
                            for (int k = 0; k < 3; k++)
                                c.hubs[w][k] = v.wheels[w][0] * v.world[k] + v.wheels[w][1] * v.world[4 + k] + v.wheels[w][2] * v.world[8 + k] + v.world[12 + k];
                        for (int k = 0; k < 3; k++)
                        {
                            c.pos[k] = v.world[12 + k];
                            c.move[k] = v.world[12 + k] - o.world[12 + k];
                            c.center[k] = v.center[k];
                            c.half[k] = v.half[k];
                            for (int a = 0; a < 3; a++)
                                c.axis[a][k] = v.world[a * 4 + k];
                        }
                        if (std::max({ std::fabs(c.move[0]), std::fabs(c.move[1]), std::fabs(c.move[2]) }) < 30.0f)
                            s_cars.push_back(c);
                        break;
                    }
        }
        const Car* best = nullptr;
        float bestScore = 1e9f;
        int bestIndex = -1;
        g_wheel.valid = false;
        // A wheel's draw sits at its hub: that names its car outright.
        float bestHub = 0.6f;
        for (const Car& c : s_cars)
            if (c.hasWheels)
                for (int w = 0; w < 4; w++)
                {
                    const float d = std::max({ std::fabs(world[12] - c.hubs[w][0]), std::fabs(world[13] - c.hubs[w][1]),
                        std::fabs(world[14] - c.hubs[w][2]) });
                    if (d < bestHub)
                    {
                        bestHub = d;
                        best = &c;
                        bestScore = 0.0f;
                        bestIndex = w;
                    }
                }
        if (!best)
        for (const Car& c : s_cars)
        {
            const float d[3] = { world[12] - c.pos[0], world[13] - c.pos[1], world[14] - c.pos[2] };
            float score = 0.0f;
            for (int a = 0; a < 3; a++)
            {
                const float len2 = c.axis[a][0] * c.axis[a][0] + c.axis[a][1] * c.axis[a][1] + c.axis[a][2] * c.axis[a][2];
                if (len2 < 1e-6f)
                {
                    score = 1e9f;
                    break;
                }
                const float local = (d[0] * c.axis[a][0] + d[1] * c.axis[a][1] + d[2] * c.axis[a][2]) / len2;
                score = std::max(score, (std::fabs(local - c.center[a]) - 0.7f) / std::max(c.half[a], 0.1f));
            }
            if (score < bestScore)
            {
                bestScore = score;
                best = &c;
            }
        }
        if (!best || bestScore > 0.0f)
            return false;
        g_lastHub = bestHub < 0.6f;
        if (g_lastHub && bestIndex >= 0 && best->radius > 0.1f)
        {
            auto unit = [](const float* v, float* o) {
                const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (l < 1e-6f)
                    return false;
                for (int k = 0; k < 3; k++)
                    o[k] = v[k] / l;
                return true;
            };
            g_wheel.valid = unit(best->axis[1], g_wheel.axle) && unit(best->axis[2], g_wheel.up);
            memcpy(g_wheel.hub, best->hubs[bestIndex], sizeof(g_wheel.hub));
            g_wheel.radius = best->radius;
            g_wheel.maxDelta = best->maxDelta;
        }
        for (int a = 0; a < 3; a++)
        {
            const float len2 = std::max(best->axis[a][0] * best->axis[a][0] + best->axis[a][1] * best->axis[a][1] + best->axis[a][2] * best->axis[a][2], 1e-6f);
            g_lastLocal[a] = ((world[12] - best->pos[0]) * best->axis[a][0] + (world[13] - best->pos[1]) * best->axis[a][1] +
                                 (world[14] - best->pos[2]) * best->axis[a][2]) / len2;
        }
        memcpy(out, best->move, sizeof(float) * 3);
        g_vehicleHits++;
        g_lastCarId = best->id;
        return true;
    }

    // Where the moving draws of a picture were and how far each went since the picture
    // before (world units): the parts of one car lie within a couple of metres of
    // each other and move together, so a draw near a known one, or near where one was
    // headed from the frame before, takes its move.
    struct CarMove
    {
        float pos[3], disp[3];
    };
    inline std::vector<CarMove> g_carsNow, g_carsBefore;
    inline uint64_t g_carsFrame = ~0ull;

    inline void RollCars(uint64_t frame)
    {
        if (frame == g_carsFrame)
            return;
        if (g_carsFrame + 1 == frame)
            g_carsBefore.swap(g_carsNow);
        else
            g_carsBefore.clear();
        g_carsNow.clear();
        g_carsFrame = frame;
    }

    inline void NoteCar(uint64_t frame, const Matrix& world, const float* disp)
    {
        RollCars(frame);
        if (g_carsNow.size() < 128)
            g_carsNow.push_back({ { world[12], world[13], world[14] }, { disp[0], disp[1], disp[2] } });
    }

    inline bool CarEstimate(uint64_t frame, const Matrix& world, float* out)
    {
        RollCars(frame);
        float best = 4.0f;
        bool found = false;
        auto consider = [&](const std::vector<CarMove>& cars, float headed) {
            for (const CarMove& c : cars)
            {
                const float d = std::max({ std::fabs(world[12] - c.pos[0] - headed * c.disp[0]),
                    std::fabs(world[13] - c.pos[1] - headed * c.disp[1]), std::fabs(world[14] - c.pos[2] - headed * c.disp[2]) });
                if (d < best)
                {
                    best = d;
                    found = true;
                    memcpy(out, c.disp, sizeof(float) * 3);
                }
            }
        };
        consider(g_carsNow, 0.0f);
        consider(g_carsBefore, 1.0f);
        return found;
    }

    inline bool h_pushLoose(MeshHistory& h, uint64_t frame, uint64_t picture, const Matrix& clip)
    {
        if (h.draws.size() < 512)
            h.draws.push_back({ frame, picture, clip });
        return true;
    }

    inline DrawMotion MatchDraw(MeshHistory& h, uint64_t frame, int offset, const PictureMatrices& now, const Matrix& world, const Matrix& clip, Matrix& reprojection)
    {
        const uint64_t picture = frame + offset;
        // The camera's reprojection goes from a picture to the one before it, so the draws
        // of the picture before are the ones to compare with. Of a mesh drawn many times (trees)
        // an instance is static when some earlier draw of the mesh gives exactly the camera's
        // motion; only when none does is the nearest one taken as the object's own.
        PictureMatrices before;
        Matrix inverse;
        bool found = false, isStatic = false, haveBefore = false;
        uint32_t seenBefore = 0;
        float minDeviation = 1e9f;
        g_lastNearest = -1;
        g_lastSource = 0;
        g_lastCarId = 0;
        g_lastNearDeviation = -1;
        haveBefore = !now.cut && picture > 0 && LookupPicture(picture - 1, before) && Inverse(clip, inverse);
        const bool historyBefore = !h.draws.empty();
        if (haveBefore)
        {
            auto deviation = [&](const Matrix& m, float limit) {
                float worst = 0.0f;
                for (const float x : { -0.7f, 0.0f, 0.7f })
                    for (const float y : { -0.7f, 0.0f, 0.7f })
                        for (const float z : { 0.9f, 0.99f })
                        {
                            const float pw = x * m[3] + y * m[7] + z * m[11] + m[15];
                            const float cw = x * now.reprojection[3] + y * now.reprojection[7] + z * now.reprojection[11] + now.reprojection[15];
                            if (std::fabs(pw) < 1e-6f || std::fabs(cw) < 1e-6f)
                                return 1e9f;
                            const float ox = (x * m[0] + y * m[4] + z * m[8] + m[12]) / pw;
                            const float oy = (x * m[1] + y * m[5] + z * m[9] + m[13]) / pw;
                            const float cx = (x * now.reprojection[0] + y * now.reprojection[4] + z * now.reprojection[8] + now.reprojection[12]) / cw;
                            const float cy = (x * now.reprojection[1] + y * now.reprojection[5] + z * now.reprojection[9] + now.reprojection[13]) / cw;
                            worst = std::max(worst, std::hypot(ox - cx, oy - cy));
                            if (worst > limit)
                                return worst;
                        }
                return worst;
            };
            struct Candidate
            {
                const MeshDraw* draw;
                float displacement[3];
            };
            Candidate candidates[64];
            uint32_t candidateCount = 0;
            for (const MeshDraw& p : h.draws)
            {
                if (p.picture + 1 != picture)
                    continue;
                seenBefore++;
                const Matrix candidate = Multiply(inverse, p.clip);
                const float dev = deviation(candidate, 0.03f);
                minDeviation = std::min(minDeviation, dev);
                if (dev <= 0.003f)
                {
                    isStatic = found = true;
                    break;
                }
                const Matrix w = Multiply(p.clip, before.inverseViewProjection);
                Candidate c{ &p, { world[12] - w[12], world[13] - w[13], world[14] - w[14] } };
                const float d = std::max({ std::fabs(c.displacement[0]), std::fabs(c.displacement[1]), std::fabs(c.displacement[2]) });
                if (g_lastNearest < 0 || d < g_lastNearest)
                {
                    g_lastNearest = d;
                    g_lastNearDeviation = dev;
                }
                if (candidateCount < 64)
                    candidates[candidateCount++] = c;
            }
            // A mesh with one earlier draw is that object, however far it went (a car at speed
            // moves metres a picture). Of several (wheels, trees) the one whose move matches the
            // last confident one of the picture (the vehicle's own) is taken, else the nearest.
            // The cars of the picture (see CarEstimate): a draw near one moves as it does, so
            // the wheels of a car are told from another car's by where they are.
            float est[3];
            const bool listed = !isStatic && !g_propMode && VehicleMove(picture, world, est);
            const bool haveEst = !isStatic && !g_propMode && (listed || CarEstimate(frame, world, est));
            auto norm = [](const float* v, const float* r) {
                return std::max({ std::fabs(v[0] - r[0]), std::fabs(v[1] - r[1]), std::fabs(v[2] - r[2]) });
            };
            static const float kZero[3] = {};
            const Candidate* pick = nullptr;
            if (!isStatic && candidateCount > 0)
            {
                float best = 1.5f;
                if (haveEst)
                    for (uint32_t i = 0; i < candidateCount; i++)
                    {
                        const float r = norm(candidates[i].displacement, est);
                        if (r < best)
                        {
                            best = r;
                            pick = &candidates[i];
                        }
                    }
                if (!pick && candidateCount == 1 && norm(candidates[0].displacement, kZero) < (g_propMode ? 1.5f : 30.0f))
                    pick = &candidates[0];
                if (!pick && candidateCount > 1)
                {
                    best = g_propMode ? 1.5f : 2.0f;
                    float second = 1e9f;
                    for (uint32_t i = 0; i < candidateCount; i++)
                    {
                        const float r = norm(candidates[i].displacement, kZero);
                        if (r < best)
                        {
                            second = best;
                            best = r;
                            pick = &candidates[i];
                        }
                        else if (r < second)
                            second = r;
                    }
                    // A prop among many of its own mesh (a row of fence segments) is told only when one is clearly nearest.
                    if (g_propMode && pick && second < best * 2.0f + 0.05f)
                        pick = nullptr;
                }
            }
            static const bool spinOff = !(std::getenv("NFSMW_MV_WHEEL_SPIN") && std::getenv("NFSMW_MV_WHEEL_SPIN")[0] == '1');
            Matrix vpBeforeSpin;
            if (listed && g_wheel.valid && !spinOff && LookupViewProjection(picture - 1, vpBeforeSpin))
            {
                // A wheel's own draw (at its hub): the circle it turned through, with the hub's
                // move from the draw's own earlier one when there is one (its origin is the hub).
                const float* move = pick ? pick->displacement : est;
                float forward[3];
                memcpy(forward, move, sizeof(forward));
                reprojection = Multiply(inverse, Multiply(world, Multiply(WheelPrevious(g_wheel, forward, g_wheel.maxDelta), vpBeforeSpin)));
                found = true;
                g_lastSource = 4;
                memcpy(g_lastMove, move, sizeof(g_lastMove));
                g_spinDraws++;
            }
            else if (pick)
            {
                reprojection = Multiply(inverse, pick->draw->clip);
                found = true;
                g_lastSource = 3;
                memcpy(g_lastMove, pick->displacement, sizeof(g_lastMove));
                if (!g_propMode)
                    NoteCar(frame, world, pick->displacement);
            }
            else if (haveEst && !isStatic)
            {
                // No earlier draw to go by: the same mesh was not drawn, or not where the car was.
                // The draw is placed where the car's move puts it, without the wheel's own turn.
                Matrix vpBefore;
                if (LookupViewProjection(picture - 1, vpBefore))
                {
                    Matrix worldBefore = world;
                    for (int k = 0; k < 3; k++)
                        worldBefore[12 + k] -= est[k];
                    reprojection = Multiply(inverse, Multiply(worldBefore, vpBefore));
                    found = true;
                    g_lastSource = listed ? 1 : 2;
                    memcpy(g_lastMove, est, sizeof(g_lastMove));
                }
            }
        }
        g_lastDeviation = minDeviation;
        memcpy(g_lastWorld, &world[12], sizeof(g_lastWorld));
        g_lastOffset = offset;
        if (h.draws.size() < 512)
            h.draws.push_back({ frame, picture, clip });
        if (!found)
        {
            g_lastReason = !haveBefore ? 0 : !historyBefore ? 1 : seenBefore == 0 ? 2 : 3;
            return DrawMotion::NoPrevious;
        }
        return isStatic ? DrawMotion::Static : DrawMotion::Moving;
    }

    // A listed car's draws in the car's own space. A draw's clip is its model matrix times the
    // view-projection: clip = L * R * VP, R the car's matrix of the list (exact: the car's body
    // draws hold it, L the identity) and L the part's own (a wheel's, a mirror's, the body's).
    // L is what the draw puts on the car, so the part is found again in the picture before by
    // L alone (a car's metres of motion are not in it), and the part's place in the picture
    // before is exactly L(before) * R(before) * VP(before). No guessing from world positions.
    struct LocalPart
    {
        uint64_t key;
        Matrix l;
        float reach[2];
        bool hasReach;
        float world[3] = {}, carPosition[3] = {};  // the part's origin and its car's, in the world
    };
    struct LocalCar
    {
        uint64_t frame = ~0ull;
        std::vector<LocalPart> now, before;
    };
    inline std::unordered_map<uint32_t, LocalCar> g_localCars;
    inline uint64_t g_localHits = 0, g_localRigid = 0, g_localMiss[4] = {};
    inline float g_lastReach[2];
    // Why the last draw the local path left was left (for the log): 0 not near a car, 1 near one but outside
    // its box / too far for a first sighting, 2 near and a part of that key was seen (see g_declineL).
    inline int g_declineKind = 0;
    inline float g_declineDistance, g_declineBox, g_declineL, g_declineScale;
    // Set by the caller for a vertex shader that also draws scenery (trees, props): its parts only belong to a
    // car when they are inside the car's box, matched before or not.
    inline bool g_strictOwnership = false;
    // The vertex buffers of the draw being noted (guest physical, masked), set by the caller: a car whose part
    // table is known only owns the draws that use one of its buffers.
    inline uint32_t g_drawBuffers[10];
    inline int g_drawBufferCount = 0;
    inline uint64_t g_ownedDraws = 0, g_unownedDraws = 0;
    inline bool OwnershipGate()
    {
        static const bool on = [] { const char* v = std::getenv("NFSMW_MV_OWNERSHIP"); return !v || v[0] != '0'; }();
        return on;
    }
    // 1: the car's table is known and the draw uses one of its buffers; 0: known and it does not; -1: no table.
    // The table is (low, high) pairs of the blocks the car's solids occupy, sorted and not overlapping.
    inline int CarOwnsDraw(const Vehicle& v)
    {
        if (!v.buffers || v.buffers->empty())
            return -1;
        const std::vector<uint32_t>& r = *v.buffers;
        for (int i = 0; i < g_drawBufferCount; i++)
        {
            // The last range that starts at or before the address.
            size_t lo = 0, hi = r.size() / 2;
            while (lo < hi)
            {
                const size_t mid = (lo + hi) / 2;
                if (r[mid * 2] <= g_drawBuffers[i])
                    lo = mid + 1;
                else
                    hi = mid;
            }
            if (lo && g_drawBuffers[i] < r[(lo - 1) * 2 + 1])
                return 1;
        }
        return 0;
    }
    inline float g_lastMetres[3], g_lastDistance;  // the chosen part's place along the car's axes in metres, and its distance from the car

    inline bool VehicleLocalMotion(uint64_t frame, uint64_t key, const Matrix& clip, Matrix& reprojection)
    {
        g_declineKind = 0;
        // How far a part's place on its car may move between pictures and still be the same part (wheel hop,
        // a spoiler's shake are centimetres). Anything that moves more relative to the car is not attached
        // to it: a sign or a cone beside the car, whose place on it changes by the car's own step. It was
        // 0.35 m, which let such things match and take the car's motion.
        static const float kPartMatch = [] {
            const char* v = std::getenv("NFSMW_MV_PART_MATCH");
            return v ? std::clamp(float(std::atof(v)), 0.02f, 1.0f) : 0.25f;
        }();
        // The pictures a draw may hold the list's matrices of. A car's draws mostly hold the
        // renderer's own picture's, but when the guest runs ahead some hold the next one's (or
        // the one before's): the car's list matrix is then off by a car's step, and the part
        // matrix worked out against it is too. L is a fingerprint (it does not change from
        // picture to picture), so the picture whose list gives the L the part had in the
        // picture before is the one the draw holds. With no earlier L, the renderer's own.
        static const int kPictures[3] = { 0, -1, 1 };
        struct Candidate
        {
            int k;
            uint32_t id;
            Matrix car, inverseViewProjection;
            bool cut;
            float distance;
            Matrix previousViewProjection;  // the camera of the picture before, stepped back smoothly
            float center[3], half[3];  // the car's box in its own space
            int owns;  // 1 the car's part table has the draw's buffers, -1 no table (the heuristics decide)
        };
        Candidate candidates[12];
        int candidateCount = 0;
        float nearestAny = 1e9f;
        for (const int k : kPictures)
        {
            if (int64_t(frame) + k < 0)
                continue;
            PictureMatrices p;
            if (!LookupPicture(frame + k, p))
                continue;
            const Matrix w = Multiply(clip, p.inverseViewProjection);
            std::lock_guard lock(g_mutex);
            const Entry& e = g_entries[(frame + k) % g_entries.size()];
            if (e.picture != frame + k)
                continue;
            for (const Vehicle& v : e.vehicles)
            {
                // A car whose part table is known owns only the draws that use its parts' buffers.
                const int owns = OwnershipGate() ? CarOwnsDraw(v) : -1;
                // The draw's origin is L's translation from the car's.
                const float d = std::hypot(w[12] - v.world[12], w[13] - v.world[13], w[14] - v.world[14]);
                nearestAny = std::min(nearestAny, d);
                if (d < (owns == 1 ? 8.0f : 4.0f) && candidateCount < 12)
                {
                    Candidate& c = candidates[candidateCount++] = { k, v.id, v.world, p.inverseViewProjection, e.cut, d, e.smoothedPrevious, {}, {}, owns };
                    memcpy(c.center, v.center, sizeof(c.center));
                    memcpy(c.half, v.half, sizeof(c.half));
                }
            }
        }
        // Draws a listed car's own that the path leaves to the matcher, for the log: 0 a car
        // within 8 m but none close enough, 1 the picture is a cut, 2 a matrix without an
        // inverse, 3 no pose of the car in the picture before.
        if (!candidateCount)
        {
            if (nearestAny < 8.0f)
                g_localMiss[0]++;
            return false;
        }
        Matrix inverseClip;
        if (!Inverse(clip, inverseClip))
        {
            g_localMiss[2]++;
            return false;
        }
        int chosen = -1;
        float chosenScore = 1e9f;
        Matrix local{};
        g_declineDistance = g_declineBox = g_declineL = 1e9f;
        uint32_t declinedId = 0;
        float declinedDistance = 1e9f, chosenWorld[3] = {}, chosenCarPosition[3] = {}, declinedWorld[3] = {}, declinedCarPosition[3] = {};
        Matrix declinedLocal{};
        for (int i = 0; i < candidateCount; i++)
        {
            const Candidate& c = candidates[i];
            Matrix inverseCandidate;
            if (c.cut || !Inverse(c.car, inverseCandidate))
                continue;
            const Matrix w0 = Multiply(clip, c.inverseViewProjection);
            const Matrix l = Multiply(w0, inverseCandidate);
            LocalCar& other = g_localCars[c.id];
            if (other.frame != frame)
            {
                if (other.frame + 1 == frame)
                    other.before.swap(other.now);
                else
                    other.before.clear();
                other.now.clear();
                other.frame = frame;
            }
            float nearestL = kPartMatch;
            bool any = false, staticInWorld = false;
            for (const LocalPart& p : other.before)
                if (p.key == key)
                {
                    const float d = std::hypot(l[12] - p.l[12], l[13] - p.l[13], l[14] - p.l[14]);
                    g_declineL = std::min(g_declineL, d);
                    // A part that stays where it was in the world while the car moved is not attached to it: a
                    // prop or a tree beside the car (whose place on the car changes by the car's own step, which
                    // a part's tolerance lets through at low speeds).
                    const float carStep = std::hypot(c.car[12] - p.carPosition[0], c.car[13] - p.carPosition[1], c.car[14] - p.carPosition[2]);
                    if (carStep > 0.1f && std::hypot(w0[12] - p.world[0], w0[13] - p.world[1], w0[14] - p.world[2]) < 0.03f + 0.15f * carStep)
                        staticInWorld = true;
                    if (d < nearestL)
                    {
                        nearestL = d;
                        any = true;
                    }
                }
            // Matched ones first (the nearest L, the renderer's picture on a tie); then, for a part
            // seen for the first time, the renderer's own picture and the nearest car, as long as it
            // is inside the car's box (a car's parts are: the road's edge, a barrier or a sign that
            // happens to be a few metres from it are not, and would take the car's motion).
            // The part's place in metres along the car's axes (a car's matrix may carry a scale, which
            // L's translation then has too: the box is in metres).
            float metres[3];
            for (int a = 0; a < 3; a++)
            {
                const float* row = &c.car[a * 4];
                const float length = std::max(std::hypot(row[0], row[1], row[2]), 1e-4f);
                metres[a] = ((w0[12] - c.car[12]) * row[0] + (w0[13] - c.car[13]) * row[1] + (w0[14] - c.car[14]) * row[2]) / length;
            }
            const float inBox = std::max({ std::fabs(metres[0] - c.center[0]) - c.half[0], std::fabs(metres[1] - c.center[1]) - c.half[1],
                std::fabs(metres[2] - c.center[2]) - c.half[2] });
            g_declineDistance = std::min(g_declineDistance, c.distance);
            g_declineScale = std::hypot(c.car[0], c.car[1], c.car[2]);
            g_declineBox = std::min(g_declineBox, inBox);
            if (c.k == 0 && c.distance < declinedDistance)
            {
                declinedDistance = c.distance;
                declinedId = c.id;
                declinedLocal = l;
                memcpy(declinedWorld, &w0[12], sizeof(declinedWorld));
                memcpy(declinedCarPosition, &c.car[12], sizeof(declinedCarPosition));
            }
            // A car with a part table that does not list the draw's buffers may still own it (a traffic car's
            // wheels are not in its table): only well inside its box, where scenery is not.
            const float margin = c.owns == 0 ? 0.3f : 0.8f;
            const float score = c.owns == 1 ? (any ? nearestL + (c.k == 0 ? 0.0f : 0.001f) : 1000.0f + (c.k == 0 ? 0.0f : 10.0f) + c.distance) :
                staticInWorld || (c.owns == 0 && !(inBox < margin)) || (g_strictOwnership && !(inBox < 0.8f)) ? 1e8f : any ? nearestL + (c.k == 0 ? 0.0f : 0.001f) :
                (c.distance < 3.0f && inBox < margin ? 1000.0f + (c.k == 0 ? 0.0f : 10.0f) + c.distance : 1e8f);
            if (score < chosenScore)
            {
                chosenScore = score;
                chosen = i;
                local = l;
                memcpy(chosenWorld, &w0[12], sizeof(chosenWorld));
                memcpy(chosenCarPosition, &c.car[12], sizeof(chosenCarPosition));
                memcpy(g_lastMetres, metres, sizeof(metres));
                g_lastDistance = c.distance;
            }
        }
        if (chosen < 0 || chosenScore >= 1e8f)
        {
            g_declineKind = g_declineL < 1e8f ? 2 : 1;
            g_localMiss[chosen < 0 ? 1 : 0]++;
            // Remember where the part sat on the nearest car: a part that sits there again in the next
            // picture is attached to it (a prop or a sign beside the car moves by the car's own step), even
            // when it is outside the box (a wheel or a grille of a traffic car's the box is too small for).
            if (declinedId && !g_strictOwnership)
            {
                LocalCar& car = g_localCars[declinedId];
                if (car.frame == frame && car.now.size() < 256)
                {
                    LocalPart part = { key, declinedLocal, { 0.0f, 0.0f }, false };
                    memcpy(part.world, declinedWorld, sizeof(part.world));
                    memcpy(part.carPosition, declinedCarPosition, sizeof(part.carPosition));
                    car.now.push_back(part);
                }
            }
            return false;
        }
        const int bestK = candidates[chosen].k;
        const uint32_t bestId = candidates[chosen].id;
        const Matrix bestCar = candidates[chosen].car;
        Matrix carBefore{}, viewProjectionBefore{};
        bool haveBefore = false;
        {
            std::lock_guard lock(g_mutex);
            const uint64_t before = frame + bestK - 1;
            const Entry& e = g_entries[before % g_entries.size()];
            if (e.picture == before)
                for (const Vehicle& v : e.vehicles)
                    if (v.id == bestId)
                    {
                        carBefore = v.world;
                        // The camera of that picture as it was: a car that moves with the camera (a chase
                        // camera's) has a relative motion of nearly nothing, and a smoothed camera step
                        // (scaled by up to 4) put its error there as a vector (up to 0.1 NDC in one picture).
                        viewProjectionBefore = e.viewProjection;
                        haveBefore = true;
                        break;
                    }
        }

        LocalCar& car = g_localCars[bestId];
        if (car.frame != frame)
        {
            if (car.frame + 1 == frame)
                car.before.swap(car.now);
            else
                car.before.clear();
            car.now.clear();
            car.frame = frame;
        }
        // The same part in the picture before: the same mesh at the same place on the car (a
        // car's four wheels are one mesh, a metre or more apart; a wheel's hop is centimetres).
        Matrix localBefore = local;
        bool matched = false, havePreviousReach = false;
        float nearest = kPartMatch, previousReach[2] = {};
        for (const LocalPart& p : car.before)
            if (p.key == key)
            {
                const float d = std::hypot(local[12] - p.l[12], local[13] - p.l[13], local[14] - p.l[14]);
                if (d < nearest)
                {
                    nearest = d;
                    localBefore = p.l;
                    matched = true;
                    havePreviousReach = p.hasReach;
                    previousReach[0] = p.reach[0];
                    previousReach[1] = p.reach[1];
                }
            }
        const size_t slot = car.now.size();
        if (slot < 256)
        {
            LocalPart part = { key, local, { 0.0f, 0.0f }, false };
            memcpy(part.world, chosenWorld, sizeof(part.world));
            memcpy(part.carPosition, chosenCarPosition, sizeof(part.carPosition));
            car.now.push_back(part);
        }
        if (!haveBefore)
        {
            g_localMiss[3]++;
            return false;
        }
        reprojection = Multiply(inverseClip, Multiply(Multiply(localBefore, carBefore), viewProjectionBefore));
        (matched ? g_localHits : g_localRigid)++;
        g_lastSource = 5;
        g_lastCarId = bestId;
        g_lastHub = matched;
        {
            // Where the part's origin goes between the pictures, in NDC (for the log).
            const float cw = clip[15], x = clip[12] / cw, y = clip[13] / cw, z = clip[14] / cw;
            float p[4];
            for (int j = 0; j < 4; j++)
                p[j] = x * reprojection[j] + y * reprojection[4 + j] + z * reprojection[8 + j] + reprojection[12 + j];
            g_lastReach[0] = p[0] / p[3] - x;
            g_lastReach[1] = p[1] / p[3] - y;
            if (slot < 256)
            {
                car.now[slot].reach[0] = g_lastReach[0];
                car.now[slot].reach[1] = g_lastReach[1];
                car.now[slot].hasReach = true;
            }
            // NFSMW_LOG_JUMPS=1: a part whose movement changed by a visible amount since the picture before.
            static const bool logJumps = std::getenv("NFSMW_LOG_JUMPS") != nullptr || std::getenv("NFSMW_MV_LOG_JUMPS") != nullptr;
            static uint32_t s_jumpLines = 0;
            if (logJumps && havePreviousReach && s_jumpLines < 30000)
            {
                const float jump = std::hypot(g_lastReach[0] - previousReach[0], g_lastReach[1] - previousReach[1]);
                if (jump > 0.006f)
                {
                    s_jumpLines++;
                    fprintf(stderr, "[mvj] %llu car%08x key%016llx k%d jump %.4f reach %.4f %.4f before %.4f %.4f local %.2f %.2f %.2f dL %.3f move %.3f\n",
                        (unsigned long long)frame, bestId, (unsigned long long)key, bestK, jump, g_lastReach[0], g_lastReach[1],
                        previousReach[0], previousReach[1], local[12], local[13], local[14], nearest,
                        std::hypot(bestCar[12] - carBefore[12], bestCar[13] - carBefore[13], bestCar[14] - carBefore[14]));
                }
            }
        }
        for (int i = 0; i < 3; i++)
        {
            g_lastMove[i] = bestCar[12 + i] - carBefore[12 + i];
            g_lastLocal[i] = local[12 + i];
            g_lastWorld[i] = Multiply(local, bestCar)[12 + i];
        }
        return true;
    }

    // The distance from a draw's origin to the nearest listed car in the picture's own list (for the log).
    inline float NearestListedCar(uint64_t frame, const Matrix& clip)
    {
        PictureMatrices p;
        if (!LookupPicture(frame, p))
            return 1e9f;
        const Matrix w = Multiply(clip, p.inverseViewProjection);
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[frame % g_entries.size()];
        float best = 1e9f;
        if (e.picture == frame)
            for (const Vehicle& v : e.vehicles)
                best = std::min(best, std::hypot(w[12] - v.world[12], w[13] - v.world[13], w[14] - v.world[14]));
        return best;
    }

    // True when a listed car of the picture has a part with one of the draw's buffers (g_drawBuffers).
    inline bool AnyCarOwnsDraw(uint64_t frame)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[frame % g_entries.size()];
        if (e.picture != frame)
            return false;
        for (const Vehicle& v : e.vehicles)
            if (CarOwnsDraw(v) == 1)
            {
                g_ownedDraws++;
                return true;
            }
        g_unownedDraws++;
        return false;
    }

    // NFSMW_FIND_SMACKABLES: logs the scan's candidate blocks whose matrix (at +0x48) is a draw's world matrix.
    inline void MatchCandidates(uint64_t frame, const Matrix& clip, uint32_t vertexShader, uint32_t count)
    {
        if (g_matchPending.exchange(false))
        {
            g_matchFrom = frame + 1;
            g_matchUntil = frame + 3;
        }
        if (g_matrixCandidates.empty() || frame < g_matchFrom || frame >= g_matchUntil || !g_guestBase)
            return;
        PictureMatrices p;
        if (!LookupPicture(frame, p))
            return;
        const Matrix w = Multiply(clip, p.inverseViewProjection);
        if (!(std::fabs(w[15] - 1.0f) < 0.05f))
            return;
        auto flt = [&](uint32_t a) {
            uint32_t v;
            memcpy(&v, g_guestBase + a, 4);
            v = __builtin_bswap32(v);
            float f;
            memcpy(&f, &v, 4);
            return f;
        };
        static uint32_t s_lines = 0;
        for (const uint32_t a : g_matrixCandidates)
            if (std::fabs(flt(a + 0x78) - w[12]) < 0.02f && std::fabs(flt(a + 0x7C) - w[13]) < 0.02f && std::fabs(flt(a + 0x80) - w[14]) < 0.02f &&
                std::fabs(flt(a + 0x48) - w[0]) < 0.02f && std::fabs(flt(a + 0x4C) - w[1]) < 0.02f && s_lines++ < 400)
                fprintf(stderr, "[smackhit] frame %llu vs%08x i%u block %08X pos %.2f %.2f %.2f\n", (unsigned long long)frame, vertexShader, count, a, w[12], w[13], w[14]);
    }

    inline DrawMotion NoteDraw(uint64_t frame, uint64_t mesh, uint64_t looseMesh, const Matrix& clip, Matrix& reprojection)
    {
        static std::unordered_map<uint64_t, MeshHistory> s_meshes;
        static uint64_t s_pruned = 0;
        // The guest may be a picture or two off the renderer's: the view-projection
        // a draw holds is found among the neighbours'. A matrix that is no world *
        // view-projection of any of them (w = clip * inverse(VP) is not a plain
        // transform) is not a clip matrix.
        static const int kOffsets[6] = { 0, -1, 1, -2, 2, 3 };
        PictureMatrices now;
        Matrix world{};
        int offset = 0;
        bool isClip = false;
        for (const int o : kOffsets)
        {
            if (int64_t(frame) + o < 0 || !LookupPicture(frame + o, now))
                continue;
            world = Multiply(clip, now.inverseViewProjection);
            if (std::fabs(world[15] - 1.0f) < 0.05f && std::fabs(world[3]) < 0.02f && std::fabs(world[7]) < 0.02f &&
                std::fabs(world[11]) < 0.02f)
            {
                isClip = true;
                offset = o;
                break;
            }
        }
        if (!isClip)
            return DrawMotion::NotClip;


        static const bool s_logOffsets = std::getenv("NFSMW_LOG_OFFSETS") != nullptr;
        static int s_offsetLines = 0;
        // The draws of a car that hold its render matrix (the body's parts) hold
        //     clip = R * VP
        // exactly, R the car's matrix in the list of the picture whose VP it is built with.
        // For each car, the best (picture offset, draw) by that test, one line a picture.
        struct ExactBest
        {
            uint32_t id = 0;
            float err = 1e9f;  // of the draws not holding the car's matrix: the best any picture's list gives
            int k = 0, a = 0, b = 0;
        };
        static uint64_t s_offsetFrame = ~0ull;
        static std::vector<ExactBest> s_exact;
        if (s_logOffsets && s_offsetLines < 40000)
        {
            if (frame != s_offsetFrame)
            {
                if (s_offsetFrame != ~0ull && (s_offsetFrame % 10) < 3)
                    for (const ExactBest& b : s_exact)
                    {
                        fprintf(stderr, "[mvy] %llu car%08x exact %d other %d other-best %d err %.5f\n", (unsigned long long)s_offsetFrame, b.id, b.a, b.b, b.k, b.err);
                        s_offsetLines++;
                    }
                s_offsetFrame = frame;
                s_exact.clear();
            }
            if ((frame % 10) < 3)
            {
                auto relativeError = [&](const Entry& e, const Vehicle& v) {
                    Matrix inverseCar;
                    if (!Inverse(v.world, inverseCar))
                        return 1e9f;
                    const Matrix used = Multiply(inverseCar, clip);
                    float diff = 0.0f, norm = 0.0f;
                    for (int i = 0; i < 16; i++)
                    {
                        diff += (used[i] - e.viewProjection[i]) * (used[i] - e.viewProjection[i]);
                        norm += e.viewProjection[i] * e.viewProjection[i];
                    }
                    return std::sqrt(diff / std::max(norm, 1e-9f));
                };
                Entry now0;
                if (LookupEntry(frame, now0))
                {
                    const Vehicle* owner = nullptr;
                    bool exact = false;
                    float nearest = 3.5f;
                    for (const Vehicle& v : now0.vehicles)
                    {
                        if (relativeError(now0, v) < 2e-3f)
                        {
                            owner = &v;
                            exact = true;
                            break;
                        }
                        const float d = std::hypot(world[12] - v.world[12], world[13] - v.world[13], world[14] - v.world[14]);
                        if (d < nearest)
                        {
                            nearest = d;
                            owner = &v;
                        }
                    }
                    if (owner)
                    {
                        ExactBest* entry = nullptr;
                        for (ExactBest& b : s_exact)
                            if (b.id == owner->id)
                                entry = &b;
                        if (!entry)
                        {
                            s_exact.push_back({ owner->id });
                            entry = &s_exact.back();
                        }
                        if (exact)
                            entry->a++;
                        else
                        {
                            entry->b++;
                            for (int o = -4; o <= 4; o++)
                            {
                                Entry listed;
                                if (int64_t(frame) + o < 0 || !LookupEntry(frame + o, listed))
                                    continue;
                                for (const Vehicle& v : listed.vehicles)
                                    if (v.id == owner->id)
                                    {
                                        const float err = relativeError(listed, v);
                                        if (err < entry->err)
                                        {
                                            entry->err = err;
                                            entry->k = o;
                                        }
                                    }
                            }
                        }
                    }
                }
            }
        }

        static const bool s_noLocal = std::getenv("NFSMW_MV_NO_LOCAL") != nullptr;
        if (!s_noLocal && VehicleLocalMotion(frame, looseMesh, clip, reprojection))
            return DrawMotion::Moving;
        // Only the game's listed cars are followed. A draw the car path did not take (a vehicle
        // shader on a prop, a car the list does not hold) gets no vehicle vector: the matcher
        // below took trees and props for moving things. NFSMW_MV_GENERIC=1 brings it back.
        static const bool s_generic = std::getenv("NFSMW_MV_GENERIC") != nullptr;
        static const bool s_props = [] { const char* v = std::getenv("NFSMW_MV_PROPS"); return !v || v[0] != '0'; }();
        g_propMode = !s_generic;
        if (!s_generic && !s_props)
            return DrawMotion::Static;
        static const bool s_logUnowned = std::getenv("NFSMW_LOG_OFFSETS") != nullptr;
        if (s_logUnowned && (frame % 10) < 3)
        {
            // A draw within 14 m of a listed car that the car's path did not take: which
            // picture's list and view-projection it holds, if any (exact when the part is the body).
            static uint64_t s_unownedFrame = ~0ull;
            static int s_unownedLines = 0, s_unownedThisFrame = 0;
            if (frame != s_unownedFrame)
            {
                s_unownedFrame = frame;
                s_unownedThisFrame = 0;
            }
            Entry listed;
            if (s_unownedThisFrame < 8 && s_unownedLines < 20000 && LookupEntry(frame, listed))
            {
                float nearest = 1e9f;
                for (const Vehicle& v : listed.vehicles)
                    nearest = std::min(nearest, std::hypot(world[12] - v.world[12], world[13] - v.world[13], world[14] - v.world[14]));
                if (nearest < 14.0f)
                {
                    float bestError = 1e9f;
                    int bestOffset = 0;
                    uint32_t bestId = 0;
                    for (int o = -14; o <= 2; o++)
                    {
                        Entry e;
                        if (int64_t(frame) + o < 0 || !LookupEntry(frame + o, e))
                            continue;
                        float norm = 0.0f;
                        for (const float x : e.viewProjection)
                            norm += x * x;
                        for (const Vehicle& v : e.vehicles)
                        {
                            Matrix inverseCar;
                            if (!Inverse(v.world, inverseCar))
                                continue;
                            const Matrix used = Multiply(inverseCar, clip);
                            float diff = 0.0f;
                            for (int i = 0; i < 16; i++)
                                diff += (used[i] - e.viewProjection[i]) * (used[i] - e.viewProjection[i]);
                            const float err = std::sqrt(diff / std::max(norm, 1e-9f));
                            if (err < bestError)
                            {
                                bestError = err;
                                bestOffset = o;
                                bestId = v.id;
                            }
                        }
                    }
                    fprintf(stderr, "[mvu] %llu near %.1f best-picture-offset %d car%08x err %.5f world %.0f %.0f %.0f\n",
                        (unsigned long long)frame, nearest, bestOffset, bestId, bestError, world[12], world[13], world[14]);
                    s_unownedThisFrame++;
                    s_unownedLines++;
                }
            }
        }

        const uint64_t picture = frame + offset;
        if (frame > s_pruned + 600)
        {
            s_pruned = frame;
            std::erase_if(s_meshes, [&](const auto& kv) { return kv.second.frame + 2 < frame; });
        }
        auto roll = [&](MeshHistory& h) {
            if (h.frame != frame)
            {
                std::erase_if(h.draws, [&](const MeshDraw& d) { return d.frame + 3 < frame; });
                h.frame = frame;
            }
        };
        MeshHistory& exact = s_meshes[mesh];
        MeshHistory& loose = s_meshes[looseMesh ^ 0x5bd1e9955bd1e995ull];
        roll(exact);
        roll(loose);
        // The exact key (vertex streams, index range) first; a mesh whose streams move from
        // picture to picture (dynamic vertex data) is found again by the loose one (shaders and
        // index count only), which has more instances to choose from.
        DrawMotion motion = MatchDraw(exact, frame, offset, now, world, clip, reprojection);
        if (motion == DrawMotion::NoPrevious)
        {
            const int reason = g_lastReason;
            motion = MatchDraw(loose, frame, offset, now, world, clip, reprojection);
            if (motion == DrawMotion::NoPrevious)
                g_lastReason = reason;
            else
                g_looseMatches++;
        }
        else if (h_pushLoose(loose, frame, picture, clip))
        {
        }
        return motion;
    }
    // The view-projection a picture was published with (false when none).
    inline bool LookupViewProjection(uint64_t picture, Matrix& out)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
        if (e.picture != picture)
            return false;
        out = e.viewProjection;
        return true;
    }

}
