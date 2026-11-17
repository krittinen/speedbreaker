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
#include <cstdint>
#include <cstring>
#include <memory>
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
        // The address blocks (guest physical, sorted (low, high) pairs) of the models of the parts the car is built
        // from (its CarRenderInfo's part table): a draw whose vertex buffers are in them is a part of this car, and
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
        constexpr int kSmoothPictures = 6;
        if (!cut && currentStep > 1e-6f)
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

    // Command processor: the reprojection of the picture whose draws are being run (picture = swaps run so far),
    // with the camera's step smoothed; the identity when none was recorded.
    inline Matrix LookupSmoothed(uint64_t picture)
    {
        std::lock_guard lock(g_mutex);
        const Entry& e = g_entries[picture % g_entries.size()];
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

    // The pixel shader constants the blur reads, from the slot of the
    // reprojection matrix (kSlots of them): the camera's reprojection.
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


    inline bool h_pushLoose(MeshHistory& h, uint64_t frame, uint64_t picture, const Matrix& clip)
    {
        if (h.draws.size() < 512)
            h.draws.push_back({ frame, picture, clip });
        return true;
    }

    // The draws of a prop (a barrel, a fence, a lightpost) or of the scenery that no car owns: the draw of the same mesh in
    // the picture before. The camera's reprojection goes from a picture to the one before it, so the draws of the
    // picture before are the ones to compare with. Of a mesh drawn many times (trees) an instance is static when some
    // earlier draw of the mesh gives exactly the camera's motion; only when none does is an earlier draw taken as the
    // object's own: the only one within 1.5 m, or one clearly nearer than the rest.
    inline DrawMotion MatchDraw(MeshHistory& h, uint64_t frame, int offset, const PictureMatrices& now, const Matrix& world, const Matrix& clip, Matrix& reprojection)
    {
        const uint64_t picture = frame + offset;
        PictureMatrices before;
        Matrix inverse;
        bool found = false, isStatic = false;
        if (!now.cut && picture > 0 && LookupPicture(picture - 1, before) && Inverse(clip, inverse))
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
                const Matrix candidate = Multiply(inverse, p.clip);
                if (deviation(candidate, 0.03f) <= 0.003f)
                {
                    isStatic = found = true;
                    break;
                }
                const Matrix w = Multiply(p.clip, before.inverseViewProjection);
                if (candidateCount < 64)
                    candidates[candidateCount++] = { &p, { world[12] - w[12], world[13] - w[13], world[14] - w[14] } };
            }
            auto distance = [](const float* v) { return std::max({ std::fabs(v[0]), std::fabs(v[1]), std::fabs(v[2]) }); };
            const Candidate* pick = nullptr;
            if (!isStatic && candidateCount == 1 && distance(candidates[0].displacement) < 1.5f)
                pick = &candidates[0];
            else if (!isStatic && candidateCount > 1)
            {
                float best = 1.5f, second = 1e9f;
                for (uint32_t i = 0; i < candidateCount; i++)
                {
                    const float r = distance(candidates[i].displacement);
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
                if (pick && second < best * 2.0f + 0.05f)
                    pick = nullptr;
            }
            if (pick)
            {
                reprojection = Multiply(inverse, pick->draw->clip);
                found = true;
            }
        }
        if (h.draws.size() < 512)
            h.draws.push_back({ frame, picture, clip });
        if (!found)
            return DrawMotion::NoPrevious;
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
        float world[3] = {}, carPosition[3] = {};  // the part's origin and its car's, in the world
    };
    struct LocalCar
    {
        uint64_t frame = ~0ull;
        std::vector<LocalPart> now, before;
    };
    inline std::unordered_map<uint32_t, LocalCar> g_localCars;
    // Set by the caller for a vertex shader that also draws scenery (trees, props): its parts only belong to a
    // car when they are inside the car's box, matched before or not.
    inline bool g_strictOwnership = false;
    // The vertex buffers of the draw being noted (guest physical, masked), set by the caller: a car whose part
    // table is known only owns the draws that use one of its buffers.
    inline uint32_t g_drawBuffers[10];
    inline int g_drawBufferCount = 0;
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

    inline bool VehicleLocalMotion(uint64_t frame, uint64_t key, const Matrix& clip, Matrix& reprojection)
    {
        // How far a part's place on its car may move between pictures and still be the same part (wheel hop,
        // a spoiler's shake are centimetres). Anything that moves more relative to the car is not attached
        // to it: a sign or a cone beside the car, whose place on it changes by the car's own step. It was
        // 0.35 m, which let such things match and take the car's motion.
        constexpr float kPartMatch = 0.25f;
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
                const int owns = CarOwnsDraw(v);
                // The draw's origin is L's translation from the car's.
                const float d = std::hypot(w[12] - v.world[12], w[13] - v.world[13], w[14] - v.world[14]);
                if (d < (owns == 1 ? 8.0f : 4.0f) && candidateCount < 12)
                {
                    Candidate& c = candidates[candidateCount++] = { k, v.id, v.world, p.inverseViewProjection, e.cut, d, e.smoothedPrevious, {}, {}, owns };
                    memcpy(c.center, v.center, sizeof(c.center));
                    memcpy(c.half, v.half, sizeof(c.half));
                }
            }
        }
        if (!candidateCount)
            return false;
        Matrix inverseClip;
        if (!Inverse(clip, inverseClip))
            return false;
        int chosen = -1;
        float chosenScore = 1e9f;
        Matrix local{};
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
            }
        }
        if (chosen < 0 || chosenScore >= 1e8f)
        {
            // Remember where the part sat on the nearest car: a part that sits there again in the next
            // picture is attached to it (a prop or a sign beside the car moves by the car's own step), even
            // when it is outside the box (a wheel or a grille of a traffic car's the box is too small for).
            if (declinedId && !g_strictOwnership)
            {
                LocalCar& car = g_localCars[declinedId];
                if (car.frame == frame && car.now.size() < 256)
                {
                    LocalPart part{ key, declinedLocal };
                    memcpy(part.world, declinedWorld, sizeof(part.world));
                    memcpy(part.carPosition, declinedCarPosition, sizeof(part.carPosition));
                    car.now.push_back(part);
                }
            }
            return false;
        }
        const int bestK = candidates[chosen].k;
        const uint32_t bestId = candidates[chosen].id;
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
        float nearest = kPartMatch;
        for (const LocalPart& p : car.before)
            if (p.key == key)
            {
                const float d = std::hypot(local[12] - p.l[12], local[13] - p.l[13], local[14] - p.l[14]);
                if (d < nearest)
                {
                    nearest = d;
                    localBefore = p.l;
                }
            }
        if (car.now.size() < 256)
        {
            LocalPart part{ key, local };
            memcpy(part.world, chosenWorld, sizeof(part.world));
            memcpy(part.carPosition, chosenCarPosition, sizeof(part.carPosition));
            car.now.push_back(part);
        }
        if (!haveBefore)
            return false;
        reprojection = Multiply(inverseClip, Multiply(Multiply(localBefore, carBefore), viewProjectionBefore));
        return true;
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
                return true;
        return false;
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

        // A listed car's part (found by the car's part table): its motion on the car, then the car's.
        if (VehicleLocalMotion(frame, looseMesh, clip, reprojection))
            return DrawMotion::Moving;
        // Anything else: the same mesh in the picture before (static scenery, or a prop that moves).
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
            motion = MatchDraw(loose, frame, offset, now, world, clip, reprojection);
        else
            h_pushLoose(loose, frame, picture, clip);
        return motion;
    }
}
