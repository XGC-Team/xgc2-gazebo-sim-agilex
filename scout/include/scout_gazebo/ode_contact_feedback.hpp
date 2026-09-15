#ifndef SCOUT_GAZEBO_ODE_CONTACT_FEEDBACK_HPP
#define SCOUT_GAZEBO_ODE_CONTACT_FEEDBACK_HPP

#include <array>
#include <cmath>
#include <cstddef>

namespace scout_gazebo {
namespace ode_contact {

using Vector = std::array<double, 3>;

// Columns of the wheel LINK's pre-integration link-to-world rotation. Keeping
// this adapter independent of Gazebo lets the production projection be tested
// without substituting a different implementation in the test.
struct Frame {
    std::array<Vector, 3> columns{};
    double time_s{-1};
    bool valid{false};
};

enum class Rejection { none, missing_frame, wrong_time, nonfinite, nonpositive };

struct Load {
    Vector raw{};
    Vector world{};
    double normal_n{0};
    Rejection rejection{Rejection::missing_frame};
};

inline bool finite(const Vector& v) {
    for (double x : v) if (!std::isfinite(x)) return false;
    return true;
}

// Gazebo Classic ODE stores each body's wrench in that body's LINK frame,
// despite Contact.hh's world-frame comment. Contact.time and beforePhysicsUpdate
// share the same simulation stamp. A frame is valid for exactly that step; the
// caller consumes/invalidates it at UpdateEnd and on pause/reset/rewind.
inline Load restore(const Frame& frame, double contact_time_s,
                    double collection_time_s, bool wheel_is_body1,
                    const Vector& body1_force, const Vector& body2_force,
                    const Vector& world_up) {
    Load result;
    if (!frame.valid) return result;
    if (!std::isfinite(frame.time_s) || !std::isfinite(contact_time_s) ||
        !std::isfinite(collection_time_s)) {
        result.rejection = Rejection::nonfinite;
        return result;
    }
    constexpr double stamp_tolerance_s = 1e-9;
    if (std::abs(contact_time_s - frame.time_s) > stamp_tolerance_s ||
        std::abs(collection_time_s - frame.time_s) > stamp_tolerance_s) {
        result.rejection = Rejection::wrong_time;
        return result;
    }
    // Do not negate body2 or use the other body's rotation. Only the selected
    // body's wrench is relevant (the other body may have a different frame).
    result.raw = wheel_is_body1 ? body1_force : body2_force;
    if (!finite(result.raw) || !finite(world_up)) {
        result.rejection = Rejection::nonfinite;
        return result;
    }
    for (std::size_t c = 0; c < 3; ++c) {
        if (!finite(frame.columns[c])) {
            result.rejection = Rejection::nonfinite;
            return result;
        }
        for (std::size_t r = 0; r < 3; ++r)
            result.world[r] += frame.columns[c][r] * result.raw[c];
    }
    double support = 0;
    for (std::size_t r = 0; r < 3; ++r) support += result.world[r] * world_up[r];
    // Validate BEFORE clipping. std::max(0, NaN) can silently turn bad feedback
    // into zero load; an absolute value or a force norm would invent support.
    if (!finite(result.world) || !std::isfinite(support)) {
        result.rejection = Rejection::nonfinite;
        return result;
    }
    if (support <= 0) {
        result.rejection = Rejection::nonpositive;
        return result;
    }
    result.normal_n = support;
    result.rejection = Rejection::none;
    return result;
}

} // namespace ode_contact
} // namespace scout_gazebo
#endif
