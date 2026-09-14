#pragma once

#include <array>
#include <cmath>

namespace wescore {

// Body FLU: forward v [m/s], left-positive yaw rate [rad/s]. Wheel targets
// are joint-coordinate rad/s in FR, FL, RL, RR order. All four joint axes
// transform to the same body-frame rotation axis in the Scout model.
struct WheelAllocation {
  std::array<double, 4> target{{0.0, 0.0, 0.0, 0.0}};
  double effective_yaw_span = 0.0;  // [m], allocation ratio, NOT contact track
};

inline bool AllocateScoutWheels(double v, double yaw_rate, double radius,
                               double geometric_track, double common_gain,
                               double angular_gain, WheelAllocation &out) {
  out = WheelAllocation{};
  if (!std::isfinite(v) || !std::isfinite(yaw_rate) ||
      !std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(geometric_track) || geometric_track <= 0.0 ||
      !std::isfinite(common_gain) || !std::isfinite(angular_gain)) {
    return false;
  }
  const double span = geometric_track * angular_gain;
  const double differential = yaw_rate * span * 0.5;
  const double left = common_gain * (v - differential) / radius;
  const double right = common_gain * (v + differential) / radius;
  if (!std::isfinite(span) || !std::isfinite(left) || !std::isfinite(right)) {
    return false;
  }
  out.effective_yaw_span = span;
  out.target = {{right, left, left, right}};
  return true;
}

}  // namespace wescore
