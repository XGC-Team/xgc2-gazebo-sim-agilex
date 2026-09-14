#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace wescore {

// All speeds here are signed: body +x forward, yaw +z counter-clockwise,
// and positive joint speed drives either side forward. Dimensions are SI.
struct WheelAllocationConfig {
  double radius_m = 0.08;
  double allocation_track_m = 0.416503;
  double common_gain = 1.0;
  double differential_gain = 1.0;
  double wheel_limit_rad_s = 24.0;
};

struct WheelTargets {
  // FR, FL, RL, RR; these are joint targets, not motor-shaft RPM.
  std::array<double, 4> radians_per_second;
  double unsaturated_left;
  double unsaturated_right;
  bool saturated;
};

inline void ValidateAllocation(const WheelAllocationConfig &c) {
  if (!std::isfinite(c.radius_m) || c.radius_m <= 0.0 ||
      !std::isfinite(c.allocation_track_m) || c.allocation_track_m <= 0.0 ||
      !std::isfinite(c.common_gain) || c.common_gain <= 0.0 ||
      !std::isfinite(c.differential_gain) || c.differential_gain <= 0.0 ||
      !std::isfinite(c.wheel_limit_rad_s) || c.wheel_limit_rad_s <= 0.0) {
    throw std::invalid_argument("wheel allocation: finite positive SI parameters required");
  }
}

inline WheelTargets AllocateWheelTargets(double linear_m_s, double yaw_rad_s,
                                         const WheelAllocationConfig &c) {
  ValidateAllocation(c);
  if (!std::isfinite(linear_m_s) || !std::isfinite(yaw_rad_s)) {
    throw std::invalid_argument("wheel allocation: non-finite body command");
  }
  const double half_span = 0.5 * c.allocation_track_m * c.differential_gain;
  const double left = c.common_gain * (linear_m_s - half_span * yaw_rad_s) / c.radius_m;
  const double right = c.common_gain * (linear_m_s + half_span * yaw_rad_s) / c.radius_m;
  if (!std::isfinite(left) || !std::isfinite(right)) {
    throw std::overflow_error("wheel allocation: target overflow");
  }
  const double left_limited = std::max(-c.wheel_limit_rad_s, std::min(c.wheel_limit_rad_s, left));
  const double right_limited = std::max(-c.wheel_limit_rad_s, std::min(c.wheel_limit_rad_s, right));
  return {{{right_limited, left_limited, left_limited, right_limited}}, left, right,
          left != left_limited || right != right_limited};
}

// This is a coordinate decomposition of wheel tread speeds, NOT a chassis
// forward-kinematics model. Contact, slip and inertia still determine v/w.
inline std::array<double, 2> WheelTreadModes(const WheelTargets &t,
                                          double radius_m) {
  if (!std::isfinite(radius_m) || radius_m <= 0.0) {
    throw std::invalid_argument("wheel tread modes: invalid radius");
  }
  for (double value : t.radians_per_second) {
    if (!std::isfinite(value)) throw std::invalid_argument("invalid wheel target");
  }
  const double right = 0.5 * (t.radians_per_second[0] + t.radians_per_second[3]);
  const double left = 0.5 * (t.radians_per_second[1] + t.radians_per_second[2]);
  return {{0.5 * radius_m * (right + left), radius_m * (right - left)}};
}

}  // namespace wescore
