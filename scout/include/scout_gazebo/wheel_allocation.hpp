#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace wescore {

// Body convention: +x forward, +y left, +yaw counter-clockwise.
// Output order: front-right, front-left, rear-left, rear-right [rad/s].
// allocation_span_m is a command-map coefficient, NOT the contact track.
// The effective coefficient is identifiable from RPM common/difference modes
// even when the RPM-to-wheel gear ratio is not independently known.
struct WheelAllocation {
  double radius_m = 0.08;
  double allocation_span_m = 0.416503;
  double common_gain = 1.0;

  void Validate() const {
    if (!std::isfinite(radius_m) || radius_m <= 0.0 ||
        !std::isfinite(allocation_span_m) || allocation_span_m <= 0.0 ||
        !std::isfinite(common_gain) || common_gain <= 0.0) {
      throw std::invalid_argument("wheel allocation: radius, span and gain must be finite and positive");
    }
  }

  std::array<double, 4> Apply(double linear_mps, double yaw_radps) const {
    Validate();
    if (!std::isfinite(linear_mps) || !std::isfinite(yaw_radps)) {
      throw std::invalid_argument("wheel allocation: non-finite body command");
    }
    const double half_difference = 0.5 * allocation_span_m * yaw_radps;
    const double right = common_gain * (linear_mps + half_difference) / radius_m;
    const double left = common_gain * (linear_mps - half_difference) / radius_m;
    if (!std::isfinite(left) || !std::isfinite(right)) {
      throw std::overflow_error("wheel allocation: wheel target overflow");
    }
    return {{right, left, left, right}};
  }
};

}  // namespace wescore
