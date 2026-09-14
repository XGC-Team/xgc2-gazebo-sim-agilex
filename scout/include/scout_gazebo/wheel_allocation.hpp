#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace wescore {

// Body +x is forward, +z yaw is counterclockwise. Positive joint angular
// velocity drives forward after the left/right URDF axis transforms.
// Inputs: linear [m/s], angular [rad/s], radius/separation [m], gains [1].
// Output order is FR, FL, RL, RR, in joint rad/s (not motor RPM).
// separation*angular_gain is an allocation span, not a contact track width.
inline std::array<double, 4> AllocateWheelSpeeds(
    double linear, double angular, double radius, double separation,
    double common_gain, double angular_gain) {
  if (!std::isfinite(linear) || !std::isfinite(angular) ||
      !std::isfinite(radius) || radius <= 0.0 ||
      !std::isfinite(separation) || separation <= 0.0 ||
      !std::isfinite(common_gain) || !std::isfinite(angular_gain)) {
    throw std::invalid_argument("Wheel allocation requires finite SI inputs and positive geometry");
  }
  const double differential = angular * angular_gain * separation * 0.5;
  const double left = common_gain * (linear - differential) / radius;
  const double right = common_gain * (linear + differential) / radius;
  if (!std::isfinite(left) || !std::isfinite(right)) {
    throw std::overflow_error("Wheel allocation overflow");
  }
  return {{right, left, left, right}};
}

}  // namespace wescore
