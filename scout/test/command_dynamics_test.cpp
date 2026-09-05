#include "scout_gazebo/command_dynamics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void Near(double actual, double expected) {
  Check(std::abs(actual - expected) < 1.0e-10, "unexpected command response");
}

void FinalZeroExecutesWithoutFurtherInput() {
  wescore::CommandDynamics dynamics;
  dynamics.Configure(0.15, 0.15);
  dynamics.Push(10.0, 1.0, -0.4);
  Check(dynamics.Advance(12.0).linear > 0.99, "initial drive must become active");
  dynamics.Push(12.0, 0.0, 0.0);
  // No more Push calls. The independent scheduler still completes the stop.
  const auto stopped = dynamics.Advance(17.0);
  Near(stopped.linear, 0.0);
  Near(stopped.angular, 0.0);
}

void SamplingDoesNotChangeThePlant() {
  wescore::CommandDynamics sparse, frequent;
  sparse.Configure(0.15, 0.15);
  frequent.Configure(0.15, 0.15);
  sparse.Push(10.0, 1.0, 0.3);
  frequent.Push(10.0, 1.0, 0.3);
  for (int i = 1; i <= 100; ++i) frequent.Advance(10.0 + i * 0.01);
  sparse.Push(11.0, -0.2, -0.1);
  frequent.Push(11.0, -0.2, -0.1);
  for (int i = 1; i <= 100; ++i) frequent.Advance(11.0 + i * 0.01);
  const auto a = sparse.Advance(12.0);
  const auto b = frequent.Advance(12.0);
  Near(a.linear, b.linear);
  Near(a.angular, b.angular);
}

void PauseResetAndInstantResponse() {
  wescore::CommandDynamics dynamics;
  dynamics.Configure(0.15, 0.15);
  dynamics.Push(10.0, 1.0, 0.2);
  const auto value = dynamics.Advance(10.5);
  for (int i = 0; i < 20; ++i) Near(dynamics.Advance(10.5).linear, value.linear);
  Near(dynamics.Advance(1.0).linear, 0.0);
  Near(dynamics.Advance(20.0).linear, 0.0);
  dynamics.Configure(0.0, 0.0);
  dynamics.Push(20.0, 0.6, -0.3);
  Near(dynamics.Advance(20.0).linear, 0.6);
  dynamics.Push(21.0, 0.0, 0.0);
  Near(dynamics.Advance(21.0).linear, 0.0);
  dynamics.Push(22.0, 1.0, 0.0);
  dynamics.Reset();
  Near(dynamics.Advance(23.0).linear, 0.0);
}

void DelayedStepAndInvalidInput() {
  wescore::CommandDynamics dynamics;
  dynamics.Configure(0.5, 0.0);
  dynamics.Push(0.0, 1.0, 0.2);
  Near(dynamics.Advance(0.49).linear, 0.0);
  Near(dynamics.Advance(0.5).linear, 1.0);
  dynamics.Push(0.5, 0.0, 0.0);
  Near(dynamics.Advance(0.99).linear, 1.0);
  Near(dynamics.Advance(1.0).linear, 0.0);
  bool rejected = false;
  try { dynamics.Configure(-1.0, 0.0); }
  catch (const std::invalid_argument&) { rejected = true; }
  Check(rejected, "negative delay must be rejected");
  rejected = false;
  try { dynamics.Push(2.0, std::numeric_limits<double>::quiet_NaN(), 0.0); }
  catch (const std::invalid_argument&) { rejected = true; }
  Check(rejected, "non-finite input must be rejected");
}
}  // namespace

int main() {
  FinalZeroExecutesWithoutFurtherInput();
  SamplingDoesNotChangeThePlant();
  PauseResetAndInstantResponse();
  DelayedStepAndInvalidInput();
  std::cout << "Command dynamics: 4 scenario groups passed\n";
}
