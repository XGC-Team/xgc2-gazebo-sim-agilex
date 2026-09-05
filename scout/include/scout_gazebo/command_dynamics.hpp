#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace wescore {

struct CommandVelocity {
  double linear;
  double angular;
};

// Receipt and execution are separate. Advance() must be called by the control
// scheduler even when no new command arrives. Its time argument is ROS/sim time.
class CommandDynamics {
 public:
  void Configure(double delay, double time_constant) {
    if (!std::isfinite(delay) || delay < 0.0 ||
        !std::isfinite(time_constant) || time_constant < 0.0) {
      throw std::invalid_argument("command delay and time constant must be finite and nonnegative");
    }
    delay_ = delay;
    time_constant_ = time_constant;
    Reset();
  }

  void Reset() {
    history_.clear();
    delayed_ = {0.0, 0.0};
    filtered_ = {0.0, 0.0};
    initialized_ = false;
    last_time_ = 0.0;
  }

  void Push(double now, double linear, double angular) {
    if (!std::isfinite(now) || !std::isfinite(now + delay_) ||
        !std::isfinite(linear) || !std::isfinite(angular)) {
      throw std::invalid_argument("command and simulation time must be finite");
    }
    Advance(now);
    history_.push_back({now + delay_, linear, angular});
    // Preserve the original bounded history and retain the newest command,
    // including a final zero. Normal command rates stay well below this limit.
    while (history_.size() > 2048U) history_.pop_front();
  }

  CommandVelocity Advance(double now) {
    if (!std::isfinite(now)) {
      throw std::invalid_argument("simulation time must be finite");
    }
    if (!initialized_ || now < last_time_) {
      Reset();
      initialized_ = true;
      last_time_ = now;
    }
    // Split integration at each delayed input transition. This makes response
    // independent of the arrival/timer sampling frequency, including long gaps.
    while (!history_.empty() && history_.front().due <= now) {
      const Command next = history_.front();
      const double transition = std::max(last_time_, next.due);
      Integrate(transition - last_time_);
      delayed_ = {next.linear, next.angular};
      last_time_ = transition;
      history_.pop_front();
    }
    Integrate(now - last_time_);
    last_time_ = now;
    return filtered_;
  }

 private:
  struct Command { double due; double linear; double angular; };
  void Integrate(double dt) {
    const double alpha = time_constant_ > 0.0 ? -std::expm1(-dt / time_constant_) : 1.0;
    filtered_.linear += alpha * (delayed_.linear - filtered_.linear);
    filtered_.angular += alpha * (delayed_.angular - filtered_.angular);
  }

  std::deque<Command> history_;
  CommandVelocity delayed_{0.0, 0.0};
  CommandVelocity filtered_{0.0, 0.0};
  double delay_ = 0.15;
  double time_constant_ = 0.15;
  double last_time_ = 0.0;
  bool initialized_ = false;
};

}  // namespace wescore
