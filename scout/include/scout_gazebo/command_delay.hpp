#pragma once

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
class CommandDelay {
 public:
  void Configure(double delay) {
    if (!std::isfinite(delay) || delay < 0.0) {
      throw std::invalid_argument("command delay must be finite and nonnegative");
    }
    delay_ = delay;
    Reset();
  }

  void Reset() {
    history_.clear();
    delayed_ = {0.0, 0.0};
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
    // Consume every due command, retaining the last one. This is a pure
    // transport delay and zero-order hold, without an actuator response model.
    while (!history_.empty() && history_.front().due <= now) {
      const Command next = history_.front();
      delayed_ = {next.linear, next.angular};
      history_.pop_front();
    }
    last_time_ = now;
    return delayed_;
  }

 private:
  struct Command { double due; double linear; double angular; };

  std::deque<Command> history_;
  CommandVelocity delayed_{0.0, 0.0};
  double delay_ = 0.0;
  double last_time_ = 0.0;
  bool initialized_ = false;
};

}  // namespace wescore
