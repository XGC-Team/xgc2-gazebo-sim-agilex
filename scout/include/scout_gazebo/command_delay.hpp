#pragma once

#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace wescore {

struct CommandVelocity {
  double linear;
  double angular;
};

struct DelayedCommandSample {
  CommandVelocity velocity{0.0, 0.0};
  double received_s = 0.0;
  double due_s = 0.0;
  std::uint64_t sequence = 0U;
  bool valid = false;
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
    delayed_ = DelayedCommandSample{};
    initialized_ = false;
    last_time_ = 0.0;
  }

  void Push(double now, double linear, double angular) {
    if (!std::isfinite(now) || !std::isfinite(now + delay_) ||
        !std::isfinite(linear) || !std::isfinite(angular)) {
      throw std::invalid_argument("command and simulation time must be finite");
    }
    Advance(now);
    DelayedCommandSample sample;
    sample.velocity = {linear, angular};
    sample.received_s = now;
    sample.due_s = now + delay_;
    sample.sequence = ++sequence_;
    sample.valid = true;
    history_.push_back(sample);
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
    while (!history_.empty() && history_.front().due_s <= now) {
      delayed_ = history_.front();
      history_.pop_front();
    }
    last_time_ = now;
    return delayed_.velocity;
  }

  // Caller records its own dispatch time after Advance(). A due time is not
  // a wheel-publication time or the later physics step's applied-torque time.
  const DelayedCommandSample &Sample() const { return delayed_; }

 private:
  std::deque<DelayedCommandSample> history_;
  DelayedCommandSample delayed_;
  double delay_ = 0.0;
  double last_time_ = 0.0;
  std::uint64_t sequence_ = 0U;
  bool initialized_ = false;
};

}  // namespace wescore
