#!/usr/bin/env python3
"""Compile the production allocation/delay kernels; no ROS/Gazebo runtime.

The numerical scheduler test supplies ideal tick timestamps. It is not a
measurement of roscpp callback latency, /clock cadence or Gazebo response.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


class SimClockSchedulerTest(unittest.TestCase):
    def test_normal_release_and_paused_hold_have_distinct_clocks(self):
        source = (PACKAGE / 'src/scout_skid_steer.cpp').read_text()
        header = (PACKAGE / 'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('ros::Timer control_timer_', header)
        self.assertIn('ros::WallTimer hold_timer_', header)
        self.assertIn('control_timer_ = nh_->createTimer(', source)
        self.assertIn('private_nh.param("command_delay_s", command_delay_s_, 0.005);', source)
        self.assertIn('private_nh.param("control_period_s", control_period_s_, 0.001);', source)
        hold = source.split('void ScoutSkidSteer::HoldTick', 1)[1].split(
            'void ScoutSkidSteer::ControlTick', 1)[0]
        self.assertIn('if (held) PublishZeroMotors();', hold)
        self.assertNotIn('Advance(', hold)
        self.assertNotIn('AllocateWheelSpeeds(', hold)
        self.assertIn('AllocateWheelSpeeds(', source)
        self.assertNotIn('command_time_constant', source)
        self.assertNotIn('command_tau', source)

    def test_actual_cpp_kernels(self):
        code = r'''
#include "scout_gazebo/command_delay.hpp"
#include "scout_gazebo/wheel_allocation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
void require(bool ok) { if (!ok) throw std::runtime_error("contract failure"); }
void near(double a, double b) { require(std::abs(a-b) < 1e-11); }
// Returns extra scheduling wait beyond the fixed five milliseconds.
double extra_wait(double receipt, double tick) {
  wescore::CommandDelay d; d.Configure(.005); d.Push(receipt, .5, -.3);
  long k = static_cast<long>(std::ceil(receipt/tick));
  for (int n=0; n<100; ++n, ++k) {
    const double t = k*tick;
    const auto v = d.Advance(t);
    if (v.linear != 0) {
      require(t >= receipt+.005);
      near(v.linear, .5); near(v.angular, -.3);
      return t-receipt-.005;
    }
  }
  throw std::runtime_error("queued command never released");
}
int main() {
  unsigned allocations = 0;
  for (int iv=-15; iv<=15; ++iv) for (int iw=-10; iw<=10; ++iw) {
    const double v=iv*.1, w=iw*.1, radius=.08, track=.416503;
    for (double angular_gain : {.8, 1., 2.3}) {
      const auto out=wescore::AllocateWheelSpeeds(v,w,radius,track,1.04,angular_gain);
      const double old_left=(v-(w*angular_gain)*(track*.5))/radius*1.04;
      const double old_right=(v+(w*angular_gain)*(track*.5))/radius*1.04;
      near(out[0],old_right); near(out[1],old_left);
      near(out[2],old_left); near(out[3],old_right);
      near(radius*(out[0]+out[1])*.5,1.04*v);
      near(radius*(out[0]-out[1]),1.04*angular_gain*track*w);
      ++allocations;
    }
  }
  bool invalid=false;
  try { wescore::AllocateWheelSpeeds(0,0,0,.4,1,1); }
  catch (const std::invalid_argument&) { invalid=true; }
  require(invalid);
  invalid=false;
  try { wescore::AllocateWheelSpeeds(std::numeric_limits<double>::quiet_NaN(),0,.08,.4,1,1); }
  catch (const std::invalid_argument&) { invalid=true; }
  require(invalid);
  std::cout << "{\"allocation_cases\":" << allocations << ",\"schedulers\":[";
  bool first=true;
  for (double rtf : {.1,1.,10.}) {
    double old_max=0, new_max=0;
    for (int i=0; i<1000; ++i) {
      const double receipt=.10100017+i*.000113;
      old_max=std::max(old_max,extra_wait(receipt,.01*rtf));
      new_max=std::max(new_max,extra_wait(receipt,.001));
    }
    require(old_max<=.01*rtf+1e-12); require(new_max<=.001+1e-12);
    if (!first) { std::cout << ","; }
    first=false;
    std::cout << "{\"rtf\":" << rtf << ",\"old_extra_max_sim_s\":" << old_max
              << ",\"new_extra_max_sim_s\":" << new_max << "}";
  }
  wescore::CommandDelay d; d.Configure(.005); d.Push(1,.5,.2);
  near(d.Advance(1).linear,0); near(d.Advance(1.005).linear,.5);
  near(d.Advance(1.005).linear,.5); // paused clock
  d.Push(1.1,0,0); near(d.Advance(1.104).linear,.5); near(d.Advance(1.106).linear,0);
  d.Push(1.2,-.5,-.2); near(d.Advance(1.206).linear,-.5);
  near(d.Advance(.1).linear,0); // rewind drops held and future motion
  d.Push(.2,.8,0); d.Reset(); near(d.Advance(.3).linear,0); // existing safety hold
  std::cout << "],\"pause_stop_reverse_rewind_hold\":true,\"ros_runtime_tested\":false}\n";
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            exe = Path(directory) / 'test'
            cpp.write_text(code)
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-Wall', '-Wextra',
                            '-Werror', '-I'+str(PACKAGE/'include'), str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
