#!/usr/bin/env python3
"""ROS-free queue/scheduling regression, not a Gazebo or ROS delivery test."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SCOUT = Path(__file__).resolve().parents[1]


class CommandClockTest(unittest.TestCase):
    def test_scheduler_uses_plant_time_and_independent_hold(self):
        source = (SCOUT / 'src/scout_skid_steer.cpp').read_text()
        header = (SCOUT / 'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('ros::Timer control_timer_', header)
        self.assertIn('nh_->createTimer(', source)
        self.assertIn('ros::Duration(0.001)', source)
        self.assertNotIn('nh_->createWallTimer(', source)
        self.assertIn('command_delay_.Advance(ros::Time::now().toSec())', source)
        self.assertIn('hold_gate_.setZeroThunk(&ScoutSkidSteer::HoldZeroThunk, this)', source)
        self.assertIn('command_delay_.Reset();', source)
        self.assertIn('"command_delay_s", command_delay_s_, 0.005', source)

    def test_actual_delay_kernel_on_physics_grids(self):
        code = r'''
#include "scout_gazebo/command_delay.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  for (const int step_us : {1000, 2000, 4000}) {
    // The wall-time factor does not appear in the queue or ROS-time schedule.
    // This checks the queue contract only; callback lateness needs a ROS run.
    for (const double wall_factor : {0.2, 1.0, 2.0, 5.0}) {
      (void)wall_factor;
      wescore::CommandDelay queue;
      queue.Configure(0.005);
      queue.Push(1.0, 0.5, -0.3);
      int released_us = -1;
      for (int elapsed_us = 0; elapsed_us <= 12000; elapsed_us += step_us) {
        const auto output = queue.Advance(1.0 + elapsed_us * 1.e-6);
        if (output.linear != 0.0 && released_us < 0) released_us = elapsed_us;
        if (elapsed_us < 5000) assert(output.linear == 0.0);
      }
      assert(released_us >= 5000 && released_us < 5000 + step_us);
      assert(queue.Advance(1.02).linear == 0.5);
      // A final single zero must be released without a later input callback.
      queue.Push(1.03, 0.0, 0.0);
      assert(queue.Advance(1.034).linear == 0.5);
      assert(queue.Advance(1.036).linear == 0.0);
      queue.Push(2.0, 0.5, 0.2);
      for (int tick = 0; tick < 100; ++tick)
        assert(queue.Advance(2.0).linear == 0.0); // pause
      assert(queue.Advance(2.006).linear == 0.5);
      assert(queue.Advance(0.0).linear == 0.0); // rewind clears old command
      assert(queue.Advance(10.0).linear == 0.0);
      queue.Push(10.0, 0.5, 0.2);
      queue.Reset(); // Hold must clear even pending commands
      assert(queue.Advance(10.1).linear == 0.0);
      std::cout << step_us << ',' << wall_factor << ',' << released_us << '\n';
    }
  }
}
'''
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / 'clock.cpp'
            binary = directory / 'clock'
            source.write_text(code)
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(SCOUT / 'include'), str(source), '-o', str(binary)],
                           check=True, timeout=30)
            result = subprocess.run([str(binary)], check=True, capture_output=True,
                                    text=True, timeout=10)
            self.assertEqual(len(result.stdout.strip().splitlines()), 12)


if __name__ == '__main__':
    unittest.main()
