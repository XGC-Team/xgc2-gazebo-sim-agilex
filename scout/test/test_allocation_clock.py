#!/usr/bin/env python3
"""Compile production allocation/delay headers; guard ROS-time node wiring."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class AllocationClockTest(unittest.TestCase):
    def test_production_headers(self):
        code = r'''
#include "scout_gazebo/wheel_allocation.hpp"
#include "scout_gazebo/command_delay.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  wescore::WheelAllocation a, b;
  constexpr double radius=.08, track=.416503, gain=1.04, yaw_gain=2.306318917;
  for (int i=-20; i<=20; ++i) for (int j=-20; j<=20; ++j) {
    double v=.05*i, w=.05*j;
    assert(wescore::AllocateScoutWheels(v,w,radius,track,gain,yaw_gain,a));
    assert(a.target[0]==a.target[3] && a.target[1]==a.target[2]);
    assert(std::abs(radius*(a.target[0]+a.target[1])/(2*gain)-v)<1e-12);
    assert(std::abs(radius*(a.target[0]-a.target[1])/(gain*a.effective_yaw_span)-w)<1e-12);
    assert(wescore::AllocateScoutWheels(-v,-w,radius,track,gain,yaw_gain,b));
    for (int k=0;k<4;++k) assert(std::abs(a.target[k]+b.target[k])<1e-12);
    assert(wescore::AllocateScoutWheels(v,-w,radius,track,gain,yaw_gain,b));
    assert(std::abs(a.target[0]-b.target[1])<1e-12);
  }
  assert(!wescore::AllocateScoutWheels(1,0,0,track,gain,yaw_gain,a));
  assert(!wescore::AllocateScoutWheels(std::numeric_limits<double>::quiet_NaN(),0,radius,track,gain,yaw_gain,a));
  assert(!wescore::AllocateScoutWheels(1,1,radius,1e308,1,1e308,a));
  for (double value:a.target) assert(value==0);
  wescore::CommandDelay queue;
  queue.Configure(.005);
  queue.Push(1.,.5,.3);
  assert(queue.Advance(1.004).linear==0);
  assert(queue.Advance(1.005).linear==.5);
  assert(queue.Advance(1.005).angular==.3);
  queue.Push(1.020,0.,0.);
  assert(queue.Advance(1.024).linear==.5);
  assert(queue.Advance(1.025).linear==0.);
  queue.Push(1.030,-.5,-.3);
  assert(queue.Advance(1.040).linear==-.5);
  assert(queue.Advance(.5).linear==0.);
  queue.Push(.6,.5,.3);
  queue.Reset();
  assert(queue.Advance(.7).linear==0.);
  std::cout << "1681 allocation cases and delay/stop/reversal/reset cases passed\n";
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'test.cpp'; binary=Path(directory)/'test'
            source.write_text(code)
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                            '-I'+str(ROOT/'include'),str(source),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_node_clock_and_allocation_wiring(self):
        source=(ROOT/'src/scout_skid_steer.cpp').read_text()
        header=(ROOT/'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('nh_->createTimer(', source)
        self.assertIn('ros::Duration(0.01)', source)
        self.assertIn('ros::Timer control_timer_', header)
        self.assertNotIn('createWallTimer', source)
        self.assertIn('command_delay_.Configure(command_delay_s_)',source)
        self.assertNotIn('command_time_constant',source)
        self.assertIn('AllocateScoutWheels(command.linear, command.angular', source)
        self.assertIn('hold_gate_.setZeroThunk',source)
        self.assertIn('command_delay_.Reset()',source)

if __name__=='__main__': unittest.main()
