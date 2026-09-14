#!/usr/bin/env python3
"""Compile the production delay core; check dispatch wiring without ROS/Gazebo.

The simulated scheduler below tests time-domain arithmetic, not roscpp callback
latency. A real /clock replay and Gazebo acceptance remain separate gates.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]

class InnerLoopDelayTest(unittest.TestCase):
    def test_production_wiring(self):
        source = (PACKAGE/'src/scout_skid_steer.cpp').read_text()
        header = (PACKAGE/'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('command_delay_.Configure(command_delay_s_);', source)
        self.assertNotIn('command_time_constant', source)
        self.assertNotIn('command_tau', source)
        self.assertIn('private_nh.param("command_delay_s", command_delay_s_, 0.005);', source)
        self.assertIn('nh_->createTimer(', source)
        self.assertIn('ros::Duration(0.001)', source)
        self.assertNotIn('createWallTimer(', source)
        self.assertIn('ros::Timer control_timer_;', header)
        self.assertIn('ControlTick(const ros::TimerEvent', source)
        # Paused-plant HOLD must remain independent of the ROS-time timer.
        self.assertIn('hold_gate_.setZeroThunk(&ScoutSkidSteer::HoldZeroThunk, this);', source)
        self.assertIn('static_cast<ScoutSkidSteer *>(self)->PublishZeroMotors();', source)

    def test_actual_delay_core(self):
        code = r'''
#include "scout_gazebo/command_delay.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void eq(double a,double b){if(std::fabs(a-b)>1e-12)throw std::runtime_error("delay mismatch");}
int main(){
 wescore::CommandDelay d; d.Configure(.125);
 d.Push(0,.5,.3);
 eq(d.Advance(.124).linear,0); eq(d.Advance(.125).linear,.5);
 eq(d.Advance(.125).angular,.3);
 d.Push(.25,0,0); // one stop message, no heartbeat required
 eq(d.Advance(.374).linear,.5); eq(d.Advance(.375).linear,0);
 d.Push(.5,-.5,-.3); eq(d.Advance(.625).linear,-.5);
 eq(d.Advance(.625).angular,-.3);
 eq(d.Advance(.1).linear,0);
 d.Configure(0);d.Push(1,.2,-.1);eq(d.Advance(1).linear,.2);
 d.Reset();eq(d.Advance(2).linear,0);
 // Use binary-exact microsecond integer grids before converting to seconds.
 // Same physics-time observations, any wall-time dilation, same queue output.
 int cases=0;
 for(int h_us : {1000,2000,4000}) {
  for(int phase_us=0;phase_us<h_us;phase_us+=125) {
   for(double rtf : {.1,.25,.5,1.,2.,4.}) {
    d.Configure(.005);
    const double receipt=(100000+phase_us)*1e-6;
    d.Push(receipt,.5,-.3);
    double first=-1.;
    for(int k=0;k<140000/h_us;++k) {
     const double sim=k*h_us*1e-6;
     const double wall=sim/rtf;
     (void)wall; // dispatch decisions must not depend on this clock
     if(sim<receipt) continue;
     auto out=d.Advance(sim);
     if(out.linear!=0. && first<0.) first=sim;
    }
    if(first<receipt+.005-1e-12 || first>receipt+.005+h_us*1e-6+1e-12)
      throw std::runtime_error("sampled release outside [delay,delay+grid]");
    // Stop is sampled even without another Push: no hidden command heartbeat.
    const double stop=.2;
    d.Push(stop,0,0);
    eq(d.Advance(.2).linear,.5);
    eq(d.Advance(.21).linear,0);
    ++cases;
   }
  }
 }
 // Counterexample to the old clock domain: 10 ms wall polls at RTF=4
 // first see a .005 s deadline at .040 s simulation time.
 eq(std::ceil(.005/(.01*4.))*.01*4., .040);
 std::cout << "6 FIFO scenarios and " << cases
           << " sampled-clock cases passed; ROS scheduler not emulated\n";
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory)/'test.cpp'
            exe = Path(directory)/'test'
            cpp.write_text(code)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                            '-I'+str(PACKAGE/'include'),str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__ == '__main__':
    unittest.main()
