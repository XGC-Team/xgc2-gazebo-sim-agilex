#!/usr/bin/env python3
"""Pure-delay numerical regression plus a production wiring guard; no Gazebo."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]

class InnerLoopDelayTest(unittest.TestCase):
    def test_production_wiring(self):
        source=(PACKAGE/'src/scout_skid_steer.cpp').read_text()
        header=(PACKAGE/'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('command_delay_.Configure(command_delay_s_);',source)
        self.assertNotIn('command_time_constant', source)
        self.assertNotIn('command_tau', source)
        self.assertIn('private_nh.param("command_delay_s", command_delay_s_, 0.005);',source)
        self.assertIn('nh_->createTimer(', source)
        self.assertIn('ros::Duration(0.001)', source)
        self.assertIn('ros::Timer control_timer_', header)
        self.assertNotIn('createWallTimer(', source)
        self.assertNotIn('ros::WallTimer', header)
        # Paused-clock hold must not rely on the stopped simulation timer.
        self.assertIn('hold_gate_.setZeroThunk(&ScoutSkidSteer::HoldZeroThunk, this);', source)
        self.assertIn('static_cast<ScoutSkidSteer *>(self)->PublishZeroMotors();', source)

    def test_actual_delay_core(self):
        code=r'''
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
 d.Push(.25,0,0);
 eq(d.Advance(.374).linear,.5); eq(d.Advance(.375).linear,0);
 d.Push(.5,-.5,-.3); eq(d.Advance(.625).linear,-.5);
 eq(d.Advance(.625).angular,-.3);
 eq(d.Advance(.1).linear,0);
 d.Configure(0);d.Push(1,.2,-.1);eq(d.Advance(1).linear,.2);
 d.Reset();eq(d.Advance(2).linear,0);
 // Fixed 5 ms delay at 1/2/4 ms simulation-step resolution. The synthetic
 // wall pause repetitions must not mature commands without clock progress.
 for(int h_ms : {1,2,4}) {
   for(int wall_repetitions : {1,10,100}) {
     d.Configure(.005); d.Push(0,.5,-.3);
     for(int i=0;i<wall_repetitions;++i) eq(d.Advance(0).linear,0);
     int first_ms=-1;
     for(int t_ms=h_ms;t_ms<=20;t_ms+=h_ms) {
       const auto out=d.Advance(t_ms*.001);
       if(out.linear!=0 && first_ms<0) first_ms=t_ms;
       if(t_ms<5) eq(out.linear,0);
     }
     const int expected=((5+h_ms-1)/h_ms)*h_ms;
     if(first_ms!=expected) throw std::runtime_error("step quantization changed");
   }
 }
 // Multiple due commands collapse to the newest, including one final stop.
 d.Configure(.005);d.Push(0,.5,.3);d.Push(.001,-.2,-.1);d.Push(.002,0,0);
 eq(d.Advance(.008).linear,0);eq(d.Advance(100).angular,0);
 // Sparse sampling is a late dispatch, never an artificial ramp.
 d.Configure(.005);d.Push(0,.5,.3);eq(d.Advance(.080).linear,.5);
 std::cout<<"pure delay: stop/reverse/pause/rewind; 9 step/pause cases; burst and sparse dispatch passed\n";
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp=Path(directory)/'test.cpp'; exe=Path(directory)/'test';cpp.write_text(code)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                            '-I'+str(PACKAGE/'include'),str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
