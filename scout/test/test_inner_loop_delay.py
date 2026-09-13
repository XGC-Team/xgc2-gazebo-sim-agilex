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
        self.assertIn('command_dynamics_.Configure(command_delay_s_, 0.0);',source)
        self.assertIn('command_time_constant_s_ = 0.0;',source)
        self.assertIn('is disabled for the wheel-PI plant',source)
        self.assertIn('private_nh.param("command_delay_s", command_delay_s_, 0.005);',source)

    def test_actual_delay_core(self):
        code=r'''
#include "scout_gazebo/command_dynamics.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void eq(double a,double b){if(std::fabs(a-b)>1e-12)throw std::runtime_error("delay mismatch");}
int main(){
 wescore::CommandDynamics d; d.Configure(.125,0.0);
 d.Push(0,.5,.3);
 eq(d.Advance(.124).linear,0); eq(d.Advance(.125).linear,.5);
 eq(d.Advance(.125).angular,.3); // paused clock neither decays nor advances
 d.Push(.25,0,0); // one stop message, no heartbeat required
 eq(d.Advance(.374).linear,.5); eq(d.Advance(.375).linear,0);
 d.Push(.5,-.5,-.3); eq(d.Advance(.625).linear,-.5);
 eq(d.Advance(.625).angular,-.3);
 eq(d.Advance(.1).linear,0); // rewind invalidates queued/held motion
 d.Configure(0,0);d.Push(1,.2,-.1);eq(d.Advance(1).linear,.2);
 d.Reset();eq(d.Advance(2).linear,0);
 std::cout<<"6 pure-delay/stop/reverse/clock regression scenarios passed\n";
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp=Path(directory)/'test.cpp'; exe=Path(directory)/'test';cpp.write_text(code)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                            '-I'+str(PACKAGE/'include'),str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
