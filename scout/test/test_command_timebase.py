#!/usr/bin/env python3
"""Time-base wiring and pure delay checks; not a ROS callback integration test."""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
PACKAGE=Path(__file__).resolve().parents[1]
class ClockTest(unittest.TestCase):
    def test_runtime_timebase(self):
        s=(PACKAGE/'src/scout_skid_steer.cpp').read_text()
        self.assertIn('nh_->createTimer(',s)
        self.assertNotIn('createWallTimer(',s)
        self.assertIn('ros::Duration(0.01)',s)
        self.assertIn('command_delay_.Configure(command_delay_s_)',s)
        self.assertIn('AllocateWheelTargets(',s)
        self.assertNotIn('command_time_constant',s)
    def test_delay_on_simulated_sampling_grid(self):
        code=r"""
#include "scout_gazebo/command_delay.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void check(bool x){if(!x)throw std::runtime_error("sampled delay mismatch");}
int main(){
 for(double h:{.001,.002,.004}){
  double reference_application=-1;
  for(double rtf:{.25,.5,1.,2.}){
   wescore::CommandDelay q;q.Configure(.005);q.Advance(0);
   q.Push(.04,.5,.3);
   double next=.01,applied=-1;
   for(int k=40; k<101; ++k){
    const double t=k*.001;
    if(std::abs(t/h-std::round(t/h))>1e-9)continue;
    const double wall=t/rtf;(void)wall;
    if(t+1e-12<next)continue;
    while(next<=t+1e-12)next+=.01;
    const auto v=q.Advance(t);
    if(v.linear!=0 && applied<0)applied=t;
   }
   check(applied>=.045-1e-12 && applied<.045+.01+h+1e-12);
   if(reference_application<0)reference_application=applied;
   check(std::abs(applied-reference_application)<1e-12);
   q.Push(.12,0,0);check(q.Advance(.124).linear==.5);
   check(q.Advance(.14).linear==0);
   q.Push(.15,-.5,-.3);check(q.Advance(.16).linear==-.5);
   for(int k=0;k<10;++k)check(q.Advance(.16).linear==-.5);
   q.Reset();check(q.Advance(.16).linear==0);
  }
 }
 std::cout<<"3 physics grids x 4 wall-time factors: 5ms delay plus fixed ROS sampling; hold/stop checked\n";
}
"""
        with tempfile.TemporaryDirectory() as directory:
            cpp=Path(directory)/'test.cpp';exe=Path(directory)/'test'
            cpp.write_text(code)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(PACKAGE/'include'),str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__':unittest.main()
