#!/usr/bin/env python3
"""Compile production command headers; does not claim ROS timer/physics acceptance."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

CPP = r'''
#include "scout_gazebo/command_delay.hpp"
#include "scout_gazebo/wheel_allocation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
void require(bool x) { if (!x) throw std::runtime_error("contract assertion"); }
void close(double a, double b) { require(std::abs(a-b) < 1e-10); }
int main() {
  wescore::WheelAllocation a; a.radius_m=.08; a.common_gain=1.04;
  a.allocation_span_m=.416503*.8;
  std::mt19937 gen(93); std::uniform_real_distribution<double> random(-1.,1.);
  for (int k=0;k<2000;++k) {
    const double v=random(gen), w=random(gen);
    const auto y=a.Apply(v,w), opposite=a.Apply(-v,-w);
    close(y[0],(v+w*.8*.416503*.5)/.08*1.04);
    close(y[1],(v-w*.8*.416503*.5)/.08*1.04);
    close(y[0],y[3]);close(y[1],y[2]);
    for(int j=0;j<4;++j)close(y[j],-opposite[j]);
    close(.08*(y[0]+y[1])/(2*1.04),v);
    close(.08*(y[0]-y[1])/(1.04*a.allocation_span_m),w);
  }
  a.allocation_span_m=.96;
  const auto pure=a.Apply(0,.5);require(pure[0]>0 && pure[1]<0);
  close(.08*(pure[0]-pure[1])/1.04,.48);
  bool rejected=false; a.radius_m=0;
  try{a.Apply(.5,0);}catch(const std::invalid_argument&){rejected=true;}require(rejected);
  a.radius_m=.08; a.allocation_span_m=-.4;rejected=false;
  try{a.Apply(.5,0);}catch(const std::invalid_argument&){rejected=true;}require(rejected);
  wescore::CommandDelay delay;delay.Configure(.005);
  delay.Push(0,.5,.3);close(delay.Advance(.004).linear,0);require(!delay.Sample().valid);
  close(delay.Advance(.008).linear,.5);close(delay.Sample().received_s,0);
  close(delay.Sample().due_s,.005);require(delay.Sample().sequence==1);
  close(delay.Advance(.008).angular,.3); // repeated clock holds, no inertia
  delay.Push(.020,0,0);close(delay.Advance(.024).linear,.5);
  close(delay.Advance(.028).linear,0); // a single stop executes without another input
  require(delay.Sample().sequence==2);close(delay.Sample().due_s,.025);
  delay.Push(.032,-.5,-.3);close(delay.Advance(.040).linear,-.5);
  close(delay.Advance(.010).linear,0);require(!delay.Sample().valid);
  delay.Push(.012,.2,0);close(delay.Advance(.020).linear,.2);
  require(delay.Sample().sequence==4); // provenance sequence survives a clock rewind
  delay.Reset();close(delay.Advance(.024).linear,0);
  std::cout<<"2000 allocation trials and delay/stop/reverse/repeated-clock tests passed\n";
}
'''

class ContractTest(unittest.TestCase):
    def test_actual_headers(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'test.cpp'; exe=Path(directory)/'test'
            source.write_text(CPP)
            subprocess.run([os.environ.get('CXX','g++'),'-std=c++17','-Wall','-Wextra','-Werror',
                            '-I'+str(ROOT/'include'),str(source),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_runtime_wiring(self):
        source=(ROOT/'src/scout_skid_steer.cpp').read_text()
        header=(ROOT/'include/scout_gazebo/scout_skid_steer.hpp').read_text()
        self.assertIn('nh_->createTimer(',source)
        self.assertNotIn('createWallTimer',source)
        self.assertIn('ros::Timer control_timer_',header)
        self.assertIn('command_delay_.Configure(command_delay_s_);',source)
        self.assertIn('command_delay_s_, 0.005)',source)
        self.assertIn('wheel_allocation_.Apply(command.linear, command.angular)',source)
        self.assertNotIn('time_constant',source)
        self.assertIn('command_dispatch_trace',source)

if __name__=='__main__': unittest.main()
