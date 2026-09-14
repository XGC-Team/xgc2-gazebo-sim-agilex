#!/usr/bin/env python3
"""Compile the production allocation header, without ROS/Gazebo or field data."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]

CODE = r'''
#include "scout_gazebo/wheel_allocation.hpp"
#include <iostream>
#include <limits>
#include <random>
void check(bool v){if(!v)throw std::runtime_error("allocation assertion failed");}
void near(double a,double b){check(std::abs(a-b)<1e-11*(1+std::abs(b)));}
int main(){
 wescore::WheelAllocationConfig c;
 c.radius_m=.1;c.allocation_track_m=.4;c.common_gain=1.1;c.differential_gain=2;
 c.wheel_limit_rad_s=100;
 std::mt19937 rng(93);std::uniform_real_distribution<double> u(-1,1);
 for(int k=0;k<1000;++k){
   double v=u(rng),w=u(rng);
   auto t=wescore::AllocateWheelTargets(v,w,c);
   near(t.radians_per_second[0],1.1*(v+.4*w)/.1);
   near(t.radians_per_second[1],1.1*(v-.4*w)/.1);
   near(t.radians_per_second[0],t.radians_per_second[3]);
   near(t.radians_per_second[1],t.radians_per_second[2]);
   auto mode=wescore::WheelTreadModes(t,.1);
   near(mode[0],1.1*v);near(mode[1],1.1*.8*w);
   auto rev=wescore::AllocateWheelTargets(-v,-w,c);
   for(int j=0;j<4;++j)near(rev.radians_per_second[j],-t.radians_per_second[j]);
   auto mirror=wescore::AllocateWheelTargets(v,-w,c);
   near(mirror.radians_per_second[0],t.radians_per_second[1]);
 }
 c.wheel_limit_rad_s=3;
 auto clipped=wescore::AllocateWheelTargets(.4,.5,c);
 check(clipped.saturated);near(clipped.radians_per_second[0],3);
 near(clipped.radians_per_second[1],2.2);
 auto modes=wescore::WheelTreadModes(clipped,.1);
 near(modes[0],.26);near(modes[1],.08);
 auto zero=wescore::AllocateWheelTargets(0,0,c);
 check(!zero.saturated);for(double x:zero.radians_per_second)near(x,0);
 bool rejected=false;
 try{wescore::AllocateWheelTargets(std::numeric_limits<double>::quiet_NaN(),0,c);}
 catch(const std::invalid_argument&){rejected=true;}check(rejected);
 c.radius_m=0;rejected=false;
 try{wescore::AllocateWheelTargets(0,0,c);}
 catch(const std::invalid_argument&){rejected=true;}check(rejected);
 std::cout<<"1000 signed allocation cases and four saturation/invalid groups passed\n";
}
'''

class AllocationTest(unittest.TestCase):
    def test_production_header(self):
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            executable = Path(directory) / 'test'
            cpp.write_text(CODE)
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                            '-Wall', '-Wextra', '-Werror', '-I'+str(PACKAGE/'include'),
                            str(cpp), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)

if __name__ == '__main__':
    unittest.main()
