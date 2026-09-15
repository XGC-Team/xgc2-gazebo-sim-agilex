#!/usr/bin/env python3
"""Real Gazebo regression; authored but NOT run in the source-only environment."""
import math
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from gazebo_msgs.srv import GetModelState

class PhysicalBackend(unittest.TestCase):
    def setUp(self):
        self.rows=[];self.fault='';self.backend=''
        self.sub=rospy.Subscriber('/brush_test/simulation/implicit_wheels',Float64MultiArray,self.report)
        self.fs=rospy.Subscriber('/brush_test/simulation/dynamics_fault',String,lambda m:setattr(self,'fault',m.data))
        self.bs=rospy.Subscriber('/brush_test/simulation/wheel_physics_backend',String,lambda m:setattr(self,'backend',m.data))
        self.pub=rospy.Publisher('/brush_test/cmd_vel',Twist,queue_size=1)
        rospy.wait_for_service('/gazebo/get_model_state',timeout=30)
        self.state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.wait(lambda:len(self.rows)==4 and all(r[3]>1 for r in self.rows) and self.pub.get_num_connections()>0,30)
    def report(self,msg):
        if len(msg.data)==56:self.rows=[list(msg.data[i*14:(i+1)*14]) for i in range(4)]
    def wait(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if self.fault:self.fail(self.fault)
            if predicate():return
            time.sleep(.01)
        self.fail('timed out waiting for physical response')
    def command(self,v,w):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.pub.publish(msg)
    def test_loaded_turn_stop_and_force_limits(self):
        self.assertEqual(self.backend,'implicit_brush')
        self.command(.30,.25) # one message, not a hidden closed-loop body command
        self.wait(lambda:all(abs(r[1]-r[2])<.15 for r in self.rows) and abs(self.rows[0][1])>1,15)
        for row in self.rows:
            self.assertTrue(all(math.isfinite(x) for x in row))
            self.assertLessEqual(math.hypot(row[4],row[5]),.6*row[3]+1e-5)
            self.assertLessEqual(abs(row[6]),6.00001)
        result=self.state('brush_test','world');self.assertTrue(result.success)
        self.assertGreater(result.twist.angular.z,.01)
        self.assertGreater(math.hypot(result.twist.linear.x,result.twist.linear.y),.1)
        self.command(0,0) # normal Stop, not pose/velocity reset
        self.wait(lambda:all(abs(r[2])<.02 for r in self.rows),15)
        result=self.state('brush_test','world')
        self.assertLess(math.hypot(result.twist.linear.x,result.twist.linear.y),.03)
        self.assertLess(abs(result.twist.angular.z),.04)
    def tearDown(self):
        self.command(0,0)
        self.sub.unregister();self.fs.unregister();self.bs.unregister();self.pub.unregister()
if __name__=='__main__':
    rospy.init_node('implicit_wheel_regression')
    rostest.rosrun('gazebo_sim_scout','implicit_wheel_runtime',PhysicalBackend)
