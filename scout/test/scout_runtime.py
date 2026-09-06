#!/usr/bin/env python3
"""Disposable Gazebo acceptance for paused multi-spawn and one-shot stop."""
import time
import socket
import struct
import unittest
import rospy
import rostest
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetPhysicsProperties
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty

class ScoutRuntime(unittest.TestCase):
    def test_paused_spawn_and_single_zero(self):
        rospy.wait_for_service('/gazebo/get_world_properties', timeout=30)
        world=rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        physics=rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            if {'scout1','scout2','mecanum1'}.issubset(world().model_names): break
            time.sleep(.1)
        self.assertTrue({'scout1','scout2','mecanum1'}.issubset(world().model_names))
        self.assertTrue(physics().pause)
        before=rospy.Time.now().to_sec()
        time.sleep(.5)
        self.assertEqual(before,rospy.Time.now().to_sec())
        rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        pub=rospy.Publisher('/scout1/cmd_vel',Twist,queue_size=1)
        targets=[]
        sub=rospy.Subscriber('/scout1/scout_motor_fr_controller/command',Float64,lambda m:targets.append(m.data))
        rospy.sleep(2)
        self.assertGreater(rospy.Time.now().to_sec(), before)
        self.assertGreater(pub.get_num_connections(),0)
        command=Twist(); command.linear.x=.6
        pub.publish(command)
        rospy.sleep(1.5)
        state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.assertGreater(abs(state('scout1','world').twist.linear.x),.2)
        self.assertTrue(targets and abs(targets[-1])>1)
        targets.clear()
        pub.publish(Twist()) # Exactly one zero input; no follow-up callbacks.
        rospy.sleep(2)
        self.assertGreater(len(targets),50)
        self.assertLess(abs(targets[-1]),1e-3)
        self.assertLess(abs(state('scout1','world').twist.linear.x),.04)
        def hold(robot, value, sequence):
            digest=2166136261
            for byte in robot.encode(): digest=((digest ^ byte)*16777619)&0xffffffff
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as client:
                client.settimeout(2)
                client.sendto(struct.pack('<IBBHI32s',0x58474348,1,value,0,sequence,robot.encode()),('127.0.0.1',20000+digest%20000))
                ack,_=client.recvfrom(1024)
            self.assertEqual(ack,struct.pack('<IBBBBI',0x58474348,1,value,0,0,sequence))
        for cycle in range(3):
            hold('scout2',True,10+cycle)
            pub.publish(command)
            rospy.sleep(1.2)
            self.assertGreater(abs(targets[-1]),1)
            hold('scout1',True,20+cycle)
            for _ in range(30):
                pub.publish(command)
                rospy.sleep(.05)
            self.assertEqual(targets[-1],0)
            self.assertLess(abs(state('scout1','world').twist.linear.x),.04)
            hold('scout1',False,30+cycle)
            rospy.sleep(.25)
            self.assertEqual(targets[-1],0)
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('scout_runtime')
    rostest.rosrun('gazebo_sim_scout','scout_runtime',ScoutRuntime)
