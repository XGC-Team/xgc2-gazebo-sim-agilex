#!/usr/bin/env python3
"""Disposable Gazebo acceptance for paused multi-spawn and one-shot stop."""
import time
import math
import unittest
import rospy
import rostest
from native_simulation import NativeSimulation
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class ScoutRuntime(unittest.TestCase):
    def test_paused_spawn_and_single_zero(self):
        native=NativeSimulation()
        self.addCleanup(native.close)
        models={entity['ref']['id'] for entity in native.query('/v1/entities')['entities']}
        self.assertTrue({'scout1','scout2','mecanum1'}.issubset(models))
        self.assertTrue(native.query('/v1/world')['paused'])
        before=rospy.Time.now().to_sec()
        time.sleep(.5)
        self.assertEqual(before,rospy.Time.now().to_sec())
        native.resume()
        pub=rospy.Publisher('/scout1/cmd_vel',Twist,queue_size=1)
        targets=[]
        sub=rospy.Subscriber('/scout1/joint_states',JointState,lambda m:targets.append(max(abs(x) for x in m.velocity)))
        rospy.sleep(2)
        self.assertGreater(rospy.Time.now().to_sec(), before)
        self.assertGreater(pub.get_num_connections(),0)
        command=Twist(); command.linear.x=.6
        pub.publish(command)
        rospy.sleep(1.5)
        state=native.model_state
        self.assertGreater(abs(state('scout1','world').twist.linear.x),.2)
        self.assertTrue(targets and abs(targets[-1])>1)
        targets.clear()
        pub.publish(Twist()) # Exactly one zero input; no follow-up callbacks.
        rospy.sleep(2)
        self.assertGreater(len(targets),50)
        self.assertLess(abs(targets[-1]),1e-3)
        self.assertLess(abs(state('scout1','world').twist.linear.x),.04)
        # A sustained arc must not build up uncontrolled lateral momentum.
        command.angular.z=.4
        end=rospy.Time.now().to_sec()+10
        while rospy.Time.now().to_sec()<end:
            pub.publish(command)
            rospy.sleep(.05)
        turning=state('scout1','world')
        pub.publish(Twist())
        rospy.sleep(2)
        stopped=state('scout1','world').twist.linear
        q=turning.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        velocity=turning.twist.linear
        lateral=-math.sin(yaw)*velocity.x+math.cos(yaw)*velocity.y
        self.assertLess(abs(lateral),.2)
        self.assertGreater(turning.twist.angular.z,.15)
        self.assertLess(math.hypot(stopped.x,stopped.y),.04)
        command.angular.z=0
        def hold(robot, value, sequence):
            native.hold(robot, bool(value))
        for cycle in range(3):
            hold('scout2',True,10+cycle)
            pub.publish(command)
            rospy.sleep(1.2)
            self.assertGreater(abs(targets[-1]),1)
            hold('scout1',True,20+cycle)
            for _ in range(30):
                pub.publish(command)
                rospy.sleep(.05)
            self.assertLess(abs(targets[-1]),.04)
            self.assertLess(abs(state('scout1','world').twist.linear.x),.04)
            hold('scout1',False,30+cycle)
            rospy.sleep(.25)
            self.assertLess(abs(targets[-1]),.04)
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('scout_runtime')
    rostest.rosrun('gazebo_sim_scout','scout_runtime',ScoutRuntime)
