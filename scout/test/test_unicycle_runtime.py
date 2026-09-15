#!/usr/bin/env python3
"""Real Gazebo smoke test; requires an isolated ROS/Gazebo test environment."""
import math
import socket
import struct
import threading
import time
import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Empty


class UnicycleRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.drive = None
        self.sub = rospy.Subscriber('/unicycle_test/simulation/drive/velocity', TwistStamped, self.on_drive)
        self.pub = rospy.Publisher('/unicycle_test/cmd_vel', Twist, queue_size=1)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        self.state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        mode = rospy.wait_for_message('/unicycle_test/simulation/dynamics_mode', String, timeout=30)
        self.assertEqual(mode.data, 'unicycle')
        deadline = time.monotonic() + 10
        while self.pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertGreater(self.pub.get_num_connections(), 0)
        rospy.sleep(.5)

    def on_drive(self, message):
        with self.lock:
            self.drive = (message.twist.linear.x, message.twist.angular.z)

    def command(self, v, w):
        message = Twist()
        message.linear.x, message.angular.z = v, w
        self.pub.publish(message)  # single message: Stop must not require heartbeat

    def measured(self):
        state = self.state('unicycle_test', 'world')
        self.assertTrue(state.success, state.status_message)
        q = state.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        v = math.cos(yaw)*state.twist.linear.x + math.sin(yaw)*state.twist.linear.y
        lateral = -math.sin(yaw)*state.twist.linear.x + math.cos(yaw)*state.twist.linear.y
        return v, state.twist.angular.z, lateral

    def hold(self, held):
        robot = b'unicycle_test'
        code = 2166136261
        for byte in robot:
            code = ((code ^ byte) * 16777619) & 0xffffffff
        port = 20000 + code % 20000
        packet = struct.pack('<IBBHI32s', 0x58474348, 1, int(held), 0, 17, robot)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(2)
            sock.sendto(packet, ('127.0.0.1', port))
            ack, _ = sock.recvfrom(64)
        self.assertEqual(len(ack), 12)
        self.assertEqual(struct.unpack_from('<I', ack)[0], 0x58474348)
        self.assertEqual(ack[5], int(held))
        self.assertEqual(ack[6], 0)
        self.assertEqual(struct.unpack_from('<I', ack, 8)[0], 17)

    def test_joint_input_stop_and_paused_hold(self):
        paused = False
        try:
            self.command(.4, .2)
            rospy.sleep(.3)
            v, w, lateral = self.measured()
            self.assertAlmostEqual(v, .4, delta=.02)
            self.assertAlmostEqual(w, .2, delta=.02)
            self.assertAlmostEqual(lateral, 0, delta=.02)
            self.command(0, 0)
            rospy.sleep(.3)
            v, w, _ = self.measured()
            self.assertAlmostEqual(v, 0, delta=.01)
            self.assertAlmostEqual(w, 0, delta=.01)
            self.command(.3, -.15)
            rospy.sleep(.2)
            self.pause(); paused = True
            self.hold(True)
            # Use wall time while /clock is frozen; verify drive-state clearing.
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                with self.lock:
                    cleared = self.drive == (0.0, 0.0)
                if cleared:
                    break
                time.sleep(.01)
            self.assertTrue(cleared)
            self.unpause(); paused = False
            rospy.sleep(.1)
            self.hold(False)
            rospy.sleep(.1)
            v, w, _ = self.measured()
            self.assertAlmostEqual(v, 0, delta=.01)
            self.assertAlmostEqual(w, 0, delta=.01)
        finally:
            if paused:
                self.unpause()
            self.command(0, 0)
            self.hold(False)


if __name__ == '__main__':
    rospy.init_node('scout_unicycle_runtime_test')
    rostest.rosrun('gazebo_sim_scout', 'scout_unicycle_runtime', UnicycleRuntimeTest)
