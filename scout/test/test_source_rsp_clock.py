#!/usr/bin/env python3
"""Run the product source-RSP launch against isolated clocks and upstream RSP.

No Gazebo, live master, GUI, or custom RSP implementation is required. Each
case runs in a fresh process because rospy's clock is process-global.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

PACKAGE = Path(__file__).resolve().parents[1]


def wait_until(predicate, message, timeout=5):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError(message)
        time.sleep(.02)


def run_case(mode, namespace, temp):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    assert port not in (11311, 11312)
    master_uri = 'http://127.0.0.1:%d' % port
    os.environ.update(ROS_MASTER_URI=master_uri, ROS_IP='127.0.0.1',
                      ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=temp)
    os.environ['ROS_PACKAGE_PATH'] = str(PACKAGE.parent) + ':' + os.environ.get('ROS_PACKAGE_PATH', '/opt/ros/noetic/share')
    import roslaunch
    import roslaunch.node_args
    import rosmaster.master
    import rospy
    import tf2_py
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(PACKAGE / 'launch/spawn_accurate.launch'), config,
        argv=['ns:=audit_robot', 'robot_state_publisher_ns:=' + namespace,
              'robot_description_param:=' + namespace.rstrip('/') + '/robot_description',
              'tf_prefix:=audit_robot', 'frame_prefix:=audit_robot/', 'run_mode:=' + mode], verbose=False)
    # Resolve the complete product launch and real Scout URDF, but only execute
    # its RSP. No Gazebo or chassis controller is started by this test.
    nodes = [node for node in config.nodes if node.package == 'robot_state_publisher']
    assert len(nodes) == 1
    node = nodes[0]
    master = rosmaster.master.Master(port)
    process = None
    master.start()
    try:
        api = xmlrpc.client.ServerProxy(master_uri)
        api.setParam('/test_setup', '/use_sim_time', mode == 'simulation')
        for param in config.params.values():
            api.setParam('/test_setup', param.key, param.value)
        rospy.init_node('source_rsp_clock_test', disable_signals=True)
        received = []
        statics = []
        buffer = tf2_py.BufferCore(rospy.Duration(10))

        def dynamic(msg):
            for tr in msg.transforms:
                buffer.set_transform(tr, 'upstream_rsp')
                received.append(tr.header.stamp.to_sec())

        def fixed(msg):
            for tr in msg.transforms:
                buffer.set_transform_static(tr, 'upstream_rsp')
                statics.append(tr.child_frame_id)

        subscriptions = [rospy.Subscriber('/tf', TFMessage, dynamic),
                         rospy.Subscriber('/tf_static', TFMessage, fixed)]
        joints = rospy.Publisher('/audit_robot/joint_states', JointState, queue_size=10)
        clock_topic = '/clock' if mode == 'simulation' else '/xgc/source/gazebo/clock'
        clock = rospy.Publisher(clock_topic, Clock, queue_size=1, latch=True)
        # A wrong hybrid /clock remap must fail even if a foreign clock exists.
        decoy = rospy.Publisher('/clock', Clock, queue_size=1, latch=True) if mode == 'hybrid' else None
        if decoy is not None:
            decoy.publish(Clock(clock=rospy.Time.from_sec(5000)))
        clock.publish(Clock(clock=rospy.Time.from_sec(100)))
        logfile = Path(temp) / 'rsp.log'
        with logfile.open('w') as stream:
            machine = roslaunch.core.local_machine()
            args = roslaunch.node_args.create_local_process_args(node, machine)
            env = roslaunch.core.setup_env(node, machine, master_uri)
            process = subprocess.Popen(args, env=env, stdout=stream, stderr=subprocess.STDOUT)
            wait_until(lambda: joints.get_num_connections() == 1 and clock.get_num_connections() >= 1,
                       'product RSP did not subscribe to joints/source clock')
            wait_until(lambda: statics, 'sensor static TF disappeared')

            def publish_sample(stamp, position=0.):
                msg = JointState()
                msg.header.stamp = rospy.Time.from_sec(stamp)
                msg.name, msg.position = ['front_left_wheel'], [position]
                joints.publish(msg)

            sent = [100 + i * .05 for i in range(25)]
            expected_stamps = [rospy.Time.from_sec(stamp).to_sec() for stamp in sent]
            for i, stamp in enumerate(sent):
                clock.publish(Clock(clock=rospy.Time.from_sec(stamp)))
                time.sleep(.025)
                publish_sample(stamp, i * .01)
                time.sleep(.025)
            wait_until(lambda: len(received) == len(sent), 'source TF samples missing')
            assert received == expected_stamps, (received, expected_stamps)
            assert 'Received JointState is' not in logfile.read_text(), logfile.read_text()
            midpoint = (sent[0] + sent[-1]) / 2
            buffer.lookup_transform_core('audit_robot/base_link', 'audit_robot/front_left_wheel_link', rospy.Time.from_sec(midpoint))
            # Static sensor relations remain usable in the wall domain as well.
            sensor_frames = ['box_link', 'camera_link', 'imu_link', 'laser_link', 'rslidar']
            for frame in sensor_frames:
                buffer.lookup_transform_core('audit_robot/base_link', 'audit_robot/' + frame, rospy.Time.from_sec(time.time()))

            # Pause: repeated input at the frozen timestamp cannot advance TF.
            for _ in range(5):
                publish_sample(sent[-1])
                time.sleep(.05)
            assert received == expected_stamps, received
            assert 'Received JointState is' not in logfile.read_text()

            # Advance source time while retaining a stale sample: the upstream
            # freshness warning must still work (we did not disable it).
            clock.publish(Clock(clock=rospy.Time.from_sec(150)))
            time.sleep(.1)
            publish_sample(sent[-1])
            wait_until(lambda: 'Received JointState is' in logfile.read_text(), 'source age check was disabled or uses the wrong clock')
            publish_sample(150, .3)
            wait_until(lambda: received[-1] == 150, 'TF did not recover after source clock resumed')

            state = api.getSystemState('/test_setup')[2]
            node_name = node.namespace + node.name
            subscribed = sorted(topic for topic, nodes in state[1] if node_name in nodes)
            assert clock_topic in subscribed, subscribed
            if mode == 'hybrid':
                assert '/clock' not in subscribed, subscribed
            assert api.getParam('/test_setup', '/use_sim_time')[2] == (mode == 'simulation')
            assert api.getParam('/test_setup', node_name + '/use_sim_time')[2] is True
            print(json.dumps(dict(mode=mode, namespace=namespace, isolated_master_port=port,
                                  subscriptions=subscribed, fresh_tf_count=len(sent),
                                  fresh_warning=False, source_stamps_preserved=True,
                                  fixed_sensor_queries=sensor_frames, pause_no_advance=True,
                                  stale_source_warning=True, resume_tf=True)), flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        rospy.signal_shutdown('source RSP test completed')
        master.stop()


if __name__ == '__main__':
    if len(sys.argv) == 3:
        with tempfile.TemporaryDirectory(prefix='xgc-source-rsp-test-') as temp:
            run_case(sys.argv[1], sys.argv[2], temp)
    else:
        for mode in ('simulation', 'hybrid'):
            for namespace in ('/', '/audit_robot'):
                subprocess.run([sys.executable, __file__, mode, namespace], check=True, timeout=20)
