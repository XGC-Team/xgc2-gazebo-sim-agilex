#!/usr/bin/env python3
"""Real isolated ROS data + Gazebo model owners + world XRPC HOLD conformance."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
from xml.sax.saxutils import escape
from xgc2_xrpc.http import Client, Fault
from xgc2_xrpc.runtime import Runtime


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def asset(plugin, namespace, robot_id, *, mecanum=False, ideal=False):
    body = 'base_footprint' if mecanum else 'base_link'
    names = (['upper_left_wheel', 'upper_right_wheel', 'lower_left_wheel', 'lower_right_wheel']
             if mecanum else ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'])
    inertia = '<inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>'
    links = '<link name="'+body+'"><gravity>false</gravity><inertial><mass>10</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial></link>'
    for index, name in enumerate(names):
        link = name if mecanum else name+'_link'
        joint = name+'_joint' if mecanum else name
        links += f'<link name="{link}"><gravity>false</gravity><pose>{0.2 if index<2 else -0.2} {0.2 if index%2==0 else -0.2} 0 0 0 0</pose>{inertia}<collision name="tire"><geometry><cylinder><radius>0.08</radius><length>0.08</length></cylinder></geometry></collision></link>'
        links += f'<joint name="{joint}" type="revolute"><parent>{body}</parent><child>{link}</child><axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>6</effort><velocity>26</velocity></limit></axis></joint>'
    fields = ('<driveModel>'+('ideal' if ideal else 'high_fidelity')+'</driveModel>') if mecanum else '<wheelRadius>0.08</wheelRadius><wheelTrack>0.416503</wheelTrack>'
    return f'<sdf version="1.6"><model name="asset">{links}<plugin name="drive" filename="{plugin}"><robotNamespace>{namespace}</robotNamespace><chassisRobotId>{robot_id}</chassisRobotId>{fields}</plugin></model></sdf>'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plugins', required=True)
    parser.add_argument('--world-plugins', required=True)
    parser.add_argument('--gzserver', required=True)
    args = parser.parse_args()
    executable = Path(args.gzserver).resolve(strict=True)
    version_environment = dict(os.environ)
    for key in ['DISPLAY', 'WAYLAND_DISPLAY']:
        version_environment.pop(key, None)
    version_result = subprocess.run([str(executable), '--version'], env=version_environment,
                                    text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    version = version_result.stdout.strip()
    assert 'Gazebo multi-robot simulator, version 11.' in version, version
    rootfs = Path('/etc/os-release').read_text()
    with tempfile.TemporaryDirectory(prefix='sol6-models-native-') as directory:
        root = Path(directory)
        root.chmod(0o700)
        endpoint = root/'world.sock'
        environment = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{free_port()}',
                           ROS_HOME=str(root/'ros'), ROS_LOG_DIR=str(root/'logs'), HOME=str(root),
                           GAZEBO_MASTER_URI=f'http://127.0.0.1:{free_port()}', GAZEBO_MODEL_DATABASE_URI='',
                           GAZEBO_PLUGIN_PATH=args.plugins+':'+args.world_plugins+':/opt/ros/noetic/lib')
        for key in ['DISPLAY', 'WAYLAND_DISPLAY']:
            environment.pop(key, None)
        os.environ.update({key: environment[key] for key in ['ROS_MASTER_URI', 'ROS_HOME', 'ROS_LOG_DIR']})
        log = open(root/'native.log', 'w+')
        master = subprocess.Popen(['roscore', '-p', environment['ROS_MASTER_URI'].rsplit(':',1)[1]], env=environment,
                                  stdout=log, stderr=subprocess.STDOUT)
        print(json.dumps({'process': 'roscore', 'pid': master.pid, 'exe': shutil.which('roscore'),
                          'argv': master.args, 'display': None, 'wayland_display': None}), flush=True)
        process = None
        runtime = Runtime(blocking_workers=1, max_calls=8)
        discovery = Client(str(endpoint), runtime=runtime)
        client = None
        try:
            import rosgraph
            deadline = time.monotonic()+15
            while not rosgraph.is_master_online():
                if master.poll() is not None or time.monotonic()>deadline:
                    raise AssertionError('private ROS master startup failed')
                time.sleep(.025)
            roster = ['unicycle', 'skid', 'implicit', 'mecanum-ideal', 'mecanum-physical']
            world = root/'fixture.world'
            world.write_text(f'<sdf version="1.6"><world name="isolated"><gravity>0 0 -9.81</gravity><physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics><plugin name="native-world" filename="libxgc2_simulation_world.so"><socket_path>{escape(str(endpoint))}</socket_path><target_id>models-native</target_id><resource_root>{escape(str(root))}</resource_root>'+''.join('<chassis_robot_id>'+name+'</chassis_robot_id>' for name in roster)+'<required_component>dispatcher-test</required_component></plugin><plugin name="data-barrier" filename="libxgc2_dispatcher_fixture.so"/></world></sdf>')
            process = subprocess.Popen([str(executable), '--verbose', '--pause', '-s', 'libxgc2_simulation_ros_data.so', str(world)],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT)
            print(json.dumps({'process': 'gzserver', 'pid': process.pid, 'exe': str(executable),
                              'argv': process.args, 'version': version, 'version_exit_code': version_result.returncode, 'rootfs': rootfs,
                              'display': None, 'wayland_display': None, 'gzclient': False}), flush=True)
            deadline = time.monotonic()+20
            while not endpoint.exists():
                if process.poll() is not None or time.monotonic()>deadline:
                    raise AssertionError('native world startup failed')
                time.sleep(.025)
            description = json.loads(discovery.call('/v1/describe', method='GET').body)
            client = Client(str(endpoint), runtime=runtime, instance_id=description['service_ref']['instance_id'])
            import rospy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool
            rospy.init_node('isolated_chassis_data', anonymous=True, disable_signals=True, disable_rostime=True)
            joints = {}
            subscriptions = [rospy.Subscriber('/data_'+name.replace('-', '_')+('/simulation/drive/joints' if name=='mecanum-physical' else '/joint_states'), JointState,
                             lambda value, name=name: joints.__setitem__(name, value), queue_size=1) for name in roster]
            publishers = {name: rospy.Publisher('/data_'+name.replace('-', '_')+'/cmd_vel', Twist, queue_size=1) for name in roster}
            seq = 0
            def query(route):
                return json.loads(client.call(route, method='GET', timeout=5).body)
            def operation(route, value, method="POST"):
                nonlocal seq
                seq += 1
                accepted = json.loads(client.call(route, dict(value, operation_timeout_ms=5000), request_id=f'operation-{seq}', method=method, timeout=5).body)
                completed = json.loads(client.call('/v1/operations/'+accepted['id']+'/wait', {}, timeout=8).body)
                assert completed['state']=='succeeded', completed
                return completed['result']
            def hold(value):
                status = query('/v1/chassis/hold')
                result = json.loads(client.call('/v1/chassis/hold', {'expected_revision': status['revision'],
                    'changes': [{'robot_id': name, 'held': value} for name in roster]}, timeout=5).body)
                assert result['stage']=='applied', result
                if value: assert all(item['zero_applied'] for item in result['changes']), result
                return result
            plugins = ['libscout_unicycle_plugin.so', 'libscout_gazebo.so', 'libscout_implicit_wheel_plugin.so',
                       'libgazebo_sim_mecanum_contract.so', 'libgazebo_sim_mecanum_contract.so']
            threads = []
            for index, (name, plugin) in enumerate(zip(roster, plugins)):
                model = {'id': name, 'role': 'robot', 'pose': {'position': [index*5,0,0], 'orientation': [0,0,0,1]},
                         'asset': {'id': name, 'realization': {'media_type':'application/sdf+xml',
                           'content': asset(plugin, '/data_'+name.replace('-', '_'), name, mecanum=index>=3, ideal=index==3)}}}
                operation('/v1/entities', {'entity': model})
                deadline=time.monotonic()+5
                while publishers[name].get_num_connections()==0:
                    assert time.monotonic()<deadline, 'native command subscriber missing: '+name
                    time.sleep(.01)
                time.sleep(.05)
                threads.append(len(list(Path('/proc/'+str(process.pid)+'/task').iterdir())))
            # No additional per-model dispatcher/spinner threads after warm-up.
            assert max(threads[1:])-threads[1] <= 1, threads
            assert query('/v1/world')['paused'] is True
            blocker=rospy.Publisher('/fixture/block_dispatcher', Bool, queue_size=1)
            deadline=time.monotonic()+5
            while blocker.get_num_connections()==0:
                assert time.monotonic()<deadline
                time.sleep(.01)
            blocker.publish(Bool(data=True))
            deadline=time.monotonic()+5
            while not query('/v1/extensions/dispatcher-test/state')['blocked']:
                assert time.monotonic()<deadline, 'dispatcher barrier was not entered'
                time.sleep(.01)
            command=Twist(); command.linear.x=1.0
            for _ in range(5):
                for publisher in publishers.values(): publisher.publish(command)
                time.sleep(.02)
            held=hold(True)
            assert all(item['held'] for item in held['robots'])
            for _ in range(5):
                for publisher in publishers.values(): publisher.publish(command)
                time.sleep(.02)
            hold(False)
            operation('/v1/extensions/dispatcher-test/release', {})
            time.sleep(.1)
            operation('/v1/world/step', {'steps':100})
            time.sleep(.05)
            for name in ['unicycle','mecanum-ideal']:
                assert max(abs(value) for value in query('/v1/entities/'+name)['entities'][0]['state']['twist']['linear']) < 1e-8
            for name in ['skid','implicit','mecanum-physical']:
                assert name in joints, 'native data stream missing: '+name
                assert max(abs(value) for value in joints[name].velocity) < 1e-6, (name, joints[name])
            for _ in range(5):
                for publisher in publishers.values(): publisher.publish(command)
                time.sleep(.02)
            operation('/v1/world/step', {'steps':100})
            time.sleep(.05)
            assert query('/v1/entities/unicycle')['entities'][0]['state']['twist']['linear'][0] > .1
            assert query('/v1/entities/mecanum-ideal')['entities'][0]['state']['twist']['linear'][0] > .1
            assert max(abs(value) for value in joints['skid'].velocity) > .1
            before_hold=query('/v1/chassis/hold')
            for changes, revision in [([{'robot_id':'data_skid', 'held':True}], before_hold['revision']),
                                      ([{'robot_id':'skid', 'held':True}], before_hold['revision']-1)]:
                try:
                    rejected=client.call('/v1/chassis/hold', {'expected_revision':revision, 'changes':changes}, timeout=2)
                    assert rejected.status>=400, 'invalid identity/revision changed native control'
                except Fault:
                    pass
                assert query('/v1/chassis/hold')['revision']==before_hold['revision']
            hold(True) # real movement occurred; HOLD applies at frozen physics time
            for name in ['unicycle','mecanum-ideal']:
                assert max(abs(value) for value in query('/v1/entities/'+name)['entities'][0]['state']['twist']['linear']) < 1e-8
            stopping = threading.Event()
            def publish_during_delete():
                while not stopping.is_set():
                    for publisher in publishers.values(): publisher.publish(command)
                    stopping.wait(.002)
            producer = threading.Thread(target=publish_during_delete)
            producer.start()
            try:
                for name in roster:
                    native=query('/v1/entities/'+name)['entities'][0]
                    operation('/v1/entities/'+name, {'generation':native['ref']['generation']}, method='DELETE')
            finally:
                stopping.set(); producer.join(timeout=2)
            assert not producer.is_alive()
            assert process.poll() is None, 'native callback drain crashed the world'
            print(json.dumps({'semantic_checks_passed': True, 'models':roster, 'thread_counts_after_create':threads,
                              'paused_hold_zero_applied':True, 'held_commands_discarded':True, 'queued_before_hold_and_release_discarded':True, 'negative_identity_and_revision':True, 'native_motion_verified':True}))
        except Exception:
            log.flush(); log.seek(0); print(log.read(), flush=True)
            raise
        finally:
            cleanup_errors = []
            for resource in [client, discovery, runtime]:
                if resource is not None:
                    try: resource.close()
                    except Exception as error: cleanup_errors.append(str(error))
            for child, interrupt in [(process, True), (master, False)]:
                if child is None: continue
                if child.poll() is None:
                    if interrupt: child.send_signal(signal.SIGINT)
                    else: child.terminate()
                timed_out = False
                try: child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    timed_out = True; child.kill(); child.wait(timeout=5)
                    cleanup_errors.append('native drain timed out' if interrupt else 'ROS master stop timed out')
                print(json.dumps({'stopped_pid': child.pid, 'signal': 'SIGINT' if interrupt else 'SIGTERM',
                                  'exit_code': child.returncode, 'timeout_kill': timed_out}), flush=True)
            if process is not None:
                if endpoint.exists(): cleanup_errors.append('world socket survived native shutdown')
                if process.returncode != 0: cleanup_errors.append('gzserver native shutdown exit was not zero')
                print(json.dumps({'graceful_native_drain': not cleanup_errors,
                                  'socket_removed': not endpoint.exists()}), flush=True)
            if cleanup_errors:
                log.flush(); log.seek(0); print(log.read(), flush=True)
            log.close()
            if cleanup_errors: raise RuntimeError('; '.join(cleanup_errors))
if __name__=='__main__': main()
