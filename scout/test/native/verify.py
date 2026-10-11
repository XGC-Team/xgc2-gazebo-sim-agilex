#!/usr/bin/env python3
"""Chassis HOLD on a real headless gzserver.

The world plugin (one HOLD host per world), the Scout skid/unicycle/implicit-wheel plugins and the Mecanum
ideal/high-fidelity plugin run in one isolated world with a private ROS master. The test talks to the world
host over its http.v1 Unix socket (POST /v1/call/xgc2.chassis.hold/<Method>) and drives the robots with ROS cmd_vel.

    verify.py --plugins <lib dirs> --mecanum-source <gz-mecanum checkout> [--gzserver /usr/bin/gzserver]

--plugins lists the directories that hold libxgc2_simulation_*.so, the Scout and Mecanum plugins and the
libxgc2_dispatcher_fixture.so built from this directory (see CMakeLists.txt).
"""
import argparse
import http.client
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
from xml.etree import ElementTree
from xml.sax.saxutils import escape

DT = 0.001
CRUISE = 0.3  # m/s, commanded to every robot
HOLD_SERVICE = 'xgc2.chassis.hold'
HOLD_CALL = '/v1/call/%s/' % HOLD_SERVICE
# Gazebo Classic 11 sometimes aborts in the shutdown of its own transport library (a connection is destroyed on
# its IO thread: "boost thread: trying joining itself"; every frame is inside libgazebo_transport).
GAZEBO_TRANSPORT_ABORT = 'boost thread: trying joining itself'


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout):
        super().__init__('localhost', timeout=timeout)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


class Fault(Exception):
    def __init__(self, status, body):
        super().__init__('HTTP %d: %s' % (status, body))
        self.status, self.body = status, body


class World:
    """The http.v1 client of one world host, pinned to an instance."""

    def __init__(self, path):
        self.path = str(path)
        self.sequence = 0
        self.instance = None

    def raw(self, method, target, body=None, *, instance=True, timeout=5.0):
        self.sequence += 1
        headers = {'X-Request-ID': 'hold-test-%d' % self.sequence, 'X-Xrpc-Timeout-Ms': str(int(timeout * 1000))}
        if instance and self.instance:
            headers['X-Xrpc-Instance-ID'] = self.instance if instance is True else instance
        payload = None
        if body is not None:
            payload = json.dumps(body)
            headers['Content-Type'] = 'application/json'
        connection = UnixConnection(self.path, timeout + 2)
        try:
            connection.request(method, target, body=payload, headers=headers)
            response = connection.getresponse()
            data = response.read()
        finally:
            connection.close()
        return response.status, json.loads(data) if data else None

    def call(self, method, target, body=None, **options):
        status, value = self.raw(method, target, body, **options)
        if status >= 400:
            raise Fault(status, value)
        return value

    def get(self, target, **options):
        return self.call('GET', target, **options)

    def operation(self, method, target, body=None):
        body = dict(body or {}, operation_timeout_ms=5000)
        accepted = self.call(method, target, body, timeout=10)
        done = self.call('POST', '/v1/operations/%s/wait' % accepted['id'], {}, timeout=15)
        if done['state'] != 'succeeded':
            raise Fault(0, done)
        return done['result']

    # The HOLD capability: methods of the generic http.v1 addressing, POST /v1/call/<service>/<Method>.
    def hold_raw(self, method, body=None, **options):
        return self.raw('POST', HOLD_CALL + method, {} if body is None else body, **options)

    def hold(self, method, body=None, **options):
        return self.call('POST', HOLD_CALL + method, {} if body is None else body, **options)

    def describe(self):
        return self.hold('Describe')

    def state(self, ids=None):
        return self.hold('State', {} if ids is None else {'robot_ids': list(ids)})

    def robot(self, rid):
        return self.state([rid])['robots'][0]

    def engage(self, ids):
        return self.hold('Engage', {'robot_ids': list(ids)})

    def release(self, changes, instance=None):
        return self.hold('Release', {
            'expected_instance': instance or self.instance,
            'changes': [{'robot_id': rid, 'expected_revision': revision} for rid, revision in changes]})

    def hold_entities(self):
        """The entities of the HOLD capability in the describe facts of the world."""
        capabilities = self.get('/v1/describe')['facts']['capabilities']
        return next(c['entities'] for c in capabilities if c['name'] == HOLD_SERVICE)

    # entities
    def entity(self, name):
        return self.get('/v1/entities/' + name)['entities'][0]

    def speed(self, name):
        twist = self.entity(name)['state']['twist']
        return math.hypot(twist['linear'][0], twist['linear'][1])

    def position(self, name):
        return self.entity(name)['state']['pose']['position']


# ---------------------------------------------------------------------------------------------------------
# Assets. Scout skid and implicit-wheel drive a four-wheeled body on the ground plane; the Scout unicycle and
# the ideal Mecanum are kinematic and float. The Mecanum assets are the model of the gz-mecanum checkout.

WHEEL_RADIUS = 0.08


def inertia_box(mass, x, y, z):
    return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)


def inertial(mass, inertia):
    return ('<inertial><mass>%g</mass><inertia><ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz></inertia></inertial>'
            % ((mass,) + tuple(inertia)))


def scout_sdf(plugin, namespace, rid, fields=''):
    """Four wheels on revolute joints named like the Scout's, wheel links named <joint>_link."""
    track, base = 0.4165, 0.45
    wheels = [('front_left_wheel', base / 2, track / 2), ('front_right_wheel', base / 2, -track / 2),
              ('rear_left_wheel', -base / 2, track / 2), ('rear_right_wheel', -base / 2, -track / 2)]
    body = '<link name="base_link">%s</link>' % inertial(20, inertia_box(20, 0.6, 0.4, 0.2))
    for name, x, y in wheels:
        radius, width, mass = WHEEL_RADIUS, 0.06, 1.5
        axial = mass * radius * radius / 2
        radial = mass * (3 * radius * radius + width * width) / 12
        body += (
            '<link name="%(n)s_link"><pose>%(x)g %(y)g 0 0 0 0</pose>%(inertial)s'
            '<collision name="tire"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>%(r)g</radius>'
            '<length>%(w)g</length></cylinder></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode>'
            '</friction><contact><ode><kp>1000000</kp><kd>1</kd><min_depth>0.001</min_depth></ode></contact>'
            '</surface></collision></link>'
            '<joint name="%(n)s" type="revolute"><parent>base_link</parent><child>%(n)s_link</child><axis>'
            '<xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>6</effort><velocity>26</velocity>'
            '</limit><dynamics><damping>0.1</damping></dynamics></axis></joint>'
            % dict(n=name, x=x, y=y, r=radius, w=width, inertial=inertial(mass, (radial, axial, radial))))
    return ('<sdf version="1.6"><model name="asset">%s<plugin name="drive" filename="%s"><robotNamespace>%s'
            '</robotNamespace><chassisRobotId>%s</chassisRobotId>%s</plugin></model></sdf>'
            % (body, plugin, namespace, rid, fields))


def unicycle_sdf(namespace, rid):
    return ('<sdf version="1.6"><model name="asset"><link name="base_link"><gravity>false</gravity>%s</link>'
            '<plugin name="drive" filename="libscout_unicycle_plugin.so"><robotNamespace>%s</robotNamespace>'
            '<chassisRobotId>%s</chassisRobotId><wheelRadius>0.08</wheelRadius><wheelTrack>0.416503</wheelTrack>'
            '</plugin></model></sdf>' % (inertial(20, inertia_box(20, 0.6, 0.4, 0.2)), namespace, rid))


def mecanum_sdf(source, namespace, rid, mode):
    """The model of the gz-mecanum checkout, without its visuals (the server has no renderer)."""
    template = Path(source) / 'models/xgc2_mecanum_ugv/model.sdf.xacro'
    expanded = subprocess.run(
        ['xacro', str(template), 'chassis_robot_id:=' + rid, 'robot_namespace:=' + namespace,
         'drive_model:=' + mode, 'mesh_prefix:=model://unused'],
        check=True, stdout=subprocess.PIPE, text=True).stdout
    root = ElementTree.fromstring(expanded)
    for link in root.iter('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    return ElementTree.tostring(root, encoding='unicode')


class Robot:
    """A chassis model of the test. `kinematic` plugins write the velocity of the model, the others drive wheels.

    The implicit wheel plugin takes the normal load of its tires from the contact wrench of Gazebo, which this
    Gazebo reports in the frame of the rotating wheel link: the load, and with it the grip of the tires, varies
    with the angle of the wheels and a vehicle that is commanded to stop may keep sliding. Its rest is therefore
    read from the speed of its wheels (`wheels`), which the plugin controls, not from the motion of the body."""

    def __init__(self, name, kind, sdf, height, kinematic, wheels=False):
        self.name, self.kind, self.sdf, self.height = name, kind, sdf, height
        self.kinematic, self.wheels = kinematic, wheels
        self.namespace = '/hold_' + name.replace('-', '_')
        self.generation = None
        self.moving_above = 1.0 if wheels else 0.1  # rad/s of the wheels, m/s of the body
        self.rest_below = 0.1 if wheels else 0.02


def robots(mecanum_source):
    def build(name, kind, factory, height, kinematic, wheels=False):
        ns = '/hold_' + name.replace('-', '_')
        return Robot(name, kind, factory(ns, name), height, kinematic, wheels)
    return [
        build('scout-unicycle', 'scout unicycle', unicycle_sdf, 0.1, True),
        build('scout-skid', 'scout skid steer',
              lambda ns, rid: scout_sdf('libscout_gazebo.so', ns, rid), WHEEL_RADIUS + 0.002, False),
        build('scout-implicit', 'scout implicit wheel',
              lambda ns, rid: scout_sdf('libscout_implicit_wheel_plugin.so', ns, rid), WHEEL_RADIUS + 0.002, False,
              wheels=True),
        build('mecanum-ideal', 'mecanum ideal',
              lambda ns, rid: mecanum_sdf(mecanum_source, ns, rid, 'ideal'), 0.1, True),
        build('mecanum-hifi', 'mecanum high fidelity',
              lambda ns, rid: mecanum_sdf(mecanum_source, ns, rid, 'high_fidelity'), 0.052, False),
    ]


# ---------------------------------------------------------------------------------------------------------
# Processes

class Stack:
    """A private ROS master and, on demand, a gzserver with the isolated test world."""

    def __init__(self, root, plugin_dirs, gzserver):
        self.root = Path(root)
        self.plugin_dirs = [str(Path(p).resolve()) for p in plugin_dirs]
        self.gzserver = str(Path(gzserver).resolve(strict=True))
        self.socket = self.root / 'world.sock'
        self.ros_port = free_port()
        self.gazebo_port = free_port()
        self.master = None
        self.process = None
        self.log = None
        self.runs = 0
        self.environment = dict(
            os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % self.ros_port, ROS_HOME=str(self.root / 'ros'),
            ROS_LOG_DIR=str(self.root / 'ros-logs'), HOME=str(self.root),
            GAZEBO_MASTER_URI='http://127.0.0.1:%d' % self.gazebo_port, GAZEBO_MODEL_DATABASE_URI='',
            GAZEBO_PLUGIN_PATH=':'.join(self.plugin_dirs + [os.environ.get('GAZEBO_PLUGIN_PATH', '')]))
        for key in ('DISPLAY', 'WAYLAND_DISPLAY'):
            self.environment.pop(key, None)
        self.world_file = self.root / 'hold.world'
        self.world_file.write_text(
            '<sdf version="1.6"><world name="hold">'
            '<gravity>0 0 -9.81</gravity>'
            '<physics type="ode"><max_step_size>%g</max_step_size><real_time_update_rate>1000</real_time_update_rate>'
            '</physics>'
            '<model name="ground"><static>true</static><link name="surface"><collision name="plane"><geometry>'
            '<plane><normal>0 0 1</normal><size>400 400</size></plane></geometry><surface><friction><ode>'
            '<mu>1</mu><mu2>1</mu2></ode></friction></surface></collision></link></model>'
            '<plugin name="native-world" filename="libxgc2_simulation_world.so"><socket_path>%s</socket_path>'
            '<target_id>hold-test</target_id><resource_root>%s</resource_root>'
            '<required_component>dispatcher-test</required_component></plugin>'
            '<plugin name="dispatcher-barrier" filename="libxgc2_dispatcher_fixture.so"/></world></sdf>'
            % (DT, escape(str(self.socket)), escape(str(self.root))))

    def start_master(self):
        self.log = open(self.root / 'processes.log', 'w+')
        self.master = subprocess.Popen(['roscore', '-p', str(self.ros_port)], env=self.environment,
                                       stdout=self.log, stderr=subprocess.STDOUT)
        import rosgraph
        deadline = time.monotonic() + 20
        os.environ.update(ROS_MASTER_URI=self.environment['ROS_MASTER_URI'], ROS_HOME=self.environment['ROS_HOME'],
                          ROS_LOG_DIR=self.environment['ROS_LOG_DIR'])
        while not rosgraph.is_master_online():
            if self.master.poll() is not None or time.monotonic() > deadline:
                raise AssertionError('the private ROS master did not start')
            time.sleep(0.05)

    def start_gazebo(self):
        """Starts gzserver and returns once the world host answers; returns the world client."""
        self.runs += 1
        out = open(self.root / ('gzserver-%d.log' % self.runs), 'w+')
        command = [self.gzserver, '--verbose', '-s', 'libxgc2_simulation_ros_data.so', str(self.world_file)]
        self.process = subprocess.Popen(command, env=self.environment, stdout=out, stderr=subprocess.STDOUT)
        self.process.log = out
        client = World(self.socket)
        deadline = time.monotonic() + 40
        while True:
            if self.process.poll() is not None:
                raise AssertionError('gzserver exited early (%s)' % self.process.returncode)
            try:
                description = client.get('/v1/describe')
                break
            except (OSError, Fault, http.client.HTTPException):
                if time.monotonic() > deadline:
                    raise AssertionError('the world host did not answer')
                time.sleep(0.05)
        client.instance = description['service_ref']['instance_id']
        client.description = description
        while client.get('/v1/health')['lifecycle'] != 'ready':
            if time.monotonic() > deadline:
                raise AssertionError('the world did not become ready')
            time.sleep(0.05)
        return client

    def log_text(self):
        return (self.root / ('gzserver-%d.log' % self.runs)).read_text(errors='replace')

    def stop_gazebo(self):
        """SIGINT and wait; returns the exit status."""
        process, self.process = self.process, None
        if process is None:
            return None
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            raise AssertionError('gzserver did not exit after SIGINT')
        return process.returncode

    def close(self):
        errors = []
        if self.process is not None and self.process.poll() is not None:
            errors.append('gzserver died with status %s' % self.process.returncode)
        if self.process is not None and self.process.poll() is None:
            self.process.kill()
            self.process.wait()
            errors.append('gzserver had to be killed')
        if self.master is not None and self.master.poll() is None:
            self.master.terminate()
            try:
                self.master.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.master.kill()
                self.master.wait()
        return errors


# ---------------------------------------------------------------------------------------------------------
# The test

def wait_until(predicate, timeout, message, interval=0.05):
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value:
            return value
        if time.monotonic() > deadline:
            raise AssertionError('timeout: ' + (message() if callable(message) else message))
        time.sleep(interval)


class Fleet:
    """Streams cmd_vel to the robots that are switched on, like a controller would."""

    def __init__(self, bots, rospy, twist):
        self.rospy, self.twist = rospy, twist
        self.publishers = {b.name: rospy.Publisher(b.namespace + '/cmd_vel', twist, queue_size=1) for b in bots}
        self.commands = {b.name: self.command(CRUISE) for b in bots}
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def command(self, speed):
        message = self.twist()
        message.linear.x = speed
        return message

    def run(self):
        while not self.stopping.is_set():
            for name, message in list(self.commands.items()):
                if message is not None:
                    self.publishers[name].publish(message)
            self.stopping.wait(0.05)

    def drive(self, name, speed=CRUISE):
        self.commands[name] = self.command(speed)

    def silence(self, name):
        self.commands[name] = None

    def send(self, name, speed=CRUISE, count=1):
        for _ in range(count):
            self.publishers[name].publish(self.command(speed))
            time.sleep(0.02)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=5)


class WheelProbe:
    """The wheel speeds a model plugin publishes on joint_states, as the largest |rad/s|."""

    def __init__(self, bot, rospy, joint_state):
        self.stamp, self.speed = 0.0, 0.0
        self.subscriber = rospy.Subscriber(bot.namespace + '/joint_states', joint_state, self.on_message, queue_size=1)

    def on_message(self, message):
        self.speed = max((abs(v) for v in message.velocity), default=0.0)
        self.stamp = time.monotonic()

    def read(self):
        wait_until(lambda: time.monotonic() - self.stamp < 0.5, 5, 'the model publishes its joint states')
        return self.speed


class Run:
    def __init__(self, stack, world, bots, fleet, rospy):
        self.stack, self.world, self.bots, self.fleet, self.rospy = stack, world, bots, fleet, rospy
        self.checks = []
        self.probes = {}

    def ok(self, name):
        self.checks.append(name)
        print('ok  ' + name, flush=True)

    def entity(self, bot):
        return {'id': bot.name, 'role': 'robot',
                'pose': {'position': [0, 2.0 * self.bots.index(bot), bot.height], 'orientation': [0, 0, 0, 1]},
                'asset': {'id': bot.name, 'realization': {'media_type': 'application/sdf+xml', 'content': bot.sdf}}}

    def create(self, bot):
        result = self.world.operation('POST', '/v1/entities', {'entity': self.entity(bot)})
        bot.generation = result['entities'][0]['ref']['generation']

    def remove(self, bot):
        self.world.operation('DELETE', '/v1/entities/' + bot.name, {'generation': bot.generation})

    def activity(self, bot):
        """How much the robot moves: the speed of its body in m/s, the speed of its wheels for a wheel robot."""
        return self.probes[bot.name].read() if bot.wheels else self.world.speed(bot.name)

    def moving(self, bot):
        return self.activity(bot) > bot.moving_above

    def resting(self, bot):
        return self.activity(bot) < bot.rest_below

    def at_rest(self, bot, duration=1.0):
        """The robot does not move over `duration`, although commands keep arriving."""
        if bot.wheels:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                assert self.activity(bot) < bot.rest_below, (bot.name, self.activity(bot))
                time.sleep(0.05)
            return
        before = self.world.position(bot.name)
        time.sleep(duration)
        after = self.world.position(bot.name)
        assert math.hypot(after[0] - before[0], after[1] - before[1]) < 0.005 * duration, (bot.name, before, after)
        assert self.world.speed(bot.name) < bot.rest_below, (bot.name, self.world.speed(bot.name))

    def settled(self, bot, why):
        """Waits until the robot is at rest and stays so: a robot that was braking is not rest yet."""
        wait_until(lambda: self.resting(bot), 20, lambda: '%s rests (%s; state %s, activity %.3f)' % (
            bot.name, why, self.world.robot(bot.name), self.activity(bot)))
        time.sleep(0.5)
        self.at_rest(bot, 0.5)

    def others_move(self, bot):
        for other in self.bots:
            if other is not bot:
                wait_until(lambda: self.moving(other), 2, other.name + ' stopped with ' + bot.name)

    def stages(self, bot, final, timeout=20):
        """Follows the stages of an engaged robot until `final`; returns the state."""
        seen = []

        def reached():
            robot = self.world.robot(bot.name)
            if not seen or seen[-1] != robot['stage']:
                seen.append(robot['stage'])
            return robot if robot['stage'] == final else None

        robot = wait_until(reached, timeout, '%s reaches %s (stages %s)' % (bot.name, final, seen))
        order = ['gated', 'zero_written', 'stopped']
        assert seen == order[order.index(seen[0]):order.index(final) + 1], (bot.name, seen)
        return robot


def surface(run):
    world, names = run.world, sorted(b.name for b in run.bots)
    description = world.describe()
    assert description['instance'] == world.instance, 'HOLD and the transport share one instance'
    assert description['robots'] == names, description
    assert description['feedback'] == {name: True for name in names}
    assert description['capabilities'] == ['engage', 'release', 'state']
    # The capability is listed in the describe facts with the bound entities, and nowhere else.
    facts = world.get('/v1/describe')['facts']
    assert facts['capabilities'] == [{'name': HOLD_SERVICE, 'entities': names}], facts
    assert not [c for c in world.description['capabilities'] if 'hold' in c], 'no string capability for HOLD'
    for target in ('/v1/chassis/descriptor', '/v1/chassis/hold', '/v1/runtime-policy'):
        assert world.raw('GET', target)[0] == 404, 'retired route ' + target
    for robot in world.state()['robots']:
        assert (robot['held'], robot['revision'], robot['stage']) == (False, 0, 'released'), robot
    # The method addressing: POST /v1/call/<service>/<Method>; closed request schemas.
    assert world.raw('GET', HOLD_CALL + 'Describe')[0] == 404, 'a call is a POST'
    assert world.raw('POST', HOLD_CALL + 'Missing', {})[0] == 404
    assert world.raw('POST', '/v1/call/other.service/Describe', {})[0] == 404
    assert world.hold_raw('Describe', {'extra': 1})[0] == 400
    assert world.hold_raw('Engage', {})[0] == 400
    assert world.hold_raw('Engage', {'robot_ids': ['a'], 'all': True})[0] == 400
    assert world.hold_raw('Engage', {'robot_ids': ['a'], 'extra': 1})[0] == 400
    assert world.hold_raw('Release', {'changes': []})[0] == 400
    status, body = world.hold_raw('State', {'robot_ids': ['nobody']})
    assert status == 404 and body['details']['robot_ids'] == ['nobody'], (status, body)
    assert world.engage(['nobody'])['robots'] == [{'robot_id': 'nobody', 'outcome': 'unknown_robot'}]
    assert world.release([('nobody', 0)])['robots'] == [{'robot_id': 'nobody', 'outcome': 'unknown_robot'}]
    run.ok('surface: HOLD methods, describe facts, closed schemas, retired routes, shared instance')


def engage_release_cycle(run, bot):
    world = run.world
    before = world.robot(bot.name)
    assert not before['held'] and before['stage'] == 'released'
    engaged = world.engage([bot.name])['robots'][0]
    assert engaged['outcome'] == 'engaged' and engaged['held'], engaged
    assert engaged['revision'] == before['revision'] + 1 and engaged['stage'] in ('gated', 'zero_written'), engaged
    again = world.engage([bot.name])['robots'][0]
    assert again['outcome'] == 'already_held' and again['revision'] == engaged['revision'], again
    wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' zero written')
    if bot.wheels:
        # The body of this model may slide on (see Robot): the wheels brake, the stage follows the evidence.
        wait_until(lambda: run.resting(bot), 20, lambda: bot.name + ' brakes its wheels')
        held = world.robot(bot.name)
        assert held['gated_at_ms'] <= held['zero_written_at_ms'], held
    else:
        held = wait_until(lambda: world.robot(bot.name)['stage'] == 'stopped' and world.robot(bot.name), 20,
                          bot.name + ' stopped')
        assert held['gated_at_ms'] <= held['zero_written_at_ms'] <= held['stopped_at_ms'], held
    assert held['fault'] is None and held['revision'] == engaged['revision'] and held['stage'] != 'released', held
    run.at_rest(bot)
    run.others_move(bot)
    revision = held['revision']
    wrong = world.release([(bot.name, revision + 5)])['robots'][0]
    assert wrong['outcome'] == 'conflict' and wrong['held'] and wrong['revision'] == revision, wrong
    stale = world.release([(bot.name, revision)], instance='0' * 31 + '1')['robots'][0]
    assert stale['outcome'] == 'conflict' and stale['held'], stale
    assert world.robot(bot.name)['held'], 'a refused release leaves the robot held'
    released = world.release([(bot.name, revision)])['robots'][0]
    assert released['outcome'] == 'released' and not released['held'], released
    assert released['revision'] == revision + 1 and released['stage'] == 'released', released
    assert world.release([(bot.name, revision)])['robots'][0]['outcome'] == 'not_held', 'a replayed release is void'
    robot = world.robot(bot.name)
    assert robot['stage'] == 'released' and robot['gated_at_ms'] is None and robot['stopped_at_ms'] is None, robot
    wait_until(lambda: run.moving(bot), 8, bot.name + ' moves again on new commands')
    run.ok('%s: engage, %s, others move, conflicts, release, new commands move it'
           % (bot.kind, 'zero_written, wheels brake' if bot.wheels else 'zero_written, stopped'))


def group(run):
    world, (a, b, c) = run.world, run.bots[:3]
    rows = {r['robot_id']: r for r in world.engage([a.name, b.name, c.name])['robots']}
    assert all(r['outcome'] == 'engaged' for r in rows.values()), rows
    for bot in (a, b, c):
        wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' held')
    for bot in run.bots[3:]:
        assert not world.robot(bot.name)['held'] and run.moving(bot), bot.name + ' is not selected'
    changes = [(a.name, rows[a.name]['revision']), (b.name, rows[b.name]['revision'] + 3), (c.name, 0)]
    result = {r['robot_id']: r['outcome'] for r in world.release(changes)['robots']}
    assert result == {a.name: 'released', b.name: 'conflict', c.name: 'conflict'}, result
    assert world.robot(a.name)['stage'] == 'released' and world.robot(b.name)['held'] and world.robot(c.name)['held']
    done = world.release([(b.name, rows[b.name]['revision']), (c.name, rows[c.name]['revision'])])['robots']
    assert [r['outcome'] for r in done] == ['released', 'released']
    for bot in (a, b, c):
        wait_until(lambda: run.moving(bot), 8, bot.name + ' moves after the group')
    run.ok('group engage and partial release per robot')


def no_replay(run, bot, while_held):
    """Commands queued before a release are dropped, whenever they are dispatched."""
    world, fleet = run.world, run.fleet
    fleet.drive(bot.name, 0.0)
    run.settled(bot, 'zero command')
    fleet.silence(bot.name)
    blocker = run.rospy.Publisher('/fixture/block_dispatcher', run.bool, queue_size=1)
    wait_until(lambda: blocker.get_num_connections() > 0, 5, 'dispatcher fixture subscribes')
    blocker.publish(run.bool(data=True))
    wait_until(lambda: world.get('/v1/extensions/dispatcher-test/state')['blocked'], 5, 'dispatcher is blocked')
    try:
        fleet.send(bot.name, CRUISE, count=4)  # received now, dispatched when the barrier opens
        engaged = world.engage([bot.name])['robots'][0]
        assert engaged['outcome'] == 'engaged', engaged
        wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' held')
        revision = engaged['revision']
        if while_held:
            world.operation('POST', '/v1/extensions/dispatcher-test/release')  # dispatched while held
            time.sleep(0.5)
            assert world.robot(bot.name)['held'] and run.resting(bot)
            released = world.release([(bot.name, revision)])['robots'][0]
        else:
            released = world.release([(bot.name, revision)])['robots'][0]
            world.operation('POST', '/v1/extensions/dispatcher-test/release')  # dispatched after the release
        assert released['outcome'] == 'released', released
    finally:
        if world.get('/v1/extensions/dispatcher-test/state')['blocked']:
            world.operation('POST', '/v1/extensions/dispatcher-test/release')
    try:
        run.at_rest(bot, 1.2)
    except AssertionError as error:
        raise AssertionError('queued commands were replayed: %s' % error)
    assert not world.robot(bot.name)['held']
    fleet.drive(bot.name)
    wait_until(lambda: run.moving(bot), 8, bot.name + ' follows a command received after the release')
    run.ok('%s: commands queued before %s are not replayed, a later one moves it'
           % (bot.kind, 'the release while held' if while_held else 'the release'))


def paused_world(run):
    world, bots = run.world, run.bots
    world.operation('POST', '/v1/world/pause')
    assert world.get('/v1/world')['paused']
    time.sleep(0.3)
    revisions = {}
    for bot in bots:
        engaged = world.engage([bot.name])['robots'][0]
        assert engaged['outcome'] == 'engaged', engaged
        revisions[bot.name] = engaged['revision']
        wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 2, bot.name + ' zero while paused')
    time.sleep(1.0)
    for bot in bots:
        stage = world.robot(bot.name)['stage']
        # Rest is evidence from the feedback: the kinematic models are zeroed on the spot, the others keep
        # the velocity of the frozen physics and are not reported as stopped.
        assert stage == ('stopped' if bot.kinematic else 'zero_written'), (bot.name, stage)
    for bot in bots:
        if bot.kinematic:  # the tick writes zero again and again while paused
            generation = world.entity(bot.name)['ref']['generation']
            world.operation('POST', '/v1/entities/%s/state' % bot.name, {
                'generation': generation, 'state': {'twist': {'linear': [1, 0, 0], 'angular': [0, 0, 0]}}})
            wait_until(lambda: world.speed(bot.name) < 1e-6, 1, bot.name + ' zero is re-asserted while paused')
    world.operation('POST', '/v1/world/resume')
    for bot in bots:
        if bot.wheels:
            run.settled(bot, 'physics runs again')
        else:
            wait_until(lambda: world.robot(bot.name)['stage'] == 'stopped', 20, bot.name + ' stops once physics runs')
        assert world.robot(bot.name)['revision'] == revisions[bot.name]
    for bot in bots:
        run.at_rest(bot)
    world.operation('POST', '/v1/world/pause')
    result = world.release([(name, revision) for name, revision in revisions.items()])['robots']
    assert [r['outcome'] for r in result] == ['released'] * len(bots), result
    world.operation('POST', '/v1/world/resume')
    for bot in bots:
        wait_until(lambda: run.moving(bot), 8, bot.name + ' moves after a release while paused')
    run.ok('engage, stages, re-assert and release while the world is paused')


def recreate(run, bot, held):
    world = run.world
    revision = world.robot(bot.name)['revision']
    if held:
        revision = world.engage([bot.name])['robots'][0]['revision']
        wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' held')
    run.remove(bot)
    status, body = world.hold_raw('State', {'robot_ids': [bot.name]})
    assert status == 404 and body['details']['robot_ids'] == [bot.name], 'a removed model leaves the roster'
    assert bot.name not in world.describe()['robots'] and bot.name not in world.hold_entities()
    assert world.engage([bot.name])['robots'][0]['outcome'] == 'unknown_robot'
    run.create(bot)
    assert bot.name in world.hold_entities()
    robot = world.robot(bot.name)
    assert robot['held'] == held and robot['revision'] == revision, (robot, held, revision)
    if held:
        assert robot['stage'] in ('gated', 'zero_written')
        wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' zero on the new model')
        time.sleep(1.5)  # the controller keeps sending; a held model must not move
        run.at_rest(bot)
        released = world.release([(bot.name, revision)])['robots'][0]
        assert released['outcome'] == 'released', released
    wait_until(lambda: run.moving(bot), 10, bot.name + ' moves')
    run.ok('%s: %s model deleted and created again' % (bot.kind, 'held' if held else 'released'))


def restart(run, bot):
    """A new world host is a new instance: HOLD is not persisted and a stale caller cannot release."""
    world, stack = run.world, run.stack
    old = world.instance
    revision = world.engage([bot.name])['robots'][0]['revision']
    wait_until(lambda: world.robot(bot.name)['stage'] in ('zero_written', 'stopped'), 5, bot.name + ' held')
    if clean_exit(stack, stack.stop_gazebo()):
        assert not stack.socket.exists(), 'the world endpoint is removed on shutdown'
    world = run.world = stack.start_gazebo()
    assert world.instance != old
    assert world.describe()['instance'] == world.instance and world.describe()['robots'] == []
    run.create(bot)
    assert world.describe()['robots'] == [bot.name] and world.hold_entities() == [bot.name]
    robot = world.robot(bot.name)
    assert (robot['held'], robot['revision'], robot['stage']) == (False, 0, 'released'), robot
    stale = world.release([(bot.name, revision)], instance=old)['robots'][0]
    assert stale['outcome'] == 'conflict', stale
    status, body = world.hold_raw('Release', {
        'expected_instance': old, 'changes': [{'robot_id': bot.name, 'expected_revision': revision}]}, instance=old)
    assert status == 409 and body['error']['code'] == 'conflict', (status, body)
    engaged = world.engage([bot.name])['robots'][0]
    assert engaged['revision'] == 1
    stale = world.release([(bot.name, 1)], instance=old)['robots'][0]
    assert stale['outcome'] == 'conflict' and stale['held'], 'a stale caller cannot release'
    assert world.robot(bot.name)['held']
    released = world.release([(bot.name, 1)])['robots'][0]
    assert released['outcome'] == 'released', released
    run.ok('world host restart: new instance, no persistence, stale release rejected')


SCENARIOS = ('surface', 'cycle', 'group', 'no_replay', 'paused', 'recreate', 'restart')


def clean_exit(stack, code):
    """gzserver must exit 0 on SIGINT. The abort in the transport teardown of Gazebo itself is reported and
    tolerated (it happens once in a few dozen shutdowns and none of our code is on its stack); anything else
    fails. Returns whether the exit was clean."""
    if code == 0:
        return True
    if code == -signal.SIGABRT and GAZEBO_TRANSPORT_ABORT in stack.log_text():
        print('note: gzserver aborted in the transport teardown of Gazebo (known upstream issue)', flush=True)
        return False
    raise AssertionError('gzserver exited with %s after SIGINT' % code)


def survivors(stack, root):
    """Processes of this run that are still alive: gzserver (its argv names the temporary world) and the
    ROS master (named by its port)."""
    patterns = [str(root), r'rosmaster.* -p %d\b' % stack.ros_port, r'roscore.* -p %d\b' % stack.ros_port]
    found = []
    for pattern in patterns:
        result = subprocess.run(['pgrep', '-f', '--', pattern], stdout=subprocess.PIPE, text=True)
        found += [int(pid) for pid in result.stdout.split() if int(pid) != os.getpid()]
    return sorted(set(found))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plugins', nargs='+', required=True, help='directories with the plugin libraries')
    parser.add_argument('--mecanum-source', required=True, help='checkout of gz-mecanum (the Mecanum model)')
    parser.add_argument('--gzserver', default='/usr/bin/gzserver')
    parser.add_argument('--only', default=','.join(SCENARIOS), help='comma separated subset of ' + ', '.join(SCENARIOS))
    parser.add_argument('--keep-logs', type=Path, help='copy the process logs of the run into this directory')
    args = parser.parse_args()
    selected = args.only.split(',')
    assert set(selected) <= set(SCENARIOS), selected
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='hold-world-') as directory:
        root = Path(directory)
        root.chmod(0o700)
        stack = Stack(root, args.plugins, args.gzserver)
        fleet = run = None
        failure = None
        try:
            stack.start_master()
            import rospy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool
            rospy.init_node('hold_test', anonymous=True, disable_signals=True, disable_rostime=True)
            bots = robots(args.mecanum_source)
            world = stack.start_gazebo()
            fleet = Fleet(bots, rospy, Twist)
            run = Run(stack, world, bots, fleet, rospy)
            run.bool = Bool
            for bot in bots:
                if bot.wheels:
                    run.probes[bot.name] = WheelProbe(bot, rospy, JointState)
            threads = []
            for bot in bots:
                run.create(bot)
                threads.append(len(os.listdir('/proc/%d/task' % stack.process.pid)))
            assert max(threads) - threads[1] <= 1, 'chassis models add no threads of their own: %s' % threads
            for bot in bots:
                wait_until(lambda: run.moving(bot), 10, bot.name + ' follows its first command')
            run.ok('five chassis models driven by cmd_vel (threads after each create: %s)' % threads)
            by_name = {b.name: b for b in bots}
            if 'surface' in selected:
                surface(run)
            if 'cycle' in selected:
                for bot in bots:
                    engage_release_cycle(run, bot)
            if 'group' in selected:
                group(run)
            if 'no_replay' in selected:
                for bot in bots:
                    no_replay(run, bot, while_held=False)
                no_replay(run, by_name['scout-skid'], while_held=True)
                no_replay(run, by_name['mecanum-ideal'], while_held=True)
            if 'paused' in selected:
                paused_world(run)
            if 'recreate' in selected:
                for bot in bots:
                    recreate(run, bot, held=True)
                recreate(run, by_name['mecanum-hifi'], held=False)
            if 'restart' in selected:
                restart(run, by_name['scout-skid'])
            if clean_exit(stack, run.stack.stop_gazebo()):
                assert not stack.socket.exists(), 'the world endpoint survived the shutdown'
                run.ok('graceful gzserver shutdown')
            else:
                run.ok('gzserver shutdown: aborted in the transport teardown of Gazebo')
        except BaseException as error:
            failure = error
            raise
        finally:
            if fleet is not None:
                fleet.close()
            errors = stack.close()
            if failure is not None:
                print('---- stack: %s' % (errors or 'processes were alive'))
                for log in sorted(root.glob('gzserver-*.log')) + [root / 'processes.log']:
                    if log.exists():
                        print('---- %s (tail)' % log.name)
                        print('\n'.join(log.read_text(errors='replace').splitlines()[-60:]))
            if args.keep_logs:
                args.keep_logs.mkdir(parents=True, exist_ok=True)
                for log in list(root.glob('*.log')):
                    (args.keep_logs / log.name).write_text(log.read_text(errors='replace'))
            time.sleep(0.5)
            left = survivors(stack, root)
            for pid in left:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if failure is None:
                assert not errors, errors
                assert not left, 'processes of the run survived: %s' % left
    print(json.dumps({'checks': run.checks, 'seconds': round(time.monotonic() - started, 1)}, indent=1))


if __name__ == '__main__':
    main()
