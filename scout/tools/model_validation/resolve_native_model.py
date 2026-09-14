#!/usr/bin/env python3
"""Resolve the frozen Scout five-body model; reject unsupported silent shortcuts.

The output records source hashes and every constructor/default assumption.
Native replay still requires conformance against the original Gazebo receipt.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ORDER = ['front_right_wheel','front_left_wheel','rear_left_wheel','rear_right_wheel']


def numbers(element, path, count, default=None):
    text = element.findtext(path)
    if text is None:
        if default is None:
            raise ValueError(f'missing required vector {path}')
        return list(default)
    value = [float(v) for v in text.split()]
    if len(value) != count or not all(math.isfinite(v) for v in value):
        raise ValueError(f'invalid {path}')
    return value


def scalar(element,path,default=None):
    text = element.findtext(path)
    if text is None and default is None:
        raise ValueError(f'missing required scalar {path}')
    value = float(text) if text is not None else float(default)
    if not math.isfinite(value):
        raise ValueError(f'nonfinite {path}')
    return value


def rotation(pose):
    r,p,y=pose[3:]
    c,s,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*s-sy*c,cy*sp*c+sy*s],
            [sy*cp,sy*sp*s+cy*c,sy*sp*c-cy*s],[-sp,cp*s,cp*c]]


def rotate(matrix,value):
    return [sum(a*b for a,b in zip(row,value)) for row in matrix]


def surface(collision):
    return {'mu':scalar(collision,'surface/friction/ode/mu',1.),
            'mu2':scalar(collision,'surface/friction/ode/mu2',1.),
            'slip1':scalar(collision,'surface/friction/ode/slip1',0.),
            'slip2':scalar(collision,'surface/friction/ode/slip2',0.),
            'fdir1':numbers(collision,'surface/friction/ode/fdir1',3,[0,0,0]),
            'kp':scalar(collision,'surface/contact/ode/kp',1.e12),
            'kd':scalar(collision,'surface/contact/ode/kd',1.),
            'max_vel':scalar(collision,'surface/contact/ode/max_vel',.01),
            'min_depth':scalar(collision,'surface/contact/ode/min_depth',0.),
            'max_contacts':int(scalar(collision,'max_contacts',10))}


def resolve(model_path,world_path,params_path,step_override=None):
    model=ET.parse(model_path).getroot()
    if model.tag!='model':
        raise ValueError('runtime-model.sdf must contain the resolved single model')
    world=ET.parse(world_path).getroot()
    if world.tag!='world':
        raise ValueError('runtime-world.sdf must contain the resolved world')
    params=yaml.safe_load(params_path.read_text())
    namespace=model.attrib['name']
    commands=params[namespace+'_scout_skid_steer_controller']
    pid=params[namespace]['gazebo_ros_control']['pid_gains']
    physics=world.find('physics')
    step=scalar(physics,'max_step_size') if step_override is None else step_override
    for plugin in model.findall('plugin'):
        if 'gazebo_ros_control' in plugin.attrib.get('filename','') and plugin.find('controlPeriod') is not None:
            period=scalar(plugin,'controlPeriod')
            if abs(period-step)>1.e-12:
                raise ValueError('native reference does not implement decimated readSim; do not silently replace the control period')
    joint_map={j.attrib['name']:j for j in model.findall('joint')}
    link_map={l.attrib['name']:l for l in model.findall('link')}
    bodies=[];motors=[]
    for index,name in enumerate(['base_link']+ORDER):
        if index==0:
            link=link_map['base_link'];pose=[0]*6
        else:
            joint=joint_map[name]
            if joint.findtext('parent')!='base_link':
                raise ValueError('unexpected wheel topology')
            if joint.attrib.get('type') not in ('revolute','continuous'):
                raise ValueError('expected wheel hinge')
            pose=numbers(joint,'pose',6,[0]*6)
            link=link_map[joint.findtext('child')]
            if any(abs(v)>1.e-12 for v in numbers(link,'pose',6,[0]*6)):
                raise ValueError('nonzero wheel link/joint transform requires explicit support')
            gains=pid[name]
            if gains['d']!=0:
                raise ValueError('reference currently implements the frozen PI, not arbitrary PID')
            if gains['i_clamp_max']!=-gains['i_clamp_min']:
                raise ValueError('asymmetric integral clamp is not supported by this reference')
            if scalar(joint,'axis/dynamics/spring_stiffness',0)!=0:
                raise ValueError('nonzero spring stiffness must not be silently omitted')
            implicit=joint.findtext('physics/ode/implicit_spring_damper','0')
            if implicit not in ('0','false','False'):
                raise ValueError('implicit joint damping requires a separate implementation')
            motors.append({'name':name,'anchor':pose[:3],
                           'axis_world':rotate(rotation(pose),numbers(joint,'axis/xyz',3)),
                           'p':gains['p'],'i':gains['i'],'antiwindup':gains['antiwindup'],
                           'i_clamp_nm':gains['i_clamp_max'],
                           'effort_nm':scalar(joint,'axis/limit/effort'),
                           'velocity_radps':scalar(joint,'axis/limit/velocity'),
                           'damping_nms_per_rad':scalar(joint,'axis/dynamics/damping',0.)})
        inertial=link.find('inertial')
        inertial_pose=numbers(inertial,'pose',6,[0]*6)
        if any(abs(v)>1.e-12 for v in inertial_pose[3:]):
            raise ValueError('rotated inertial frame needs an explicit inertia transformation')
        geometry=[]
        for c in link.findall('collision'):
            if c.find('geometry/box') is not None:
                shape='box';size=numbers(c,'geometry/box/size',3)
            elif c.find('geometry/cylinder') is not None:
                shape='cylinder';size=[scalar(c,'geometry/cylinder/radius'),scalar(c,'geometry/cylinder/length'),0.]
            else:
                raise ValueError('native conformance requires explicit support for this collision shape')
            geometry.append({'name':c.attrib['name'],'shape':shape,'dimensions':size,
                             'pose':numbers(c,'pose',6,[0]*6),'surface':surface(c)})
        bodies.append({'name':link.attrib['name'],'pose':pose,'com':inertial_pose[:3],
                       'mass_kg':scalar(inertial,'mass'),
                       'inertia':[scalar(inertial,'inertia/'+key) for key in ['ixx','iyy','izz','ixy','ixz','iyz']],
                       'collisions':geometry})
    ground=world.find("model[@name='ground_plane']/link/collision")
    if ground is None or ground.find('geometry/plane') is None:
        raise ValueError('flat ground receipt required')
    gravity=numbers(world,'gravity',3)
    if gravity[:2]!=[0.,0.]:
        raise ValueError('nonvertical gravity is not supported')
    hashes={str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in [model_path,world_path,params_path]}
    return {'schema':'scout-native-ode-v1','provenance_sha256':hashes,
            'scope':'reconstruction; NOT Gazebo binary equivalence or physical acceptance',
            'assumptions':['omitted SDF contact defaults resolved as Gazebo ODESurfaceParams constructor / standard mu=1',
                           'zero initial velocity and fixed 1 s pre-record settling, no time-shift fitting',
                           'explicit hinge damping; zero torsional patch radius; no ROS scheduling jitter',
                           'controlPeriod equals physics step; output timestamps are post-integration'],
            'input_mode':'body_commands','initial_base_height_m':.184,'settle_s':1.,'tail_s':1.,
            'world':{'step_s':step,'gravity_z_mps2':gravity[2],
                     'friction_model':physics.findtext('ode/solver/friction_model'),
                     'iterations':int(scalar(physics,'ode/solver/iters')),'sor':scalar(physics,'ode/solver/sor'),
                     'cfm':scalar(physics,'ode/constraints/cfm'),'erp':scalar(physics,'ode/constraints/erp'),
                     'surface_layer_m':scalar(physics,'ode/constraints/contact_surface_layer'),
                     'max_correcting_mps':scalar(physics,'ode/constraints/contact_max_correcting_vel'),
                     'max_contacts':int(params['gazebo']['max_contacts'])},
            'command':{'delay_s':commands['command_delay_s'],'gain':commands['command_gain'],
                       'angular_gain':commands['angular_command_gain'],'allocation_span_m':commands['wheel_separation'],
                       'radius_m':commands['wheel_radius'],'max_v_mps':commands['max_linear_speed'],
                       'max_w_radps':commands['max_angular_speed']},
            'bodies':bodies,'motors':motors,'ground':surface(ground)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_directory',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--step-s',type=float,choices=[.001,.002,.004])
    args=parser.parse_args();root=args.frozen_directory
    model=resolve(root/'runtime-model.sdf',root/'runtime-world.sdf',root/'runtime-params.yaml',args.step_s)
    args.output.write_text(json.dumps(model,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
