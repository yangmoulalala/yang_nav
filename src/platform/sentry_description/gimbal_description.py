#!/usr/bin/env python3
"""Extract the observable rigid turret subtree; never invent a chassis yaw.

The root gimbal_nav has the CAD zero-heading FLU axes, origin at the chassis
centre (5 mm below the coaxial turret pivot). yaw_offset is the calibrated
CCW heading of MCU +X relative to CAD zero, in radians. It is NOT yaw_joint.
"""
import argparse
import copy
import math
import xml.etree.ElementTree as ET


def gimbal_description(source, yaw_offset=0.0):
    if not math.isfinite(yaw_offset):
        raise ValueError('gimbal yaw offset must be finite')
    robot = ET.fromstring(source)
    yaw = robot.find("joint[@name='yaw_joint']")
    if yaw is None or yaw.find('parent').get('link') != 'base_link':
        raise ValueError('expected the coaxial base_link -> yaw_link joint')
    xyz = list(map(float, yaw.find('origin').get('xyz').split()))
    if abs(xyz[0]) > 1e-9 or abs(xyz[1]) > 1e-9:
        raise ValueError('gimbal reference requires a coaxial turret pivot')
    joints = robot.findall('joint')
    reachable = {yaw.find('child').get('link')}
    selected = []
    while True:
        added = [j for j in joints if j not in selected and
                 j.find('parent').get('link') in reachable]
        if not added:
            break
        for joint in added:
            if joint.get('type') != 'fixed':
                raise ValueError('turret sensors must form a rigid subtree')
            selected.append(joint)
            reachable.add(joint.find('child').get('link'))
    result = ET.Element('robot', name='sentry_observable_gimbal')
    ET.SubElement(result, 'link', name='gimbal_nav')
    mount = copy.deepcopy(yaw)
    mount.set('name', 'gimbal_nav_mount')
    mount.set('type', 'fixed')
    mount.find('parent').set('link', 'gimbal_nav')
    mount.remove(mount.find('axis'))
    angles = list(map(float, mount.find('origin').get('rpy').split()))
    # Left multiplication by Rz(-offset) leaves roll and pitch unchanged.
    angles[2] -= yaw_offset
    mount.find('origin').set('rpy', ' '.join(format(v, '.15g') for v in angles))
    result.append(mount)
    for link in robot.findall('link'):
        if link.get('name') in reachable:
            result.append(copy.deepcopy(link))
    for joint in selected:
        result.append(copy.deepcopy(joint))
    return ET.tostring(result, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf')
    parser.add_argument('--yaw-offset', type=float, default=0.0)
    args = parser.parse_args()
    with open(args.urdf) as stream:
        print(gimbal_description(stream.read(), args.yaw_offset))


if __name__ == '__main__':
    main()
