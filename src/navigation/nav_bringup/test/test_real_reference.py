"""Safety contract for an unobserved chassis orientation in the real profile."""
import ast
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((ROOT/'params/real.yaml').read_text())


def test_obstacle_frames_and_physical_envelope_agree():
    footprints = []
    for name in ('local_costmap', 'global_costmap'):
        section = PARAMS[name][name]['ros__parameters']
        assert section['robot_base_frame'] == 'gimbal_nav'
        footprints.append(ast.literal_eval(section['footprint']))
    shaper = PARAMS['smoother_server']['ros__parameters']['zone_shaper']
    assert shaper['robot_base_frame'] == 'gimbal_nav'
    footprints.append(ast.literal_eval(shaper['footprint']))
    assert footprints[0] == footprints[1] == footprints[2]
    # The existing rectangular physical envelope at ANY chassis/turret angle
    # must lie inside the new envelope before adding collision/planning padding.
    polygon = footprints[0]
    for tick in range(360):
        a = math.radians(tick)
        for x, y in ((.38,.3),(.38,-.3),(-.3,-.3),(-.3,.3)):
            point = (math.cos(a)*x-math.sin(a)*y, math.sin(a)*x+math.cos(a)*y)
            for start, end in zip(polygon, polygon[1:]+polygon[:1]):
                cross = ((end[0]-start[0])*(point[1]-start[1]) -
                         (end[1]-start[1])*(point[0]-start[0]))
                assert cross >= -1e-7
    for node, key in (('bt_navigator','robot_base_frame'),('behavior_server','robot_base_frame'),
                      ('collision_monitor','base_frame_id'),('terrain_obstacles','base_frame')):
        assert PARAMS[node]['ros__parameters'][key] == 'gimbal_nav'


def test_directional_actions_cannot_use_turret_heading_as_chassis_heading():
    zone = PARAMS['controller_server']['ros__parameters']['FollowPath']['zone']
    assert zone['body_heading_available'] is False
    assert zone['angular_control'] is False


def test_real_prediction_cycle_matches_control_cycle():
    shared = yaml.safe_load((ROOT/'params/nav2.yaml').read_text())
    base = shared['controller_server']['ros__parameters']
    real = PARAMS['controller_server']['ros__parameters']
    assert abs(1.0/real['controller_frequency'] - base['FollowPath']['model_dt']) < 1e-9
    assert real['FollowPath']['vx_max'] == real['FollowPath']['vy_max'] == .25
    assert real['FollowPath']['vx_min'] == -.25
