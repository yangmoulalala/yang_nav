"""Safety contract for an unobserved chassis orientation on hardware and simulation."""
import ast
from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
from bringup.configuration import merge
import yaml

ROOT = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((ROOT/'params/nav2.yaml').read_text())


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


def test_shared_prediction_cycle_matches_control_cycle():
    base = PARAMS['controller_server']['ros__parameters']
    assert abs(1.0/base['controller_frequency'] - base['FollowPath']['model_dt']) < 1e-9


def test_real_profile_only_reduces_local_inflation_radius():
    real = merge(PARAMS, yaml.safe_load((ROOT / 'params/nav2_real.yaml').read_text()))
    shared = PARAMS['local_costmap']['local_costmap']['ros__parameters']
    local = real['local_costmap']['local_costmap']['ros__parameters']
    footprint = ast.literal_eval(local['footprint'])
    padding = local['footprint_padding']
    padded_radius = max(math.hypot(abs(x) + padding, abs(y) + padding) for x, y in footprint)
    assert shared['inflation_layer']['inflation_radius'] == 0.65
    assert padded_radius < local['inflation_layer']['inflation_radius'] == 0.60
    assert local == {**shared, 'inflation_layer': {**shared['inflation_layer'], 'inflation_radius': 0.60}}
    assert real['global_costmap'] == PARAMS['global_costmap']
    assert real['collision_monitor'] == PARAMS['collision_monitor']


def test_exported_real_tuning_preserves_fixed_frame_smoother(tmp_path):
    launch_spec = spec_from_file_location('navigation_launch', ROOT / 'launch/navigation.launch.py')
    launch_module = module_from_spec(launch_spec)
    launch_spec.loader.exec_module(launch_module)
    real = launch_module.navigation_params(ROOT / 'params/nav2.yaml', ROOT / 'params/nav2_real.yaml')
    custom_file = tmp_path / 'nav2.yaml'
    custom_file.write_text(yaml.safe_dump(real))
    tuned = launch_module.navigation_params(custom_file)
    normal = ('nav2_velocity_smoother', 'velocity_smoother')
    fixed = ('nav_behaviors', 'fixed_frame_velocity_smoother')
    assert launch_module.velocity_smoother_executable('real', real) == fixed
    assert launch_module.velocity_smoother_executable('real', tuned) == fixed
    assert launch_module.velocity_smoother_executable('real', PARAMS) == normal
    assert launch_module.velocity_smoother_executable('sim', PARAMS) == normal
    assert launch_module.velocity_smoother_executable('sim', tuned) == fixed


def test_real_profile_matches_controller_smoother_and_gate_without_changing_collision_monitor():
    real = merge(PARAMS, yaml.safe_load((ROOT / 'params/nav2_real.yaml').read_text()))
    follow = real['controller_server']['ros__parameters']['FollowPath']
    smoother = real['velocity_smoother']['ros__parameters']
    gate = real['cmd_gate']['ros__parameters']
    limit = gate['limits']['normal']['max_linear']
    assert limit == gate['speed_limit_reference'] == follow['zone']['speed_limit_reference'] == 2.0
    assert smoother['max_velocity'][:2] == [follow['vx_max'], follow['vy_max']] == [limit, limit]
    assert smoother['min_velocity'][:2] == [follow['vx_min'], -follow['vy_max']]
    assert smoother['max_accel'][:2] == [follow['ax_max'], follow['ay_max']] == [1.8, 1.8]
    assert smoother['max_decel'][:2] == [follow['ax_min'], follow['ay_min']] == [-2.0, -2.0]
    assert follow['TranslationVelocityCritic']['max_speed'] == 2.00
    assert follow['TranslationVelocityCritic']['direction_lookahead'] == 0.10
    assert follow['TranslationVelocityCritic']['velocity_score_time'] == 0.60
    assert follow['path_departure_limit'] == 0.15
    assert follow['fixed_frame_translation'] is True
    assert all(mode['max_linear'] <= limit for mode in gate['limits'].values())
    assert gate['require_observation']
    assert real['collision_monitor'] == PARAMS['collision_monitor']
    assert PARAMS['controller_server']['ros__parameters']['FollowPath']['vx_max'] == 2.0
    assert PARAMS['controller_server']['ros__parameters']['FollowPath']['fixed_frame_translation'] is False
    assert PARAMS['controller_server']['ros__parameters']['FollowPath']['path_departure_limit'] == 0.0
