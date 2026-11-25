import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

from sentry_gazebo.launch_utils import gz_environment, urdf_path
from sentry_gazebo.prepare_world import generate
from sentry_gazebo.sdf_builder import build


@pytest.mark.parametrize('arena', ['rmul', 'rmuc'])
def test_installed_world_resources(tmp_path, arena):
    world = generate(tmp_path, urdf_path(), arena=arena)
    assert (tmp_path / 'models/sentry_description/meshes/base_link.stl').is_file()
    assert str(tmp_path / 'models') in (tmp_path / 'env.sh').read_text()
    subprocess.run(['gz', 'sdf', '-k', str(world)], check=True,
                   env={**os.environ, **gz_environment(tmp_path)}, timeout=30)


def test_model_matches_real_imu_mount_and_separates_navigation_from_actuation():
    urdf = ET.parse(urdf_path()).getroot()
    model = ET.fromstring(build(urdf_path())).find("world/model[@name='sentry']")
    sensor = model.find("link[@name='mid360_link']/sensor[@type='imu']")
    mount = urdf.find("joint[@name='mid360_imu_joint']/origin")
    expected = list(map(float, (mount.get('xyz') + ' ' + mount.get('rpy')).split()))
    assert list(map(float, sensor.findtext('pose').split())) == pytest.approx(expected)
    assert model.find("joint[@name='yaw_joint']").get('type') == 'revolute'
    drive = model.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    assert drive.findtext('topic') == '/sim/chassis_cmd_vel'
