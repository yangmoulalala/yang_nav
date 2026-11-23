"""Launch parameters shared by sim.launch.py and prepare_world.py.

Kept in one place so the headless (server) launch, the GUI launch, the world
generator and the shell ``env.sh`` cannot drift apart.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

PKG = 'sentry_gazebo'
DESCRIPTION_PKG = 'sentry_description'


def share(name=PKG):
    """Installed share directory of ``name``."""
    return Path(get_package_share_directory(name)).resolve()


def urdf_path():
    return share(DESCRIPTION_PKG) / 'urdf' / 'sentry_robot.urdf'


def generated_dir():
    """Where the generated SDF world is written at launch."""
    return Path(os.environ.get('SENTRY_GAZEBO_GEN', '/tmp/sentry_gazebo_generated'))


def gz_environment(gen_root=None):
    resources = (Path(gen_root or generated_dir()) / 'models', share() / 'models')
    return {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(map(str, resources))}
