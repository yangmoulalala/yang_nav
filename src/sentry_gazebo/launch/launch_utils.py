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


def ws_source_dir():
    """Workspace ``src`` directory, derived from the installed share path."""
    here = share().parent
    # <ws>/install/sentry_gazebo/share/sentry_gazebo  ->  <ws>/src
    for parent in here.parents:
        if parent.name == 'install':
            return parent.parent / 'src'
    return here


def urdf_path():
    """The URDF from ``sentry_description`` (installed, else source tree)."""
    try:
        return share(DESCRIPTION_PKG) / 'urdf' / 'sentry_robot.urdf'
    except Exception:
        return ws_source_dir() / DESCRIPTION_PKG / 'urdf' / 'sentry_robot.urdf'


def generated_dir():
    """Where the generated SDF world is written at launch."""
    return Path(os.environ.get('SENTRY_GAZEBO_GEN', '/tmp/sentry_gazebo_generated'))


def models_dir():
    """Committed Gazebo resources: the official arena mesh (``rmul_2027``)."""
    return share() / 'models'


def resource_paths():
    """Directories Gazebo needs for ``model://`` lookup, highest priority first."""
    paths = [str(generated_dir() / 'models'), str(models_dir())]
    src_models = ws_source_dir() / PKG / 'models'
    if src_models.is_dir():
        paths.append(str(src_models))
    return list(dict.fromkeys(paths))


# ---------------------------------------------------------------------------
# Gazebo environment
# ---------------------------------------------------------------------------
def gz_environment():
    """Environment variables for the Gazebo process."""
    return {
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(resource_paths()),
    }
