import os
from glob import glob

from setuptools import setup

package_name = 'sentry_gazebo'


def data_tree(src):
    """Every file under ``src``, installed into share/<pkg>/<src>/... .

    ``models/`` is a Gazebo resource directory tree (the official arena mesh),
    so it has to be installed recursively.
    """
    out = []
    for root, _dirs, files in os.walk(src):
        if not files:
            continue
        out.append((os.path.join('share', package_name, root),
                    [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.4.0',
    # launch/ is not a python package: an __init__.py there would shadow ROS'
    # own "launch" module.
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # ros2 run searches lib/<pkg>; colcon installs ament_python console
        # scripts into <prefix>/bin instead, so the node launcher is a data
        # file (see the note in scripts/teleop).
        (os.path.join('lib', package_name), ['scripts/teleop']),
    ] + data_tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang',
    maintainer_email='yang@example.com',
    description='RMUL 2027 sentry Gazebo Harmonic simulation (arena + MID-360 + L2)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
