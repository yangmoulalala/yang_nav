from setuptools import setup

package_name = 'lidar_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang',
    maintainer_email='yang@example.com',
    description='Single/dual LiDAR point cloud merge for ZLIO',
    license='Apache-2.0',
    entry_points={'console_scripts': ['merge_node = lidar_fusion.merge_node:main',
                                      'base_tf = lidar_fusion.base_tf:main']},
)
