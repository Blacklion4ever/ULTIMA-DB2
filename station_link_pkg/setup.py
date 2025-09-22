from setuptools import setup
import os
from glob import glob

package_name = 'station_link_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required for ROS 2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package.xml explicitly
        ('share/' + package_name, ['package.xml']),
        # (Optional) Install launch files if you have them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'common_link_lib'],
    zip_safe=True,
    maintainer='auto-gen',
    maintainer_email='dev@example.com',
    description='ROS 2 package auto-generated for station_link_pkg',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_node = station_link_pkg.command_node:main',
            'supervision_node = station_link_pkg.supervision_node:main',
        ],
    },
)
