from setuptools import setup
import os
from glob import glob

package_name = 'communication_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # <-- Ajout du dossier launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'digi-xbee'],
    zip_safe=True,
    maintainer='ultima',
    maintainer_email='you@example.com',
    description='ROS2 communication node for Xbee',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capcom_node = communication_pkg.cap_com:main',
        ],
    },
)
