from setuptools import setup
import os
from glob import glob

package_name = 'actuation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # <-- Ajout du dossier launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to control PCA9685 servos and ESC with joystick',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'actuators = actuation_pkg.actuators:main',
            'actuation_slider = actuation_pkg.actuation_slider:main',
            'actuation_joystick = actuation_pkg.actuation_joystick:main',
        ],
    },
)

