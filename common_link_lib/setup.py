from setuptools import setup
import os
from glob import glob

package_name = 'common_link_lib'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # <-- doit correspondre au dossier
    install_requires=['setuptools'],
    data_files=[
        # Required for ROS 2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install the package.xml explicitly
        ('share/' + package_name, ['package.xml']),
        # (Optional) Install launch files if you have them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='TonNom',
    maintainer_email='ton.email@example.com',
    description='Common frame manager for rover/station',
    license='MIT',
    entry_points={
        'console_scripts': [
            # si tu as des nodes ici, sinon laisse vide
        ],
    },
)
