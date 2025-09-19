from setuptools import setup

package_name = 'sensor_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    author='TonNom',
    author_email='ton.email@example.com',
    description='Python nodes for sensor control',
    entry_points={
        'console_scripts': [
            'video_node = sensor_pkg.video_node:main',
            'set_controls_client = sensor_pkg.set_controls_client:main',
        ],
    },
)

