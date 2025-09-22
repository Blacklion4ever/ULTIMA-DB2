#!/usr/bin/env python3
import os
from pathlib import Path

# Racine du dépôt
ROOT = Path(__file__).resolve().parent

# Packages à créer
PACKAGES = {
    "station_link_pkg": {
        "nodes": ["command_node.py", "supervision_node.py", "serial_manager.py", "utils.py"]
    },
    "rover_link_pkg": {
        "nodes": ["serial_listener.py", "command_publisher.py", "utils.py"]
    },
    "common_link_lib": {
        "files": ["frame_manager.py"]
    }
}

def create_ros2_package(pkg_name, nodes):
    """Créer un package ROS2 Python"""
    pkg_path = ROOT / pkg_name
    if not pkg_path.exists():
        pkg_path.mkdir()
        print(f"[OK] Dossier créé : {pkg_path}")

    # Sous dossier du code Python
    module_path = pkg_path / pkg_name
    module_path.mkdir(exist_ok=True)

    # __init__.py
    (module_path / "__init__.py").write_text("# Init package\n")

    # setup.py
    (pkg_path / "setup.py").write_text(f"""\
from setuptools import setup

package_name = '{pkg_name}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto-gen',
    maintainer_email='dev@example.com',
    description='ROS 2 package auto-generated for {pkg_name}',
    license='MIT',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
        ],
    }},
)
""")

    # package.xml minimal
    (pkg_path / "package.xml").write_text(f"""\
<?xml version="1.0"?>
<package format="3">
  <name>{pkg_name}</name>
  <version>0.0.1</version>
  <description>Auto-generated package for {pkg_name}</description>
  <maintainer email="dev@example.com">Auto Gen</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
""")

    # Resource folder
    resource_dir = pkg_path / "resource"
    resource_dir.mkdir(exist_ok=True)
    (resource_dir / pkg_name).write_text("")

    # Création des nodes vides
    for node in nodes:
        node_file = module_path / node
        if not node_file.exists():
            node_file.write_text(f"# Node {node} - à implémenter\n")
            print(f"[OK] Node créé : {node_file}")

def create_common_lib():
    """Créer la librairie commune"""
    lib_path = ROOT / "common_link_lib"
    lib_path.mkdir(exist_ok=True)

    (lib_path / "__init__.py").write_text("# Common link lib init\n")

    frame_manager_path = lib_path / "frame_manager.py"
    if not frame_manager_path.exists():
        frame_manager_path.write_text("""\
class FrameManager:
    START_CODE = 0x55

    def __init__(self):
        self.frame_counter = 0
        self.buffer = bytearray()
        self.expected_size = None

    def build_frame(self, header, subheader, payload_bytes):
        counter = (self.frame_counter + 1) & 0xFFFF
        self.frame_counter = counter
        size = len(payload_bytes)
        frame = bytearray()
        frame.append(self.START_CODE)
        frame.extend(counter.to_bytes(2, 'big'))
        frame.append(header)
        frame.append(subheader)
        frame.append(size)
        frame.extend(payload_bytes)
        checksum = sum(frame) % 256
        frame.append(checksum)
        return bytes(frame)

    def feed(self, data):
        frames = []
        for byte in data:
            if len(self.buffer) == 0 and byte != self.START_CODE:
                continue
            self.buffer.append(byte)
            if len(self.buffer) == 6:
                payload_size = self.buffer[5]
                self.expected_size = 6 + payload_size + 1
            if self.expected_size and len(self.buffer) >= self.expected_size:
                full_frame = bytes(self.buffer[:self.expected_size])
                if self.verify_checksum(full_frame):
                    frames.append(full_frame)
                self.buffer.clear()
                self.expected_size = None
        return frames

    def verify_checksum(self, frame_bytes):
        return (sum(frame_bytes[:-1]) % 256) == frame_bytes[-1]

    def parse_frame(self, frame_bytes):
        if not self.verify_checksum(frame_bytes):
            raise ValueError("Checksum invalide")
        return {
            "start_code": frame_bytes[0],
            "counter": int.from_bytes(frame_bytes[1:3], 'big'),
            "header": frame_bytes[3],
            "subheader": frame_bytes[4],
            "size": frame_bytes[5],
            "payload": frame_bytes[6:-1],
        }
""")
        print(f"[OK] Librairie commune créée : {frame_manager_path}")

def main():
    print(f"[INFO] Génération des packages dans {ROOT}")

    # Génération station_link_pkg
    create_ros2_package("station_link_pkg", PACKAGES["station_link_pkg"]["nodes"])

    # Génération rover_link_pkg
    create_ros2_package("rover_link_pkg", PACKAGES["rover_link_pkg"]["nodes"])

    # Génération common_link_lib
    create_common_lib()

    print("[DONE] Structure ROS 2 générée avec succès.")

if __name__ == "__main__":
    main()

