#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
import tkinter as tk
import numpy as np


class SliderNode(Node):
    def __init__(self):
        super().__init__('actuation_slider')

        # Publishers
        self.pub_cam = self.create_publisher(Quaternion, 'cam_pose', 10)
        self.pub_wheel = self.create_publisher(Float64, 'wheel_pose', 10)
        self.pub_pedal = self.create_publisher(Float64, 'pedal_pose', 10)

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Actuation Control Panel")

        # Wheel
        tk.Label(self.root, text="Wheel (deg)").pack()
        self.wheel_slider = tk.Scale(self.root, from_=-45, to=45,
                                     orient=tk.HORIZONTAL, command=self.update_wheel)
        self.wheel_slider.pack()

        # Camera Tilt
        tk.Label(self.root, text="Camera Tilt (deg)").pack()
        self.tilt_slider = tk.Scale(self.root, from_=0, to=180,
                                    orient=tk.HORIZONTAL, command=self.update_cam)
        self.tilt_slider.set(90)
        self.tilt_slider.pack()

        # Camera Pan
        tk.Label(self.root, text="Camera Pan (deg)").pack()
        self.pan_slider = tk.Scale(self.root, from_=0, to=180,
                                   orient=tk.HORIZONTAL, command=self.update_cam)
        self.pan_slider.set(90)
        self.pan_slider.pack()

        # Throttle
        tk.Label(self.root, text="Throttle (-0.25 to 0.25)").pack()
        self.throttle_slider = tk.Scale(self.root, from_=-0.25, to=0.25,
                                        resolution=0.01, orient=tk.HORIZONTAL, command=self.update_pedal)
        self.throttle_slider.set(0)
        self.throttle_slider.pack()

    def update_wheel(self, value):
        msg = Float64()
        msg.data = np.deg2rad(float(value))
        self.pub_wheel.publish(msg)

    def update_cam(self, _):
        msg = Quaternion()
        msg.y = np.deg2rad(float(self.tilt_slider.get()))
        msg.z = np.deg2rad(float(self.pan_slider.get()))
        self.pub_cam.publish(msg)

    def update_pedal(self, value):
        msg = Float64()
        msg.data = float(value)
        self.pub_pedal.publish(msg)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SliderNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

