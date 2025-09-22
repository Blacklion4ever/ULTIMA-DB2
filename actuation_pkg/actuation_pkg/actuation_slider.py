#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8MultiArray
import tkinter as tk
import numpy as np

class SliderNode(Node):
    def __init__(self):
        super().__init__('slider_command_node')

        # Publishers vers station_link_pkg
        self.pub_pan_tilt = self.create_publisher(Int16MultiArray, '/rover/command/pan_tilt', 10)
        self.pub_steer_prop = self.create_publisher(Int8MultiArray, '/rover/command/steering_propulsion', 10)

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("Actuation Control Panel")

        # ---- Wheel / Steering ----
        tk.Label(self.root, text="Wheel / Steering (deg)").pack()
        self.wheel_slider = tk.Scale(self.root, from_=-100, to=100,
                                     orient=tk.HORIZONTAL, command=self.update_steer_prop)
        self.wheel_slider.pack()

        # ---- Camera Tilt ----
        tk.Label(self.root, text="Camera Tilt (deg)").pack()
        self.tilt_slider = tk.Scale(self.root, from_=-180, to=180,
                                    orient=tk.HORIZONTAL, command=self.update_pan_tilt)
        self.tilt_slider.set(0)
        self.tilt_slider.pack()

        # ---- Camera Pan ----
        tk.Label(self.root, text="Camera Pan (deg)").pack()
        self.pan_slider = tk.Scale(self.root, from_=-180, to=180,
                                   orient=tk.HORIZONTAL, command=self.update_pan_tilt)
        self.pan_slider.set(0)
        self.pan_slider.pack()

        # ---- Throttle / Propulsion ----
        tk.Label(self.root, text="Throttle (-100 → +100%)").pack()
        self.throttle_slider = tk.Scale(self.root, from_=-100, to=100,
                                        orient=tk.HORIZONTAL, command=self.update_steer_prop)
        self.throttle_slider.set(0)
        self.throttle_slider.pack()

    # ---- Callbacks ----
    def update_pan_tilt(self, _):
        pan_deg = int(self.pan_slider.get())
        tilt_deg = int(self.tilt_slider.get())
        msg = Int16MultiArray()
        msg.data = [pan_deg, tilt_deg]
        self.pub_pan_tilt.publish(msg)

    def update_steer_prop(self, _):
        steering = int(self.wheel_slider.get())
        propulsion = int(self.throttle_slider.get())
        msg = Int8MultiArray()
        msg.data = [steering, propulsion]
        self.pub_steer_prop.publish(msg)

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
