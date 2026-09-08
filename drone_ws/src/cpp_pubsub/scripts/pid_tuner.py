#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import tkinter as tk
import threading
import sys

class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner_gui')
        self.pub = self.create_publisher(Vector3, '/pid_gains', 10)

    def publish_gains(self, kp, ki, kd):
        msg = Vector3()
        msg.x = float(kp)
        msg.y = float(ki)
        msg.z = float(kd)
        self.pub.publish(msg)

def start_gui(node):
    root = tk.Tk()
    root.title("Live Drone Tuning Dashboard")
    root.geometry("350x250")
    root.configure(padx=20, pady=20)

    def on_slider_move(event):
        node.publish_gains(kp_scale.get(), ki_scale.get(), kd_scale.get())

    # Safely close everything when the 'X' button is clicked
    def on_closing():
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    root.protocol("WM_DELETE_WINDOW", on_closing)

    tk.Label(root, text="Proportional (Kp) - The Muscle", font=("Arial", 10, "bold")).pack(anchor="w")
    kp_scale = tk.Scale(root, from_=0.0, to=20.0, resolution=0.1, orient="horizontal", command=on_slider_move)
    kp_scale.set(2.0)
    kp_scale.pack(fill="x", pady=(0, 15))

    tk.Label(root, text="Integral (Ki) - The Memory", font=("Arial", 10, "bold")).pack(anchor="w")
    ki_scale = tk.Scale(root, from_=0.0, to=5.0, resolution=0.01, orient="horizontal", command=on_slider_move)
    ki_scale.set(0.0)
    ki_scale.pack(fill="x", pady=(0, 15))

    tk.Label(root, text="Derivative (Kd) - The Shock Absorber", font=("Arial", 10, "bold")).pack(anchor="w")
    kd_scale = tk.Scale(root, from_=0.0, to=10.0, resolution=0.1, orient="horizontal", command=on_slider_move)
    kd_scale.set(1.0)
    kd_scale.pack(fill="x")

    # Periodic check to allow Ctrl+C in the terminal to kill the GUI
    def check_ros():
        if rclpy.ok():
            root.after(100, check_ros)
        else:
            root.destroy()

    root.after(100, check_ros)
    root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = PIDTuner()
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    try:
        start_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()