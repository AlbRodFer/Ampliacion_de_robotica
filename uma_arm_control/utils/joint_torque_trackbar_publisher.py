#!/usr/bin/python3


#   Author: Juan M. Gandarias (http://jmgandarias.com)
#   email: jmgandarias@uma.es
# 
#   This script creates trackbars for joint_torques (tau 1 and tau 2). 
#   Run it:
#   python3 joint_torque_trackbar_publisher.py


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk

class JointTorquePublisher(Node):
    def __init__(self):
        super().__init__('joint_torque_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_torques', 10)
        self.tau_joint1 = 0.0
        self.tau_joint2 = 0.0
        self.continuous_mode = True

        self.root = tk.Tk()
        self.root.title("Joint Torque Publisher")

        self.create_trackbar('Torque 1', -5.0, 5.0, self.update_tau_joint1)
        self.create_trackbar('Torque 2', -5.0, 5.0, self.update_tau_joint2)

        self.mode_button = tk.Button(self.root, text="Switch to Instantaneous Mode", command=self.switch_mode)
        self.mode_button.pack()

        center_button = tk.Button(self.root, text="Center All", command=self.center_all)
        center_button.pack()

    def create_trackbar(self, label, min_val, max_val, callback):
        frame = tk.Frame(self.root)
        frame.pack()
        tk.Label(frame, text=f"{min_val}").pack(side=tk.LEFT)
        trackbar = tk.Scale(frame, label=label, from_=min_val, to=max_val, orient=tk.HORIZONTAL, command=callback, resolution=0.1, length=400, sliderlength=30)
        trackbar.pack(side=tk.LEFT)
        trackbar.bind("<ButtonRelease-1>", lambda event, lbl=label: self.reset_slider(lbl))
        tk.Label(frame, text=f"{max_val}").pack(side=tk.LEFT)

    def update_tau_joint1(self, value):
        self.tau_joint1 = float(value)
        self.publish_joint_torques()

    def update_tau_joint2(self, value):
        self.tau_joint2 = float(value)
        self.publish_joint_torques()

    def reset_slider(self, label):
        if not self.continuous_mode:
            for widget in self.root.winfo_children():
                if isinstance(widget, tk.Frame):
                    for child in widget.winfo_children():
                        if isinstance(child, tk.Scale) and child.cget("label") == label:
                            child.set(0.0)

    def switch_mode(self):
        self.continuous_mode = not self.continuous_mode
        mode_text = "Switch to Continuous Mode" if not self.continuous_mode else "Switch to Instantaneous Mode"
        self.mode_button.config(text=mode_text)

    def center_all(self):
        for widget in self.root.winfo_children():
            if isinstance(widget, tk.Frame):
                for child in widget.winfo_children():
                    if isinstance(child, tk.Scale):
                        child.set(0.0)

    def publish_joint_torques(self):
        msg = Float64MultiArray()
        msg.data = [self.tau_joint1, self.tau_joint2]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Torque 1 = {self.tau_joint1}, Torque 2 = {self.tau_joint2}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = JointTorquePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()