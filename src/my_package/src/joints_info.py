# This program creates a Window that has two purposes.
# Firstly, printing the values of the current joints in radian
# Secondly, you can enter the joints values you want the robot to have.
# This program has two main parts : one which creates the node and one which creates the interface. 


import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import tkinter as tk
import math
import numpy as np


class ROS2Node(Node):
    def __init__(self, gui_callback):  #Define the node
        super().__init__('tkinter_listener')
        self.gui_callback = gui_callback

        # Create the subscriber that will listen to the /lbr/joint_trajectory_controller/state topic which publishes the actual joints info
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/lbr/joint_trajectory_controller/state',
            self.listener_callback,
            10)

        # Create the publisher that will send a message on the /lbr/joint_trajectory_controller/joint_trajectory topic to impose joints positions
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/lbr/joint_trajectory_controller/joint_trajectory',
            10)
        
        # We will use a timer for the publisher
        self.timer = None
        self.publish_count = 0
        self.max_publish = 1
        self.joint_positions = []

    def listener_callback(self, msg): # Main function for the listener
        if msg.actual.positions:
            positions = msg.actual.positions # This will capture the actual position of the robot's joints
            formatted = '\n'.join([f"Joint {i+1} Position: {180/np.pi*pos:.4f}" for i, pos in enumerate(positions)])
            self.gui_callback(formatted)
        else:
            self.gui_callback("No joint positions available.")

    def send_joint_positions_once(self, joint_positions): # Main function for the publisher : Joint_positions are the positions the user will enter
        msg = JointTrajectory()
        msg.joint_names = [f'lbr_A{i+1}' for i in range(7)]
        point = JointTrajectoryPoint()
        point.positions = joint_positions # The positions become the one entered by the user
        point.time_from_start.sec = 2
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published once: {joint_positions}') 

    def start_timer_publish(self, joint_positions): # Timer definition
        self.joint_positions = joint_positions
        self.publish_count = 0
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(1.0, self.timer_callback)  

    def timer_callback(self): 
        if self.publish_count >= self.max_publish:
            self.get_logger().info("Done sending trajectory.")
            self.timer.cancel()
            return

        msg = JointTrajectory()
        msg.joint_names = [f'lbr_A{i+1}' for i in range(7)]
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start.sec = 5
        msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(f'Trajectory command sent (#{self.publish_count + 1})')
        self.publish_count += 1


class App:  # Now the interface
    def __init__(self, root):
        self.root = root # Main window
        self.root.title("Joint info and control")

        self.label = tk.Label(root, text="Waiting for joint states...")
        self.label.pack(padx=10, pady=10)

        self.entries = []
        for i in range(7):
            frame = tk.Frame(root)
            frame.pack(pady=2)
            tk.Label(frame, text=f"Joint {i+1}:").pack(side=tk.LEFT)
            # entry = tk.Entry(frame, width=10)
            # entry.pack(side=tk.LEFT)
            # self.entries.append(entry)
            entry = tk.Scale(frame, from_=-180, to=180, orient="horizontal", length=600)
            entry.pack()
            self.entries.append(entry)

        self.send_button = tk.Button(root, text="Send positions", command=self.send_positions)
        self.send_button.pack(pady=10)

        # ROS 2 init
        rclpy.init()
        self.node = ROS2Node(self.update_label)
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def update_label(self, text):
        self.label.after(0, self.label.config, {'text': text})

    def send_positions(self):
        try:
            joint_positions = [math.radians(float(entry.get())) for entry in self.entries]
            if len(joint_positions) != 7:
                raise ValueError("Enter 7 positions")
            self.node.start_timer_publish(joint_positions)
        except ValueError as e:
            self.update_label(f"Erreur : {e}")

    def ros_spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"ROS spin error: {e}")

    def on_close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("700x700")
    app = App(root)
    root.mainloop()
