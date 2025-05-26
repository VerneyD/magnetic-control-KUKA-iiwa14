import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import threading
import tkinter as tk

class ROS2Node(Node):
    def __init__(self, gui_callback):
        super().__init__('tkinter_listener')
        self.gui_callback = gui_callback
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/lbr/joint_trajectory_controller/state',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.actual.positions:
            position1 = msg.actual.positions[0]
            position2 = msg.actual.positions[1]
            position3 = msg.actual.positions[2]
            position4 = msg.actual.positions[3]
            position5 = msg.actual.positions[4]
            position6 = msg.actual.positions[5]
            position7 = msg.actual.positions[6]
            self.get_logger().info(f'Joint 1 Position: {position1:.4f}, Joint 2 Position: {position2:.4f}, Joint 3 Position: {position3:.4f}, Joint 4 Position: {position4:.4f}, Joint 5 Position: {position5:.4f}, Joint 6 Position: {position6:.4f}, Joint 7 Position: {position7:.4f} ')
            self.gui_callback(f"Joint 1 Position: {position1:.4f} \n Joint 2 Position: {position2:.4f} \n Joint 3 Position: {position3:.4f}\n Joint 4 Position: {position4:.4f}\n Joint 5 Position: {position5:.4f}\n Joint 6 Position: {position6:.4f}\n Joint 7 Position: {position7:.4f}")
        else:
            self.get_logger().warn('No joint positions received')
            self.gui_callback("No joint positions available.")

class App:
    def __init__(self, root):
        self.root = root
        self.label = tk.Label(root, text="Waiting for message...")
        self.label.pack(padx=20, pady=20)

        # Initialize ROS 2 and start node thread
        rclpy.init()
        self.node = ROS2Node(self.update_label)
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Handle GUI close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def update_label(self, text):
        self.label.after(0, self.label.config, {'text': text})

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
    root.title("ROS 2 Tkinter GUI")
    app = App(root)
    root.mainloop()
