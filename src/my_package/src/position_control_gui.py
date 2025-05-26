import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PointStamped
import tkinter as tk
from tkinter import ttk
import math
import threading
import time

class PositionControl(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.subscription = self.create_subscription(
            PointStamped,
            '/end_effector_position',
            self.msg_callback,
            10
        )
        self.publisher = self.create_publisher(TwistStamped, '/lbr/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_command)

        self.current_position = None
        self.target_position = None
        self.speed = 0.5
        self.speed_initialized = False
        self.target_reached = False
        self.command_active = False
        self.position_callback = None
        self.gui_ready = False
        self.movement_finished = True  # On considère au départ qu'il n'y a pas de mouvement en cours

    def set_gui_callback(self, callback):
        self.position_callback = callback

    def set_gui_sliders(self, set_slider_values_func):
        self.set_slider_values_func = set_slider_values_func

    def update_target(self, x, y, z, speed):
        self.target_position = {'x': x, 'y': y, 'z': z}
        self.speed = speed
        self.speed_initialized = False
        self.target_reached = False
        self.command_active = True
        self.movement_finished = False  # Nouveau mouvement commence, donc pas fini
        self.get_logger().info(f"Target updated: x={x}, y={y}, z={z}, speed={speed}")

    def is_movement_finished(self):
        return self.movement_finished

    def msg_callback(self, msg):
        self.current_position = msg.point

        if self.position_callback:
            self.position_callback(self.current_position.x, self.current_position.y, self.current_position.z)

        # Set initial sliders once
        if not self.gui_ready and hasattr(self, 'set_slider_values_func'):
            self.set_slider_values_func(self.current_position.x, self.current_position.y, self.current_position.z)
            self.gui_ready = True

        if self.command_active and not self.speed_initialized:
            dx = self.target_position['x'] - self.current_position.x
            dy = self.target_position['y'] - self.current_position.y
            dz = self.target_position['z'] - self.current_position.z

            self.speed_x = math.copysign(self.speed, dx)
            self.speed_y = math.copysign(self.speed, dy)
            self.speed_z = math.copysign(self.speed, dz)

            self.speed_initialized = True

    def publish_command(self):
        if not self.command_active or self.target_reached or self.current_position is None:
            return

        dx = self.target_position['x'] - self.current_position.x
        dy = self.target_position['y'] - self.current_position.y
        dz = self.target_position['z'] - self.current_position.z

        norm = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        if norm < 0.003:  # seuil d’arrêt global
            self.target_reached = True
            self.movement_finished = True
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = "lbr_link_0"
            self.publisher.publish(stop_msg)
            self.get_logger().info("Target reached. Stopping.")
            return

        # Normalisation du vecteur direction → synchronisation des axes
        unit_x = dx / norm
        unit_y = dy / norm
        unit_z = dz / norm

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "lbr_link_0"

        twist_msg.twist.linear.x = unit_x * self.speed
        twist_msg.twist.linear.y = unit_y * self.speed
        twist_msg.twist.linear.z = unit_z * self.speed

        self.publisher.publish(twist_msg)


def start_ros_node(control_node):
    rclpy.spin(control_node)

def create_gui(control_node):
    root = tk.Tk()
    root.title("Position Controller")

    sliders = {}

    for axis in ['x', 'y', 'z']:
        frame = ttk.Frame(root)
        frame.pack(padx=10, pady=5)
        label = ttk.Label(frame, text=f"{axis.upper()} in m:")
        label.pack(side=tk.LEFT, padx=100, pady=5)
        slider = tk.Scale(frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, length=600, showvalue=True)
        slider.set(0.0)  # Temp value, will be updated when first position received
        slider.pack(side=tk.LEFT)
        sliders[axis] = slider

    speed_frame = ttk.Frame(root)
    speed_frame.pack(padx=10, pady=5)
    speed_label = ttk.Label(speed_frame, text="Speed in cm/s:")
    speed_label.pack(side=tk.LEFT, padx=75, pady=5)
    speed_slider = tk.Scale(speed_frame, from_=0.1, to=3.33, orient=tk.HORIZONTAL, resolution=0.01, length=600, showvalue=True)
    speed_slider.set(0.5)
    speed_slider.pack(side=tk.LEFT)

    

    def set_slider_values(x, y, z):
        root.after(0, lambda: sliders['x'].set(x))
        root.after(0, lambda: sliders['y'].set(y))
        root.after(0, lambda: sliders['z'].set(z))

    control_node.set_slider_values_func = set_slider_values

    start_time = [None]  # liste mutable pour pouvoir modifier dans send_target et update_position_label

    def send_target():
        x = sliders['x'].get()
        y = sliders['y'].get()
        z = sliders['z'].get()
        speed = 0.3*speed_slider.get()
        control_node.update_target(x, y, z, speed)
        start_time[0] = time.time()  # mémoriser l'heure de départ

    def send__local_target():
        x = slider_x_local.get()
        y = slider_y_local.get()
        z = slider_z_local.get()
        speed = 0.3*speed_slider.get()
        control_node.update_target(x, y, z, speed)
        #start_time[0] = time.time()  # mémoriser l'heure de départ

    send_button = ttk.Button(root, text="Set Target", command=send_target)
    send_button.pack(pady=10)


    time_label = ttk.Label(root, text="Movement time: 0.000 s", font=("Arial", 12))
    time_label.pack(pady=10)

    position_label = ttk.Label(root, text="Current Position: x=---, y=---, z=---", font=("Arial", 12))
    position_label.pack(pady=10)

    def update_position_label(x, y, z):
        def gui_update():
            position_label.config(text=f"Current Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if start_time[0] is not None:
                if not control_node.is_movement_finished():
                    elapsed = time.time() - start_time[0]
                    time_label.config(text=f"Movement time: {elapsed:.3f} s")
                else:
                    # Mouvement fini, on stoppe la mise à jour (tu peux aussi figer la valeur)
                    pass

        root.after(0, gui_update)

    local_x_frame = ttk.Frame(root)
    local_x_frame.pack(padx=10, pady=5)
    local_x_label = ttk.Label(local_x_frame, text=f"Local reset x coordinate:")
    local_x_label.pack(side=tk.LEFT, padx=35, pady=5)
    slider_x_local = tk.Scale(local_x_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, length=600, showvalue=True)
    slider_x_local.set(0.0)  # Temp value, will be updated when first position received
    slider_x_local.pack(side=tk.LEFT)

    local_y_frame = ttk.Frame(root)
    local_y_frame.pack(padx=10, pady=5)
    local_y_label = ttk.Label(local_y_frame, text=f"Local reset y coordinate:")
    local_y_label.pack(side=tk.LEFT, padx=35, pady=5)
    slider_y_local = tk.Scale(local_y_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, length=600, showvalue=True)
    slider_y_local.set(0.0)  # Temp value, will be updated when first position received
    slider_y_local.pack(side=tk.LEFT)
    
    local_z_frame = ttk.Frame(root)
    local_z_frame.pack(padx=10, pady=5)
    local_z_label = ttk.Label(local_z_frame, text=f"Local reset z coordinate:")
    local_z_label.pack(side=tk.LEFT, padx=35, pady=5)
    slider_z_local = tk.Scale(local_z_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, resolution=0.01, length=600, showvalue=True)
    slider_z_local.set(0.0)  # Temp value, will be updated when first position received
    slider_z_local.pack(side=tk.LEFT)

    send_local_button =  ttk.Button(root, text="Set Local Target", command=send__local_target)
    send_local_button.pack(pady=10)

    control_node.set_gui_callback(update_position_label)
    control_node.set_gui_sliders(set_slider_values)

    root.mainloop()

def main():
    rclpy.init()
    control_node = PositionControl()

    ros_thread = threading.Thread(target=start_ros_node, args=(control_node,), daemon=True)
    ros_thread.start()

    create_gui(control_node)

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
