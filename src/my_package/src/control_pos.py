import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TwistStamped
import tkinter as tk
import threading
import math


class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control_node')

        # Publisher pour publier les positions de l'effecteur final
        self.publisher = self.create_publisher(TwistStamped, '/lbr/servo_node/delta_twist_cmds', 10)
        self.position_publisher = self.create_publisher(PointStamped, '/end_effector_position', 10)

        # Abonnement à la position de l'effecteur final
        self.subscription = self.create_subscription(
            PointStamped,
            '/end_effector_position',
            self.position_callback,
            10
        )

        # Variables
        self.current_position = PointStamped()
        self.final_position = PointStamped()
        self.final_position.point.x = 0.5  # Position cible par défaut
        self.final_position.point.y = 0.5
        self.final_position.point.z = 0.5
        self.time_to_reach = 5.0  # Temps de parcours (en secondes)

    def position_callback(self, msg):
        """Gestion des mises à jour de la position de l'effecteur final."""
        self.current_position = msg

    def send_position_command(self, x, y, z):
        """Envoyer une commande pour l'effecteur final."""
        self.final_position.point.x = x
        self.final_position.point.y = y
        self.final_position.point.z = z

        # Publier la position cible
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now()
        msg.header.frame_id = "base_link"
        msg.point = self.final_position.point
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Published target position: {x}, {y}, {z}")

    def publish_command(self):
        """Publier les commandes pour déplacer l'effecteur final."""
        # Calcul de la distance restante à parcourir
        dx = self.final_position.point.x - self.current_position.point.x
        dy = self.final_position.point.y - self.current_position.point.y
        dz = self.final_position.point.z - self.current_position.point.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        tolerance = 0.005  # Tolérance en mètres (5 mm)

        if distance < tolerance:
            self.get_logger().info("Target reached, stopping.")
            return

        # Calcul de la commande Twist pour le mouvement
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = dx / (self.time_to_reach * 30)  # Diviser par le temps
        twist_msg.twist.linear.y = dy / (self.time_to_reach * 30)
        twist_msg.twist.linear.z = dz / (self.time_to_reach * 30)

        # Publier la commande
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Publishing movement command: {twist_msg.twist.linear.x}, {twist_msg.twist.linear.y}, {twist_msg.twist.linear.z}")


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("End Effector Position Control")

        self.label = tk.Label(root, text="Enter target positions (x, y, z):")
        self.label.pack(padx=10, pady=10)

        self.entries = []
        for i in range(3):  # Nous envoyons x, y, z
            frame = tk.Frame(root)
            frame.pack(pady=2)
            tk.Label(frame, text=f"Coordonnée {i+1}:").pack(side=tk.LEFT)
            entry = tk.Scale(frame, from_=0, to=1, orient="horizontal", length=600)
            entry.pack()
            self.entries.append(entry)

        self.send_button = tk.Button(root, text="Send Target Position", command=self.send_target_position)
        self.send_button.pack(pady=10)

        # ROS2 Init
        rclpy.init()
        self.node = PositionControlNode()
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def send_target_position(self):
        try:
            # Récupérer les valeurs du slider (de 0 à 1)
            x = self.entries[0].get()
            y = self.entries[1].get()
            z = self.entries[2].get()

            # Publier les nouvelles positions cibles
            self.node.send_position_command(x, y, z)

        except ValueError as e:
            self.update_label(f"Error: {e}")

    def ros_spin(self):
        """ROS spin loop to process incoming messages."""
        try:
            while rclpy.ok():
                self.node.publish_command()  # Continuer à publier les commandes de mouvement
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"ROS spin error: {e}")

    def on_close(self):
        """Fermer le programme ROS proprement."""
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("700x700")
    app = App(root)
    root.mainloop()
