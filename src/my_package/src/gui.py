import tkinter as tk
import subprocess
import threading
import psutil



# Initialisation de tkinter
root = tk.Tk()
root.title("Kuka Arm Control")
root.geometry("1900x1000")

# Description frane

description_frame = tk.Frame(root)
description_frame.pack(side="top", padx=10, pady=10)

description_label = tk.Label(description_frame, text="This GUI is designed to control the KUKA iiwa 14 collaborative robot. \n You can control the robot either by entering joints positions (using Moveit) or control the end effector position by using a Xbox controller. \n Make sure to source before launching any file : in the terminal, type source install/setup.bash and source /opt/ros/humble/setup.bash", font=("Arial", 12))
description_label.pack()
# Frame des boutons
button_frame = tk.Frame(root)
button_frame.pack(side="left", padx=10, pady=10)

# Frame des terminaux
terminal_frame = tk.Frame(root)
terminal_frame.pack(side="right", padx=10, pady=10, fill="both", expand=True)

# ===== Terminal général (haut) =====
general_output_frame = tk.Frame(terminal_frame)
general_output_frame.pack(side=tk.TOP, fill="both", expand=True)

general_label = tk.Label(general_output_frame, text="ROS2 General Output", font=("Arial", 12, "bold"))
general_label.pack()

terminal_output = tk.Text(general_output_frame, height=15, width=80)
terminal_output.pack(side=tk.LEFT, expand=True, fill="both")

general_scrollbar = tk.Scrollbar(general_output_frame)
general_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

terminal_output.config(yscrollcommand=general_scrollbar.set)
general_scrollbar.config(command=terminal_output.yview)

# ===== Terminal robot_info (bas) =====
robot_info_frame = tk.Frame(terminal_frame)
robot_info_frame.pack(side=tk.BOTTOM, fill="both", expand=True)

robot_info_label = tk.Label(robot_info_frame, text="Robot Info Output (when using the controller)", font=("Arial", 12, "bold"))
robot_info_label.pack()

robot_info_output = tk.Text(robot_info_frame, height=15, width=80)
robot_info_output.pack(side=tk.LEFT, expand=True, fill="both")

robot_info_scrollbar = tk.Scrollbar(robot_info_frame)
robot_info_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

robot_info_output.config(yscrollcommand=robot_info_scrollbar.set)
robot_info_scrollbar.config(command=robot_info_output.yview)

# Commandes ROS2
LAUNCH_COMMAND_CONTROLLER = ["ros2", "launch", "my_package", "my_launch3.launch.py"]
LAUNCH_COMMAND_JOINTS = ["ros2", "launch", "my_package", "my.launch.py"]
LAUNCH_COMMAND_SIMULATION = ["ros2", "launch", "my_package", "my_launch_rviz.launch.py"]
RESET_POSITION = ["ros2", "launch", "my_package", "my_reset.launch.py"]
ROBOT_INFO = ["ros2", "launch", "my_package", "my_robot_info.launch.py"]
POSITION_CONTROL = ["ros2", "launch", "my_package", "my_position.launch.py"]

# Liste des processus lancés
processes = []

# Fonction générale pour les autres commandes (hors robot info)
def run_ros_command(command):
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        processes.append(process)

        def read_output():
            for line in iter(process.stdout.readline, ''):
                terminal_output.insert(tk.END, line)
                terminal_output.yview(tk.END)
            process.stdout.close()

        threading.Thread(target=read_output, daemon=True).start()

    except Exception as e:
        terminal_output.insert(tk.END, f"Error: {e}\n")
        terminal_output.yview(tk.END)

# Fonction spécifique pour robot info
def run_robot_info():
    try:
        process = subprocess.Popen(
            ROBOT_INFO,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        processes.append(process)

        def read_output():
            for line in iter(process.stdout.readline, ''):
                robot_info_output.insert(tk.END, line)
                robot_info_output.yview(tk.END)
            process.stdout.close()

        threading.Thread(target=read_output, daemon=True).start()

    except Exception as e:
        robot_info_output.insert(tk.END, f"Error: {e}\n")
        robot_info_output.yview(tk.END)

left_info_label2 = tk.Label(button_frame, text="You can run the joints_info.py file \n to see the joints information and also select joints values.\n The file can also be used to define a local reset position.", font=("Arial", 12))
left_info_label2.pack()
 
simulation_button = tk.Button(button_frame, text="Simulation control of the joints with Moveit", command=lambda: run_ros_command(LAUNCH_COMMAND_SIMULATION))
simulation_button.pack(pady=20)

left_info_label = tk.Label(button_frame, text="Before selecting Hardware control : \n run the LBR Server application on the Kuka SmartPad and select :\n - FRI send period: 2 ms \n - IP address: 172.31.1.148 \n - FRI control mode: POSITION_CONTROL\n  - FRI client command mode: JOINT_POSITION", font=("Arial", 12))
left_info_label.pack()


joints_button = tk.Button(button_frame, text="Hardware control of the joints with Moveit", command=lambda: run_ros_command(LAUNCH_COMMAND_JOINTS))
joints_button.pack(pady=20)

controller_button = tk.Button(button_frame, text="Hardware control with the XBox controller", command=lambda: run_ros_command(LAUNCH_COMMAND_CONTROLLER))
controller_button.pack(pady=20)

reset_button = tk.Button(button_frame, text="Reset hardware position", command=lambda: run_ros_command(RESET_POSITION))
reset_button.pack(pady=20)

position_button = tk.Button(button_frame, text="Hardware setup to start controlling position with position_control_gui", command=lambda: run_ros_command(POSITION_CONTROL))
position_button.pack(pady=20)

robot_info_button = tk.Button(button_frame, text="Get robot info when controlling with the controller", command=run_robot_info)
robot_info_button.pack(pady=20)


# robot_info_2_button = tk.Button(button_frame, text="Get robot info (node)", command=lambda: run_ros_command(ROBOT_INFO))
# robot_info_2_button.pack(pady=20)

# Fonction de réinitialisation des processus ROS2
def reset_nodes():
    try:
        for process in processes:
            try:
                process.terminate()
                process.wait()
            except Exception as e:
                terminal_output.insert(tk.END, f"Error terminating process: {e}\n")
        processes.clear()

        # Terminer tous les processus contenant "ros2"
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if "ros2" in proc.info['name']:
                    proc.terminate()
                    proc.wait()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

        terminal_output.insert(tk.END, "All ROS2 nodes terminated.\n")
        terminal_output.yview(tk.END)

    except Exception as e:
        terminal_output.insert(tk.END, f"Error resetting nodes: {e}\n")
        terminal_output.yview(tk.END)

reset_nodes_button = tk.Button(button_frame, text="Reset all nodes", command=reset_nodes)
reset_nodes_button.pack(pady=20)

# Lancement de l'application
root.mainloop()