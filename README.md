# **KUKA iiwa14 R820 Complete Control**

This repository provides **full control of the KUKA iiwa14 R820 robotic arm**.  
Both **joint control** and **position control** are supported, for:

- **Simulation** (Gazebo / Ignition)
- **Hardware robot**
- **Hybrid control** (simulation + hardware)

Additionally, this repository enables the **control of a magnetic capsule** inside a maze, by using a **magnet attached to the robot’s end-effector**.

---

## **Dependencies**

To make this repository work, you need to **clone and include the following packages** in their respective folders:

- **gz_ros2_control**  
  [https://github.com/ros-controls/gz_ros2_control.git](https://github.com/ros-controls/gz_ros2_control.git)

- **ros_gz package**  
  [https://github.com/gazebosim/ros_gz.git](https://github.com/gazebosim/ros_gz.git)

- **fri**  
  [https://github.com/lbr-stack/fri.git](https://github.com/lbr-stack/fri.git)

- **lbr_fri_idl**  
  [https://github.com/lbr-stack/lbr_fri_idl.git](https://github.com/lbr-stack/lbr_fri_idl.git)

- **lbr_fri_ros2_stack**  
  [https://github.com/lbr-stack/lbr_fri_ros2_stack.git](https://github.com/lbr-stack/lbr_fri_ros2_stack.git)

---

### ✅ After cloning, make sure to:
- Place the packages in the **correct folders** of your workspace.
- Run `colcon build` and source the `install/setup.bash`.

