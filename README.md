# **KUKA IIWA14 R820 Control using ROS2**  
**Author:** Verney DELHEZ – July 2025  

---

## **Summary**

- [Introduction](#i-introduction)  
- [Launching a basic world containing the robot](#ii-launching-a-basic-world-containing-the-robot)  
- [Plugins](#iii-plugins)  
- [Launch files](#iv-launch-files)  
- [How to control the robot in a simulation](#v-how-to-control-the-robot-with-the-simulation)  
- [How to launch the hardware control of the robot](#vi-how-to-launch-the-hardware-control-of-the-robot)  
- [Most common procedures step by step](#vii-most-common-procedures-step-by-step)  

---

## **I. Introduction**

This report is a guide useful to control the **KUKA IIWA14 R820** robotic arm in various ways, including both a simulation and a hardware mode. In those modes, it is possible to move the robot in a few different ways: simply controlling the joints of the robot, changing the position of the end_effector using a user input, and using a Xbox controller to move the robot.  

The robotic arm is a 7 joints robot, which means that there are 6 DOFs overall.  

First of all, I created a workspace containing many packages, in which folders contain files. Each package has its own purpose. For example:  
- `lbr_description` contains the files that describe the robot, for example the meshes and sdf files.  
- `lbr_bringup` contains the basic launch files used to start controlling the robot.  
- Custom packages:  
  - `my_package` → main package I created, containing customized launch files.  
  - `test_package` → contains plugins for Gazebo.  
  - `lbr_gazebo_models` → contains the models used in the Gazebo simulation.  

`my_package` contains a few folders, but only two are important:  
- `launch/` → launch files  
- `src/` → source code used to control the robot (Xbox controller node, end-effector position node, etc.)  

---

## **II. Launching a basic world containing the robot**

To launch a basic world containing the robot, the capsule and the box containing the capsule:  

```bash
cd lbr-stack  
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/.gazebo/models
ign gazebo ~/lbr-stack/src/lbr_gazebo_models/worlds/test_capsule.world
