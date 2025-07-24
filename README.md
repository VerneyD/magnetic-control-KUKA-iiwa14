This repository provides a complete control of the KUKA iiwa 14 collaborative robot.

KUKA IIWA14 R820 Control using ROS2
Verney DELHEZ – July 2025



Summary : 

Introduction 
Launching a basic world containing the robot
Plugins
Launch files
How to control the robot in a simulation 
How to launch the hardware control of the robot
Most common procedures step by step


I . Introduction

	This report is a guide useful to control the KUKA IIWA14 R820 robotic arm in various ways, including both a simulation and a hardware mode. In those modes, it is possible to move the robot in a few different ways : simply controlling the joints of the robot, changing the position of the end_effector using a user input, and using a Xbox controller to move the robot. 
	The robotic arm is a 7 joints robot, which means that there are 6 DOFs overall. 
	First of all, I created a workspace containing many packages, in which folders contain files. Each package has its own purpose. For example, the package lbr_description contains the files that describe the robot, for example the meshes and sdf files. The package lbr_bringup contains the basic launch files used to start controlling the robot. I created a few of my own packages containing the files I created and named them my_package, lbr_gazebo_models, and test_package. Each one of them has its own purpose : my_package is the main package I created, containing the customized launch files used; test_package contains some plugins useful to control the Gazebo simulation, and lbr_gazebo_models contains the models used in this Gazebo simulation. 
	my_package contains a few folders, but only two of them are really important : launch and src. The launch folder contains my launch files and src contains the raw code used to control the robot – for instance, the node used to control the robot with a Xbox controller, and a node used to get the end effector position in real time. 


	
II. Launching a basic world containing the robot

	To launch a basic world containing the robot, the capsule and the box containing the capsule, start a terminal, type
cd lbr-stack  
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/.gazebo/models
ign gazebo ~/lbr-stack/src/lbr_gazebo_models/worlds/test_capsule.world

Do not forget to press the start button in the Gazebo window, otherwise nothing will happen. 
In general, to launch a world in Gazebo, you need to type ign gazebo ~/path_to_the_world .
The world file needs to contain information about the objects you want to make spawn in the world, for example the capsule or the box. Those can be called as sdf files as it s the case in test_capsule.world.
This test_capsule.world is the main Gazebo world used for the magnet simulation : it is the world I created to mirror the real robot in real time. To make that happen, you need to launch either a simulation or the hardware -which is detailed later in this report, and this world in which you press the start button.

 III.Plugins
	
	I see plugins as laws that apply to objects. A plugin is a program -in our case, in the package test_package, that describes a motion/force of an object. I created two plugins, named my_box_ign_plugin.cpp and robot_movement_plugin.cpp. 
	my_box_ign_plugin.cpp  is the one that describes the forces applied to the capsule in the Gazebo world. It applies the magnetic force, drag force, buoyency force… and thus creates the capsule movement. This plugin has two arguments : the magnet positioned at the end effector, creating a magnetic field, and the capsule, reacting to this field. The plugin needs to know the position and orientation of both objects to calculate the applied magnetic field. 
	 robot_movement_plugin.cpp is the plugin that will make the Gazebo robot mirroring the real robot (but it also works when you launch only a simulation). Basically, it subscribes to some information published by the robot, to the topic /joint_states to be precise, and orders the Gazebo robot to move depending on those information. Whenever the hardware robot moves, it publishes its  joints speed and the Gazebo robot will do the same motion, following the exact same speed for each joint. We added a controller to make it more realistic,. 

	To call a plugin, you need to write these lines of code in the sdf files of the objects you want to apply it to: for example, in /home/srl/.gazebo/models/magnetic_capsule/model.sdf (sdf of the capsule) : 
<plugin name="my_plugins::MyBoxIgnPlugin" filename="/home/srl/lbr-stack/install/test_package/lib/libmy_box_ign_plugin.so">
	<target_link1>lbr_link_7</target_link1>
	<target_link2>magnetic_capsule</target_link2>
</plugin>

In our case, the plugin is applied to the capsule and the end_effector of the robot (lbr_link7) – and not the magnet attached to it, so the plugin itself contains lines that include the translation (the result of attaching the magnet at the end effector). 

IV. Launch files
	
	Launch files are used to control the robot. They are files that contains different command lines that call other programs -or even other launch files. To use a launch file, type in the terminal : 
cd lbr-stack 
source install/setup.bash
ros2 launch package_name launch_file
for instance ros2 launch my_package my.launch_rviz.launch.py. This launch files will launch the simulation of the robot, and you will be able to control the joints values using the Rviz interface. 
	I created a GUI to make it easier to launch those launch files. To use the GUI : run the gui.py file in VisualStudioCode (not in terminal) - /home/srl/lbr-stack/src/my_package/src/gui.py. This window will pop :


Each button is linked to a launch file. In the next parts of the guide, I will often refer to these buttons. 

V. How to control the robot with the simulation

	Press the button ‘Simulation control of the joints with Moveit‘. A Rviz window should open. Go to the joint section, choose the values and then press Plan&Execute in the Planning section. The robot should move in the window. As previously mentioned, if you run 
ign gazebo ~/lbr-stack/src/lbr_gazebo_models/worlds/test_capsule.world
in a terminal at the same time, the robot will also move in Gazebo. 

VI. How to launch the hardware control of the robot

	To use the robot, connect the Ethernet cable of the robot to the Laptop, and turn on the robot. Use the SmartPad and press ‘Applications’ and ‘LBRServer’ which might take a few seconds to show up. Them press the physical play button – not on the screen, on the SmartPad (On the left row) and then select the following :
- FRI send period: 2 ms
- IP address: 172.31.1.148
- FRI control mode: POSITION_CONTROL
- FRI client command mode: JOINT_POSITION

	This will start establishing a connection between the robot and the laptop. You have 10 seconds to press any button that will launch a file that refers to the hardware. You should here a clicking sound coming from the robot once if the connection is successful. 
Here is a brief description of all the possible controls of the hardware (the buttons of the GUI above): 
- Hardware control of the joints with Moveit : control the joints with the Rviz interface.
- Hardware control with the Xbox controller : control the end-effector’s position with a controller -plugged in the laptop using a USB cable. Sometimes it does not work perfectly because of the cable that can disconnect sometimes… so to see exactly what happens, you can type ros2 topic echo /joy in a terminal. You will see the data published by the controller, they should not be zero if you press a button or move the joystick : if it is zero there is a connection error between the laptop and the controller. I think you will easily find which button controls what direction, in case you need the details you can look at the file /home/srl/lbr-stack/src/my_package/src/command_publisher3.cpp which might help you.

- Reset hardware position : This button needs to be used with another one which is for instance ‘Hardware control of the joints with Moveit’. That is because what this launch file does is, it sends joint values to the robot, which is then going to move to this state. However, this launch file does not launch the programs that make the robot move, so if you launch it alone, it is not going to do anything except sending values in the void. Launching ‘Hardware control of the joints with Moveit’ will launch the nodes that make the robot move. 
You can also change the reset position of the robot which is defined in this file : /home/srl/lbr-stack/src/my_package/src/reset_position.cpp, line 58. 

- Hardware setup to start controlling position with position_control_gui : This button will start a position control of the end-effector of the robot. In order for it to work properly, run the /home/srl/lbr-stack/src/my_package/src/position_control_gui.py at the same time (in the terminal, type code /home/srl/lbr-stack/src/my_package/src/position_control_gui.py and it will bring you to the file in VSC, then press the run button). You should see the following window pop up :
 And the first three coordinates should not be zero if you pressed the ‘ Hardware setup to start controlling position with position_control_gui’ button first. You can then use the sliders to get to a particular position. 
If the robot needs to go through a singularity, it will simply stop and you can close the window and launch it again to go away from the singularity. There are three other sliders at the bottom of the window so that if you have a local reset position, you can access it easier by putting the values in those sliders. 
You can also change the speed of the effector. The movement is usually very precise (millimeter scale) but if you increase the speed a lot it might sometimes be going one or two millimeters further than your input) but I think it is pretty rare. 

- Get robot info when controlling with the controller : I did not use this button a lot, but it will print the values of the joints in real time which might be useful when you control the position of the end-effector, either with the Xbox controller or simply with the interface above.

- Reset all nodes : Very important button. Every time you are finished with controlling the robot, press this button. If you want to control its joints and then the position of the effector, you need to press this button in between, otherwise it will not work because the nodes will interfere.
It will also cut the connection between the robot and the laptop (with the clicking sound). Make sure to do the procedure to connect the robot in between again, using the procedure described at the start of VI. Every time you here the clicking sound meaning that the robot disconnects, you have to do the procedure again to use the robot. This implies that every time you use the ‘Reset all nodes button’, you need to do it afterwards. 
	
	
	General advice : if some launch file is not working, try to restart the laptop and it usually fixes the issue because this issues mainly comes from nodes interfering. 

	
VII. Most common procedures step by step

	This part is a description of what you will have to do when using the magnet and the capsule. 
	First, to have both the simulation and Gazebo running at the same time, you will have to :
- open a terminal
- type ign gazebo ~/lbr-stack/src/lbr_gazebo_models/worlds/test_capsule.world
- Press the play button at the bottom left corner of the Gazebo window
- type in another terminal :
	cd lbr-stack
	source install/setup.bash
	ros2 launch my_package my_launch_rviz.launch.py

	Then, to have both the hardware robot and Gazebo running at the same time : 
- open a terminal
- type export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/.gazebo/models
ign gazebo ~/lbr-stack/src/lbr_gazebo_models/worlds/test_capsule.world

- Press the play button at the bottom left corner of the Gazebo window
- run the gui.py file in VisualStudioCode (like described previously)
- turn on the robot
- on the SmartPad select Application: LBRServer
- Press the play button on the SmartPad and select the following parameters :
- FRI send period: 2 ms
- IP address: 172.31.1.148
- FRI control mode: POSITION_CONTROL
- FRI client command mode: JOINT_POSITION
- click on the button Hardware setup to start controlling position with position_control_gui OR Hardware control of the joints with Moveit depending on what you want to control
- Now you can make the robot move as you wish and Gazebo will mirror its movement.
