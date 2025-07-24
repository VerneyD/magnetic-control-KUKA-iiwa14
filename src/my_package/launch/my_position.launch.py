# Launch this to control the position of the hardware robot. Launch position_control_gui.py as well to have a nice interface to control the position and the speed of the robot.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from lbr_bringup.description import LBRDescriptionMixin
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Liste des noeuds à lancer
    nodes = []


    launch_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'moveit_servo.launch.py'
            ])
        ]), 
        launch_arguments={
            'model': 'iiwa14',
            'mode' : 'mock'
        }.items()
    )

    nodes.append(launch_servo)




    launch_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'model': 'iiwa14',
            'ctrl':'forward_position_controller'
        }.items()

    )

    nodes.append(launch_hardware)
    
    end_effector_position_node = Node(
        package='my_package',
        executable='end_effector_position_node',
    )
    nodes.append(end_effector_position_node)


    launch_rviz= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'rviz.launch.py'
            ])
        ])

    )

    nodes.append(launch_rviz)


    # Retourner la LaunchDescription avec tous les noeuds et inclusions
    return LaunchDescription(nodes)
