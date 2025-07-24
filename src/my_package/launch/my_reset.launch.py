# This program, launched with my.launch.py, will reset the robot's joints values to a certain values defined in reset_position.cpp
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

    launch_reset = Node(
        package='my_package',
        executable='reset_position',
    )


    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'true',
            'mock' : 'true',
            'model': 'iiwa14'
        }.items()
    )

    nodes.append(launch_simulation)

    
    launch_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lbr_bringup'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'model': 'iiwa14'
        }.items()

    )
    
    end_effector_position_node = Node(
        package='my_package',
        executable='end_effector_position_node',
    )
    nodes.append(end_effector_position_node)

    nodes.append(launch_hardware)

    nodes.append(launch_reset)

    
    

    # Retourner la LaunchDescription avec tous les noeuds et inclusions
    return LaunchDescription(nodes)
