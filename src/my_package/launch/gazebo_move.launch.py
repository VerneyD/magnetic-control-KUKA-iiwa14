# Gazebo Robot and coil and magnet in a box with rviz joint control, in simulation 

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


    launch_mock= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_package'),
                'launch',
                'gazebo_complete.launch.py'
            ])
        ]),
        launch_arguments={
            'model': 'iiwa14',
        }.items()
    )

    nodes.append(launch_mock)

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

    
    # end_effector_position_node = Node(
    #     package='my_package',
    #     executable='end_effector_position_node',
    # )
    # nodes.append(end_effector_position_node)



    # Retourner la LaunchDescription avec tous les noeuds et inclusions
    return LaunchDescription(nodes)
