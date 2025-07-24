# This program gives the position of the end effector and needs to be launched with other programs
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

    # Noeud pour envoyer des commandes
    controller_node = Node(
        package='my_package',
        executable='robot_info_controller',
    )

    end_effector_position_node = Node(
        package='my_package',
        executable='end_effector_position_node',
    )
    nodes.append(end_effector_position_node)

    # Description du robot
    # robot_description = LBRDescriptionMixin.param_robot_description(mode="mock")
    nodes.append(controller_node)

    # Noeud pour envoyer des commandes
    joint_node = Node(
        package='my_package',
        executable='robot_info',
    )

    # Description du robot
    # robot_description = LBRDescriptionMixin.param_robot_description(mode="mock")
    nodes.append(joint_node)
    

    # Retourner la LaunchDescription avec tous les noeuds et inclusions
    return LaunchDescription(nodes)
