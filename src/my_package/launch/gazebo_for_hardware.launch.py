# Intented to work with hardware_for_gazebo and move_group_hw in order to control the real robot with Rviz and having a Gazebo window
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Définir les arguments
    model = LaunchConfiguration("model")
    robot_name = LaunchConfiguration("robot_name")

    ld = LaunchDescription()

    # Ajout des arguments de lancement
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())

    # Générer la description du robot (URDF)
    robot_description = LBRDescriptionMixin.param_robot_description(mode="gazebo")

    # Lancer Gazebo (vide, sans robot)
    ld.add_action(GazeboMixin.include_gazebo())

    # Spawner le robot à partir du topic robot_description
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "robot_description"
        ],
        output="screen"
    )

    # Délai pour laisser Gazebo démarrer
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    ld.add_action(delayed_spawn)

    return ld
