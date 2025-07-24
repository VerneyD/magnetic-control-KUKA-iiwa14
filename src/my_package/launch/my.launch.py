#  This launch files is used to control the joints of the robot with a RViz interface. You can see the robot in live action on this laptop
# To do so, on the SmartPad : select LBRServer Application, then 

        # FRI send period: 2 ms

        # IP address: your configuration

        # FRI control mode: POSITION_CONTROL 

        # FRI client command mode: JOINT_POSITION

        #On the terminal type : cd lbr_stack
        #                       source install/setup.bash
        #                       ros2 launch my_package my.launch.py 
        #within 10 seconds





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


    # Lancer la simulation move_group
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
            'model': 'iiwa14'
        }.items()
    )

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

    # end_effector_position_node = Node(
    #     package='my_package',
    #     executable='end_effector_position_node',
    # )
    # nodes.append(end_effector_position_node)
    # Ajout des lances à la liste des noeuds
    nodes.append(launch_simulation)
    nodes.append(launch_hardware)

    # Retourner la LaunchDescription avec tous les noeuds et inclusions
    return LaunchDescription(nodes)
