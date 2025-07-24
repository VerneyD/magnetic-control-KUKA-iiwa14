# not working 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # Hardware
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_package'),
                    'launch',
                    'hardware_for_gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'model': 'iiwa14'
            }.items()
        ),

        # MoveIt + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_package'),
                    'launch',
                    'move_group_hw.launch.py'
                ])
            ]),
            launch_arguments={
                'model': 'iiwa14',
                'rviz': 'true',
            }.items()
        ),
        
        # Gazebo 
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('my_package'),
        #             'launch',
        #             'gazebo_for_hardware.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'model': 'iiwa14'
        #     }.items()
        # ),

    ])
