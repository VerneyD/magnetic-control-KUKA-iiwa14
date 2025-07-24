#  useful to launch gazebo simulation with rviz and the magnet
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.gazebo import GazeboMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Arguments nécessaires (model, robot_name, ctrl)
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(LBRROS2ControlMixin.arg_ctrl())

    # Paramètre robot_description (mode gazebo = ajoute <gazebo> tags, use_sim_time, etc.)
    robot_description = LBRDescriptionMixin.param_robot_description(mode="gazebo")

    # Node robot_state_publisher (avec use_sim_time)
    ld.add_action(
        LBRROS2ControlMixin.node_robot_state_publisher(
            robot_description=robot_description,
            use_sim_time=True
        )
    )

    # Lancer Gazebo (via ros_gz_sim) + bridge /clock
    ld.add_action(GazeboMixin.include_gazebo())
    ld.add_action(GazeboMixin.node_clock_bridge())

    # Spawner du robot dans Gazebo depuis le robot_description
    ld.add_action(GazeboMixin.node_create())

    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller="joint_state_broadcaster"
        )
    )

    # Spawner du contrôleur principal choisi (position_controller, effort_controller, etc.)
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
    )

    # Capsule magnétique (chargement différé après 5 secondes)
    spawn_capsule = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "capsule",
            "-file", "/home/srl/.gazebo/models/magnetic_capsule/model.sdf",
            "-x", "0.7",
            "-y", "-0.5",
            "-z", "0.02"
        ],
        output="screen"
    )

    ld.add_action(
        TimerAction(
            period=8.0,
            actions=[spawn_capsule]
        )
    )

    spawn_box = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-name", "box",
        "-file", "/home/srl/.gazebo/models/box/model.sdf",
        "-x", "0.7",
        "-y", "-0.5",
        "-z", "-0.45",
        "-roll", "0",
        "-pitch", "0"
    ],
    output="screen"
)

    delayed_spawn_box = TimerAction(
        period=7.0,  # délai en secondes
        actions=[spawn_box]
    )
    ld.add_action(delayed_spawn_box)

    return ld
