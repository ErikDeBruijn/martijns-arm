"""View the 7Bot URDF in Foxglove/RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("sevenbot_description")
    default_xacro = PathJoinSubstitution([pkg, "urdf", "sevenbot.urdf.xacro"])

    use_gui = DeclareLaunchArgument("use_gui", default_value="false")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("urdf", default=default_xacro)]),
            value_type=str)
    }

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[robot_description], output="screen",
    )
    jsp = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        output="screen", condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )
    jsp_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        output="screen", condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    return LaunchDescription([use_gui, rsp, jsp, jsp_gui])
