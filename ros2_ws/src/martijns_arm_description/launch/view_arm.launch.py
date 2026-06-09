"""View Martijn's arm in RViz/Foxglove with joint sliders.

Loads the URDF via xacro, starts robot_state_publisher (which publishes
TF transforms based on joint_states), and joint_state_publisher_gui
(which gives sliders to drive the joints).

Foxglove panels: 3D panel will pick up the URDF + TF automatically.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("martijns_arm_description")
    default_xacro = PathJoinSubstitution([pkg_share, "urdf", "martijns_arm.urdf.xacro"])

    use_gui = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Start joint_state_publisher_gui (sliders) i.p.v. publisher zonder UI")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("urdf", default=default_xacro)]),
            value_type=str)
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=__import__("launch.conditions", fromlist=["IfCondition"]).IfCondition(
            LaunchConfiguration("use_gui")),
    )

    jsp_no_gui = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=__import__("launch.conditions", fromlist=["UnlessCondition"]).UnlessCondition(
            LaunchConfiguration("use_gui")),
    )

    return LaunchDescription([use_gui, rsp, jsp_gui, jsp_no_gui])
