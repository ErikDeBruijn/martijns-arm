"""Spawn Martijn's arm in Gazebo Sim met ros2_control.

Topics na launch:
    /joint_states                       sensor_msgs/JointState   (van controller)
    /arm_controller/joint_trajectory    trajectory_msgs/JointTrajectory  (commando's)
    /tf, /tf_static                     transforms

Test commando om de arm te bewegen (in een aparte terminal):
    ros2 topic pub --once /arm_controller/joint_trajectory \\
        trajectory_msgs/msg/JointTrajectory '{
            joint_names: [joint_1, joint_2, joint_3],
            points: [{positions: [0.5, 0.3, -0.4], time_from_start: {sec: 2}}]
        }'
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare("martijns_arm_gazebo")
    pkg_ros_gz = FindPackageShare("ros_gz_sim")

    xacro_file = PathJoinSubstitution([pkg_gazebo, "urdf", "martijns_arm.gazebo.xacro"])
    world_file = PathJoinSubstitution([pkg_gazebo, "worlds", "empty.world"])

    headless = DeclareLaunchArgument(
        "headless", default_value="true",
        description="Start gz sim zonder GUI (true voor container)")

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # gz_args via Substitution-list zodat de paden goed worden opgelost
    gz_args = [TextSubstitution(text="-r -s --headless-rendering "), world_file]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_ros_gz, "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", gz_args)],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "martijns_arm",
                   "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0.05"],
        output="screen",
    )

    # Controllers worden geladen NA spawn, anders zien ze geen plant
    load_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    load_arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # Bridge: gz topics ↔ ROS topics (klok van gz naar /clock)
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawner pas starten als robot bestaat in gz
    delayed_jsb = RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[load_jsb]))
    delayed_arm = RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm_ctrl]))

    return LaunchDescription([
        headless, rsp, gz_sim, bridge, spawn_robot, delayed_jsb, delayed_arm,
    ])
