

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("leader_follower_navigation")
    leader_urdf = os.path.join(package_share_dir, "urdf", "leader.urdf")
    follower_urdf = os.path.join(package_share_dir, "urdf", "follower.urdf")

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
            output="screen",
            additional_env=env,
        ),
        
        # Spawning the first robot at the origin
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "leader_bot", "-b", "-file", leader_urdf,],
            output="screen",
        ),

        # Spawning the second robot at (1, 0, 0)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "follower_bot", "-b", "-file", follower_urdf, "-x", "1", "-y", "0", "-z", "0"],
            output="screen",

        ),

        # Robot state publisher for the first robot
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[leader_urdf],
            name="robot_state_publisher_1"
        ),

        # Robot state publisher for the second robot
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[follower_urdf],
            name="robot_state_publisher_2"
        ),
        Node(
            package="leader_follower_navigation",
            executable="follower_node",
            output="screen",
            name="follower_node"
        ),
    ])
