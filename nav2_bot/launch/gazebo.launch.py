import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("nav2_bot")
    urdf_file = os.path.join(package_share_dir, "urdf", "robot.urdf.xacro")
    world_file= os.path.join(package_share_dir,"worlds","nav2_world.world")
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    env = {
        "GAZEBO_MODEL_PATH": model_path, # as we only to add nav2_bot(model) into gazebo models path
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","--verbose",world_file,"-s","libgazebo_ros_factory.so",],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity","nav2_bot","-b","-file", urdf_file],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{'robot_description': Command([urdf_file])}]
            ),
        ]
    )
    