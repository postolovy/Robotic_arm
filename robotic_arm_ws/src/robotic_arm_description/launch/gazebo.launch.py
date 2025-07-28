from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("robotic_arm_description")

    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, "urdf", "Robotic_arm_urdf.urdf")
    
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = urdf_file,
        description = "Absolute path to robot urdf file")
    
    gazebo_resource_path = SetEnvironmentVariable( 
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(pkg_share).parent.resolve())
        ]
    )

    # robot_state_publisher node to publish the robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": open(urdf_file).read(),
            "use_sim_time": True
        }], 
        output="screen"
    )

    gazebo = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )   
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf"
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robotic_arm",
            # "-x", "0.0",
            # "-y", "0.0",
            # "-z", "0.5",
            # "-Y", "0.0"
        ],
        output="screen"
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])