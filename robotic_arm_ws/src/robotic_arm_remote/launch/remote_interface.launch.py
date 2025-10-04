from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")


    task_servo_node = Node(
        package="robotic_arm_remote",
        executable="task_servo_node",
        parameters=[{"use_sim_time": is_sim}]
    )

    alexa_interface_node = Node(
        package="robotic_arm_remote",
        executable="alexa_interface.py",
        parameters=[{"use_sim_time": is_sim}]
    )

    return LaunchDescription([
        is_sim_arg,
        task_servo_node,
        alexa_interface_node,
    ])
