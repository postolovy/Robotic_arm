import os 
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

spawn_delay = 8.0  # seconds, adjust if needed

def generate_launch_description(): 

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_sim = LaunchConfiguration("is_sim")

    xacro_file = os.path.join(
        get_package_share_directory('robotic_arm_description'), 
        'urdf', 'robotic_arm.urdf.xacro'
        )
    yaml_file = os.path.join(
        get_package_share_directory('robotic_arm_controllers'),
        'config', 'robotic_arm_controllers.yaml'                     
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            xacro_file,
            # ' is_sim:=', is_sim
            " is_sim:=False"
        ]), 
        value_type=str  
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        condition=UnlessCondition(is_sim)        
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node", 
        parameters=[
            {"robot_description": robot_description_content,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("robotic_arm_controllers"),
                "config",
                "robotic_arm_controllers.yaml"
            )
        ], 
        condition=UnlessCondition(is_sim)
    )

    joint_state_broadcaster_spawner = Node( 
        package='controller_manager', 
        executable='spawner', 
        arguments=[
            'joint_state_broadcaster', 
            '--controller-manager',
            '/controller_manager'
        ]
    )

    arm_links_controller_spawner = Node( 
        package='controller_manager', 
        executable='spawner', 
        arguments=[
            'arm_links_controller', 
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            yaml_file
        ]
    )

    gripper_controller_spawner = Node( 
        package='controller_manager', 
        executable='spawner', 
        arguments=[
            'gripper_controller', 
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            yaml_file
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner, 
        arm_links_controller_spawner, 
        gripper_controller_spawner
    ])