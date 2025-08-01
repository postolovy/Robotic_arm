import os 
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction 

spawn_delay = 8.0  # seconds, adjust if needed

def generate_launch_description(): 
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
            xacro_file
        ]), 
        value_type=str  
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]        
    )

    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': robot_description_content},
    #         yaml_file
    #     ]
    # )

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
        robot_state_publisher,
        # TimerAction(period=spawn_delay, actions=[
        #     joint_state_broadcaster_spawner, 
        #     arm_links_controller_spawner, 
        #     gripper_controller_spawner
        # ]) 
        # # controller_manager, 
        joint_state_broadcaster_spawner, 
        arm_links_controller_spawner, 
        gripper_controller_spawner
    ])