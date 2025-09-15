from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, TextSubstitution

def generate_launch_description(): 

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotic_arm_description"), 
            "launch", 
            "gazebo.launch.py"
            )
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotic_arm_controllers"), 
            "launch", 
            "controller.launch.py"
        ), 
        launch_arguments={"is_sim" : "True"}.items()
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotic_arm_moveit"), 
            "launch", 
            "moveit.launch.py"
        ), 
        launch_arguments={"is_sim" : "True"}.items()
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotic_arm_remote"), 
            "launch", 
            "remote_interface.launch.py"
        )
    )


    return LaunchDescription([
        SetEnvironmentVariable(
            name="PATH",
            value=[
                TextSubstitution(text="/home/postolovy/openssl-3.0.9/bin:"),
                EnvironmentVariable("PATH", default_value=TextSubstitution(text="")),
            ],
        ),
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                TextSubstitution(text="/home/postolovy/openssl-3.0.9/lib64:"),
                EnvironmentVariable("LD_LIBRARY_PATH", default_value=TextSubstitution(text="")),
            ],
        ),
        gazebo,
        controller,
        moveit,
        remote_interface
    ])