from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(
                        "smb_bringup",
                    ),
                    "launch",
                    "smb_sim_exploration.launch.py"
                ])
            ) 
        ),
        Node(
            package='my_mission_package',  # replace with your package name
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        )
    ])
