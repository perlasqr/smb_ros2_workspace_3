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
                    "TODO_STUDENTS_smb_sim.launch.py"
                ])
            ) 
        ),
        Node(
            package='delayed_publisher',  # replace with your package name
            executable='delayed_publisher',
            name='delayed_publisher_node',
            output='screen'
        )
    ])
