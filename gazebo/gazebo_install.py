import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('your_package'),
        'worlds',
        'your_world.sdf'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo_sim',
            arguments=[world_file],
            output='screen'
        )
    ])
