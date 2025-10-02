import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    package_name = "sim_tutorial"

   # Declare launch arguments
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Topic on which to publish velocity commands'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Initial linear speed'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Initial angular speed'
    )
    
    # Create the teleop_twist_keyboard node
    teleop_node = Node(
        package=package_name,  # Replace with your actual package name
        executable='teleop_twist_keyboard',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed')
            }
        ]
    )

    # Launch them all!
    return LaunchDescription(
        [
            cmd_vel_topic_arg,
            linear_speed_arg,
            angular_speed_arg,
            teleop_node,
        ]
    )
