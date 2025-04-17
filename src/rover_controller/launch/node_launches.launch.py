from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_controller',
            executable='lidar_processor_node',
            name='lidar_processor_node',
            output='screen'
        ),
        Node(
            package='rover_controller',
            executable='proximity_warning_node',
            name='proximity_warning_node',
            output='screen'
        ),
        Node(
            package='rover_controller',
            executable='emergency_stop_node',   
            name='emergency_stop_node',
            output='screen'
        ),
    ])