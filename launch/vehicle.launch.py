import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi-truck-scenario',
            executable='map_node',
            name='map_simulation',
        ),
        # launch rviz2 for 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/zdm/ros2_ws/config_1.rviz'],  # Optional: specify a config file
        ),
        Node(
            package='multi-truck-scenario',
            executable='vehicle_node',
            name='vehicle_comms',
        )
    ])