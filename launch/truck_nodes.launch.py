import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    truck_id = os.environ.get('TRUCK_ID', 'default')
    node_name = f"truck_{truck_id}"

    return LaunchDescription([
        Node(
            package='multi_truck_scenario',
            executable='vehicle_node',
            name=node_name,
        )
    ])