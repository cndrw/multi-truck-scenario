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
            parameters=[{
                'vin': truck_id,
                'engine_state' : 0,
                    'speed': 0.0,
                    'indicator_state': 0,
                    'position_x': 0,
                    'position_y': 0,
                    'position_z': 0,
                    'direction': 0,
                    'is_simulated': True,
                    'scenario_detector': 0, # Hardcode
                    'decision_algorithm': 0
                }],
                arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])