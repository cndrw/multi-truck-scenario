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
                    'indicator_state': vehicle['indicator'],
                    'position_x': vehicle['position_x'],
                    'position_y': vehicle['position_y'],
                    'position_z': vehicle['position_z'],
                    'direction': vehicle['direction_angle'],
                    'is_simulated': True,
                    'scenario_detector': 0, # Hardcode
                    'decision_algorithm': 0
                }],
                arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])