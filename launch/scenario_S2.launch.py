import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import random

## --------------------------------------------------------------------------------

# Get the current working directory (workspace directory)
workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

# Construct the path to the config file directly from the src directory
config_file_path = os.path.join(workspace_dir, 'src', 'multi-truck-scenario', 'config', 'config_1.rviz')

## Hard coded should be fixed later, works on my machine
## --------------------------------------------------------------------------------

## Tasks: - Direction of the vehicles with rnd generated offset
##        - Rotate map & Vehicles
##        - 
def generate_launch_description():
    
    # offset_range = random.uniform(-5, 5)
    base_rotation = 0.0
    no_of_vehicles = 3
    dir_offset_min = -5
    dir_offset_max = 5
    offset_val_list = [random.uniform(dir_offset_min, dir_offset_max) for i in range(no_of_vehicles)]

    vehicles = [
        {'name': 'vehicle_1', 'vin': 1, 'speed': 0.0, 'indicator': 0, 'position_x': 2.0, 'position_y': 0.0, 'position_z': 0.0, 'direction_angle': base_rotation + 90.0 + offset_val_list[0]},
        {'name': 'vehicle_2', 'vin': 2, 'speed': 0.0, 'indicator': 0, 'position_x': 3.0, 'position_y': 2.0, 'position_z': 0.0, 'direction_angle': base_rotation + 180.0 + offset_val_list[1]},
        {'name': 'vehicle_3', 'vin': 3, 'speed': 0.0, 'indicator': 0, 'position_x': 1.0, 'position_y': 3.0, 'position_z': 0.0, 'direction_angle': base_rotation + 270.0 + offset_val_list[2]},
        # {'name': 'vehicle_4', 'vin': 3, 'speed': 0, 'indicator': 0, 'position_x': 3, 'position_y': 1, 'position_z': 0, 'direction_angle': 0},
        # Add more vehicles as needed
    ]

    vehicle_nodes = []
    for vehicle in vehicles:
        vehicle_nodes.append(
            Node(
                package='multi_truck_scenario',
                executable='vehicle_node',
                name=vehicle['name'],
                parameters=[{
                    'vin': vehicle['vin'],
                    'engine_state' : vehicle['engine'],
                    'speed': vehicle['speed'],
                    'indicator_state': vehicle['indicator'],
                    'position_x': vehicle['position_x'],
                    'position_y': vehicle['position_y'],
                    'position_z': vehicle['position_z'],
                    'direction': vehicle['direction_angle']
                }],
                arguments=['--ros-args', '--log-level', 'INFO']
            )
        )

    return LaunchDescription(vehicle_nodes + [
        Node(
            package='multi_truck_scenario',
            executable='map_node',
            name='map_simulation',
        ),
        # launch rviz2 for 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments = ['-d', config_file_path],  # Optional: specify a config file
        )
    ])