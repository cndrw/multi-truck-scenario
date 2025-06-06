import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import random
from pathlib import Path
import sys

## make sure to choose one option of the following, don't forget the map node
## --------------------------------------------------------------------------------
## os option - older and less intuitive
# import os
# workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# config_file_path = os.path.join(workspace_dir, 'config', 'config_1.rviz')

## --------------------------------------------------------------------------------
## Pathlib option - more modern and recommended
workspace_dir = Path(__file__).resolve().parent.parent
config_file_path = workspace_dir / 'config' / 'config_1.rviz'
config_file_path = config_file_path.resolve()
## --------------------------------------------------------------------------------
## Add the script directory to the Python path
script_dir = Path(__file__).resolve().parent.parent / 'script'
sys.path.insert(0, str(script_dir))
# Import the function that generates the RViz-compatible grid data
from image2grid_converter import generate_rviz_static_map
## --------------------------------------------------------------------------------

def generate_launch_description():
    
    # Define image path using Pathlib
    image_path = script_dir / 'big_test.png'
    
    # Generate static map using the function from image_converter.py
    result = generate_rviz_static_map(str(image_path))
    # static_map = result['static_map']
    # width = result['width']
    # height = result['height']
    
    # static_map = [
    #     100, 0, 0, 100,  # Black, Grey, Grey, Black 
    #     0, 0, 0, 0,        # Grey, Grey, Grey, Grey
    #     0, 0, 0, 0,        # Grey, Grey, Grey, Grey
    #     100, 0, 0, 100   # Black, Grey, Grey, Black
    # ]

    # extract grid values from result
    # grid_values = {
    #     'height': result['height'],
    #     'width': result['width'],
    #     'static_map': result['static_map']
    # }

    # extract vehicle start coordinates from result


    vehicles = [
        {'name': 'vehicle_1', 'vin': 1, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 2.0, 'position_y': 0.0, 'position_z': 0.0, 'direction_angle': 90.0},
        {'name': 'vehicle_2', 'vin': 2, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 3.0, 'position_y': 2.0, 'position_z': 0.0, 'direction_angle': 180.0}, 
        {'name': 'vehicle_3', 'vin': 3, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 0.0, 'position_y': 1.0, 'position_z': 0.0, 'direction_angle': 0.0},
        {'name': 'vehicle_4', 'vin': 4, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 1.0, 'position_y': 3.0, 'position_z': 0.0, 'direction_angle': 270.0}
        # Add more vehicles as needed
    ]

    dir_offset = 5 # offset in degrees
    offset_val_list = [random.uniform(-dir_offset, dir_offset) for i in range(len(vehicles))] # random offset values

    # add small offset to each angle
    # for i, v in enumerate(vehicles):
        # v['direction_angle'] += offset_val_list[i]

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

    return LaunchDescription(
        vehicle_nodes + [
        Node(
            package='multi_truck_scenario',
            executable='map_node',
            name='map_simulation',
            ## add parameter for grid data
            ## parameters not yet implemented in map.cpp
            parameters=[{
                 'height': result['height'],
                 'width': result['width'],
                 'static_map': result['static_map'],
            }],
        ),
        # launch rviz2 for 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments = ['-d', str(config_file_path)],  # Pathlib option - type casting is needed
            # arguments = ['-d', config_file_path],  # os option
        )
    ])