import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import random

## make sure to choose one option of the following, don't forget the map node
## --------------------------------------------------------------------------------
## os option - older and less intuitive
# import os
# workspace_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# config_file_path = os.path.join(workspace_dir, 'config', 'config_1.rviz')

## --------------------------------------------------------------------------------
## Pathlib option - more modern and recommended
from pathlib import Path
workspace_dir = Path(__file__).resolve().parent.parent
config_file_path = workspace_dir / 'config' / 'config_1.rviz'
config_file_path = config_file_path.resolve()
## --------------------------------------------------------------------------------
# import the image2grid script and execute it for the map node
#from script.image2grid_converter import generate_cpp_grid_from_image
# specify path of image using Pathlib
#scenario_S2_map_path = workspace_dir / 'script' / 'painting_10x10_6_colors.png'
# convert image to grid data - see below for the implementation of the parameters
#grid_values = generate_cpp_grid_from_image(scenario_S2_map_path)
## --------------------------------------------------------------------------------

def generate_launch_description():
    
    static_map = [
        100, 100, 0, 100, 100, #0/0, 0/1, ... here as Matrix in RVIZ 0/0 is bottom left
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        100, 100, 100, 100, 100
    ]

    # Hardcoded width and height for the map
    grid_values = {
        'height': 4,
        'width': 5,
        'color_map': static_map
    }


    vehicles = [
        {'name': 'vehicle_1', 'vin': 1, 'engine': 0, 'speed': 0.0, 'indicator': 1, 'position_x': 2.0, 'position_y': 0.0, 'position_z': 0.0, 'direction_angle': 90.0},
        {'name': 'vehicle_2', 'vin': 2, 'engine': 0, 'speed': 0.0, 'indicator': 1, 'position_x': 3.0, 'position_y': 2.0, 'position_z': 0.0, 'direction_angle': 180.0}, 
        #{'name': 'vehicle_3', 'vin': 3, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 0.0, 'position_y': 1.0, 'position_z': 0.0, 'direction_angle': 0.0},
        #{'name': 'vehicle_4', 'vin': 4, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 1.0, 'position_y': 3.0, 'position_z': 0.0, 'direction_angle': 270.0}
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
                 'height': grid_values['height'],
                 'width': grid_values['width'],
                 'static_map': grid_values['color_map'],
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