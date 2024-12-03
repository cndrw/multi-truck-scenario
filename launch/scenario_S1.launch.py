import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import random
import sys

# Add the script directory to the Python path
script_dir = Path(__file__).resolve().parent.parent / 'script'
sys.path.insert(0, str(script_dir))

# Import the function that generates the RViz-compatible grid data
from image2grid_converter import generate_rviz_static_map

# Define the function for launching the nodes
def generate_launch_description():

    # Define image path using Pathlib
    image_path = script_dir / 'painting_10x10_6_colors.png'
    
    # Generate static map using the function from image_converter.py
    result = generate_rviz_static_map(str(image_path))
    static_map = result['static_map']
    width = result['width']
    height = result['height']
    
    # Print the generated static map and dimensions (for debugging purposes)
    print("Generated Static Map:", static_map)
    print(f"Map Width: {width}, Map Height: {height}")

    # Example vehicle setup (can be expanded)
    vehicles = [
        {'name': 'vehicle_1', 'vin': 1, 'engine': 0, 'speed': 0.0, 'indicator': 1, 'position_x': 2.0, 'position_y': 0.0, 'position_z': 0.0, 'direction_angle': 90.0},
        {'name': 'vehicle_2', 'vin': 2, 'engine': 0, 'speed': 0.0, 'indicator': 1, 'position_x': 3.0, 'position_y': 2.0, 'position_z': 0.0, 'direction_angle': 180.0},
    ]

    # Vehicle nodes
    vehicle_nodes = []
    for vehicle in vehicles:
        vehicle_nodes.append(
            Node(
                package='multi_truck_scenario',
                executable='vehicle_node',
                name=vehicle['name'],
                parameters=[{
                    'vin': vehicle['vin'],
                    'engine_state': vehicle['engine'],
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

    # Return launch description with nodes
    return LaunchDescription(
        vehicle_nodes + [
            # Launch the map node with the static_map and its dimensions
            Node(
                package='multi_truck_scenario',
                executable='map_node',
                name='map_simulation',
                parameters=[{
                    'height': LaunchConfiguration('height', default=str(height)),  # Convert height to string
                    'width': LaunchConfiguration('width', default=str(width)),    # Convert width to string
                    'static_map': LaunchConfiguration('static_map', default=str(static_map)),  # Convert static_map to string
                }],
            ),
            # Launch RViz with the provided config
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', str(script_dir.parent / 'config' / 'config_1.rviz')],  # Path to RViz config file
            )
        ]
    )
