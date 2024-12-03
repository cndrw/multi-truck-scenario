import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import random
import sys
from pathlib import Path

# Add the 'script' directory to the Python path so that we can import our function
script_dir = Path(__file__).resolve().parent.parent / 'script'
sys.path.insert(0, str(script_dir))

# Import the function from the image2grid_converter module
from image2grid_converter import generate_cpp_grid_from_image

def generate_launch_description():
    
    image_path = script_dir / 'scenario_S2.png'
    
    # Generate C++ grid data and dimensions
    grid_data_info = generate_cpp_grid_from_image(str(image_path))
    static_map = grid_data_info['cpp_grid_data']
    width = grid_data_info['width']
    height = grid_data_info['height']

    # Print the data (for debugging)
    print("C++ Grid Data:\n", static_map)
    print(f"Image Dimensions: Width={width}, Height={height}")

    grid_values = {
        'height': height,
        'width': width,
        'color_map': static_map
    }

    vehicles = [
        {'name': 'vehicle_1', 'vin': 1, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 2.0, 'position_y': 0.0, 'position_z': 0.0, 'direction_angle': 90.0},
        {'name': 'vehicle_2', 'vin': 2, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 3.0, 'position_y': 2.0, 'position_z': 0.0, 'direction_angle': 180.0}, 
        {'name': 'vehicle_3', 'vin': 3, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 0.0, 'position_y': 1.0, 'position_z': 0.0, 'direction_angle': 0.0},
        {'name': 'vehicle_4', 'vin': 4, 'engine': 0, 'speed': 0.0, 'indicator': 0, 'position_x': 1.0, 'position_y': 3.0, 'position_z': 0.0, 'direction_angle': 270.0}
    ]

    dir_offset = 5 # offset in degrees
    offset_val_list = [random.uniform(-dir_offset, dir_offset) for i in range(len(vehicles))] # random offset values

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
        )
    ])
