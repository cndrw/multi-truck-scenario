import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import random
from pathlib import Path
import sys

## Pathlib option - add config file path
workspace_dir = Path(__file__).resolve().parent.parent
config_file_path = workspace_dir / 'config' / 'config_1.rviz'
config_file_path = config_file_path.resolve()
## --------------------------------------------------------------------------------
## Add the script directory to the Python path
script_dir = Path(__file__).resolve().parent.parent / 'script'
sys.path.insert(0, str(script_dir))

# Import the function that generates the RViz-compatible grid data
from image2grid_converter import generate_rviz_static_map
from image2scene import output_event, output_event_streets
## --------------------------------------------------------------------------------

def generate_launch_description():

    # Define image path using Pathlib - set image that is needed for this scene
    image_path = script_dir / 'scenario_S1_colored.png'

    crossing_vals = output_event(str(image_path))
    street_vals = output_event_streets(str(image_path))
    width_values,height_values,bot_left_x_values,bot_left_y_values = crossing_vals[0],crossing_vals[1],crossing_vals[2],crossing_vals[3]
    width_street_left, width_street_right, width_street_top, width_street_bottom = street_vals[0],street_vals[1],street_vals[2],street_vals[3]
    # Generate static map using the function from image_converter.py
    result = generate_rviz_static_map(str(image_path))

    return LaunchDescription([
        Node(
            package='multi_truck_scenario',
            executable='map_node',
            name='map_simulation',
            parameters=[{
                'height': result['height'],
                'width': result['width'],
                'static_map': result['static_map'],
                'crossing_width_values': width_values,
                'crossing_height_values': height_values,
                'crossing_bot_left_x_values': bot_left_x_values,
                'crossing_bot_left_y_values': bot_left_y_values,
                'street_width_left': width_street_left,
                'street_width_right': width_street_right,
                'street_width_top': width_street_top,
                'street_width_bottom': width_street_bottom,
            }],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments = ['-d', str(config_file_path)],  
        )
    ])