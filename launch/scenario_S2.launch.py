import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    vehicles = [
        {'name': 'vehicle_1', 'vin': 0, 'speed': 0, 'indicator': 0, 'position_x': 0, 'position_y': 0, 'position_z': 0, 'direction_x': 0, 'direction_y': 0, 'direction_z': 0},
        {'name': 'vehicle_2', 'vin': 1, 'speed': 0, 'indicator': 0, 'position_x': 1, 'position_y': 0, 'position_z': 0, 'direction_x': 0, 'direction_y': 0, 'direction_z': 0},
        {'name': 'vehicle_3', 'vin': 2, 'speed': 0, 'indicator': 0, 'position_x': 2, 'position_y': 1, 'position_z': 0, 'direction_x': 0, 'direction_y': 0, 'direction_z': 0},
        {'name': 'vehicle_4', 'vin': 3, 'speed': 0, 'indicator': 0, 'position_x': 3, 'position_y': 1, 'position_z': 0, 'direction_x': 0, 'direction_y': 0, 'direction_z': 0},
        # Add more vehicles as needed
    ]

    vehicle_nodes = []
    for vehicle in vehicles:
        vehicle_nodes.append(
            Node(
                package='multi-truck-scenario',
                executable='vehicle_node',
                name=vehicle['name'],
                parameters=[{
                    'vin': vehicle['vin'],
                    'speed': vehicle['speed'],
                    'indicator_state': vehicle['indicator'],
                    'position_x': vehicle['position_x'],
                    'position_y': vehicle['position_y'],
                    'position_z': vehicle['position_z'],
                    'direction_x': vehicle['direction_x'],
                    'direction_y': vehicle['direction_y'],
                    'direction_z': vehicle['direction_z']
                }]
            )
        )

    return LaunchDescription(vehicle_nodes + [
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