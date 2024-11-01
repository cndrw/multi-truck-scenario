import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    vehicles = [
        {'name': 'vehicle_1', 'position_x': 0, 'position_y': 0, 'direction': 0},
        {'name': 'vehicle_2', 'position_x': 1, 'position_y': 0, 'direction': 90},
        {'name': 'vehicle_3', 'position_x': 2, 'position_y': 1, 'direction': 180},
        {'name': 'vehicle_4', 'position_x': 3, 'position_y': 1, 'direction': 270},
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
                    'FIN': 0,
                    'speed_V': 0,
                    'Blinker': 0,
                    'position_x': vehicle['position_x'],
                    'position_y': vehicle['position_y'],
                    'direction': vehicle['direction']
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