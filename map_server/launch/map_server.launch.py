import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_path',
            default_value='/home/user/ros2_ws/src/warehouse_project/map_server/config',
            description='Ruta al directorio del archivo de mapa YAML'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_sim.yaml',
            description='Nombre del archivo de mapa YAML'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': PathJoinSubstitution([
                    LaunchConfiguration('map_file_path'),
                    LaunchConfiguration('map_file')
                ])}
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/warehouse_project/map_server/rviz/visualize_map_config.rviz'],
            parameters=[{'use_sim_time': True}]
        ),
    ])
