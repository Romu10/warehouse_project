import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Replace 'your_package_name' and 'your_configuration_basename' with the correct values
your_package_name = 'cartographer_slam'
your_configuration_basename = 'cartographer.lua'

cartographer_config_dir = os.path.join(get_package_share_directory(your_package_name), 'config')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', your_configuration_basename]
        ),
    
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])