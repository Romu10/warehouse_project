import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

filters_yaml = os.path.join(get_package_share_directory(
        'path_planner_server'), 'config', 'filters.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/pathplanning.rviz'],
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/planner_server.yaml"]
        ),
    
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            remappings=[('/cmd_vel', '/robot/cmd_vel')],
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/controller.yaml"]
        ),
        
        Node(
             package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/bt.yaml"]
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/recovery.yaml"],
            remappings=[('/cmd_vel', '/robot/cmd_vel')],
            output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['planner_server', 'controller_server', 'bt_navigator', 'recoveries_server', 'filter_mask_server', 'costmap_filter_info_server']}],
        ),



    ])