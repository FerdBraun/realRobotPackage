import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Specify the name of the package and path to xacro file
    pkg_name = 'real'
    lidar  = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rplidar_ros'),
            'launch',
            'rplidar_c1_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="filter1",
        parameters=[{
            'filter1': {
                'name': 'filter1',
                'type': 'laser_filters/LaserScanRangeFilter',
                'params': {
                    'lower_threshold': 0.25,
                    'upper_threshold': 5.0,
                    'lower_replacement_value': float('-inf'),
                    'upper_replacement_value': float('inf')
                }
            }
        }],
        remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
        ],
    )
    return LaunchDescription([
        lidar,
        laser_filter,
        Node(
               package='tf2_ros',
               executable='static_transform_publisher',
               name='depth_to_world_transform',
               parameters=[{'use_sim_time': False}],
               arguments=['0', '0', '0', '-1.54', '0', '0', 'base_link', 'laser']
           ),
    ])
