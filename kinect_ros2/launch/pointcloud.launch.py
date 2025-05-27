import os

import launch_ros
from launch_ros.actions.node import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find(
        "kinect_ros2"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")
    bridge_script = os.path.join(pkg_share, 'src/bridge_server.py')
    bridge = Node(executable=bridge_script,
            name='bridge',
            parameters=[{'use_sim_time': False}]
            )
    return LaunchDescription(
        [
            bridge,
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="kinect",
                parameters=[{'use_sim_time': False}],
            ),
            #Node(
            #    package="rviz2",
            #    executable="rviz2",
            #    name="rviz2",
            #    output="screen",
            #    arguments=["-d", LaunchConfiguration("rvizconfig")],
            #),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='depth_to_world_transform',
                parameters=[{'use_sim_time': False}],
                arguments=['0', '0', '0.07', '0', '0', '-1.5708', 'base_link', 'kinect_depth']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='kinect_rgb_to_depth',
                arguments=['0', '0', '0', '0', '0', '0', 'kinect_depth', 'kinect_rgb']
            )

        ]
    )
