import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'real'
 
    # Configure the node
 
   
    bridge_script = os.path.join(get_package_share_directory(pkg_name), 'topics/bridge_client.py')

    bridge = Node(executable=bridge_script,
            name='bridge',
            parameters=[{'use_sim_time': False}] )
    imu_script = os.path.join(get_package_share_directory(pkg_name), 'topics/imu_sockettest.py')

    imu = Node(executable=imu_script,
            name='imu',
            parameters=[{'use_sim_time': False}] )
    

    # Run the node
    return LaunchDescription([
        imu,
        #bridge,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_to_world_transform',
            parameters=[{'use_sim_time': False}],
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
            ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            remappings=[
                #('/imu/data_raw', '/imu'),  # Перенаправляем ваш топик на ожидаемый
              #  ('/imu/mag', '/mag')       # Если используете магнитометр
              ('/imu/data', '/imu')   
            ],
            parameters=[{
                'use_mag': False,          # Отключаем магнитометр, если не используется
                'publish_tf': False,        # Отключаем TF, если не нужно
                  # Система координат
            }]
        )

    ])