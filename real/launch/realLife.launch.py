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
    file_subpath = 'description/example_robot.urdf.xacro'

    # Process the URDF file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': False}]  # Используем реальное время
    )

    # Joint State Publisher GUI (опционально, для тестирования)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Для реального робота вам нужно будет:
    # 1. Подключиться к реальным датчикам (IMU, лидары и т.д.)
    # 2. Получать реальные состояния суставов от железа
    # 3. Запустить необходимые контроллеры

    # Пример ноды для подключения к реальному железу
    # hardware_interface = Node(
    #     package='your_hardware_package',
    #     executable='hardware_interface',
    #     name='hardware_interface',
    #     output='screen'
    # )
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

    decompresser_script = os.path.join(get_package_share_directory(pkg_name), 'topics/decompresser.py')

    decompresser = Node(executable=decompresser_script,
            name='decompresser',
            parameters=[{'use_sim_time': False}] )
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

        #decompresser,
        #node_robot_state_publisher,
        #joint_state_publisher_gui,  # Можно убрать для реального робота
        # hardware_interface,
        # Другие необходимые ноды для вашего конкретного железа
    ])
