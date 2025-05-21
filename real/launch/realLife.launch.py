import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

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
    decompresser_script = os.path.join(get_package_share_directory(pkg_name), 'topics/decompresser.py')

    decompresser = Node(executable=decompresser_script,
            name='decompresser',
            parameters=[{'use_sim_time': False}] )
    return LaunchDescription([
        decompresser,
        node_robot_state_publisher,
        joint_state_publisher_gui,  # Можно убрать для реального робота
        # hardware_interface,
        # Другие необходимые ноды для вашего конкретного железа
    ])
