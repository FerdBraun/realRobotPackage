from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'frame_id': 'map'},  # Основная система координат
                {'resolution': 0.1},  # Увеличьте для повышения производительности
                {'sensor_model.max_range': 5.0}, # Ограничьте дальность
                {'compress_map': True},  # Сжатие карты
                {'incremental_2D_projection': False},  # Отключите если не нужно 2D
                {'pointcloud_min_z': 0.15},  # Фильтр пола
                {'pointcloud_max_z': 2.0},  # Фильтр потолка
                {'occupancy_min_z': 0.2},
                {'occupancy_max_z': 2.0},
                {'publish_octomap': True},
                {'publish_occupancy_grid': True},
                {'publish_occupied_3d': True},
                {'publish_free_space': True},
                {'latch': False},  # Динамическое обновление
                {'track_changes': True},  # Накопление изменений
                {'listen_to_changes': True},
                {'use_height_map': True},
                {'use_colored_map': False},
            ],
            remappings=[
                ('cloud_in', '/kinect/points'),
                ('octomap_binary', '/octomap_3d'),
                ('projected_map', '/map_2d'),
            ],
        ),
        # Добавьте static_transform_publisher если нужно
        #
    ])