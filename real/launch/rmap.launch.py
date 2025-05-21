from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'real'
    frame_id = LaunchConfiguration('frame_id', default='base_link')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rgb_topic = LaunchConfiguration('rgb_topic', default='/kinect/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/kinect/depth/image_raw')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/kinect/points')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/kinect/camera_info')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu')
    delete_db_on_start=LaunchConfiguration('delete_db_on_start', default='true')
    
    rtab_script = os.path.join(get_package_share_directory(pkg_name), 'topics/rtabRelay.py')
    
    rtab = Node(
        executable=rtab_script,
        name='imu',
        parameters=[{'use_sim_time': False}]
    )
    

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('rgb_topic', default_value=rgb_topic),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic),
        DeclareLaunchArgument('imu_topic', default_value=imu_topic),
        DeclareLaunchArgument('delete_db_on_start', default_value=delete_db_on_start),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ]),
            launch_arguments={
                'frame_id': frame_id,
                'subscribe_depth': 'true',  # Отключаем depth image
                'Grid/Sensor': 'True', # ОШИБКА, GRID SENSOR НЕ РАБОТАЕТ
                'subscribe_scan_cloud': 'false',
                'subscribe_rgb': 'true',
                'subscribe_scan': 'false',
                'subscribe_odom': 'false',
                'qos': '2',
                'use_sim_time': use_sim_time,
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.02',
                'wait_for_transform': '0.20',
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'imu_topic': imu_topic,
                'cloud_topic': cloud_topic,
                
                # Оптимизированные параметры для уменьшения шума и улучшения очистки
                'Rtabmap/DetectionRate': '3.0',  # Увеличили частоту детекции
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/AngularUpdate': '0.05',  # Увеличили для более плавных обновлений
                'RGBD/LinearUpdate': '0.05',   # Увеличили для более плавных обновлений
                'RGBD/OptimizeFromGraphEnd': 'false',
                'Reg/Strategy': '1',  # 1=Vis, 0=ICP
                'Vis/FeatureType': '6',  # ORB features
                'Vis/MaxFeatures': '1000',  # Уменьшили количество фич для уменьшения шума
                'Vis/MinInliers': '20',  # Увеличили минимальное количество inliers
                'Vis/CorType': '1',  # 1=FAST/FREAK
                'Vis/PnPRefineIterations': '3',  # Увеличили количество итераций
                'Vis/EstimationType': '1',  # 1=PnP
                
                'imu_local_transform': 'true',
                'Optimizer/GravitySigma': '0.1',
                'RGBD/ProximityPathFiltering': 'true',
                'RGBD/ProximityPathMaxNeighbors': '5',  # Уменьшили количество соседей
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityByTime': 'false',
                
                # Параметры памяти и очистки
                'Mem/STMSize': '20',  # Уменьшили размер кратковременной памяти
                'Mem/BadSignaturesIgnored': 'true',
                'Mem/ReduceGraph': 'true',
                'Mem/IncrementalMemory': 'false',
                'Mem/RehearsalSimilarity': '0.5',  # Более агрессивная очистка
                'Mem/NotLinkedNodesKept': 'false',  # Удаляем не связанные узлы
                'Mem/LoopRatio': '0.8',  # Более агрессивное объединение похожих узлов
                
                'RGBD/LocalRadius': '1.5',  # Уменьшили радиус локальной карты
                'Reg/Force3DoF': 'true',
                'RGBD/OptimizeMaxError': '2.0',  # Уменьшили максимальную ошибку
                'Optimizer/Robust': 'true',
                'Optimizer/Iterations': '20',  # Увеличили количество итераций
                
                # Параметры сетки
                'Grid/FromDepth': 'true',
                'Grid/3D': 'true',
                'Grid/CellSize': '0.05',  # Размер ячейки сетки
                'Grid/RayTracing': 'true',
                'Grid/RayTracingRange': '3.0',  # Дистанция трассировки лучей
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/MinGroundHeight': '-0.1',
                'Grid/NoiseFilteringRadius': '0.3',  # Фильтрация шума
                'Grid/NoiseFilteringMinNeighbors': '10',  # Минимальное количество соседей
                'Grid/NormalSegmentation': 'true',
                'Grid/MaxGroundAngle': '30',
                'Grid/ClusterRadius': '0.2',
                'Grid/MinClusterSize': '20',
                'Grid/FloorHeight': '-0.07',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'publish_tf': 'true',
                'grid_published': 'true',
                'grid_map_topic': '/map',
                'grid_map_publish_rate': '1.0',
                'map_always_update': 'true',
                'map_negative_poses_ignored': 'false',
                'map_negative_scan_empty_ray_tracing': 'true',
            }.items()
        ),
        rtab
    ])
