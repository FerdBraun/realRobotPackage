from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    pkg_name = 'real'
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('real'),
            'description',
            'nav2_params.yaml'
        ]),
        description='Full path to the Nav2 parameters file'
    )
    nav2_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'tf_buffer_size': '120',  # Добавьте это
            'tf_timeout': '1.0'      # И это
        }.items()
    )
    # Путь к скрипту преобразования команд скорости
    cmd_vel_converter_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'cmd_vel_to_spider_commands.py'
    )
    mapnav2_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'nav2MapTranslator.py'
    )
    mapnav2 = Node(
        executable=mapnav2_script,
        name='mapnav2',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )
    cmd_vel_converter = Node(
        executable=cmd_vel_converter_script,
        name='cmd_vel_to_spider_commands',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )

    # Узлы NAV2
    planner_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'planner.py'
    )

    planner= Node(
        executable=planner_script,
        name='cmd_vel_quantizer',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )

    return LaunchDescription([
                                mapnav2,
                                use_sim_time,
                                params_file,
                                nav2_launch,
                                cmd_vel_converter,
                                planner
                              ] )
