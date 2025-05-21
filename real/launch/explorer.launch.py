from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'real'
    

    explorer_script = os.path.join(
        get_package_share_directory(pkg_name),
        'topics',
        'autonomous_explorer.py'
    )

    explorer= Node(
        executable=explorer_script,
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': False}],

    )

    return LaunchDescription([
                              explorer,
                              ] )