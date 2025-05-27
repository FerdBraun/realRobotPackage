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
    rtab_script = os.path.join(get_package_share_directory(pkg_name), 'topics/rtabRelay.py')
    
    rtab = Node(
        executable=rtab_script,
        name='rtab_relay',
        parameters=[{'use_sim_time': False}]
    )
    

    return LaunchDescription([
      IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('real'),
                'launch',
                'rtabmap.launch.py'
            ]),
            
           
        ),
        rtab
    ])
