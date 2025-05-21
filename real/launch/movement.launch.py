
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
 

    command_sender_script = os.path.join(get_package_share_directory(pkg_name), 'topics/spider_robot_command_sender.py')

    command_sender = Node(executable=command_sender_script,
            name='command_sender',
            parameters=[{'use_sim_time': False}] )
    
    socket_sender_script = os.path.join(get_package_share_directory(pkg_name), 'topics/socket_transporter.py')

    socket_sender = Node(executable=socket_sender_script,
            name='socket_sender',
            parameters=[{'use_sim_time': False}] )

    # Run the node
    return LaunchDescription([
        command_sender,
        socket_sender
    ])