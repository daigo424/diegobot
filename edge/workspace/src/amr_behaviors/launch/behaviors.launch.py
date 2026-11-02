import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manipulation = os.path.join(get_package_share_directory('amr_manipulation'), 'launch', 'manipulation.launch.py')
    config = os.path.join(get_package_share_directory('amr_behaviors'), 'config', 'behavior_server.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(manipulation),
                                 launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items()),
        Node(package='amr_behaviors', executable='behavior_server', output='screen', parameters=[config]),
    ])
