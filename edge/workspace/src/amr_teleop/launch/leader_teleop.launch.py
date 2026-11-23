import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('amr_teleop'), 'config', 'leader_teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('follow', default_value='true'),
        # Ctrl+Cの後にコントローラを元に戻す呼び出しを待つため、既定の5秒より長く待ってから強制終了させる。
        Node(package='amr_teleop', executable='leader_teleop', output='screen', sigterm_timeout='10',
             parameters=[config, {'follow': LaunchConfiguration('follow')}]),
    ])
