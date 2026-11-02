import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_launch = os.path.join(get_package_share_directory('amr_moveit_config'), 'launch')
    config = os.path.join(get_package_share_directory('amr_manipulation'), 'config', 'manipulator.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # 実機のアームはラズパイのros2_controlが動かし、ここでは計画と指令だけを行う。
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit_launch, 'move_group.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit_launch, 'moveit_rviz.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('use_rviz'))),
        Node(package='amr_manipulation', executable='manipulator', output='screen', parameters=[config]),
    ])
