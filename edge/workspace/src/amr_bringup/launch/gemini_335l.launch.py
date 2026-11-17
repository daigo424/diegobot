import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gemini = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')),
        launch_arguments={
            'camera_name': 'head_camera',
            # Pi 4のUSBポートは給電が足りず、既定の強さだとレーザーを点けた途端にカメラがUSBから切れる。
            'laser_energy_level': '5',
            # 点群は受け取った側(サーバー)で深度から作る。
            'enable_point_cloud': 'false',
            'depth_width': '848',
            'depth_height': '480',
            'color_width': '1280',
            'color_height': '720',
            # Pi 4では多いとカラーのMJPEG展開と圧縮の処理が追いつかない。
            'depth_fps': '15',
            'color_fps': '15',
        }.items(),
    )
    return LaunchDescription([gemini])
