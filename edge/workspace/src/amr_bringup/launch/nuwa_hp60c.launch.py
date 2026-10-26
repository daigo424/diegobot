import os
import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Orbbec系(orbbec_camera)のカメラと同じトピック名に揃え、カメラを差し替えても購読側を変えずに済むようにする。
NAMESPACE = 'camera'
NODE_NAME = 'camera'
PREFIX = f'/{NAMESPACE}/{NODE_NAME}'
COLOR_IMAGE = f'{PREFIX}/color/image_raw'
DEPTH_IMAGE = f'{PREFIX}/depth/image_raw'
# ascamera_nodeは見つけた順に末尾の番号を振るため、最初に見つかったカメラ(0)を前提にする。
ASCAMERA_REMAPPINGS = [
    (f'{PREFIX}/rgb0/image', COLOR_IMAGE),
    (f'{PREFIX}/rgb0/camera_info', f'{PREFIX}/color/camera_info'),
    (f'{PREFIX}/depth0/image_raw', DEPTH_IMAGE),
    (f'{PREFIX}/depth0/camera_info', f'{PREFIX}/depth/camera_info'),
    (f'{PREFIX}/depth0/points', f'{PREFIX}/depth/points'),
]


def republisher(name, in_topic, out_transport, extra_params=None):
    # ascamera_nodeはimage_transportを使わず生のImageしか出さないため、圧縮トピックを外付けで作る。
    return ComposableNode(
        package='image_transport',
        plugin='image_transport::Republisher',
        name=name,
        namespace=NAMESPACE,
        parameters=[{'in_transport': 'raw', 'out_transport': out_transport, **(extra_params or {})}],
        # 出力はプラグインが`out/<transport名>`に直接advertiseするため、`out`だけのremapは効かない。
        remappings=[('in', in_topic), (f'out/{out_transport}', f'{in_topic}/{out_transport}')],
    )


def generate_launch_description():
    # ascameraは設定ファイルもSDKの.soもインストールせず、ソースツリー上のものを参照する作りのため、
    # ソースの場所を直接指定する。
    ascamera_dir = DeclareLaunchArgument(
        'ascamera_dir',
        default_value='/workspace/src/vendor/angstrong/ascamera',
    )
    ascamera_dir_path = LaunchConfiguration('ascamera_dir')

    ascamera_node = Node(
        namespace=NAMESPACE,
        package='ascamera',
        executable='ascamera_node',
        name=NODE_NAME,
        remappings=ASCAMERA_REMAPPINGS,
        respawn=True,
        output='both',
        # ascamera_nodeはSDKの.soへのRPATHを持たないため、ここで探索パスに加える。
        additional_env={
            'LD_LIBRARY_PATH': [
                ascamera_dir_path,
                f'/libs/lib/{platform.machine()}-linux-gnu:',
                os.environ.get('LD_LIBRARY_PATH', ''),
            ],
        },
        parameters=[
            {'usb_bus_no': -1},
            {'usb_path': 'null'},
            # 接続されたカメラの機種に合う設定ファイルを、このディレクトリから自動で選ぶ。
            {'confiPath': [ascamera_dir_path, '/configurationfiles']},
            {'color_pcl': False},
            {'pub_tfTree': True},
            {'depth_width': 640},
            {'depth_height': 480},
            {'rgb_width': 640},
            {'rgb_height': 480},
            {'fps': 30},
        ],
    )

    # republisherは購読者がいる間だけ入力を購読するため、使わないトピックのためにカメラの配信が始まることはない。
    # use_intra_process_commsは有効にしない(有効だとRepublisherが購読者0になっても入力の購読を止めないことがある)。
    extras_container = ComposableNodeContainer(
        name='camera_extras_container',
        namespace=NAMESPACE,
        package='rclcpp_components',
        executable='component_container',
        output='both',
        composable_node_descriptions=[
            republisher('color_compressed', COLOR_IMAGE, 'compressed'),
            republisher('color_theora', COLOR_IMAGE, 'theora'),
            # 深度は16UC1でJPEG(既定)に変換できないため、16bitを保てるPNGにする。
            republisher('depth_compressed', DEPTH_IMAGE, 'compressed', {'.out.format': 'png'}),
            republisher('depth_compressed_depth', DEPTH_IMAGE, 'compressedDepth'),
        ],
    )

    return LaunchDescription([ascamera_dir, ascamera_node, extras_container])
