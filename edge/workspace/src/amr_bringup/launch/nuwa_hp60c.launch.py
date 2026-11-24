import os
import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
# カメラを上下逆さに付けているため、画像はrotate_imageで回してから決まったトピック名で出す。
ASCAMERA_REMAPPINGS = [
    (f'{PREFIX}/depth0/points', f'{PREFIX}/depth/points'),
]
ROTATE_REMAPPINGS = [
    ('color/image_in', f'{PREFIX}/rgb0/image'),
    ('color/camera_info_in', f'{PREFIX}/rgb0/camera_info'),
    ('color/image', COLOR_IMAGE),
    ('color/camera_info', f'{PREFIX}/color/camera_info'),
    ('depth/image_in', f'{PREFIX}/depth0/image_raw'),
    ('depth/camera_info_in', f'{PREFIX}/depth0/camera_info'),
    ('depth/image', DEPTH_IMAGE),
    ('depth/camera_info', f'{PREFIX}/depth/camera_info'),
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
            # Pi 4では多いとドライバと圧縮の処理が追いつかず、画像が大きく遅れて届く。
            {'fps': 15},
        ],
    )

    # republisherは購読者がいる間だけ入力を購読するため、使わないトピックのためにカメラの配信が始まることはない。
    # use_intra_process_commsは有効にしない(有効だとRepublisherが購読者0になっても入力の購読を止めないことがある)。
    # 回すのと圧縮するのが1つのスレッドを取り合うと、カメラの15fpsに追いつかないため、複数スレッドで回す。
    extras_container = ComposableNodeContainer(
        name='camera_extras_container',
        namespace=NAMESPACE,
        package='rclcpp_components',
        executable='component_container_mt',
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='amr_bringup',
                plugin='amr_bringup::RotateImage',
                name='rotate_image',
                namespace=NAMESPACE,
                remappings=ROTATE_REMAPPINGS,
            ),
            republisher('color_compressed', COLOR_IMAGE, 'compressed'),
            republisher('color_theora', COLOR_IMAGE, 'theora'),
            # 深度は16UC1でJPEG(既定)に変換できないため、16bitを保てるPNGにする。
            republisher('depth_compressed', DEPTH_IMAGE, 'compressed', {'.out.format': 'png'}),
            republisher('depth_compressed_depth', DEPTH_IMAGE, 'compressedDepth'),
        ],
    )

    # カメラが深度の大きく抜けた状態から戻らなくなることがあり、ドライバを開き直すと直る。見張りが終了したら
    # launchごと止め、コンテナの再起動(restart: unless-stopped)でカメラを開き直す。
    # 見張りは回す前の深度を見る。回した深度を購読すると、誰も深度を使っていない間もラズパイで回し続けることになり、
    # 同じラズパイで動くアームのドライバの周期が乱れるため。
    depth_watchdog = Node(
        namespace=f'{NAMESPACE}/{NODE_NAME}',
        package='amr_bringup',
        executable='depth_watchdog.py',
        output='both',
        parameters=[{'upside_down': True}],
        remappings=[('depth/image_raw', f'{PREFIX}/depth0/image_raw')],
    )
    reopen_on_watchdog_exit = RegisterEventHandler(OnProcessExit(
        target_action=depth_watchdog,
        on_exit=[EmitEvent(event=Shutdown(reason='depth watchdog requested reopening the camera'))],
    ))

    return LaunchDescription([ascamera_dir, ascamera_node, extras_container, depth_watchdog, reopen_on_watchdog_exit])
