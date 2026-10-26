from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

PREFIX = '/camera/camera'


def generate_launch_description():
    # 点群は1フレーム約10MBありWi-Fiでは重いため、ロボットからは圧縮画像だけを受け取り、こちらで組み立てる。
    point_cloud_xyzrgb = ComposableNode(
        package='depth_image_proc',
        plugin='depth_image_proc::PointCloudXyzrgbNode',
        name='point_cloud_xyzrgb',
        namespace='camera',
        parameters=[{
            'image_transport': 'compressed',
            'depth_image_transport': 'compressedDepth',
            'exact_sync': True,
        }],
        # HP60Cの深度はRGBに位置合わせ済みで出てくるため、そのままRGBと重ねて色付き点群にできる。
        remappings=[
            ('rgb/image_rect_color', f'{PREFIX}/color/image_raw'),
            ('rgb/camera_info', f'{PREFIX}/color/camera_info'),
            ('depth_registered/image_rect', f'{PREFIX}/depth/image_raw'),
            ('points', f'{PREFIX}/depth_registered/points'),
        ],
    )

    container = ComposableNodeContainer(
        name='perception_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        composable_node_descriptions=[point_cloud_xyzrgb],
    )

    # TensorRTエンジン(export_engineで作った.engine)もここで指定する。
    model = DeclareLaunchArgument('model', default_value='yoloe-26s-seg-pf.pt')
    object_detector = Node(
        package='amr_perception',
        executable='object_detector',
        output='both',
        parameters=[{'model': LaunchConfiguration('model')}],
        remappings=[('image/compressed', f'{PREFIX}/color/image_raw/compressed')],
    )

    return LaunchDescription([model, container, object_detector])
