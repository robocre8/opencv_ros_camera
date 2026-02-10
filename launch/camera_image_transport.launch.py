import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------
    # Launch Arguments
    # -----------------------------
    cam_frame_id = LaunchConfiguration('cam_frame_id')
    port_no = LaunchConfiguration('port_no')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    publish_frequency = LaunchConfiguration('publish_frequency')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    camera_info_file = LaunchConfiguration('camera_info_file')

    declare_cam_frame_id = DeclareLaunchArgument(
        'cam_frame_id',
        default_value='camera_optical',
        description='Camera frame ID'
    )

    declare_port_no = DeclareLaunchArgument(
        'port_no',
        default_value='0',
        description='Camera device port number'
    )

    declare_image_width = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )

    declare_image_height = DeclareLaunchArgument(
        'image_height',
        default_value='360',
        description='Camera image height'
    )

    declare_publish_frequency = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='Camera publish frequency (Hz)'
    )

    declare_jpeg_quality = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='50',
        description='JPEG compression quality (0–100)'
    )

    declare_camera_info_file = DeclareLaunchArgument(
        'camera_info_file',
        default_value=os.path.join(get_package_share_directory('opencv_ros_camera'), 'config', 'camera_info.yaml'),
        description='yaml file containig the camera calibration data'
    )

    # -----------------------------
    # Camera Node
    # -----------------------------
    opencv_ros_camera_node = Node(
        package='opencv_ros_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
                'frame_id': cam_frame_id,
                'port_no': port_no,
                'frame_width': image_width,
                'frame_height': image_height,
                'publish_frequency': publish_frequency,
            },
            {
                'camera_info_url': ['file://', camera_info_file]
            }
        ],
    )

    # -----------------------------
    # Image Transport Republisher
    # -----------------------------
    image_compress_node = Node(
        package='image_transport',
        executable='republish',
        name='raw_to_compressed_republisher',
        output='screen',
        parameters=[{
            'jpeg_quality': jpeg_quality
        }],
        remappings=[
            ('in', [ '/', cam_frame_id, '/image' ]),
            ('out', [ '/', cam_frame_id, '/image_raw' ]),
            ('out/compressed', [ '/', cam_frame_id, '/image_raw/compressed' ]),
            ('out/compressedDepth', [ '/', cam_frame_id, '/image_raw/compressedDepth' ]),
            ('out/theora', [ '/', cam_frame_id, '/image_raw/theora' ]),
            ('out/zstd', [ '/', cam_frame_id, '/image_raw/zstd' ]),
        ]
    )

    # -----------------------------
    # Launch Description
    # -----------------------------
    return LaunchDescription([
        declare_cam_frame_id,
        declare_port_no,
        declare_image_width,
        declare_image_height,
        declare_publish_frequency,
        declare_jpeg_quality,
        declare_camera_info_file,
        opencv_ros_camera_node,
        image_compress_node
    ])
