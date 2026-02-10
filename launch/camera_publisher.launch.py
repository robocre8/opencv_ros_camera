import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cam_frame_id = "camera_optical"
    
    camera_info_file = os.path.join(
        get_package_share_directory('opencv_ros_camera'),
        'config',
        'camera_info.yaml'
    )

    opencv_ros_camera_node = Node(
        package='opencv_ros_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{'frame_id': f'{cam_frame_id}',
                      'port_no': 2,
                      'frame_width': 640,
                      'frame_height': 360,
                      'publish_frequency': 30.0
                    },
                    {
                      'camera_info_url': f'file://{camera_info_file}'
                    }
                    ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(opencv_ros_camera_node)
    
    return ld      # return (i.e send) the launch description for excecution
