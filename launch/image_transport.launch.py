import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    cam_frame_id = "camera_optical"

    opencv_ros_camera_node = Node(
        package='opencv_ros_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{'frame_id': f'{cam_frame_id}',
                      'port_no': 0,
                      'frame_width': 640,
                      'frame_height': 360,
                      'publish_frequency': 30.0}
                    ],
    )

    image_compress_node = Node(
      package='image_transport',
      executable='republish',
      name='raw_to_compressed_republisher',
      output='screen',
      parameters=[
          {'jpeg_quality': 50} 
      ],
      remappings=[
          ('in', f'/{cam_frame_id}/image'),
          ('out', f'/{cam_frame_id}/image_raw'),
          ('out/compressed', f'/{cam_frame_id}/image_raw/compressed'),
          ('out/compressedDepth', f'/{cam_frame_id}/image_raw/compressedDepth'),
          ('out/theora', f'/{cam_frame_id}/image_raw/theora'),
          ('out/zstd', f'/{cam_frame_id}/image_raw/zstd')
      ]
  )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(opencv_ros_camera_node)
    ld.add_action(image_compress_node)
    
    return ld      # return (i.e send) the launch description for excecution
    
