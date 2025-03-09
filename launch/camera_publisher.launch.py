import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file
    # pkg_path = os.path.join(get_package_share_directory('opencv_ros_camera_py'))
    # eimu_ros_config_file = os.path.join(pkg_path,'config','opencv_ros_camera_params.yaml')

    opencv_ros_camera_node = Node(
        package='opencv_ros_camera',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{'frame_id': "camera_optical",
                      'port_no': 2,
                      'frame_width': 640,
                      'frame_height': 480,
                      'compression_format': "jpeg", # you can also use "jpeg"
                      'publish_frequency': 30.0}
                    ],
    )

     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(opencv_ros_camera_node)
    
    return ld      # return (i.e send) the launch description for excecution
    
