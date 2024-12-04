from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[
                {'image_width': 640, 'image_height': 480, 'framerate': 30.0, 'io_method': 'mmap'}
            ]
        ),
        Node(
            package='image_conversion_pkg',
            executable='image_conversion_node',
            name='image_conversion_node',
            parameters=[
                {'input_topic': '/image_raw', 'output_topic': '/converted_image'}
            ]
        ),
    ])
