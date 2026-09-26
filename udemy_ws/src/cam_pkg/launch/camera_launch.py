from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam_pkg',
            namespace='camera_publisher',
            executable='camera_publisher',
            name='main',
            remappings=[
            ('/camera_publisher/video_topic', '/video_topic'),
        ]
        ),
        Node(
            package='cam_pkg',
            namespace='camera_subcriber',
            executable='camera_subcriber',
            name='main',
            remappings=[
            ('/camera_subcriber/video_topic', '/video_topic'),
        ]
        )
    ])