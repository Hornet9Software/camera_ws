from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    leftcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="leftcam",
        namespace="leftcam",
        parameters=[
            {"video_device": "/dev/video2"},
            {"camera_info_path": "left.ini"}
        ]
    )
    rightcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="rightcam",
        namespace="rightcam",
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_info_path": "right.ini"}
        ]
    )
    bottomcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottomcam",
        namespace="bottomcam",
        parameters=[
            {"video_device": "/dev/video0"}
        ]
    )
    ld.add_action(leftcam)
    ld.add_action(rightcam)
    ld.add_action(bottomcam)
    return ld