from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    leftcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="left",
        namespace="left",
        parameters=[
            {"video_device": "/dev/video2"},
            {"camera_info_url": "file:///home/shengbin/camera_ws/calibration/left.yml"}
        ]
    )
    rightcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="right",
        namespace="right",
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_info_url": "file:///home/shengbin/camera_ws/calibration/right.yml"}
        ]
    )
    # bottomcam = Node(
    #     package="v4l2_camera",
    #     executable="v4l2_camera_node",
    #     name="bottomcam",
    #     namespace="bottomcam",
    #     parameters=[
    #         {"video_device": "/dev/video0"},
    #         {"camera_info_url": "file:///home/shengbin/calibration/webcam.yml"}
    #     ]
    # )
    ld.add_action(leftcam)
    ld.add_action(rightcam)
    # ld.add_action(bottomcam)
    return ld