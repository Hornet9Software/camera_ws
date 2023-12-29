from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch file for pool test.
# Inits 3 camera driver nodes, 1 gate detection node, and 1 pool lines detection node.

camera_init_delay = 10.0

camera_parameters = [
    {"white_balance_temperature_auto": False},
    {"white_balance_temperature": 5500},
    {"brightness": 20},
    {"output_encoding": "yuv422_yuy2"},
    {"exposure_auto": 1},
    {"backlight_compensation": 0},
    {"time_per_frame": "[1,20]"},
]


def generate_launch_description():
    # Declare the paths to camera calibration files
    left_camera_calibration_arg = DeclareLaunchArgument(
        "left_camera_calibration",
        default_value="file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/left.yaml",
        description="Path to left camera calibration YAML file",
    )

    right_camera_calibration_arg = DeclareLaunchArgument(
        "right_camera_calibration",
        default_value="file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/right.yaml",
        description="Path to right camera calibration YAML file",
    )

    bottom_camera_calibration_arg = DeclareLaunchArgument(
        "bottom_camera_calibration",
        default_value="file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/bottom.yaml",
        description="Path to bottom camera calibration YAML file",
    )

    # Camera driver nodes
    leftcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="left",
        namespace="left",
        output="screen",
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_frame_id": "left_camera_frame"},
            {"camera_info_url": LaunchConfiguration("left_camera_calibration")},
            *camera_parameters,
        ],
    )
    rightcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="right",
        namespace="right",
        output="screen",
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_frame_id": "right_camera_frame"},
            {"camera_info_url": LaunchConfiguration("right_camera_calibration")},
            *camera_parameters,
        ],
    )
    bottomcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottom",
        namespace="bottom",
        output="screen",
        parameters=[
            {"video_device": "/dev/video2"},
            {"camera_frame_id": "bottom_camera_frame"},
            {"camera_info_url": LaunchConfiguration("bottom_camera_calibration")},
            *camera_parameters,
        ],
    )

    # Launch lines.py and gate_yolo.py
    lines_node = Node(
        package="camera",
        executable="lines",
        output="screen",
    )
    gate_yolo_node = Node(
        package="camera",
        executable="gate_yolo",
        output="screen",
    )

    return LaunchDescription(
        [
            left_camera_calibration_arg,
            right_camera_calibration_arg,
            bottom_camera_calibration_arg,
            # May not need to delay.
            TimerAction(period=camera_init_delay, actions=[leftcam]),
            TimerAction(period=camera_init_delay * 2, actions=[rightcam]),
            TimerAction(period=camera_init_delay * 3, actions=[bottomcam]),
            lines_node,
            gate_yolo_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
