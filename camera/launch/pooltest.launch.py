from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch file for pool test.
# Inits 3 camera driver nodes, 1 gate detection node, and 1 pool lines detection node.

camera_init_delay = 5.0

camera_parameters = [
    {"width": 640},
    {"height": 480},
    {"codec": "unknown"},
    {"loop": 0},
    {"latency": 2000},
    {"framerate": 20.0},
]

# For v4l2_camera package, to reduce load on driver.
# camera_parameters = [
#     {"white_balance_temperature_auto": False},
#     {"white_balance_temperature": 5500},
#     {"brightness": 20},
#     {"output_encoding": "yuv422_yuy2"},
#     {"exposure_auto": 1},
#     {"backlight_compensation": 0},
#     {"time_per_frame": "[1,20]"},
# ]


def generate_launch_description():
    # Declare the paths to camera calibration files]
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
        package="ros_deep_learning",
        executable="video_source",
        name="left",
        namespace="left",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video0"},
            *camera_parameters,
        ],
    )

    rightcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="right",
        namespace="right",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video4"},
            *camera_parameters,
        ],
    )

    bottomcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="bottom",
        namespace="bottom",
        output="screen",
        parameters=[
            {"resource": "v4l2:///dev/video2"},
            *camera_parameters,
        ],
    )

    # Rectify Node
    rectify_node = Node(
        package="camera",
        executable="calibration",
        output="screen",
    )

    # Launch lines.py and gate_yolo.py
    lines_node = Node(
        package="camera",
        executable="lines",
        output="screen",
    )

    left_gate_yolo_node = Node(
        package="camera",
        executable="gate_yolo",
        output="screen",
        parameters=[
            {"camera": "left"},
        ],
    )

    right_gate_yolo_node = Node(
        package="camera",
        executable="gate_yolo",
        output="screen",
        parameters=[
            {"camera": "right"},
        ],
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

            rectify_node,
            lines_node,
            left_gate_yolo_node,
            right_gate_yolo_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
