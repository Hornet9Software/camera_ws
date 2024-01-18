from controls_core.params import bottom_cam_dev, left_cam_dev, right_cam_dev
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

calibration_data_dir = (
    "/home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/"
)


def generate_launch_description():
    # Camera driver nodes
    leftcam = Node(
        package="ros_deep_learning",
        executable="video_source",
        name="left",
        namespace="left",
        output="screen",
        parameters=[
            {"resource": f"v4l2://{left_cam_dev}"},
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
            {"resource": f"v4l2://{right_cam_dev}"},
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
            {"resource": f"v4l2://{bottom_cam_dev}"},
            *camera_parameters,
        ],
    )

    # Rectify Node
    left_rectify_node = Node(
        package="camera",
        executable="calibration",
        namespace="left",
        parameters=[{"calibration_data_path": f"{calibration_data_dir}left.yaml"}],
        output="screen",
    )

    right_rectify_node = Node(
        package="camera",
        executable="calibration",
        namespace="right",
        parameters=[{"calibration_data_path": f"{calibration_data_dir}right.yaml"}],
        output="screen",
    )

    bottom_rectify_node = Node(
        package="camera",
        executable="calibration",
        namespace="bottom",
        parameters=[{"calibration_data_path": f"{calibration_data_dir}bottom.yaml"}],
        output="screen",
    )

    bottom_compressed_node = Node(
        package="camera",
        executable="compressed",
        namespace="bottom",
        output="screen",
    )

    left_compressed_node = Node(
        package="camera",
        executable="compressed",
        namespace="left",
        output="screen",
    )

    right_compressed_node = Node(
        package="camera",
        executable="compressed",
        namespace="right",
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
            # May not need to delay.
            TimerAction(period=camera_init_delay, actions=[leftcam]),
            TimerAction(period=camera_init_delay * 2, actions=[rightcam]),
            TimerAction(period=camera_init_delay * 3, actions=[bottomcam]),
            left_compressed_node,
            right_compressed_node,
            bottom_compressed_node,
            # left_rectify_node,
            # right_rectify_node,
            # bottom_rectify_node,
            # lines_node,
            left_gate_yolo_node,
            right_gate_yolo_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
