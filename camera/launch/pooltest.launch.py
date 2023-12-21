from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

# launch file for pool test, 
# inits 3 camera driver nodes, 1 enhance node, 1 qualification_gate_detector node, and stereo_proc node

def generate_launch_description():

    # Include another launch file
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/stereo_image_proc.launch.py'])
    )

    # Declare the paths to left and right camera calibration files
    left_camera_calibration_arg = DeclareLaunchArgument(
        'left_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/left.yaml',
        description='Path to left camera calibration YAML file'
    )
    
    right_camera_calibration_arg = DeclareLaunchArgument(
        'right_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/right.yaml',
        description='Path to right camera calibration YAML file'
    )

    bottom_camera_calibration_arg = DeclareLaunchArgument(
        'bottom_camera_calibration', 
        # relative path to calibration file, with respect to ROS2 workspace
        default_value='file:///home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/bottom.yaml',
        description='Path to bottom camera calibration YAML file'
    )    

    # camera driver nodes
    leftcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="left",
        namespace="left",
        output='screen',
        parameters=[
            {"video_device": "/dev/video4"},
            {"camera_frame_id": "left_camera_frame"},
            {"camera_info_url": LaunchConfiguration('left_camera_calibration')}
        ]
    )
    rightcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="right",
        namespace="right",
        output='screen',
        parameters=[
            {"video_device": "/dev/video2"},
            {"camera_frame_id": "right_camera_frame"},
            {"camera_info_url": LaunchConfiguration('right_camera_calibration')}
        ]
    )
    bottomcam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottom",
        namespace="bottom",
        output='screen',
        parameters=[
            {"video_device": "/dev/video0"},
            {"camera_frame_id": "bottom_camera_frame"},
            {"camera_info_url": LaunchConfiguration('bottom_camera_calibration')}
        ]
    )

    image_proc_node = Node(
        package="image_proc",
        executable="image_proc",
        name="image_proc_bottom",
        namespace="bottom",
        output='screen',
        remappings=[
            ("image_raw", "/bottom/image_raw"),
            ("camera_info", "/bottom/camera_info")
        ]
    )
    enhance_node = Node(
        package="camera",
        executable="enhance",
        output='screen',
    )
    qualification_gate_detector = Node(
        package="camera",
        executable="qualification_gate",
        output='screen',
    )

    # Add your nodes to the launch description
    return LaunchDescription([
        left_camera_calibration_arg, 
        right_camera_calibration_arg, 
        bottom_camera_calibration_arg,
        leftcam,
        rightcam,
        bottomcam,
        image_proc_node,
        included_launch,
        enhance_node,
        qualification_gate_detector,
    ])

if __name__ == '__main__':
    generate_launch_description()