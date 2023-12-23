# Description: Launch file for camera driver nodes using usb_cam package
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Launch the USB camera driver nodes
    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='left',
        namespace='left',
        parameters=[{
            'video_device': '/dev/video4',
            'camera_name': 'left_camera',
            'camera_frame_id': 'left_camera_frame',
            'camera_info_url': 'file:///home/shengbin/camera_ws/src/camera/calibration/calibrationdata/left.yaml',
            'pixel_format': 'yuyv2rgb',
        }]
    ))

    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='right',
        namespace='right',
        parameters=[{
            'video_device': '/dev/video2',
            'camera_name': 'right_camera',
            'camera_frame_id': 'right_camera_frame',
            'camera_info_url': 'file:///home/shengbin/camera_ws/src/camera/calibration/calibrationdata/right.yaml',
            'pixel_format': 'yuyv2rgb',
            }]
    ))

    ld.add_action(Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='bottom',
        namespace='bottom',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_name': 'bottom_camera',
            'camera_frame_id': 'bottom_camera_frame',
            'camera_info_url': 'file:///home/shengbin/camera_ws/src/camera/calibration/calibrationdata/bottom.yaml',
            'pixel_format': 'yuyv2rgb',
        }],
    ))

    # Launch bottom image proc for bottom camera
    # Include another launch file
    bottom_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bottom_image_proc.launch.py'])
    )

    ld.add_action(bottom_image_proc)

    # Launch stereo image proc for left and right cameras
    stereo_image_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/stereo_image_proc.launch.py'])
    )

    ld.add_action(stereo_image_proc)


    return ld
