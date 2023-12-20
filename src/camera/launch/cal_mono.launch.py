from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This launch file launches the camera driver nodes for calibration, hence no path to yaml file is specified

def generate_launch_description():
    # Declare the paths to left and right camera calibration files
    camera_calibration_arg = DeclareLaunchArgument(
        'mono_camera_calibration', 
        default_value='',
        description='Path to right camera calibration YAML file'
    )

    # camera driver nodes
    cam = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="bottom",
        namespace="bottom",
        parameters=[
            {"video_device": "/dev/video6"},
            {"camera_info_url": LaunchConfiguration('mono_camera_calibration')}
        ]
    )

    # Add your nodes to the launch description
    return LaunchDescription([
        camera_calibration_arg,
        cam,
    ])

if __name__ == '__main__':
    generate_launch_description()
