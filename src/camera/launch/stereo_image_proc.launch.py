from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        # Declare the paths to left and right camera calibration files
        DeclareLaunchArgument('left_camera_calibration', default_value='file:///home/shengbin/camera_ws/calibration/left.yml',
                              description='Path to left camera calibration YAML file'),
        DeclareLaunchArgument('right_camera_calibration', default_value='file:///home/shengbin/camera_ws/calibration/right.yml',
                              description='Path to right camera calibration YAML file'),

        # Launch the stereo_image_proc node
        ComposableNodeContainer(
            name='stereo_image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_pipeline',
                    plugin='stereo_image_proc::StereoImageProcNode',
                    name='stereo_image_proc',
                    namespace='stereo_image_proc',
                    parameters=[
                        {'calibration_file_left': LaunchConfiguration('left_camera_calibration')},
                        {'calibration_file_right': LaunchConfiguration('right_camera_calibration')}
                    ]
                ),
            ],
            output='screen',
        )
    ])
