from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

# Launch file for pool test.
# Inits 3 camera driver nodes, 1 gate detection node, and 1 pool lines detection node.

cam_init_delay = 5.0


calibration_data_dir = (
    "/home/bb/poolTest_ws/src/camera_ws/camera/calibration/calibrationdata/"
)


def generate_launch_description():
    left_pipeline_group = GroupAction(
        actions=[
            PushRosNamespace("left"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("camera"), "/launch/pipeline.launch.py"]
                ),
                launch_arguments={
                    "cam_name": "left",
                    "cam_init_delay": f"{cam_init_delay}",
                }.items(),
            ),
        ]
    )

    right_pipeline_group = GroupAction(
        actions=[
            PushRosNamespace("right"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("camera"), "/launch/pipeline.launch.py"]
                ),
                launch_arguments={
                    "cam_name": "right",
                    "cam_init_delay": f"{cam_init_delay * 2.0}",
                }.items(),
            ),
        ]
    )

    bottom_pipeline_group = GroupAction(
        actions=[
            PushRosNamespace("bottom"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("camera"), "/launch/pipeline.launch.py"]
                ),
                launch_arguments={
                    "cam_name": "bottom",
                    "cam_init_delay": f"{cam_init_delay * 3.0}",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [left_pipeline_group, right_pipeline_group, bottom_pipeline_group]
    )


if __name__ == "__main__":
    generate_launch_description()
