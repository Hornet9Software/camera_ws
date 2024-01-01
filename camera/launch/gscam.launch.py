# Copyright 2022 Clyde McQueen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example pipeline using rclcpp_components.

This launches the gscam and other nodes into a container so that they run in the same process.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node



def generate_launch_description():
    gscam_config = 'gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-raw, width=640, height=480, format=(string)YUY2" ! videoconvert'
    camera_info_url = 'package://gscam/examples/uncalibrated_parameters.ini'


    bottomcam = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_node',
        parameters=[{
            'gscam_config': gscam_config,
            'camera_info_url': camera_info_url,
        }],
        # Future-proof: enable zero-copy IPC when it is available
        # https://github.com/ros-perception/image_common/issues/212
        arguments=[{'use_intra_process_comms': True}],
        namespace="bottom",
        output="screen",
    )

    return LaunchDescription(
        [
            bottomcam
        ]
    )



