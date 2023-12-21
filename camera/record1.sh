#!/bin/bash

# Get current timestamp
timestamp=$(date +%Y%m%d%H%M%S)

# Define filename
filename="pooltest_bags/detect/${timestamp}.bag"

# Record the published topics
ros2 bag record -o $filename /left/camera_info /right/camera_info /bottom/camera_info /left/image_raw /right/image_raw /bottom/image_raw /left/image_rect_color /right/image_rect/color /left/gray_world/compressed /gate/finalmask/compressed /object/gate/distance /object/gate/bearing