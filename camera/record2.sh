#!/bin/bash

# Get current timestamp
timestamp=$(date +%Y%m%d%H%M%S)

# Define filename
filename="pooltest_bags/track/${timestamp}.bag"

# Record the published topics
ros2 bag record -o $filename /sensors/imu /left/image_color_rect /right/image_color_rect /left/gray_world/compressed /gate/finalmask/compressed /object/gate/distance /object/gate/bearing