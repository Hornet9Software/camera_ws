#!/bin/bash

# Get current timestamp
timestamp=$(date +%Y%m%d%H%M%S)

# Define filename
filename="pooltest_bags/${timestamp}.bag"

# Record the published topics
ros2 bag record -o $filename /left/camera_info /left/image_raw/compressed /left/image_rect /left/image_rect_color /right/camera_info /right/image_raw/compressed /right/image_rect /right/image_rect_color /bottom/camera_info /bottom/image_raw/compressed
