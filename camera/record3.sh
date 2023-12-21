#!/bin/bash

# Get current timestamp
timestamp=$(date +%Y%m%d%H%M%S)

# Define filename
filename="pooltest_bags/imu/${timestamp}.bag"

# Record the published topics
ros2 bag record -o $filename /sensors/imu /left/image_raw/compressed /right/image_raw/compressed /bottom/image_raw/compressed