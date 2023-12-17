#!/bin/bash

trap "kill $LAUNCH_PID" INT

source install/setup.bash

# Run the launch file in the background
ros2 launch /src/camera/launch/cam_driver.launch.py &

# Get the PID of the launch process so we can kill it later
LAUNCH_PID=$!

# Give the launch file some time to start
sleep 5

# Record the published topics
ros2 bag record -a -o /path/to/output.bag