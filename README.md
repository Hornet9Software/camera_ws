# camera_ws
ROS2 Humble package to perform perception and localisation for Hornet 9.0 AUV.

# POOL TEST

## HSV Gate Detection
To debug hsv bounds for detection, run debug_detect.launch.py (which calls debug_detect.py)

    ros2 launch camera debug.launch.py

Remember to update hsv bounds in qualification_gate.py after finding desired hsv bounds.

## Main Task
To test detection, run pooltest.launch.py
This launch file executes 3 camera driver nodes, 1 pool lines detection node and 1 qualification_gate detector node.

      ros2 launch camera pooltest.launch.py

Record rosbags (edit ur script file to record specific topics)

      cd src/camera_ws/camera/record_scripts
      ./recordbags.sh (whichever script file of choice)

All recorded bags will be located at src/camera_ws/camera/pooltest_bags/, rename bags after pool test for easier reference

# First Setup

Install the following utilities:

    sudo apt install libtheora-dev libogg-dev libboost-python-dev guvcview
    sudo apt-get install ros-humble-rqt ros-humble-rqt-common-plugins
    sudo apt install ros-humble-tf-transformations

Install the following packages:

    sudo apt-get install ros-humble-usb-cam
    sudo apt-get install ros-humble-v4l2-camera
    sudo apt-get install ros-humble-vision-opencv
    sudo apt-get install ros-humble-image-common
    sudo apt-get install ros-humble-image-pipeline
    sudo apt-get install ros-humble-image-transport-plugins

    pip3 install scikit-image

    # To enable ML-based Object Detection
    pip install ultralytics

To enable CUDA on the Jetson, add the following to `~/.bashrc`:

    export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

Then, follow the instructions at https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048 to install PyTorch.
For Ultralytics compatibility, best to use PyTorch v2.0.0 and torchvision v0.15.0. Do not install using other pip wheels.
Follow the instructions strictly.

Navigate to workspace

    rosdep install --from-paths src -r -y
    colcon build --symlink-install (remove existing /build /install /log before running this)
    source install/setup.bash

If you are met with 'setup.py install is deprecated' when running colcon build, 
Install the setuptools 58.2.0 version using the following command:
        
    pip install setuptools==58.2.0

Once installed, rerun colcon build:
        
    colcon build 
    source install/setup.bash

Update the path to camera calibration files inside launch files to suit your device. 

Example

    left_camera_calibration_arg = DeclareLaunchArgument(
        'left_camera_calibration', 
        default_value='file:///home/**{user}**/**{workspace name}**/src/camera/calibration/calibrationdata/left.yml',
        description='Path to left camera calibration YAML file'
    )
    
    right_camera_calibration_arg = DeclareLaunchArgument(
        'right_camera_calibration', 
        default_value='file:///home/**{user}**/**{workspace name}**/src/camera/calibration/calibrationdata/right.yml',
        description='Path to right camera calibration YAML file'

> [!NOTE]
> Replace {user} with your device's username.
> Replace {workspace name} with your workspace name


# Objectives

- [ ] Object Detection
- [ ] Pose Estimation from IMU and Depth Sensor
- [ ] Velocity Estimation using Optical Flow
      
~~- [ ] Point Cloud using Stereo Vision~~

~~- [ ] Localisation and Mapping~~

# Current Progress (Object Detection)
Qualification
- [X] Detect Qualification Gate (yellow/orange)
- [ ] Provide Bearing and Distance of Qualification Gate
Task 1
- [ ] Detect Task 1 Gate (Red and Green)
- [ ] Provide Bearing and Distance of Task 1 Gate
- [ ] Detect Orange Flare
- [ ] Provide Bearing and Distance of Orange Flare

Task 2 & 3
- [ ] Detect Buckets (Red, Blue)
- [ ] Pinpoint Center of Bucket

Task 4
- [ ] Detect Flare and classify colour (Red, Green, Blue)
- [ ] Provide Bearing and Distance of Each Flare

## Task 1 - Obstacle Flare
<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/dc0c511f-0dc3-4bbb-ab3e-32b9a07cc8d8" width="500" />

## Task 2 & 3 - Buckets (Red and Blue)
<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/219b9140-c8f1-4126-9e2f-66bacbc49b4b" width="500" />

## Task 4 - Flares
<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/0f2c6411-0ddf-421d-a441-532369c2fe9c" width="500" />


# Current Progress (Mapping)
- [x] USB Camera Driver Node
- [x] Camera Calibration
- [x] Produce Disparity Image
- [ ] Finetune Disparity Image
- [ ] Produce PointCloud (unable to assign frame id to v4l2 camera driver node, one fix could be to use usb_cam as USB Camera Driver Package)

# Packages used
- v4l2camera
- image_pipeline
- vision_opencv
- image_common
- image_transport_plugins
- ros2_shared

[This article](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) details how to build and run the camera driver. It focuses on Raspberry Pi OS with the Raspberry Pi Camera Module V2 but should generalise for most systems.\
[This article](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329) is used for reference in calibrating the cameras. Only modification would be to use v4l2_camera instead of opencv_cam. 

Articles are just for reference, all relevant commands have been compiled below.

## For WSL users
You will have to reconfigure your linux kernel to allow the USB camera to attach to your WSL. [Here's the tutorial](https://www.youtube.com/watch?v=t_YnACEPmrM&t=481s)\
It took me quite a while to get it working so jiayou :) 

## Common bugs
If you encounter an error with the Cmake directory being different from your own directory, delete the build folder and colcon build again.

Personally, I'm unable to get this driver working with my own webcam(possibly incompatiblity or WSL issue), but it's able to work with the usb cameras at the lab. \
So try it out with the cameras first if you can't get your own webcam running.

# Usage

## Specifically for WSL (Skip if you dual-booted)
Open a terminal and run wsl if you don't have one already running.\
Open another powershell in administrator mode and go into it.\
Run ``` usbipd wsl list ```\
Find the busid of the USB camera.\
Run ``` usbipd wsl attach --busid {BUSID} ``` with your specific busid.\
Do this for both cameras.\
Run ```usbipd wsl list ``` again and verify both usb cameras have the status "Attached".\
Continue with main procedure.

## Identify Video Ports for each USB Camera
1) In the WSL or linux terminal, verify you have the usb device connected with ```lsusb```
You should see the camera in your connected USB devices.
2) Then run ```ls /dev/video*```. You should see at least 2 video devices, though its normal to see multiple folders for the same camera.
If you have multiple folders per camera, note that only 1 of the folders is the "real" folder containing the camera. 
3) Verify which folders these are with ``` guvcview -d {DEVICE_NUMBER} ``` , where DEVICE_NUMBER is the integer next to your folder. ex. guvcview -d 0 for /dev/video0.
Take note of the device number of the left and right camera respectively.

## Testing v4l2_camera

Suppose we have two cameras (left camera and right camera).

1) Open two terminals and go to workspace and run
```
  source install/setup.bash
```
2) In one terminal (left camera) run :
```
  ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/left -p video_device:=/dev/video{LEFT_DEVICE_NUMBER} -p camera_info_url:=file:///home/{user}/camera_ws/calibration/left.yml
```
3) In the other terminal (right camera), run :
```
  ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/right -p video_device:=/dev/video{RIGHT_DEVICE_NUMBER} -p camera_info_url:=file:///home/{user}/camera_ws/calibration/right.yml
```
4) Open a third terminal, which will be used to view the two image topics, and run
```
  rqt
```
5) In rqt, go into Plugins > Visualization > Image View.
6) You should now see two image view windows in your rqt. Subscribe one to left/image_raw and the other to right/image_raw

## Testing v4l2_camera launch file

1) Open a terminal and go to workspace and run
```
  source install/setup.bash
  ros2 launch camera stereo.launch.py 
```
2) Open another terminal and run
```
  rqt
```
3) In rqt, go into Plugins > Visualization > Image View to view topics published by both camera driver nodes.

## Get Disparity Image
1) Open a terminal and go to workspace and run stereo.launch.py 
```
  source install/setup.bash
  ros2 launch camera stereo.launch.py 
```
2) Open a second terminal and run rqt to view images from left and right cameras
```
  rqt
```
3) Open a third terminal and run stereo_image_proc.launch.py
```
  ros2 launch camera stereo_image_proc.launch.py 
```
4) Open a fourth terminal and run image_view to view disparity image
```
  ros2 run image_view disparity_view --ros-args --remap image:=/disparity
```

<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/24635edb-9f64-4fcf-a10f-be6e930200e8" width="500" />

To adjust disparity parameters
'''
      
    ros2 run rqt_reconfigure rqt_reconfigure
'''

5) To view the node diagrams, open a new terminal and run
```
  rqt_graph
```

<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/21d3495f-4c80-41d6-949b-c82616d0ebe6" width="500" />

## Procedure to recalibrate camera

To perform calibration, you will need a [chessboard](https://github.com/opencv/opencv/blob/master/doc/pattern.png). 

You can use either a image on a laptop/phone, print it out, or get a physical one if you happen to have one. For mine I printed it out.

There are several types of chessboard patterns for camera calibration, the image above is a 9x6 chessboard pattern.

In a new terminal, run:
```
  colcon build
  source install/setup.bash
```
Now run cameracalibrator:
```
ros2 run camera_calibration cameracalibrator --size=9x6 --square=0.063 --approximate=0.3 --no-service-check --ros-args --remap /left:=/left/image_rect --remap /right:=/right/image_rect
```

NOTE: these parameters assume you're using the image I linked earlier. If you're using another pattern, make sure to adjust size and square parameters accordingly.

For more information on the different parameters, do refer to the image below.

<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/e7636447-db43-4c4c-833b-ccaf73ec6049" width="500" />

You should see a window pop up with the two image views and 3 buttons: Calibrate,Save, Commit.

You will also four progress bars at the top right: X, Y, Size, Skew.

Adjust your pattern until both images light up with coloured lines connecting the corners of each square on the pattern.

Now move the pattern around (sideways for X, up and down for Y, back and forth for Size and tilt from side to side for Skew) until all progress bars are green.

Click "Calibrate" and you will see the config file print out in your terminal.

It should look something like this:
```
[image]

width
640

height
480

[narrow_stereo/left]

camera matrix
485.379981 0.000000 354.463797
0.000000 487.632900 278.138775
0.000000 0.000000 1.000000

distortion
0.423469 -0.796529 -0.010101 0.027240 0.000000

rectification
0.983556 -0.029013 -0.178258
0.029254 0.999571 -0.001274
0.178219 -0.003962 0.983983

projection
671.624840 0.000000 509.165472 0.000000
0.000000 671.624840 270.051386 0.000000
0.000000 0.000000 1.000000 0.000000

[image]

width
640

height
480

[narrow_stereo/right]

camera matrix
508.540260 0.000000 361.956670
0.000000 505.720299 256.264324
0.000000 0.000000 1.000000

distortion
0.363035 -0.330695 -0.013869 0.043509 0.000000

rectification
0.980416 -0.022306 -0.195669
0.022041 0.999751 -0.003534
0.195699 -0.000848 0.980664

projection
671.624840 0.000000 509.165472 -88.265910
0.000000 671.624840 270.051386 0.000000
0.000000 0.000000 1.000000 0.000000
```
Split this into two seperate .ini files (left and right) and save them into your calibration folder (~/camera_ws/calibration).

Convert these .ini files to .yml file using the following command:

    ros2 run camera_calibration_parsers convert left.ini left.yml
    ros2 run camera_calibration_parsers convert right.ini right.yml


## For Object Detection

Edit ~/camera_ws/src/camera/enhance.py to subscribe to desired topic (can be from a rosbag).

Go to workspace and run:
```
  colcon build
  source install/setup.bash
```

In current terminal, run:
```
  ros2 run camera enhance
```

In another terminal, run:
```
  ros2 run camera qualification_gate
```
> [!NOTE]
> this repo does not contain rosbags, you will need to download it yourself from the Software Groupchat

In the third terminal, run ros bag of your choice:
```
  Example: ros2 bag play <path to bag folder of choice>
  Example: ros2 bag play ~/camera_ws/src/camera/bags/competition/
```

You will see a slider window to adjust hsv bounds, you may look at the qualification_gate node to see how to implement the slider window for other nodes. (Eg. to detect red/green/blue flares)

Result:

<img src="https://github.com/ShengBin-101/camera_ws/assets/52733750/30c5a0d3-a484-474a-afc7-3f42617f4502" width="500" />
