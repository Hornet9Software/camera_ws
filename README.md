# camera_ws
Trial and Error repo for reading stereo image from camera & publishing. Now able to calibrate cameras.
# Packages used
[v4l2camera](https://github.com/tier4/ros2_v4l2_camera/tree/galactic)
[image_pipeline](https://github.com/ros-perception/image_pipeline.git)

[This article](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) details how to build and run the camera driver. It focuses on Raspberry Pi OS with the Raspberry Pi Camera Module V2 but should generalise for most systems.
[I used this](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329) for reference in calibrating the cameras. Only modification would be to use v4l2_camera instead of opencv_cam. 

Articles are just for reference, all relevant commands have been compiled below.

# How to install

First make sure you've installed the following utilities:
```
    sudo apt install libtheora-dev libogg-dev libboost-python-dev guvcview rqt
    sudo apt-get install ros-humble-rqt ros-humble-rqt-common-plugins
```
Then git clone this repo into your root directory(or whatever project directory you use) and go into the camera_ws workspace.
Go into the /src folder of the worksapce and run the following script:

    git clone --branch humble https://gitlab.com/boldhearts/ros2_v4l2_camera.git
    git clone --branch humble https://github.com/ros-perception/vision_opencv.git
    git clone --branch humble https://github.com/ros-perception/image_pipeline.git
    git clone --branch humble https://github.com/ros-perception/image_common.git
    git clone --branch humble https://github.com/ros-perception/image_transport_plugins.git
    git clone https://github.com/ptrmu/ros2_shared.git
    cd ..
    rosdep install --from-paths src -r -y
    colcon build 
    source install/local_setup.bash

## For WSL users:
You will have to reconfigure your linux kernel to allow the USB camera to attach to your WSL. [Here's the tutorial](https://www.youtube.com/watch?v=t_YnACEPmrM&t=481s)
It took me quite a while to get it working so jiayou :) 

## Common bugs
If you encounter an error with the Cmake directory being different from your own directory, delete the build folder and colcon build again.

Personally, I'm unable to get this driver working with my own webcam(possibly incompatiblity or WSL issue), but it's able to work with the usb cameras at the lab. 
So try it out with the cameras first if you can't get your own webcam running.

# Usage

## Specifically for WSL:
Open a terminal and run wsl if you don't have one already running.
Open another powershell in administrator mode and go into it.
Run ``` usbipd wsl list ```
Find the busid of the USB camera.
Run ``` usbipd wsl attach --busid {BUSID} ``` with your specific busid.
Do this for both cameras.
Run ```usbipd wsl list ``` again and verify both usb cameras have the status "Attached".
Continue with main procedure.

## Main procedure
1) In the WSL or linux terminal, verify you have the usb device connected with ```lsusb```
You should see the camera in your connected USB devices.
2) Then run ```ls /dev/video*```. You should see at least 2 video devices, though its normal to see multiple folders for the same camera.
If you have multiple folders per camera, note that only 1 of the folders is the "real" folder containing the camera. 
3) Verify which folders these are with ``` guvcview -d {DEVICE_NUMBER} ``` , where DEVICE_NUMBER is the integer next to your folder. ex. guvcview -d 0 for /dev/video0.
Take note of the device number of the left and right camera respectively. 
4) Start one more tab in your terminal, going into the same workspace and doing the standard procedure of
```
  source /opt/ros/humble/setup.bash
  colcon build
  source install/local_setup.bash
```
You now have a "left camera" terminal and a "right camera" terminal.
For the following two steps, use the left and right device numbers you got in step 3.
5) In "left camera", run :
```
  ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/left -p video_device:=/dev/video{LEFT_DEVICE_NUMBER} -p camera_info_path:=left.ini
```
6) In "right camera",run:
```
  ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/right -p video_device:=/dev/video{RIGHT_DEVICE_NUMBER} -p camera_info_path:=right.ini
```
7) Open a third terminal, "view" terminal, and run
```
  source /opt/ros/humble/setup.bash
  rqt
```
8) In rqt, go into Plugins > Visualization > Image View.
9) You should now see two image view windows in your rqt. Subscribe one to left/image_raw and the other to right/image_raw
If you wish to recalibrate the camera, proceed to next section. Otherwise congrats, you've now successfully gotten stereo vision!

## Procedure to recalibrate camera
Make sure you did all the steps in the previous section ( Main Procedure ).
To perform calibration, you will need a [chessboard](https://github.com/opencv/opencv/blob/master/doc/pattern.png). You can use either a image on a laptop/phone, print it out, or get a physical one if you happen to have one. For mine I printed it out.
In "view" terminal, exit out of rqt.
Do standard procedure in workspace:
```
  colcon build
  source install/local_setup.bash
```
Now run:
```ros2 run camera_calibration cameracalibrator --size=8x6 --square=0.063 --approximate=0.3 --no-service-check --ros-args --remap /left:=/left/image_rect --remap /right:=/right/image_rect```
NOTE: these parameters assume you're using the image I linked earlier. If you're using another pattern, make sure to adjust size and square parameters accordingly.
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
Split this into two seperate .ini files (left and right) and save them into your workspace folder.
Now when running the camera, it will read these files to calibrate the camera.

#Running of Nodes
    cd ~/camera_ws/
    colcon build
    source install/setup.bash
    ros2 run v4l2_camera v4l2_camera_node
Open new terminal to run video processor node.

    ros2 run camera cam_process
Open new terminal to run rqt to view images.

    rqt
Open new terminal to run rqt_graph to view nodes and topics.

    rqt_graph
