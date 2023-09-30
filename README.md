# ros tutorials -b feature/img_capture

This project is a sub-branch of my ROS tutorials package.
This simple python package demonstrates how to subscribe a ROS image message and save image as a file.

## Requirements

- A USB-camera or similar
- For ROS-melodic
  - UVC-camera package which can be installed by
  - ```$ sudo apt install ros-melodic-uvc-camera```
- For ROS-noetic
  - USB-cam package which can be installed by
  - ```$ sudo apt install ros-noetic-usb-cam```

## Installation

git clone this package into the /src under catkin workpsace directory.
If your catkin workspace is named as "catkin_ws" under your home directory,

```bash
cd ~/catkin_ws/src
git clone https://github.com/kyuhyong/ros_tutorial.git -b feature/img_capture [your package name]
cd ..
catkin_make
```

Once catkin_make is done for the first time, make sure to source setup.bash under /devel directory.

```bash
source ~/catkin_ws/devel/setup.bash
```

## How To Use

Start run ros_master and then open two terminals.

### Launch camera node

Plug in your usb camera to the laptop or PC then check if the camera is detected under /dev/video#
Modify parameters in start_camera.launch per video path.

```bash
roslaunch image_capture start_camera.launch
```

### Launch image_capture node

Modify start_capture.launch file for your choice

- **file_name** : Captured image will be saved to ~/image_capture/<file_name>/<file_name>_count.jpg
- **image_path** : Image path to subscribe message from usb camera
- **image_count_from** : Starting number for <file_name>_[count].jpg

then enter below command

```bash
roslaunch image_capture start_capture.launch
```

### Capture Image

Keyboard control

- Press 'c' key to capture an image
- Press 'q' key to quit program