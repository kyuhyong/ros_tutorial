# ros tutorials

A Python package for ROS to demonstrate basics of how node implemented in a standard way.

## Installation

git clone this package into the /src under catkin workpsace directory.
If your catkin workspace is named as "catkin_ws" under your home directory,

```bash
cd ~/catkin_ws/src
git clone https://github.com/kyuhyong/ros_tutorial.git [your package name]
cd ..
catkin_make
```

Once catkin_make is done for the first time, make sure to source setup.bash under /devel directory.

```bash
source ~/catkin_ws/devel/setup.bash
```

## Execute nodes

Start run ros_master and then open two terminals.
Entering below command to terminal will start publish a string message as 'msg_tx'

```bash
rosrun ros_tutorials message_publisher.py
```

To subscribe this message run.

```bash
rosrun ros_tutorials message_subscriber.py
```
