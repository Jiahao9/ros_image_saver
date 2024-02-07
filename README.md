# ros_image_saver

A ROS package for saving images to create a dataset for deep learning purposes. As a feature of this package, photos are saved every 5 seconds, allowing adjustments such as angles to be made during that time.

## Environments

- ROS OS

## Installation

```bash
# move catkin_ws/src
$ cd ~/catkin_ws/src
# git clone this package
$ git clone https://github.com/Jiahao9/image_saver.git
# build this package
$ cd ~/catkin_ws && catkin_make
```

Camera shutter click sound
```bash
# install pip
$ sudo apt-get install python3-pip
# install pygame
$ pip3 install pygame
```

## usage
```bash
# First, launch your camera driver

# Then, run the node
$ roscore
$ rosrun image_saver image_saver_node.py
```

## Caution
- Please change the topic of the image you want to capture to /rgb/image_raw in the image_saver_node.py file.