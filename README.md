# infinitam_ros branch forked from [InfiniTAM v3](https://github.com/victorprad/InfiniTAM)

This repository is a fork of [InfiniTAM v3](https://github.com/victorprad/InfiniTAM) and adds the infinitam_ros branch that enables ROS as an input source to the InfiniTAM system. 

# Building the System

## System Requirements

The following 3rd party libraries are needed for compiling InfiniTAM. The given version numbers are checked and working, but different versions might be fine as well. 

  - cmake (e.g. version 2.8.10.2 or 3.2.3)
    REQUIRED for Linux, unless you write your own build system
    OPTIONAL for MS Windows, if you use MSVC instead
    available at http://www.cmake.org/

  - OpenGL / GLUT (e.g. freeglut 2.8.0 or 3.0.0)
    REQUIRED for the visualisation
    the library should run without
    available at http://freeglut.sourceforge.net/

  - CUDA (e.g. version 6.0 or 7.0)
    OPTIONAL but REQUIRED for all GPU accelerated code
    at least with cmake it is still possible to compile the CPU part without
    available at https://developer.nvidia.com/cuda-downloads

  - ROS (tested on indigo, but should work on other versions too).

  - Boost

## Build Process (for infinitam_ros branch)

infinitam\_ros is catkinized, so you can clone it into your catkin workspace's src/ directory and run `catkin_make' from the workspace directory. 

If you are new to the ROS catkin build system or want to build and run InfiniTAM in a separate directory, you can do the following: 

1. Create a new catkin workspace (replace indigo with your ROS version)
```
$ source /opt/ros/indigo/setup.bash

$ mkdir -p ~/infinitam_ws/src
$ cd ~/infinitam_ws/
$ catkin_make
$ source devel/setup.bash
```

2. Clone infinitam_ros into the src/ dir
```
$ cd ~/infinitam_ws/src/
$ git clone git@github.com:ravich2-7183/InfiniTAM.git
$ git checkout infinitam_ros
```

3. Build infinitam_ros
```
$ cd ~/infinitam_ws/
$ catkin_make
```

4. Run infinitam_ros
```
$ cd ~/infinitam_ws/
$ source devel/setup.bash
$ rosrun InfiniTAM InfiniTAM
```

# Expected input topics
The InfiniTAM node subscribes to the following topics:

1. /camera/depth/image_raw

of type sensor_msgs/Image with encoding "mono16" or "16UC1" (depth is in millimeters). 

2. /camera/rgb/image_color

of type sensor_msgs/Image with encoding  "bgr8" (a common ROS Image message encoding for color images). 
