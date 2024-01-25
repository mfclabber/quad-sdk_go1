## Quad-SDK Unitree Go1
![image](https://github.com/mfclabber/quad-sdk_go1/assets/118126641/09f7f5fa-0c98-4aea-8e35-a7afb66cd213)

## Install ([unitree_legged_sdk v3.8.6](https://github.com/unitreerobotics/unitree_legged_sdk.git)):
Before starting the installation, familiarize yourself with the [basic project](https://github.com/robomechanics/quad-sdk/wiki)
```
cd <path_to_quad-sdk>/src/quad-sdk-go1
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
```
Build
```
cd unitree_legged_sdk
mkdir build 
cd build 
cmake .. && make
cd <path_to_quad-sdk>
catkin build || catkin_make
```

## Run real Unitree go1

Pre-configure a wired connection over UDP->Ethernet
![image](https://github.com/mfclabber/quad-sdk_go1/assets/118126641/7834b7db-a54b-4788-acf5-c27d6c5c633c)


Launching visualization, controller and driver:
```
roslaunch quad_utils test_remote_driver.launch
```

The command for the robot to stand up:
```
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
```

Launching a local glider:
```
roslaunch quad_utils quad_plan.launch reference:=twist logging:=true
```

Keyboard control via a ROS packet that sends the direction of movement to the local glider:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel
```

## Overview

Quad-SDK is an open source, ROS-based full stack software framework for agile quadrupedal locomotion. The design of Quad-SDK is focused on the vertical integration of planning, control, estimation, communication, and development tools which enable agile quadrupedal locomotion in simulation and hardware with minimal user changes for multiple platforms. The modular software architecture allows researchers to experiment with their own implementations of different components while leveraging the existing framework. Quad-SDK also offers Gazebo simulation support and a suite of visualization and data-processing tools for rapid development. Refer to the [paper] for high-level details of the framework.

**Keywords:** Legged Robotics, Quadrupeds, Planning, Control, Leaping, ROS

### License

The source code is released under a [MIT License](LICENSE).

**Authors: Joe Norby, Yanhao Yang, Ardalan Tajbakhsh, Jiming Ren, Justin K. Yim, Alexandra Stutt, Qishun Yu, Nikolai Flowers, and Aaron M. Johnson<br />
Affiliation: [The Robomechanics Lab at Carnegie Mellon University](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainer: Ardalan Tajbakhsh, atajbakh@andrew.cmu.edu**

The packages in Quad-SDK have been tested under [ROS] Melodic on Ubuntu 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
https://git.sberrobots.ru/quadruped_memorandum/quad-sdk-go1
## Installation

Refer to the [Quad-SDK Wiki](https://github.com/robomechanics/quad-sdk/wiki/1.-Getting-Started-with-Quad-SDK) for installation, dependency, and unit testing information. Currently Quad-SDK requires ROS Melodic on Ubuntu 18.04. All other dependencies are installed with the included setup script.

## Usage
1
Launch the simulation with:

```
roslaunch quad_utils quad_gazebo.launch robot_type:=go1
```

Stand the robot with:
```
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
```
Run the stack with twist input:
```
roslaunch quad_utils quad_plan.launch reference:=twist logging:=true
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel
```
Run the stack with global planner:
```
roslaunch quad_utils quad_plan.launch reference:=gbpl logging:=true
```
Refer to the [Wiki](https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software) for more information on alternate usage.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).


[paper]: https://www.andrew.cmu.edu/user/amj1/papers/Quad_SDK_ICRA_Abstract.pdf
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
