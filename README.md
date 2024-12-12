# hero_chassis_controller

## Overview

This is a controller package for the DynamicX final assignment. All the requirements have completed. See
the [Details](doc/Details.md) and the [PPT](https://365.kdocs.cn/l/cniyRaL5AqI9) in detail.

**Keywords:** ROS, ros_control, PID, Mecanum wheel, Odometry, TF.

### License

The source code is released under
a [BSD 3-Clause license](https://github.com/gdut-dynamic-x/rm_template/blob/master/LICENSE).

**Author:** Zhipeng Zhong  
**Affiliation:** GDUT  
**Maintainer:** Zhipeng Zhong, zzp89877@gmail.com

The **hero_chassis_controller** package has been tested under [ROS](http://www.ros.org) Indigo, Melodic, and Noetic on
respectively Ubuntu 14.04, 18.04, and 20.04. This is research code, expect that it changes often and any fitness for a
particular purpose is disclaimed.

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

- `roscpp`

- `control_toolbox`

- `dynamic_reconfigure`

- `hardware_interface`

- `pluginlib`

- `std_msgs`

- `tf`

- `teleop_twist_keyboard`

  and so on......

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using:

```bash
cd catkin_workspace/src
git clone https://github.com/HydrogenZp/hero_chassis_controller.git 
catkin build
```

## Functions

This package includes all the functions that the requirement needed and I added two feature functions (See the PPT
file).

## Usage

Control the chassis by using:

```bash
roslaunch hero_chassis_controller keyboard_twist.launch
```

Let the chassis move randomly by using:

```bash
roslaunch hero_chassis_controller random_move.launch
```

## Config files

### Config file folder /cfg

- **PID.cfg**  
  Configuration file for PID parameters.

### Config file folder /config

- **hero_chassis_controller.yaml**  
  General configuration file for the controller.

## Nodes

### Subscribed Topics

- **/cmd_vel**  
  The required velocity messages.

### Published Topics

- **/odom**  
  The odometry.

- **/power_topic**  
  The real-time output power.