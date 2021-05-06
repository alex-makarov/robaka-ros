# ROBAKA

**NEW**: [Join](https://join.slack.com/t/robaka/shared_invite/zt-q52yfvnl-IP0h~JDOmgh3VmJ7Hh69Jw) the community on Slack to discuss Robaka implementations!

![Robaka 2](doc/images/robaka2.png?raw=true "Robaka")

[![AR tag follower](https://img.youtube.com/vi/KP6Jw8Xr8P8/0.jpg)](https://www.youtube.com/watch?v=KP6Jw8Xr8P8)
[![BBQ cart](https://img.youtube.com/vi/KbH6WZnc5S4/0.jpg)](https://www.youtube.com/watch?v=KbH6WZnc5S4)

This is the ROS package for Robaka, my ROS and SLAM mobile testbed. Robaka uses ros_control with hardware driver from [this repo](https://github.com/alex-makarov/hoverboard-driver), which relies on hoverboard custom firmware by [Bipropellant](https://github.com/bipropellant).

If you're looking for the Arduino-based Robaka 1, the code is on `robaka-1` branch.


To start:
```
$ sudo apt-get install ros-melodic-desktop-full
$ rosdep install robaka
$ catkin_make
$ roslaunch headlessrobaka robaka.launch
```

On desktop:
```
$ roslaunch uirobaka.launch
OR
$ roslaunch robaka robaka_cartographer_localization.launch load_state_filename:=<FULLPATH>/laserdata25.bag.pbstream
```

Refer to  https://google-cartographer-ros.readthedocs.io/en/latest/ for SLAM details.

Run live SLAM:
```
$ roslaunch robaka slammingrobaka.launch
```

Record bag:
```
$ rosbag record -a -O ./mylaserdata10.bag
```

Validate bag:
```
$ cartographer_rosbag_validate -bag_filename=mylaserdata10.bag
```

Offline SLAM:
```
$ roslaunch robaka offline_slam.launch bag_filenames:=/home/des/catkin_ws/mylaserdata10.bag
```

Save map:
```
$ rosrun map_server map_saver -f map1
```

Save online SLAM state for cartographer .pbstream:
```
$ rosservice call /finish_trajectory 0
$ rosservice call /write_state "{filename: '/home/alex/mylaserbag23.bag.pbstream'}"
```

NOTE:
patch cartographer as described here https://github.com/googlecartographer/cartographer/issues/1498#issuecomment-464308882
to fix global costmap issue

![SLAM](doc/images/robaka2.gif?raw=true "Robaka")

* 2020-03-26
Added teleop with PS4 controller, using the [driver](http://wiki.ros.org/ds4_driver) from Naoki Mizuno

* 2020-04-08
Added semantic segmentation for driveable surface. Not tested yet. [Cloned repo](https://github.com/alex-makarov/ros-semantic-segmentation), implemented by Dheera Venkatraman.

* 2020-04-18
Semantic segmentation actually works on a pre-trained network. Started work on transforming the mask using camera homography into a costmap, documenting [here](doc/semantic-segmentation/README.md)

* 2021-03-10
Added LCD display to show robot status.
Added "AR tag follower" mode. 
Switched hoverboard driver from speed control to PWM control (much smoother motion).



