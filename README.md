# ROBAKA

![Robaka 2](doc/images/robaka2.png?raw=true "Robaka")

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

