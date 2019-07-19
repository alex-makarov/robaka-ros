# ROBAKA

This is the ROS package for Robaka, my ROS and SLAM mobile testbed. It uses Robaka ROS node running on Arduino, and doing SLAM with Google Cartographer.

To start:
```
$ sudo apt-get install ros-melodic-desktop-full
$ rosdep install robaka
$ catkin_make
$ roslaunch headlessrobaka robaka.launch [port:=/dev/ttyACM0]
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

