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
```

Depends on https://google-cartographer-ros.readthedocs.io/en/latest/.

Run live SLAM:
```
$ roslaunch cartographer_ros my_robot.launch
```

Record bag:
```
$ rosbag record -O ./mylaserdata10.bag /tf /odom /scan /imu /sonar /cmd_vel
```

Validate bag:
```
$ cartographer_rosbag_validate -bag_filename=mylaserdata10.bag
```

Offline SLAM:
```
$ roslaunch cartographer_ros offline_my_robot.launch bag_filenames:=/home/des/catkin_ws/mylaserdata10.bag
```

Save map:
```
$ rosrun map_server map_saver -f map1
```
