Work in progress

1. Start camera, CSI: `rosrun jetbot_ros jetbot_camera` or RealSense: `roslaunch realsense2_camera rs_camera.launch`

2. Calculate camera homography using the [estimator](https://github.com/alex-makarov/drive_ros_camera_homography):
 - Print the pattern
 - Place the robot in front of the pattern
 - Run the calibration: `roslaunch drive_ros_camera_homography homography_estimator.launch` and use dynamic_reconfigure to tune the parameters.
 - Frames for the pattern should be bigger than the pattern (didn't work for me otherwise). 
 - Homography will be saved into the current folder, see example in the repo.

3. Use DCNN inference for [semantic segmentation](https://github.com/alex-makarov/ros-semantic-segmentation) to get the mask, like `rosrun semantic_segmentation segmentation_node __ns:=/jetbot_camera _topic_image:=camera`

4. TODO: Apply homography to the mask from segmentation and convert it into the local costmap

