# ROS Helpful Topic and Message Type Cheatsheet

# Topics

Ex. `topic_name` (`message_type_name`) - Description of topic

## Sensor Topics
* `/scan` (`sensor_msgs/LaserScan`) - Topic that the Hokuyo LIDAR sends
* `/scan_processed` - Topic that the processed Velodyne LIDAR sends
* `/imu/data` (`sensor_msgs/Imu`) - Topic that the IMU sends
* `/tf` (`tf/tfMessage`) - Topic that shows the various transformed coordinate systems
* `/odom` (`geometry_msgs/Point`) - Topic that shows the Odometry data

## Camera Topics
* `/zed/rgb/image_rect_color` (`sensor_msgs/Image`) - Topic that gives the rectified image of the camera (in full color)
* `/zed/[right or left]/image_raw_color` (`sensor_msgs/Image`) - Topic that gives the raw image of either of the right or left cameras (in full color)
* `/zed/depth/depth_registered` (`sensor_msgs/Image`) - Topic that gives the depth data from the camera (black and white)


## External Usage Topics
* `/vesc/high_level/ackermann_cmd_mux/input/nav_0` (`ackermann_msgs/AckermannDriveStamped`) - What to write to in order to run the car (without bypassing the safety controller)


## Safety Controller Topics
* `/ackermann_cmd_input` (`ackermann_msgs/AckermannDriveStamped`) - The input about to be published to run the RACECAR
* `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`) - The output to actually drive the car (without anymore checking involved)

## AR Topics
* `/visualization_marker` (`visualization_msgs/Marker`) - 
* `/ar_pose_marker` (`ar_track_alvar_msgs/AlvarMarkers`) - Gives an array of the AlvarMarkers detected

# Messages

Ex. `from message_folder import message_type` - What used for
    * `important_sub_topic` - what represents

## Sensor Messages
* `from sensor_msgs.msg import LaserScan` - The information coming from the LIDAR
    * `LaserScan.ranges` - An array of 1081 distances coming from the laser scan
* `from sensor_msgs.msg import Image` - The image published by the camera 

## Ackermann Messages
* `from ackermann_msgs.msg import AckermannDriveStamped` - The drive command that the Ackermann command mux takes in

