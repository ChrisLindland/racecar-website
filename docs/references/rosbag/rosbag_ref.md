# Rosbags

rosbag Files

Sometimes, it isn’t always time-efficient to take the car for a drive every time you want to test your algorithm. This will help with that problem. Essentially, we are creating a recording of messages that ROS is processing, and when we play this recording, it’s as if the car itself is sending the messages in live time.

Note: Make sure to only record the messages you need. Otherwise, the rosbag file will be too large and will take up more memory than we have on the car, which could lead to a system crash or a completely bricked car. Be careful when you record these. A good safety to have is to use the “-d” flag, for duration. Check this out in the documentation.

All of this info and more is located in the rosbag documentation.

To record:

>> rosbag record -O <your_bag_name_here>.bag scan

To playback:

>> rosbag play <your_bag_name_here>.bag

To play at a multiplied speed, like 2x speed, use the following command

>> rosbag play -r 2 <your_bag_name_here>.bag

There should be a bag file on the car, inside the ~/bagFiles directory. Try playing it right now. Open up the lidar visualizer [TODO: get instructions on how to open lidar visualizer], to see the data from the car driving through a hallway.

If you get a “tried to contact rosmaster” error, or lidar data isn’t consistent:

Make sure teleop isn’t running while you run a rosbag. Otherwise, your bagfile and the live lidar will be sending lidar messages, which will cause problems. Sometimes, killing teleop also kills the rosmaster node, which bagfiles need to run. To start the rosmaster node, run roscore is 


A final note:
Ensure you are naming your rosbags well, otherwise your folder will quickly get crowded with messy files. Also, there isn’t any reason to keep a bag file for a long time, so if you have a lot, go ahead and delete some. 


To visualize:
On the car's terminals:
Open `roscore`, open `teleop`, open `rviz`, stop `teleop`, run your rosbag.
