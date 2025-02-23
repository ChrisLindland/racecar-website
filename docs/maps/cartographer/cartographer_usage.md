# Making Maps
**Note: These instructions assume you have installed Google Cartographer according to [these](http://bwsi-racecar.com/maps/cartographer/cartographer_installation/) installation instructions.**
## Running off of live data

<font color="A0A0A0">*Running live is nice is that all happens all at once, but it can be laggy, and it can become messy if you wish to restart the process.*</font>

1. If you haven't already, make a folder to store maps. We recommend making it in the home directory `mkdir ~/mapfiles`.
2. Run `teleop`.
4. In another car's terminal, run `source ~/cartographer_ws/devel_isolated/setup.bash`, then `roslaunch cartographer_ros racecar_2d.launch` to start making the map.<br>
&nbsp;&nbsp;Note: If you forget to source the setup file, you will get an error like: "No such file or directory: /home/racecar/cartographer_ws/install_isolated/share/racecar_description/urdf/racecar.xacro..."
5. Run `rviz`, and add the "\map" topic if it's not there already. 
  - Do this in Docker (make sure you ran it with the car number argument), or on your Ubuntu machine (with ROS installed), or on the car itself (if you have a monitor).
  - If you are making a map with a rosbag, be warned that you will not see any map until you start playing the rosbag.
  - To add a topic, click the "Add" button.<br>
![](img/rviz_cartographer1_small.png)<br>
Then go to the, "By Topic" tab, and add the topic you are interested in.<br>
![](img/rviz_cartographer2_small.png)<br>
  - Also, rviz can be finicky at times. If nothing appears even after running teleop or playing the rosbag, try changing the "Fixed Frame" to "map". Then check and uncheck the the checkboxes for the topics you are interested in. If that didn't work, try re-running Rviz. Check that you are running the programs you need to run.
6. Drive the car around the area you wish to map out. Try to drive in closed loops when possible.
7. When you are satisfied with your map, keep cartographer running. To save the map, run `rosrun map_server map_saver -f ~/mapfiles/<your_map_name>`
8. Now you may kill cartographer.

## Running off of a rosbag

<font color="A0A0A0">*Rosbags are nice in that they allow you to isolate data collection from data processing.*</font>

### Recording the rosbag.

1. If you haven't already, make a folder to store rosbags. We recommend making it in the home directory `mkdir ~/bagfiles`.
2. Place the car in a good starting position.
3. In a car's terminal, run `teleop`.
4. `cd` into your rosbag folder and run `rosbag record -a -o <your_rosbag_name>` to start recording data on all the topics.
5. Drive the car around the area you wish to map out. Try to drive in closed loops when possible.
6. When you are satisfied with your data collection (try to shoot for a minute or two of good data), kill the rosbag to stop recording. It may take a few seconds to stop, so let it die in peace.
7. (optional) The bagfile naming system is kinda gross. Use an `mv` command to rename your bag file to something pretty. If you don't know what we mean by that, [Google](https://www.google.com/) (and by that I mean [DuckDuckGo](https://duckduckgo.com/) or [Ecosia](https://www.ecosia.org/)) "renaming files in terminal".

### Creating the map
<u>To get a .pgm file and a .yaml file</u> (*this is what our particle filter uses*):<br>
Follow the same instructions as for running off of live data, but in step 2, instead of running `teleop`, run `roscore`, and in step 6, instead of driving the car around, run `rosbag play ~/bagfiles/<your_rosbag_name>.bag`. Save the map when the rosbag stops playing. (You'll know it is done when it prints "Done." in the terminal).<br>
<font color="AAA0A0"> Note: Remember to kill teleop! If you don't kill teleop, cartographer will see the rosbag data and current data at the same time! Plus, since the `-a` flag passed to `rosbag record` means record everything, playing the rosbag plays drive command data!</font>

<details><summary><u>Alternatively, to get a .pbstream file</u> (<i>not recommended; this is usually for further use within Google Cartographer</i>):</summary>
1. Run <code>roslaunch cartographer_ros offline_racecar_2d.launch bag_filenames:=${HOME}/bagfiles/&lt;your_rosbag_name&gt;.bag</code><br>
&ensp; Warning: this will pull up an rviz window. If you're ssh-ed in, then whoops.<br>
2. Wait for the bag to finish playing, then watch the terminal and wait until it's done "optimizing".
</details>
