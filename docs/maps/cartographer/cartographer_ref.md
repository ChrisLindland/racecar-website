# Installation

## Install Google Cartographer

Based on official Google Cartographer [instructions](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html):
```bash
# Update apt-get (good housekeeping)
sudo apt-get update
# Install ninja
sudo apt-get install ninja-build python-wstool python-rosdep

# Make a workspace for cartographer
mkdir ~/cartographer_ws
cd ~/cartographer_ws
wstool init src

# Fetch cartographer_ros
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Install proto3
src/cartographer/scripts/install_proto3.sh

# Remove deb dependencies and reinitialize them
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
# Must run from within "cartographer_ws" folder:
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install. Must run from within "cartographer_ws" folder:
catkin_make_isolated --install --use-ninja
```

Then add these lines to the end of the car's "~/.bashrc" file if they're not already there:
```bash
source ~/racecar_ws/devel/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash
``` 

## Install MIT Racecar stuff

Clone this repo into your `racecar_ws` (not your `cartographer_ws`!) and `catkin_make`:
```bash
cd ~/racecar_ws/src
git clone https://github.com/mit-rss/cartographer_config.git
cd ~/racecar_ws
catkin_make
source devel/setup.bash
```
Then download this zip file [here on Google Drive](https://drive.google.com/file/d/1a71YjMlLNQapo6Cs3l7ezS-TKVErK0Gs/) onto the car, and dump it into someplace logical (like the Downloads folder). Then extract the zip file and `cd` into the resulting "racecar_cartographer_files" folder, and copy the files over into the following paths within "cartographer_ws":
```bash
cp ./racecar_config_files/racecar_2d.lua ~/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files/racecar_2d.lua
cp ./racecar_config_files/racecar_2d_localization.lua ~/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files/racecar_2d_localization.lua
cp ./racecar_launch_files/racecar_2d.launch ~/cartographer_ws/src/cartographer_ros/cartographer_ros/launch/racecar_2d.launch
cp ./racecar_launch_files/offline_racecar_2d.launch ~/cartographer_ws/src/cartographer_ros/cartographer_ros/launch/offline_racecar_2d.launch
cp ./racecar_launch_files/demo_racecar_2d_localization.launch ~/cartographer_ws/src/cartographer_ros/cartographer_ros/launch/demo_racecar_2d_localization.launch
cp -r racecar_description ~/cartographer_ws/src/
```
Finally `catkin_make` again to install these files:
```bash
cd ~/cartographer_ws
catkin_make_isolated --install --use-ninja
```
# Making Maps
## Running off of live data
*Running live is nice is that all happens all at once, but it can be laggy, and it can become messy if you wish to restart the process.*
1. If you haven't already, make a folder to store maps. We recommend making it in the home directory `mkdir ~/mapfiles`.
2. Run `teleop`.
4. In another car's terminal, run `source ~/cartographer_ws/devel_isolated/setup.bash`, then `roslaunch cartographer_ros racecar_2d.launch` to start making the map.<br>
&nbsp;&nbsp;Note: If you forget to source the setup file, you will get an error like: "No such file or directory: /home/racecar/cartographer_ws/install_isolated/share/racecar_description/urdf/racecar.xacro..."
5. Run `rviz`, and add the "\map" topic if it's not there already. Do this in Docker (make sure you ran it with the car number argument), or on your Ubuntu machine (with ROS installed), or on the car itself (if you have a monitor).
6. Drive the car around the area you wish to map out. Try to drive in closed loops when possible.
7. When you are satisfied with your map, keep cartographer running. To save the map, run `rosrun map_server map_saver -f ~/mapfiles/<your_map_name>`
8. Now you may kill cartographer.

## Running off of a rosbag
*Rosbags are nice in that they allow you to isolate data collection from data processing.*
### Recording the rosbag.

1. If you haven't already, make a folder to store rosbags. We recommend making it in the home directory `mkdir ~/bagfiles`.
2. Place the car in a good starting position.
3. In a car's terminal, run `teleop`.
4. `cd` into your rosbag folder and run `rosbag record -a -o <your_rosbag_name>` to start recording data on all the topics.
5. Drive the car around the area you wish to map out. Try to drive in closed loops when possible.
6. When you are satisfied with your data collection (try to shoot for a minute or two of good data), kill the rosbag to stop recording. It may take a few seconds to stop, so let it die in peace.
7. (optional) The bagfile naming system is kinda gross. Use an `mv` command to rename your bag file to something pretty. If you don't know what we mean by that, [Google](https://www.google.com/) (and by that I mean [DuckDuckGo](https://duckduckgo.com/) or [Ecosia](https://www.ecosia.org/)) "renaming files in terminal".

### Creating the map
<u>To get a .pgm file and a .yaml file (*this is what our localization package uses!*):</u><br>
Follow the same instructions as for running off of live data, but instead of running `teleop`, run `roscore`, and instead of driving the car around, run `rosbag play ~/bagfiles/<your_rosbag_name>.bag`. Save the map when the rosbag stops playing.<br>
<font color="AAA0A0"> Note: Please don't blow off these instructions! If you don't kill teleop, cartographer will see the rosbag data and current data at the same time! Plus, since the `-a` flag passed to `rosbag record` means record everything, playing the rosbag plays drive command data! Brick your cars, folks! </font>

<details><summary><u>Alternatively, to get a .pbstream file (<i>not recommended; this is usually for further use within Google Cartographer</i>):</u></summary>
1. Run <code>roslaunch cartographer_ros offline_racecar_2d.launch bag_filenames:=${HOME}/bagfiles/&lt;your_rosbag_name&gt;.bag</code><br>
&ensp; Warning: this will pull up an rviz window. If you're ssh-ed in, then whoops.<br>
2. Wait for the bag to finish playing, then watch the terminal and wait until it's done "optimizing".
</details>
