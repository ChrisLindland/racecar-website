# New Instructions
1. Download this zip file [here on Google Drive](https://drive.google.com/file/d/1wgUndGvzvPm53jQaUjKVcDCtw8LXd0cM/view?usp=sharing) onto your computer and extract its contents. Then use `scp` to dump it onto the car into the car's Downloads:
```bash
  scp -r <path_to_my_computers_downloads_folder>/racecar_cartographer_installation racecar@192.168.1.<car_number>:~/Downloads/
```
2. Make sure your car's router is plugged into wifi.
3. `ssh` into the car `cd` to the "racecar_cartographer_installation" folder
4. Run Mat's first shell script. (This replaces Install Google Cartographer):
```bash
  bash cartographer_installation.sh
```
Warning: this will take a diddly doodlely long time.
5. Run Mat's second shell script. (This replaces Install MIT Racecar stuff):
```bash
  bash racecar_cartographer_installation.sh
```

# Old Instructions
### Install Google Cartographer
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
source ~/racecar_ws/.catkin_ws/devel/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash
``` 

### Install MIT Racecar stuff

Clone this repo into your `racecar_ws` (not your `cartographer_ws`!) and `catkin_make`:
```bash
cd ~/racecar_ws/src
git clone https://github.com/mit-rss/cartographer_config.git
cd ~/racecar_ws
catkin_make
source devel/setup.bash
```
Then download this zip file [here on Google Drive](https://drive.google.com/file/d/1a71YjMlLNQapo6Cs3l7ezS-TKVErK0Gs/) onto your computer and extract its contents. Then use `scp` to dump it onto the car into someplace logical (like the Downloads folder):
```bash
  scp -r <path_to_my_computers_downloads_folder>/racecar_cartographer_files racecar@192.168.1.<car_number>:~/Downloads/
```
Then on the racecar, `cd` into the resulting "racecar_cartographer_files" folder, and copy the files over into the following paths within "cartographer_ws":
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
