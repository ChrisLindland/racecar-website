# Installation Instructions for [This Particle Filter](https://github.com/mit-racecar/particle_filter)
1. Make a localization folder in home directory and `cd` into it:
```bash
  mkdir ~/localization
  cd ~/localization
```
2. Within "~/localization," try:
```bash
  sudo pip install cython
  git clone http://github.com/kctess5/range_libc
  cd range_libc/pywrapper
  # once upon a time, some RSS teams were able to compile GPU ray casting methods
  # ./compile_with_cuda.sh
  # but now, with re-flashed cars, we are humbled back to the peasant days of regular compiling
  ./compile.sh
```
Since compiling with cuda fails, we do regular compiling. Thankfully, this does not make the localization program prohibitively slow.

3. Make a catkin workspace folder and `cd` in to it: 
```bash
  mkdir ~/localization/localization_ws
  cd ~/localization/localization_ws
```
4. Within "~/localization/localization_ws," make a new source folder using wstool:
```bash
  wstool init src
```
5. Install rosdep.
```bash
  rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```
6. Then download this zip file [here on Google Drive](https://drive.google.com/file/d/1n4dGdirW0J5r6NKri8jONzLk8GGCK_cX/view?usp=sharing) onto your computer and extract its contents. Then use `scp` to dump it onto the car into someplace logical (like the Downloads folder):
```bash
  scp -r <path_to_my_computers_downloads_folder>/particle_filter_files racecar@192.168.1.<car_number>:~/Downloads/
```
Then on the racecar, `cd` into the resulting "particle_filter_files" folder, and copy the files over into the following paths within "localization" (note that these files come from [this repo](https://github.com/mit-racecar/particle_filter)):
```bash
  cp -r ./rviz ~/localization
  cp ./.catkin_workspace ~/localization/localization_ws/
  cp -r ./launch ~/localization/localization_ws/src
  cp -r ./maps ~/localization/localization_ws/src
  cp ./src/* ~/localization/localization_ws/src
  cp ./CMakeLists.txt ~/localization/localization_ws/src
  cp ./package.xml ~/localization/localization_ws/src
```
7. Catkin make this thing!
```bash
  cd ~/localization/localization_ws
  catkin_make
```
