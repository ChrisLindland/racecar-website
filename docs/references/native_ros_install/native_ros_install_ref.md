<center><h1>Setup (Native ROS - Ubuntu/Debian)</h1></center>

<hr/>

## Step 1: Install ROS

Based on your flavor of GNU/Linux, follow the linked installation instructions below. Be sure to install the <code>ros-VERSION-desktop-full</code> version of ROS.
<br><br>
<ul>
<li><a href="https://wiki.ros.org/melodic/Installation/Ubuntu">Install ROS on Ubuntu 18.04</a></li>
<li><a href="https://wiki.ros.org/kinetic/Installation/Ubuntu">Install ROS on Ubuntu 16.04</a></li>
<li><a href="https://wiki.ros.org/melodic/Installation/Debian">Install ROS on Debian Stretch</a></li>
<li><a href="https://wiki.ros.org/kinetic/Installation/Debian">Install ROS on Debian Jessie</a></li>
</ul>

<i>There is an experimental ROS installation for Arch Linux. While we love Arch,
we've found the ROS package unreliable. If you must, you can
follow the experimental instructions <a href="https://wiki.ros.org/melodic/Installation/ArchLinux">here</a>.</i>
<br><br>

## Step 2: Install Additional ROS Packages
After ROS installation completes, install these additional ROS packages:

```sh
# install on Ubuntu 18.04 & Debian Stretch
sudo apt install ros-melodic-velodyne ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-serial

# install on Ubuntu 16.04 & Debian Jessie
sudo apt install ros-kinetic-velodyne ros-kinetic-ackermann-msgs ros-kinetic-joy ros-kinetic-serial
```

## Step 3: Install the racecar simulator code

First make a <code>racecar_ws</code>:
```sh
mkdir -p ~/racecar_ws/src
```

Clone the racecar code:
```sh
cd ~/racecar_ws/src
git clone https://github.com/mit-racecar/racecar_simulator.git
```

Make the code:
```sh
    cd ~/racecar_ws
    catkin_make
    source devel/setup.bash
```