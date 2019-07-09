## Overview
We will be working with both the RACECAR hardware and simulator today. Although basic interactions with the hardware require only
[SSH](https://en.wikipedia.org/wiki/Secure_Shell), the full experience
requires a local installation of
[Robot Operating System (ROS)](http://www.ros.org/) on a GNU/Linux machine.
(Note: Although ROS recently started supporting an
[official Windows 10 build](https://wiki.ros.org/Installation/Windows),
it is new and thus untested with our platform.)

## What is ROS?
Despite its name, the Robot _Operating System_ is not actually a bona fide OS. Rather it is a set of robotics middleware built on top of GNU/Linux. ROS is most commonly used in conjunction with
[Ubuntu](https://www.ubuntu.com/), as ROS releases are tied to Ubuntu releases.
For example:

- Ubuntu 18.04, Bionic Beaver → ROS Melodic Morenia
- Ubuntu 16.04, Xenial Xerus → ROS Kinetic Kame

That being said, [Debian](https://www.debian.org/) is also well supported since Ubuntu is derived from it.

If you have never used ROS before, it is best thought of as a standardized
[pub/sub messaging protocol](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern)
with some handy libraries and visualization tools.

## Setup
The next few sections will walk you through getting your personal machine setup with ROS as either a native install, Docker install, or Virtual Machine.

* If you have a GNU/Linux machine (especially Ubuntu or Debian) and are
comfortable on it, a native install will give you the best performance.
* If you are on Windows, MacOS, or an unsupported flavor of GNU/Linux, BSD,
etc., we encourage using our new [Docker](https://www.docker.com/) image.
* If you have your own copy of VMWare installed, and prefer VMs to Docker
(why would you though?), we can provide a Debian-based VM preloaded with all the software you'll need.

_If you already have ROS installed, you're good to go! Just make sure you have
the following ROS packages installed: velodyne, ackermann-msgs, joy, and serial.
Installation instructions for these packages are included at the end of the
Native ROS Install section._

## Setup (Native ROS - Ubuntu/Debian)
### Step 1: Install ROS
Based on your flavor of GNU/Linux, follow the linked installation instructions below. Be sure to install the `ros-VERSION-desktop-full` version of ROS.

* [Install ROS on Ubuntu 18.04](https://wiki.ros.org/melodic/Installation/Ubuntu)
* [Install ROS on Ubuntu 16.04](https://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Install ROS on Debian Stretch](https://wiki.ros.org/melodic/Installation/Debian)
* [Install ROS on Debian Jessie](https://wiki.ros.org/kinetic/Installation/Debian)

_There is an experimental ROS installation for Arch Linux. While we love Arch,
we've found the ROS package unreliable. If you must, you can
follow the experimental instructions [here](https://wiki.ros.org/melodic/Installation/ArchLinux)._


### Step 2: Install Additional ROS Packages
After ROS installation completes, install these additional ROS packages:
```sh
# install on Ubuntu 18.04 & Debian Stretch
sudo apt install ros-melodic-velodyne ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-serial

# install on Ubuntu 16.04 & Debian Jessie
sudo apt install ros-kinetic-velodyne ros-kinetic-ackermann-msgs ros-kinetic-joy ros-kinetic-serial
```

### Step 3: Install the racecar simulator code

First make a `racecar_ws`:

    mkdir -p ~/racecar_ws/src

Clone the racecar code:

    cd ~/racecar_ws/src
    git clone https://github.com/mit-racecar/racecar_simulator.git

Make the code:

    cd ~/racecar_ws
    catkin_make
    source devel/setup.bash

## Setup (Docker)
### Step 1: Install Docker 
Based on your OS, follow the linked installation instructions below.

* Windows
    * Follow these instructions until Step 3, item 2:
    * [Installer](https://docs.docker.com/toolbox/toolbox_install_windows/)
    
* [MacOS](https://docs.docker.com/docker-for-mac/install/)
* [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
* [Debian](https://docs.docker.com/install/linux/docker-ce/debian/)
* [Fedora](https://docs.docker.com/install/linux/docker-ce/fedora/)

### Step 2: Create a Mount Folder
Create a folder to connect your Docker image to:

    # Windows (using Powershell)
    mkdir C:\Users\YOUR_USER_NAME\mount
    mkdir C:\Users\YOUR_USER_NAME\mount\jupyter_ws

    # MacOS
    mkdir -p ~/mount/jupyter_ws

    # GNU/Linux
    mkdir -p ~/mount/jupyter_ws

### Step 2: Run the Docker Image

Start the docker image by running:

    # On Windows
    docker run -ti --net=host -v /C/Users/YOUR_USER_NAME/mount:/mnt/jupyter_ws fishberg/racecar

    # On MacOS
    sudo docker run -ti --net=host -v ~/mount:/mnt/jupyter_ws fishberg/racecar

    # On GNU/Linux
    sudo docker run -ti --net=host -v ~/mount:/mnt/jupyter_ws fishberg/racecar

This will download the docker image the first time it is run and will cache it for future use.

On some operating systems (OS X?) the `--net=host` flag does not properly forward ports. This can be fixed by manually specifying:

    sudo docker run -tip 6080:6080 -p 5900:5900 fishberg/racecar

# Racecar Docker

This code defines a docker image to interface with the MIT Racecar.
The image is built from a Debian base, includes the latest version of ROS, and the [racecar simulator](https://github.com/mit-racecar/racecar_simulator). It can be interfaced through a terminal or graphically through a browser or VNC client.

## Running the Image

If you do not already have docker, follow the install instructions for your OS [here](https://docs.docker.com/install/).

Start the docker image by running:

    sudo docker run -ti --net=host fishberg/racecar

The first time you run this command you will need to wait a little while for the image to download.
Future runs should be instantaneous and won't require an internet connection.
The image is currently ~2.25GB (ROS is big).

### Troubleshooting

Unfortunately, due to the way that networking is implemented in macOS, the `--net=host` flag does not work
\[[1](https://docs.docker.com/docker-for-mac/networking/),[2](https://github.com/docker/for-mac/issues/2716)\].
You can partially fix this by running:

    sudo docker run -tip 6080:6080 -p 5900:5900 fishberg/racecar

If you are running Windows you may also need to run the following before running docker to get the terminal to work properly:

     alias docker="winpty docker"

See the [Additional Docker Options](https://github.com/mit-racecar/racecar_docker#additional-docker-options) section for more useful docker flags.

## Using the Image

When you run the command above, you will be presented with a new bash shell in the folder `racecar_ws`.
This shell has ROS installed (e.g. try running `roscore`).
It also has the programs `screen` and `tmux` installed.
These programs allow you to run many shells from within the same window.

In addition to the terminal interface, you can interact with the image visually through either your browser or through VNC.
This allows you to use programs like `rviz`.

To use the image in the browser, navigate to [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html). Hit the "Connect" button and you're in!

Alternatively, you can interface with the image using any VNC client with address `localhost:5900`.

Some operating systems (e.g. windows) don't like the `localhost` variable. If you can't connect you may have to type in the docker image's actual IP address. Find the IP address by typing `hostname -I` in the image's terminal.
Then visit either of the links above, replacing `localhost` with this IP.

The visual interface has two buttons that launch a terminal and `rviz` respectively.
By default, clicking on the terminal button when a terminal is already open minimizes that window.
To open multiple terminals, type <kbd>CTRL</kbd> and then click on the terminal icon.

### Running the Racecar Simulator

To get started with the simulator, first run the following in any shell:

    roslaunch racecar_simulator simulate.launch

Then open `rviz`.
You should see a blue car on a black and white background (a map) and some colorful dots (simulated lidar).
If you click the green 2D Pose Estimate arrow on the top you can change the position of the car.
Alternatively use a joystick to drive the car as described below.

## Additional Docker Options

### Running on a specific car

By default the image is set up to use ROS locally. If you want to connect the image to another `rosmaster` (e.g. a racecar) you need to change some ROS variables. You can do this automatically by running:

    sudo docker run -ti --net=host fishberg/racecar CAR_NUMBER

This sets the correct `ROS_IP`, `ROS_MASTER_URI`, and `/etc/hosts` variables, assuming that the car's IP is `192.168.1.CAR_NUMBER`. When you launch `rviz` it will display topics published on the racecar. Note that this won't work on macOS due to the networking issues described above.

You will also be able to `ssh` into the racecar from within the docker image by typing:

    ssh racecar@racecar

### Mounting a local drive

Docker images do not save changes made to them by default which can be dangerous when writing lots of code.
Plus, the docker image may not have your favorite text editor, window manager, etc. installed.
To solve both of these issues, you can mount a local folder into the docker image.
This will make sure your changes are written and give you the freedom to edit the code in whatever environment you would like.

We recommend that you mount into the `/racecar_ws/src` folder.
This is typically where all of your code will live while working with the racecar or the racecar simulator.
You can do this by adding the following to your docker run command:

    sudo docker run -tiv /full/path/to/local/folder:/racecar_ws/src --net=host fishberg/racecar

### Using a Joystick

To use a joystick in the image (e.g. to use with the simulator),
you need to forward inputs from that USB device into docker.
Most joysticks map to `/dev/input/js0` by default, so you can add that device with:

    sudo docker run -ti --net=host --device=/dev/input/js0 fishberg/racecar

## Building the Image From Scratch

To build the image from scratch, run:

    git clone https://github.com/mit-racecar/racecar_docker.git
    cd racecar_docker
    sudo docker build -t racecar .

Then run with:

    sudo docker run -ti --net=host racecar
