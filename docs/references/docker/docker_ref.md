<center>
<h1> Docker</h1>
</center>

Docker runs a mini virtual machine whose GUI (graphical user interface) can be viewed in a browser or VNC client. The point of Docker in this class is to get ROS onto non-Linux machines, so <font color="AA0000">if you have something like Ubuntu or Arch-Linux, see [these](http://bwsi-racecar.com/references/native_ros_install/native_ros_install_ref/) native ROS installation instructions instead</font>.

For those of you with non-Linux machines, the "fishberg/racecar" <a href=https://en.wikipedia.org/wiki/System_image> image </a> already has an installation of ROS as well as a basic simulator for running code on virtual racecars.

Unfortunately, a lot can go wrong with setting up the docker image. Be sure to check the troubleshooting sections for any errors that make pop up.

<hr/>
<!-- -------------DOCKER INSTALLATION INSTRUCTIONS------------- -->
## Setup

### Step 1: Install Docker 
Based on your OS, follow the linked installation instructions below.

* [Windows](https://docs.docker.com/toolbox/toolbox_install_windows/)
* [MacOS](https://docs.docker.com/docker-for-mac/install/) (You will have to make an account)
* [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/) (If you really want Docker anyways)
* [Debian](https://docs.docker.com/install/linux/docker-ce/debian/)
* [Fedora](https://docs.docker.com/install/linux/docker-ce/fedora/)

### Step 2: Create a Mount Folder
Create a folder to connect your Docker image to:

```sh
# Windows (using Powershell)
    mkdir C:\Users\YOUR_USER_NAME\mount
    mkdir C:\Users\YOUR_USER_NAME\mount\racecar_ws

# MacOS
    mkdir -p ~/mount/racecar_ws

# GNU/Linux
    mkdir -p ~/mount/racecar_ws
```

<details>
<summary>What is a mounting?</summary>
Being a "light" virtual machine, Docker machines do not by default write to any permanent memory on your computer, so without mounting, your Docker machine will always revert to the original "fishberg/racecar" image every time you restart Docker (<i>thus not saving any code you wrote on it!</i>).<br>
Mounting by contrast allows Docker to access and modify all the files in a mount folder on your host machine. We will use "~/mount" as the mount folder.
</details>
<br>

### Step 3: Install the Racecar Simulator
Make a `src` folder in your `racecar_ws`:
```sh
# Windows (using Powershell)
    mkdir C:\Users\YOUR_USER_NAME\mount\racecar_ws\src
    
# MacOS/GNU/Linux
    mkdir -p ~/mount/racecar_ws/src
    
```
Clone the racecar code:
```sh
cd ~/mount/racecar_ws/src
git clone https://github.com/mit-racecar/racecar_simulator.git
```

Make the catkin space:
```sh
cd ~/mount/racecar_ws 
catkin_make
source devel/setup.bash
```
<details>
<summary>What is <code>catkin_make</code>?</summary>
It basically looks at the files in the "src" folder automatically generates and compiles stuff into the "devel" and "build" folders. For more details, Google it if you wish, but also, don't worry about it too much since it's not critical for programming the racecars.
</details>
<br>

<hr/>
<!-- -------------DOCKER USAGE INSTRUCTIONS------------- -->
## Using the Docker Image
<hr/>

<!-- RUNNING INSTRUCTIONS -->
### Running the Virtual Machine
Start the generic docker image by running:

    # On Windows (run this either in the Docker Toolbox terminal or Powershell)
    docker run -ti --net=host -v /c/Users/<username>/mount/racecar_ws:/racecar_ws fishberg/racecar

    # On MacOS (make sure the Docker application is running!)
    sudo docker run -tip 6080:6080 -v ~/mount:/mnt fishberg/racecar

    # On GNU/Linux
    sudo docker run -ti --net=host -v ~/mount:/mnt fishberg/racecar

The first time you run this command you will need to wait a little while for the image to download.
Future runs should be instantaneous and won't require an internet connection.
The image is currently ~2.25GB (ROS is big).

__If are using a specific car__ (for seeing the car's camera output or ssh'ing), you can add your car's number onto the end. For example, for Mac you would run
```sh
sudo docker run -tip 6080:6080 -v ~/mount:/mnt fishberg/racecar <YOUR_CAR_NUMBER>
```
The car number is the last part of the car's IP address (the IP should be of the form 192.168.1.&#60;YOUR_CAR_NUMBER&#62;).

<!-- RUNNING TROUBLESHOOTING-->
<details>
<summary><b><font color="AA0000">Running Troubleshooting</font></b></summary>
<ul>
<li>
Error something along the lines of "[] is not a launch file nor a package..."
  <ul>
  <li>Remake the catkin space (see <a href=insert_link>step 3</a>); make sure you are in the racecar_ws. It may throw an error about <code>cmake</code>, but just try it again. If you then get a similar but different error...</li>
  <li>Try recloning the code (see <a href=insert_link>step 3</a>) and <code>catkin_make</code> again.</li>
  <li> Ubuntu: you might need to install some extra packages. Run<br>
      
```sh
apt-get install -y \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-ackermann-msgs \
    ros-melodic-joy \
    ros-melodic-map-server \
    build-essential
```
  </li>
  </ul>
</li>

<li>
Windows: try running one docker image for building spaces, and one for running launch files. In one Powershell, run
    
```sh
docker run -ti --net=host -v /c/Users/YOUR_USERANME/mount:/mnt fishberg/racecar
```
In "racecar_ws",<br>

```sh
catkin_make
catkin_make #not a typo; do it twice
source devel/setup.bash
```
In a different Powershell, run <br>
```sh
docker run -ti --net=host -v /c/Users/YOUR_USERNAME/mount/racecar_ws:/racecar_ws/src fishberg/racecar
```
In <b>this</b> docker image, you can run any roslaunch commands you need to.
</li>


<li>
Mac error: “Error response from daemon: Bad response from Docker engine.”
   <ul><li>Docker isn't running, start the application</li></ul>
</li>

<li>
Windows error: "...A connection attempt failed because the connected party did not properly respond after a period of time, ..."
   <ul><li>Try rerunning the docker command in another Powershell window</li></ul>
</li>

<li>
If an instructor deems it necessary, you may also try re-building the "fishberg/racecar" image from scratch.<br>
To build the image from scratch, run:<br>

```sh
git clone https://github.com/fishberg/racecar-docker.git   
cd racecar-docker
sudo docker build -t racecar
```
Then run with:
```sh
sudo docker run -ti --net=host -v ~/mount:/mnt racecar
```
</li>
</ul>
</details>
<br>

<!-- USAGE INSTRUCTIONS -->
### Using the Image

When you start up the virtual machine, you will be presented with a new bash shell in the folder `racecar_ws`.
This shell will have ROS installed (e.g. try running `roscore`).
It also has the programs `screen` and `tmux` installed.
These programs allow you to run many shells from within the same window.

In addition to the terminal interface, you can interact with the image visually through either your browser or through VNC.
This allows you to use programs like RViz.

To use the image in the browser, navigate to [http://192.168.99.100:6080/vnc.html](http://192.168.99.100:6080/vnc.html) (Windows) or [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html) (Mac). Hit the "Connect" button and you're in!

The visual interface has two buttons that launch a terminal and RViz respectively.
By default, clicking on the terminal button when a terminal is already open minimizes that window.
To open multiple terminals, type <kbd>CTRL</kbd> and then click on the terminal icon.

Be sure to check the troubleshooting section if it doesn't work.

<details>
<summary><b>Running the Racecar Simulator</b></summary>

To get started with the simulator, first run the following in any shell:
```sh
roslaunch racecar_simulator simulate.launch
```
Then open RViz.
You should see a blue car on a black and white background (a map) and some colorful dots (simulated lidar).
If you click the green 2D Pose Estimate arrow on the top you can change the position of the car.
Alternatively use a joystick to drive the car as described below.
</details>

<details>
<summary><b>Using a Joystick</b></summary>

Unfortunately, we can currently only get a joystick to work on Linux and Windows machines due to Mac's different USB system. To use a joystick in the image (e.g. to use with the simulator),
you need to forward inputs from that USB device into docker.
Most joysticks map to "/dev/input/js0" by default, so you can add that device with the flag "--device=/dev/input/js0". For example, for Linux you can run
```sh
sudo docker run -ti --net=host --device=/dev/input/js0 -v ~/mount:/mnt fishberg/racecar <YOUR_CAR_NUMBER>
```
</details>

<!-- USAGE TROUBLESHOOTING -->
<details>
<summary><b><font color="AA0000">Usage Troubleshooting</font></b></summary>
<ul>
<li>If the browser link doesn't work, use the provided link after you run the docker image in your terminal (it should be in the paragraph that appears after running the image). You can also try replacing the IP in the address with the results of typing <code>hostname -I</code> in the image's terminal.</li>
<li>If a red banner appear at the top of the noVNC client saying "Failed to connect to server", close your broswer and try re-running the docker image.</code>
<li>RViz: if your RViz isn't setup correctly (ex. you don't see the map or LIDAR scans), make sure the "Displays" option is checked under "Panels". Under the "Global Settings" drop-down, set the topic to "map"; under "Map", set the topic to "/map"; and under "LaserScans", set the size to 0.1. If there is no LaserScans, Map, or RobotModel drop-down, click on "Add" at the bottom of the left panel, and select all of them.</li>
</ul>
</details>
<br>
