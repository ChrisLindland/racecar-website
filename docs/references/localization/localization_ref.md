<center><h1>Particle Filter Localization</h1></center>
<hr/>

<!-- --------------------INSTALLING PARTICLE FILTER-------------------- -->
<details><summary><h2 id="installation-instructions">Installation Instructions</h2></summary>

To install <a href=https://github.com/mit-racecar/particle_filter>this particle filter</a> ROS package:
<ol type="1">
<li> Make a localization folder in home directory and <code>cd</code> into it:
```bash
  mkdir ~/localization
  cd ~/localization
```
</li>
<li> Within "~/localization," try:
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
</li>
<li> Make a catkin workspace folder and <code>cd</code> in to it: 
```bash
  mkdir ~/localization/localization_ws
  cd ~/localization/localization_ws
```
</li>
<li> Within "~/localization/localization_ws," make a new source folder using wstool:
```bash
  wstool init src
```
</li>
<li> Install rosdep.
```bash
  rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```
</li>
<li> Then download this zip file <a href="https://drive.google.com/file/d/1n4dGdirW0J5r6NKri8jONzLk8GGCK_cX/view?usp=sharing">here on Google Drive</a> onto your computer and extract its contents. Then use <code>scp</code> to dump it onto the car into someplace logical (like the Downloads folder):
```bash
  scp -r <path_to_my_computers_downloads_folder>/particle_filter_files racecar@192.168.1.<car_number>:~/Downloads/
```
Then on the racecar, <code>cd</code> into the resulting "particle_filter_files" folder, and copy the files over into the following paths within "localization" (note that these files come from [this repo](https://github.com/mit-racecar/particle_filter)):
```bash
  cp -r ./rviz ~/localization
  cp ./.catkin_workspace ~/localization/localization_ws/
  cp -r ./launch ~/localization/localization_ws/src
  cp -r ./maps ~/localization/localization_ws/src
  cp ./src/* ~/localization/localization_ws/src
  cp ./CMakeLists.txt ~/localization/localization_ws/src
  cp ./package.xml ~/localization/localization_ws/src
```
</li>
<li> Catkin make this thing!
```bash
  cd ~/localization/localization_ws
  catkin_make
```
</li>
</ol>
</details>
<!-- --------------------USING PARTICLE FILTER LOCALIZATION-------------------- -->
<hr/>
## Using the Particle Filter
*Note: These instructions assume you have installed the Particle Filter according to the above installation instructions.*
<hr/>
### Running Localization

<ol type="1">

<li>If you have followed the installation instructions as intended, the maps the particle filter uses will be in "~/localization/localization_ws/src/maps". Assuming you have a ".pgm" file and a ".yaml" file in your "~/mapfile" folder, then you can copy all these files with:
```sh
cp ~/mapfiles/* ~/localization/localization_ws/src/maps
```
</li>

<li>To select which map to use for localization, you'll need to modify your "map_server.launch" file in "~/localization/localization_ws/src/launch". You may need to chmod it to edit it. Launch files essentially tell <code>roslaunch</code> how to run nodes in a package. The modification is simply replacing "basement_fixed.map.yaml" with the name of your ".yaml" file.
  <ul><li>
  Debugging tip: We recommend keeping the default path "$(find particle_filter)/maps/&#60;yaml_file_name&#62;.yaml". If <code>roslaunch</code> doesn't seem to find the ".yaml", double check the filename (no really, stop and do that). Then you can try putting in the full path: "/home/racecar/localization/localization_ws/src/maps/&#60;yaml_file_name&#62;.yaml". Do not try using "~/"! <code>roslaunch</code> does not know what "~/" means like the terminal shell does.
  </li></ul>
</li>

<li>Now we can get cooking! In the car's terminal, run <code>teleop</code>.</li>

<li>Then in another tab/window, run:
```bash
  source ~/localization/localization_ws/devel/setup.bash
  roslaunch particle_filter localize.launch
```
  <ul>
    <li> After the program prints "…Received first LiDAR message," it should start to print "iters per sec: 20  possible: 21" to confirm that it is getting scan data and making localization estimates.</li>
    <li> Debugging tips:
      <ul>
        <li> We found that it is usually necessary for the vesc to be running completely (i.e. there’s a good Traxxis battery) in order for this to work.</li>
        <li> If the <code>roslaunch</code> starts launching, but then returns an error message like <font color="A00000">"cannot locate node of type particle_filter"</font>, it likely means that the "particle_filter.py" file in "~/localization/localization_ws/src/" needs executable permissions. You can give it these permissions by running <code>chmod +x particle_filter.py</code>. </li>
        <li> If the <code>roslaunch</code> starts launching, but then the python file returns an error message like <font color="A00000">"ImportError: No module named range_libc"</font>, it likely means that the range_libc installation failed. Try it again according to our instructions [here](http://bwsi-racecar.com/maps/localization/particle_filter_installation/). Maybe the "setup.py" file in "~/localization/range_libc/pywrapper/" needs executable permissions. </li>
      </ul>
    </li>
  </ul>
</li>

<li> Also, just as with cartographer, you may open RViz. Interesting topics to try will be "/map", "/scan", and "/pf/viz/particles" topics.
  <ul>
    <li> Wondering how to add topics in RViz? See step 7 of the "Running off of live data" section of <a href="http://bwsi-racecar.com/maps/cartographer/cartographer_usage/#running-off-of-live-data">our Cartographer page</a>. </li>
    <li> Again, rviz can be finicky at times. If nothing appears even after running teleop or playing the rosbag, try changing the "Fixed Frame" to "map". Then check and uncheck the the checkboxes for the topics you are interested in. If that didn't work, try re-running Rviz. Check that you are running the programs you need to run. </li>
  </ul>
</li>

<li> The car likely does not know where it is starting on the map. Give it an estimate of where it is using the "2D Pose Estimate" tool.

<img src="../img/localize_pose_rviz_small.png"/>

  <ul>
    <li>Click on the map for position, drag for orientation.</li>
    <li>If you want to be extra fancy, you can have the car do this itself, by publishing a <a href="http://docs.ros.org/api/geometry_msgs/html/index-msg.html">PoseWithCovarianceStamped</a> message to the /initialpose topic. You can see what these messages look like by running <code>rostopic echo /initialpose</code> and doing a 2D pose estimate in RViz.</li>
    <li>A good idea would be to place the car in a fixed starting position and publish this <i>known</i> position to "/initialpose" when you press a button. Then you could press another button to change state and start running.</li>
  </ul>
</li>

<li> (optional)<details><summary>Don’t like your view locked to (0,0,0)?</summary>
Make it follow the car by changing your frame to something on the car.<br>
<img src="../img/rviz_target_frame_small.png"/>
  <ul>
    <li>First use the "Focus Camera" tool and click near the pose estimates (red arrows) to center the view on the car initially.</li>
    <li>Then change "Target Frame" to something on the car (like "base_link") to keep up with the car’s changes in position.</li>
</details></li>
</ol>

## Using Pose Estimate Data in ROS
<font color="00AA00" size="4"><b> This is where you get the pose estimate of where the car is on the map! </b></font><br>
Want to know where you are Subscribe to "pf/viz/inferred\_pose"!

  * To extract meaningful data from these messages, you can figure it out on your own.
  * Use `rostopic type` to see what datatype the messages are. Once you have the name, you can find more info on [ros.org](http://docs.ros.org/api/geometry_msgs/html/index-msg.html).
  * In your python code, remember to import the associated datatype: `from geometry_msgs.msg import <datatype_of_inferred_pose>`
  * If you receive a ROS message in a python program and are unsure of what it is or what it contains, try printing it out.
  * Quaternions Help (if you think angular info will help)
  You may have noticed the rotations for these ROS geometry messages are encoded in quaternions. Why? I really don’t know, but it allows us to track the car’s rotation from -2π to 2π. If you care to amuse yourself for a few minutes, feel free to look up quaternions and derive the conversion back to an angle. Y'all are smart. Or you may just use the ROS’s built-in transformations:
```python
    from tf.transformations import euler_from_quaternion
    . . .
    def quatToAng3D(quat):
        euler = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        return euler
```
  For reference, roll = `euler[0]`, pitch = `euler[1]`, yaw = `euler[2]`, and yaw is rotation about the z-axis.

<details><summary><h3>Google Cartographer Localization</h3></summary>
To run localization in Google Cartographer, you won't need an image and an ".yaml" file, but rather this file structure called a ".pbstream". Here's how you get this thing:
  
(1). `cd` into the folder you want your ".pbstream" stored.
<br>
(2). Run `roslaunch cartographer_ros offline_racecar_2d.launch bag_filenames:=${HOME}/bagfiles/<your_rosbag_name>.bag`<br>
&ensp; Warning: this will pull up an rviz window, so whoops if you're ssh-ed in.
<br>
(3). Wait for the bag to finish playing, then watch the terminal and wait until it's done "optimizing".
<br>
Now you wanna localize. Here's how you do something like that (though it also tries to make another map, which is concerning; maybe you need to modify one of the config files to include `max_submaps_to_keep = 3`, as the [Google Cartographer website](https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html) suggests).
<br>
(4). Run the localization by entering the following `roslaunch cartographer_ros demo_racecar_2d_localization.launch \ load_state_filename:=${HOME}/<path_to_file>/<my_file_name>.pbstream`.
<br>
(5). We don't really know where to get pose data. And if you wanted to give the program pose estimated, good stinkin' luck, buddy. The best we can offer is intercepting stuff sent across the "tf" topic. While the localization is running, enter `rostopic echo tf`. The "base_link" frame may have relevant data.
<br>
<h4> Change log (how did we concoct some of those launch and configuration files):</h4>
(1). Copy the launch file demo_backpack_2d_localization.launch and rename it by entering `cp demo_backpack_2d_localization.launch demo_racecar_2d_localization.launch`.<br>
&ensp; Within this new file change robot_description to "$(find xacro)/xacro '$(find racecar_description)/urdf/racecar.xacro'")"<br>
&ensp; Configuration_basename becomes racecar_2d_localization.lua<br>
&ensp; Don't remap from "echoes". Instead:<br>
&ensp; Remap from /odom to /vesc/odom<br>
&ensp; Remap from imu to /imu/datav
(2). Delete the robag node.<br>
(3). First, enter `cp offline_backpack_2d.launch offline_racecar_2d.launch`<br>
Also, change the "configuration_basename" argument from backpack_2d.lua to racecar_2d.lua<br>
Delete the "urdf_basename" parameter entirely.<br>
Don't remap from "echoes". Instead:<br>
remap from /odom to /vesc/odom<br>
remap from imu to /imu/data<br>
</details>
