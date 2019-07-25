# Localization
**Note: These instructions assume you have installed Google Cartographer according to [these](http://bwsi-racecar.com/maps/localization/particle_filter_installation/) installation instructions.**
### Maps
* If you have followed the installation instructions as intended, the maps the particle filter uses will be in "\~/localization/localization_ws/src/maps". Assuming you have a .pgm file and a .yaml file in your "~/mapfile" folder, then you can copy all these files with: `cp ~/mapfiles/* ~/localization/localization_ws/src/maps`.
* To use the map for localization, modify the mapParams.yaml file <br>(it’ll be obvious, trust me). You may need to chmod it to edit it.

### Usage in ROS
<u>To run localization</u>:

* In the car's terminal, run `teleop`.</li>
* In the car's terminal, run:
```bash
  source ~/localization/localization_ws/devel/setup.bash
  roslaunch particle_filter localize.launch
```
After the program prints "…Received first LiDAR message," it should start to print "iters per sec: 20  possible: 21" to confirm that it is getting scan data and making localization estimates. We found that it is usually necessary for the vesc to be running completely (i.e. there’s a good Traxxas battery) in order for this to work.
* Also, just as with cartographer, you may open rviz. Interesting topics to try will be

<div style="margin-top: -20px;">
<u>To use the data in ROS</u>:
<div style="margin-top: 10px;">
<ul>
<li>Want to know where you are in this world? Subscribe to <del>(insert pop culture here youtube channel)</del> pf/viz/inferred_pose!</li>
<li>To extract meaningful data from these messages, you can figure it out on your own.
  <ul>
  <li> Use `rostopic type` to see what datatype the messages are. Once you have the name, you can find more info on <a href=http://docs.ros.org/api/geometry_msgs/html/index-msg.html>ros.org</a>.</li>
  <li> Or just use `rostopic echo`.</li>
  <li> <details><summary>Quaternions Help (if you think angular info will help)</summary>
You may have noticed the rotations are encoded in quaternions. Why? I really don’t know, but it allows us to track the car’s rotation from -2π to 2π. If you care to amuse yourself for a few minutes, feel free to look up quaternions and derive the conversion back to an angle. Otherwise, if y’all just need to get this racecar up and running, here’s <details><summary>a little converter function:</summary>
```python
import math
. . .
def quatToAng2D(quat):
    dc=2*math.acos(quat.w)
    ds=2*math.asin(quat.z)
    if ds>0:
        if dc<math.pi:
            ang=ds
        else:
            ang=dc
    else:
        if dc<math.pi:
            ang=ds
        else:
            ang=-dc
    return ang
```
Or to use the ROS’s built-in transformations:
```python
from tf.transformations import euler_from_quaternion
. . .
def quatToAng3D(quat):
    euler = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    return euler
```
For reference, roll = `euler[0]`, pitch = `euler[1]`, yaw = `euler[2]`, and yaw is rotation about the z-axis (equivalent to `ang` in the previous function).

</details></details></li></li></ul></div></div>


<details><summary><font color=#FFA0A0><b>To visualize the data onscreen.</b></font></summary>
<div style="margin-top: 10px;">
<ol type="1" start=1>
  <li>In the computer's terminal, run `rviz`.</li>
  <li>&nbsp;<div style="margin-top: -25px;"><details><summary>In rviz, add the /map, /scan, and /pf/viz/particles topics. </summary>
    <ol type="a">
    <li>In rviz, press "add". </li>
    <li>In the popup, go to the "By topic" tab and select "LaserScan" from the "\scan" topic, and hit the "ok".<br>
    <img src="/img/localize_topics_rviz.png" width=350px/></li>
    <li>Repeat for each topic.</li>
    </ol>
  </details></div></li>
  <li>&nbsp;<div style="margin-top: -25px;"><details><summary>The car likely does not know where it is starting on the map. Give it an estimate of where it is using the "2D Pose Estimate" tool.</summary>
    <img src="/img/localize_pose_rviz.png" width=350px/><br>
    Click on the map for position, drag for orientation.
  </details></div></li>
  <li>&nbsp;<div style="margin-top: -25px;"><details><summary>Don’t like your view locked to (0,0,0)? Make it follow the car by changing your frame to something on the car.</summary>
    <img src="/img/rviz_target_frame.png" width=650px/>
    <ol type="a">
    <li>First use the "Focus Camera" tool and click near the pose estimates (red arrows) to center the view on the car initially.</li>
    <li>Then change "Target Frame" to something on the car to keep up with the car’s changes in position. The "laser" (LIDAR) is a good thing to follow.</li>
    </ol>
  </details></div></li> 
</ol>
</div></details><br>

<details><summary><h3>Google Cartographer Localization</h3></summary>
Basically, Chris wrote some stuff, unfortunately, it ended up not being helpful because Google Cartographer is darn dense and we haven't fully figured it out. Either that, or it's just plain wonk. Wonk means bad. Either way, I didn't have the heart to delete Chris's hard work (but I did have the heart to edit it and make it correct as possible), and besides, maybe some really ROS-y or Google-y person will one day find this helpful...<br>
To run localization in Google Cartographer, you won't need an image and an ".yaml" file, but rather this diddly doo-dad called a ".pbstream" file. Here's how you get this thing:
  
1. `cd` into the folder you want your ".pbstream" stored.
2. Run `roslaunch cartographer_ros offline_racecar_2d.launch bag_filenames:=${HOME}/bagfiles/<your_rosbag_name>.bag`<br>
&ensp; Warning: this will pull up an rviz window, so whoops if you're ssh-ed in.<br>
3. Wait for the bag to finish playing, then watch the terminal and wait until it's done "optimizing".
Now you wanna localize. Here's how you do something like that (though it also tries to make another map, which is concerning; maybe you need to modify one of the config files to include `max_submaps_to_keep = 3`, as the [Google Cartographer website](https://google-cartographer-ros.readthedocs.io/en/latest/going_further.html) suggests):
4. Run the localization by entering the following `roslaunch cartographer_ros demo_racecar_2d_localization.launch \ load_state_filename:=${HOME}/<path_to_file>/<my_file_name>.pbstream`.
5. We don't really know where to get pose data. And if you wanted to give the program pose estimated, good stinkin' luck, buddy. The best we can offer is intercepting stuff sent across the "tf" topic. While the localization is running, enter `rostopic echo tf`. The "base_link" frame may have relevant data.

<h4> Change log (how did we concoct some of those launch and configuration files):</h4>
1. Copy the launch file demo_backpack_2d_localization.launch and rename it by entering `cp demo_backpack_2d_localization.launch demo_racecar_2d_localization.launch`.
&ensp; Within this new file change robot_description to "$(find xacro)/xacro '$(find racecar_description)/urdf/racecar.xacro'")"
&ensp; Configuration_basename becomes racecar_2d_localization.lua
&ensp; Don't remap from "echoes". Instead:
&ensp; Remap from /odom to /vesc/odom
&ensp; Remap from imu to /imu/data
2. Delete the robag node.

3. First, enter `cp offline_backpack_2d.launch offline_racecar_2d.launch`
Also, change the "configuration_basename" argument from backpack_2d.lua to racecar_2d.lua
Delete the "urdf_basename" parameter entirely.
Don't remap from "echoes". Instead:
remap from /odom to /vesc/odom
remap from imu to /imu/data
</details>
