# ROS Command Line cheatsheet

## Launching ROS
* `roscore` - Allows all ROS nodes to communicate with each other, so necessary for all ROS usage
* `rosrun [package] [executable]` - Launches a ROS node, executable can be a `.py` file or a C/C++ executable
* `roslaunch [package] [launch_script]` - Uses a `.launch` file to run multiple other ROS nodes and `roscore`

## Debugging Information
* `rosnode [command]` - Displays debugging information about ROS nodes
    * `rosnode list` - Lists the active nodes
    * `rosnode ping [node_name]` - Tests connectivity to node
    * `rosnode info [node_name]` - Prints information about specific node
    * `rosnode machine [machine_name]` - Lists nodes running on a machine
    * `rosnode kill [node_name]` - Kills a running node
        * `-a` kills all nodes
* `rostopic [command]` - Displays debugging information about ROS topics
    * `rostopic list` - Lists the active topics
    * `rostopic bw [topic_name]` - Prints the bandwidth used by topic
    * `rostopic hz [topic_name]` - Prints the publishing rate of the topic
    * `rostopic echo [topic_name]` - Print topic messages to the screen
    * `rostopic type [topic_name]` - Print topic's message type
    * `rostopic pub [topic_name] [message type] (message)` - Publish data to the topic
    * `rostopic find [message_type]` - Find topics by message type
* `rosmsg [command]` - Displays information about ROS message data structures
    * `rosmsg show [message_name]` - Print the fields in the message
    * `rosmsg users [message_name]` - Search for code using the message
    * `rosmsg package [package_name]` - List all the messages in a package
    * `rosmsg packages [message_name]` - List all the packages with specific message
    * `rosmsg md5 [message_name]` - Print the message md5 sum

## Recording
* `rosbag [command]` - Used for storing ROS graph data
    * `rosbag record [topics_names]` - Records the ROS data into a .bag file (-a records all topics)
    * `rosbag play [.bag file]` - Plays back the ROS data that was recorded

## Visualization Tools
* `rqt_graph` - Displays interactive graph of ROS nodes/topics
* `rqt_image_view` - Displays any topic messages whose type is `sensor_msgs/Image`
* `rqt_bag` - Graphical tool for watching .bag files
* `rqt_deps`- Generates PDF of ROS dependencies
* `rqt_plot` - Plots numerical data on ROS topic over time
* `rqt_logger`
* `rviz` - 3D visualization of robot with sensor data plotted around it
* `gazebo` - A simulation system where the ROS robot can operate just like it normally would
