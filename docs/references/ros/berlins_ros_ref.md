## Robot Operating System (ROS) Essentials

ROS's fundamental design philosophy is that all functions of a robot should be split up into different "programs", called nodes, that communicate with each other in order to fulfill their tasks. For example, a very simple pick-up-and-drive robot would have two nodes: one that would control the driving and one that would control the arm. The driving node could communicate with the arm node, possibly to tell it that the robot has stopped, and vice-versa. 

Whenever a node wants to output information, it establishes itself as a Publisher (more on that later), and conversely a node that wants to take in information must establish itself as a Subscriber. In order to communicate, there exists a master node called *roscore* that helps link these nodes together. Each ROS node tells roscore what messages it is outputting (as a Publisher) and what messages it wants to receive (as a Subscriber). Roscore then links up matching nodes and allows them to communicate, as described in Figure 1. 

![](/rosgraphthatssupereasytochange.png "ROS Graph") 
###### Figure 1. Taken  with modification by Avalon Vinella from "Programming Robots with ROS" published by O'Reilly Media

<br>

> ##### Note: Roscore deviates from the traditional "middle-man" analogy in that it does not actually recieve/send any tangible information apart from the communication aspects mentioned above. All roscore does is link up nodes, which drastically increases the efficiency and security of the system. 

This is nice and all, but we don't want roscore to hook up just any two Publisher and Subscriber nodes. A walkie-talkie with no channels would be complete chaos. In ROS, these channels are called *topics* and they help roscore decide which nodes to link up. For example, a node that controls the camera might output the information it sees on a topic called "camera", and a node that makes sure the robot does not crash into a pedestrian would tell roscore it wants to listen to this "camera" topic. Roscore would then link the two together and allow them to communicate. 
 
In the real world, things are a little more complicated. A single node can sometimes be a Pulbisher and a Subscriber, or be talking to many different nodes about many different topics. Roscore handles connecting the relevant nodes using the topic feature and makes sure communication is running smoothly. Below is the standard syntax for establishing a node as a Subscriber or Publisher, which you will need to understand and sometimes write.

**Publisher:** To have a node ouput information on a topic, you must write:

```python
pub = rospy.Publisher(topic, dataType)
```

where topic and dataType are swapped out for the topic and 
<span class="container"> <a href="#" data-toggle="tooltip" title="Familiar data types are things like String and int, but a Class can also work as a data type. In fact, traditional data types are just Classes that have been pre-implemented into the base code!">data type</a> </span> 
of the information the node is sending.
**Subscriber:** To have a node input information on a topic, you must write:

```python
sub = rospy.Subscriber(topic, dataType, callback_function)
```
which is similar to the Publisher code except for the extra paramater "callback_function". This paramater is the function that you want to run whenever the node receives data on this topic.   

There is more to talk about regarding Subscribers and Publishers, (our cheatsheet is found [here](ros-topic-message-cheatsheet.md) and the ros wiki page on this can be found [here](https://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)), but this is the necessary information needed to begin to understand ROS code and even write your own.
