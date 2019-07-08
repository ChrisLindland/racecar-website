# ROS in Python cheatsheet

## What to import
* `import rospy` - To get all of the essential ROS classes
* `from [message folder].msg import [name of message type]` - To import a message type (use * to get all of them from the folder)
* `from cv_bridge import CvBridge, CvBridgeException` - To get the bridge to translate from image msg to OpenCV image

## `rospy` classes and functions to use
* `rospy.init_node(node_name)` - Initializes the node onto the ROS Graph
* `rospy.Subscriber` - A subscriber will allow receiving messages from a topic onto a callback function
    * Init: `sub = rospy.Subscriber(topic_name, message_type, callback_function_name, queue_size = _)` - Initializes with a topic and message type onto a callback function which is called whenever it is published onto
    * Callback Function: `def callback_function_name([self if class], message_type):` - The callback function is given the data from the message
* `rospy.Publisher` - Publishers allow us to publish messages onto a topic
    * Init: `pub = rospy.Publisher(topic_name, message_type, queue_size = _)` - Initialize it as an object with a topic and a message type (can already be existing)
    * Publish: `pub.publish([variable of message_type type])` - Publishes the data
* `rospy.is_shutdown()` - While the current roscore is still running
* `rospy.spin()` - Gives the functionality to ROS until roscore is shut down

## Python Conventions
* `if __name__ == "__main__":` - Acts as a main function, only runs if `python [script_name]` is used on it
* Naming
    * `variable_name` (underscores)
    * `ClassName` (CamelCase)
* 4 spaces per tab
* Put all functionality in a class (?)
