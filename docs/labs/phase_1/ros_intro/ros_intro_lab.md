<center><h1>
Lab: Intro to Terminal & ROS
</h1></center>
<font color=#A0A0A0 size=2>
*Personal note: We realize some of you may find this a bit slow/pedantic, but please know that we aren’t doing this just to annoy you. Knowledge being what it is, it is often necessary to learn a language before trying to write with it. And if you do enjoy the language itself, that’s cool beans too!*
</font>


<hr/><!-------------------------------------------->
<h2>
Manipulating Files Using the Terminal
</h2>

<ol type="1"><li><b>
Make sure you have an installation of ROS.
</b><ul><li>
If you installed ROS on your (presumably Ubuntu) computer or if you have a monitor plugged in to your car, you are good to go.
</li><li>
Otherwise, run Docker (because the fishberg/racecar <a href=https://en.wikipedia.org/wiki/System_image> image </a> has an installation of ROS) and mount to a folder on your host machine. (Good luck! Hehe.)
</li></ul></li>

<li><b>
Use terminal commands to navigate to a folder that holds programs.
</b><br>
The terminal command for navigating to a folder is <code><a href="https://en.wikipedia.org/wiki/Cd_(command)">cd</a> &#60;path_to_folder&#62;/</code>.<br>
<ul><li>
If using a racecar, navigate to the "~/racecar_ws/src/scripts/" folder.
</li><li>
If using your own computer, navigate to wherever makes sense.
</li><li>
If using Docker on your computer, navigate to your mount folder (in either your host machine terminal or the Docker machine terminal).<br>
</li></ul>
<details><summary>Some Common Folders, For Reference</summary>
<ul><li>
"/" is the folder that holds all other folders
</li><li>
"~/" is the home folder
</li><li>
"./" is the current folder
</li><li>
"../" is the folder that holds the current folder
</li><li>
"~/racecar_ws/src/scripts/" is a good folder for store programs on the racecars
</li></ul></details>
</li>

<li><b>
Create a new folder for this lab's programs, and name it "example_ROS_files".
</b><br>
The terminal command is <code><a href="https://en.wikipedia.org/wiki/Mkdir">mkdir</a> &#60;new_folder_name&#62;</code>.
</li></ul></li>

<li><b>
Download the ROS lab starter files from <a href=https://drive.google.com/drive/folders/1WypmNAGqlyrXJbgG3Q5NoyoC7V5afSzM?usp=sharing> here on Google Drive </a> and move them to the folder you just created.
</b><br>
The terminal command for moving files is <br><code><a href="https://en.wikipedia.org/wiki/Mv">mv</a> &#60;folder_path_to_copy_from&#62;/&#60;file_name&#62; &#60;folder_path_to_copy_into&#62;/&#60;new_file_name&#62;</code>.<br>
<ul><li>
If you're directly on the car, make sure that your car's router is plugged into an active ethernet port, or just connect the car to Wifi network.
</li><li>
If you're using Docker, you'll want to do this on your host machine. If you've mounted correctly, the files should also appear in the Docker machine's mount folder.
</li></ul>
</li>

<hr/><!-------------------------------------------->
<h2>
Creating a ROS Network from Scratch
</h2>

<li><b>
In a terminal that has ROS, run <code>roscore</code>.
</b><br>
In short, roscore is a sort of master node looks at all the other nodes’ publishers and subscribers and connects them appropriately. See the "Connecting Nodes" section of the <a href=http://bwsi-racecar.com/ros-reference/ROS_reference/#connecting_nodes>ROS Reference Page</a>.
</li>

<li><b>
Take a look at myBasicNode1.py and try to predict what it does.
</b><br>
(i.e. open it with some text editor)<br>
If you still aren’t sure what’s going on after about 5 min, just move on to the next step.
</li>

<li><b>
In another window/tab of terminal, run myBasicNode1.py.
</b><br>
The terminal command is <code>&#60;path_to_file&#62;/&#60;python_file_name&#62;</code>.<br>
This works because we have <code>#!/usr/bin/env python</code> at the top of our python files. Otherwise we would need to use <code>python &#60;path_to_file&#62;/&#60;python_file_name&#62;</code> to run our file as a python program.
<details><summary>
Does your code not run at all (no Python errors or anything)? Has your permission been denied? Well then folks, it’s debugging time!
</summary>
<ul><li>
What’s probably going on is you don’t have the permissions set to let your programs be executable. We can check this by trying <code>ls -la</code> while in your program’s folder. If your terminal spits back "-rw-r--r--" preceding your filename ("myBasicNode1.py"), that is indeed what's happening.
</li><li>
Add e<font color="00AA00"><b>x</b></font>ecutable permission by running <code>chmod +x &#60file_name&#62</code>.
</li><li> Hopefully you'll see something different if you run <code>ls -la</code> again.
</li></ul>
</details>
</li>

<li><b>
Try <code>rosnode list</code>, then <code>rostopic list</code>, then <code>rostopic echo blah</code>, then <code>rostopic echo blah/data</code>. Then take a look at "myBasicNode1.py" again.
</b><br>
<ul><li>
Just use one more window/tab; you don’t need to run these all at the same time.
</li><li>
Where exactly in "myBasicNode1.py" do you see some of the names that <code>rosnode list</code> and <code>rostopic list</code> spat out?
</li><li>
Take note of the small difference between "blah" and "blah/data". It will become more important as we begin to deal with messages that have many attributes.
</li><li>
Now if there’s a detail about the node that still doesn’t make sense, please ask your squad members or your TA’s what’s the situation here because the rest of the lab builds upon this.
</li><li>
See the <a href=http://bwsi-racecar.com/cheatsheets/ros/> ROS Command Line Cheatsheet </a> for more fun and informative ROS commands.
</li></ul>
</li>

<li><b>
Now take a look at myNode1.py and fill it in to make it work like myBasicNode1.py.
</b><br>
In case you haven’t heard of classes before, we created a class in "myNode1.py". Used this way, the class structure simply helps organization. We will not expect you to write classes from scratch, but that said, classes are good to know if you plan on working with object-oriented programming long-term.</font><br>
</li>

<li><b>
Now fill out "myNode2.py" so that it subscribes to the "blah" topic and prints out the messages it receives. Run it at the same time as "myNode1.py".
</b><br>
See the "Subscribers" section of the <a href=http://bwsi-racecar.com/ros-reference/ROS_reference/#subscribers>ROS Reference Page</a> for a review of subscribers and callback functions if needed.
</li>

<li><b>
Say you called your messages "msg". Try printing "msg" versus "msg.data" to tell the difference between the two.
</b><br>
You should see a similar difference as to when you tried echoing "blah" vs "blah/data". 
</li>

<li><b>
While the nodes are running, try <code>rqt_graph</code>.
</b><br>
Is this the graph you anticipate? If not, please ask your squad members or your TA’s what’s going on here because this is decently important to understanding ROS.
</li>
</ol>
