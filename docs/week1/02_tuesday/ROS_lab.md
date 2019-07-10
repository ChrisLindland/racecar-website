# Creating a ROS Network Using Rospy
<font color=#A0A0A0> In this lab, we will create an example node network. <br> <font size=2>*Personal note: We realize this may feel a bit slow/pedantic, but please know that we aren’t doing this just to annoy you. Knowledge being what it is, it is often necessary to learn a language before trying to write with it. And if you do enjoy the language itself, that’s cool beans too!*</font></font> 
<ol type="1">

<li>Get docker running on your computer with functional mounting. (Good luck!)</li>
<li>Create a new folder.<ol type="a">
<li>Navigate to your mount folder on either your native terminal or on the docker/virtual terminal.<br>
The terminal command is <font color=#00A0F0>`<a href="#" data-toggle="tooltip" title="i.e. change directory">cd</a> &#60;folder path&#62;`</font>.<br>
<font color=#A0A0A0>For example `cd Documents/team_42_labs/`.</font></li>
<li>Create a new folder, and name it sensibly like "example_ROS".<br>
The terminal command is <font color=#00A0F0>`<a href="#" data-toggle="tooltip" title="i.e. make directory">mkdir</a> &#60;folder name&#62;`</font>.</li>
</ol></li>

<li>On your native machine, download the ROS lab starter files from Slack and dump them in the folder you created.</li>

<li>In the docker/virtual terminal, run <font color=#00A0F0>`roscore`</font>.<br>
<font color=#A0A0A0>When we get our ROS essentials page up and running, it will explain what `roscore` does; in short, `roscore` looks at the nodes’ publishers and subscribers and connects them appropriately.</font> </li>

<li>Take a look at myBasicNode1.py and try to predict what it does.<br>
<font color=#A0A0A0>If you still aren’t sure what’s going on after about 5 min, just move on to the next step.</font></li>

<li>In the docker/virtual terminal, run myBasicNode1.py.<br>
Once in the program’s folder, the terminal command is <font color=#00A0F0>`<a href="#" data-toggle="tooltip" title="`./` refers to the current folder">./</a> &#60;program name&#62;`</font>.<br>
<ol type="a">
<li>Does your code not run at all (no Python errors or anything)? Has your permission been denied? Well then folks, it’s debugging time!</li>
<li>What’s probably going on is you don’t have the permissions set to let your programs be executable. We can check this by trying <font color=#00A0F0>`ls -la`</font> while in your program’s folder. If your terminal spits back `-rw-r--r--` preceding your filename (myBasicNode1.py), that is indeed the case.</li>
<li>Change the permissions by running <font color=#00A0F0>`<a href="#" data-toggle="tooltip" title="Note that `chmod 777` is not recommended for files containing sensitive or proprietary information because from a security standpoint, it makes those files very accessible.">chmod 777</a> &#60 filename &#62`</font>. Do this for all the python files.</li>
<li> Hopefully things should look different now when you run `ls -la` again.
</li>
</ol>
<font color=#A0A0A0>Now if there’s a detail about the node that still doesn’t make sense, please ask your squad members or your TA’s what’s the situation here because the rest of the lab builds upon this. </font></li>

<li>In the docker/virtual terminal, try <font color=#00A0F0>`rosnode list`</font>, then <font color=#00A0F0>`rostopic list`</font>, then <font color=#00A0F0>`rostopic echo blah`</font>.<br>
<font color=#A0A0A0>Just use one window; you don’t need to run these all at the same time.</font></li>

<li> Now take a look at myNode1.py and fill it in to make it work like myBasicNode1.py.<br>
<font color=#A0A0A0>In case you haven’t heard of classes before, we created a class in `myNode1.py`. In this application, using the class structure simply helps organization.<br>We will not expect you to write classes from scratch; just be able to use their structure. But if your are able to leverage classes, that’s cool too ;)</font><br>
</li>

<li> Now fill out myNode2.py so that it subscribes to the `blah` topic and prints out the messages it receives.
</li>

<li> Say you called your messages `msg`. Try printing `msg` versus `msg.data` to tell the difference between the two.
</li>

<li> While the nodes are running, try <font color=#00A0F0> `rqt_graph` </font> in another docker terminal.<br>
<font color=#A0A0A0>Is this the graph you anticipate? If not, please ask your squad members or your TA’s what’s going on here because this is decently important to understanding ROS.</font>
</li>

<li> Now for a nice shortcut. Was it annoying opening so many terminal windows to run all your python files? Don’t worry; Andrew may have some special sauce for y’all later in the week!
</li>
</ol>
