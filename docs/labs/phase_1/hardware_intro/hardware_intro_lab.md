# Setting up and Driving 

Welcome to the racecar! First things first, we need to set up the cars to be able to control them.

### Goals
1. Learn how to set up the car
2. Be able to use ssh
3. Practice controlling the car
4. Implement basic autonomous driving programs

## Setting up the car
1. Set the car on a block on your table, such that the wheels __aren't touching the table__. Unless you're actually testing your code on the ground, leave the car on this block.
2. Unplug the battery charger, and connect the battery to the board. On some cars, there will be two identical wires; you only need to plug one into the battery. On these, *quickly* press the battery's power button. If you hold it, it will change the settings. On other cars, there is a wire marked blue and a wire marked green, which you must plug into the corresponding ports).

![](../Resources/CarEPower.jpg)

3. Turn on the car using the power button on the TX2

![](../Resources/CarPower.jpg)

The lights should turn green.

4. Plug in the motor battery

![](../Resources/CarTPower.jpg)

Your car should look like this:

![](../Resources/CarFinal.jpg)

## SSH'ing

To access the car, we're going to be ssh'ing and scp'ing files. To ssh, type in your terminal 

`ssh racecar@192.168.1.YOUR_CAR_NUMBER`. The password is racecar@mit. 

Make sure everyone on your team is able to do this before moving on! 

The `scp` command allows you to copy files back and forth from a remote machine. The format is 

`scp LOCAL_FILE_PATH REMOTE_USERNAME@IP:REMOTE_FILE_PATH`. 

For example, if you wanted to move a file on your Mac desktop called `drive.py` to the racecar_ws of car 101, you would run: 

`scp ~/Desktop/drive.py racecar@192.168.1.101:/home/racecar/racecar_ws/`. 

__You must run this outside of the racecar, ie. not in an ssh'd terminal__.

## Controlling the car
Once you've successfully ssh'd in, type `teleop`. This will allow us to actually drive the car. If there is an error about it being unable to connect to the joystick, end the program (ctrl-C) and try again. If there is still an error, as a TA for help. 

Now you can drive the car using the controller! The cars have a Dead Man's Switch for safety; this means that to use the controller to drive, you have to hold down LB at the same time. If the Mode light is turned on, press Mode. The controller must also be in the X mode (this is changed by a switch in between the bumpers). Try driving around! The left joystick controls speed, and the right joystick turns.

Keep in mind that these are expensive piece of equipment; always be aware of your/the car's surroudings when driving, and communicate with other teams that are also driving their cars to avoid damaging them.

#### Common Errors
If your car doesn't drive with the controller...
* Check `teleop` for any errors. There should be a "force feedback" error for the joystick, but everything else should run fine.
* If there is a vesc error, quit teleop, wait a minute or two and try again. This often happens if you try to run teleop immediately after turning on the car or pluggin in the traxxis battery, since it takes a minute for the vesc to actually start running.
* If you get individual errors for the Hokuyo or IMU, check that these are properly plugged into the USB hub. For the Hokuyo, check that the ethernet is plugged in as well, and for the IMU, check the blue light is turned on on the IMU. If it isn't check that the mini USB is plugged in.
* If you get multiple errors for the vesc, Hokuyo, and IMU, there might be a problem with your USB hub. Try power-cycling the car (turning it off and on). If that doesn't fix it, check that the USB hub is connected to the TX2 board and is getting power from the battery through the power harness.
* If your controller was on D and then you switched it to X, try power-cycling the car.

When you think you have a good feel of the car, move on to the next section.

## Intro to autonomous driving
First, let's just program the racecar to drive forwards on its own. Open `controller.py`. Change the car's velocity in the function `drive_callback`. You can change the speed with the variable `self.cmd.drive.speed`. Start off with small values like 0.5 so you don't accidently crash!

When you want to run your code on the car, press 'A' on the controller after teleop has been turned on.

Next, try having the car drive in a circle. You can change the wheel angle with the variable `self.cmd.drive.steering_angle` (the car uses radians). Explore the range of values that might work best for your car (speed, turning angle, etc).
