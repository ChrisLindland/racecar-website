# Car Cheatsheet

## Setting up the car
1. Set the car on a block of some sort, such that the wheels aren't touching the table/ground
2. Unplug the battery charger, and connect the battery into the car (one some cars, there will be two identical wire; you only need to plug in one. On these battery, *quickly* press the battery's power button. If you hold it, it will change the settings. On others, there is a wire marked blue and one marked green, which you must plug into the corresponding ports).

![](img/CarEPower.jpg)
 
3. Turn on the car using the power button on the TX2

![](img/CarPower.jpg)

4. Plug in the motor battery

![](img/CarTPower.jpg) 

Your car should look like this:

![](img/CarFinal.jpg)

##SSHFS Mounting
In order to access the files on the car, we're going to be using SSHFS to mount its files onto your computer, where you can edit them in VSCode. When you save these files locally, they will automatically get updated on the car! Follow these instructions to set it up:
* Mac OSX and Ubuntu:
1. Install SSHFS:
- On Mac OSX: download FUSE and SSHFS [here](https://osxfuse.github.io/)
- On Ubuntu: in terminal, type `sudo apt-get install sshfs`
2. Create a local directory to store the car's files: `sudo mkdir /path/yourDirectory`. `/path` can be any folder on your computer where you want to mount the car.
3. Then `sudo sshfs -o allow_other,defer_permissions racecar@racecar:/home/racecar /path/yourDirectory`. The password is racecar@mit.
4. In order to make the mount permanent (so you don't have to do this every time you restart the car), we have to edit your `/etc/fstab` file. Type `sudo vim /etc/fstab/`. If this file isn't empty, go to the bottom by pressing `G$`. Press `i`, make a new line, and type in `sshfs#racecar@racecar:/home/racecar /path/yourDirectory`. Press `esc` then type `:wq` to quit.
* Windows:
1. Install the latest versions of [WinFSP](https://github.com/billziss-gh/winfsp/releases/tag/v1.4.19049) and [win-sshfs](https://github.com/billziss-gh/sshfs-win/releases/tag/v2.7.17334)
2. In file explorer, right click on This PC and choose Map network drive. Choose a local drive to mount, and in the Folder field type `\\sshfs\racecar@racecar\home\racecar`. Check the Connect Using Differ
ent Credentials box and connect. The password is racecar@mit.

## SSHing
1. Make sure you're connected to the car's wifi
2. In the Docker terminal, ssh in to the car. You could also ssh in on your native machine if you have a Mac or have an ssh key manager.  
 We know very well this requires much more explanation. If we haven't written instructions by the time we ask you to do this, then you know we've goofed.
3. In the past, we would use the command: `ssh racecar@192.168.1.car_number`. The password is racecar@mit

## Using the Controller
* You must hold down LB to use the controller to drive
* If the Mode light is turned on, press Mode until it turns off
* The controller must be in the D mode (this is changed by a switch in between the bumpers).
* The left joystick controls speed, and the right joystick turns
