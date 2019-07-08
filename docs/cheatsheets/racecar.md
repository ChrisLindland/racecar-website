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

## SSHing
1. Make sure you're connected to the car's wifi
2. On your computer, use this command: `ssh racecar@192.168.1.car_number`. The password is racecar@mit
If you get an error, ask one of the TAs for help

## Using the Controller
* You must hold down LB to use the controller to drive
* If the Mode light is turned on, press Mode until it turns off
* The controller must be in the D mode (this is changed by a switch in between the bumpers).
* The left joystick controls speed, and the right joystick turns

## Driving
* `self.cmd.drive.speed`: changes the car's velocity
* `self.cmd.drive.steering_angle`: changes the angle of the wheels
