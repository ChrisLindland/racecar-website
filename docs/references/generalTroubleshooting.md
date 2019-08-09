
## General Troubleshooting

For a lot of these issues, the answer boils down to *power cycling* (turning the not-working-thing on and off again). So many things can be fixed by power cycling that before you decide something is not working, power cycle at least once and try again.

* Note: This applies to terminals too! Often "teleop" will give a Hokuyo or Vesc error which can be fixed by terminating the process and running teleop again. 

### Car Hardware Issues

* Car does not turn on/dies when pushed: The first place to look is at the cars's batteries. Check that both your batteries are charged and fully connected. Then, check if the problem continues with a different battery. If the problem still persists, it may be the car's power harness that is broken. Use a spare power harness to test. If that oes not work, plug a TX2 battery charger straight into the TX2 motherboard. If no red light appears on the motherboard, your TX2 has been fried and its time to get another one.

### "Teleop" Errors

* Hokuyo/Vesc/IMU exception: Stop the process and try teleop again. If the issue persists, power cycle the car and try again. From here on out, either there could be an issue with the USB hub or the specific component that is throwing an error. If it's an issue with the USB hub, you will get both Vesc and IMU errors. Otherwise, it's an issue with your specific component. Make sure every one of your components are plugged in and has full battery if applicable. Otherwise, try borrowing spare components to see if its a hardware problem specific to your parts.

* Teleop does not throw error but your car doesn't drive: Is your controller on X mode? Did you get a "force input" error in teleop? If your controller was on D mode at any time after the car booted up, you need to switch the controller onto X mode and restart the car. You should also check that your T battery has charge, that your controller dongle is attached to the car, and that the Vesc has power.

### SSH Issues

* Cannot ssh into your car: make sure you are on the right wi-fi and that the car is on the right wi-fi (by logging into the car with a monitor). If the problem persists, make sure that your car and router were configured correctly (check the router configuration file).

* Laggy connection/connection turns off: make sure you are using the same wifi as the car. For example, if you are using the 5G, make sure the car is using the 5G as well. The lagginess can also be due to the amount of cars present which we can't do much about :( Switching off the 5G if everyone is using the 5G can help reduce the laginness, however.

### Lidar/Hokuyo Errors

* Make sure that you can hear the lidar making a spinning sound whenever the car is powered on. Then, make sure your ethernet cable is connected. Finally, make sure you have the correct ethernet settings from within the car (ie. the Hokuyo

### Zed Errors

* "Camera is already in use": your best bet is to shut the car down since there is probably a hidden ZED running on somebody's computer. That should fix the issue.

### OpenCV 

* Error -215: Your code can't find the image! Trace back your image variable and make sure that it is compatible with openCV (is there a cv2.imread there?). If you are working with template images/outside images, make sure your image is actually on your machine and that your pathing is correct. If you are interfacing with ZED, make sure you have Zed up and running and that your camera is plugged in.

