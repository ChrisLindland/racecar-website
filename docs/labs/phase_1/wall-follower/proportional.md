## Proportional Control 
Let's try having the car drive a bit more independently. We're going to want it to be able to adjust its speed and ideally its direction in order to move on its own; we can implement these through a method called proportional control. Using proportional control, the car will have a goal of being a certain distance away from an object. If it's far away, the car will speed up until it gets closer, and if it's too close, the car will back up. 

![](../Resources/ProportionalControl.gif)

The farther the car is away from the desired distance, the faster it will move. This means the we can essentially guide the car; if we hold something in front of it and move around, it should be able to follow! 

Now, let's implement it: choose a distance threshold, probably between 0.5 and 1.5. Have the car aim to stay this distance away from an object in front of it. Of course, this will only allow the car to follow it in a straight line. You can use this same idea to adjust the angle of the wheels so the car will stay squared to the object as it moves.

You can work in `controller.py`. Instead of working in `drive_callback`, write a new function `propControl`, then publish in the callback.
