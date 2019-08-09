# Week 1 Final Challenge

We've reached your first cumulative challenge! We'll be giving you a more complex racetrack to navigate, and you'll also have a new type of autonomous driving to explore called potential fields.

## How do potential fields work?
Unlike wall following, potential fields takes in all the surrounding LIDAR data to decide how it should move. This means we now have the freedom to drive in any space, with or without walls!

Let's imagine that every piece of LIDAR data has a vector pointing from the car to the point. In order for the car to avoid this obstacle, we want the car to consider this vector in the *opposite* direction so that it will move away from it. So, if it sees something 1 meter in front of it, the car will interpret this as a vector of length 1 meter in the backwards direction. In reality, the car will get a piece of data in every direction; we can then add up all of these vectors to create a final vector that will tell the car what direction it should move in and how fast it should go.

![](../Resources/ptFieldsDiagram.jpg)

In this diagram, there is also an attracting force (the goal), but we will only be using repelling forces (obstacles). You can just set a default speed and direction to account for this. 

For speed, we have to find a way to adjust for how far away a point is, but we want the opposite effect of proportional control; if an obstacle is closer to the car, we want it to take priority over other, farther away obstacles that we don't need to deal with at the moment. You can tell in the diagram that the vectors closest to the obstacle are larger. 

So if we have LIDAR input and car interpretations like this...

![](../Resources/PtFieldGif.gif)

The car should have a final force vector like this:

![](../Resources/PtFieldsFinal.jpg)

We can implement this by adjusting each raw distance by some factor such that shorter distances produce much higher speeds and longer distances are slightly slower (think about what mathematical functions would give you a larger output for a smaller input). You also will want to multiple this value by some constant so that we get appropriate speeds for the car.

Think about why we might prefer potential fields over wall following for this racetrack; if there are still walls, how could potential fields work better?

Functions to write:
* `convertPoints`: Convert points to cartesian coordinates
* `calcFinalVector`: Calculate final drive vector
* `drive_callback`: Publish the update speed and angle of the drive vector

Currently, the final vector is of form [speed, angle]. You can change this if you'd like (ie if you want to change it to [x-speed, y-speed])

You can also implement a PID controller to try and make your driving more accurate.

For now, copy the starter code into the `wall_follower` starter code so roslaunch will work correctly. You can either manuaully copy in the code or use `cp ptFieldSim.py /your/path/wall_follower.py` in your terminal/powershell. 
