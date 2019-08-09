## What is this repo?
This github repo contains all the labs the 2019 TAs wrote for the BWSI Racecar program.

HERE IS THE ASSOCIATED WEBSITE:  
[http://bwsi-racecar.com/](http://bwsi-racecar.com/)

THE PROGRAMS, HOWEVER ARE IN ANOTHER REPO (cause you know, kids, public repo, solution code...). For any given lab, the associated starter code and/or solution code (when applicable) should be on the [2019-BWSI github repo](https://github.mit.edu/2019-BWSI/Writeups) in the corresponding directory within the "labs_programs" folder.

The labs are located in 4 phases. They are derived from the weeks of the BWSI course but they do not all exactly take up one week's worth of class time. They are ordered in order of decreasing student involvement, meaning earlier phases involve algorithms that students can and should master, while later phases involve increasingly impenetrable behemoths of code (concluding in machine learning) that students can only hope to use and not ever modify (within the scope of the class).

The resources folder contains installation instructions and cheatsheets from other years. We have brief writeups on hardware like the ZED camera, lidar, and the speakers. We also have installation and usage instructions on cartographer, arTags, and localization. Finally, there is a full docker setup writeup for running the racecar simulator.

## Here are links to all other racecar classes and repos:
https://github.mit.edu/2019-BWSI/Writeups/blob/master/RACECAR_Links.md

## Recommended work for future work for future instructors / TAs (from POV of a current TA)
* @Eyassu: get some TA's to come in before the class starts!!! Not only will they need to learn the racecars, but this is not a final product. I mean, it's much more of a final product that anything that preceded it, but I know for a fact that there are some broken links in here, and there may be some even broken-er stuff too. So do what you need to do. Hire through Lincoln if necessary.
* @TA's: 
  * do all the labs
  * look for extra documents in the repo that are not on the website and see if they are helpful
  * if you feel good about everything and have some extra time, try to take a look at ros packages and launch files maybe (would it make sense to use these to launch things more efficiently?)
  * write up a bash installation script for the particle filter (or add it to an image we can flash onto the cars)
  * write up a cheatsheet for tmux
  * move the racecars in small boxes to large boxes
  * manage the wires on the cars (especially so that nobody accidently unplugs the Hokuyo power cable or the VESC PWM cable when putting in the Traxxas battery)
  * continue to document new bugs and stuff
  * turn off the 2.4 gHz wifi networks on all the routers
  * learn how to reflash a car (in case somebody really kills something really really horribly)
  * sauter mini usb ports back onto TX2 and IMU boards if you have the time and tools to do so
  * figure out how to sshfs onto the car
  * make the kids clean up their stinkin trash
