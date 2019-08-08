# Rideshare Challenge

For this challenge, you are going to mimic a rideshare program with your car! There will be two sections in the track: a "pickup" area and a "drop-off" area. The pickup area will have an AR tag that will correspond to a set location in the drop-off area. You will have to localize using your map to reach the correct drop-off location. Once you reach your goal, you should return to the pickup area, where you will read a different tag and start the process again. This means that you will have to set both the drop-off location as a goal, as well as the starting location (the AR tag will always be in the same place).

You are picking up three "customers": 1, 2, and 5 (this is the number the AR tag returns when read). These will be randomly given to you at the pickup point (one at a time) and they have to be returned to their "houses". The order of the houses, from left to right, is 1 -> 2 -> 5 (2 is the corner spot). After you have dropped off the last customer, have your robot return to the pickup spot and stop.

The topic you will need for the lab is `/pf/viz/inferred_pose`.

There will also be obstacles in the track, so you will have to integrate your Potential Field control as well.

THERE IS NO STARTER CODE FOR THIS LAB - flex those coding muscles! ᕙ〳 ʘ – ʘ 〵ᕗ

## Securing the Bag
Navigate to the [Installation: Google Cartographer](http://bwsi-racecar.com/maps/cartographer/cartographer_installation/) on our website and follow the *new* instructions there. Once you have done this, you will need to record a map. You can find instructions for how to do this in *References*, in the **Cartographer** section under ["Running off of a Rosbag"](http://bwsi-racecar.com/maps/cartographer/cartographer_usage/).

After you have done this, read through the [Particle Filter](http://bwsi-racecar.com/maps/localization/particle_filter_usage/) section of *References* for instructions on how to work with the maps you have made (like looking at the relative X and Y values of your current position).

![secure](sb.png)
