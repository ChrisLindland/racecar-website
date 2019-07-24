## End of Week Challenge: Line Follow

For this week's final challenge, your team will have to program your racecar to get to the finish line of the course by following the colored tape laid down on the ground. Your car needs to do two things:

1. Recognize where the line of your choice is compared to other lines
2. Follow the target line realiably
3. Reach the end without human intervention

Here is a [link](https://drive.google.com/drive/folders/1sJcN_bGXFaw41tjOZieuiYVl6eKphxPS?usp=sharing) to a blank copy of color\_segmentation and driveNode. driveNode and color\_segmentation have been modified so that it will publish the image from color\_segmentation to the topic IMAGE\_TOPIC ("/zed/zed\_node/color\_seg\_output"). Log into docker using the same command you were using before **but** after fishberg/racecar, add a space and your car number. You will then be able to open rqt\_image\_view in docker and choose the color\_seg\_output topic to view the zed camera feed after the color\_seg filter. If you get a "connection invalid" error, **ignore it**, it tends to work anyways. 

**You must be running driveNode.py in order for the images to be output**

Apart from the publisher thing that was added, these are basically **blank files**; feel free to mess around with them as you wish. We would recommend to look up the HSV values for the color you want and try those first since slider\_colorSegmentation.py has worked for some teams but not for others.

For the first day, we will only have one red line; this will be expanded to more lines tomorrow. Some lines are easier to follow but take longer, other lines are harder to see but get to the end faster, and some lines are easily accessed by following other lines first. Once you can follow a colored line, how can you make your robot reliably switch lines in exactly the way you want?
