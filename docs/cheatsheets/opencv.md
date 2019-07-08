# OpenCV in Python Cheatsheet

## What to import
* `import cv2` - Imports OpenCV module
* `import numpy as np` - Imports Numpy module
* `from cv_bridge import CvBridge, CvBridgeException` - Imports cv_bridge to allow changing between ROS messages and images that can be altered with OpenCV.

## ROS and OpenCV
* `CvBridge()` - Instantiates a new bridge object.
* `bridge.imgmsg_to_cv2(image_msg, "bgr8")` - Converts the image message into an OpenCV image.
* `bridge.cv2_to_imgmsg(image_cv)` - Converts the OpenCV image into an image message.
* `cv2.imread('example.jpg')` - Reads an image that can be edited using OpenCV. The parameter is the image file.
* `cv2.imshow('image', img)` - Displays an image in a window. First parameter is the window name, second is the image.
* `cv2.imwrite('image.png',img)` - Saves an image. First parameter is the file name, second is the image to be saved.
* `cv2.waitKey(0)` - Waits indefinitely for a key stroke. Needed to process many other GUI events, so is necessary to display the image.
* `cv2.destroyAllWindows()` - Destroys all windows created by OpenCV.

## Other OpenCV functions
* `cv2.line(img ,(0,0), (100,100), (255,0,0), 5)` - Draws a line on img, that begins at (0,0) and ends at (100,100), with color (255,0,0) (blue, since it's BGR), and thickness of 5 px.
* `cv2.rectangle(img, (0,100), (150,0), (0,255,0),2)` - Draws a rectangle on img with the top left corner at (0,100) and bottom right corner at (150,0), with color (0,255,0) (green) and thickness of 2px.
* `cv2.resize(img, (int (img.shape[1]/4)), (int (img.shape[0]/4)))` - Resizes img to be a 1/4th its width (img.shape[1]) and 1/4th its height (img.shape[0]).
* `cv2.putText(img, 'hello', (150,150), textFont, 5, (255,255,255), 2, cv2.LINE_AA)` - Puts the text "hello" on img at (150,150) (bottom-left corner), in font textFont with scale 5, in color (255,255,255) (white), with thickness of 2 px and lineType of cv2.LINE_AA.
* `cv2.cvtColor(img, cv2.COLOR_BGR2HSV)` - Converts img from the BGR color space to HSV.
* `cv2.inRange(hsvImage, lower, upper)` - Thresholds the HSV image to get only the colors that fall within the lower and upper bounds, where lower and upper are numpy arrays that specify HSV values.
* `cv2.bitwise_and(img, img, mask=imgMask)` - Masks the original image with imgMask being the result of cv2.inRange().
* `cv2.findContours(img,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)` - Finds contours. First parameter is the source image, second is contour retrieval mode, and third is contour approximation method. Returns the modfied image, the contours, and hierarchy.
* `cv2.drawContours(img, [cont], 0, (0,255,0),3)` - Draw contours on img in the color (0,255,0) (green) with thickness of 3px. [cont] is a list of the contours, and 0 is the index of contours (for drawing individual ones). To draw all contours, pass -1 as the third argument.

## Sources
* [OpenCV Documentation](https://docs.opencv.org/3.4.1/)
