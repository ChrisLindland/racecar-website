# Router Configuration

Some of the routers have the actual TPLink firmware, but some of them have some other jank firmware that's a little bit harder to navigate. All of the routers need to be on the 1 subnet (192.168.1.XX vs the default 192.168.0.XX). If it's not, the Hokuyo ethernet connection will interfere with the car's router connection, and you won't be able to ssh in. 

Otherwise, the only other thing you might want to change is a static IP for your car, so that the IP won't change everytime you restart. You can do this by specifiying your car's mac address (found by either going to Network Information on the car directly or running `ifconfig` in a car terminal) and setting it to a certain IP (we usually stick to 192.168.1.YOUR_CAR_NUMBER).
