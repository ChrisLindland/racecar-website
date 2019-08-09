# Sound Tutorial

We'll be using your car's speakers as a debugging tool. When you add a new state or driving mode to your car, you might want to add a new sound so you can be sure that the correct code is running. Look in `soundNode.py` and use the subscriber's topic to send your car's state to this node from other nodes.

In order to get sound files onto the racecar, first download the file onto your own machine. From your computer (*not* ssh'ed in to the car), use `scp` to copy the file over: 
`scp /local/path/fileName.wav racecar@racecar:/<idkwhatthepathis>/sounds/fileName.wav`
