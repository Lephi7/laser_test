
# laser_test



##  1. What is it ?

This is a proof of concept.
This makes the turtle be at a certain arbitrary distance of an object.
The user input could be modified to be a stream of data of a laser indicating the position of the robot relative to the object
(the code would need some rewrite tho).
We can get a precision of around 1cm, but it varies from launch to launch, would need better calibration.
This is one of my first own ROS code, take it with a grain of salt as it sucks.

## 2. How to use it ?

First terminal :
```terminal
$ cd your_ws/laser_test
$ colcon build
$ source install/setup.bash
$ ros2 launch laser_pos turtle_follower.launch.py
```

Second terminal :
```terminal
$ source install/setup.bash
$ ros2 run object_moving pos_of_object
```

HAVE FUN
