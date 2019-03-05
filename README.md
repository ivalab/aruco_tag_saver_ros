# ros_aruco
Save ground truth with aruco tag (kinect, meta ports) and publish to ROS topic.

Thanks to [Anina Mu](https://github.com/aninamu) and all ORS18-19 team members ([Joshua](https://github.com/josterdude), [Nicholas](https://github.com/nflint7) and [Jianni](https://github.com/jadkisson6)) to develop this wrapper.

### Looking for a non-ROS version?
please check [aruco_tag_saver](https://github.com/ivalab/aruco_tag_saver/blob/master/README.md)

### Setup
```
roscore
roslaunch freenect_launch freenect.launch
```

### Get kinect frames
```
rosrun ros_aruco aruco_kinect.py
```

### Get transform matrix
```
rosrun ros_aruco aruco_transform.py
```
### View the topic receiving the transform matrix
```
rostopic echo aruco_transform
```

