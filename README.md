# ROS-UNavSim interface
The interface between ROS and UNavSim is handled by the package `unavsim_ros_pkgs`.
if you want the sensor data from your simulated robot in UNavSim to published as a ROS2 topic, then:

```
ros2 launch unavsim_ros_pkgs unavsim_node.launch.py
```