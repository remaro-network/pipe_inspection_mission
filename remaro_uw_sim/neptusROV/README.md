# Dependencies

- Neptus (LSTS Toolchain) installation and usage instructions are available [here](github.com/LSTS/neptus/wiki).
- Dune (LSTS Toolchain) installation and usage instructions are available [here](github.com/LSTS/dune/wiki).
- The bridge between Dune/Neptus and ROS is [imcpy_ros_bridge](https://github.com/olayasturias/imcpy_ros_bridge).


# Building

Simply run `colcon build --packages-select neptus_interface` from your ros2 workspace.

# UNavSim settings
Find the UnavSim settings file inside the folder `Documents/AirSim` (note that UNavSim is developed on top of AirSim).
Inside your `settings.json` file you need to add this line:  
`"PhysicsEngineName":"ExternalPhysicsEngine"`.

# Run
First run the AirSim simulator and Neptus and then execute the ros node bridging them:

```
ros2 run neptus_interface neptus2unavsim
```

### Example
Let's set up an example of UNavSim working with a simulated OMST vehicle in Neptus. For that, you will have Dune, Neptus, UNavSim, and this ROS node all working at the same time. In this example I will show you how to run them all one by one.

1. **Dune**

First, cd to your `dune/build` directory and run:

```
./dune -c lauv-simulator-1 -p Simulation
```
2. **Neptus**
Then, from the directory where you cloned Neptus, execute Neptus as follows:

```
./neptus
```
In the Neptus interface, connecto to the `lauv-simulator-1` vehicle, and run your favourite mission.

3. **UNavSim**

Then, form UNav-Sim, run the simulation in your favourite environment with the `AirSimGameMode` setup.

4. **ROS**

Finally, you can have Neptus and UNavSim running all at once with this ROS package. We have a launcher prepared for you that runs both the imcpy_ros_bridge  package (to bridge between Dune ROS) and the bridge between ROS and UNavSim within this package:

```
ros2 launch neptus_interface lauv_simulator_1.launch.py
```

# Testing that it works

You can test if the bridge is working by manually publishing the pose message as:

```
ros2 topic pub --once /local_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```
