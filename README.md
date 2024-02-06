# pipe_inspection_mission
This repository contains the pipeline inspection mission example from our paper [Mission Planning and Safety Assessment for Pipeline Inspection Using Autonomous Underwater Vehicles: A Framework based on Behavior Trees](arxiv.org).

This example repository aims to demonstrate the framework introduced in the paper. A simple example of a squared trajectory is integrated withing the ROS2 packages `imcpy_ros_bridge` and `remaro_uw_sim`.

![graphical-abstract](media/GA.png)

## Components
- `imcpy_ros_bridge`: A bridge interface between IMC (Interface Message Control), ROS (Robot Operating System), and the behaviour trees, facilitating control, communication and data exchange in the robot mission.
- `remaro_uw_sim`: package to bridge between UNavSim, ROS and Dune. It also includes a data recording script.

## Installation

### Prerequisites
- LSTS Toolchain components, for controlling the AUV:
  - Dune: for setting the comunication and send commands to the AUV. Installation and usage instructions are available [here](github.com/LSTS/dune/wiki).
  - Neptus: Dune's graphical interface. Installation and usage instructions are available [here](github.com/LSTS/neptus/wiki).
- UNavSim: for simulating realistic renderings of underwater environments, and getting sensor recordings such as camera, segmentation and IMU. Installation instructions available [here](https://github.com/open-airlab/UNav-Sim).
- ROS2. These repos have been tested under ROS2 Foxy and Humble.
- py_trees_ros. Installation instructions available [here](https://github.com/splintered-reality/py_trees_ros).


### Clone the Repository
`imcpy_ros_bridge` and `remaro_uw_sim` are ROS2 packages. For ROS version compatibility we refer to the documentation from the original repos. You can clone this repository in your colcon workspace to compile these ROS packages as follows:
```bash
cd $HOME/<path-to-your-colcon-ws>/src
git clone https://github.com/remaro-network/pipe_inspection_mission
cd ..
colcon build
```

## Usage
Here we will demonstrate the usage of the framework with a simple square trajectory example.

### ROS topics
The ROS2 topics published during the mission are:

| Package                | Publisher node | Topic                              | Type                                   | Content                         |
|------------------------|----------------|------------------------------------|----------------------------------------|---------------------------------|
| `unavsim_ros_pkgs`     | `unavsim_node` | `/<camera_name>/Scene`             | `sensor_msgs/msg/Image`                | UNavSim RGB camera              |
|                        |                | `/<camera_name>/Segmentation`      | `sensor_msgs/msg/Image`                | UNavSim segmentation labels    |
|                        |                | `/<camera_name>/DepthPlanar`       | `sensor_msgs/msg/Image`                | UNavSim depth camera            |
|                        |                | `/<camera_name>/Scene/camera_info` | `sensor_msgs/msg/CameraInfo`           | UNavSim camera intrinsics       |
|                        |                | `/imu/Imu`                         | `sensor_msgs/msg/Imu`                  | UNavSim's IMU measurement       |
|                        |                | `/altimeter/barometer`             | `unavsim_interfaces/msg/Altimeter`     | UNavSim altimeter measurements  |
|                        |                | `/tf`                              | `tf2_msgs/msg/TFMessage`               | 6 DOF pose in UNavSim           |
| `imcpy_ros_bridge`     | `imc2ros`      | `/from_imc/base_link`              | `geometry_msgs/msg/PoseStamped`        | 6 DOF pose in DUNE              |
|                        |                | `/from_imc/estimated_state`        | `imc_ros_msgs/msg/EstimatedState`      | 6 DOF pose estimated by DUNE    |


## Acknowledgements

<a href="https://remaro.eu/">
    <img height="60" alt="REMARO Logo" src="https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png">
</a>

This work is part of the Reliable AI for Marine Robotics (REMARO) Project. For more info, please visit: <a href="https://remaro.eu/">https://remaro.eu/

<br>

<a href="https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-2020_en">
    <img align="left" height="60" alt="EU Flag" src="https://remaro.eu/wp-content/uploads/2020/09/flag_yellow_low.jpg">
</a>

This project has received funding from the European Union's Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.