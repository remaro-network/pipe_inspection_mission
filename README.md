# pipe_inspection_mission
This repository contains the pipeline inspection mission example from our paper [Mission Planning and Safety Assessment for Pipeline Inspection Using Autonomous Underwater Vehicles: A Framework based on Behavior Trees](arxiv.org)
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


### Clone the Repository
`imcpy_ros_bridge` and `remaro_uw_sim` are ROS2 packages. For ROS version compatibility we refer to the documentation from the original repos. You can clone this repository in your colcon workspace to compile these ROS packages as follows:
```bash
cd $HOME/<path-to-your-colcon-ws>/src
git clone https://github.com/remaro-network/pipe_inspection_mission
cd ..
colcon build
```

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