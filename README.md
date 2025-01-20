# ROS2 Elevator Simulation

##  Description
This is a ROS 2 (Humble) simulation package for an **elevator system** using Gazebo and RViz. It models a simple elevator with a prismatic joint and allows control through ROS 2 topics.

![image](https://github.com/user-attachments/assets/35bd1f32-5a3a-4513-811a-b854403af22f)
![image](https://github.com/user-attachments/assets/4616104d-5f80-43b4-b918-23bb424034b3)


---

## Dependencies
This package requires the following dependencies:

### ** Install ROS 2 Humble**
Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

### ** Install Required ROS 2 Packages**
Run the following command to install necessary dependencies:
```bash
sudo apt update && sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2
```

### Navigate to your ros2_ws/src directory and gitclone this repo:
```
git clone https://github.com/Chillhopper/ROS2_Elevator_Simulation.git
```
