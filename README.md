# mobro_sim
This is the ROS package for the simple robot simulation packages for learning ROS 2 through the mobile robot.

## 1. Supported OS
- Ubuntu 22.04

## 2. Preparation
### 1. Prerequisites
Ensure the following software is installed:

- **ros2**: Install ROS 2 Humble by following the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 2. Installation
1. **Navigate to the repository directory**:
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/ros2_ws/src
   ```
2. **Clone this repository**:
   ```bash
   git clone https://github.com/Hayato-80/mobro_sim.git
   ```
3. **Build the package**
   ```bash
   cd ~/ros2_ws/
   colcon build --symlink-install
   ```   
4. **Source the setup script**
   ```bash
   source install/setup.bash
   ```
## 3. Simulation 
### Simulation on Gazebo 11
1. **1st terminal: Launch gazebo and rviz2**:
   ```bash
   cd ~/ros2_ws/
   ros2 launch mobro_gazebo mobro_gazebo.launch.py
   ```
2. **3rd terminal: Run teleop_twist_keyboard**
   ```bash
   # Please make sure this terminal is active
   cd ~/ros2_ws/
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
