# sKratch

sKratchBOT, a mobile robot with a custom-made base using Kelo wheels and a Kinova 7-DOF Manipulator, developed for the b-it bots team for the RoboCup @Work League

## Requirements
* Minimun Ubuntu 22.04 
* Ros humble ([see installation guide](https://docs.ros.org/en/humble/Installation.html))

## Dependencies
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs gazebo libgazebo-dev python3-colcon-common-extensions ros-humble-xacro ros-humble-joint*
```

## Setup Instructions
### 1. Create a ROS 2 Workspace 
```bash
mkdir -p ~/skratch_ws/src
cd ~/skratch_ws/src
```

### 2. Clone Required Repositories

#### 2.1 Clone `skratch_description` (URDF)

```bash
git clone -b dev_classic --single-branch https://github.com/b-it-bots/skratch_description.git
```

#### 2.2 Clone the skratch_simulation for gazebo setup

```bash
git clone -b dev_classic --single-branch https://github.com/b-it-bots/skratch_simulation.git
```

### 3. Build the Workspace

Navigate to the workspace root and build:
```bash
cd ~/skratch_ws
colcon build --symlink-install --packages-select package_name
```

Currently available packages:

```bash
colcon build --symlink-install --packages-select skratch_description skratch_gazebo
```


### 4. Run Gazebo Simulation

Source the setup file and launch Gazebo:
```bash
source ~/skratch_ws/install/setup.bash 
ros2 launch skratch_gazebo gazebo.launch.py
``` 

once the robot is launched you can use teleop twist keyboard to control the roobt.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

