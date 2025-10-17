# sKratch

sKratchBOT, a mobile robot with a custom-made base using Kelo wheels and a Kinova 7-DOF Manipulator, developed for the b-it bots team for the RoboCup @Work League

## Requirements
* Minimun Ubuntu 22.04 
* Ros humble ([see installation guide](https://docs.ros.org/en/humble/Installation.html))

## Setup Instructions
1. Create a ROS 2 Workspace 
```
mkdir -p ~/skratch_ws/src
cd ~/skratch_ws/src
```

2. Clone the Repository
```
git clone git@github.com:AnudeepSajja/sKratch.git .s
```

3. Build the Workspace

Navigate to the workspace root and build:
```
cd ~/skratch_ws
colcon build --symlink-install --packages-select package_name
```

Currently available packages:

```
colcon build --symlink-install --packages-select skratch_description skratch_gazebo
```


4. Run Gazebo Simulation

Source the setup file and launch Gazebo:
```
source ~/skratch_ws/install/setup.bash 
ros2 launch skratch_gazebo gazebo.launch.py
``` 

once the robot is launched you can use teleop twist keyboard to control the roobt.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

