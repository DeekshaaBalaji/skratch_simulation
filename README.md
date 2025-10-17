# skratch_simulation

## Create ROS2 workspace
```
mkdir -p ~/sKratch_ws/src
cd ~/sKratch_ws/src
```
## Prerequisites
```
git clone https://github.com/b-it-bots/skratch_description.git
git clone -b ros2-develop https://github.com/kelo-robotics/kelo_tulip.git
```
Clone this repository
```
https://github.com/b-it-bots/skratch_simulation.git
```
Source ROS and build workspace
```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the simulation
Launch robot in simulation
```
ros2 launch skratch_gazebo simulation.launch.py
```

In another terminal run `teleop_twist_keyboard` to control the robot using keyboard
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```