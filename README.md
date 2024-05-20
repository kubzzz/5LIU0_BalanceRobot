# 5LIU0_BalanceRobot
Control project to balance a simple low-cost robot

Authors: Martijn Strolenberg

## The following programs and exetensions are used
## ROS2  
### 1: ROS2 humble
To install ROS2 humble you can follow the official website instructions
install:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


### 2: Gazebo version 11.10.2
install: 
```
sudo apt install ros-rolling-gazebo-ros-pkgs 
```
(to install gazebo with all ros dependencies)

### 3: RViz2
install: 
```
sudo apt install ros-rolling-rviz2
```

(ROS Extensions)
#### 4: Colcon
install:
```
sudo apt install python3-colcon-common-extensions
```

#### 5: Robot discription
install:
```
sudo apt install ros-rolling-xacro ros-rolling-joint-state-publisher-gui ros-rolling-urdf-tutorial
```

#### 6: ROS2_Control
install:
```
sudo apt install ros-rolling-ros2-control ros-rolling-ros2-controllers ros-rolling-gazebo-ros2-control
```

#### 7: teleop_key
install:
```
sudo apt install ros-rolling-teleop-twist-keyboard
```

# How to run the package
### 1: First make sure you installed all above programs 

### 2: Clone the repository: (for example in home folder)
https://github.com/kubzzz/5LIU0_BalanceRobot.git

### 3: Copy the models in gazebo folder
if it is your first time running gazebo you might need to start it up first before it makes .gazebo/ folder.
navigate to repository home folder (5LIU0_BalanceRobot)
```
cp -r Gazebo/balance_robot/ $HOME/.gazebo/models/
```
to put all models in the gazebo directory

### 4: Make the ros package
navigate to the ros workspace directory
```
cd $HOME/5LIU0_BalanceRobot/balanceRobot_ws
```
Now build the ros package with
```
colcon build
```
Source the package with
```
source install/setup.bash
```

### 5: Run the simulation launch file
make sure your still in the ros workspace directory (balanceRobot_ws)
```
ros2 launch balance_robot rsp_sim.launch.py
```
Now Gazebo and RViz should open up and start the robot simulation Note: you might need to start it up for a second time to spawn the robot.

### 6: Control the robot with your keyboard
make sure your still in the ros workspace directory (balanceRobot_ws)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Make sure the terminal running this is the active one, now if the robots wheels are not in the air you can drive it arround