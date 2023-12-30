# 5LIU0_BalanceRobot
Control project to balance a simple low-cost robot

## The following programs and exetensions are used
### Gazebo version 11.10.2
install: 
```
sudo apt install ros-humble-gazebo-ros-pkgs 
```
(to install gazebo with all ros dependencies)

### RViz2
install: 
```
sudo apt install ros-humble-rviz2
```

### ROS2  
#### 1: ROS2 humble
install:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

(Extensions)
#### 2: Colcon
install:
```
sudo apt install python3-colcon-common-extensions
```

#### 3: Robot discription
install:
```
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-urdf-tutorial
```

#### 4: ROS2_Control
install:
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
```

#### 5: teleop_key
install:
```
sudo apt install ros-humble-teleop-twist-keyboard
```

# How to run the package
#### 1: First make sure you installed all above programs 

#### 2: Clone the repository: (for example in home folder)
https://github.com/kubzzz/5LIU0_BalanceRobot.git

#### 3: Copy the models in gazebo folder
navigate to repository home folder (5LIU0_BalanceRobot)
```
cp -r Gazebo/balance_robot/ $HOME/.gazebo/models/
```
to put all models in the gazebo directory

#### 4: Make the ros package
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

#### 5: Run the simulation launch file
make sure your still in the ros workspace directory (balanceRobot_ws)
```
ros2 launch balance_robot rsp_sim.launch.py
```
Now Gazebo and RViz should open up and start the robot simulation

#### 6: Control the robot with your keyboard
make sure your still in the ros workspace directory (balanceRobot_ws)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Make sure the terminal running this is the active one, now if the robots wheels are not in the air you can drive it arround