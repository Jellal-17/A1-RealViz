# A1-RealViz

A real-time visualization and control tool for the Unitree A1 robot. This repository provides:

- Interactive joint control sliders to command the robot's joints.
- Real-time graphical plots to visualize joint positions and foot forces.
- Synchronized visualization of the robot in Gazebo and RViz.

## Features
**1. Interactive Joint Control:**

- GUI sliders to control all 12 joints (hip, thigh, and calf).
- Safety limits enforced for each joint to protect the robot.

**2. Real-Time Visualization:**

- Joint positions and foot force values plotted live using matplotlib.

**3. Gazebo and RViz Integration:**
  
- Launch the Unitree A1 robot in Gazebo for visualization.
- View joint states and foot forces in RViz.

**4. ROS Support:**

- Publishes and subscribes to ROS topics for smooth integration with the Unitree robot.

## Dependencies
**System Dependencies:**\
```ROS melodic```\
```Gazebo 9+ (for simulation)```\
```Unitree Legged SDK```

**Python Libraries:**\
```rospy```\
```matplotlib```\
```numpy```\
```tkinter```

Install missing dependencies:
```
sudo apt-get update
sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher
pip install matplotlib numpy
```
**Installation**
1. Clone this repository:
```
cd ~/catkin_ws/src
git clone https://github.com/your-username/A1-RealViz.git
cd ..
catkin_make
```

2. Source your workspace:
```
source ~/catkin_ws/devel/setup.bash
```

## How to Launch

Follow these steps to launch the tools in the correct order:

### 1. Start ROS Master
Start the ROS core:
```
roscore
```
### 2. Launch Gazebo Simulation
Launch the Unitree A1 robot in **Gazebo** for visualization:
```
roslaunch unitree_joint_publisher display.launch
```
### 3. Start Joint Command Publisher
Launch the joint command publisher node that enables joint control:
```
rosrun unitree_joint_publisher joint_command_publisher
```
### 4. Start Joint Control GUI
Launch the GUI to interactively control each joint using sliders:
```
rosrun unitree_joint_publisher joint_command_gui.py
```

Use the sliders to adjust the joint angles. The safety limits for each joint are enforced automatically.

### 5. Launch Real-Time Plots
To visualize joint positions and foot force data over time:
```
rosrun unitree_joint_publisher real_time_plot.py
```

## Overview of Files
### Launch Files
- ```display.launch```: Launches Gazebo and the robot visualization.
- ```control.launch```: Starts joint control publishers.

### Python Scripts
- ```joint_command_gui.py```: GUI-based sliders to command joint positions.
- ```real_time_plot.py```: Plots joint positions and foot forces in real-time.
  
### C++ Nodes
- ```joint_publisher.cpp```: Publishes joint states and foot force data.
- ```joint_command_publisher.cpp```: Subscribes to desired joint positions and sends commands to the robot.

## Usage Workflow
1. Start the ROS master and launch Gazebo.
2. Use the Joint Command GUI to control the robot's joints interactively.
3. Visualize joint movements and foot forces in the real-time plot.
4. View the robot's real-time state in Gazebo and RViz.

## Troubleshooting
### 1. Gazebo or RViz not launching:

- Ensure the launch files are installed correctly.
- Source the workspace: ```source ~/catkin_ws/devel/setup.bash ```
### 2. GUI Not Opening:

- Ensure tkinter is installed:
```
sudo apt-get install python3-tk
```
### 3. Real-Time Plot Not Updating:

- Verify the ```/joint_states``` and ```/foot_forces``` topics are being published.

## Future Improvements
- Add logging support for joint movements and forces.
- Integrate 3D sensor data (e.g., RealSense) for depth visualization.
## License
This project is licensed under the MIT License.
