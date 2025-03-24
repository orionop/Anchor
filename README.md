# Anchor - An Autonomous Quadcopter Landing System 🚀

An autonomous system for landing a quadcopter on a moving target using computer vision, sensor fusion, and PID control. The system integrates real-time data from cameras, IMUs, and GPS to guide the quadcopter to a stable landing on a moving target.

## Project Overview
This project demonstrates an autonomous system that allows a quadcopter to land on a moving target. The system uses sensors and vision algorithms to track the target and adjust the quadcopter's position in real time for a precise landing.

## Tech Stack
- **Programming Languages**: Python, C++
- **Simulation**: Gazebo, ROS (Robot Operating System)
- **Libraries**: OpenCV, NumPy, SciPy, PID Control
- **Microcontroller**: Raspberry Pi
- **Sensors**: IMU (MPU6050), Camera (Pi Camera), GPS
- **Flight Control**: Pixhawk, ArduPilot
- **Communication**: MAVLink (for communication with Pixhawk)
- **Path Planning**: A* Algorithm for optimal target tracking

## Features
- **Real-time target tracking** using a camera and OpenCV.
- **Sensor Fusion** with IMU and GPS for stable flight.
- **PID Controller** to stabilize the quadcopter during landing.
- **Simulation Environment** in Gazebo with ROS integration.
- **Moving Target Simulation** in Gazebo to replicate real-world conditions.

#### **Installation Instructions**

1. Clone the repository:
```git clone https://github.com/yourusername/autonomous-quadcopter-landing-system.git```

2. Install dependencies:
 ```  pip install -r requirements.txt```

3. Set up ROS environment (if using ROS):
   ```source /opt/ros/noetic/setup.bash```

4. Set up Gazebo and ROS packages:
  ``` cd ~/catkin_ws catkin_make source devel/setup.bash```

5. Run the simulation:
  ``` roslaunch quadcopter_landing.launch```


## Usage

Once the simulation runs, you can control the quadcopter’s flight and landing by modifying the parameters in the configuration files or adjusting the PID controller values.

## How to Run 

1. Install required libraries:
   ```pip install -r requirements.txt```

2. Launch Gazebo and ROS:
   ```roslaunch quadcopter_landing.launch```






