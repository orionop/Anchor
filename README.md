# Anchor - Autonomous Quadcopter Landing System 🚁

<div align="center">
  <img src="docs/images/quadcopter.png" alt="Quadcopter Landing System" width="600"/>
  <br>
  <p>
    <b>An autonomous system for precise quadcopter landing on moving targets using computer vision and sensor fusion.</b>
  </p>
</div>

## 🌟 Features

- **Real-time Target Tracking**: Computer vision-based tracking of moving landing targets
- **Sensor Fusion**: Kalman filter-based fusion of IMU and GPS data for accurate position estimation
- **PID Control**: Dual PID controllers for precise position and altitude control
- **Safety Features**: Velocity limiting, anti-windup protection, and landing safety checks
- **ROS Integration**: Full ROS compatibility for easy integration with other robotic systems
- **Simulation Support**: Gazebo simulation environment for testing and development

## 📋 Prerequisites

- Python 3.8+
- ROS Noetic
- OpenCV 4.5+
- NumPy 1.21+
- Gazebo Simulator

## 🚀 Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/orionop/anchor.git
   cd anchor
   ```

2. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Set up ROS environment**
   ```bash
   source /opt/ros/noetic/setup.bash
   ```

4. **Build the ROS workspace**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## 💻 Usage

### Running the System

1. **Start the control node**
   ```bash
   rosrun anchor control_node.py
   ```

2. **Launch the target tracking**
   ```bash
   rosrun anchor target_tracker.py
   ```

3. **Monitor the system**
   ```bash
   rostopic echo /control_debug
   ```

### Configuration

The system can be configured through YAML files in the `config` directory:

- `altitude_pid.yaml`: Altitude control parameters
- `position_pid.yaml`: Position control parameters

Example configuration:
```yaml
# Altitude PID Controller Configuration
Kp: 1.0  # Proportional gain
Ki: 0.1  # Integral gain
Kd: 0.05  # Derivative gain

# Anti-windup limits
integral_max: 100.0
integral_min: -100.0
```

## 📚 Documentation

### System Architecture

The system consists of four main components:

1. **Target Tracking** (`target_tracking.py`)
   - Computer vision-based target detection
   - Real-time position estimation
   - ROS topic publishing for target position

2. **Sensor Fusion** (`imu_gps_fusion.py`)
   - Kalman filter implementation
   - IMU and GPS data fusion
   - Accurate position estimation

3. **PID Control** (`pid_controller.py`)
   - Dual PID controllers for position and altitude
   - Anti-windup protection
   - Configuration file support

4. **Control Node** (`control_node.py`)
   - Main control loop
   - Safety checks
   - ROS integration

### ROS Topics

- `/target_position`: Target position updates (Point)
- `/cmd_vel`: Control commands (Twist)
- `/control_debug`: Debug information (Float64)
- `/pid_debug`: PID controller debug data (Float64)

### Safety Features

- Velocity limiting
- Minimum altitude protection
- Landing threshold checks
- Error handling and recovery
- Resource cleanup

## 🛠️ Development

### Adding New Features

1. Create a new branch
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. Make your changes
3. Run tests
4. Submit a pull request

### Running Tests

```bash
python -m pytest tests/
```

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📧 Contact

Project Link: [https://github.com/orionop/anchor](https://github.com/orionop/anchor)






