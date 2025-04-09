import rospy
import numpy as np
from std_msgs.msg import Float64, Point
from geometry_msgs.msg import Twist
from pid_controller import PIDController
from imu_gps_fusion import SensorFusion
from target_tracking import TargetTracker

class QuadcopterControlNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('quadcopter_control')
        
        # Initialize controllers
        self.altitude_pid = PIDController('config/altitude_pid.yaml')
        self.position_pid = PIDController('config/position_pid.yaml')
        
        # Initialize sensor fusion
        self.sensor_fusion = SensorFusion()
        
        # Initialize target tracker
        self.target_tracker = TargetTracker()
        
        # Initialize state variables
        self.current_position = np.zeros(3)
        self.current_altitude = 100.0  # Starting altitude (meters)
        self.target_altitude = 0.0     # Target altitude (meters)
        self.target_position = None
        
        # Initialize ROS publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/control_debug', Float64, queue_size=10)
        
        # Initialize ROS subscribers
        rospy.Subscriber('/target_position', Point, self.target_callback)
        
        # Initialize ROS rate
        self.rate = rospy.Rate(30)  # 30 Hz control loop
        
        # Safety parameters
        self.max_velocity = 2.0  # m/s
        self.min_altitude = 0.5  # meters
        self.landing_threshold = 0.1  # meters

    def target_callback(self, msg):
        """Handle incoming target position updates"""
        self.target_position = np.array([msg.x, msg.y, msg.z])

    def compute_control(self):
        """Compute control signals for position and altitude"""
        try:
            # Get fused position estimate
            fused_position = self.sensor_fusion.update(
                gps_data=np.array([self.current_position[0], self.current_position[1], self.current_altitude]),
                imu_data=np.array([0, 0, 0, 0, 0, 9.81])  # Example IMU data
            )
            
            if fused_position is None:
                rospy.logwarn("Failed to get fused position estimate")
                return None
            
            # Compute position control if target is available
            position_cmd = Twist()
            if self.target_position is not None:
                position_error = self.target_position - fused_position[:2]
                position_cmd.linear.x = self.position_pid.compute(0, position_error[0])
                position_cmd.linear.y = self.position_pid.compute(0, position_error[1])
            
            # Compute altitude control
            altitude_error = self.target_altitude - self.current_altitude
            position_cmd.linear.z = self.altitude_pid.compute(self.target_altitude, self.current_altitude)
            
            # Apply velocity limits
            position_cmd.linear.x = np.clip(position_cmd.linear.x, -self.max_velocity, self.max_velocity)
            position_cmd.linear.y = np.clip(position_cmd.linear.y, -self.max_velocity, self.max_velocity)
            position_cmd.linear.z = np.clip(position_cmd.linear.z, -self.max_velocity, self.max_velocity)
            
            return position_cmd
            
        except Exception as e:
            rospy.logerr(f"Error computing control: {str(e)}")
            return None

    def control_loop(self):
        """Main control loop"""
        try:
            while not rospy.is_shutdown():
                # Compute control signals
                cmd_vel = self.compute_control()
                
                if cmd_vel is not None:
                    # Check if we've reached landing conditions
                    if (self.current_altitude <= self.min_altitude and 
                        abs(self.current_altitude - self.target_altitude) < self.landing_threshold):
                        rospy.loginfo("Landing complete")
                        break
                    
                    # Publish control commands
                    self.cmd_vel_pub.publish(cmd_vel)
                    
                    # Update current altitude (simulated)
                    self.current_altitude -= cmd_vel.linear.z * 0.1
                    
                    # Publish debug information
                    self.debug_pub.publish(Float64(self.current_altitude))
                
                self.rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Error in control loop: {str(e)}")
        finally:
            # Stop the quadcopter
            self.cmd_vel_pub.publish(Twist())

if __name__ == "__main__":
    try:
        control_node = QuadcopterControlNode()
        control_node.control_loop()
    except rospy.ROSInterruptException:
        pass
