import time
import rospy
import yaml
import numpy as np
from std_msgs.msg import Float64

class PIDController:
    def __init__(self, config_file=None):
        # Default PID parameters
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05
        
        # Anti-windup parameters
        self.integral_max = 100.0
        self.integral_min = -100.0
        
        # Controller state
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()
        
        # Load configuration if provided
        if config_file:
            self.load_config(config_file)
        
        # Initialize ROS publisher for debugging
        self.debug_pub = rospy.Publisher('/pid_debug', Float64, queue_size=10)

    def load_config(self, config_file):
        """Load PID parameters from YAML configuration file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                
            self.Kp = config.get('Kp', self.Kp)
            self.Ki = config.get('Ki', self.Ki)
            self.Kd = config.get('Kd', self.Kd)
            self.integral_max = config.get('integral_max', self.integral_max)
            self.integral_min = config.get('integral_min', self.integral_min)
            
            rospy.loginfo("PID configuration loaded successfully")
            
        except Exception as e:
            rospy.logerr(f"Error loading PID configuration: {str(e)}")

    def compute(self, setpoint, current_value):
        """
        Compute PID output with anti-windup and derivative kick prevention
        """
        try:
            # Calculate time step
            current_time = time.time()
            dt = current_time - self.prev_time
            if dt <= 0:
                return 0
            
            # Calculate error
            error = setpoint - current_value
            
            # Calculate integral term with anti-windup
            self.integral += error * dt
            self.integral = np.clip(self.integral, self.integral_min, self.integral_max)
            
            # Calculate derivative term (prevent derivative kick)
            derivative = -(current_value - self.prev_error) / dt
            
            # Calculate PID output
            output = (self.Kp * error + 
                     self.Ki * self.integral + 
                     self.Kd * derivative)
            
            # Update state
            self.prev_error = current_value
            self.prev_time = current_time
            
            # Publish debug information
            self.debug_pub.publish(Float64(output))
            
            return output
            
        except Exception as e:
            rospy.logerr(f"Error in PID computation: {str(e)}")
            return 0

    def reset(self):
        """Reset controller state"""
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

if __name__ == "__main__":
    try:
        rospy.init_node('pid_controller')
        
        # Create PID controller
        pid = PIDController()
        
        # Example usage
        current_altitude = 100.0
        target_altitude = 0.0
        
        while not rospy.is_shutdown() and current_altitude > target_altitude:
            control_signal = pid.compute(target_altitude, current_altitude)
            rospy.loginfo(f"Control Signal: {control_signal:.2f}")
            
            # Simulate quadcopter response
            current_altitude -= control_signal * 0.1
            rospy.sleep(0.1)
            
    except rospy.ROSInterruptException:
        pass
