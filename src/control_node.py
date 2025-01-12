import rospy
from std_msgs.msg import Float64
from pid_controller import PIDController  # Importing the PIDController class from the previous file

class QuadcopterControlNode:
    def __init__(self):
        # Initialize the PID controller with arbitrary tuning values
        self.pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
        # Define the current altitude (simulating quadcopter's altitude)
        self.current_altitude = 100.0  # Starting altitude (meters)
        # Define the target altitude (landing target)
        self.target_altitude = 0.0  # Target altitude (meters)
        
        # Initialize the ROS publisher to publish control signals
        self.control_signal_pub = rospy.Publisher('/control_signal', Float64, queue_size=10)

        # Initialize the ROS rate to run the control loop at 10 Hz
        self.rate = rospy.Rate(10)  # 10 Hz

    def control_loop(self):
        while self.current_altitude > self.target_altitude:
            # Compute the control signal (PID output)
            control_signal = self.pid.compute(self.target_altitude, self.current_altitude)
            
            # Update the current altitude (simulating quadcopter's descent)
            self.current_altitude -= control_signal * 0.1  # Control signal is reducing altitude
            
            # Publish the control signal to the ROS topic
            self.control_signal_pub.publish(control_signal)
            
            # Log the current altitude and control signal for debugging
            rospy.loginfo(f"Current Altitude: {self.current_altitude:.2f} m | Control Signal: {control_signal:.2f}")
            
            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('quadcopter_control')
    
    # Create the control node instance
    control_node = QuadcopterControlNode()
    
    # Start the control loop
    control_node.control_loop()
