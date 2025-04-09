import numpy as np
from filterpy.kalman import KalmanFilter

class SensorFusion:
    def __init__(self):
        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=6, dim_z=3)  # 6 state variables (position and velocity in x,y,z), 3 measurements (GPS)
        
        # State transition matrix
        self.kf.F = np.array([
            [1, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Measurement noise
        self.kf.R *= 1000.  # Measurement noise (GPS uncertainty)
        
        # Process noise
        self.kf.Q[0:3, 0:3] *= 0.1  # Position uncertainty
        self.kf.Q[3:6, 3:6] *= 0.1  # Velocity uncertainty
        
        # Initial state
        self.kf.x = np.zeros(6)
        self.kf.P *= 1000.

    def update(self, gps_data, imu_data):
        """
        Update the Kalman filter with new sensor data
        gps_data: [latitude, longitude, altitude]
        imu_data: [roll, pitch, yaw, acc_x, acc_y, acc_z]
        """
        try:
            # Predict step
            self.kf.predict()
            
            # Update step with GPS data
            self.kf.update(gps_data)
            
            # Use IMU data to update velocity estimates
            self.kf.x[3:6] = imu_data[3:6]  # Update velocity with IMU acceleration
            
            return self.kf.x[:3]  # Return fused position estimate
            
        except Exception as e:
            print(f"Error in sensor fusion: {str(e)}")
            return None

# Example usage:
if __name__ == "__main__":
    fusion = SensorFusion()
    
    # Example sensor data
    gps_data = np.array([19.0760, 72.8777, 100])  # Example GPS data
    imu_data = np.array([0.1, 0.2, 0.3, 0.1, 0.1, 9.81])  # Example IMU data
    
    fused_position = fusion.update(gps_data, imu_data)
    if fused_position is not None:
        print(f"Fused Position: {fused_position}")
