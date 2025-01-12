import numpy as np

class SensorFusion:
    def __init__(self, imu_data, gps_data):
        self.imu_data = imu_data  # e.g., [roll, pitch, yaw]
        self.gps_data = gps_data  # e.g., [latitude, longitude, altitude]

    def fuse_sensors(self):
        # In a real-world scenario, this could be a Kalman filter or complementary filter.
        fused_data = np.average([self.imu_data, self.gps_data], axis=0)
        return fused_data

# Example usage:
if __name__ == "__main__":
    imu_data = np.array([10, 5, 3])  # Example IMU data (roll, pitch, yaw)
    gps_data = np.array([19.0760, 72.8777, 100])  # Example GPS data (latitude, longitude, altitude)

    fusion = SensorFusion(imu_data, gps_data)
    fused_data = fusion.fuse_sensors()
    print(f"Fused Data: {fused_data}")
