import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        return output

if __name__ == "__main__":
    pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    current_altitude = 100  # current altitude of the quadcopter
    target_altitude = 0    # the target altitude for landing

    while current_altitude > target_altitude:
        control_signal = pid.compute(target_altitude, current_altitude)
        print(f"Control Signal: {control_signal}")
        # In real-world application, control the motors here.
        current_altitude -= control_signal * 0.1  # simulate descent
        time.sleep(0.1)
