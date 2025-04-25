import serial
import numpy as np
import time


class Odometry:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
    def update(self, v, omega, dt):
        # Calculate the change in position and orientation
        delta_x = v * np.cos(self.theta) * dt
        delta_y = v * np.sin(self.theta) * dt
        delta_theta = omega * dt

        # Update the position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize the angle theta to be within -pi to pi
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

    def get_pose(self):
        return self.x, self.y, self.theta
    
def read_serial_data(serial_port):
    line = serial_port.readline().decode('utf-8').strip()
    v, omega = map(float, line.split(','))
    return v, omega

def main():
    # Initialize serial port (adjust '/dev/ttyUSB0' or 'COM3' to your specific port)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)  # Allow time for the connection to establish

    # Initialize odometry with the starting position (0, 0, 0)
    odom = Odometry()

    # Define time interval
    dt = 0.250  # time interval in seconds

    try:
        while True:
            # Read velocity data from serial
            v, omega = read_serial_data(ser)
            
            # Update odometry
            odom.update(v, omega, dt)
            
            # Get and print the current pose
            x, y, theta = odom.get_pose()
            print(f"x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}")
            
            # Wait for the next time interval
            time.sleep(dt)
    
    except KeyboardInterrupt:
        print("Stopping odometry calculation.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()