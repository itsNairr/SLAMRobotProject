import serial
import numpy as np
import time
class Odometry:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta

    def update(self, v, omega, dt):
        self.x     += v * np.cos(self.theta) * dt
        self.y     += v * np.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + np.pi) % (2*np.pi) - np.pi  # wrap

    def pose(self):
        return self.x, self.y, self.theta

def read_serial_data(ser):
    line = ser.readline().decode().strip()
    if not line:
        return None
    try:
        v, omega, dt = map(float, line.split(','))
        return v, omega, dt
    except ValueError:
        return None

def main():
    ser = serial.Serial('COM3', 9600, timeout=0.25)
    time.sleep(2)                             # wait for Arduino reset
    odom = Odometry()
    print("Starting odometry test.")
    print("Press Ctrl-C to stop.")

    try:
        while True:
            data = read_serial_data(ser)
            if data is None:
                continue                      # skip bad/empty lines
            v, omega, dt = data
            odom.update(v, omega, dt)
            x, y, th = odom.pose()
            print(f"x={x:.2f}  y={y:.2f}  θ={th:.2f}")
            # no explicit sleep—Arduino dictates the cadence
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()