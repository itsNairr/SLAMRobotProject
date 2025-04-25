import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import math
import serial
import numpy as np
import time


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Initialize publisher for Odometry and Velocity messages and other broadcasters stuff (help me plz)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.broadcaster = TransformBroadcaster(self)
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)  # Allow time for the connection to establish

        # Initialize odometry with the starting position (0, 0, 0)
        self.odom = RobotOdometry()
        
        # Define time interval
        self.dt = 0.250  # time interval in seconds
        
        # Timer to periodically update odom
        self.create_timer(self.dt, self.update_odometry)

    def read_serial_data(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            v, omega = map(float, line.split(','))
            return v, omega
        except:
            self.get_logger().warn("Failed to read serial data")
            return 0.0, 0.0

    def update_odometry(self):
        # Read velocity data from serial
        v, omega = self.read_serial_data()
        
        # Update odometry
        self.odom.update(v, omega, self.dt)
        
        # Get the current pose
        x, y, theta = self.odom.get_pose()

        # Quaternion from Euler angles
        quaternion = Quaternion()
        q = self.euler_to_quaternion(0, 0, theta)
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]
        
        # Publish the odometry data
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.angular.z = omega
        self.odom_publisher.publish(odom_msg)

        # Publish the transform data
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion
        self.broadcaster.sendTransform(t)


        self.get_logger().info(f"x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}")

    def destroy_node(self):
        super().destroy_node()
        self.ser.close()

class RobotOdometry:
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
    
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()

    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
