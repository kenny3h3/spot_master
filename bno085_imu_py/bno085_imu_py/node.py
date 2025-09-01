#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR
)

def quat_wxyz_to_xyzw(q):
    w, x, y, z = q
    return (x, y, z, w)

class BNO085ImuNode(Node):
    def __init__(self):
        super().__init__('bno085_imu')
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('i2c_address', 0x4B)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate_hz', 50)

        bus = int(self.get_parameter('i2c_bus').value)
        addr = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        rate_hz = float(self.get_parameter('pub_rate_hz').value)

        self.get_logger().info(f'Opening BNO085 on I2C bus={bus} addr=0x{addr:02X}')
        i2c = I2C(bus)
        self.imu = BNO08X_I2C(i2c, address=addr)

        # Enable desired features
        self.imu.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.imu.enable_feature(BNO_REPORT_GYROSCOPE)
        self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(BNO_REPORT_MAGNETOMETER)

        self.pub = self.create_publisher(Imu, 'imu', 10) 
        self.mag_pub = self.create_publisher(MagneticField, 'mag', 10)
        period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self.timer = self.create_timer(period, self.publish_imu)

    def publish_imu(self):
        msg = Imu()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id

        try:
            # Orientation (quaternion)
            q = self.imu.quaternion  # (w,x,y,z) or None
            if q:
                x,y,z,w = quat_wxyz_to_xyzw(q)
                msg.orientation.x = x
                msg.orientation.y = y
                msg.orientation.z = z
                msg.orientation.w = w

            # Angular velocity (rad/s)
            g = self.imu.gyro  # (x,y,z) rad/s
            if g:
                msg.angular_velocity.x = g[0]
                msg.angular_velocity.y = g[1]
                msg.angular_velocity.z = g[2]

            # Linear acceleration (m/s^2)
            a = self.imu.acceleration  # (x,y,z) m/s^2
            if a:
                msg.linear_acceleration.x = a[0]
                msg.linear_acceleration.y = a[1]
                msg.linear_acceleration.z = a[2]

            # Covariances unbekannt → -1 (nicht bereitgestellt)
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'IMU read failed: {e}')

	# Magnetometer (µT)
        try:
            m = self.imu.magnetic  # (x,y,z) µTesla
            if m:
                mag_msg = MagneticField()
                mag_msg.header.stamp = now
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = m[0]
                mag_msg.magnetic_field.y = m[1]
                mag_msg.magnetic_field.z = m[2]
            self.mag_pub.publish(mag_msg)
        except Exception as e:
    	    self.get_logger().warn(f'Mag read failed: {e}')

def main():
    rclpy.init()
    node = BNO085ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
