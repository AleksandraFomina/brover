#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty
import math
import time


class OdomPose2D(Node):
    def __init__(self):
        super().__init__('odom_pose2d_node')

        self.R = 0.0625     # wheel radius
        self.W = 0.44       # distance between left-right centers

        self.wheel_omegas = [0.0] * 6
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_offset = 0.0

        self.last_time = time.monotonic()

        self.create_subscription(Imu, '/bhi360/imu', self.imu_callback, 20)

        self.create_subscription(Float32, '/m_odom1', lambda msg: self.wheel_callback(0, msg), 10)
        self.create_subscription(Float32, '/m_odom2', lambda msg: self.wheel_callback(1, msg), 10)
        self.create_subscription(Float32, '/m_odom3', lambda msg: self.wheel_callback(2, msg), 10)
        self.create_subscription(Float32, '/m_odom4', lambda msg: self.wheel_callback(3, msg), 10)
        self.create_subscription(Float32, '/m_odom5', lambda msg: self.wheel_callback(4, msg), 10)
        self.create_subscription(Float32, '/m_odom6', lambda msg: self.wheel_callback(5, msg), 10)

        
        self.pub_pose = self.create_publisher(Pose2D, '/odom_pose2d', 20)
      
        self.create_timer(0.02, self.update_odometry)  # 50 Hz

     
        self.reset_service = self.create_service(
            Empty,
            '/odom/reset',
            self.handle_reset_odom
        )

        self.get_logger().info("Pose2D node started with reset service")


    def handle_reset_odom(self, request, response):
        self.get_logger().info("Odometry reset requested")

        self.x = 0.0
        self.y = 0.0
        self.yaw_offset = self.yaw
        self.yaw = 0.0
        return response


    def wheel_callback(self, idx, msg: Float32):
        try:
            self.wheel_omegas[idx] = float(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Invalid wheel msg on idx {idx}: {e}")

   
    def imu_callback(self, msg: Imu):
        quaternion = [msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w]
       
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        #self.get_logger().info(f"Euler angles: {euler}")

   
    def update_odometry(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0 or dt > 0.2:
            return

        # averaged left/right wheel speeds
        wL = (self.wheel_omegas[0] + self.wheel_omegas[1] + self.wheel_omegas[2]) / 3.0
        wR = (-self.wheel_omegas[3] - self.wheel_omegas[4] - self.wheel_omegas[5]) / 3.0

        vL = wL * self.R
        vR = wR * self.R

        v = (vL + vR) / 2.0
        yaw = self.yaw-self.yaw_offset

        self.x += v * math.cos(yaw) * dt
        self.y += v * math.sin(yaw) * dt

        pose = Pose2D()
        pose.x = float(self.x)
        pose.y = float(self.y)
        pose.theta = float(yaw)

        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPose2D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
