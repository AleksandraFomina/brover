#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math



class SimpleMotion(Node):
    def __init__(self):
        super().__init__("simple_motion")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Pose2D, "/odom_pose2d", self.odom_callback, 10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_received = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = "forward"  # forward → rotate_left → rotate_right -> backward → stop

    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta

        if not self.start_received:
            self.start_received = True
            self.x0 = self.x
            self.y0 = self.y
            self.yaw0 = self.yaw

    def control_loop(self):
        if not self.start_received:
            return  

        cmd = Twist()

        if self.state == "forward":
            dist = math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2)

            if dist < 1.0:
                cmd.linear.x = 0.3
            else:
                self.get_logger().info("Reached 1m forward.")
                self.state = "rotate_left"
                self.yaw0 = self.yaw  

        
        elif self.state == "rotate_left":
            dtheta = self.normalize_angle(self.yaw - self.yaw0)

            if abs(dtheta) < 1.57:
                cmd.angular.z = 0.5
                self.get_logger().info('Angle: "%s"' % dtheta)
            else:
                self.get_logger().info("Turned PI/2 radians.")
                self.state = "rotate_right"
                self.x0 = self.x
                self.y0 = self.y
        
        elif self.state == "rotate_right":
            dtheta = self.normalize_angle(self.yaw - self.yaw0)

            if abs(dtheta) > 0.01:
                cmd.angular.z = -0.3
                self.get_logger().info('Angle: "%s"' % dtheta)
            else:
                self.get_logger().info("Turned -PI/2 radians.")
                self.state = "back"
                self.x0 = self.x
                self.y0 = self.y

      
        elif self.state == "back":
            dist = math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2)

            if dist < 1.0:
                cmd.linear.x = -0.2
            else:
                self.get_logger().info("Reached 1m backward.")
                self.state = "stop"

       
        elif self.state == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Done.")
            rclpy.shutdown()
            return

        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMotion()
    rclpy.spin(node)


if __name__ == "__main__":
    main()