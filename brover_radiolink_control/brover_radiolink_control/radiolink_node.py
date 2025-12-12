import scipy.signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import scipy
import numpy as np

SLOW = 0.2
NORMAL = 0.4
FAST = 0.8
ROTATION = 2


class RadiolinkController(Node):
    def __init__(self):
        super().__init__("radiolink")

      
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_msg = Twist()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vel_coeff = SLOW

        self.off_mode = True
        self.is_on = 0


    def joy_callback(self, msg: Joy):

        if msg.axes[2] == 1.0 or msg.axes[4] < 0.5:
            self.off_mode = True
        else:
            self.off_mode = False
            self.is_on = 1

        if self.off_mode == True:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0

        else:
            if msg.axes[6] > 0.5:
                self.vel_coeff = SLOW
            elif msg.axes[6] < -0.5:
                self.vel_coeff = FAST
            else: self.vel_coeff = NORMAL 

            self.cmd_vel_msg.linear.x = self.vel_coeff * msg.axes[1]
            self.cmd_vel_msg.angular.z = self.vel_coeff *ROTATION* msg.axes[0]           

    def timer_callback(self):
        if self.is_on:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if self.off_mode: 
                self.is_on = 0
        #self.get_logger().info('Publishing: "%s"' % self.is_on)


def main(args=None):
    rclpy.init(args=args)

    radiolink = RadiolinkController()

    rclpy.spin(radiolink)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    radiolink.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()