import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class RobotMover(Node):

    def __init__(self):
        super().__init__('move_publisher')

        self.__pubs = []
        for i in range(6):
            topic = "/m_vel"+str(i+1)
            self.__pubs.append(self.create_publisher(Float32, topic, 10))


        self.__subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.__max_vel = 10.13
        self.__W = 0.44 # width
        self.__B = self.__W/2.0
        self.__R = 0.0625 #wheel radius

        vel123 = Float32()
        vel456 = Float32()
        self.__vel = [vel123, vel456]


    def timer_callback(self):
        
        for  i, pub in enumerate(self.__pubs):
            if i<3: pub.publish(self.__vel[0])
            else: pub.publish(self.__vel[1])
           # self.get_logger().info('v1,v2,v3:  "%s"' % self.__vel[0].data)
           # self.get_logger().info('v4,v5,v6: "%s"' % self.__vel[1].data)
  

    def listener_callback(self, msg):
        x = float(msg.linear.x)
        zB = float(msg.angular.z)*self.__B
        self.__vel[0].data= (x - zB)/self.__R #left wheels = (v - w*B)/r
        self.__vel[1].data= -(x + zB)/self.__R #right wheels (v + w*B)/r
        # self.get_logger().info('v1,v2,v3:  "%s"' % self.__vel[0].data)
        # self.get_logger().info('v4,v5,v6: "%s"' % self.__vel[1].data)

        
        


def main(args=None):
    rclpy.init(args=args)

    robot_mover = RobotMover()

    rclpy.spin(robot_mover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()