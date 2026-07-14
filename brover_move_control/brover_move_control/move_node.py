import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class RobotMover(Node):
    def __init__(self):
        super().__init__("move_publisher")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_velocity_topic_prefix", "/m_vel")
        self.declare_parameter("wheel_radius", 0.0625)
        self.declare_parameter("track_width", 0.42)
        self.declare_parameter("max_wheel_velocity", 6.0)
        self.declare_parameter("wheel_deadband", 0.001)
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("cmd_vel_timeout", 0.25)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.half_track_width = self.get_parameter("track_width").value / 2.0
        self.max_wheel_velocity = self.get_parameter(
            "max_wheel_velocity"
        ).value
        self.wheel_deadband = self.get_parameter("wheel_deadband").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value

        topic_prefix = self.get_parameter("wheel_velocity_topic_prefix").value
        self.wheel_publishers = [
            self.create_publisher(Float32, f"{topic_prefix}{idx}", 10)
            for idx in range(1, 7)
        ]

        self.left_velocity = Float32()
        self.right_velocity = Float32()
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )
        self.create_timer(
            self.get_parameter("publish_period").value,
            self.timer_callback,
        )

    def timer_callback(self):
        if self.is_cmd_vel_stale():
            self.set_wheel_velocities(0.0, 0.0)

        for idx, publisher in enumerate(self.wheel_publishers):
            velocity = self.left_velocity if idx < 3 else self.right_velocity
            publisher.publish(velocity)

    def is_cmd_vel_stale(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        return elapsed > self.cmd_vel_timeout

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = (linear - angular * self.half_track_width) / self.wheel_radius
        right = -(linear + angular * self.half_track_width) / self.wheel_radius
        self.set_wheel_velocities(left, right)

    def set_wheel_velocities(self, left, right):
        self.left_velocity.data = self.limit_velocity(left)
        self.right_velocity.data = self.limit_velocity(right)

    def limit_velocity(self, value):
        if abs(value) < self.wheel_deadband:
            return 0.0
        if not math.isfinite(value):
            self.get_logger().warning(
                "Non-finite wheel velocity requested, stopping"
            )
            return 0.0
        return max(
            -self.max_wheel_velocity,
            min(self.max_wheel_velocity, value),
        )


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    try:
        rclpy.spin(robot_mover)
    except KeyboardInterrupt:
        pass
    finally:
        robot_mover.set_wheel_velocities(0.0, 0.0)
        robot_mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
