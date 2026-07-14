import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class RobotMover(Node):
    def __init__(self):
        super().__init__("move_publisher")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "wheel_velocity_topic_prefix",
            "/m_vel",
        )

        self.declare_parameter("wheel_radius", 0.0625)
        self.declare_parameter("track_width", 0.42)

        self.declare_parameter("max_linear_velocity", 0.375)
        self.declare_parameter("max_wheel_velocity", 6.0)
        self.declare_parameter("min_wheel_velocity", 1.0)
        self.declare_parameter("wheel_deadband", 0.2)

        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("cmd_vel_timeout", 0.25)

        self.wheel_radius = self.get_parameter(
            "wheel_radius"
        ).value

        self.track_width = self.get_parameter(
            "track_width"
        ).value

        self.half_track_width = self.track_width / 2.0

        self.max_wheel_velocity = self.get_parameter(
            "max_wheel_velocity"
        ).value

        self.min_wheel_velocity = self.get_parameter(
            "min_wheel_velocity"
        ).value

        self.wheel_deadband = self.get_parameter(
            "wheel_deadband"
        ).value

        self.max_linear_velocity = self.get_parameter(
            "max_linear_velocity"
        ).value

        self.cmd_vel_timeout = self.get_parameter(
            "cmd_vel_timeout"
        ).value

        self.check_parameters()

        # Физический максимум при движении прямо:
        # 6 рад/с × 0.0625 м = 0.375 м/с.
        physical_max_linear_velocity = (
            self.max_wheel_velocity * self.wheel_radius
        )

        self.max_linear_velocity = min(
            self.max_linear_velocity,
            physical_max_linear_velocity,
        )

        # Физический максимум угловой скорости при развороте
        # на месте: примерно 1.786 рад/с.
        self.max_angular_velocity = (
            self.max_wheel_velocity
            * self.wheel_radius
            / self.half_track_width
        )

        topic_prefix = self.get_parameter(
            "wheel_velocity_topic_prefix"
        ).value

        self.wheel_publishers = [
            self.create_publisher(
                Float32,
                f"{topic_prefix}{idx}",
                10,
            )
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

        self.get_logger().info(
            "Move controller started: "
            f"track={self.track_width:.3f} m, "
            f"wheel radius={self.wheel_radius:.4f} m, "
            f"wheel velocity={self.min_wheel_velocity:.1f}"
            f"...{self.max_wheel_velocity:.1f} rad/s, "
            f"max linear velocity="
            f"{self.max_linear_velocity:.3f} m/s"
        )

    def check_parameters(self):
        if self.max_linear_velocity <= 0.0:
            raise ValueError(
                "max_linear_velocity must be positive"
            )
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")

        if self.track_width <= 0.0:
            raise ValueError("track_width must be positive")

        if self.max_wheel_velocity <= 0.0:
            raise ValueError(
                "max_wheel_velocity must be positive"
            )

        if not (
            0.0
            <= self.wheel_deadband
            <= self.min_wheel_velocity
            <= self.max_wheel_velocity
        ):
            raise ValueError(
                "Expected: 0 <= wheel_deadband "
                "<= min_wheel_velocity "
                "<= max_wheel_velocity"
            )

    def timer_callback(self):
        if self.is_cmd_vel_stale():
            self.set_wheel_velocities(0.0, 0.0)

        self.publish_wheel_velocities()

    def publish_wheel_velocities(self):
        for idx, publisher in enumerate(
            self.wheel_publishers
        ):
            if idx < 3:
                publisher.publish(self.left_velocity)
            else:
                publisher.publish(self.right_velocity)

    def is_cmd_vel_stale(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9

        return elapsed > self.cmd_vel_timeout

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        if (
            not math.isfinite(linear)
            or not math.isfinite(angular)
        ):
            self.get_logger().warning(
                "Non-finite cmd_vel received, stopping"
            )
            self.set_wheel_velocities(0.0, 0.0)
            return

        linear = self.clamp(
            linear,
            -self.max_linear_velocity,
            self.max_linear_velocity,
        )

        angular = self.clamp(
            angular,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        left = (
            linear
            - angular * self.half_track_width
        ) / self.wheel_radius

        right = -(
            linear
            + angular * self.half_track_width
        ) / self.wheel_radius

        maximum_requested = max(
            abs(left),
            abs(right),
        )

        if maximum_requested > self.max_wheel_velocity:
            scale = (
                self.max_wheel_velocity
                / maximum_requested
            )

            left *= scale
            right *= scale

        self.set_wheel_velocities(left, right)

    def set_wheel_velocities(self, left, right):
        self.left_velocity.data = self.limit_velocity(
            left
        )
        self.right_velocity.data = self.limit_velocity(
            right
        )

    def limit_velocity(self, value):
        if not math.isfinite(value):
            return 0.0

        magnitude = abs(value)

        if magnitude < self.wheel_deadband:
            return 0.0

        if magnitude < self.min_wheel_velocity:
            magnitude = self.min_wheel_velocity

        if magnitude > self.max_wheel_velocity:
            magnitude = self.max_wheel_velocity

        return math.copysign(magnitude, value)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    try:
        rclpy.spin(robot_mover)
    except KeyboardInterrupt:
        pass
    finally:
        robot_mover.set_wheel_velocities(0.0, 0.0)
        robot_mover.publish_wheel_velocities()

        robot_mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()