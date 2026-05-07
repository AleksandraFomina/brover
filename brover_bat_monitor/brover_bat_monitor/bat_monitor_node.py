#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from cyphal_ros2_bridge.srv import CallHMILed


class LowBatteryLedBlinker(Node):
    def __init__(self):
        super().__init__('bat_monitor')

        self.voltage_threshold = 13.8
        self.low_battery = False
        self.led_state = False

        self.bat_sub = self.create_subscription(
            BatteryState,
            '/bat',
            self.battery_callback,
            10
        )

        self.led_client = self.create_client(CallHMILed, '/hmi/led')

        self.timer = self.create_timer(0.25, self.timer_callback)

        self.get_logger().info('Low battery LED blinker started')

    def battery_callback(self, msg: BatteryState):
        voltage = msg.voltage

        if voltage <= self.voltage_threshold:
            if not self.low_battery:
                self.get_logger().warn(
                    f'Low battery detected: {voltage:.2f} V <= {self.voltage_threshold:.2f} V'
                )
            self.low_battery = True
        else:
            if self.low_battery:
                self.get_logger().info(
                    f'Battery voltage restored: {voltage:.2f} V'
                )
                self.send_led(0, 0, 0)

            self.low_battery = False
            self.led_state = False

    def timer_callback(self):
        if not self.low_battery:
            self.send_led(0, 255, 0)
            return

        if not self.led_client.service_is_ready():
            self.get_logger().warn('/hmi/led service is not ready')
            return

        self.led_state = not self.led_state

        if self.led_state:
            self.send_led(255, 0, 0)
        else:
            self.send_led(0, 0, 0)

    def send_led(self, r, g, b):
        request = CallHMILed.Request()

        request.led.r = r
        request.led.g = g
        request.led.b = b
        request.led.interface = 0

        future = self.led_client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'LED service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = LowBatteryLedBlinker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.send_led(0, 0, 0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()