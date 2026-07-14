#!/usr/bin/env python3
import rclpy
from cyphal_ros2_bridge.srv import CallHMILed
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class LowBatteryLedBlinker(Node):
    def __init__(self):
        super().__init__("bat_monitor")

        self.declare_parameter("battery_topic", "/bat")
        self.declare_parameter("led_service", "/hmi/led")
        self.declare_parameter("voltage_threshold", 13.8)
        self.declare_parameter("blink_period", 0.25)
        self.declare_parameter("led_interface", 0)

        self.voltage_threshold = self.get_parameter("voltage_threshold").value
        self.led_interface = self.get_parameter("led_interface").value
        self.low_battery = False
        self.led_state = False
        self.pending_led_call = None
        self.last_led = None

        self.create_subscription(
            BatteryState,
            self.get_parameter("battery_topic").value,
            self.battery_callback,
            10,
        )
        self.led_client = self.create_client(
            CallHMILed,
            self.get_parameter("led_service").value,
        )
        self.create_timer(
            self.get_parameter("blink_period").value,
            self.timer_callback,
        )

        self.get_logger().info("Low battery LED blinker started")

    def battery_callback(self, msg):
        voltage = msg.voltage
        if voltage <= self.voltage_threshold:
            if not self.low_battery:
                self.get_logger().warning(
                    "Low battery detected: "
                    f"{voltage:.2f} V <= {self.voltage_threshold:.2f} V"
                )
            self.low_battery = True
            return

        if self.low_battery:
            self.get_logger().info(
                f"Battery voltage restored: {voltage:.2f} V"
            )

        self.low_battery = False
        self.led_state = False

    def timer_callback(self):
        if (
            self.pending_led_call is not None
            and not self.pending_led_call.done()
        ):
            return

        if self.low_battery:
            self.led_state = not self.led_state
            if self.led_state:
                self.send_led(255, 0, 0)
            else:
                self.send_led(0, 0, 0)
            return

        self.send_led(0, 255, 0)

    def send_led(self, r, g, b):
        desired_led = (r, g, b)
        if desired_led == self.last_led and not self.low_battery:
            return
        if (
            self.pending_led_call is not None
            and not self.pending_led_call.done()
        ):
            return
        if not self.led_client.service_is_ready():
            self.get_logger().warning("/hmi/led service is not ready")
            return

        request = CallHMILed.Request()
        request.led.r = r
        request.led.g = g
        request.led.b = b
        request.led.interface = self.led_interface

        self.last_led = desired_led
        self.pending_led_call = self.led_client.call_async(request)
        self.pending_led_call.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            future.result()
        except Exception as exc:
            self.last_led = None
            self.get_logger().error(f"LED service call failed: {exc}")
        finally:
            self.pending_led_call = None


def main(args=None):
    rclpy.init(args=args)
    node = LowBatteryLedBlinker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_led(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
