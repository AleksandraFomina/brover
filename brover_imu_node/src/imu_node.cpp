#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "hidapi/hidapi.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define MAX_STR 255

using namespace std::chrono_literals;

class BHI360 : public rclcpp::Node
{
public:
  BHI360()
  : Node("bhi360_imu_node")
  {
    this->declare_parameter("freq", 500.0);
    this->declare_parameter("imu_topic", "/bhi360/imu");
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("reconnect_period", 1.0);

    const auto imu_topic = this->get_parameter("imu_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    reconnect_period_ = this->get_parameter("reconnect_period").as_double();

    const double freq = this->get_parameter("freq").as_double();
    if (freq <= 0.0) {
      throw std::runtime_error("IMU frequency must be positive");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / freq),
      std::bind(&BHI360::timer_callback, this));

    if (!open_imu_hid()) {
      RCLCPP_WARN(this->get_logger(), "IMU is not available yet, waiting for reconnect");
    }
  }

  ~BHI360() override
  {
    close_imu_hid();
    hid_exit();
  }

private:
  static int16_t read_i16(const uint8_t * buffer, size_t offset)
  {
    int16_t value;
    std::memcpy(&value, buffer + offset, sizeof(value));
    return value;
  }

  void timer_callback()
  {
    if (acc_handle_ == nullptr) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(
        now - last_reconnect_attempt_).count();
      if (elapsed < reconnect_period_) {
        return;
      }
      last_reconnect_attempt_ = now;
      if (!open_imu_hid()) {
        return;
      }
    }

    uint8_t buffer[64] = {};
    const int bytes_number = hid_read(acc_handle_, buffer, sizeof(buffer));

    if (bytes_number < 0) {
      RCLCPP_ERROR(this->get_logger(), "IMU USB read error, closing device");
      close_imu_hid();
      return;
    }

    if (bytes_number < 26) {
      return;
    }

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    const int16_t acc_x = read_i16(buffer, 2);
    const int16_t acc_y = read_i16(buffer, 4);
    const int16_t acc_z = read_i16(buffer, 6);

    imu_msg.linear_acceleration.x = acc_x * 9.81f / 4096.0f;
    imu_msg.linear_acceleration.y = acc_y * 9.81f / 4096.0f;
    imu_msg.linear_acceleration.z = acc_z * 9.81f / 4096.0f;

    const int16_t gyr_x = read_i16(buffer, 8);
    const int16_t gyr_y = read_i16(buffer, 10);
    const int16_t gyr_z = read_i16(buffer, 12);

    imu_msg.angular_velocity.x = gyr_x * 6.28f * 2000.0f / 32768.0f / 360.0f;
    imu_msg.angular_velocity.y = gyr_y * 6.28f * 2000.0f / 32768.0f / 360.0f;
    imu_msg.angular_velocity.z = gyr_z * 6.28f * 2000.0f / 32768.0f / 360.0f;

    const int16_t quat_x = read_i16(buffer, 14);
    const int16_t quat_y = read_i16(buffer, 16);
    const int16_t quat_z = read_i16(buffer, 18);
    const int16_t quat_w = read_i16(buffer, 20);

    imu_msg.orientation.x = quat_x / 16384.0f;
    imu_msg.orientation.y = quat_y / 16384.0f;
    imu_msg.orientation.z = quat_z / 16384.0f;
    imu_msg.orientation.w = quat_w / 16384.0f;

    publisher_->publish(imu_msg);
  }

  bool open_imu_hid()
  {
    if (hid_init() != 0) {
      RCLCPP_ERROR(this->get_logger(), "Unable to initialize hidapi");
      return false;
    }

    hid_device_info * imu_info = hid_enumerate(0xCafe, 0x4004);
    if (imu_info == nullptr) {
      return false;
    }

    acc_handle_ = hid_open_path(imu_info->path);
    hid_free_enumeration(imu_info);

    if (acc_handle_ == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open IMU HID interface");
      return false;
    }

    hid_set_nonblocking(acc_handle_, 1);
    log_device_info();
    return true;
  }

  void log_device_info()
  {
    wchar_t wstr[MAX_STR] = {};
    if (hid_get_manufacturer_string(acc_handle_, wstr, MAX_STR) == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU manufacturer: %ls", wstr);
    }
    if (hid_get_product_string(acc_handle_, wstr, MAX_STR) == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU product: %ls", wstr);
    }
    if (hid_get_serial_number_string(acc_handle_, wstr, MAX_STR) == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU serial: %ls", wstr);
    }
  }

  void close_imu_hid()
  {
    if (acc_handle_ != nullptr) {
      hid_close(acc_handle_);
      acc_handle_ = nullptr;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::string frame_id_;
  double reconnect_period_ = 1.0;
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};
  hid_device * acc_handle_ = nullptr;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BHI360>());
  rclcpp::shutdown();
  return 0;
}
