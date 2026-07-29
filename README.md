# BRover ROS 2 packages

Репозиторий `brover` содержит основные ROS 2 пакеты управления ровером BRover. Пакеты рассчитаны на ROS 2 Jazzy и работают вместе с `cyphal_ros2_bridge`, который связывает ROS-топики и сервисы с бортовой CAN/Cyphal сетью.

На ровере пакет обычно запускается автоматически через systemd-сервис `ros_nodes.service`.

## Состав

```text
brover_control/             общий launch-файл и конфигурация
brover_move_control/        преобразование /cmd_vel в команды шести колес
brover_radiolink_control/   управление от Radiolink-джойстика
brover_odom_simple/         простая 2D-одометрия по энкодерам и IMU yaw
brover_bat_monitor/         мониторинг батареи и управление HMI LED
brover_imu_node/            публикация BHI360 IMU из USB HID интерфейса
test_move.py                ручной сценарий движения для проверки управления
test_encoders.py            терминальный монитор значений /m_odom1 ... /m_odom6
```

## Запуск

Основной launch-файл находится в `brover_control/launch/brover_control_launch.xml`. Он запускает:

- `cyphal_ros2_bridge/cyphal_bridge`;
- `brover_move_control/move_node` с именем ноды `control_move`;
- `joy/joy_node` с именем ноды `joy`;
- `brover_radiolink_control/radiolink_node` с именем ноды `radiolink_control`;
- `brover_imu_node/imu_node` с именем ноды `imu`;
- `brover_odom_simple/odom_pose2d` с именем ноды `odom`;
- `brover_bat_monitor/bat_monitor_node` с именем ноды `bat_monitor`.

## Основные топики

| Топик | Тип | Назначение |
| --- | --- | --- |
| `/joy` | `sensor_msgs/msg/Joy` | вход от джойстика Radiolink через `joy_node` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | команда линейной и угловой скорости ровера |
| `/m_vel1` ... `/m_vel6` | `std_msgs/msg/Float32` | заданные угловые скорости колес, рад/с |
| `/m_odom1` ... `/m_odom6` | `std_msgs/msg/Float32` | измеренные скорости/одометрия колес от приводов |
| `/bhi360/imu` | `sensor_msgs/msg/Imu` | данные IMU BHI360 |
| `/odom_pose2d` | `geometry_msgs/msg/Pose2D` | простая оценка положения `x`, `y`, `theta` |
| `/bat` | `sensor_msgs/msg/BatteryState` | состояние батареи |

Дополнительно `brover_odom_simple` предоставляет сервис:

```text
/odom/reset
```

Он сбрасывает `x`, `y` и принимает текущий yaw IMU за новый ноль.

## Обновление пакета

Чтобы обновить пакет на ровере, подключитесь к Raspberry Pi по SSH и выполните команды:

```bash
cd ~/ros2_ws/src/brover
git pull

cd ~/ros2_ws
colcon build --symlink-install --packages-up-to brover_control

sudo systemctl restart ros_nodes.service
```

После перезапуска сервиса можно проверить его состояние:

```bash
systemctl status ros_nodes.service
```

## Проверка

Запустить монитор энкодеров:

```bash
cd ~/ros2_ws/src/brover
python3 test_encoders.py
```

Запустить тестовый сценарий движения:

```bash
cd ~/ros2_ws/src/brover
python3 test_move.py
```

`test_move.py` публикует команды в `/cmd_vel`: движение вперед примерно на 1 метр, поворот, возврат и остановка. Запускайте его только на подготовленной площадке, где ровер может безопасно двигаться.
