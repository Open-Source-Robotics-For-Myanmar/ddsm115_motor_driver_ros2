# ddsm115_motor_driver_ros2

ROS 2 `ros2_control` hardware interface for Waveshare DDSM115 motor wheels on an RS485 bus.

The normal drive stack is:

```text
Nav2 /cmd_vel_nav
Teleop /cmd_vel_teleop
        |
        v
twist_mux
        |
        v
diff_drive_controller
        |
        v
ros2_control hardware interface
        |
        v
DDSM115 motors
```

## Install Required ROS 2 Humble Packages

Run the following commands to update your packages and install the required dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ament-cmake \
  ros-humble-hardware-interface \
  ros-humble-pluginlib \
  ros-humble-rclcpp \
  ros-humble-rclcpp-lifecycle \ 
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rqt-publisher \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-teleop-twist-joy
```
## Motor ID Setup

Assign a unique ID to each motor before running the driver. Connect only one motor to the RS485 adapter while assigning an ID. If multiple unconfigured motors are connected, they can all receive the same ID.

Build and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select ddsm115_motor_driver_ros2
source install/setup.bash
```

Assign ID `1`:

```bash
ros2 run ddsm115_motor_driver_ros2 ddsm115_set_id.py /dev/ttyACM0 1
```

Power-cycle the motor after the command finishes, then repeat for the other motors:

```bash
ros2 run ddsm115_motor_driver_ros2 ddsm115_set_id.py /dev/ttyACM0 2
ros2 run ddsm115_motor_driver_ros2 ddsm115_set_id.py /dev/ttyACM0 3
ros2 run ddsm115_motor_driver_ros2 ddsm115_set_id.py /dev/ttyACM0 4
```

## Dependencies

Install the standard ROS 2 control packages:

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-twist-mux
```

If the serial port needs permission:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in after changing group membership.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ddsm115_motor_driver_ros2
source install/setup.bash
```

## Configure Hardware

Edit the ros2_control block in:

```text
description/urdf/ddsm115_base.urdf
```

The important hardware settings are:

```xml
<hardware>
  <plugin>ddsm115_motor_driver_ros2/Ddsm115HardwareInterface</plugin>
  <param name="port_name">/dev/ttyACM0</param>
</hardware>
```

Set each motor ID and direction on its joint:

```xml
<joint name="front_left_wheel_joint">
  <param name="id">1</param>
  <param name="direction">forward</param>
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

Use `direction="backward"` if that motor spins opposite to the expected robot direction.

Default joint mapping:

| Joint | Motor ID | Direction |
| --- | --- | --- |
| `front_left_wheel_joint` | `1` | `forward` |
| `front_right_wheel_joint` | `2` | `backward` |
| `rear_left_wheel_joint` | `3` | `forward` |
| `rear_right_wheel_joint` | `4` | `backward` |

If you already have a robot URDF, copy the `<ros2_control>` block into your real URDF and launch with:

```bash
ros2 launch ddsm115_motor_driver_ros2 ddsm115_motor_driver.launch.py \
  robot_description_file:=/path/to/your_robot.urdf
```

## Configure diff_drive_controller

Edit:

```text
config/ros2_control.yaml
```

Important values:

| Parameter | Description |
| --- | --- |
| `left_wheel_names` | Left wheel joints controlled by `diff_drive_controller`. |
| `right_wheel_names` | Right wheel joints controlled by `diff_drive_controller`. |
| `wheel_radius` | Wheel radius in meters. DDSM115 is about `0.0575` m radius. |
| `wheel_separation` | Distance in meters between left and right wheel contact lines. Measure your robot. |
| `wheels_per_side` | `2` for a four-wheel skid/differential base. |
| `odom_frame_id` | Usually `odom`. |
| `base_frame_id` | Usually `base_link` or `base_footprint`. |
| `cmd_vel_timeout` | Stops the controller if command velocity stops arriving. |

## Configure twist_mux

Edit:

```text
config/twist_mux.yaml
```

Default inputs:

| Source | Topic | Priority |
| --- | --- | --- |
| Nav2 | `/cmd_vel_nav` | `10` |
| Teleop | `/cmd_vel_teleop` | `100` |

The launch file remaps `twist_mux` output to:

```text
/diff_drive_controller/cmd_vel_unstamped
```

That is the command topic used by `diff_drive_controller` when `use_stamped_vel: false`.

## Run

Start the full stack:

```bash
ros2 launch ddsm115_motor_driver_ros2 ddsm115_motor_driver.launch.py
```

The launch starts:

- `robot_state_publisher`
- `controller_manager` / `ros2_control_node`
- `joint_state_broadcaster`
- `diff_drive_controller`
- `twist_mux`

## Test Commands

Drive forward through the Nav2 mux input:

```bash
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

Turn in place:

```bash
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

Stop:

```bash
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

Check controllers:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Check odometry:

```bash
ros2 topic echo /diff_drive_controller/odom
```

## Print Wheel Encoder Counts

With the motor-driver launch running, start the encoder-count printer in a
second terminal:

```bash
source install/setup.bash
ros2 run ddsm115_motor_driver_ros2 ddsm115_encoder_count_printer.py
```

It prints every wheel received on `/joint_states` as a continuous encoder count
using the DDSM115 scale of `32767` counts per revolution. To change the output
rate (seconds) or input topic:

```bash
ros2 run ddsm115_motor_driver_ros2 ddsm115_encoder_count_printer.py \
  --ros-args -p print_period:=1.0 -p joint_states_topic:=/joint_states
```

## Troubleshooting

If `joint_state_broadcaster` prints this, it is normal:

```text
'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
```

If the spawner says a controller is already loaded or cannot be configured from `active` state, an old launch is probably still running. Stop the old launch, then check:

```bash
ros2 control list_controllers
```

If controllers are still listed, stop the leftover ROS processes or restart the terminal/session before launching again. The launch file spawns `joint_state_broadcaster` and `diff_drive_controller` together, so a clean launch should not try to configure an already-active controller.

## Nav2 Connection

Configure Nav2 to publish velocity commands to:

```text
/cmd_vel_nav
```

Or remap Nav2 controller output:

```text
/cmd_vel -> /cmd_vel_nav
```

Teleop can publish to:

```text
/cmd_vel_teleop
```

Because teleop has higher priority in `twist_mux.yaml`, it overrides Nav2 while active.

## Notes

- The DDSM115 hardware interface exports standard `velocity` command interfaces and `position`/`velocity` state interfaces.
- The hardware interface sends `0 RPM` to all motors when deactivated or cleaned up.
- Odometry comes from `diff_drive_controller` using wheel velocity state. Tune `wheel_radius` and `wheel_separation` carefully for accurate odom.
- If the robot moves backward, swap `forward`/`backward` in the URDF joint params.
- If the robot rotates when commanded forward, check left/right wheel joint grouping in `ros2_control.yaml`.
