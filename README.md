# ros2-inspection-robot-hw

A `ros2_control` hardware interface for a differential-drive robot, talking over serial
to an Arduino motor controller.

This is the piece that lets `diff_drive_controller` drive real motors. It implements
`hardware_interface::SystemInterface`: ROS 2 asks for wheel velocities, this reads
encoders and writes motor commands over a serial link, and the controller manager runs
the loop.

**Scope: this is a learning-derived reference workspace, not a product.** The
`diffdrive_arduino` package is adapted from Josh Newans' open-source package of the same
name, and the `serial` package is wjwwood's C++ serial library, both used under their
original licenses. See [Attribution](#attribution).

---

## Why a hardware interface instead of a driver node

A standalone driver node that subscribes to a topic and writes to a serial port is
simpler, and it is what
[ros2-diffdrive-robot](https://github.com/Pratyush150/ros2-diffdrive-robot) does. It also
means you write your own odometry, your own velocity limits, and your own kinematics.

Implementing `SystemInterface` instead means the whole `ros2_control` ecosystem applies:
`diff_drive_controller` handles kinematics and publishes odometry and TF,
`joint_state_broadcaster` publishes joint states, and the controller manager owns the
update loop with a fixed rate. You write only the part that is genuinely
robot-specific — reading encoders and writing motor commands.

That is the trade this repo demonstrates.

---

## How it works

**`DiffDriveArduino`** is the `SystemInterface` plugin, exported through
`robot_hardware.xml` and loaded by the controller manager via `pluginlib`.

- `configure()` reads hardware parameters from the URDF `<ros2_control>` block:
  `left_wheel_name`, `right_wheel_name`, `loop_rate`, `device`, `baud_rate`, `timeout`,
  `enc_counts_per_rev`. Nothing is hardcoded in C++.
- `export_state_interfaces()` exposes position and velocity state for each wheel.
- `export_command_interfaces()` exposes a velocity command for each wheel.
- `read()` pulls encoder counts over serial and converts them to wheel angle and
  velocity using `rads_per_count`, derived from counts per revolution.
- `write()` converts the commanded angular velocity into the motor controller's units
  and sends it.

**`Wheel`** holds per-wheel state — encoder count, command, position, velocity — and does
the count-to-radians conversion in one place. Getting `enc_counts_per_rev` wrong scales
every reported velocity by a constant factor, and it will look like a badly tuned
controller rather than a units bug, so this conversion living in exactly one place is
deliberate.

**`ArduinoComms`** wraps the serial connection: open with a timeout, send a command
string, read the response. It speaks the `ros_arduino_bridge` command set — encoder
read, motor write, PID parameter set.

**`FakeRobot`** is the same interface with no serial link. It integrates commanded
velocity into position and reports it straight back. This lets you bring up the
controller stack, check the TF tree, and verify parameter wiring with no hardware
connected. Debugging a controller configuration and a serial protocol at the same time is
how people lose days.

---

## Current state

Read this before you clone it.

The `SystemInterface` implementation targets an older `ros2_control` API — it uses
`hardware_interface::BaseInterface`, `configure()`, `start()` and `stop()`, which were
replaced by the lifecycle-node API (`on_init()`, `on_activate()`, `on_deactivate()`) in
later releases. On a current ROS 2 distro it needs porting to the lifecycle interface.
Parts of the headers in this snapshot are commented out and the workspace does not build
as-is.

It is published as a reference for the structure and the hardware-interface pattern, not
as a package you can `colcon build` today. That is stated here rather than discovered
after twenty minutes of build errors.

---

## Build and run

Requires ROS 2, `ros2_control`, `ros2_controllers`, and the vendored `serial` package.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Pratyush150/ros2-inspection-robot-hw.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Bring up against the fake hardware first:

```bash
ros2 launch diffdrive_arduino fake_robot.launch.py
```

Then against real hardware:

```bash
ros2 launch diffdrive_arduino test_robot.launch.py
```

Check the controllers are live before blaming the hardware:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Drive it:

```bash
ros2 topic pub /diff_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

## Configuration

The controller side, in `controllers/robot_controller_example.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50            # Hz — must be achievable given serial round-trip time
    diff_controller:
      type: diff_drive_controller/DiffDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_controller:
  ros__parameters:
    left_wheel_names: ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']
    wheel_separation: 0.3      # metres, measured between wheel contact patches
    wheel_radius: 0.05         # metres
    base_frame_id: base_link
```

`wheel_separation` and `wheel_radius` must match the real robot, not the URDF's
approximation. A wrong radius scales all odometry linearly; a wrong separation makes
rotation odometry drift while straight lines look fine. That asymmetry is a useful
diagnostic: if driving straight tracks well but turns accumulate heading error,
`wheel_separation` is the first thing to measure again.

The hardware side lives in the URDF `<ros2_control>` block: `device`, `baud_rate`,
`timeout`, `enc_counts_per_rev`, wheel joint names.

---

## File map

```
diffdrive_arduino/
├── include/diffdrive_arduino/
│   ├── diffdrive_arduino.h     SystemInterface declaration
│   ├── arduino_comms.h         serial protocol wrapper
│   ├── wheel.h                 per-wheel state and count-to-radians conversion
│   ├── fake_robot.h            hardware-free interface for bring-up
│   └── config.h                parameter struct with defaults
├── src/                        implementations of the above
├── launch/
│   ├── fake_robot.launch.py    controller stack, no hardware
│   └── test_robot.launch.py    controller stack against the serial device
├── controllers/
│   └── robot_controller_example.yaml
├── robot_hardware.xml          pluginlib export for the real interface
└── fake_robot_hardware.xml     pluginlib export for the fake interface

serial/                         vendored wjwwood/serial C++ library
serial_motor_demo/              standalone serial driver node and Tkinter GUI
```

---

## What this is and is not

**It is** a readable example of the `ros2_control` hardware-interface pattern: parameter
plumbing from URDF to C++, state and command interface export, encoder-to-radians
conversion, and a fake backend for bring-up without hardware.

**It is not** a maintained driver. It targets an older `ros2_control` API, has no serial
reconnect logic, no CRC or framing validation on the serial protocol, and no watchdog if
the Arduino stops responding mid-motion. On a real robot all three of those matter.

For link-loss handling done properly — watchdogs, staleness tracking, reconnect with
backoff — see [px4-mavlink-companion](https://github.com/Pratyush150/px4-mavlink-companion),
where the same class of problem is solved for a flight controller link.

---

## Attribution

`diffdrive_arduino` is adapted from
[joshnewans/diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino) (BSD),
part of the [Articulated Robotics](https://articulatedrobotics.xyz/) material.
`serial` is [wjwwood/serial](https://github.com/wjwwood/serial) (MIT), vendored so the
workspace builds without a system package. `serial_motor_demo` is likewise from the
Articulated Robotics project. Original license headers are retained.

---

## Related work

Actively developed engineering tools:

| Repo | What it does |
|---|---|
| [px4-mavlink-companion](https://github.com/Pratyush150/px4-mavlink-companion) | MAVLink bridge, stale-telemetry watchdog, offboard control, serial auto-discovery |
| [flight-log-analyzer](https://github.com/Pratyush150/flight-log-analyzer) | PX4 ULog / ArduPilot log analysis producing a ranked findings report |
| [jetson-realtime-detection](https://github.com/Pratyush150/jetson-realtime-detection) | Real-time detection and tracking with per-stage latency profiling |
| [lidar-slam-toolkit](https://github.com/Pratyush150/lidar-slam-toolkit) | LiDAR SLAM configs plus extrinsics, time-sync and drift diagnostics |
| [drone-control-toolkit](https://github.com/Pratyush150/drone-control-toolkit) | PID with anti-windup, cascaded loops, LQR, EKF and complementary estimators |
| [ros2-drone-bringup](https://github.com/Pratyush150/ros2-drone-bringup) | ROS 2 bringup for a PX4 aircraft: geodesy, missions, geofence, SITL |

---

## License

MIT for the material authored here. Vendored and adapted components retain their
original licenses.

Copyright (c) 2026 Pratyush Vatsa
