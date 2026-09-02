# Third-party components

This workspace vendors several upstream ROS 2 packages so the robot can be
built from a single checkout. They are **not my work** and retain their own
licences and copyright. My contribution is the integration: the workspace
layout, hardware wiring, controller configuration and bring-up.

| Directory | Upstream author | Licence | Purpose |
|---|---|---|---|
| `serial/` | William Woodall (OSRF) | MIT | Cross-platform C++ serial port library |
| `diffdrive_arduino/` | Josh Newans | BSD 3-Clause | `ros2_control` hardware interface for an Arduino diff-drive base |
| `serial_motor_demo/` | Josh Newans | BSD 3-Clause | Serial motor command/encoder message definitions and driver |

Upstream sources:

- <https://github.com/wjwwood/serial>
- <https://github.com/joshnewans/diffdrive_arduino>
- <https://github.com/joshnewans/serial_motor_demo>

If you are evaluating this repository as a work sample, treat the vendored
directories as dependencies rather than as authored code.
