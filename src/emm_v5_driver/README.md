# emm_v5_driver

Low-level protocol builder + SocketCAN transport for the Emm_V5.0 closed-loop stepper driver.

## What this package does
- Builds payload bytes for common commands (enable, position mode, sync start, homing).
- Sends commands via Linux SocketCAN using **extended CAN frames**.
- Splits long commands into multiple frames using the manual’s rule:
  - CAN ID = `(addr << 8) | packet_index`
  - Each frame’s `data[0]` is the function code (`0xFD`, `0xF3`, ...)
  - Up to 7 bytes payload per frame.

## What this package does NOT do (by design)
- No ROS topics/services here.
- No trajectory interpolation.
- No MoveIt integration.

Those belong in `emm_v5_hardware` (ros2_control SystemInterface) and higher-level nodes.
