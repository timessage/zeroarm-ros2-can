# zero_arm_bringup

Bringup package for running the `zero_arm` with MoveIt2 on real Emm_V5.0 CAN hardware.

## What you get
- `launch/real_can_moveit.launch.py`: starts
  - robot_state_publisher
  - ros2_control_node (loads EmmV5 hardware plugin)
  - joint_state_broadcaster + arm_controller spawners
  - move_group
  - RViz
- `config/emm_v5_hw.yaml`: joint parameters template (CAN addr, reduction ratio, limits, ...)
- `config/zero_arm_can.*.xacro`: a URDF wrapper that swaps the ros2_control backend without editing `moveit_config`.

## Run
```bash
# make sure SocketCAN is up
sudo ip link set can0 up type can bitrate 1000000

# launch
ros2 launch zero_arm_bringup real_can_moveit.launch.py config_yaml:=$(ros2 pkg prefix zero_arm_bringup)/share/zero_arm_bringup/config/emm_v5_hw.yaml
```

## Notes
- The hardware plugin starts in open-loop (`open_loop: true`). This is intentional to validate direction/ratio safely.
- Once you add real angle feedback, set `open_loop: false`.
