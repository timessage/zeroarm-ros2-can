# emm_v5_hardware

`ros2_control` **SystemInterface** plugin for controlling Emm_V5.0 closed-loop stepper drivers over **SocketCAN**.

## Design goals
- Works with MoveIt2 by exposing standard `position` command interfaces.
- Sends driver commands using `emm_v5_driver` (protocol + SocketCAN splitting).
- Safe-by-default skeleton: `open_loop=true` mirrors commanded position into state so MoveIt/RViz can run before you have read-back.

## Quick config
In your `ros2_control` Xacro:

```xml
<ros2_control name="ZeroArm" type="system">
  <hardware>
    <plugin>emm_v5_hardware/EmmV5System</plugin>
    <param name="config_yaml">$(find emm_v5_hardware)/config/emm_v5_hw.yaml</param>
    <param name="can_iface">can0</param> <!-- optional: overrides YAML -->
    <param name="use_sync_start">true</param>
    <param name="open_loop">true</param>
  </hardware>
  <!-- joints... -->
</ros2_control>
```

## YAML structure
See `config/emm_v5_hw.yaml`. The plugin accepts either:
1) `config_yaml` (preferred) — one file contains all joint params.
2) per-joint params in the ros2_control URDF — fallback if yaml-cpp isn’t available.

## What’s implemented
- `write()`: converts target joint position → delta(deg) → pulses, uses FD position mode, optional sync-start broadcast.
- `read()`: open-loop mirror (TODO: add real angle read-back).

## Next steps you’ll add later
- Implement angle read-back (driver read-angle command) and update `read()`.
- Add safety: soft limits, STOP, watchdog, CAN timeout strategy.
