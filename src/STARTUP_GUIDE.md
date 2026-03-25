# Zero Arm startup guide

## 1. Build
```bash
cd ~/your_ws
colcon build --symlink-install --packages-select   emm_v5_driver emm_v5_hardware zero_arm_description moveit_config zero_arm_bringup
source install/setup.bash
```

## 2. Bring up the CAN interface
Check whether the PEAK adapter appears as `can0`:
```bash
ip -br link | grep can
```
If not up yet:
```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 berr-reporting on
ip -details link show can0
```

## 3. Launch the real hardware stack
```bash
source ~/your_ws/install/setup.bash
ros2 launch zero_arm_bringup real_can_moveit.launch.py
```

## 4. Verify controllers
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states --once
```
Expected controllers:
- `joint_state_broadcaster`
- `arm_controller`

## 5. Optional: set software zero for joint1
Only after feedback is valid and the arm is physically at your chosen zero position:
```bash
ros2 service call /emm_v5/set_zero std_srvs/srv/Trigger {}
```
Clear all stored zero offsets:
```bash
ros2 service call /emm_v5/clear_zero std_srvs/srv/Trigger {}
```

## 6. Basic troubleshooting
### No `can0`
```bash
lsusb | grep -i peak
sudo modprobe peak_usb
ip -br link | grep can
```

### Launch starts but arm does not move
```bash
ros2 topic echo /joint_states
ros2 control list_controllers
```
Check that:
- `open_loop: false` and feedback becomes valid
- all motor addresses match YAML (`1..6`)
- driver bitrate is 1 Mbps
- the interface name in YAML is really `can0`

### Feedback never becomes valid
The hardware waits for at least one 0x32 reply from each joint before enabling motion in closed loop mode.
Use:
```bash
candump can0
```
You should see extended CAN frames returning from addresses 1..6.
