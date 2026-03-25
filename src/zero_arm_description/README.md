# zero_arm_description

This package contains a ROS 2-friendly URDF generated from the author's model and adapted for ROS 2 usage.

## What you still need to provide
Copy the author's mesh files into:

  zero_arm_description/meshes/

Expected mesh filenames (case-sensitive):
  - base_link.STL
  - ee_link.STL
  - link1.STL
  - link2.STL
  - link3.STL
  - link4.STL
  - link5.STL

## Run
In your ROS 2 Humble workspace:

  cd ~/ws/src
  # copy this folder zero_arm_description here
  cd ~/ws
  colcon build --symlink-install
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch zero_arm_description display.launch.py
