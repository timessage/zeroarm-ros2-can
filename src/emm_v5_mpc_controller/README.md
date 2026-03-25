# emm_v5_mpc_controller

这是一个位于 MoveIt 和现有 `/arm_controller` 之间的 `FollowJointTrajectory` 桥接节点。

它的工作方式：
- 对外提供 `/mpc_controller/follow_joint_trajectory` action server
- 接收 MoveIt 发来的整条关节轨迹
- 每个控制周期读取 `/joint_states`
- 对每个关节做轻量离散搜索 MPC
- 输出单点 `trajectory_msgs/JointTrajectory` 到 `/arm_controller/joint_trajectory`

## 适用场景
- 底层硬件已经有“分段执行 + 到位返回”逻辑
- 希望上层执行更稳、更顺，但不想直接把 MoveIt 的密集轨迹点全量直通到底层

## 重要说明
- 这是一版可编进工程的第一版，不是已经做过完整实机验证的最终版。
- 它更像“上层参考整形 + 轻量 MPC 跟踪桥接”，不是替代底层驱动的全栈控制器。

## 接入方式
1. 编译本包
2. 启动本节点
3. 把 `moveit_config/config/moveit_controllers.yaml` 里的默认控制器改成 `mpc_controller`
4. 让 `mpc_controller` 再把单点轨迹发到你现有的 `/arm_controller/joint_trajectory`

推荐把你当前稳定的 `emm_v5_system.cpp` 保留为：
- 6° 分段
- FD9F 优先
- 软件兜底
- joint1 非 cyclic

这样分层会更稳。
