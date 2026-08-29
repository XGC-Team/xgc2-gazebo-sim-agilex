# SCOUT MINI Gazebo 模型与实物接口对齐记录

本文记录 2026-07-10 完成的 SCOUT MINI Gazebo 模型、ROS1 接口、NMPC
约束和参考轨迹检查、2026-07-31 完成的 1000 Hz/250 Hz 物理步长回归，以及
2026-08-30 基于 Scout 实车开环 bag 完成的响应带对齐。
目的不是宣称仿真已经完成严格系统辨识，而是说明本轮保留了哪些修改、为什么早期
摩擦力和 PID 调参没有解决问题，以及当前结论的证据边界。

## 1. 结论摘要

本轮恢复 NMPC 跟踪的主要原因按影响排序如下：

1. ACADOS 内部预测约束与控制器输出、Gazebo 底盘执行约束对齐。
2. 移除默认圆轨迹中不可执行的 `CIRCLE_ENTRY` 过渡段。
3. 修正 IMU 话题和里程计速度坐标系。
4. 按手册值更新轮子几何，并使轮速 P 增益与 Gazebo 物理步长匹配。
5. 随机目标轨迹增加逐采样点硬限值检查。
6. 将 ODE `fdir1` 固定到 wheel-local 轮轴，消除随轮角旋转的摩擦方向与周期横滑。
7. 用显式命令延迟和一阶执行器响应逼近实车底盘带宽，不把 VRPN 内容滞后塞进物理模型。

最终结果不是通过极端 PID 或继续增大地面摩擦获得的。历史 1 ms/1000 Hz 成功
基线使用 `P=6, I=0, D=0`；2026-07-31 已重建并运行验证的 XGC accurate/process
dev candidate 使用 `4 ms/250 Hz` 和 `P=2, I=0, D=0`。2026-08-30 的当前候选继续
固定该物理步长/P 组合，只修正接触坐标系、命令增益和执行器动态。两套 P 不能脱离
物理步长互换；raw world 未被 launch 自身改写成 250 Hz，验收仍须核对实际 world timing。

## 2. 参数修改清单

### 2.1 几何参数

| 参数 | 原值 | 当前默认值 | 当前依据 | 置信度 |
| --- | ---: | ---: | --- | --- |
| 轴距 | `0.463951 m` | `0.451 m` | SCOUT MINI 用户手册 | 中等，尚未实车测量 |
| 物理轮距 | `0.416503 m` | `0.490 m` | SCOUT MINI 用户手册 | 中等，尚未实车测量 |
| 轮速反解轮距 | `0.416503 m` | 默认跟随 `0.490 m` | 与当前物理轮距默认一致 | 需要实车偏航标定 |
| 轮半径 | `0.08 m` | `0.08 m` | 原模型及碰撞圆柱 | 本轮未修改 |

相关实现：

```text
products/ros1/simulator/gazebo-sim/agilex/scout/urdf/mini.xacro
products/ros1/simulator/gazebo-sim/agilex/scout/launch/mini_description.launch
products/ros1/simulator/gazebo-sim/agilex/scout/launch/spawn_accurate.launch
products/ros1/simulator/gazebo-sim/agilex/scout/src/scout_skid_steer.cpp
```

`wheel_track` 控制 URDF 中左右轮关节中心的位置；`wheel_separation` 控制差速轮速反解。
两者当前默认相等，但保留为两个可覆盖参数，因为滑移转向车辆的有效运动学轮距可能
与几何轮距不同。

### 2.2 轮速控制和命令校准

| 参数 | 原场景默认值 | 当前值 |
| --- | ---: | ---: |
| `wheel_pid_p` | `10.0` | `2.0`（XGC accurate dev candidate） |
| `wheel_pid_i` | `0.0` | `0.0` |
| `wheel_pid_d` | `0.0` | `0.0` |
| `command_gain` | `1.03` | `1.10` |
| `angular_command_gain` | `2.50` | `1.15` |
| `command_delay_s` | `0` | `0.15` |
| `command_time_constant_s` | `0` | `0.15` |

当前增益来自 2026-08-19 Xavier-40 的非 NMPC、非手控脚本开环包：`cmd=0.5 m/s`
时 VRPN 位置回归约 `0.52 m/s`；`cmd=0.5 rad/s` 时 VRPN/IMU/估计器深稳态约
`0.525–0.531 rad/s`。`delay/tau=0.15/0.15 s` 只拟合 `/scout_status` 与 IMU 的宽松
10/50/90% 响应带，不拟合 VRPN pose 的成批传输和内容表观滞后。`P=2` 仍是 4 ms 下
通过稳定性验证的保守值，历史 `P=6` 只属于 1 ms 基线。

### 2.3 摩擦和质量

当前 wheel-local 接触坐标为：

```text
wheel_contact_fdir1 = [0, 0, 1]  # 轮轴，方向 1 为横向
wheel_contact_mu1   = 0.10
wheel_contact_slip1 = 5.0
wheel_contact_mu2   = 1.0        # 方向 2 为纵向
wheel_contact_slip2 = 0.0
```

这里的 `slip1` 是 ODE force-dependent slip（速度/力），不是无量纲轮胎滑移率。它与
`mu1` 只作为一组有效行为参数保留，不能从单个正向开环包唯一辨识。质量和惯量没有在
缺少实车称重、
质心和转动惯量数据的情况下继续修改。手册给出的整车整备质量是 `23 kg`，但现有
模型还包含上装和传感器 link，因此目前不能宣称总质量和惯量已经严格对齐。

## 3. 轴距和轮距的证据边界

本地手册：

```text
products/robotics/agilex/docs/SCOUT_MINI_USER_MANUAL.pdf
```

该手册第 8 页列出：

```text
Axle Track / 轴距:       451 mm
Front/rear track / 轮距: 490 mm
Kerb weight:              23 kg
```

官方在线手册也给出相同参数：

- [AgileX SCOUT MINI 性能参数](https://agilexrobotics.gitbook.io/scout_mini/1-scout-mini-jian-jie-introduction)

但是原来的 `0.463951/0.416503` 也不是本项目随意生成的，它们可在 AgileX 官方
`scout_ros` URDF 中找到：

- [AgileX scout_mini.urdf.xacro](https://github.com/agilexrobotics/scout_ros/blob/master/scout_description/urdf/scout_mini.urdf.xacro)

因此本轮应定义为“按用户手册对齐”，而不是已经确认上游数值为简单笔误。造成差异的
可能原因包括旧车型、旧 CAD、ROS 模型长期未更新或不同产品批次。

对当前实车的最终判定必须测量：

1. 前后轮轴心的纵向距离。
2. 同一车轴左右轮中心面的横向距离。
3. 实车固定角速度命令下的稳态偏航角速度。

如果实测尺寸不同，可通过 launch 参数覆盖，不需要再次修改 URDF：

```text
ugv_wheelbase:=<measured_wheelbase>
ugv_wheel_track:=<measured_track>
ugv_wheel_separation:=<identified_effective_track>
```

## 4. 为什么只调摩擦和 PID 没有成功

### 4.1 NMPC 预测的输入与底盘实际输入不一致

原 ACADOS 求解器内部仍使用较宽约束：

```text
angular velocity: [-2.5, 2.5] rad/s
speed state:      [-0.5, 3.0] m/s
```

但控制器输出和 Gazebo 底盘随后被裁剪到实物限制：

```text
linear velocity:  [-1.5, 1.5] m/s
angular velocity: [-0.5235, 0.5235] rad/s
```

求解器因此预测车辆会执行大输入，而仿真底盘只能执行裁剪后的小输入。误差继续增大后，
NMPC 会持续请求不可达输入并最终失去跟踪。摩擦系数和轮速 PID 都无法修复这个预测模型
与执行器之间的结构性不一致。

当前 NMPC 在 stage `0..N-1` 设置输入界，在 stage `1..N` 设置速度界；stage 0
状态由当前测量单独固定：

```text
speed:              [-1.5, 1.5] m/s
linear acceleration: [-2.0, 2.0] m/s^2
angular velocity:   [-0.5235, 0.5235] rad/s
```

实现位置：

```text
products/ros1/controller/ugv-controller/unicycle_ugv_controller/
  src/nmpc/unicycle_nmpc_solver.cpp
```

### 4.2 默认入圆轨迹本身超限

旧场景使用 `CIRCLE_ENTRY`，车辆起点和圆起点同为 `(3, 0)`。该过渡使用位置多项式连接
静止状态和圆切向速度，在接近零速时产生方向奇异，轨迹验证状态出现
`kFlagYawRateLimit`。这不是轮速跟踪误差，而是参考轨迹本身不可执行。

当前默认圆轨迹改为：

```text
initial position: (3, 0)
initial yaw:      pi / 2
trajectory type: direct circle
radius:           3.0 m
line speed:       1.0 m/s
yaw rate:         1 / 3 = 0.333 rad/s
```

该参考低于 `0.5235 rad/s` 的角速度限制。

### 4.3 D 项在当前 Gazebo 速度环中不稳定

历史 1 ms 基线测试过 `P=6, D=0.05` 和 `P=6, D=0.10`。两组均出现非有限状态，
车辆无法形成有效响应。当前证据只能说明 D 项与该离散速度环、关节速度信号和接触
模型组合不稳定，尚未证明唯一的底层数值原因。因此 1 ms 和 4 ms 配置均保留 `D=0`，
没有用 D 项掩盖其他问题。

### 4.4 单一纯偏航测试不足以决定参数

只看原地旋转可能通过增大摩擦、P 或角速度增益获得更快偏航，但会同时改变线速度、
耦合运动和接触稳定性。本轮改为并行测试：

```text
linear-only
yaw-only
linear-yaw coupled
```

历史 1 ms 测试表明 `P=6` 已能跟踪轮速；4 ms 下实证排除了 `P=6`，并验证了
`P=2`。简化模型的稳定上界约为 `P<4.7`，所以不能声称 P2 是唯一稳定值；当前选择
P2 是为了给接触非线性、调度抖动和无 effort limit 留出裕量。

### 4.5 周期侧滑首先是接触坐标错误

旧 `fdir1=[1,0,0]` 位于 wheel collision local frame，会随 wheel link 自转。方向 1/2
因此相对地面周期旋转，纯偏航时出现与轮角相关的 stick-slip 和大尺度转动中心漂移。
将 `fdir1` 改为与轮轴平行的 local `z` 后，同一 250 Hz/P2 条件下，0.5 rad/s 纯转的
平滑 yaw-rate 标准差从约 `0.077` 降到小于 `0.002 rad/s`，5 s 平移路径从约
`0.25 m` 降到约 `0.036 m`。这是结构修正；摩擦数值只在此后用于宽松响应校准。

## 5. ROS 接口修复

### 5.1 IMU 话题

状态估计器输入由旧话题改为实物接口一致的话题：

```text
/ugv1/imu/data_raw
```

仿真 IMU `100 Hz`，与 AgileX 实车 HI226 上机有效频率对齐；状态估计发布约 `100 Hz`。
`rostopic hz` / rosbag 计数可能看到约 200 Hz，那是一拍多样包，不是融合合同。
历史上曾把仿真 IMU 从 100 Hz 调到 200 Hz；那不是 NMPC 失败根因，直接问题是估计器
订阅了错误话题。2026-08 按现场调试把仿真频率改回 100 Hz。
改驱动或仿真任一端频率时必须成对提交，并在 `serial_imu` 提交里写明对应仿真修改。

噪声只做数量级对齐，不是完整 IMU 辨识。Gazebo 原生 `<imu>` 高斯 `stddev` 取 Xavier-40
开环包 `estimator-imu-check-20260819-220847` 的 rest_0 样本标准差（100 Hz）：

```text
gyro  std  0.0013 / 0.0011 / 0.0007 rad/s
accel std  0.025  / 0.020  / 0.062  m/s^2
```

插件 `gaussianNoise` 必须为 `0`，避免和 SDF 噪声叠两层、还把同一标量加到陀螺和加计。
不要把估计器 yaml 的 `gyro_noise_std` / `accel_noise_std` 抄进仿真。静止偏置随机游走
未辨识，SDF 里不写 `bias_*`。

### 5.2 里程计速度坐标系

Gazebo `ModelStates` 提供世界坐标系中的 twist，而 `nav_msgs/Odometry.child_frame_id`
声明为 `base_link`。旧实现直接复制 twist，造成消息语义错误。

当前 `gazebo_model_odom.cpp` 将线速度和角速度从 world 旋转到 `base_link` 后发布。
这主要修复状态解释、诊断和回归测试，不应与物理模型响应混为一谈。

## 6. 随机轨迹硬限值

随机目标规划器原来只在优化代价中对超限值增加软惩罚，发布前校验还使用空的
`TrajectoryLimits2`。此外，`0.02 s` 的验证采样可能漏掉 `0.01 s` 原始轨迹中的窄峰。

当前处理包括：

1. 对每个原始采样点检查速度、线加速度和角速度。
2. 对插值轨迹再次检查。
3. 任何物理限值 flag 均阻止轨迹发布。
4. 规划器通过延长分段时间重新生成满足限制的轨迹。

相关实现：

```text
products/common/math/include/xgc2_math/trajectory/se2_target_trajectory.hpp
products/ros1/controller/ugv-controller/unicycle_reference_trajectory/
  src/target_replanner_node.cpp
```

## 7. 验证结果

### 7.1 当前 250 Hz 实车开环响应带回归

实车依据是 `estimator-imu-check-20260819-220847`。该包由脚本直接发布 `/cmd_vel`，
没有 NMPC/VRPN 位姿闭环；VRPN twist 不作为真值。当前候选在独立、无宿主端口容器中
使用同一 30 s 命令序列，结果为：

```text
cmd v=0.5       -> pose steady 0.524 m/s, static-to-static 2.619 m
cmd w=+/-0.5    -> pose steady +/-0.519 rad/s, settled +/-2.588 rad
10/50/90% v     -> 0.17 / 0.26 / 0.51 s
10/50/90% w     -> 0.21 / 0.32 / 0.57 s
5 s pure-yaw XY -> net/radius about 0.030 m, path about 0.036 m
```

初始 yaw `0/pi/4/pi/2` 与正负旋转结果一致；pose/twist/joint state 全程有限。实车对应
宽松门为稳态 `v=0.49–0.55 m/s`、`w=0.50–0.55 rad/s`，5 s 位移
`2.45–2.70 m`、转角 `2.50–2.70 rad`，纯转平移不超过 `0.05 m` 且不得周期增长。

### 7.2 历史 1000 Hz 开环纯偏航

以下是历史 1 ms/1000 Hz、`P=6` 候选参数下的近似稳态响应：

| 命令角速度 | 仿真实际角速度 |
| ---: | ---: |
| `0.2 rad/s` | `0.104 rad/s` |
| `0.3 rad/s` | `0.271 rad/s` |
| `0.4 rad/s` | `0.384 rad/s` |
| `0.5 rad/s` | `0.489 rad/s` |

该结果说明低角速度仍有明显衰减，但 `0.2-0.5 rad/s` 不再完全无响应。它是
1000 Hz 历史基线，不应直接外推到当前 250 Hz/`P=2` 配置，也不是最终实车辨识结果。

### 7.3 历史圆轨迹 NMPC

圆轨迹参数：

```text
radius = 3.0 m
speed  = 1.0 m/s
target yaw rate = 0.333 rad/s
```

历史 1 ms/1000 Hz 的 6 秒稳态窗口测得：

```text
mean command linear velocity = 1.015 m/s
mean command yaw rate        = 0.361 rad/s
mean odom linear velocity    = 1.015 m/s
mean odom yaw rate           = 0.327 rad/s
mean radius error            = 0.055 m
reference flags              = 0
```

### 7.4 历史随机目标轨迹

以下是历史 1 ms/1000 Hz、P=6 工作区连续四次随机重规划的结果，不是 P2/250 Hz
重新标定数据。最大角速度分别为：

```text
0.437, 0.516, 0.459, 0.484 rad/s
```

均低于 `0.5235 rad/s`，且轨迹状态 `flags=0`。

### 7.5 自动测试

2026-07-10 历史 1 ms/P=6 工作区测试结果：

```text
164 tests, 0 errors, 0 failures, 0 skipped
```

额外覆盖包括 ACADOS 运行时物理约束、非法约束拒绝和随机目标轨迹物理限值检查。

## 8. 启动方式与频率边界

以下 helper 命令用于复现历史 1 ms/1000 Hz、P=6 跟踪场景；脚本最终进入
`gazebo_sim_examples/scout_ugv1_nmpc_tracking.launch`，不是本轮 XGC P2/250 Hz
Experiment 启动链。

历史默认圆轨迹 GUI：

```bash
scripts/run-ugv-tracking-sim.sh \
  --container xgc2-ros1-ugv-circle \
  --keep-container
```

历史随机目标 GUI：

```bash
scripts/run-ugv-tracking-sim.sh \
  --container xgc2-ros1-ugv-random \
  --random-targets \
  --replan-period 8 \
  --random-range 3 \
  --random-seed 42 \
  --keep-container
```

当前 P2/250 Hz 回归从 XGC Experiment
`b2e780a0-7b8a-4d29-b18d-7506937ee818` 的 `run-all` 工作流启动。Gazebo Server
process 必须收到 world timing：

```text
overrideWorldPhysicsTiming = true
gazeboMaxStepSize          = 0.004
gazeboRealTimeUpdateRate   = 250
```

每台 Scout robot process 必须另行收到：

```text
wheelPidP                  = 2.0
wheelContactFdir1          = "0 0 1"
wheelContactMu1            = 0.10
wheelContactSlip1          = 5.0
wheelContactMu2            = 1.0
wheelContactSlip2          = 0.0
commandGain                = 1.10
angularCommandGain         = 1.15
commandDelaySeconds        = 0.15
commandTimeConstantSeconds = 0.15
```

不能使用上述历史 helper 命令来声称复现了 P2/250 Hz 结果。

## 9. 剩余风险和后续标定

当前仍不能称为完整 sim-to-real 模型，至少还需完成：

1. 实测轴距、几何轮距、轮胎有效半径和整车质量。
2. 补录负向、多个速度档和不同地面/载荷的同型开环数据；当前只有单车单地面正向激励。
3. 检查低角速度 `0.2 rad/s` 附近的静摩擦、死区和轮速控制误差。
4. 在六个当前 Experiment 的真实 ProcessDefinition/Automation pin 链上完成闭环复测；
   历史 `gazebo_sim_examples` helper 带旧参数，不能替代该门。
5. 检查随机轨迹中实际里程计角速度的瞬时过冲。测试曾观察到约 `0.566 rad/s`，虽然
   命令严格限制在 `0.5235 rad/s` 内。
6. 在真实地面类型和载荷条件下分别建立参数集，避免用单组摩擦参数覆盖所有场景。

在这些测量完成前，应把当前 dev candidate 视为“接口一致、约束一致、控制响应可用”
的仿真基线，而不是已经精确复现实车动力学的数字孪生。
