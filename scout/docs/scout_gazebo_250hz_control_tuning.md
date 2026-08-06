# Scout Gazebo 1000 Hz/250 Hz 物理频率与轮速环调试记录

本文记录 2026-07-31 对四辆 Scout Mini 仿真的物理频率、轮端模型、控制增益和
Formation-DMPC 闭环回归。结论只适用于 XGC accurate/process 启动链中的
Gazebo Classic、ODE、`gazebo_ros_control` 和 Scout 模型组合。

本文所称“当前”是当天已重建、已 provision、已运行验证的 dev candidate/effective
runtime，不表示这些工作区修改已经发布成产品版本。

## 1. 当前结论

两组配置的状态如下：

| 配置 | Gazebo 步长 | 更新率 | 轮速 P | 结论 |
| --- | ---: | ---: | ---: | --- |
| 历史成功基线 | `0.001 s` | `1000 Hz` | `6.0` | 已成功运行四车闭环 |
| 当前 dev candidate | `0.004 s` | `250 Hz` | `2.0` | 单车阶跃和四车闭环实证通过 |
| 已排除组合 | `0.004 s` | `250 Hz` | `6.0` | 约 0.2 s 出现首批 status 4，随后退化为 NaN |

当前 XGC accurate/process 运行链的有效值为：

```text
max_step_size             = 0.004
real_time_update_rate     = 250
wheel_contact_mu2         = 0.10
wheel_contact_slip2       = 5.0
wheel_pid_p               = 2.0
wheel_pid_i               = 0.0
wheel_pid_d               = 0.0
```

物理频率和轮速 P 必须作为一组参数评审。禁止只把 1000 Hz 改成 250 Hz 而继续使用
`P=6`。

Scout world 源文件仍保留历史 `0.001/1000`；XGC Gazebo process 在启动时根据
Experiment 工作流参数生成 world override，得到本轮实测的 `0.004/250`。raw
`gazebo_sim_scout` 的 simple 启动链也没有被本轮统一成上述接触参数，不能把该表
外推成所有 Scout 启动方式的默认值。

## 2. 现场模型和控制链

当前轮关节使用：

```text
hardware_interface/VelocityJointInterface
wheel-axis inertia J ≈ 0.0096 kg·m²
joint damping b = 0.1
```

当 `/ugvN/gazebo_ros_control/pid_gains/*` 存在时，
`gazebo_ros_control/DefaultRobotHWSim` 通过速度误差计算轮端力矩，近似为：

```text
tau = P * (omega_command - omega)
```

上层 `velocity_controllers/JointVelocityController` 在这条链路中主要传递速度命令。
Scout 的 continuous wheel joint 当前没有有限的 `effort` limit，因此异常速度误差
不会在 URDF 边界被可靠截断。这使不稳定的离散速度环可以把无界大力矩直接送入 ODE。

轮地接触仍为：

```text
kp = 1e6
kd = 1
mu2 = 0.10
slip2 = 5.0
```

硬接触是可能的放大环节。本轮没有保存逐物理步的轮端力矩记录，因此不能仅凭最终
NaN 确定接触与轮速环的精确先后顺序。

## 3. 1000 Hz 与 250 Hz 的离散稳定性

对轮轴速度作局部线性化，并使用物理步长作显式离散近似，可得到闭环极点：

```text
z ≈ 1 - dt * (P + b) / J
```

代入 `J=0.0096`、`b=0.1`：

| `dt` | `P` | 近似极点 `z` | 判定 |
| ---: | ---: | ---: | --- |
| `0.001` | `6.0` | `0.365` | 稳定 |
| `0.004` | `6.0` | `-1.54` | 模大于 1，交替发散 |
| `0.004` | `2.0` | `0.125` | 稳定且留有裕量 |

该近似下，4 ms 的 P 稳定上界约为：

```text
P < 2J/dt - b ≈ 4.7
```

`P=3` 或 `P=4` 在简化模型中可能仍稳定，但接触非线性、调度抖动和无 effort limit
都会压缩裕量。因此当前选择经过闭环实证的保守值 `P=2`，不为追求更快轮速响应逼近
理论边界。

## 4. P=6/250 Hz 失败时间线与证据边界

故障实例在 Gazebo 仿真时间 `1223.188 s` 同时进入
`OPTIMIZING_ROLLING`。首批 ACADOS status 4 出现时间为：

| 车辆 | 相对 rolling 开始时间 |
| --- | ---: |
| `ugv2` | `+168 ms` |
| `ugv4` | `+184 ms` |
| `ugv3` | `+188 ms` |
| `ugv1` | `+204 ms` |

这相当于 42–51 个 4 ms 物理步，与极点 `-1.54` 在几十步内交替放大的预测一致。
静止阶段速度误差为零，因此所有进程和仪表可以先显示正常；第一次收到有限
`cmd_vel` 后才触发发散。

这些观测与以下工作假设一致：

1. 隔离轮轴的 4 ms/P=6 显式离散近似预测交替发散。
2. continuous wheel joint 没有 effort limit，错误可以持续放大并污染 ODE 状态。
3. VRPN 将异常底盘状态送入每车的 `unicycle_ugv_controller` tracking NMPC。
4. tracking NMPC 的后继阶段动力学、加速度和速度边界可能因此不可行或数值失败，
   ACADOS 返回 status 4。
5. Gazebo model twist 最终变为 NaN，位姿投影随后退化。

这里的 status 4 来自每车 tracking NMPC，不是 Formation-DMPC 求解器。tracking
NMPC 的 stage 0 只固定当前状态，速度界施加在后继阶段，因此不能把失败简写成
“stage-0 状态等式与速度界直接冲突”。

日志中的 future-timestamp 拒绝发生在 `1280.852 s`，远晚于首个 status 4，因此它是
故障后的时间/状态污染，不是起因。算法给出的 `cmd_vel` 也始终在既有
`±0.2 m/s`、`±0.5235 rad/s` 有限范围内，不能解释无界物理状态。

本轮没有保存逐步轮速、轮端力矩和第一个 ODE 非有限值的同一时间序列。因此离散分析、
失败时间和 P2/P6 A/B 结果对轮速环根因提供了强证据，但不构成唯一根因的形式证明。

## 5. P=2/250 Hz 实证

### 5.1 冷启动单车阶跃

保持世界、ODE、接触参数和算法不变，仅使用明确的
`mu2=0.10/slip2=5.0/P=2.0`。对 `ugv1` 施加 1 秒：

```text
linear.x = 0.20 m/s
angular.z = 0.10 rad/s
```

结果：

```text
horizontal displacement = 0.1949 m
maximum wheel speed      = 2.814 rad/s
final chassis z          = 0.1800 m
pose/twist/joint state   = finite
```

位移取 `/gazebo/model_states` 阶跃前后水平位置差；最大轮速取同一窗口四个 wheel
joint 的绝对最大值。

### 5.2 四车 Formation-DMPC

四车实验使用 `0.004/250` 和同一历史正确的 Formation-DMPC 工作流连续观察 45 秒。
可复现实验身份为：

```text
Experiment  = b2e780a0-7b8a-4d29-b18d-7506937ee818
XGC Session = cf846bff-2c0f-4dd9-988d-afcafc1d781e
ROS UUID    = c99a25de-8c4f-11f1-a59d-e750384f233e
Scout ProcessDefinition = scout-gazebo-robot 1.11.0
```

位移是每台车相对 45 秒窗口首个有限 `/gazebo/model_states` 样本的三维最大位移；
最大线速度取同一窗口 model twist 的线速度模：

| 车辆 | 最大位移 | 最大线速度 |
| --- | ---: | ---: |
| `ugv1` | `13.344 m` | `0.455 m/s` |
| `ugv2` | `12.572 m` | `0.450 m/s` |
| `ugv3` | `10.113 m` | `0.450 m/s` |
| `ugv4` | `10.848 m` | `0.442 m/s` |

同时满足：

```text
planning_state                = 1
sync participants             = 4
sync missing responses        = 0
participant status            = all OK
Gazebo pose/twist             = finite
wheel joint state             = finite
/formation/pairwise_distances = continuous
/xgc/formation_scene          = continuous
solver diagnostics            = all four vehicles publishing
tracking NMPC status 4        = 0
Robot projection               = 4 live/online/operationalReady
world camera snapshot/WebRTC   = 3840x2160 H264 at about 30 Hz
camera enhancement markers     = ugv1..ugv4 ready
Lichtblick scene               = continuous
```

求解器诊断检查必须最后 source Formation-DMPC 工作区；使用 Paper 工作区的旧同名
消息会产生 MD5 mismatch，该失败探针不算有效观测。

P2/P6 A/B 结果强烈支持“P=6/250 Hz 触发物理状态破坏”的解释。成功窗口中 Robot
projection、VRPN、世界相机和 Lichtblick 同时正常，另行排除了该成功运行中的持续
世界坐标或模型映射故障；它仍不替代逐轮力矩记录所需的唯一因果证明。

## 6. 源码和持久化真值

当前值必须在以下位置保持一致：

```text
products/ros1/simulator/gazebo-sim/agilex/scout/
  launch/accurate.launch
  launch/multi_accurate.launch
  launch/spawn_accurate.launch
  config/scout_mini_ros_control.yaml
  test/test_stable_defaults.py

xgc2/process-catalog/current/ros1/simulator/gazebo-sim/scout/
  xgc2-gazebo-sim-scout.json
xgc2/process-catalog/current/platform/
  gazebo-server.json

xgc2/scripts/
  provision-robot-simulation-automation.sh
  provision-ros-basic-services-automation.sh
  provision-default-experiment-workflows.sh
```

ProcessDefinition、Scout 子工作流、单车工作流和 Experiment Robot Runtime 必须形成
一致的不可变 pin 链；各层 pin 的对象和 digest 类型不同，不能把它们写成同一个
digest。旧参数三元组的运行时映射已经删除；当前 Automation 明确持久化
`0.10/5.0/2.0`，不依赖兼容分支。

`gazebo-server.json` 负责实际 world timing override；两个 ROS/Experiment provisioning
脚本负责把 `gazeboMaxStepSize` 和 `gazeboRealTimeUpdateRate` 冻结到工作流。只修改
Scout launch 或 world 源文件都不足以改变 XGC Experiment 的有效频率。

## 7. 回归与回退判据

250 Hz 只有在以下条件全部满足时才能作为本 dev candidate 的有效基线：

1. 冷启动后所有 Scout pose、twist 和 wheel joint state 始终为有限值。
2. 小阶跃下车辆产生合理位移，轮速不出现交替指数增长。
3. Formation-DMPC 进入 rolling 后至少观察一个完整窗口，所有实体车辆都移动。
4. 同步器无持续 missing/failed participant。
5. Robot projection 持续为 `live/online/operationalReady`。
6. 世界相机、增强图像和 Lichtblick 场景持续更新。

若任何一项失败，应先恢复已经实证成功的组合：

```text
max_step_size         = 0.001
real_time_update_rate = 1000
wheel_pid_p           = 6.0
```

禁止组合使用 `0.004/250 + P=6`。恢复 1000 Hz 是可靠性回退，不代表 P=2/250 的
离散分析失效；它只说明当前完整系统还有未覆盖的 250 Hz 耦合问题。

## 8. CPU 开销观测

以下是同一开发机上的运行时快照，不是受控 benchmark：

| 场景 | `gzserver` CPU |
| --- | ---: |
| 历史四车 `0.001/1000 + P=6` | 约 `180%`，即 1.8 个逻辑核 |
| 四车 `0.004/250 + P=2`，没有活跃 4K 消费者 | 约 `20%` |
| 四车 `0.004/250 + P=2`，活跃 4K 世界相机 | 约 `60%` |
| 六 PX4、250 Hz、4K 相机和六路 DMPC 同时运行 | `109%–134%` |

这些快照提示不同运行链路都可能贡献开销，但上表只测量了 `gzserver`，而且各场景同时
改变了多个变量，不能由此定量分离物理、相机和算法的占比：

- 1000 Hz 会增加 ODE、接触和 ros_control 的每秒更新次数。
- 4K 世界相机有消费者时才持续承担渲染/编码成本；Media Edge 本身接近空闲，主要
  成本在 Gazebo render/encode 和增强端。
- 本轮前端 session hook 重复刷新曾让 Core 占用约 5.5 个核，修复后完整六机运行中
  Core 瞬时约 `20%–38%`。该问题与 Gazebo 物理频率无关。

这些数值只用于判断开销来源。更换 GPU、相机 profile、是否有 WebRTC 消费者、世界
碰撞复杂度和算法数量都会改变绝对值。

## 9. 剩余安全边界

后续应基于 Scout 实际电机、减速器和轮端能力，为四个 wheel joint 增加明确的
`effort` 和 `velocity` limit。数值必须来自硬件规格或实测，不能为了让仿真“不炸”
随意填写。即使 P 参数错误，有限执行器边界也不应允许单个控制误差摧毁整个 ODE
世界。
