# Agent Note: EDU 双车底盘联调

Status: implemented

## Problem

两车要用同一坐标系交换位姿。默认雷达只覆盖车头，且导航和双车控制可能同时向同一话题写速度。断联恢复时，旧会话和单向可达都可能掩盖通信失效。

## Decision

`duel_hw.launch.py` 复用框架的纯定位、碰撞监测和收拢底盘 Bridge。两台车的 PC 与各自的树莓派同域，双车跨域依靠双向 UDP 心跳。心跳包包含共享地图和标定文件的内容指纹、进程会话、序列号和对端确认号。每侧使用本机单调时钟，双向连续确认后才把对端位姿交给仲裁。

`duel_base` 唯一输出 `/cmd_vel` 给现有碰撞监测。底盘默认禁用，启用后优先接受新鲜的 `/cmd_vel_duel`，其次接受 `/cmd_vel_nav_out`。定位或双向确认到期会清除速度缓存。运动许可要求对端位姿接收和最近被确认报文的本机发送时间均在 0.30 秒内。旧会话须静默 0.5 秒才允许接收重启后的会话。场地使用带姿态的矩形参考系与箱体边界，预检包含平移和旋转产生的扫掠，以及双方位置的不确定距离。

## Alternatives considered

- 两个独立 ROS 节点转发心跳和速度选择结果，利于分开部署。合并入口可以让断联与速度选择共用一份接收时间，减少过期状态在 DDS 队列中停留。
- 仅依据对端心跳判断在线，报文更短。增加确认号后，两边都能在单向传输中断时停车。
- 以收到确认的时刻续期可以适应链路延迟。按原报文的本机发送时刻计算许可有效期，能给碰撞监测和底盘响应留出时间，并限制迟到确认延长运动许可。

## Consequences

两台 PC 需要同一份地图、栅格图像和标定文件，且起点要与现场标记一致。示例文件给出场地尺寸和格式。扫掠预检采用带余量的名义包络和低速上限，侧移和后退能覆盖静态图中的障碍，通行区域会比按精确方形车体计算更窄。0.5 秒是从故障注入到停稳的目标，实车耗时取决于调度、通信和底盘制动，须在场地实测。

在项目支持的 Ubuntu ROS 2 环境中用 `bash ros2/build.bash --base-paths src ../simulation/navigation --packages-up-to alohamini_duel` 构建，再用 `ros2 launch alohamini_duel duel_hw.launch.py` 传入本机编号、地图、标定文件、Host 和对端 PC 地址。Bridge 和底盘仲裁各自保留运动使能。启动先核对停放位置、定位和双向心跳。

共享仿真与记分规则见 [双车共享仿真](2026-11-20-edu-duel-simulation.md)。

## 场地标定

按包内 `config/measurements_example.yaml` 记录测点。墙角从选定的场地原点开始逆时针填写，第一段确定场地横轴。箱角按逆时针填写。两车起点为共用地图中的 `[x, y, yaw]`，单位为米和弧度。

```bash
ros2 run alohamini_duel duel_calibrate --measurements 测点.yaml --map map.yaml --output 场地标定.yaml
```

命令按矩形校验墙角，用测点确定的内侧矩形作为场地边界，用场地坐标下的外接矩形包住每个箱体。默认接受 0.03 米拟合误差，可用 `--tolerance` 设置。检查墙边、箱体和双车起点间的名义净空，通过后才写入新文件并打印场地指纹。地图 YAML 和图像共同参与指纹计算，管理地图还会校验版本清单。

## 实机只读预检

两车机械臂收拢，分别停在标定点。使用同一份地图和场地标定，在两车的 `duel_hw.launch.py` 加上 `referee_status:=true`。预检机须能通过 DDS 读取两车各自的域。

```bash
ros2 run alohamini_duel duel_check --arena 场地标定.yaml --map map.yaml --domains 5 6
```

预检读取地图指纹、双向心跳、雷达、IMU、轮速、两路速度、碰撞监测生命周期和 Bridge 参数与诊断。当数据过期、轮速非零、仲裁或 Bridge 已使能、出现多路速度发布者时返回失败。碰撞监测在静止时可停止重复发布零速，此时利用新鲜轮速核对静止反馈。报告包含每车距标定起点的偏差和名义净空，供现场核对实际停放位置。预检不调用使能服务，实际制动与定位精度需要现场测量。

Humble 双域端到端测试验证正常静止时预检通过，并核实额外的速度发布者会令预检失败。相关回归共 42 项通过。

## 断联停车记录

```bash
ros2 run alohamini_duel duel_stop_watch --arena 场地标定.yaml --map map.yaml --domains 5 6 --duration 30
```

记录器复用双车状态话题和现场标定配置，同时读取两台车的仲裁速度和轮速。现场试验判读期间需保留记录机到两车的 DDS 遥测连接。出现联机下降沿时，若本车定位仍有效，开始记录零速及五帧连续静止反馈。报告分别给出两车的观测时间，标明断联前是否测到运动。任一车缺少有效下降沿、零速或停稳证据，或两车均未观察到运动时，命令以失败退出。

耗时来自记录机接收 ROS 消息时的单调时钟。默认模式从记录到联机下降沿开始计时。现场记录机与两车需要对齐 ROS 时钟，以核对轮速消息的新鲜度。

需要按实际故障触发时刻判定时，记录器可使用同一台 Linux 记录机的事件标记。

```bash
ros2 run alohamini_duel duel_stop_watch --arena 场地标定.yaml --map map.yaml --domains 5 6 --duration 30 --fault-event ./本次故障.json > ./本次停车报告.json
```

等待记录器输出就绪提示后，由独立故障触发器在本机实际发生故障时取得 `time.monotonic_ns()`，并用 `FaultEvent(triggered_ns, source).write(path, recorder_boot_id())` 原子写入约定的新路径。这两个接口位于 `alohamini_duel.fault_event`。事件须与记录器使用同一单调时钟，并发生在两车检测断联之前。

标记模式分别报告从故障到检测断联、仲裁零速、第一帧静止轮速及五帧确认的耗时。故障前已静止的车会单独标出。缺少有效标记、停车证据或故障前运动证据时返回失败。最迟轮速停稳超过 0.5 秒时也返回失败。轮速结果需要现场的车体位移和滑行证据共同判读。记录器只订阅遥测，不发送运动或故障指令。

Humble 双域端到端测试通过实际 UDP 故障注入取得双车停稳记录，相关回归共 45 项通过。

## 固定航点导航

双车启动时可传 `use_nav2:=true`。固定航点任务使用框架的规划器、控制器、速度平滑器和 `NavigateThroughPoses`。平滑器输出重映射到 `/cmd_vel_nav_out`，由双车仲裁器统一输出 `/cmd_vel`，再经过碰撞监测。纯定位启动仍使用默认配置。框架独立的单车导航入口仍提供 `FollowWaypoints`。

`duel_route` 读取按顺序排列的 `[x, y, yaw]` 列表，坐标属于共用 `map`，朝向为弧度。每个目标都检查场地与箱体净空。软件裁判状态和个人路线可以并行运行，路线目标来自配置文件。

```bash
ros2 run alohamini_duel duel_route --robot-id sword_a --arena 场地标定.yaml --map map.yaml --route 航点.yaml
```

在相应车辆的 ROS 域运行。实机启动时启用 `referee_status:=true`，按现场流程操作 Bridge 和速度仲裁的使能。命令等待定位、双向心跳、已使能仲裁及 Nav2 就绪，开始后在联机状态失效或中断时撤销自己的目标。仿真运行时加 `--simulation`，示例航点位于包的 `config/route_example.yaml`。

Humble 隔离仿真验收到达固定航点、断联取消及重连后保持静止。完整回归共 48 项通过。
