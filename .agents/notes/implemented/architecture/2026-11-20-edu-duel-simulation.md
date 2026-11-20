# Agent Note: EDU 双车共享仿真

Status: implemented

## Problem

单车仿真只能扫描静态场地。双车定位、断联停车和电子裁判需要彼此独立的模拟物理状态、通信状态和回合状态。

## Decision

两辆车复用框架的平面运动、传感器和时钟消息。一个协调进程同步步进两个底盘，并通过独立的 ROS 上下文发布各自的扫描和反馈。雷达对场地和另一辆车的真值包络求交，给同一时刻的两车运动提案做相对扫掠检查。示例栅格地图与场地配置随包安装。

每辆仿真车的定位、碰撞监测和速度仲裁运行在独立 ROS 域。底盘节点通过回环 UDP 同步位姿，另用域内 ROS 状态消息把定位与双向联机情况交给独立的 `duel_referee` 进程。裁判按回合编号接收两侧的方向选择，在两车联机时统一判分。断联会注销该回合的待判输入，双方恢复后进入新的回合。底盘节点的链路开关用于切断 UDP 并保留仿真时钟、雷达和裁判进程。

## Alternatives considered

- 运行两份现成的单车仿真，可以沿用现有启动脚本。统一推进两个底盘能让它们共用时钟并检测相对扫掠。
- 把心跳收到的位姿注入扫描，接入代码较少。独立的真值世界可以在断开心跳通路后继续生成正确的回波。

## Consequences

场地回波和双车碰撞可以进行可重复的故障注入。当前仿真是二维运动学模型，场地和机器人高度、轮胎打滑以及实车停稳距离仍由现场验证承担。定位使用支持光束跳过的 AMCL 概率似然场来处理临时回波。计分表现为双域共享的比分和回合结果，方向选择是软件输入。

构建方式见 [双车底盘联调](2026-11-20-edu-duel-base.md)。加载 ROS 工作区后运行 `ros2 launch alohamini_duel duel_sim.launch.py`。默认使用域 45 和 46，在对应域查看 `/duel/scoreboard`，通过 `/duel/choice` 发送形如 `1:left` 的回合编号和方向，方向为 `left`、`center` 或 `right`。仿真初始车距用于检验联机，移动由仿真底盘和仲裁的使能服务分别控制。

使用 Cyclone DDS 时，仿真启动入口提供较大的回环参与者索引范围。另开终端接入仿真时使用相同的配置。

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=45
export CYCLONEDDS_URI="$(cat "$(ros2 pkg prefix alohamini_duel)/share/alohamini_duel/config/cyclonedds_sim.xml")"
ros2 topic echo /duel/scoreboard
```

Humble 下退出前依次在两个仿真域执行 `ros2 run alohamini_navigation map_session stop --runtime-mode simulation`，让碰撞监测先结束生命周期再关闭启动进程。

在支持的 ROS 2 环境运行 `python -m pytest ros2/src/alohamini_duel/test`，其中实际管线测试启动双域仿真，检查运动、断联、停车、回波、恢复及有序停机。

双车实机底盘联调可分别加 `referee_status:=true`，令独立裁判读取定位和联机状态。裁判机需能通过 DDS 发现两个域。在该机器上加载同一份标定与地图后运行 `ros2 run alohamini_duel duel_referee --arena 场地标定.yaml --map map.yaml --domains 5 6`。这个进程的输出为软件比分。

## Testing

Ubuntu 22.04 ARM64 容器使用 ROS 2 Humble、Cyclone DDS 和仓库的 TF2 修补版。构建 8 个相关包，41 项测试通过，双域仿真按生命周期顺序干净退出。真机验收继续使用现场地图、标定和停稳反馈。
