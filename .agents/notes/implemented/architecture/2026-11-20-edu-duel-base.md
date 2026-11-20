# Agent Note: EDU 双车底盘联调

Status: implemented

## Problem

两车要用同一坐标系交换位姿。默认雷达只覆盖车头，且导航和双车控制可能同时向同一话题写速度。断联恢复时，旧会话和单向可达都可能掩盖通信失效。

## Decision

`duel_hw.launch.py` 复用框架的纯定位、碰撞监测和收拢底盘 Bridge。两台车的 PC 与各自的树莓派同域，双车跨域依靠双向 UDP 心跳。心跳包包含共享地图和标定文件的内容指纹、进程会话、序列号和对端确认号。每侧使用本机单调时钟，双向连续确认后才把对端位姿交给仲裁。

`duel_base` 唯一输出 `/cmd_vel` 给现有碰撞监测。底盘默认禁用，启用后优先接受新鲜的 `/cmd_vel_duel`，其次接受 `/cmd_vel_nav_out`。定位或双向确认到期会清除速度缓存。场地使用带姿态的矩形参考系与箱体边界，预检包含平移和旋转产生的扫掠，以及双方位置的不确定距离。

## Alternatives considered

- 两个独立 ROS 节点转发心跳和速度选择结果，利于分开部署。合并入口可以让断联与速度选择共用一份接收时间，减少过期状态在 DDS 队列中停留。
- 仅依据对端心跳判断在线，报文更短。增加确认号后，两边都能在单向传输中断时停车。

## Consequences

两台 PC 需要同一份地图、栅格图像和标定文件，且起点要与现场标记一致。示例文件给出场地尺寸和格式。扫掠预检采用带余量的名义包络和低速上限，侧移和后退能覆盖静态图中的障碍，通行区域会比按精确方形车体计算更窄。软件断联门限是 0.5 秒，实车停稳耗时取决于定位、调度与底盘制动，须在场地实测。

在项目支持的 Ubuntu ROS 2 环境中用 `bash ros2/build.bash --base-paths src ../simulation/navigation --packages-up-to alohamini_duel` 构建，再用 `ros2 launch alohamini_duel duel_hw.launch.py` 传入本机编号、地图、标定文件、Host 和对端 PC 地址。Bridge 和底盘仲裁各自保留运动使能。启动先核对停放位置、定位和双向心跳。

共享仿真与记分规则见 [双车共享仿真](2026-11-20-edu-duel-simulation.md)。
