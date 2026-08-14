# CUArm 数据可视化

## 运行

```bash
cd cuarm_data_visualizer
python3 -m http.server 8765
```

浏览器打开 http://127.0.0.1:8765 。Plotly 使用同目录下的 `plotly-2.35.2.min.js`（不再访问 CDN）。字段说明请点页面上的「字段说明」，或打开 **[fields.html](fields.html)**。不要用浏览器直接打开 `README.md`：`python3 -m http.server` 会按纯文本发送且不声明编码，中文会乱码。

## 推荐用法

1. 打开 `log_YYYYMMDD_HHMMSS.txt`
2. 再打开同名的 `log_YYYYMMDD_HHMMSS.structure.json`（「结构 JSON」按钮）——列名与维度会自动对齐
3. 若没有 sidecar：在左侧填关节/夹爪数（例如 `6` 关节、夹爪空 → **155** 列），靠 `log_schema.json` 解析

最后一行若被进程退出截断，会自动跳过，状态栏会提示跳过行数。

## 时间轴

默认用 `Meta.running_cost`（控制周期耗时，单位 **微秒**）做累积：

- `t[0] = 0`
- `t[r] = t[r-1] + running_cost[r-1] × 10⁻⁶`（秒）

这样周期抖动、过载拖长都会反映在横轴上。没有有效 `running_cost` 时，才退回左侧填写的 `sample_time`（固定步长）；再没有则用行号。

## 字段从哪来？

日志列布局以 `cuarm_rt_control/app/log_utils.ipp` 的 `writeToLogFile` 为准。

| 文件 | 作用 |
|------|------|
| `log_schema.json` | 字段布局定义（改 C++ 写日志后同步改这个） |
| `log_*.structure.json` | 运行时由控制程序自动生成，含关节数 + 完整列名 |

改 `writeToLogFile` 时同步三处：

1. `log_utils.ipp` 里的 `writeToLogFile`（写数据）
2. 同文件的 `appendLogColumnNames`（写 sidecar 列名，顺序必须一致）
3. `cuarm_data_visualizer/log_schema.json`（网页离线/旧日志解析）

## 字段含义

**单位**：非力矩模式时，角度相关量写成 **度**；力矩模式保持原单位。`*_cost` 为微秒。`tool_pose` 的 xyz 是米、姿态是四元数。笛卡尔 `rx/ry/rz` 为旋转向量（非力矩时转成度）。

### Meta（整机）

| 字段 | 含义 |
|------|------|
| `running_cost` | 整圈控制周期墙钟时间（含 sleep），微秒；也用于时间轴 |
| `robotics_total_cost` | 规划 / 运动学这一段耗时 |
| `hardware_communication_cost` | 从周期开始到硬件读写结束 |
| `communication_cost` | 到发完状态、入日志队列为止（sleep 前） |
| `motor_communication_cost` | 电机总线读写耗时 |
| `gripper_communication_cost` | 夹爪通信耗时 |
| `button_communication_cost` | 按钮通信耗时 |
| `actuator_mode` | 下发给驱动器的模式：`0` 位置 / `1` 速度 / `2` 力矩 |
| `control_strategy` | 规划策略：`0` 关节 / `1` 笛卡尔 / `2` 直线 / `3` 零空间 / `4` 重力补偿 / `5` 示教 / `6` 回放 |
| `branch_state` | 本周期主循环分支（故障、恢复、新指令、无客户端等） |
| `system_state` | `1` 启动 / `2` 空闲 / `3` 运动 / `4` 到位收敛 / `5` 错误 / `6` 恢复 / `7` 关机 / `8` 离线 |
| `plan_result` | `0` 成功 / `1` 位姿不可达 / `2` 直线路径失败 |
| `system_diagnostic_flags` | 系统诊断位掩码（限位、饱和、跟踪失败、碰撞等） |
| `button_state` | 物理按钮状态 |
| `simulation` | `1` 不把命令真正发给硬件 |
| `target_type` | 上层目标类型：位置 / 速度 / 力矩（决定 PID 怎么算） |

### 每关节：指令链路

```
用户目标
  → target_before_interpolation   插值前的关节目标
  → sliced_tar                    插值后、本周期要跟的目标
  → pid_cmd                       按「目标类型 × 执行器模式」算出的命令
  → guard_pos_cmd                 关节限位保护后
  → sat_cmd                       软限位饱和后
  → jump_cmd                      限制单步跳变后
  → motor_target                  最终下发给电机的命令
```

| 字段 | 含义 |
|------|------|
| `motor_target` | 本周期发给硬件的最终命令 |
| `joint_position / velocity / acceleration` | 规划器侧关节状态（`srv_state`） |
| `joint_torque` | 规划器侧关节力矩 |
| `motor_position / velocity / torque` | 硬件读回来的关节量（`robot_state`） |
| `pid_cmd` | 位置跟位置直接透传；位置跟速度是 `Kp*(tar-x)`；力矩环再加阻尼等 |
| `guard_pos_cmd` | 速度/力矩模式下，防止下一拍冲出位置限位 |
| `sat_cmd` | 按软限位把命令夹到合法范围 |
| `jump_cmd` | 限制相邻周期命令跳变 |
| `joint_gravity` | 逆动力学算出的重力矩 |
| `friction` | 摩擦补偿（库仑 + 粘滞，用滤波速度） |
| `gravity_kd_effect` | 重力环阻尼项（当前代码里常为 0） |
| `filtered_joint_vel` | 低通 + 死区后的关节速度，给摩擦模型用 |
| `arm_diagnostic_flags` | 该关节诊断位（硬限位、跟踪失败等） |

对比跟踪：`sliced_tar` vs `joint_position`。对比下发是否被削：`pid_cmd` → `sat_cmd` → `jump_cmd` → `motor_target`。

### 末端 / 任务空间（每臂固定列）

| 字段 | 含义 |
|------|------|
| `tool_pose_{x,y,z,qw,qx,qy,qz}` | 工具位姿：米 + 四元数 |
| `task_target_before_interpolation_*` | 笛卡尔目标，插值前 |
| `task_sliced_tar_*` | 笛卡尔插值后的本周期目标 |
| `task_pose_*` | 当前任务空间位姿（旋转向量） |

关节空间看 `sliced_tar`，笛卡尔/直线看 `task_sliced_tar`。

### 夹爪（有夹爪才写）

| 字段 | 含义 |
|------|------|
| `gripper_target` | 夹爪目标 |
| `gripper_position` | 夹爪实际位置 |
| `gripper_diagnostic_flags` | 夹爪诊断位 |
