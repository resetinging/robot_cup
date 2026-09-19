# 项目架构（STM32F407 智能搬运车）

**一句话**：裸机固件。TIM2 中断跑实时控制（电机/舵机/IMU/循迹），主循环跑任务状态机（扫码 → 任务1 → 任务2 → 返程），视觉/识别全部交给 K230 上位机。

```
MCU: STM32F407VGT6 @168MHz   CubeMX 6.9.2 + Keil MDK (ARMCC V5)
上位机: K230 视觉板 (USART6)    调试: 串口助手 (USART2)
```

## 分层

```
main.c            总状态机 + TIM2 中断（唯一调度者）
├─ Task/          业务层：搬运动作、方案查表、上位机协议、PID 控制环
├─ Hardware/      设备层：麦轮底盘、灰度循迹、舵机、步进电机封装
└─ Software/      驱动/算法层：Emm_V5 电机协议、ICM42688、PID、编解码 I2C
```

## 外设映射

| 外设 | 连接 | 职责 |
|---|---|---|
| USART1 | 4× 张大头 Y42 闭环步进 | 麦轮速度/位置指令（只发不收，DMA 队列 + 多机同步帧） |
| USART2 | PC | 调试打印（`uart_printf` / `printf`） |
| USART3 | LX-16A 总线舵机 | 转盘 5 槽位 + 锁位 |
| USART6 | K230 上位机 | 帧协议收发（中断收 / 队列发） |
| TIM2 @1kHz | — | 控制心跳 |
| TIM9 CH2 | PWM 舵机 | 抬举物块 |
| I2C1 / I2C2 / 软 I2C | 8 路灰度 / ICM42688 / 颜色传感器 | 循迹、姿态、备用颜色识别 |

## 运行时模型

```
TIM2 @1kHz 中断                        主循环 while(1)
├ Chassis_Process / Servo_Process      └ overall_task_state
├ 每 5ms  IMU 姿态解算                    1 扫二维码
├ 每 18ms 灰度 PID → 差速循迹            2 任务1：循迹收 5 色块 → plan1 → 5 槽位
└ 每 1s  调试打印                        3 任务2：循迹收 3 奖杯 → 固定放置 B→A→C
                                        4 返程回 HOME
```
- 阻塞式动作（定距移动、视觉对准）都在主循环串行执行；移动靠电机内部闭环，按“行程/速度”估算时间延时等待。
- 中断内只做：电机/舵机队列泵、IMU 采样、循迹 PID 与收料节奏、丢线停车。

## 核心模块

| 模块 | 说明 |
|---|---|
| `Task/task.c` | 规则附录查表：QR 1~16（颜色序）/ 1~6（奖杯序）→ 抓取槽位序列；场地标定距离数组 |
| `Task/control.c` | 静态角度环（原地转）、动态角度环（保航向）、十字/圆心两段式对准、yaw 标定 |
| `Task/usart_sent.c` | USART6 帧状态机：请求 `AA 01/02/03/05 0A`，回传十字/圆心/颜色/二维码/完毕/报错 |
| `Hardware/Chassis.c` | 麦轮正逆解算、定位置移动、差速循迹、对圆心速度闭环 |
| `Hardware/gray.c` | 8 路灰度加权求线位 → PID → 左右轮差速 |
| `Software/Emm_V5.c` | 张大头电机协议 + DMA 发送队列（可中断调用、队列满保护、紧急清队列） |
| `Software/Navigation/kalman.c` | ICM42688 姿态解算（AHRS）+ yaw 偏置接口 |

## 通信协议（USART6，`AA <类型> <数据> 0A`）

| 方向 | 帧 | 含义 |
|---|---|---|
| 车→机 | `AA 01/02/03/05 0A` | 请求 十字 / 圆心 / 颜色 / 二维码 |
| 机→车 | `AA 0C x y yaw 0A` | 十字偏差（3×float 小端） |
| 机→车 | `AA 08 x y 0A` | 圆心偏差 |
| 机→车 | `AA 01 <色> 0A` / `AA 04 <号> 0A` | 颜色（0~4）/ 二维码编号 |
| 机→车 | `AA 00 0A` / `AA 0F 0A` | 完毕 / 报错（立即停车） |
| 双向 | `GO`、`CROSS:x,y,yaw`、`ALIGN_DONE`、`CIRCLE_DONE` | ASCII 兼容通道 |

## 配置开关（编译期）

- `Servo_motor.h`：`start_from_home / start_from_task1 / start_from_task2 / debug_in_round / debug_in_color` —— 选择上电起点。
- `Chassis.h`：`TWO_IS_ROUND`（场地形态）、`CIRCLE_ALIGN_SPEED_MODE` + `CIRCLE_ALIGN_USE_ANGLE_HOLD`（对圆心控制方式）。

## 待处理

1. `Task/task.c:33` `-0.17if` 非法字面量（应为 `-0.17f`），当前源码未编译验证。
2. `while(cross_flag)`、`while(home_flag==0){}` 无超时，视觉失联会死等。
3. 闲置代码：`color_reg.c`、`Software/Navigation`（里程计/Flash 路径）、UART5、`MPU9250.*`、`Chassis_Diag_Run`。
