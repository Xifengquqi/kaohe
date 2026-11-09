# HELIOS 2026 电控考核

三个任务的代码、文档与测试数据统一放在这一个仓库里。

| 任务 | 主要内容 | 目录 | 状态 |
|---|---|---|---|
| 任务 01 Frameworks | BSP / Module / App 分层框架，FreeRTOS，C++17 | `firmware/` | 🚧 |
| 任务 02 Gimbal Master | yaw-pitch 二轴自稳云台 + 上位机 | `firmware/` + `host/` | 🚧 |
| 任务 03A 倒立摆仿真 | Webots 一阶倒立摆建模与控制对比 | `sim/` | ⬜ |

## 目录说明

```
helios-2026/
├── firmware/        单片机代码（任务01 框架 + 任务02 云台应用）
│   ├── Core/        CubeMX 生成的 HAL 层
│   ├── bsp/         外设抽象：UART / CAN / I2C（任务01）
│   ├── module/      传感器、舵机、蓝牙等模块
│   ├── app/         应用层：云台控制（任务02）
│   └── docs/        框架架构图、接口说明、移植步骤
├── host/            任务02 上位机（Python：波形显示 + 在线调参）
├── sim/             任务03A Webots world + 控制器
├── docs/            通用文档：进度、AI 会话记录、测试数据汇总
└── README.md
```

## 开发环境

| 项 | 版本/型号 |
|---|---|
| 主控 | STM32F103C8T6（云台）、STM32F407VET6（移植验证） |
| 工具链 | arm-none-eabi-gcc 14.3 + CMake 4.4 + Ninja |
| RTOS | FreeRTOS（CubeMX 生成） |
| 调试 | J-Link + SEGGER Ozone |
| 上位机 | Python 3 + pyserial + pyqtgraph |

## 编译

```bash
cd firmware
cmake --preset Debug
cmake --build --preset Debug
```

产物：`firmware/build/Debug/duoji.elf`

## 关于 AI 的使用

本项目使用 AI Agent 辅助理解概念、排查问题和生成部分模板代码。
所有 AI 给出的关键结论都经过**手册核对 / 代码验证 / 实测现象**确认，
过程记录见 `docs/AI-会话记录.md`。

## 文档索引

- [AI 会话记录](docs/AI-会话记录.md)
- [开发进度](docs/01-进度.md)
