# 二轴云台（STM32F103C8T6 + MPU6050 + 双舵机）

在原来"舵机转动"工程的基础上加了 MPU6050（DMP 姿态解算）和 PID 闭环，
做成一个能自动保持水平的二轴云台。

---

## 1. 接线

| MPU6050 模块 | STM32F103C8T6 | 说明 |
|---|---|---|
| VCC | 3.3V | 不要接 5V |
| GND | GND | 必须共地 |
| SCL | PB6 | I2C1_SCL |
| SDA | PB7 | I2C1_SDA |
| AD0 | GND | 器件地址 0x68（接 3.3V 则变 0x69） |
| INT | 不接 | 程序用轮询读 FIFO，不用中断 |

| 舵机 | STM32F103C8T6 | 说明 |
|---|---|---|
| 俯仰舵机信号（橙/白线） | PA6 | TIM3_CH1，50Hz，500~2500us |
| 航向舵机信号（橙/白线） | PA7 | TIM3_CH2，50Hz，500~2500us |
| 舵机电源（红） | 独立 5V | **不要**用开发板的 3.3V/5V 排针带两个舵机 |
| 舵机地（棕/黑） | GND | 和开发板共地，否则 PWM 会乱 |

> MPU6050 模块要固定在**云台的负载平台**上（跟着舵机一起转），
> 这样舵机转动 → 姿态变化 → PID 反向补偿，形成闭环。

---

## 2. 目录结构（驱动库统一管理）

```
duoji/
├── Core/
│   ├── Inc/            STM32CubeMX 生成的 HAL 头文件
│   ├── Src/
│   │   ├── main.c      云台主程序（只在这个文件里写业务代码）
│   │   └── ...         HAL / 启动相关
│   └── hardware/       ★ 所有外设驱动都放这里，统一管理
│       ├── mpu6050.c/.h                  MPU6050 寄存器读写（HAL 硬件 I2C 版）
│       ├── inv_mpu.c/.h                  InvenSense 驱动 + DMP 初始化/解算
│       ├── inv_mpu_dmp_motion_driver.c/.h DMP 固件与功能配置
│       ├── dmpKey.h / dmpmap.h           DMP 地址表（驱动库压缩包里缺这两个文件，已补上）
│       └── pid.c/.h                      位置式 PID
├── CMakeLists.txt      新增源文件/头文件路径都在这里登记
└── duoji.ioc           STM32CubeMX 工程文件
```

CMakeLists.txt 里对应的两处：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/hardware/mpu6050.c
    Core/hardware/inv_mpu.c
    Core/hardware/inv_mpu_dmp_motion_driver.c
    Core/hardware/pid.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Core/hardware
)
```

> 原来的软件 I2C（`IIC.c` / `IIC.h`，GPIO 模拟时序）已经删掉，改用 CubeMX
> 里配置好的硬件 I2C1（PB6/PB7，400kHz）。`mpu6050.c` 里的读写函数也换成了
> `HAL_I2C_Mem_Read/Write`，速度更快、时序更稳，也不会和 CubeMX 的引脚配置打架。

---

## 3. 代码对应教程里的 4 步

全部改动都在 `Core/Src/main.c`，并且写在 CubeMX 的 `USER CODE` 区域里，
以后重新生成代码不会被覆盖。

**① 引入头文件**（`USER CODE BEGIN Includes`）

```c
#include "mpu6050.h"
#include "inv_mpu.h"
#include "pid.h"
```

**② 定义全局变量放姿态角**（`USER CODE BEGIN PV`）

```c
float pitch = 0.0f;   /* 俯仰角 -90 ~ +90  */
float roll  = 0.0f;   /* 横滚角 -180 ~ +180 */
float yaw   = 0.0f;   /* 航向角 -180 ~ +180 */
```

**③ 在 while(1) 之前初始化**（`USER CODE BEGIN 2` → `Gimbal_Init()`）

```c
Gimbal_Init();               /* 内部调用 MPU_DMP_INIT() */
```

**④ 主循环里读姿态并控制**（`Gimbal_Update()`，每 10ms 一次）

```c
if (MPU6050_GetData(&pitch, &roll, &yaw) != 0) return;   /* 读取三个姿态角 */
pitch_out = PID_Compute(&pid_pitch, pitch, CONTROL_DT);  /* PID 算补偿量 */
Servo_SetPulse(TIM_CHANNEL_1, SERVO_MID_US + pitch_out); /* 输出给舵机 */
```

### 函数名对照（教程写法 vs 驱动库真实函数）

| 教程/笔记里写的 | 驱动库里真正的函数 |
|---|---|
| `MPU_DMP_INIT()` | `mpu_dmp_init()` |
| `TU6050_GetData()` / `MPU6050_GetData()` | `mpu_dmp_get_data(&pitch, &roll, &yaw)` |

为了照抄教程也能编译，main.c 开头做了宏别名：

```c
#define MPU_DMP_INIT()              mpu_dmp_init()
#define MPU6050_GetData(p, r, y)    mpu_dmp_get_data((p), (r), (y))
```

---

## 4. 编译与烧录

VSCode（装了 STM32Cube 插件）里直接点 Build；或者命令行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成 `build/Debug/duoji.elf`，用 STM32CubeProgrammer / J-Link 烧进去。
本工程实测：FLASH 39.1KB / 64KB，RAM 2.3KB / 20KB。

> 注：驱动库压缩包里没有 `dmpKey.h`、`dmpmap.h`，但 `inv_mpu_dmp_motion_driver.c`
> 需要它们才能编译。工程里补了精简版（只保留实际用到的常量，数值取自
> InvenSense eMPL 5.1 原版），编译即可通过。

---

## 5. 调试与调参

1. **先看姿态**：Debug 时把 `pitch / roll / yaw` 加到 Watch 窗口，
   转动模块看数值是否跟着变（不动或全是 0 → 检查 I2C 接线 / AD0）。
2. **方向反了**：云台越补越歪、往一个方向"跑飞"，把 `main.c` 里
   `PITCH_SERVO_SIGN` 或 `YAW_SERVO_SIGN` 的 `+1` 改成 `-1`。
3. **抖 / 回中慢**：
   - 抖：先把 `Kp`（当前 8.0）调小，或把 `Kd` 加一点；
   - 回中慢、有静差：把 `Ki`（当前 0.3）慢慢加大。
   - 参数在 `Gimbal_Init()` 里的 `PID_Init()` 处改。
4. **限幅**：`PID_OUT_LIMIT`（当前 400us ≈ 36°）限制单次最大摆动，
   防止舵机堵转，按机械行程改。
5. **航向角漂移**：MPU6050 没有磁力计，DMP 的 yaw 会缓慢漂移，属正常现象；
   只做俯仰+横滚增稳时可以不用 yaw。

---

## 6. 初始化失败排查

`Gimbal_Init()` 里如果 `MPU_DMP_INIT()` 一直失败，程序会重试 10 次后
让舵机停在中位（不会乱摆）。此时重点查：

- SCL/SDA 有没有接反、有没有上拉（模块一般自带 4.7k 上拉）；
- AD0 是否接 GND（对应地址 0x68）；
- 供电是否 3.3V、是否共地；
- 用万用表量 PB6/PB7 静态应为高电平。

`mpu_dmp_init()` 的返回值含义（0 成功）：

| 返回值 | 卡在哪一步 |
|---|---|
| 1 | 设置传感器失败 |
| 2 | 配置 FIFO 失败 |
| 3 | 设置采样率失败 |
| 4 | 加载 DMP 固件失败 |
| 5 | 设置陀螺仪方向失败 |
| 6 | 使能 DMP 功能失败 |
| 7 | 设置 DMP 输出速率失败 |
| 9 | 启动 DMP 失败 |
| 10 | `mpu_init()` 失败（大多是 I2C 通信不上） |

---

## 7. 关于这块模块的芯片（WHO_AM_I = 0x74）

用 J-Link 直接读板上数据确认过：I2C 是通的（地址 0x68 有 ACK，能读写寄存器），
但这块模块的 **WHO_AM_I（寄存器 0x75）读出 0x74**，正品 MPU6050 应该是 **0x68**；
而且 0x06~0x0B（加速度零偏寄存器）读出来是乱值。

InvenSense 原版驱动在 `mpu_init()` 里会做"产品版本自检"，
读到不认识的版本号就直接 `return -1`，于是 `mpu_dmp_init()` 返回 10，
表现就是：**程序在跑、舵机回中，但 pitch/roll/yaw 全是 0**。

已在 `Core/hardware/inv_mpu.c` 的 `mpu_init()` 里加了兼容处理：

```c
else {
    /* 兼容非标芯片: 有些模块 WHO_AM_I 不是 0x68(如 0x74),
       产品版本位读出来是乱值, 这里不再直接判失败, 按普通灵敏度处理 */
    st.chip_cfg.accel_half = 0;
}
```

实测这颗芯片的数据寄存器、量程（16384 LSB/g）和 DMP 固件都完全兼容，
改完后 `mpu_ready = 1`，`pitch/roll/yaw` 正常输出。
以后换上正品 MPU6050（WHO_AM_I=0x68）也不受这段代码影响。
