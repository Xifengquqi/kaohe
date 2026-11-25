/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 二轴云台 (STM32F103C8T6 + MPU6050 DMP + 双舵机)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"   /* MPU6050 寄存器读写 (已改成 HAL 硬件 I2C 版本) */
#include "inv_mpu.h"   /* mpu_dmp_init() / mpu_dmp_get_data() 姿态解算 */
#include "pid.h"       /* 二轴云台的 PID 控制器 */
#include "uart_stm32.h"/* BSP: 调试串口 USART1 (PA9/PA10) */
#include "frame.h"     /* 通信帧打包 */
#include <string.h>

/* 教程笔记里写的是 MPU_DMP_INIT / TU6050_GetData,
 * 驱动库实际导出的函数名是 mpu_dmp_init / mpu_dmp_get_data,
 * 这里做个别名, 两种写法都能编译通过。 */
#define MPU_DMP_INIT()              mpu_dmp_init()
#define MPU6050_GetData(p, r, y)    mpu_dmp_get_data((p), (r), (y))
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 三个角度打包成一个结构体, 任务间传递用 */
typedef struct {
    float pitch;
    float roll;
    float yaw;
} Attitude_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 舵机脉宽: 定时器 1us 计一个数, 周期 20000 个数 = 20ms = 50Hz
 * 500us -> 0 度, 1500us -> 90 度(中位), 2500us -> 180 度 */
#define SERVO_MIN_US        500U
#define SERVO_MID_US        1500U
#define SERVO_MAX_US        2500U

/* 控制周期: 10ms = 100Hz, 和 DMP 输出速率(DEFAULT_MPU_HZ=100)一致 */
#define CONTROL_PERIOD_MS   10U
#define CONTROL_DT          0.01f

/* PID 输出限幅(单位 us): 限制每次最多摆动多少, 防止堵转、过冲。
 * 400us 大约对应 36 度, 想更"猛"可以加大, 想更"稳"可以减小 */
#define PID_OUT_LIMIT       250.0f

/* 安装方向修正: 云台往一边"跑飞"或越补越歪时,
 * 把对应轴的 +1 改成 -1 (只改符号, 不用动别的) */
#define PITCH_SERVO_SIGN    (+1)
#define YAW_SERVO_SIGN      (-1)

/* 希望云台保持的姿态: 水平 */
#define TARGET_PITCH        0.0f
#define TARGET_YAW          0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* MPU6050 DMP 解算出来的三个姿态角, 单位: 度
 * (调试时把这三个变量加到 Watch 窗口就能看到实时姿态) */
float pitch = 0.0f;   /* 俯仰角: -90 ~ +90   */
float roll  = 0.0f;   /* 横滚角: -180 ~ +180 */
float yaw   = 0.0f;   /* 航向角: -180 ~ +180 */

/* 传感器任务写、控制任务读的"最新姿态"
 * 姿态是状态量(只需要最新值), 不是事件, 所以不用队列,
 * 一个 volatile 结构体就够; Cortex-M3 上 32 位对齐读写是原子的 */
volatile Attitude_t g_attitude = { 0.0f, 0.0f, 0.0f };

/* 两轴 PID 控制器 */
PID_t pid_pitch;
PID_t pid_yaw;

/* 当前输出的脉宽, 调试用 */
uint16_t servo_pitch_us = SERVO_MID_US;
uint16_t servo_yaw_us   = SERVO_MID_US;

/* BSP 层的串口对象 + 上层使用的接口指针
 * 上层只认 iuart_t*, 看不到 UART_HandleTypeDef */
static uart_stm32_t s_dbg_uart;
static iuart_t     *s_dbg_uart_if = NULL;

static uint32_t last_tick = 0;   /* 上一次控制的时间戳 */

/* 调试用: MPU6050 初始化结果, 加到 Ozone 的 Watch 窗口里能直接看出来
 *   mpu_ready    : 1 = 初始化成功, 0 = 失败(这时舵机保持中位不动)
 *   mpu_init_ret : mpu_dmp_init() 最后一次返回值, 0=成功, 其他=卡在哪一步 */
uint8_t mpu_ready = 0;
uint8_t mpu_init_ret = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void Gimbal_Init(void);
static void Gimbal_Update(void);
static void Servo_SetPulse(uint32_t channel, int16_t pulse_us);
static void StartSensorTask(void const *argument);
static void StartCommTask(void const *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  设置某一路舵机的脉宽, 自动限幅到 500~2500us
  * @param  channel  : TIM_CHANNEL_1(俯仰) / TIM_CHANNEL_2(航向)
  * @param  pulse_us : 期望脉宽, 单位 us
  */
static void Servo_SetPulse(uint32_t channel, int16_t pulse_us)
{
    if (pulse_us < (int16_t)SERVO_MIN_US) pulse_us = (int16_t)SERVO_MIN_US;
    if (pulse_us > (int16_t)SERVO_MAX_US) pulse_us = (int16_t)SERVO_MAX_US;

    __HAL_TIM_SET_COMPARE(&htim3, channel, (uint16_t)pulse_us);

    if (channel == TIM_CHANNEL_1)
        servo_pitch_us = (uint16_t)pulse_us;
    else
        servo_yaw_us = (uint16_t)pulse_us;
}

/**
  * @brief  云台初始化: 舵机回中 -> MPU6050 DMP 初始化 -> PID 参数
  * @note   在 main() 的 USER CODE BEGIN 2 里调用, 即 while(1) 之前
  */
static void Gimbal_Init(void)
{
    uint8_t ret;
    uint8_t retry = 0;

    /* 1. 先把两路 PWM 写中位再启动, 上电时舵机不会突然抖一下 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MID_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_MID_US);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    servo_pitch_us = SERVO_MID_US;
    servo_yaw_us   = SERVO_MID_US;

    /* 2. MPU6050 + DMP 初始化, 失败重试 10 次(约 2 秒)
     *    返回值: 0=成功, 1~10=不同阶段的错误码(见 inv_mpu.c 注释) */
    do
    {
        ret = MPU_DMP_INIT();
        if (ret == 0) break;
        HAL_Delay(200);
    } while (++retry < 10);

    mpu_init_ret = ret;
    mpu_ready = (ret == 0);

    /* 初始化没成功: 舵机停在中位不动, 主循环只空转, 不会乱摆 */
    if (!mpu_ready) return;

    /* 3. PID 参数: 先只调 Kp(云台抖就减小, 回中慢就适当加 Ki) */
    PID_Init(&pid_pitch, 4.0f, 0.3f, 0.0f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
    PID_Init(&pid_yaw,   4.0f, 0.3f, 0.0f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
        /* 上电自校准: 静止采 20 次, 取平均作为"水平"基准 */
    {
        float sp = 0.0f, sr = 0.0f, sy = 0.0f;
        int   i, n = 0;

        for (i = 0; i < 20; i++)
        {
            float p, r, y;
            if (MPU6050_GetData(&p, &r, &y) == 0)
            {
                sp += p; sr += r; sy += y; n++;
            }
            HAL_Delay(10);          /* 20 次 × 10ms = 200ms */
        }
        if (n > 0)
        {
            PID_SetTarget(&pid_pitch, sp / n);
            PID_SetTarget(&pid_yaw,   sy / n);
        }
        else
        {
            PID_SetTarget(&pid_pitch, TARGET_PITCH);
            PID_SetTarget(&pid_yaw,   TARGET_YAW);
        }
    }
    
}

/**
  * @brief  把当前姿态打包成帧, 通过 USART1 发给上位机
  * @note   角度单位 0.01 度, 一帧 14 字节
  */
static void Telemetry_Send(void)
{
    static uint16_t seq = 0;
    uint8_t payload[8];
    uint8_t buf[FRAME_MAX_SIZE];
    uint16_t n;
    int16_t p = (int16_t)(pitch * 100.0f);
    int16_t r = (int16_t)(roll  * 100.0f);
    int16_t y = (int16_t)(yaw   * 100.0f);

    memcpy(&payload[0], &p, 2);
    memcpy(&payload[2], &r, 2);
    memcpy(&payload[4], &y, 2);
    memcpy(&payload[6], &seq, 2);
    seq++;

    n = frame_pack(CMD_ATTITUDE, payload, 8, buf, sizeof(buf));
    if (n > 0) {
        iuart_send(s_dbg_uart_if, buf, n, 5);   /* 只通过接口发, 不碰 HAL */
    }
}

/**
  * @brief  传感器任务: 每 5ms 取一次 DMP 姿态, 写进 g_attitude
  * @note   DMP 是 100Hz 输出, 5ms 轮询会有一半次数读不到新数据(返回 1),
  *         这是正常的, 跳过即可
  */
static void StartSensorTask(void const *argument)
{
    float p, r, y;

    (void)argument;

    for (;;)
    {
        if (mpu_ready && MPU6050_GetData(&p, &r, &y) == 0)
        {
            g_attitude.pitch = p;
            g_attitude.roll  = r;
            g_attitude.yaw   = y;
        }
        osDelay(5);
    }
}

/* ---------------- 通信: 指令处理 ---------------- */

/**
  * @brief  回一个 ACK 帧: [被回应的命令, 0=失败 1=成功]
  */
static void Comm_SendAck(uint8_t cmd, uint8_t ok)
{
    uint8_t  payload[2];
    uint8_t  buf[FRAME_MAX_SIZE];
    uint16_t n;

    payload[0] = cmd;
    payload[1] = ok;

    n = frame_pack(CMD_ACK, payload, 2, buf, sizeof(buf));
    if (n > 0) {
        iuart_send(s_dbg_uart_if, buf, n, 5);
    }
}

/**
  * @brief  处理一条来自上位机/蓝牙的指令
  * @note   指令格式见 docs/02-通信协议.md
  */
static void Comm_HandleFrame(const Frame *f)
{
    switch (f->cmd)
    {
    case CMD_SET_TARGET:            /* 目标角: int16 pitch, int16 yaw (0.01 度) */
        if (f->len >= 4)
        {
            int16_t p, y;
            memcpy(&p, &f->data[0], 2);
            memcpy(&y, &f->data[2], 2);

            PID_SetTarget(&pid_pitch, (float)p / 100.0f);
            PID_SetTarget(&pid_yaw,   (float)y / 100.0f);
            Comm_SendAck(CMD_SET_TARGET, 1);
        }
        else
        {
            Comm_SendAck(CMD_SET_TARGET, 0);
        }
        break;

    case CMD_SET_PID:               /* PID 参数: kp, ki, kd (都是 100 倍整数) */
        if (f->len >= 6)
        {
            int16_t kp, ki, kd;
            memcpy(&kp, &f->data[0], 2);
            memcpy(&ki, &f->data[2], 2);
            memcpy(&kd, &f->data[4], 2);

            pid_pitch.Kp = pid_yaw.Kp = (float)kp / 100.0f;
            pid_pitch.Ki = pid_yaw.Ki = (float)ki / 100.0f;
            pid_pitch.Kd = pid_yaw.Kd = (float)kd / 100.0f;
            PID_Reset(&pid_pitch);      /* 换参数后清掉旧的积分, 避免冲击 */
            PID_Reset(&pid_yaw);

            Comm_SendAck(CMD_SET_PID, 1);
        }
        else
        {
            Comm_SendAck(CMD_SET_PID, 0);
        }
        break;

    default:
        Comm_SendAck(f->cmd, 0);    /* 不认识的命令, 回失败 */
        break;
    }
}

/**
  * @brief  通信任务: 收字节 -> 喂状态机 -> 收到整帧就执行
  */
static void StartCommTask(void const *argument)
{
    FrameParser parser;
    Frame       frame;
    uint8_t     byte;

    (void)argument;

    frame_parser_init(&parser);

    for (;;)
    {
        /* 阻塞等 1 个字节, 最长 20ms; 超时就继续等 */
        if (iuart_recv(s_dbg_uart_if, &byte, 1, 20) != 0) {
            continue;
        }

        /* 状态机: 1 = 收到完整且 CRC 正确的帧 */
        if (frame_parser_feed(&parser, byte, &frame) == 1) {
            Comm_HandleFrame(&frame);
        }
    }
}

/**
  * @brief  云台控制: 读姿态 -> PID 计算 -> 输出 PWM, 每 10ms 调用一次
  */
static void Gimbal_Update(void)
{
    float pitch_out, yaw_out;

    if (!mpu_ready) return;

    /* 姿态由 SensorTask 更新, 这里只取最新值, 不再占用 I2C 时间 */
    pitch = g_attitude.pitch;
    roll  = g_attitude.roll;
    yaw   = g_attitude.yaw;

    /* 目标角度 - 当前角度 = 偏差, PID 算出需要补偿的脉宽 */
    pitch_out = PID_Compute(&pid_pitch, pitch, CONTROL_DT);
    yaw_out   = PID_Compute(&pid_yaw,   yaw,   CONTROL_DT);

    /* 云台增稳: 传感器测到往哪边歪, 舵机就往反方向补多少 */
    Servo_SetPulse(TIM_CHANNEL_1,
                   (int16_t)(SERVO_MID_US + PITCH_SERVO_SIGN * pitch_out));
    Servo_SetPulse(TIM_CHANNEL_2,
                   (int16_t)(SERVO_MID_US + YAW_SERVO_SIGN * yaw_out));

    /* 每 10 个控制周期(100ms) 发一帧姿态给上位机 */
    {
        static uint8_t div = 0;
        if (++div >= 10) {
            div = 0;
            Telemetry_Send();
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 串口/外设初始化由 CubeMX 生成的 MX_xxx_Init() 完成;
     云台的初始化搬到 StartDefaultTask 里, 否则会在调度器启动前卡住两秒 */

  /* 把 CubeMX 的 huart1 包成 iuart 接口, 上层只通过接口用串口 */
  uart_stm32_init(&s_dbg_uart, &huart1);
  s_dbg_uart_if = uart_stm32_as_iuart(&s_dbg_uart);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 768);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 传感器任务: 优先级高于控制任务, 保证 5ms 采样不被拖慢 */
  osThreadDef(sensorTask, StartSensorTask, osPriorityAboveNormal, 0, 512);
  if (osThreadCreate(osThread(sensorTask), NULL) == NULL)
  {
    Error_Handler();
  }

  /* 通信任务: 优先级最低, 丢一帧不影响云台稳定 */
  osThreadDef(commTask, StartCommTask, osPriorityBelowNormal, 0, 256);
  if (osThreadCreate(osThread(commTask), NULL) == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 每 10ms 执行一次姿态读取 + PID + 舵机输出 */
    if ((HAL_GetTick() - last_tick) >= CONTROL_PERIOD_MS)
    {
      last_tick = HAL_GetTick();
      Gimbal_Update();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* 控制任务: 100Hz 算 PID + 输出舵机 */
  uint32_t last = HAL_GetTick();

  Gimbal_Init();

  for(;;)
  {
    if ((HAL_GetTick() - last) >= CONTROL_PERIOD_MS)
    {
      last += CONTROL_PERIOD_MS;
      Gimbal_Update();
    }
    osDelay(1);   /* 让出 CPU, 1ms 后再看时间 */
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
