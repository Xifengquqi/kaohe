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
/*
 * 硬件连接 (详细接线见工程根目录 README.md):
 *   MPU6050  VCC -> 3.3V   GND -> GND
 *            SCL -> PB6 (I2C1_SCL)
 *            SDA -> PB7 (I2C1_SDA)
 *            AD0 -> GND  (器件地址 0x68)    INT 悬空即可
 *   俯仰舵机 信号 -> PA6 (TIM3_CH1)
 *   航向舵机 信号 -> PA7 (TIM3_CH2)
 *   舵机电源单独 5V 供电, 必须和开发板共地!
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"   /* MPU6050 寄存器读写 (已改成 HAL 硬件 I2C 版本) */
#include "inv_mpu.h"   /* mpu_dmp_init() / mpu_dmp_get_data() 姿态解算 */
#include "pid.h"       /* 二轴云台的 PID 控制器 */

/* 教程笔记里写的是 MPU_DMP_INIT / TU6050_GetData,
 * 驱动库实际导出的函数名是 mpu_dmp_init / mpu_dmp_get_data,
 * 这里做个别名, 两种写法都能编译通过。 */
#define MPU_DMP_INIT()              mpu_dmp_init()
#define MPU6050_GetData(p, r, y)    mpu_dmp_get_data((p), (r), (y))
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
#define PID_OUT_LIMIT       400.0f

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

/* USER CODE BEGIN PV */
/* MPU6050 DMP 解算出来的三个姿态角, 单位: 度
 * (调试时把这三个变量加到 Watch 窗口就能看到实时姿态) */
float pitch = 0.0f;   /* 俯仰角: -90 ~ +90   */
float roll  = 0.0f;   /* 横滚角: -180 ~ +180 */
float yaw   = 0.0f;   /* 航向角: -180 ~ +180 */

/* 两轴 PID 控制器 */
PID_t pid_pitch;
PID_t pid_yaw;

/* 当前输出的脉宽, 调试用 */
uint16_t servo_pitch_us = SERVO_MID_US;
uint16_t servo_yaw_us   = SERVO_MID_US;

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
/* USER CODE BEGIN PFP */
static void Gimbal_Init(void);
static void Gimbal_Update(void);
static void Servo_SetPulse(uint32_t channel, int16_t pulse_us);
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
    PID_Init(&pid_pitch, 8.0f, 0.3f, 0.0f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
    PID_Init(&pid_yaw,   8.0f, 0.3f, 0.0f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
    PID_SetTarget(&pid_pitch, TARGET_PITCH);
    PID_SetTarget(&pid_yaw,   TARGET_YAW);
}

/**
  * @brief  云台控制: 读姿态 -> PID 计算 -> 输出 PWM, 每 10ms 调用一次
  */
static void Gimbal_Update(void)
{
    float pitch_out, yaw_out;

    if (!mpu_ready) return;

    /* 读取 DMP 解算好的姿态角(单位: 度), 返回 0 表示读到有效数据 */
    if (MPU6050_GetData(&pitch, &roll, &yaw) != 0) return;

    /* 目标角度 - 当前角度 = 偏差, PID 算出需要补偿的脉宽 */
    pitch_out = PID_Compute(&pid_pitch, pitch, CONTROL_DT);
    yaw_out   = PID_Compute(&pid_yaw,   yaw,   CONTROL_DT);

    /* 云台增稳: 传感器测到往哪边歪, 舵机就往反方向补多少 */
    Servo_SetPulse(TIM_CHANNEL_1,
                   (int16_t)(SERVO_MID_US + PITCH_SERVO_SIGN * pitch_out));
    Servo_SetPulse(TIM_CHANNEL_2,
                   (int16_t)(SERVO_MID_US + YAW_SERVO_SIGN * yaw_out));
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
  /* USER CODE BEGIN 2 */
  Gimbal_Init();                 /* 舵机 + MPU6050 DMP 初始化 */
  last_tick = HAL_GetTick();     /* 控制周期计时起点 */
  /* USER CODE END 2 */

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
  htim3.Init.Prescaler = 71;                 /* 72MHz / 72 = 1MHz, 1us 计一个数 */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;                 /* 20000us = 20ms -> 50Hz 舵机周期 */
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
  hi2c1.Init.ClockSpeed = 400000;            /* 400kHz, MPU6050 支持 */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  * @param  line: source line number
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
