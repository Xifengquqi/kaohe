#ifndef __MPU6050_H
#define __MPU6050_H
#include "main.h"

#define delay_ms            HAL_Delay

/* 硬件 I2C: CubeMX 已初始化 hi2c1, 以下宏为空操作 */
#define MPU_IIC_Init()      ((void)0)
#define MPU_IIC_Start()     ((void)0)
#define MPU_IIC_Stop()      ((void)0)
#define MPU_IIC_Send_Byte(x) ((void)0)
#define MPU_IIC_Read_Byte(x) (0)
#define MPU_IIC_Wait_Ack()  (0)

#define MPU_SELF_TESTX_REG      0X0D
#define MPU_SELF_TESTY_REG      0X0E
#define MPU_SELF_TESTZ_REG      0X0F
#define MPU_SELF_TESTA_REG      0X10
#define MPU_SAMPLE_RATE_REG     0X19
#define MPU_CFG_REG             0X1A
#define MPU_GYRO_CFG_REG        0X1B
#define MPU_ACCEL_CFG_REG       0X1C
#define MPU_MOTION_DET_REG      0X1F
#define MPU_FIFO_EN_REG         0X23
#define MPU_I2CMST_CTRL_REG     0X24
#define MPU_I2CSLV0_ADDR_REG    0X25
#define MPU_I2CSLV0_REG         0X26
#define MPU_I2CSLV0_CTRL_REG    0X27
#define MPU_INTBP_CFG_REG       0X37
#define MPU_INT_EN_REG          0X38
#define MPU_INT_STA_REG         0X3A

#define MPU_ACCEL_XOUTH_REG     0X3B
#define MPU_ACCEL_XOUTL_REG     0X3C
#define MPU_ACCEL_YOUTH_REG     0X3D
#define MPU_ACCEL_YOUTL_REG     0X3E
#define MPU_ACCEL_ZOUTH_REG     0X3F
#define MPU_ACCEL_ZOUTL_REG     0X40

#define MPU_TEMP_OUTH_REG       0X41
#define MPU_TEMP_OUTL_REG       0X42

#define MPU_GYRO_XOUTH_REG      0X43
#define MPU_GYRO_XOUTL_REG      0X44
#define MPU_GYRO_YOUTH_REG      0X45
#define MPU_GYRO_YOUTL_REG      0X46
#define MPU_GYRO_ZOUTH_REG      0X47
#define MPU_GYRO_ZOUTL_REG      0X48

#define MPU_I2CSLV0_DO_REG      0X63
#define MPU_I2CMST_DELAY_REG    0X67
#define MPU_SIGPATH_RST_REG     0X68
#define MPU_MDETECT_CTRL_REG    0X69
#define MPU_USER_CTRL_REG       0X6A
#define MPU_PWR_MGMT1_REG       0X6B
#define MPU_PWR_MGMT2_REG       0X6C
#define MPU_FIFO_CNTH_REG       0X72
#define MPU_FIFO_CNTL_REG       0X73
#define MPU_FIFO_RW_REG         0X74
#define MPU_DEVICE_ID_REG       0X75

/* MPU6050 7位地址 (AD0接GND=0x68) */
#define MPU_ADDR                0X68

uint8_t MPU_Init(void);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);
uint8_t MPU_Read_Byte(uint8_t reg);

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);

short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif