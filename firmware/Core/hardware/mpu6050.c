#include "stm32f1xx_hal.h"
#include "mpu6050.h"

/* 外部 I2C 句柄, CubeMX 生成的 hi2c1 */
extern I2C_HandleTypeDef hi2c1;

/* HAL I2C 超时 (ms) */
#define MPU_I2C_TIMEOUT  100

/* MPU6050 8 位地址 (HAL 已自动左移, 传 7 位地址) */
#define MPU_I2C_ADDR     (MPU_ADDR << 1)

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Init(void)
{
    uint8_t res;

    MPU_IIC_Init();//硬件I2C已由CubeMX初始化,此函数为空操作
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
    MPU_Set_Rate(100);						//设置采样率100Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
        MPU_Set_Rate(100);						//设置采样率为100Hz
    } else return 1;
    return 0;
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
    return MPU_Set_LPF(rate/2);
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;
    return temp*100;;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];
        *gy=((uint16_t)buf[2]<<8)|buf[3];
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;;
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;;
}

/* ============================================================
 *  以下四个函数改为使用 HAL 硬件 I2C (CubeMX 生成的 hi2c1)
 *  替换原来的软件 I2C 逐字节操作
 * ============================================================ */

//IIC连续写 (HAL 硬件 I2C 版本)
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    /* HAL_I2C_Mem_Write: 写指定寄存器起始的连续数据
     * addr 左移 1 位 = 8 位地址 (HAL 要求) */
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len,
                          MPU_I2C_TIMEOUT) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

//IIC连续读 (HAL 硬件 I2C 版本)
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len,
                         MPU_I2C_TIMEOUT) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

//IIC写一个字节 (HAL 硬件 I2C 版本)
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    if (HAL_I2C_Mem_Write(&hi2c1, MPU_I2C_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, &data, 1,
                          MPU_I2C_TIMEOUT) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

//IIC读一个字节 (HAL 硬件 I2C 版本)
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU_I2C_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT, &data, 1,
                     MPU_I2C_TIMEOUT);
    return data;
}