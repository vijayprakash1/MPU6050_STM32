#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "stdint.h"
#include "stm32l4xx_hal.h"
#include "string.h"
#include "stdbool.h"

#define WHO_AM_I_REG		  0x75
#define MPU_ADDR			    0x68
#define PWR_MAGT_1_REG		0x6B
#define CONFIG_REG			  0x1A
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C
#define RA_FF_THR         0x1D
#define RA_FF_DUR         0x1E
#define RA_MOT_THR        0x1F
#define SMPLRT_DIV_REG		0x19
#define INT_STATUS_REG		0x3A
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define FIFO_EN_REG 		  0x23
#define INT_ENABLE_REG 		0x38
#define I2CMACO_REG 		  0x23
#define USER_CNT_REG		  0x6A
#define FIFO_COUNTH_REG 	0x72
#define FIFO_R_W_REG 		  0x74
#define RA_MOT_DUR        0x20
#define RA_MOT_DETECTION  0x69
#define CLOCK_INTERNAL    0x00  
#define CLOCK_PLL_XGYRO   0x01
#define CLOCK_PLL_YGYRO   0x02
#define CLOCK_PLL_ZGYRO   0x03
#define CLOCK_PLL_EXT32K  0x04
#define CLOCK_PLL_EXT19M  0x05
#define CLOCK_KEEP_RESET  0x07
#define RA_MOT_CTRL       0x69

#define GYRO_FS_250         0x00   // for 250
#define GYRO_FS_500         0x01   // for 500
#define GYRO_FS_1000        0x02   // for 1000
#define GYRO_FS_2000        0x03   // for 2000


#define ACCEL_FS_2          0x00   // for 2g
#define ACCEL_FS_4          0x01   // for 4g
#define ACCEL_FS_8          0x02   // for 8g
#define ACCEL_FS_16         0x03   // for 16g

#define DLPF_BW_256         0x00   // 0 ms
#define DLPF_BW_188         0x01   // 2ms
#define DLPF_BW_98          0x02   // 3ms
#define DLPF_BW_42          0x03   // 4.9ms
#define DLPF_BW_20          0x04   // 8.5ms
#define DLPF_BW_10          0x05   // 13.8ms
#define DLPF_BW_5           0x06   // 19ms



void MPU_Init(I2C_HandleTypeDef *I2Chnd);
void MPU_Config(uint8_t ClkSrc,uint8_t Gyro_Scale, uint8_t Accel_Scale, uint8_t DLFP ,bool Sleep_Mode_Bit );
void Get_Accel_Raw(int16_t* ax, int16_t* ay, int16_t* az);
void Get_Gyro_Raw(int16_t* gx, int16_t* gy, int16_t* gz);
uint8_t getFullScaleAccelRange(void);
uint8_t getFullScaleGyroRange(void);
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void setFreeFallDetectionThreshold(uint8_t threshold);
void setFreeFallDetectionDuration(uint8_t duration);
void setMotionDetectionThreshold(uint8_t threshold);
void setMotionDetectionDuration(uint8_t duration);
void setFreeFallEnable(bool enable);
void setIntMotionEnable(bool enable);
void setIntZeroMotionEnable(bool enable);
void setAccelerometerPoweronDelay(uint8_t delay);
void setMotionDetectionDuration(uint8_t duration);



#endif


