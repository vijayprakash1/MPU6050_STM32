#include "MPU6050.h"


static I2C_HandleTypeDef i2cHandler;

void MPU_Init(I2C_HandleTypeDef *I2Chnd){

        memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));

}

void I2C_Write(uint8_t ADDR, uint8_t data)
{

	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR;
	MPUADDR = (MPU_ADDR<<1);
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2, 100);


}


void I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 10);

}

void setClockSource(uint8_t ClkSrc, bool Sleep_Mode_Bit)
{
  uint8_t Buffer = 0;  //Clock Source 

	I2C_Write(PWR_MAGT_1_REG, 0x80);   //Reset Device
	HAL_Delay(100);
	Buffer = ClkSrc & 0x07; //change the 7th bits of register
	Buffer |= (Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.    
	

}  


void setFullScaleGyroRange(uint8_t Gyro_Scale)
{

	I2C_Write(GYRO_CONFIG_REG, Gyro_Scale);
	
}


 void setFullScaleAccelRange(uint8_t Accel_Scale)
{
	
	 I2C_Write(ACCEL_CONFIG_REG,Accel_Scale);

 }

void set_DLPF(uint8_t DLPF)
{
  
	I2C_Write(CONFIG_REG,DLPF);

}
 

void setFreeFallDetectionThreshold(uint8_t threshold){


					I2C_Write(RA_FF_THR,threshold);
	

}
void setFreeFallDetectionDuration(uint8_t duration){


					I2C_Write(RA_FF_DUR, duration);


}
void setMotionDetectionThreshold(uint8_t threshold){

					I2C_Write(RA_MOT_THR,threshold);

}
void setMotionDetectionDuration(uint8_t duration){

					I2C_Write(RA_MOT_DUR,duration);
          

}

void setAccelerometerPoweronDelay(uint8_t delay){
        
	      
				I2C_Write(RA_MOT_CTRL,delay);


}


void setFreeFallEnable(bool enable){
        
	
	      uint8_t buffer = 0;
	      buffer |= (enable <<7) & 0x40;
				I2C_Write(INT_ENABLE_REG, buffer);

}


void setIntZeroMotionEnable(bool enable){

				uint8_t buffer = 0;
				buffer |= (enable << 5) & 0x10;
	      I2C_Write(INT_ENABLE_REG, buffer);	      
					

}

void setIntMotionEnable(bool enable){

			uint8_t buffer = 0;
	    buffer |= (enable << 6)& 0x06;
			I2C_Write(INT_ENABLE_REG, buffer);

}


void MPU_Config(uint8_t ClkSrc,uint8_t Gyro_Scale, uint8_t Accel_Scale, uint8_t DLPF, bool Sleep_Mode_Bit )
{

		setClockSource(ClkSrc, Sleep_Mode_Bit );
    setFullScaleGyroRange(Gyro_Scale);
    setFullScaleAccelRange(Accel_Scale);
	  set_DLPF(DLPF);
    

}


uint8_t get_deviceID()
{
	
		uint8_t i2cBuf[2];
		uint8_t buffer[1];
		I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
		if((i2cBuf[1] && 0x01))
		{

		I2C_Read(WHO_AM_I_REG, buffer, 1);
	
		
	}		
    return buffer[0];
}


bool testConnection()
{
      
	return get_deviceID() == 0x34;
	
}

void Get_Accel_Raw(int16_t* ax, int16_t* ay, int16_t* az)
{

	uint8_t i2cBuf[2];
	uint8_t AcceArr[6];
	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1] && 0x01))
	{

		I2C_Read(ACCEL_XOUT_H_REG, AcceArr, 6);
		*ax = ((AcceArr[0]<<8) + AcceArr[1]); //Store first two bytes into accelX
		*ay = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis //Store middle two bytes into accelY
		*az = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
	}
}	
	
void Get_Gyro_Raw(int16_t* gx, int16_t* gy, int16_t* gz)
{


	uint8_t i2cBuf[2];
	uint8_t GyroArr[6];
	I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1] && 0x01))
	{

		I2C_Read(GYRO_XOUT_H_REG, GyroArr, 6);
		*gx = ((GyroArr[0]<<8) + GyroArr[1]); //Store first two bytes into accelX
		*gy = ((GyroArr[2]<<8) + GyroArr[3]); // y-Axis //Store middle two bytes into accelY
		*gz = ((GyroArr[4]<<8) + GyroArr[5]); // z-Axis
		
	}		


}


uint8_t getFullScaleAccelRange(void) {
    uint8_t i2cBuf[2];
    uint8_t buffer[1];
		I2C_Read(INT_STATUS_REG,&i2cBuf[1],1);
		if((i2cBuf[1] && 0x01))
		{
    I2C_Read(GYRO_CONFIG_REG, buffer, 1);
		
		}	
		return buffer[0];
}

uint8_t getFullScaleGyroRange(void ) {
    uint8_t i2cBuf[2];
	  uint8_t buffer[1];
		I2C_Read(INT_STATUS_REG,&i2cBuf[1],1);
		if((i2cBuf[1] && 0x01))
		{
    I2C_Read(ACCEL_XOUT_H_REG, buffer, 1);
		
		}	
		return buffer[0];
}



void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	
	  uint8_t i2cBuf[2];
	  uint8_t buffer[14];
		I2C_Read(INT_STATUS_REG,&i2cBuf[1],1);
		if((i2cBuf[1] && 0x01))
		{
    I2C_Read(ACCEL_XOUT_H_REG, buffer, 14);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
		}
}

