#include "stm32f10x.h"
#include "MPU6050_Reg.h"
#define MPU6050_ADDRESS 0xD0
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT){
    uint32_t Timeout = 10000;
    while (I2C_CheckEvent(I2C2,I2C_EVENT)!=SUCCESS)
    {
        Timeout -- ;
        if(!Timeout)break;
    }
    
}
void MPU6050_WriteReg(uint8_t Reg_Address,uint8_t byte){
    
    I2C_GenerateSTART(I2C2,ENABLE);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C2,Reg_Address);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    I2C_SendData(I2C2,byte);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    I2C_GenerateSTOP(I2C2,ENABLE);

}
uint8_t MPU6050_ReadReg(uint8_t Reg_Address){

    I2C_GenerateSTART(I2C2,ENABLE);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C2,Reg_Address);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_GenerateSTART(I2C2,ENABLE);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2,MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(I2C2,DISABLE);
    I2C_GenerateSTOP(I2C2,ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
    uint8_t data =  I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return data;
}

void MPU6050_Init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    I2C_InitTypeDef   I2C_InitStructure;
    I2C_InitStructure.I2C_Ack= I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress= I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed= 50000;
    I2C_InitStructure.I2C_DutyCycle= I2C_DutyCycle_2 ;
    I2C_InitStructure.I2C_Mode= I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1= 0x00;
    I2C_Init(I2C2,&I2C_InitStructure);

    I2C_Cmd(I2C2,ENABLE);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}