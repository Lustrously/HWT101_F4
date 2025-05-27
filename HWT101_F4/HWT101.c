#include "HWT101.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdio.h>  
#include "wit_c_sdk.h"

/* 私有函数 */
static void SDA_IN(void);
static void SDA_OUT(void);
static void Delay(uint32_t count);


#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
float fAcc[3], fGyro[3], fAngle[3];

static void CmdProcess(void);
static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);



//-------------------------------------------------------
// 延时函数（保持F1原始参数）
//-------------------------------------------------------
#define F1_CLOCK       72    // F1主频72MHz
#define F4_CLOCK       168   // F4主频168MHz
#define DELAY_SCALE    ((float)F4_CLOCK / F1_CLOCK)  // 延时缩放系数

static void Delay(uint32_t count)
{
    // 保持F1的循环次数，通过缩放系数补偿主频差异
    volatile uint32_t i = (uint32_t)(count * 8 * DELAY_SCALE);
    while(i--);
}

//-------------------------------------------------------
// GPIO方向切换
//-------------------------------------------------------
static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;        // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 启用内部上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);
}

static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // 保持上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);
}

//-------------------------------------------------------
// 初始化函数
//-------------------------------------------------------
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置SCL引脚 */
    GPIO_InitStruct.Pin = IIC_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);
    
    /* 初始配置SDA为输出 */
    SDA_OUT();
    
    /* 释放总线 */
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN|IIC_SDA_PIN, GPIO_PIN_SET);
}

//-------------------------------------------------------
// 基本I2C操作
//-------------------------------------------------------
void IIC_Start(void)
{
    SDA_OUT();
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    Delay(5);
    
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET);
    Delay(5);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
}

void IIC_Stop(void)
{
    SDA_OUT();
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET);
    Delay(5);
    
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    Delay(5);
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();
    
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    Delay(1);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    Delay(5);
    
    while(HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN))
    {
        if(++ucErrTime > 50)
        {
            IIC_Stop();
            return 1;
        }
        Delay(5);
    }
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    return 0;
}

void IIC_Ack(void)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    SDA_OUT();
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET);
    Delay(5);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    Delay(5);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
}

void IIC_NAck(void)
{
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    SDA_OUT();
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
    Delay(5);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
    Delay(5);
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
}

//-------------------------------------------------------
// 数据收发
//-------------------------------------------------------
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
    
    for(t=0; t<8; t++)
    {
        HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, (txd & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        txd <<= 1;
        Delay(2);
        
        HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
        Delay(5);
        HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
        Delay(3);
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();
    
    for(i=0; i<8; i++)
    {
        HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET);
        Delay(5);
        HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
        receive <<= 1;
        if(HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN)) receive |= 0x01;
        Delay(5);
    }
    if(ack) IIC_Ack();
    else IIC_NAck();
    return receive;
}

//-------------------------------------------------------
// 高级读写函数
//-------------------------------------------------------
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    IIC_Start();
    IIC_Send_Byte(dev & 0xFE);
    if(IIC_Wait_Ack()) return 0;
    
    IIC_Send_Byte(reg);
    if(IIC_Wait_Ack()) return 0;
    
    IIC_Start();
    IIC_Send_Byte(dev | 0x01);
    if(IIC_Wait_Ack()) return 0;
    
    for(uint32_t i=0; i<length; i++)
    {
        data[i] = IIC_Read_Byte((i != (length-1)) ? 1 : 0);
    }
    IIC_Stop();
    return 1;
}

int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
    IIC_Start();
    IIC_Send_Byte(dev & 0xFE);
    if(IIC_Wait_Ack()) return 0;
    
    IIC_Send_Byte(reg);
    if(IIC_Wait_Ack()) return 0;
    
    for(uint32_t i=0; i<length; i++)
    {
        IIC_Send_Byte(data[i]);
        if(IIC_Wait_Ack()) return 0;
    }
    IIC_Stop();
    return 1;
}




void delay_ms(uint16_t ucMs)
{
	    volatile uint32_t i = ucMs * 168000 / 4;  // 近似实现（避免HAL_Delay）
    while(i--);
	
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			delay_ms(5);
			if(s_cDataUpdate != 0)
			{
//				printf("find %02X addr sensor\r\n", i);
//				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
//	printf("can not find sensor\r\n");
//	printf("please check your connection\r\n");
}


void HWT101_Init()
{
		WitInit(WIT_PROTOCOL_I2C, 0x50);
		WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
		WitRegisterCallBack(CopeSensorData);
		WitDelayMsRegister(delay_ms);
		AutoScanSensor();	

}



void HWT101_GetValue()
{
		WitReadReg(AX, 12);
//		delay_ms(50);

		if(s_cDataUpdate)
		{
			for(int i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
				//printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				s_cDataUpdate &= ~ACC_UPDATE;
			}
			if(s_cDataUpdate & GYRO_UPDATE)
			{
//				printf("gyro: %.3f\r\n", fGyro[2]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				printf("angle: %.3f\r\n",  fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				//printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				s_cDataUpdate &= ~MAG_UPDATE;
			}
		}
	
	
	
}
