#ifndef __IOI2C_H
#define __IOI2C_H

#include "stm32f4xx_hal.h"

/* Ӳ�����Ŷ��� */
#define IIC_GPIO_PORT    GPIOB
#define IIC_SCL_PIN      GPIO_PIN_10
#define IIC_SDA_PIN      GPIO_PIN_11

/* �������� */
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
void HWT101_Init();
void HWT101_GetValue();

/* ���Ժ� */
//#define I2C_DEBUG  // ȡ��ע�����õ������



//����-------------------------------------------------------------------------------------------------
		/* USER CODE BEGIN 2 */

					//	HWT101_Init();
		/* USER CODE END 2 */

		/* USER CODE BEGIN 3 */
					//		extern 	float fAcc[3], fGyro[3], fAngle[3];
					//		HWT101_GetValue();

		/* USER CODE END 3 */


//-----------------------------------------------------------------------------------------------------

#endif