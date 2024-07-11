/*
 * Driver_gpio.h
 *
 *  Created on: 09-Jul-2024
 *      Author: Gurjinder Singh
 */

#ifndef INC_DRIVER_GPIO_H_
#define INC_DRIVER_GPIO_H_

#include "stm32f4xx.h"

typedef struct
{
	uint8_t GPIO_PinNo;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtr;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAF_Mode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t * GPIO_RegConfig;
	GPIO_PinConfig_t   GPIO_PinConfig;
}GPIO_Handle_t;


//GPIO Pin no
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


//GPIO Pin Mode
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

//GPIO Pin Output Type
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

//GPIO Pin Speed
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MED			1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

//GPIO Pin Pull up/down config
#define GPIO_PIN_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


//GPIO APIs
//GPIO Clock Setup
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI );
// GPIO Init
void GPIO_Init(GPIO_Handle_t *pGPIoHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
//Data Read/Write
uint8_t GPIO_ReadInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);
uint16_t GPIO_ReadInPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value);
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo);
//IRQ handle
void GPIO_IRQ_Config(uint8_t IRQNum, uint8_t IRQPrio, uint8_t EN_DI);
void GPIO_IRQ_Handle(uint8_t PinNo);

#endif /* INC_DRIVER_GPIO_H_ */
