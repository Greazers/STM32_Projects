/*
 * Driver_gpio.c
 *
 *  Created on: 09-Jul-2024
 *      Author: Gurjinder Singh
 */
#include "Driver_gpio.h"


//GPIO Clock Enable
void GPIO_PClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EN_DI )
{
	if(EN_DI == ENABLE)
	{
		if( pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if( pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if( pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if( pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if( pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if( pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if( pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if( pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if( pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if( pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if( pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if( pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if( pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if( pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else{
			GPIOH_PCLK_DI();
		}
	}
}
// GPIO Init
void GPIO_Init(GPIO_Handle_t *pGPIoHandle)
{
	//Note: GPIO_Handle_t is filled by user from application layer. And this API will Fill that info to GPIO's Peripheral Registers
	//1. Mode of pin
	if(pGPIoHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non Interrupt
		pGPIoHandle->GPIO_RegConfig->MODER &= ~(0x03 <<  (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo)); // clear Register before setting its value
		pGPIoHandle->GPIO_RegConfig->MODER |= ((pGPIoHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo));
	}
	else{
		//interrupt
	}

	//2. Speed of Pin
	pGPIoHandle->GPIO_RegConfig->OSPEEDR &= ~(0x03 <<  (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo)); // clear Register before setting its value
	pGPIoHandle->GPIO_RegConfig->OSPEEDR |= (pGPIoHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo));


	//3. pupd setting
	pGPIoHandle->GPIO_RegConfig->PUPDR &= ~(0x03 <<  (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo)); // clear Register before setting its value
	pGPIoHandle->GPIO_RegConfig->PUPDR |= (pGPIoHandle->GPIO_PinConfig.GPIO_PinPuPdCtr << (2 * pGPIoHandle->GPIO_PinConfig.GPIO_PinNo));

	//4. OP type
	pGPIoHandle->GPIO_RegConfig->OTYPER &= ~(0x01 <<  pGPIoHandle->GPIO_PinConfig.GPIO_PinNo); // clear Register before setting its value
	pGPIoHandle->GPIO_RegConfig->OTYPER |= (pGPIoHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIoHandle->GPIO_PinConfig.GPIO_PinNo);

	//5. Alternate function
	//uint8_t AF_reg = pGPIoHandle->GPIO_PinConfig.GPIO_PinNo/8;
	if(pGPIoHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		pGPIoHandle->GPIO_RegConfig->AFR[pGPIoHandle->GPIO_PinConfig.GPIO_PinNo/8] &= ~(0x0F << (( pGPIoHandle->GPIO_PinConfig.GPIO_PinNo % 8) * 4));

		pGPIoHandle->GPIO_RegConfig->AFR[pGPIoHandle->GPIO_PinConfig.GPIO_PinNo/8] |= (pGPIoHandle->GPIO_PinConfig.GPIO_PinAF_Mode << (( pGPIoHandle->GPIO_PinConfig.GPIO_PinNo % 8) * 4));
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if( pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if( pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if( pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if( pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if( pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if( pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if( pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else{
		GPIOH_REG_RESET();
	}

}
//Data Read/Write
uint8_t GPIO_ReadInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	return (uint8_t)((pGPIOx->IDR >> PinNo) & 0x01);

}
uint16_t GPIO_ReadInPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo, uint8_t value)
{
	if(value == SET){
		pGPIOx->ODR |= (1 << PinNo);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNo);
	}

}
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (uint32_t)value;
}
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNo)
{
	pGPIOx->ODR ^= (1 << PinNo) ;
}
//IRQ handle
void GPIO_IRQ_Config(uint8_t IRQNum, uint8_t IRQPrio, uint8_t EN_DI)
{

}
void GPIO_IRQ_Handle(uint8_t PinNo)
{

}


