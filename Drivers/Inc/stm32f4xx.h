/*
 * stm32f4xx.h
 *
 *  Created on: 09-Jul-2024
 *      Author: Gurjinder Singh
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>

#define __vo volatile

#define FLASH_BASEADDR			0x08000000U									// 1 MB Flash Memory Start Address
#define SRAM1_BASEADDR			0x20000000U									//SRAM1 Start Address
#define SRAM1_SIZE				112*1024U      							   	//SRAM1 Size 112 KB
#define SRAM2_BASEADDR			0x2001C000U (SRAM1_BASEADDR + SRAM1_SIZE) 	// SRAM2 Size is 16KB
#define SRAM					SRAM1_BASEADDR  							//128KB SRAM Start Address
#define ROM						0x1FFF0000U  								//30KB System Memory

#define PERPH_BASE 				0x40000000U
#define APB1PERPH_BASE 			PERPH_BASE
#define APB2PERPH_BASE 			0x40010000U
#define AHB1PERPH_BASE 			0x40020000U
#define AHB2PERPH_BASE 			0x50000000U


/*
 * Define Base address of peripherals which are hanging on AHB1 Bus
 * 168MHz Max
 * AHB1>> APB1 & APB2
 * Peripherals : GPIOA to GPIOI
*/

#define GPIOA_BASEADDR			(AHB1PERPH_BASE + 0x0000U)
#define GPIOB_BASEADDR			(AHB1PERPH_BASE + 0x0400U)
#define GPIOC_BASEADDR			(AHB1PERPH_BASE + 0x0800U)
#define GPIOD_BASEADDR			(AHB1PERPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR			(AHB1PERPH_BASE + 0x1000U)
#define GPIOF_BASEADDR			(AHB1PERPH_BASE + 0x1400U)
#define GPIOG_BASEADDR			(AHB1PERPH_BASE + 0x1800U)
#define GPIOH_BASEADDR			(AHB1PERPH_BASE + 0x1C00U)
#define CRC_BASEADDR			(AHB1PERPH_BASE + 0x3000U)
#define RCC_BASEADDR			(AHB1PERPH_BASE + 0x3800U)
#define FLASH_INT_BASEADDR		(AHB1PERPH_BASE + 0x3C00U)
#define DMA1_BASEADDR			(AHB1PERPH_BASE + 0x6000U)
#define DMA2_BASEADDR			(AHB1PERPH_BASE + 0x6400U)
#define DMA2D_BASEADDR			(AHB1PERPH_BASE + 0xB000U)
#define USB_OTG_HS_BASEADDR		(AHB1PERPH_BASE + 0x20000U)

/*
 * Define Base address of peripherals which are hanging on APB1 Bus
 * 142MHz Max
 * Peripherals : TIM2-7, TIM12-14, I2C1-3, SPI2-3, USART2-3, UART4-5, CAN1-2, DAC1-2, WWDG, RTC, PWR
*/
#define TIM2_BASEADDR			(APB1PERPH_BASE + 0x0000)
#define TIM3_BASEADDR			(APB1PERPH_BASE + 0x0400)
#define TIM4_BASEADDR			(APB1PERPH_BASE + 0x0800)
#define TIM5_BASEADDR			(APB1PERPH_BASE + 0x0C00)
#define TIM6_BASEADDR			(APB1PERPH_BASE + 0x1000)
#define TIM7_BASEADDR			(APB1PERPH_BASE + 0x1400)
#define TIM12_BASEADDR			(APB1PERPH_BASE + 0x1800)
#define TIM13_BASEADDR			(APB1PERPH_BASE + 0x1C00)
#define TIM14_BASEADDR			(APB1PERPH_BASE + 0x2000)
#define WWDG_BASEADDR			(APB1PERPH_BASE + 0x2C00)
#define IWDG_BASEADDR			(APB1PERPH_BASE + 0x3000)
#define SPI2_I2S2_BASEADDR		(APB1PERPH_BASE + 0x3800)
#define SPI3_I2S3_BASEADDR		(APB1PERPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERPH_BASE + 0x4C00)
#define I2C1_BASEADDR			(APB1PERPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERPH_BASE + 0x5800)
#define CAN1_BASEADDR			(APB1PERPH_BASE + 0x6400)
#define PWR_BASEADDR			(APB1PERPH_BASE + 0x7000)

/*
 * Define Base address of peripherals which are hanging on APB2 Bus
 * 84MHz Max
 * Peripherals : TIM1, TIM8-11, USART1, USART6, SPI1, ADC1-3, EXTI, SDIO
*/
#define TIM1_BASEADDR			(APB2PERPH_BASE + 0x0000)
#define TIM8_BASEADDR			(APB2PERPH_BASE + 0x0400)
#define USART1_BASEADDR			(APB2PERPH_BASE + 0x1000)
#define ADC1_TO_3_BASEADDR		(APB2PERPH_BASE + 0x2000)
#define SDIO_BASEADDR			(APB2PERPH_BASE + 0x2C00)
#define SPI1_BASEADDR			(APB2PERPH_BASE + 0x3000)
#define SYSCFG_BASEADDR			(APB2PERPH_BASE + 0x3800)
#define EXTI_BASEADDR			(APB2PERPH_BASE + 0x3C00)
#define LCD_TFT_BASEADDR		(APB2PERPH_BASE + 0x6800)


/*
 * Cortex M4 related Peripherals
 * Private Peripherals
 * */
#define NVIC_BASE				0xE000E100UL

/*
 * GPIO related structure
 * */
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

/*
 * I2C register structure
 * */
typedef struct
{
	uint16_t CR1;
	uint16_t Reserved1;
	uint16_t CR2;
	uint16_t Reserved2;
	uint16_t OAR1;
	uint16_t Reserved3;
	uint16_t OAR2;
	uint16_t Reserved4;
	uint16_t DR;
	uint16_t Reserved5;
	uint16_t SR1;
	uint16_t Reserved6;
	uint16_t SR2;
	uint16_t Reserved7;
	uint16_t CCR;
	uint16_t Reserved8;
	uint16_t TRISE;
	uint16_t Reserved9;
	uint16_t FLTR;
	uint16_t Reserved10;
}I2C_RegDef_t;


/*
 * SPI register structure
 * */
typedef struct
{
	uint16_t CR1;
	uint16_t Reserved1;
	uint16_t CR2;
	uint16_t Reserved2;
	uint16_t SR;
	uint16_t Reserved3;
	uint16_t DR;
	uint16_t Reserved4;
	uint16_t CRCPR;
	uint16_t Reserved5;
	uint16_t RXCRCR;
	uint16_t Reserved6;
	uint16_t TXCRCR;
	uint16_t Reserved7;
	uint16_t I2SCFGR;
	uint16_t Reserved8;
	uint16_t I2SPR;
	uint16_t Reserved9;
}SPI_RegDef_t;


/*
 * RCC register structure
 * */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
		 uint32_t Reserved1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
		 uint32_t Reserved2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
		 uint32_t Reserved3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
		 uint32_t Reserved4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
		 uint32_t Reserved5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
		 uint32_t Reserved6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	     uint32_t Reserved7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;


/*
 * EXTI related structure
 * */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RSTR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;




/*Peripheral Access through registers*/
#define GPIOA					((GPIO_RegDef_t* )GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t* )GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t* )GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t* )GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t* )GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t* )GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t* )GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t* )GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t* )RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t* )EXTI_BASEADDR)



/**************************************************************************************************************************/


//Offsets of Peripherals


#define NVIC_ISER_OFSET			0x00UL

#define	NVIC_ISER0

/**************************************************************************************************************************/

/*
 * Macros for GPIOx Clock enable
 * */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1<<7))


/*
 * Macros for GPIOx Clock disable
 * */
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<7))




// Need To update start///////////////////////

/*
 * Macros for I2Cx Clock enable
 * */
#define I2Cx_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))

/*
 * Macros for I2Cx Clock disable
 * */
#define I2Cx_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))



/*
 * Macros for SPIx Clock enable
 * */
#define SPIx_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))

/*
 * Macros for SPIx Clock disable
 * */
#define SPIx_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))


/*
 * Macros for UARTx Clock enable
 * */
#define UARTx_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))

/*
 * Macros for UARTx Clock disable
 * */
#define UARTx_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))

/*
 * Macros for SYSCFG Clock enable
 * */
#define SYSCFG_PCLK_EN()			(RCC->AHB1ENR |= (1<<0))

/*
 * Macros for SYSCFG Clock disable
 * */
#define SYSCFG_PCLK_DI()			(RCC->AHB1ENR &= ~(1<<0))


// Need To update end///////////////////////

//GPIOx Peripheral Reset
#define GPIOA_REG_RESET()			do{RCC->AHB1RSTR |= (1<<0U); RCC->AHB1RSTR &= ~(1<<0U);}while(0)
#define GPIOB_REG_RESET()			do{RCC->AHB1RSTR |= (1<<1U); RCC->AHB1RSTR &= ~(1<<1U);}while(0)
#define GPIOC_REG_RESET()			do{RCC->AHB1RSTR |= (1<<2U); RCC->AHB1RSTR &= ~(1<<2U);}while(0)
#define GPIOD_REG_RESET()			do{RCC->AHB1RSTR |= (1<<3U); RCC->AHB1RSTR &= ~(1<<3U);}while(0)
#define GPIOE_REG_RESET()			do{RCC->AHB1RSTR |= (1<<4U); RCC->AHB1RSTR &= ~(1<<4U);}while(0)
#define GPIOF_REG_RESET()			do{RCC->AHB1RSTR |= (1<<5U); RCC->AHB1RSTR &= ~(1<<5U);}while(0)
#define GPIOG_REG_RESET()			do{RCC->AHB1RSTR |= (1<<6U); RCC->AHB1RSTR &= ~(1<<6U);}while(0)
#define GPIOH_REG_RESET()			do{RCC->AHB1RSTR |= (1<<7U); RCC->AHB1RSTR &= ~(1<<7U);}while(0)


#define ENABLE						1U
#define DISABLE						0U
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#endif /* INC_STM32F4XX_H_ */
