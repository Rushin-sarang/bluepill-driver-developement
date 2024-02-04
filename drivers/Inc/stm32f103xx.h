/*
 * stm32f103xx.h
 *
 *  Created on: Jan 26, 2024
 *      Author: rushin
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>

#define __vo volatile
/*
 * 	base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR												0x08000000U
#define SRAM1_BASEADDR												0x20000000U
#define SRAM 														SRAM1_BASEADDR
#define ROM_BASEADDR											    0x1FFFF000U

/*
 *  AHB and APBx Bus Peripherals base address
 */

#define PERIPH_BASEADDR 											0x40000000U
#define APB1PERIPH_BASEADDR											PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR											0x40010000U
#define AHBPERIPH_BASEADDR											0x40018000U

/*
 *  Base addresses of peripherals which are hanging on APB2 bus
 */

#define RCC_BASEADDR												(AHBPERIPH_BASEADDR + 0x9000)

/*
 *  Base addresses of peripherals which are hanging on APB2 bus
 */

#define GPIOA_BASEADDR												(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR												(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR												(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR												(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR												(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR												(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR												(APB2PERIPH_BASEADDR + 0x2000)

#define EXTI_BASEADDR												(APB2PERIPH_BASEADDR + 0x0400)

#define SPI1_BASEADDR												(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR												(APB2PERIPH_BASEADDR + 0x3800)

/*
 *  Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR												(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR												(APB1PERIPH_BASEADDR + 0x5800)

#define USART2_BASEADDR												(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR												(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR												(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR												(APB1PERIPH_BASEADDR + 0x5000)

#define SPI2_BASEADDR												(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR												(APB1PERIPH_BASEADDR + 0x3C00)

/***************************************peripheral register definition structures****************************************************/

typedef struct
{
	__vo uint32_t CRL;    /* Port configuration low register , offset : 0x00 */
	__vo uint32_t CRH;	  /* Port configuration high register , offset : 0x04 */
	__vo uint32_t IDR;    /* Port input data register, offset : 0x08 */
	__vo uint32_t ODR;    /* Port output data register, offset : 0x0C */
	__vo uint32_t BSRR;   /* Port bit set/reset register, offset : 0x10 */
	__vo uint32_t BRR;    /* Port bit reset register, offset : 0x14 */
	__vo uint32_t LCKR;   /* Port configuration lock register, offset : 0x18 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;		 /* Clock control register, offset : 0x00 */
	__vo uint32_t CFGR;      /* Clock configuration register , offset : 0x04 */
	__vo uint32_t CIR;       /* Clock interrupt register , offset : 0x08 */
	__vo uint32_t APB2RSTR;  /* APB2 peripheral reset register , offset : 0x0C */
	__vo uint32_t APB1RSTR;  /* APB1 peripheral reset register , offset : 0x10 */
	__vo uint32_t AHBENR;    /* AHB peripheral clock enable register , offset : 0x14 */
	__vo uint32_t APB2ENR;   /* APB2 peripheral clock enable register , offset : 0x18 */
	__vo uint32_t APB1ENR;   /* APB1 peripheral clock enable register , offset : 0x1C */
	__vo uint32_t BDCR;      /* Backup domain control register , offset : 0x20 */
	__vo uint32_t CSR;       /* Control/status register , offset : 0x24 */
}RCC_RegDef_t;

/*
 *  peripheral definitions ( peripherals base address typecasted to xxx_RegDef_t)
 */

#define GPIOA 														((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 														((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 														((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 														((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 														((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 														((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 														((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC															((RCC_RegDef_t*)RCC_BASEADDR)

/*
 *  Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() 											( RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN() 											( RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN() 											( RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN() 											( RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN() 											( RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN() 											( RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN() 											( RCC->APB2ENR |= (1 << 8))

/*
 *  Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()												( RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()												( RCC->APB1ENR |= (1 << 22))

/*
 *  Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()												( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()												( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()												( RCC->APB1ENR |= (1 << 15))

/*
 *  Clock Enable Macros for USARTx peripherals
 */

#define USART1_PLCK_EN()											( RCC->APB2ENR |= (1 << 14))
#define USART2_PLCK_EN()											( RCC->APB1ENR |= (1 << 17))
#define USART3_PLCK_EN()											( RCC->APB1ENR |= (1 << 18))
#define UART4_PLCK_EN()												( RCC->APB1ENR |= (1 << 19))
#define UART5_PLCK_EN()												( RCC->APB1ENR |= (1 << 20))

/*
 *  Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()												( RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI() 											( RCC->APB2ENR &= ~(1 << 8))

/*
 *  Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()												( RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()												( RCC->APB1ENR &= ~(1 << 22))

/*
 *  Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()												( RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()												( RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()												( RCC->APB1ENR &= ~(1 << 15))

/*
 *  Clock Disable Macros for USARTx peripherals
 */

#define USART1_PLCK_DI()											( RCC->APB2ENR &= ~(1 << 14))
#define USART2_PLCK_DI()											( RCC->APB1ENR &= ~(1 << 17))
#define USART3_PLCK_DI()											( RCC->APB1ENR &= ~(1 << 18))
#define UART4_PLCK_DI()												( RCC->APB1ENR &= ~(1 << 19))
#define UART5_PLCK_DI()												( RCC->APB1ENR &= ~(1 << 20))
/*
 *	GPIO Reset MACROS
 */

#define GPIOA_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOB_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));}while(0)
#define GPIOC_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define GPIOD_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));}while(0)
#define GPIOE_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));}while(0)
#define GPIOF_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7));}while(0)
#define GPIOG_REG_RESET()											do{ (RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8));}while(0)

/*
 *  Additional MACROS
 */

#define ENABLE 														1
#define DISABLE 													0
#define SET															ENABLE
#define RESET														DISABLE
#define GPIO_PIN_SET												SET
#define GPIO_PIN_RESET												RESET

#include "stm32f103xx_gpio_driver.h"

#endif /* INC_STM32F103XX_H_ */
