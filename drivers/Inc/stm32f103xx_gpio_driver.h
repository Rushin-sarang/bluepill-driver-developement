/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jan 26, 2024
 *      Author: admin
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"
/*
 *  this is Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinConfig;
}GPIO_PinConfig_t;

/*
 *  this is Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;            /* this holds the base address of GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;  /* this holds GPIO pin configuration settings */
}GPIO_Handle_t;

/********************************************************************************************************************************
 *                                        APIs supported by this driver
 *                         For more informations about the APIs check the function definitions
 *******************************************************************************************************************************/

/*
 * 	GPIO pin numbers
 */

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

//#define GPIO_PIN_IT_FT			4
//#define GPIO_PIN_IT_RT			5
//#define GPIO_PIN_IT_RFT			6

/*
 *  possible GPIO PinModes
 */

#define GPIO_PIN_IN				0							//MODE[1:0] --> [ 0 , 0 ]
#define GPIO_PIN_OU_SPEED_LOW 	2							//MODE[1:0] --> [ 1 , 0 ]
#define GPIO_PIN_OU_SPEED_MED	1							//MODE[1:0] --> [ 0 , 1 ]
#define GPIO_PIN_OU_SPEED_FAST	3							//MODE[1:0] --> [ 1 , 1 ]

/*
 *  GPIO pin possible Configs
 */

// MODE[1:0] == [ 0 , 0 ]

#define GPIO_PIN_IN_ANALOG		GPIO_PIN_IN + 0				//CNF[1:0] --> [ 0 , 0 ], MODE[1:0] == [ 0 , 0 ]
#define GPIO_PIN_IN_FLOAT		GPIO_PIN_IN + 4				//CNF[1:0] --> [ 0 , 1 ], MODE[1:0] == [ 0 , 0 ]
#define GPIO_PIN_IN_PP			GPIO_PIN_IN + 8				//CNF[1:0] --> [ 1 , 0 ], MODE[1:0] == [ 0 , 0 ]

// MODE[1:0] > [ 0 , 0 ]

#define GPIO_PIN_OU_PP			0							//CNF[1:0] --> [ 0 , 0 ], MODE[1:0] > [ 0 , 0 ]
#define GPIO_PIN_OU_OD			4							//CNF[1:0] --> [ 0 , 1 ], MODE[1:0] > [ 0 , 0 ]
#define GPIO_PIN_ALTFN_PP 		8							//CNF[1:0] --> [ 1 , 0 ], MODE[1:0] > [ 0 , 0 ]
#define GPIO_PIN_ALTFN_OD		12							//CNF[1:0] --> [ 1 , 1 ], MODE[1:0] > [ 0 , 0 ]

/*
 *  Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EorD);

/*
 *  Init and Deinit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOhandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *   Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EorD);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
