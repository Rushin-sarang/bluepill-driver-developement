/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jan 26, 2024
 *      Author: admin
 */

#include "stm32f103xx_gpio_driver.h"

/*
 *  Peripheral clock setup
 */

/***************************************************************************
 * @fn						-	GPIO_PeriClockControl
 *
 * @brief					-	This function enables or disables peripheral clock for the given gpio port
 *
 * @param[in]				-	base address of gpio peripheral
 * @param[in]      			-	ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					-	void
 *
 * @note					-	none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EorD){
	if(EorD == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC){
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD){
					GPIOD_PCLK_DI();
				}else if(pGPIOx == GPIOE){
					GPIOE_PCLK_DI();
				}else if(pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
				}else if(pGPIOx == GPIOG){
					GPIOG_PCLK_DI();
				}
	}
}

/*
 *  Init and Deinit
 */
/***************************************************************************
 * @fn						-	GPIO_Init
 *
 * @brief					-	This function configures given gpio pin
 *
 * @param[in]				-	handle structure for gpio pin
 * @param[in]      			-
 * @param[in]				-
 *
 * @return					-	void
 *
 * @note					-	none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOhandle){
	uint32_t temp = 0;
	if(pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber < 8){
		temp = ( ((pGPIOhandle->GPIO_PinConfig.GPIO_PinMode) + (pGPIOhandle->GPIO_PinConfig.GPIO_PinConfig)) << (4 * pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOhandle->pGPIOx->CRL |= temp;
		temp=0;
	}else{
		temp = ( ((pGPIOhandle->GPIO_PinConfig.GPIO_PinMode) + (pGPIOhandle->GPIO_PinConfig.GPIO_PinConfig)) << (4 * ((pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber) - 8)) );
		pGPIOhandle->pGPIOx->CRH |= temp;
		temp=0;
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}else if(pGPIOx == GPIOC){
				GPIOC_REG_RESET();
			}else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}else if(pGPIOx == GPIOE){
				GPIOE_REG_RESET();
			}else if(pGPIOx == GPIOF){
				GPIOF_REG_RESET();
			}else if(pGPIOx == GPIOG){
				GPIOG_REG_RESET();
			}
}

/*
 *   Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;

	value = (uint8_t) (((pGPIOx->IDR) >> PinNumber) & 0x00000001);

	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;

	value = (uint16_t) (pGPIOx->IDR);

	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *  IRQ Configuration and ISR Handling
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EorD);
void GPIO_IRQHandling(uint8_t PinNumber);
