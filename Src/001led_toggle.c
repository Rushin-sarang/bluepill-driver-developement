/*
 * 001led_toggle.c
 *
 *  Created on: 04-Feb-2024
 *      Author: admin
 */

#include "stm32f103xx.h"

void delay (void){
	for (uint32_t i = 0; i < 500000; i++);
}

int main (void){
	GPIO_Handle_t Ledpin;

	Ledpin.pGPIOx = GPIOC;
	Ledpin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	Ledpin.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_OU_SPEED_FAST;
	Ledpin.GPIO_PinConfig.GPIO_PinConfig = GPIO_PIN_OU_PP;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&Ledpin);

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}
	return 0;
}
