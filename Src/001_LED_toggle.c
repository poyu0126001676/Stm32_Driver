/*
 * 001_LED_toggle.c
 *
 *  Created on: Apr 13, 2025
 *      Author: poyu0
 */

#include "stm32f407_gpio_driver.h"

int main(){
	GPIO_Handle_t GPIOLed;

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
		for(uint32_t i=0;i< 1000000;i++);
	}
	return 0;


}

