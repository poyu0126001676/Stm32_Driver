/*
 * 002_LED_Button.c
 *
 *  Created on: Apr 14, 2025
 *      Author: poyu0
 */

#include "stm32f407_gpio_driver.h"

void delay(){
	for(uint32_t i=0;i<5000000;i++);
}

int main(){
	GPIO_Handle_t GPIOLed, GPIOBtn;

	//  LED
	GPIOLed.pGPIOx = GPIOD;

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;    //  PA12
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);

	//  Button
	GPIOBtn.pGPIOx = GPIOA;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;    //  PA0
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == GPIO_PIN_SET){
			delay();    //  Avoid de-bounce of the button
			while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == GPIO_PIN_SET);
			GPIO_ToggleOutputPin(GPIOD, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
	return 0;


}


