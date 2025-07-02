/*
 * 005button_interrupt.c
 *
 *  Created on: May 3, 2025
 *      Author: poyu0
 */

#include "stm32f407_gpio_driver.h"
#include <string.h>
#include <stdio.h>

int main(){
	GPIO_Handle_t GPIOLed, GPIOBtn;

	memset(&GPIOLed,0,sizeof(GPIOLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);

	SYSCFG_PCLK_EN();
	SYSCFG->EXTICR[0] &= ~(0xF << 0);
	SYSCFG->EXTICR[0] |=  (0x0 << 0);

	//  Button
	GPIOBtn.pGPIOx = GPIOA;

	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	EXTI->RTSR |=  (1 << 0);
	EXTI->FTSR &= ~(1 << 0);
	EXTI->PR   |=  (1 << 0);
	EXTI->IMR  |=  (1 << 0);

	//GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
	// IRQ configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);

	printf("IRQ\r\n");

	while(1);


}

void EXTI0_IRQHandler(void){
	for(uint32_t i=0;i<1000000;i++);
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}

