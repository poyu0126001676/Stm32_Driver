/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Apr 12, 2025
 *      Author: poyu0
 */
#include "stm32f407_gpio_driver.h"
#include "stm32f407.h"

/*
 *   Initialize and De-Initialize
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of GPIN pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 3){
		//  Non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  //  PinMode is two-bit size
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//  Clearing the bits first
		pGPIOHandle->pGPIOx->MODER |= temp;			// only change specific bits, that is why use "OR" operation
	}
	else{
		//  Interrupt Mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);   //  Clear the corresponding RTSR bit
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);   //  Clear the corresponding FTSR bit
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// 1. configure the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
		}
		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode % 4;

		// get which GPIO
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		// enable peripheral clock
		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);


		// 3. enable the exit interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinMode);
	}
	temp = 0;

	// 2. configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//  Clearing the bits first
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. configure the pull-up or pull-down settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//  Clearing the bits first
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. configure the output type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//  Clearing the bits first
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// 5. configure the alternative functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2));		//  Clearing the bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);




	}
}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}
/*
 *   Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 *   Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & (1 << 0));
	return value;    //  value is 0 or 1
}



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;    //  value is 0 or 1
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);		//  Write 1 to the output data register at the bit field corresponding to the pin number
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);		//  Write 0
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}




/*
 *   IRQ configuration and Handling
 *   Note : This is in processor side ( NVIC )
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		//	Use "Interrupt Set-Enable Register"
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64) );
		}

	}
	else{
		//	Use "Interrupt Set-Enable Register"
		if(IRQNumber <= 31){
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64) );
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRx_section) + (8 - NVIC_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + IPRx) |=  (IRQPriority << shift_amount) ;
}




void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}







