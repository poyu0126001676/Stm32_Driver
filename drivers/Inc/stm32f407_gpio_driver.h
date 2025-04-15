/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Apr 12, 2025
 *      Author: poyu0
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f407.h"

typedef struct{
	uint8_t GPIO_PinNumber;			// possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;		//
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;



typedef struct{
	/*
	 *      Hold the base address and the configuration settings of the GPIO peripheral
	 */
	GPIO_RegDef_t *pGPIOx;     //  This holds the base address of the GPIO port to which pin belongs x (A, B, C....)
	GPIO_PinConfig_t GPIO_PinConfig;   //  This holds GPIO pin configuration settings
}GPIO_Handle_t;



/*
 *     Prototypes for the APIs
 */

/*
 *   Initialize and De-Initialize
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 *   Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 *   Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *   IRQ configuration and Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);



/*
 *	 @GPIO_PIN_NUMBERS
 *   GPIO pin numbers
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





/*
 *	 @GPIO_PIN_MODES
 *   GPIO pin possible modes
 */

#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4		//  interrupt input falling edge
#define GPIO_MODE_IT_RT			5		//  interrupt input rising edge
#define GPIO_MODE_IT_RFT		6		//  interrupt rising, falling trigger

/*
 *   GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 *   @GPIO_PIN_SPEED
 *   GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 *   GPIO pin pull-up and pull-down configuration
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2













#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
