/*
 * stm32f407.h
 *
 *  Created on: Apr 9, 2025
 *      Author: poyu0
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

#include <stdint.h>


#define FLASH_BASEADDR			(uint32_t)0x08000000  // or 0x08000000U as unsigned
#define SRAM1_BASEADDR			(uint32_t)0x20000000  // 112KB
#define SRAM2_BASEADDR			(uint32_t)0x2001C0000  // 0x20000000 + hex(112*1024) = 0x20000000 + 0x0001C000
#define ROM_BASEADDR 			(uint32_t)0x1FFF0000   // ROM
#define SRAM 					SRAM1_BASEADDR

#define PERRIPH_BASE			0x40000000U
#define APB1PERIPH_BASE			PERRIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U


// AHB1
#define GPIOA_BASEADDR			0x40020000U
#define GPIOB_BASEADDR			0x40020400U
#define GPIOC_BASEADDR			0x40020800U
#define GPIOD_BASEADDR			0x40020C00U
#define GPIOE_BASEADDR			0x40021000U
#define GPIOF_BASEADDR			0x40021400U
#define GPIOG_BASEADDR			0x40021800U
#define GPIOH_BASEADDR			0x40021C00U
#define GPIOI_BASEADDR			0x40022000U

// APB1
#define I2C1_BASEADDR			0x40005400U
#define I2C2_BASEADDR			0x40005800U
#define I2C3_BASEADDR			0x40005C00U

#define SPI2_BASEADDR			0x40003800U
#define SPI3_BASEADDR			0x40003C00U

#define USART2_BASEADDR			0x40004400U
#define USART3_BASEADDR			0x40004800U
#define UART4_BASEADDR			0x40004C00U
#define UART5_BASEADDR			0x40005000U

// APB2
#define SPI1_BASEADDR			0x40013000U
#define USART1_BASEADDR			0x40011000U
#define USART6_BASEADDR			0x40011400U
#define EXT1_BASEADDR			0x40013C00U
#define SYSCFG_BASEADDR			0x40013800U


//  Registers of GPIO
//  The registers are updated for every AHB/APB clock -> Volatile keyword is useful.
typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;


//  Peripheral definitions
#define GPIOA			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*) GPIOI_BASEADDR)

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

#define RCC				((RCC_RegDef_t*) 0x40023800U)

/*
 * peripheral register definition structure for EXTI (External interrupt)
 */

typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

/*
 * Clock enable for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 0 )
#define GPIOB_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 1 )
#define GPIOC_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 2 )
#define GPIOD_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 3 )
#define GPIOE_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 4 )
#define GPIOF_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 5 )
#define GPIOG_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 6 )
#define GPIOH_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 7 )
#define GPIOI_PCLK_EN()		RCC->AHB1ENR |= ( 1 << 8 )

#define I2C1_PLCK_EN()		RCC->APB1ENR |= ( 1 << 21 )
#define I2C2_PLCK_EN()		RCC->APB1ENR |= ( 1 << 22 )
#define I2C3_PLCK_EN()		RCC->APB1ENR |= ( 1 << 23 )

#define SPI1_PLCK_EN()		RCC->APB2ENR |= ( 1 << 12 )
#define SPI2_PLCK_EN()		RCC->APB1ENR |= ( 1 << 14 )
#define SPI3_PLCK_EN()		RCC->APB1ENR |= ( 1 << 15 )
#define SPI4_PLCK_EN()		RCC->APB2ENR |= ( 1 << 13 )

#define USART1_PCCK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN()    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN()    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()     (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()     (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN()    (RCC->APB2ENR |= (1 << 5))

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 0 )
#define GPIOB_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 1 )
#define GPIOC_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 2 )
#define GPIOD_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 3 )
#define GPIOE_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 4 )
#define GPIOF_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 5 )
#define GPIOG_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 6 )
#define GPIOH_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 7 )
#define GPIOI_PCLK_DI()		RCC->AHB1ENR &= ( 1 << 8 )

#define I2C1_PLCK_DI()		RCC->APB1ENR &= ( 1 << 21 )
#define I2C2_PLCK_DI()		RCC->APB1ENR &= ( 1 << 22 )
#define I2C3_PLCK_DI()		RCC->APB1ENR &= ( 1 << 23 )

#define SPI1_PLCK_DI()		RCC->APB2ENR &= ( 1 << 12 )
#define SPI2_PLCK_DI()		RCC->APB1ENR &= ( 1 << 14 )
#define SPI3_PLCK_DI()		RCC->APB1ENR &= ( 1 << 15 )
#define SPI4_PLCK_DI()		RCC->APB2ENR &= ( 1 << 13 )

#define USART1_PCCK_DI()    (RCC->APB2ENR &= (1 << 4))
#define USART2_PCCK_DI()    (RCC->APB1ENR &= (1 << 17))
#define USART3_PCCK_DI()    (RCC->APB1ENR &= (1 << 18))
#define UART4_PCCK_DI()     (RCC->APB1ENR &= (1 << 19))
#define UART5_PCCK_DI()     (RCC->APB1ENR &= (1 << 20))
#define USART6_PCCK_DI()    (RCC->APB2ENR &= (1 << 5))

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= (1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 0 ));   (RCC->AHB1RSTR &= ~( 1 << 0 ));  }while(0)
#define GPIOB_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 1 ));   (RCC->AHB1RSTR &= ~( 1 << 1 ));  }while(0)
#define GPIOC_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 2 ));   (RCC->AHB1RSTR &= ~( 1 << 2 ));  }while(0)
#define GPIOD_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 3 ));   (RCC->AHB1RSTR &= ~( 1 << 3 ));  }while(0)
#define GPIOE_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 4 ));   (RCC->AHB1RSTR &= ~( 1 << 4 ));  }while(0)
#define GPIOF_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 5 ));   (RCC->AHB1RSTR &= ~( 1 << 5 ));  }while(0)
#define GPIOG_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 6 ));   (RCC->AHB1RSTR &= ~( 1 << 6 ));  }while(0)
#define GPIOH_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 7 ));   (RCC->AHB1RSTR &= ~( 1 << 7 ));  }while(0)
#define GPIOI_REG_RESET()			do{  (RCC->AHB1RSTR |= ( 1 << 8 ));   (RCC->AHB1RSTR &= ~( 1 << 8 ));  }while(0)


// some generic macros

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE

#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#define EXTI                ((EXTI_RegDef_t*)EXT1_BASEADDR)



#endif /* INC_STM32F407_H_ */
