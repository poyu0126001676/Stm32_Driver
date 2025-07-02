/*
 * stm32f407.h
 *
 *  Created on: Apr 9, 2025
 *      Author: poyu0
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

#include <stdint.h>
#include <stddef.h>

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses  ( Interrupt set-enable register)
 */

#define NVIC_ISER0          ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (volatile uint32_t*)0xE000E10c )

#define NVIC_PR_BASEADDR	( (volatile uint32_t*)0xE000E400 )

#define NVIC_PR_BITS_IMPLEMENTED		4

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses  ( Interrupt clear-enable register)
 */
#define NVIC_ICER0 			((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2  		((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0XE000E18C)


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
#define EXTI_BASEADDR			0x40013C00U
#define SYSCFG_BASEADDR			0x40013800U
#define SPI4_BASEADDR			0x40013400U


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

#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

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
 *  peripheral register definition for SPI
 */
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

#define SPI1			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*) SPI4_BASEADDR)

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	volatile uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	volatile uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	volatile uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
	volatile uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
	volatile uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
	volatile uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
	volatile uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
	volatile uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
	volatile uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
	volatile uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * peripheral register definition structure for USART
 */
typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)


/*****************************************************************************************
 *  			Bit position definitions of SPI peripheral
 *****************************************************************************************/

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/******************************************************************************************
 *			Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


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
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	volatile uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	volatile uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	volatile uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	volatile uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	volatile uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;




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

#define I2C1_PCLK_EN()		RCC->APB1ENR |= ( 1 << 21 )
#define I2C2_PCLK_EN()		RCC->APB1ENR |= ( 1 << 22 )
#define I2C3_PCLK_EN()		RCC->APB1ENR |= ( 1 << 23 )

#define SPI1_PCLK_EN()		RCC->APB2ENR |= ( 1 << 12 )
#define SPI2_PCLK_EN()		RCC->APB1ENR |= ( 1 << 14 )
#define SPI3_PCLK_EN()		RCC->APB1ENR |= ( 1 << 15 )
#define SPI4_PCLK_EN()		RCC->APB2ENR |= ( 1 << 13 )

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
#define GPIOA_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 0 )
#define GPIOB_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 1 )
#define GPIOC_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 2 )
#define GPIOD_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 3 )
#define GPIOE_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 4 )
#define GPIOF_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 5 )
#define GPIOG_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 6 )
#define GPIOH_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 7 )
#define GPIOI_PCLK_DI()		RCC->AHB1ENR &= ~( 1 << 8 )

#define I2C1_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 21 )
#define I2C2_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 22 )
#define I2C3_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 23 )

#define SPI1_PCLK_DI()		RCC->APB2ENR &= ~( 1 << 12 )
#define SPI2_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 14 )
#define SPI3_PCLK_DI()		RCC->APB1ENR &= ~( 1 << 15 )
#define SPI4_PCLK_DI()		RCC->APB2ENR &= ~( 1 << 13 )

#define USART1_PCCK_DI()    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCCK_DI()    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DI()    (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCCK_DI()     (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCCK_DI()     (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCCK_DI()    (RCC->APB2ENR &= ~(1 << 5))

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 14))

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


/*
 * 	returns a code (between 0~7) for a given GPIOx bas address
 */
#define GPIO_BASEADDR_TO_CODE(x)        ( (x == GPIOA) ? 0 : \
										  (x == GPIOB) ? 1 : \
											(x == GPIOC) ? 2 : \
											(x == GPIOD) ? 3 : \
											(x == GPIOE) ? 4 : \
											(x == GPIOF) ? 5 : \
											(x == GPIOG) ? 6 : \
											(x == GPIOH) ? 7 : \
											(x == GPIOI) ? 8 : 0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51

#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


// some generic macros

#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE

#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#define FLAG_RESET			RESET
#define FLAG_SET			SET






#endif /* INC_STM32F407_H_ */
