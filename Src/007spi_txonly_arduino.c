#include <string.h>
#include "stm32f407.h"
#include "stm32f407_gpio_driver.h"
#include "stm32f407_spi_driver.h"


void delay(void)
{
	for(uint32_t i = 0 ; i < 10000000 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

/* SPI Slave Demo

 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *

 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);


	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

void SPI2_CS_GPIO_Init(void) {
    GPIO_Handle_t CSPin;

    CSPin.pGPIOx = GPIOB;
    CSPin.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_12;
    CSPin.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_OUT;
    CSPin.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;
    CSPin.GPIO_PinConfig.GPIO_PinOPType    = GPIO_OP_TYPE_PP;
    CSPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&CSPin);
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_SET); // 初始為高（未選中）
}


int main(void)
{
	char user_data[] = "Hello";
	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	SPI2_CS_GPIO_Init();
	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_RESET);

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// enable SPI
		SPI_PeripheralControl(SPI2, ENABLE);

		// 先送一個同步 byte，提醒 Arduino 準備好（值任意，只要雙方認同）
		uint8_t sync = 0xAA;
		SPI_SendData(SPI2, &sync, 1);
		delay();  // 讓 Arduino 接收完 sync

		// 先送長度（1 byte）
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// 再送實際資料
		SPI_SendData(SPI2, (uint8_t*)user_data, dataLen);
		delay();

		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, GPIO_PIN_SET);
	}

	return 0;

}
