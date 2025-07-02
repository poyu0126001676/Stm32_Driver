/*
 * 006_tx_testing.c
 *
 *  Created on: Apr 27, 2025
 *      Author: poyu0
 */

#include "stm32f407.h"
#include "stm32f407_gpio_driver.h"
#include "stm32f407_spi_driver.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*
 * 	Based on "Table 9. Alternate function mapping"
 *
 * 	PB14 -> MISO
 * 	PB15 -> MOSI
 * 	PB13 -> SCLK
 * 	PB12 -> NSS
 * 	ALT function mode : 5
 *
 * 	Initialize the GPIO pins to behave as SPI2 pins
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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;//generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

volatile uint8_t data = 0xAA;
int main(void)
{
	//char user_data[] = "Hello world";

	//uint8_t data = 0xAA;


	// 1. 開啟 GPIOB 時鐘，設定 PB13/SCK, PB15/MOSI
	GPIO_PeriClockControl(GPIOB, ENABLE);
	SPI2_GPIOInits();

	// 2. 初始化 SPI2
	SPI2_Inits();
	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, &data, 1);

    // 1) 等到 TXE = 1（資料已移進 Shift Register）
    while( !(SPI2->SR & SPI_SR_TXE) );

    // 2) 再等到 BSY = 0（真正完成傳輸、總線空閒）
    //while(  SPI2->SR & SPI_SR_BSY );
	printf("Already send \r\n");
	while (1);

}
