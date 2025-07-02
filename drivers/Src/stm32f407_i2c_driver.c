/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: May 5, 2025
 *      Author: poyu0
 */

#include "stm32f407.h"
#include "stm32f407_i2c_driver.h"
#include <stdio.h>



void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}

}




/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);










