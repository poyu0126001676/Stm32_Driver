/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Apr 23, 2025
 *      Author: poyu0
 */

#include "stm32f407_spi_driver.h"
#include "stm32f407.h"

/*
 *   Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

/*
 *   Initialize and De-Initialize
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	// enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//  configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//  2 lines -> Bidi mode should be cleared
		tempreg &= ~(1 << 15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//  Bidi mode should be set
		tempreg |= (1 << 15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//  BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		//  RXONLY bit must be set
		tempreg |= (1 << 10);
	}

	// 3. configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx);


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		// 1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);   //  TX is empty

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF)){  //  16-bit
			//  load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len-=2;
			(uint16_t*)pTxBuffer++;
		}
		else{								//  8-bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);     // Bit = 1 -> peripheral enabled
	}
	else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);     // Bit = 0 -> peripheral disabled
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		/*
		 * 1 . Save the Tx buffer address and Len information in some global variables
		 */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/*
		 * 2.  SPI state as busy in transmission,
		   so no other code can take over same SPI peripheral until transmission is over
		 */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)    // Not busy -> free or ready
	{
		/*
		 * 1 . Save the Rx buffer address and Len information in some global variables
		 */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		/*
		 * 2.  SPI state as busy in transmission,
		   so no other code can take over same SPI peripheral until transmission is over
		 */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;

}

void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//  check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){  //  16-bit
		//  load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen-=2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else{								//  8-bit
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}


	if(! pSPIHandle->TxLen){
		//  Close the spi transmission and inform the application that Tx is over.
		SPI_CloseTransmisson(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	//  check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF)){  //  16-bit
		//  load the data into the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t*)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;
		pSPIHandle->pRxBuffer-=2;
	}
	else{								//  8-bit
		*(pSPIHandle->pTxBuffer) = (uint8_t*)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}


	if(! pSPIHandle->RxLen){
		//  reception is complete, turn off the RENEIE interrupt
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}



void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// clear the OVR flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
	}
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_IRQHandling(SPI_Handle_t *pHandle){
	uint8_t temp1, temp2;


	/*
	 * first lets check for TXE
	 */
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	/*
	 *  check for RXNE
	 */
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	/*
	 * check for overrun flag
	 */
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//	weak implementation.
	//	This may override this function
}












