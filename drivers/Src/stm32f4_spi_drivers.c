/*
 * stm32f4_spi_drivers.c
 *
 *  Created on: Feb 14, 2020
 *      Author: Kevin
 */

#include "stm32f4_spi_drivers.h"


/*---------------------------SPI API's------------------------------------
 * ------------------------------------------------------------------------*/

//Enable SPI Port
/* @func	 - SPI_Init
 * @brief    - Sets SPI port configuration and initializes it
 * @param1   - *pSPIHandle - pointer to SPI handle struct that contains SPI port base address and configuration struct see: stm32f4_spi_drivers.h (SPI_Handle_t)
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_Disable(pSPIHandle->pSPIx);//Using disable to clear CR1 & CR2 control register values


	/*-----------CR1 Configuration--------------*/
	uint16_t temp = 0;//Temporary unsigned int for CR1 register value

	//set Bidirectional data mode
	temp |= (pSPIHandle->SPIConfig.BIDIMODE<<15);//Shifts value to BIDIMODE configuration bit field

	//set Output enable in bidirectional
	temp |= (pSPIHandle->SPIConfig.BIDIOE<<14);//Shifts value to BIDIOE configuration bit field

	//set Data frame format
	temp |= (pSPIHandle->SPIConfig.DFF<<11);//Shifts value to DFF configuration bit field

	//set Receive only
	temp |= (pSPIHandle->SPIConfig.RXONLY<<10);//Shifts value to RXONLY configuration bit field

	//set Software slave management
	temp |= (pSPIHandle->SPIConfig.SSM<<9);//Shifts value to SSM configuration bit field

	//set Internal slave select *Note: Must be set to 1 if SPI is set to master
	temp |= (pSPIHandle->SPIConfig.SSI<<8);//Shifts value to SSI configuration bit field

	//set Frame format
	temp |= (pSPIHandle->SPIConfig.LSBFIRST<<7);//Shifts value to LSBFIRST configuration bit field

	//set SPI enable
	temp |= (pSPIHandle->SPIConfig.SPE<<6);//Shifts value to SPE configuration bit field

	//set Baud rate control
	temp |= (pSPIHandle->SPIConfig.BR<<3);//Shifts value to BR configuration bit field

	//set Master/Slave mode selection
	temp |= (pSPIHandle->SPIConfig.MSTR<<2);//Shifts value to MSTR configuration bit field

	//set Clock polarity
	temp |= (pSPIHandle->SPIConfig.CPOL<<1);//Shifts value to CPOL configuration bit field

	//set Clock phase
	temp |= (pSPIHandle->SPIConfig.CPHA<<0);//Shifts value to CPHA configuration bit field

	/*------------CR2 Configuration-----------------*/
	uint16_t temp2 = 0;//Temporary unsigned int for CR2 register value

	//set Tx buffer empty interrupt
	temp2 |= (pSPIHandle->SPIConfig.TXEIE<<7);

	//set RX buffer not empty interrupt
	temp2 |= (pSPIHandle->SPIConfig.RXNEIE<<6);

	//set Error interrupt
	temp2 |= (pSPIHandle->SPIConfig.ERRIE<<5);

	//set Frame format
	temp2 |= (pSPIHandle->SPIConfig.FRF<<4);

	//set SS output enable
	temp2 |= (pSPIHandle->SPIConfig.SSOE<<3);

	//set Tx buffer DMA
	temp2 |= (pSPIHandle->SPIConfig.TXDMAEN<<2);

	//set Rx buffer DMA
	temp2 |= (pSPIHandle->SPIConfig.RXDMAEN<<2);

	/*----------Pass configurations to SPI control registers-----------*/

	//Pass Configuration to (SPI_CR1) Register
	pSPIHandle->pSPIx->SPI_CR1 = temp;//Sets CR1 register to value of temp

	//Pass Configuration to (SPI_CR2) Register
	pSPIHandle->pSPIx->SPI_CR2 = temp2;//Sets CR2 register to value of temp2



}

//Disable SPI Port
/* @func	 - SPI_Disable
 * @brief    - Disables SPI port by clearing all bits in configuration register
 * @param1   - *pSPIx - pointer to SPI port base address
 * */
void SPI_Disable(SPI_RegDef_t *pSPIx){
	//Blocking loops to prevent disabling communication during transmission
	while(!(pSPIx->SPI_SR & 2));//Waits for Tx buffer to be empty
	while((pSPIx->SPI_SR & 1));//Waits for Rx buffer to be empty
	while((pSPIx->SPI_SR & 128));//Waits for Busy flag

	pSPIx->SPI_CR1 &= ~(0X1111111111111111);//clears all bits in SPI_CR1 register
	pSPIx->SPI_CR2 &= ~(0X1111111111111111);//clears all configuration bits in SPI_CR2 register
}


//Enable SPI I2S Port
/* @func	 - SPI_I2S_Init
 * @brief    - Sets SPI I2S port configuration and initializes it
 * @param1   - *pI2SHandle - pointer to SPI I2S handle struct that contains SPI port base address and configuration struct see: stm32f4_spi_drivers.h (SPI_I2S_Handle_t)
 * */
void SPI_I2S_Init(SPI_I2S_Handle_t *pI2SHandle){
	SPI_Disable(pI2SHandle->pSPIx);
	SPI_I2S_Disable(pI2SHandle->pSPIx);
	uint16_t temp = 0;

	/*-------Configure SPI_I2SCFGR-----*/
	//set I2S mode selection
	temp |= (pI2SHandle->I2SConfig.I2SMOD<<11);//Shifts value to I2SMOD configuration bit field

	//set I2S Enable
	temp |= (pI2SHandle->I2SConfig.I2SE<<10);//Shifts value to I2SE configuration bit field

	//set I2S configuration mode
	temp |= (pI2SHandle->I2SConfig.I2SCFG<<8);//Shifts value to I2SCFG configuration bit field

	//set PCM frame synchronization
	temp |= (pI2SHandle->I2SConfig.PCMSYNC<<7);//Shifts value to PCMSYNC configuration bit field

	//set I2S standard selection
	temp |= (pI2SHandle->I2SConfig.I2SSTD<<4);//Shifts value to I2SSTD configuration bit field

	//set Steady state clock polarity
	temp |= (pI2SHandle->I2SConfig.CKPOL<<3);//Shifts value to CKPOL configuration bit field

	//set Data length to be transferred
	temp |= (pI2SHandle->I2SConfig.DATLEN<<1);//Shifts value to DATLEN configuration bit field

	//set Channel length (number of bits per audio channel)
	temp |= (pI2SHandle->I2SConfig.CHLEN<<0);//Shifts value to CHLEN configuration bit field. Only has meaning if DATLEN = 0

	//Pass Configuration to SPI_I2SCFGR register
	pI2SHandle->pSPIx->SPI_I2SCFGR |= temp;//Sets SPI_I2SCFGR register to value of temp. Uses OR operator to avoid changing value of reserved registers.
	temp = 0;

	/*-------Configure SPI_I2SPR-----*/
	//set Master clock output enable
	temp |= (pI2SHandle->I2SPrescaler.MCKOE<<9);//Shifts value to MCKOE configuration bit field

	//set Odd factor for the prescaler
	temp |= (pI2SHandle->I2SPrescaler.ODD<<8);//Shifts value to ODD configuration bit field

	//set I2S Linear prescaler
	temp |= (pI2SHandle->I2SPrescaler.I2SDIV<<0);//Shifts value to I2SDIV configuration bit field

	//Pass Configuration to SPI_I2SPR register
	pI2SHandle->pSPIx->SPI_I2SPR |= temp;//Sets SPI_I2SPR register to value of temp. Uses OR operator to avoid changing value of reserved registers.


	/*--------------------Configure RCC_PLLI2SCFGR-------------------*/
	uint32_t temp2  = 0;
	temp2 |= (pI2SHandle->I2SRCCPLL.PLLI2SR<<28);
	temp2 |= (pI2SHandle->I2SRCCPLL.PLLI2SN<<6);
	RCC->PLLI2SCFGR |= temp2;

}

//Disable SPI I2s Port
/* @func	 - SPI_I2S_Disable
 * @brief    - Disables SPI port by clearing all bits in configuration and prescaler registers
 * @param1   - *pSPIx - pointer to SPI port base address
 * */
void SPI_I2S_Disable(SPI_RegDef_t *pSPIx){
	pSPIx->SPI_I2SCFGR &= ~(0x1111111111111111);//clears all bits
	pSPIx->SPI_I2SPR &= ~(0x1111111111111111);//clears all bits
}


//Peripheral Clock Setup
/* @func	- SPI_PCLK_Control
 * @brief	- Enables or disables SPI peripheral clock for given address
 * @param1	- *pSPIx - pointer to SPI address
 * @param2  - EnableDisable - binary enable[1] or disable[0]
 * */
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnableDisable){
if(EnableDisable == 1){
		switch((uintptr_t)pSPIx){
		case SPI1_BASE:
			SPI1_PCLK_ENABLE();
			break;
		case SPI2_BASE:
			SPI2_PCLK_ENABLE();
			break;
		case SPI3_BASE:
			SPI3_PCLK_ENABLE();
			break;
		}
	}
	if(EnableDisable == 0){
		switch((uintptr_t)pSPIx){
		case SPI1_BASE:
			SPI1_PCLK_DISABLE();
			break;
		case SPI2_BASE:
			SPI2_PCLK_DISABLE();
			break;
		case SPI3_BASE:
			SPI3_PCLK_DISABLE();
			break;
	}

 }
}

//SPI Send & Receive
/* @func	- SPI_Send
 * @brief	- Sends output to the data register
 * @param1	- *pSPIx - pointer to SPI address
 * @param2  - *pTxBuffer - pointer the output data location
 * @param3  - length - size of output data in bytes
 * */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length){


	while(length > 0){

		while(!(pSPIx->SPI_SR & (1<<1)));//Waiting for TX buffer to be empty

		if(pSPIx->SPI_CR1 & (1<<11)){//If data frame size is 16 bit
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);//casting dereferenced data to 16 bits
			length-=2;//decrementing 2 for 2 bytes
			(uint16_t*)pTxBuffer++;
		}
		else{//data frame size is 8 bit
			pSPIx->SPI_DR = *pTxBuffer;
			length--;//decrementing 1 for 1 byte
			pTxBuffer++;
		}
	}
}


//Left off here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
uint16_t SPI_Receive(uint8_t *pRxBuffer, uint32_t length);

//IRQ Config & ISR Handling
void SPI_IRQ_EnableDisable(uint8_t IRQNumber, uint8_t EnableDisable);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
