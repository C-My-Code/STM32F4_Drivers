/*
 * stm32f4_spi_drivers.c
 *
 *  Last update on: Mar 15, 2020
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
	SPI_PCLK_Control(pSPIHandle->pSPIx, ENABLE);

	/*-----------CR1 Configuration--------------*/
	uint32_t temp = 0;//Temporary unsigned int for CR1 register value

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
	uint32_t temp2 = 0;//Temporary unsigned int for CR2 register value

	//set Error interrupt
	temp2 |= (pSPIHandle->SPIConfig.ERRIE<<5);

	//set Frame format
	temp2 |= (pSPIHandle->SPIConfig.FRF<<4);

	//set SS output enable
	temp2 |= (pSPIHandle->SPIConfig.SSOE<<2);

	/*----------Pass configurations to SPI control registers-----------*/
	//Pass Configuration to (SPI_CR1) Register
	pSPIHandle->pSPIx->CR1 = temp;//Sets CR1 register to value of temp

	//Pass Configuration to (SPI_CR2) Register
	pSPIHandle->pSPIx->CR2 |= temp2;//Sets CR2 register to value of temp2




}

//Disable SPI Port
/* @func	 - SPI_Disable
 * @brief    - Disables SPI port by clearing all bits in configuration register
 * @param1   - *pSPIx - pointer to SPI port base address
 * */
void SPI_Disable(SPI_RegDef_t *pSPIx){
	//Blocking loop to prevent disabling communication during transmission
	while((pSPIx->SR & 128));//Waits for Busy flag

	pSPIx->CR1 &= ~(0xFFFF);//clears all bits in SPI_CR1 register
	pSPIx->CR2 &= ~(0xFFFF);//clears all configuration bits in SPI_CR2 register
	SPI_PCLK_Control(pSPIx, DISABLE);

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
	SPI_PCLK_Control(pI2SHandle->pSPIx, ENABLE);
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
	pI2SHandle->pSPIx->I2SCFGR |= temp;//Sets SPI_I2SCFGR register to value of temp. Uses OR operator to avoid changing value of reserved registers.
	temp = 0;

	/*-------Configure SPI_I2SPR-----*/
	//set Master clock output enable
	temp |= (pI2SHandle->I2SPrescaler.MCKOE<<9);//Shifts value to MCKOE configuration bit field

	//set Odd factor for the prescaler
	temp |= (pI2SHandle->I2SPrescaler.ODD<<8);//Shifts value to ODD configuration bit field

	//set I2S Linear prescaler
	temp |= (pI2SHandle->I2SPrescaler.I2SDIV<<0);//Shifts value to I2SDIV configuration bit field

	//Pass Configuration to SPI_I2SPR register
	pI2SHandle->pSPIx->I2SPR |= temp;//Sets SPI_I2SPR register to value of temp. Uses OR operator to avoid changing value of reserved registers.


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
	pSPIx->I2SCFGR &= ~(0xFFFF);//clears all bits
	pSPIx->I2SPR &= ~(0xFFFF);//clears all bits
	SPI_PCLK_Control(pSPIx, DISABLE);
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

//SPI Send - Blocking Call(non-interrupt)
/* @func	- SPI_Send
 * @brief	- Sends output to the data register
 * @param1	- *pSPIHandle - pointer to SPI handle structure
 * @param2  - *pTxBuffer - pointer to location of data to be transmitted
 * @param3  - length - size of outbound data in bytes
 * */
void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length){

	while(length > 0){

		while(!(pSPIx->SR & 2));//Waiting for TX buffer to be empty(blocking call)

		if(pSPIx->CR1 & (1<<11)){//If data frame size is 16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);//casting dereferenced data to 16 bits
			length-=2;//decrementing 2 for 2 bytes
			(uint16_t*)pTxBuffer++;
		}
		else{//data frame size is 8 bit
			pSPIx->DR = *pTxBuffer;
			length--;//decrementing 1 for 1 byte
			pTxBuffer++;
		}
	}
}



//SPI Receive - Blocking Call(non-interrupt)
/* @func	- SPI_Receive
 * @brief	- Reads data from the data register
 * @param1	- *pSPIHandle - pointer to SPI handle structure
 * @param2  - *pRxBuffer - pointer to location received data is to be stored
 * @param3  - length - size of inbound data in bytes
 * */
void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length){

	while(length > 0){
		while(pSPIx->SR & 0x01);//Waiting for data to be in buffer(blocking call)
		if(pSPIx->CR1 & (1<<11)){
		*((uint16_t*)pRxBuffer) = pSPIx->DR;
		length-=2;
		(uint16_t*)pRxBuffer++;
		}
		else{
		*pRxBuffer = pSPIx->DR;
		length--;
		pRxBuffer++;
		}
	}
}

//SPI Send - Interrupt based
void SPI_Send_Ir(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length){

	if(!(pSPIHandle->TxState == 1)){//Checks for idle Tx
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = length;
	pSPIHandle->TxState = 1;//Sets TxState to busy
	pSPIHandle->pSPIx->CR2 |= (1<<7);//Enables TXEIE Interrupt
	}

}
//SPI Receive - Interrupt based
void SPI_Receive_Ir(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length){

	if(!(pSPIHandle->RxState == 1)){
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = length;
	pSPIHandle->RxState = 1;
	pSPIHandle->pSPIx->CR2 |= (1<<6);//Enables RXNEIE Interrupt
	}
}

//SPI Interrupt Enable & Disable
/* @func	- SPI_IRQ_EnableDisable
 * @brief	- Enables or disables SPI interrupt
 * @param1	- IRQNumber - IRQNumber for the SPI line to enable or disable. See stm32f429xx.h INTERRPUT REQUEST(IRQ) NUMBER MACROS
 * @param2	- EnableDisable - 1=ENABLE 0=DISABLE
 * */
void SPI_IRQ_EnableDisable(uint8_t IRQNumber, uint8_t EnableDisable){

	if(EnableDisable>0){
		if(IRQNumber<=31){//Determines if interrupt belongs to NVIC_ISER0
			*NVIC_ISER_BASE |= (1<<IRQNumber);
		}
		else if(IRQNumber>=32&&IRQNumber<=63){//Determines if interrupt belongs to NVIC_ISER1
			*NVIC_ISER_1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=64&&IRQNumber<=95){//Determines if interrupt belongs to NVIC_ISER2
			*NVIC_ISER_2 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=96&&IRQNumber<=127){//Determines if interrupt belongs to NVIC_ISER3
			*NVIC_ISER_3 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=128&&IRQNumber<=159){//Determines if interrupt belongs to NVIC_ISER4
			*NVIC_ISER_4 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=160&&IRQNumber<=191){//Determines if interrupt belongs to NVIC_ISER5
			*NVIC_ISER_5 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=192&&IRQNumber<=223){//Determines if interrupt belongs to NVIC_ISER6
			*NVIC_ISER_6 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=224&&IRQNumber<=239){//Determines if interrupt belongs to NVIC_ISER7
			*NVIC_ISER_7 |= (1<<(IRQNumber%32));
		}
	}
	else{
		if(IRQNumber<=31){//Determines if interrupt belongs to NVIC_ICER0
			*NVIC_ICER_BASE |= (1<<IRQNumber);
		}
		else if(IRQNumber>=32&&IRQNumber<=63){//Determines if interrupt belongs to NVIC_ICER1
			*NVIC_ICER_1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=64&&IRQNumber<=95){//Determines if interrupt belongs to NVIC_ICER2
			*NVIC_ICER_2 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=96&&IRQNumber<=127){//Determines if interrupt belongs to NVIC_ICER3
			*NVIC_ICER_3= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=128&&IRQNumber<=159){//Determines if interrupt belongs to NVIC_ICER4
			*NVIC_ICER_4 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=160&&IRQNumber<=191){//Determines if interrupt belongs to NVIC_ICER5
			*NVIC_ICER_5 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=192&&IRQNumber<=223){//Determines if interrupt belongs to NVIC_ICER6
			*NVIC_ICER_6 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=224&&IRQNumber<=239){//Determines if interrupt belongs to NVIC_ICER7
			*NVIC_ICER_7 |= (1<<(IRQNumber%32));
		}

	}
}
//SPI Interrupt Priority Configuration
/* @func	- SPI_IRQ_Priority_Config
 * @brief 	- Sets the priority for a given interrupt
 * @param1	- IRQNumber - interrupt number for the SPI line you intend to configure. See stm32f429xx.h "INTERRPUT REQUEST(IRQ) NUMBER MACROS"
 * @param2	- IRQPriority  - 4 bit priority value
 * */
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t IP_reg = IRQNumber/4;//Calculates which priority register address row the interrupt belongs to (0-59)
	uint8_t IP_Section = IRQNumber%4;//This determines which partition the IRQ# belongs in. Each address row is divided into 4 byte sized partitions, one byte for each IRQNumber.
	*(NVIC_ISPR_BASE+IP_reg) |= ((IRQPriority<<4)<<(IP_Section*8));//Shifts the value 4 bits past 0th partition bit(only bits 7:4 are read) and stores it in the correct priority register address row.
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	if((pSPIHandle->pSPIx->CR2 & (1<<7))&&(pSPIHandle->pSPIx->SR&(1<<1))){//If TxBuffer empty interrupt enabled & Tx buffer empty
		SPI_Tx_IR_Handle(pSPIHandle);
	}
	if((pSPIHandle->pSPIx->CR2 & (1<<6))&&(pSPIHandle->pSPIx->SR&(1<<0))){//If RxBuffer not empty interrupt enabled & Rx buffer not empty
		SPI_Rx_IR_Handle(pSPIHandle);
	}
	if((pSPIHandle->pSPIx->CR2 & (1<<5))&&(pSPIHandle->pSPIx->SR&(1<<6))){//If error interrupt enabled & overrun error flag active
		SPI_Ovr_IR_Handle(pSPIHandle);
	}

}

static void SPI_Tx_IR_Handle(SPI_Handle_t *pSPIHandle){

	while(pSPIHandle->TxLen > 0){
			if(pSPIHandle->pSPIx->CR1 & (1<<11)){//If data frame size is 16 bit
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);//casting dereferenced data to 16 bits
				pSPIHandle->TxLen-=2;//decrementing 2 for 2 bytes
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else{//data frame size is 8 bit
				pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;//decrementing 1 for 1 byte
				pSPIHandle->pTxBuffer++;
			}
		}
		SPI_Close_Tx(pSPIHandle);
}
static void SPI_Rx_IR_Handle(SPI_Handle_t *pSPIHandle){

	while(pSPIHandle->RxLen > 0){
			if(pSPIHandle->pSPIx->CR1 & (1<<11)){//If data frame size is 16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen-=2;//decrementing 2 for 2 bytes
			(uint16_t*)pSPIHandle->pRxBuffer++;
			}
			else{
			*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
			}
		}
		SPI_Close_Rx(pSPIHandle);
}

void SPI_Ovr_IR_Handle(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

static void SPI_Close_Tx(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1<<7);//Disable Tx empty interrupt
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = 0;
}
static void SPI_Close_Rx(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1<<6);//Disable Rx not empty interrupt
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = 0;
}

//SPI Clear Configuration Data(Clear garbage data)
/* @func	- SPI_Clear_Config
 * @brief 	- Clears garbage data from configuration memory locations. If called immediately after handle declaration, you do not have to declare default configuration values.
 * @param1	- *pSPIHandle - Pointer to handle configuration you wish to clear
 * */
void SPI_Clear_Config(SPI_Handle_t *pSPIHandle){
	pSPIHandle->SPIConfig.BIDIMODE = 0;
	pSPIHandle->SPIConfig.BIDIOE = 0;
	pSPIHandle->SPIConfig.DFF = 0;
	pSPIHandle->SPIConfig.RXONLY = 0;
	pSPIHandle->SPIConfig.SSM = 0;
	pSPIHandle->SPIConfig.SSI = 0;
	pSPIHandle->SPIConfig.LSBFIRST = 0;
	pSPIHandle->SPIConfig.SPE = 0;
	pSPIHandle->SPIConfig.BR = 0;
	pSPIHandle->SPIConfig.MSTR = 0;
	pSPIHandle->SPIConfig.CPOL = 0;
	pSPIHandle->SPIConfig.CPHA = 0;
	pSPIHandle->SPIConfig.ERRIE = 0;
	pSPIHandle->SPIConfig.FRF = 0;
	pSPIHandle->SPIConfig.SSOE = 0;
}

//SPI I2S Clear Configuration Data(Clear garbage data)
/* @func	- SPI_I2S_Clear_Config
 * @brief 	- Clears garbage data from configuration memory locations. If called immediately after handle declaration, you do not have to declare default configuration values.
 * @param1	- *pI2SHandle - Pointer to handle configuration you wish to clear
 * */
void SPI_I2S_Clear_Config(SPI_I2S_Handle_t *pI2SHandle){
	pI2SHandle->I2SConfig.I2SMOD = 0;
	pI2SHandle->I2SConfig.I2SE = 0;
	pI2SHandle->I2SConfig.I2SCFG = 0;
	pI2SHandle->I2SConfig.PCMSYNC = 0;
	pI2SHandle->I2SConfig.I2SSTD = 0;
	pI2SHandle->I2SConfig.CKPOL = 0;
	pI2SHandle->I2SConfig.DATLEN = 0;
	pI2SHandle->I2SConfig.CHLEN = 0;
	pI2SHandle->I2SPrescaler.MCKOE = 0;
	pI2SHandle->I2SPrescaler.ODD = 0;
	pI2SHandle->I2SPrescaler.I2SDIV = 0;
	pI2SHandle->I2SRCCPLL.PLLI2SR = 0;
	pI2SHandle->I2SRCCPLL.PLLI2SN = 0;
}


