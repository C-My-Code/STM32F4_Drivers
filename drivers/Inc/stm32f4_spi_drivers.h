/*
 * stm32f4_spi_drivers.h
 *
 *  Created on: Feb 21, 2020
 *      Author: Kevin
 */

#ifndef INC_STM32F4_SPI_DRIVERS_H_
#define INC_STM32F4_SPI_DRIVERS_H_
#include "stm32f429xx.h"



//SPI Configuration Structure (SPI_CR1 & SPI_CR2)
typedef struct{
//CR1
uint8_t BIDIMODE;//Bidirectional data mode enable
uint8_t BIDIOE;//Output enable in bidirectional mode
uint8_t DFF;//Data frame format
uint8_t RXONLY;//Receive only
uint8_t SSM;//Software slave management
uint8_t SSI;//Internal slave select
uint8_t LSBFIRST;//Frame format
uint8_t SPE;//SPI enable
uint8_t BR;//Baud rate control
uint8_t MSTR;//Master selection
uint8_t CPOL;//Clock polarity
uint8_t CPHA;//Clock phase

//CR2
uint8_t TXEIE;//Tx buffer empty interrupt enable
uint8_t RXNEIE;//RX buffer not empty interrupt enable
uint8_t ERRIE;//Error interrupt enable
uint8_t FRF;//Frame format
uint8_t	SSOE;//SS output enable
uint8_t	TXDMAEN;//Tx buffer DMA enable
uint8_t	RXDMAEN;//Rx buffer DMA enable
}SPI_Config_t;

//SPI I2S Configuration Structure
typedef struct{
uint8_t I2SMOD;//I2S mode selection
uint8_t I2SE;//I2S Enable
uint8_t I2SCFG;//I2S configuration mode
uint8_t PCMSYNC;//PCM frame synchronization
uint8_t I2SSTD;//I2S standard selection
uint8_t CKPOL;//Steady state clock polarity
uint8_t DATLEN;//Data length to be transferred
uint8_t CHLEN;//Channel length (number of bits per audio channel)
}SPI_I2S_Config_t;

//SPI I2S Prescaler Configuration Structure
typedef struct{
uint8_t MCKOE;//Master clock output enable
uint8_t ODD;//Odd factor for the prescaler
uint8_t I2SDIV;//I2S Linear prescaler
}SPI_I2S_Prescaler_t;

//SPI I2S RCC PLL Configuration Structure(RCC_PLLI2SCFGR)
typedef struct{
uint16_t PLLI2SN;//PLLI2S multiplication factor for VCO
uint8_t  PLLI2SR;//PLLI2S division factor for I2S clocks
}SPI_I2C_RCCPLL_t;

//SPI Handle Structure
typedef struct{
SPI_RegDef_t *pSPIx;
SPI_Config_t SPIConfig;
uint8_t		 *pTxBuffer;
uint8_t		 *pRxBuffer;
uint32_t	 TxLen;
uint32_t	 RxLen;
uint8_t		 TxState;
uint8_t		 RxState;
}SPI_Handle_t;

//SPI I2S Handle Structure
typedef struct{
SPI_RegDef_t *pSPIx;
SPI_I2S_Config_t I2SConfig;
SPI_I2S_Prescaler_t I2SPrescaler;
SPI_I2C_RCCPLL_t I2SRCCPLL;
}SPI_I2S_Handle_t;


/*
 * SPI CR1 CONFIGURATION MACROS
 */
/*-------SPI CLOCK PHASE-------*/
#define SPI_CLKPHASE_FIRSTCLOCK		0 	//The first clock transition is the first data capture edge
#define SPI_CLKPHASE_SECONDCLOCK	1	//The second clock transition is the first data capture edge

/*-------SPI CLOCK POLARITY-------*/
#define SPI_CLKPOLARITY_0IDLE	0		//0 when idle
#define SPI_CLKPOLARITY_1IDLE	1		//1 when idle

/*-------SPI MSTR---------*/
#define SPI_MODE_SLAVE		0			//Slave configuration
#define SPI_MODE_MASTER		1			//Master configuration

/*-------SPI BRAUD RATE----------*/
#define SPI_BRAUD_2		0 //pclk/2
#define SPI_BRAUD_4		1	//pclk/4
#define SPI_BRAUD_8		2 //pclk/8
#define SPI_BRAUD_16	3 //pclk/16
#define SPI_BRAUD_32   	1 //pclk/32
#define SPI_BRAUD_64	5 //pclk/64
#define SPI_BRAUD_128	6 //pclk/128
#define SPI_BRAUD_256	7 //pclk/256

/*-------SPI LSBFIRST---------*/
#define SPI_LSBFIRST_MSB	0
#define SPI_LSBFIRST_LSB	1

/*-------SPI SOFTWARE SLAVE--------*/
#define SPI_SOFTSLAVE_DISABLED	0
#define SPI_SOFTSLAVE_ENABLED	1

/*-------SPI DATAFRAME MACROS--------*/
#define SPI_DFF_8	0	//8-bit data frame format is selected for transmission/reception
#define SPI_DFF_16	1	//16-bit data frame format is selected for transmission/reception

/*-------SPI BIDIRECTIONAL OUT--------*/
#define SPI_BIDIR_OUT_DISABLE	0 //Output disabled (receive-only mode)
#define SPI_BIDIR_OUT_ENABLE	1 //Output enabled (transmit-only mode)

/*-------SPI BIDIRECTIONAL DATA---------*/
#define SPI_BIDIMODE_UNI	0  //unidirectional data mode
#define SPI_BIDIMODE_BI	    1  //bidirectional data mode

/*
 * I2S CONFIGURATION MACROS
 */
/*------I2S MODE-------*/
#define I2S_SPI_MODE	0
#define I2S_I2S_MODE	1

/*------I2S CONFIG MODE-------*/
#define I2S_SLAVE_TX	0
#define I2S_SLAVE_RX	1
#define I2S_MSTR_TX		2
#define I2S_MSTR_RX		3

/*------I2S PCM FRAME SYNC-------*/
#define I2S_PCM_SHORT	0
#define I2S_PCM_LONG	1

/*------I2S STANDARD SELECTION-------*/
#define I2S_STD_I2S		0//Philips Standard
#define I2S_STD_MSB		1//Left Justified
#define I2S_STD_LSB		2//Right Justified
#define I2S_STD_PCM		3

/*------I2S CLOCK POLARITY-------*/
#define I2S_CLKPOL_LOW	0//steady state is low level
#define I2S_CLKPOL_HIGH	1//steady state is high level

/*------I2S DATA LENGTH-------*/
#define I2S_DLEN_16		0//16-bit data length
#define I2S_DLEN_24		1//24-bit data length
#define I2S_DLEN_32		2//32-bit data length

/*------I2S CHANNEL LENGTH-------*/
#define I2S_CLEN_16		0//16-bit wide
#define I2S_CLEN_32		1//32-bit wide

/*------I2S PRESCALER ODD FACTOR-------*/
#define I2S_ODD_X2		0//real divider value is = I2SDIV *2
#define I2S_ODD_X2_1	1//real divider value is = (I2SDIV * 2)+1

/*------I2S DIV LINEAR PRESCALER-------*/
//Master Clock Out Disabled
#define I2S_MSTR_DIS_441_16		53//Master Clock Out Disabled, 44.1K 16 Bit(ODD=1)
#define I2S_MSTR_DIS_441_32		19//Master Clock Out Disabled, 44.1K 32 Bit(ODD=0)
#define I2S_MSTR_DIS_96_16		12//Master Clock Out Disabled, 96K 16 Bit(ODD=1)
#define I2S_MSTR_DIS_96_32		11//Master Clock Out Disabled, 96K 32 Bit(ODD=1)
#define I2S_MSTR_DIS_192_16		11//Master Clock Out Disabled, 192K 16 Bit(ODD=1)
#define I2S_MSTR_DIS_192_32		3//Master Clock Out Disabled, 192K 32 Bit(ODD=1)

//Master Clock Out Enabled
#define I2S_MSTR_DIS_441		6//Master Clock Out Enabled, 44.1K (ODD=0)
#define I2S_MSTR_DIS_96			3//Master Clock Out Enabled, 96K (ODD=1)



/*---------------------------SPI API's------------------------------------
 * ------------------------------------------------------------------------*/
//Enable/Disable SPI Port
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Disable(SPI_RegDef_t *pSPIx);

//Enable/Disable SPI I2S
void SPI_I2S_Init(SPI_I2S_Handle_t *pI2SHandle);
void SPI_I2S_Disable(SPI_RegDef_t *pSPIx);

//Peripheral Clock Setup
void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnableDisable);

//SPI Send & Receive - Blocking Call(non-interrupt)
void SPI_Send(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
void SPI_Receive(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

//SPI Send & Receive - Interrupt
void SPI_Send_Ir(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
void SPI_Receive_Ir(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

//IRQ Config & ISR Handling
void SPI_IRQ_EnableDisable(uint8_t IRQNumber, uint8_t EnableDisable);
void SPI_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
static void SPI_Tx_IR_Handle();
static void SPI_Rx_IR_Handle();
static void SPI_Ovr_IR_Handle();

void SPI_Close_Tx(SPI_Handle_t *pSPIHandle);
void SPI_Close_Rx(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F4_SPI_DRIVERS_H_ */
