/*
 * stm32f4_gpio_drivers.h
 *
 *  Created on: Jan 16, 2020
 *      Author: Kevin
 */

#ifndef INC_STM32F4_GPIO_DRIVERS_H_
#define INC_STM32F4_GPIO_DRIVERS_H_

#include "stm32f429xx.h"


/*-------GPIO PIN MODE MACROS------------- */
#define GPIO_MODE_IN		    0
#define GPIO_MODE_OUT	        1
#define GPIO_MODE_ALT		    2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_INT_RT		4   //interrupt mode on rising edge trigger
#define GPIO_MODE_INT_FT		5	//interrupt mode on falling edge trigger
#define GPIO_MODE_INT_RFT		6	//interrupt mode on rising and falling edge trigger

/*-------GPIO PIN OUTPUT TYPE MACROS-------*/
#define GPIO_OUTPUT_PUSHPULL	0
#define GPIO_OUTPUT_OPENDRAIN	1

/*-------GPIO PIN SPEED MACROS-------------*/
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERYHIGH		3

/*-------GPIO PULLUP/PULLDOWN CONTROL MACROS-----*/
#define GPIO_NO_PULL			0
#define GPIO_PULL_UP			1
#define GPIO_PULL_DOWN			2

/*-------GPIO PIN NUMBER MACROS-------------*/
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






//GPIO pin configuration structure
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

//GPIO handle structure
typedef struct{
GPIO_RegDef_t *pGPIOx;  //This holds the base address for the GPIO port to which the pin belongs
GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*---------------------------GPIO API's------------------------------------
 * ------------------------------------------------------------------------*/

//Enable/Disable GPIO Port
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Disable(GPIO_RegDef_t *pGPIOx);

//Peripheral Clock Setup
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnableDisable);

//GPIO Read & Write
uint8_t GPIO_Read_InputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber);
uint16_t GPIO_Read_InputPort(GPIO_RegDef_t *pGPIOx); 	 //port register is 16 bits
void GPIO_Write_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber, uint8_t value);
void GPIO_Write_OutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_Toggle_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber);

//IRQ Config & ISR Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnableDisable);
void GPIO_IRQHandling(uint8_t pinnumber);












#endif /* INC_STM32F4_GPIO_DRIVERS_H_ */
