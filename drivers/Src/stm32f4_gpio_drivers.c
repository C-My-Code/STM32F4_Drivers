/*
 * stm32f4_gpio_drivers.c
 *
 *  Last Update on: Jan 25, 2020
 *      Author: Kevin Miller
 */

#include "stm32f4_gpio_drivers.h"


/*---------------------------GPIO API's------------------------------------
 * ------------------------------------------------------------------------*/

//Enable GPIO Port
/* @func	 - GPIO_Init
 * @brief    - initializes GPIO port
 * @param1   - *pGPIOHandle - pointer to GPIO handle struct that contains GPIO port base address and pin configuration struct see: stm32f4_gpio_drivers.h
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp_int = 0;

	//1.  Configure GPIO pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ //Non-Interrupt Modes
		temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Creates shifted register value
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clears register bit field to be changed
		pGPIOHandle->pGPIOx->MODER |= temp_int; //stores value in proper register
		temp_int = 0;
	}
	else{//Interrupt Modes

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RT){
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FT){
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFT){
			EXTI->EXTI_RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//3. Configure the GPIO Port selection in SYSCFG_EXTICR
		uint8_t temp_int1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4);//Determines which EXTICR the pin belongs to
		uint8_t temp_int2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4);//Creates register shift value
		SYSCFG_PCLK_ENABLE();
		uint8_t EXTICR_Port_Code = FETCH_EXTICR_PORT_CODE(pGPIOHandle->pGPIOx);//Fetches GPIO port value for EXTICR register
		SYSCFG->SYSCFG_EXTICR[temp_int1] |= (EXTICR_Port_Code<<(temp_int2*4));


		//4. enable EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp_int = 0;

	//2.  Configure speed
	temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Creates shifted register value
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clears register bit field to be changed
	pGPIOHandle->pGPIOx->OSPEEDR |= temp_int;//stores value in proper register
	temp_int = 0;

	//3. Configure Pull-Up Pull-Down
	temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Creates shifted register value
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clears register bit field to be changed
	pGPIOHandle->pGPIOx->PUPDR |= temp_int;//stores value in proper register
	temp_int = 0;

	//4. Configure Pin Output Type
	temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Creates shifted register value
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clears register bit field to be changed
	pGPIOHandle->pGPIOx->OTYPER	|= temp_int;//stores value in proper register
	temp_int = 0;

	//5. Configure Pin Alternative Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT){
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<8){ //if pin number is less than 8, it is controlled by AFRL register
			temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode <<(4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Creates shifted register value
			pGPIOHandle->pGPIOx->AFRL &= ~(15 << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));//clears register bit field to be changed
			pGPIOHandle->pGPIOx->AFRL |= temp_int;//stores value in proper register
		}
		else{
			temp_int = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode <<(4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8));//Creates shifted register value
			pGPIOHandle->pGPIOx->AFRH &= ~(15 << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8)));//clears register bit field to be changed
			pGPIOHandle->pGPIOx->AFRH |= temp_int;//stores value in proper register
		}
	}
	temp_int = 0;


}

//Disable GPIO Port
/* @func	 - GPIO_Disable
 * @brief    - Disables GPIO port
 * @param1   - *pGPIOx - pointer to GPIO port base address
 * */
void GPIO_Disable(GPIO_RegDef_t *pGPIOx){

	switch((uintptr_t)pGPIOx){
		case GPIOA_BASE:
			GPIOA_REG_RESET();
			break;
		case GPIOB_BASE:
			GPIOB_REG_RESET();
			break;
		case GPIOC_BASE:
			GPIOC_REG_RESET();
			break;
		case GPIOD_BASE:
			GPIOD_REG_RESET();
			break;
		case GPIOE_BASE:
			GPIOE_REG_RESET();
			break;
		case GPIOF_BASE:
			GPIOF_REG_RESET();
			break;
		case GPIOG_BASE:
			GPIOG_REG_RESET();
			break;
		case GPIOH_BASE:
			GPIOH_REG_RESET();
			break;
		case GPIOI_BASE:
			GPIOI_REG_RESET();
			break;
		case GPIOJ_BASE:
			GPIOJ_REG_RESET();
			break;
		case GPIOK_BASE:
			GPIOK_REG_RESET();
			break;
		}
}






//Peripheral Clock Setup
/* @func	- GPIO_PCLK_Control
 * @brief	- Enables or disables GPIO port peripheral clock for given address
 * @param1	- *pGPIOx - pointer to  GPIO  port address
 * @param2  - EnableDisable - binary enable[1] or disable[0]
 * */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnableDisable){
	if(EnableDisable == 1){
		switch((uintptr_t)pGPIOx){
		case GPIOA_BASE:
			GPIOA_PCLK_ENABLE();
			break;
		case GPIOB_BASE:
			GPIOB_PCLK_ENABLE();
			break;
		case GPIOC_BASE:
			GPIOC_PCLK_ENABLE();
			break;
		case GPIOD_BASE:
			GPIOD_PCLK_ENABLE();
			break;
		case GPIOE_BASE:
			GPIOE_PCLK_ENABLE();
			break;
		case GPIOF_BASE:
			GPIOF_PCLK_ENABLE();
			break;
		case GPIOG_BASE:
			GPIOG_PCLK_ENABLE();
			break;
		case GPIOH_BASE:
			GPIOH_PCLK_ENABLE();
			break;
		case GPIOI_BASE:
			GPIOI_PCLK_ENABLE();
			break;
		case GPIOJ_BASE:
			GPIOJ_PCLK_ENABLE();
			break;
		case GPIOK_BASE:
			GPIOK_PCLK_ENABLE();
			break;
		}
	}
	if(EnableDisable == 0){
		switch((uintptr_t)pGPIOx){
		case GPIOA_BASE:
			GPIOA_PCLK_DISABLE();
			break;
		case GPIOB_BASE:
			GPIOB_PCLK_DISABLE();
			break;
		case GPIOC_BASE:
			GPIOC_PCLK_DISABLE();
			break;
		case GPIOD_BASE:
			GPIOD_PCLK_DISABLE();
			break;
		case GPIOE_BASE:
			GPIOE_PCLK_DISABLE();
			break;
		case GPIOF_BASE:
			GPIOF_PCLK_DISABLE();
			break;
		case GPIOG_BASE:
			GPIOG_PCLK_DISABLE();
			break;
		case GPIOH_BASE:
			GPIOH_PCLK_DISABLE();
			break;
		case GPIOI_BASE:
			GPIOI_PCLK_DISABLE();
			break;
		case GPIOJ_BASE:
			GPIOJ_PCLK_DISABLE();
			break;
		case GPIOK_BASE:
			GPIOK_PCLK_DISABLE();
		    break;
	}

 }
}
//GPIO Read Input Pin
/* @func	- GPIO_Read_InputPin
 * @brief	- Reads input from 8 bit pin register
 * @param1	- *pGPIOx - pointer to GPIO port base address
 * @param2	- pinnumber - 8 bit pin number to read from
 * @return  - uint8_t - returns 8 bit unsigned integer from pin input register
 * */
uint8_t GPIO_Read_InputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinnumber) & 0x00000001);//Shifts desired pin value to 0th bit and masks other bits to ensure 0 or 1 value
	return value;
}


//GPIO Read Input Port
/* @func	- GPIO_Read_InputPort
 * @brief	- Reads input from 16 bit port register
 * @param1	- *pGPIOx - pointer to GPIO port base address
 * @return  - uint16_t - returns 16 bit unsigned integer from port input register
 * */
uint16_t GPIO_Read_InputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (pGPIOx->IDR);
	return value;
}


//GPIO Write to Output Pin
/* @func	- GPIO_Write_OutputPin
 * @brief	- Writes output value to the GPIO output port register at the selected pin position
 * @param1	- *pGPIOx - pointer to GPIO port base address
 * @param2	- pinnumber - 8 bit pin number to write to
 * @param3  - uint8_t - 8 bit output value
 * */
void GPIO_Write_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber, uint8_t value){
	if(value == 1){
		pGPIOx->ODR |= (1 << pinnumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << pinnumber);
	}
}


//GPIO Write to Output Port
/* @func	- GPIO_Write_OutputPort
 * @brief	- Writes to 16 bit output port register
 * @param1	- *pGPIOx - pointer to GPIO port base address
 * @param3  - uint8_t - 16 bit output value
 * */
void GPIO_Write_OutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){

	pGPIOx->ODR |= value;
}


//GPIO Toggle Output Pin
/* @func	- GPIO_Toggle_OutputPin
 * @brief	- Toggles GPIO output pin
 * @param1	- *pGPIOx - pointer to GPIO port base address
 * @param2	- pinnumber - 8 bit pin number to toggle to
 * */
void GPIO_Toggle_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinnumber){
	pGPIOx->ODR ^= (1 << pinnumber);
}



//GPIO Interrupt Enable & Disable
/* @func	- GPIO_IRQ_EnableDisable
 * @brief	- Enables or disables GPIO interrupt
 * @param1	- IRQNumber - IRQNumber for the EXTI line to enable or disable. See stm32f429xx.h INTERRPUT REQUEST(IRQ) NUMBER MACROS
 * @param2	- EnableDisable - 1=ENABLE 0=DISABLE
 * */
void GPIO_IRQ_EnableDisable(uint8_t IRQNumber, uint8_t EnableDisable){

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

//GPIO Interrupt Priority Configuration
/* @func	- GPIO_IRQ_Priority_Config
 * @brief 	- Sets the priority for a given interrupt
 * @param1	- IRQNumber - interrupt number for the EXTI line you intend to configure. See stm32f429xx.h "INTERRPUT REQUEST(IRQ) NUMBER MACROS"
 * @param2	- IRQPriority  - 4 bit priority value
 * */
void GPIO_IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t IP_reg = IRQNumber/4;//Calculates which priority register address row the interrupt belongs to (0-59)
	uint8_t IP_Section = IRQNumber%4;//This determines which partition the IRQ# belongs in. Each address row is divided into 4 byte sized partitions, one byte for each IRQNumber.
	*(NVIC_ISPR_BASE+IP_reg) |= ((IRQPriority<<4)<<(IP_Section*8));//Shifts the value 4 bits past 0th partition bit(only bits 7:4 are read) and stores it in the correct priority register address row.
}

void GPIO_IRQHandling(uint8_t pinnumber){
	if(EXTI->EXTI_PR & (1<<pinnumber)){//Checks for pending interrupt on the specified pin number
		EXTI->EXTI_PR  |= (1<<pinnumber);//Clears the pending interrupt register bit for that pin position. This register is cleared by setting the bit to "1"
	}
}





