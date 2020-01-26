/*
 * stm32f429xx.h
 *
 *  Last Update on: Jan 25, 2020
 *      Author: Kevin Miller
 */
#include<stdint.h>
#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_


#define ENABLE	1
#define DISABLE 0



//Nested vectored interrupt controller (NVIC)
#define NVIC_ISER_BASE			((volatile uint32_t*)0xE000E100)//Interrupt set-enable register base address
#define NVIC_ICER_BASE			((volatile uint32_t*)0XE000E180)//Interrupt clear-enable register
#define NVIC_ISPR_BASE			((volatile uint32_t*)0XE000E200)//Interrupt set-pending register
#define NVIC_ICPR_BASE			((volatile uint32_t*)0XE000E280)//Interrupt clear-pending register
#define NVIC_IABR_BASE			((volatile uint32_t*)0xE000E300)//Interrupt active bit register
#define NVIC_IPR_BASE			((volatile uint32_t*)0xE000E400)//Interrupt priority register
#define NVIC_OFFSET				0x04

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR          0x2001C000U
#define ROM                     0x1FFF0000U
#define SRAM                    SRAM1_BASEADDR
#define RCC_BASEADDR			0x40023800U

#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U

//AHB1 Bus
#define GPIOA_BASE 				0x40020000U
#define GPIOB_BASE				0x40020400U
#define GPIOC_BASE				0x40020800U
#define GPIOD_BASE				0x40020C00U
#define GPIOE_BASE				0x40021000U
#define GPIOF_BASE				0x40021400U
#define GPIOG_BASE				0x40021800U
#define GPIOH_BASE				0x40021C00U
#define GPIOI_BASE				0x40022000U
#define GPIOJ_BASE				0x40022400U
#define GPIOK_BASE				0x40022800U

//APB1 Bus
#define I2C1_BASE				0x40005400U
#define I2C2_BASE				0x40005800U
#define I2C3_BASE 				0x40005C00U
#define SPI2_BASE  				0x40003800U
#define SPI3_BASE				0x40003C00U
#define USART2_BASE				0x40004400U
#define USART3_BASE				0x40004800U
#define UART4_BASE				0x40004C00U
#define UART5_BASE				0x40005000U

//APB2 Bus
#define SPI1_BASE				0x40013000U
#define USART1_BASE				0x40011000U
#define USART6_BASE				0x40011400U
#define EXTI_BASE				0x40013C00U
#define SYSCFG_BASE				0x40013800U



/*----------Peripheral Register Definition Structures-----------*/

//GPIO Port Register Definition Structure
typedef struct{
volatile uint32_t MODER;
volatile uint32_t OTYPER;
volatile uint32_t OSPEEDR;
volatile uint32_t PUPDR;
volatile uint32_t IDR;
volatile uint32_t ODR;
volatile uint32_t BSSR;
volatile uint32_t LCKR;
volatile uint32_t AFRL;
volatile uint32_t AFRH;
}GPIO_RegDef_t;

//RCC Register Definition Structure
typedef struct{
volatile uint32_t CR;
volatile uint32_t PLLCFGR;
volatile uint32_t CFGR;
volatile uint32_t CIR;
volatile uint32_t AHB1RSTR;
volatile uint32_t AHB2RSTR;
volatile uint32_t AHB3RSTR;
uint32_t RESERVED1;
volatile uint32_t APB1RSTR;
volatile uint32_t APB2RSTR;
uint32_t RESERVED2;
uint32_t RESERVED3;
volatile uint32_t AHB1ENR;
volatile uint32_t AHB2ENR;
volatile uint32_t AHB3ENR;
uint32_t RESERVED4;
volatile uint32_t APB1ENR;
volatile uint32_t APB2ENR;
uint32_t RESERVED5;
uint32_t RESERVED6;
volatile uint32_t AHB1LPENR;
volatile uint32_t AHB2LPENR;
volatile uint32_t AHB3LPENR;
uint32_t RESERVED7;
volatile uint32_t APB1LPENR;
volatile uint32_t APB2LPENR;
uint32_t RESERVED8;
uint32_t RESERVED9;
volatile uint32_t BDCR;
volatile uint32_t CSR;
uint32_t RESERVED10;
uint32_t RESERVED11;
volatile uint32_t SSCGR;
volatile uint32_t PLLI2SCFGR;
volatile uint32_t PLLSAICFGR;
volatile uint32_t DCKCFGR;
}RCC_RegDef_t;

//Creating GPIO ports as a GPIO_RegDef_t structure
#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG   ((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH   ((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI   ((GPIO_RegDef_t*)GPIOI_BASE)
#define GPIOJ   ((GPIO_RegDef_t*)GPIOJ_BASE)
#define GPIOK   ((GPIO_RegDef_t*)GPIOK_BASE)


#define RCC   ((RCC_RegDef_t*)RCC_BASEADDR)


//EXTI Register Definition Structure
typedef struct{
volatile uint32_t EXTI_IMR;
volatile uint32_t EXTI_EMR;
volatile uint32_t EXTI_RTSR;
volatile uint32_t EXTI_FTSR;
volatile uint32_t EXTI_SWIER;
volatile uint32_t EXTI_PR;
}EXTI_RegDef_t;

#define EXTI  ((EXTI_RegDef_t*)EXTI_BASE)

//EXTICR PORT CODE MACRO
#define FETCH_EXTICR_PORT_CODE(x)		((x== GPIOA) ? 0:\
										 (x== GPIOB) ? 1:\
										 (x== GPIOC) ? 2:\
										 (x== GPIOD) ? 3:\
										 (x== GPIOE) ? 4:\
										 (x== GPIOF) ? 5:\
										 (x== GPIOG) ? 6:\
										 (x== GPIOH) ? 7:\
										 (x== GPIOI) ? 8:\
										 (x== GPIOJ) ? 9:\
										 (x== GPIOK) ? 10:0)

//SYSCFG Register Definition Structure
typedef struct{
volatile uint32_t SYSCFG_MEMRMP;
volatile uint32_t SYSCFG_PMC;
volatile uint32_t SYSCFG_EXTICR[4];
volatile uint32_t SYSCFG_CMPCR;
volatile uint32_t SYSCFG_CFGR;
}SYSCFG_RegDef_t;

#define SYSCFG 	((SYSCFG_RegDef_t*)SYSCFG_BASE)

/*  ---------------------ENABLE RCC PERIPHERAL CLOCK MACROS-----------------------------------*/

//Enable GPIO RCC Peri Clock
#define GPIOA_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<0))
#define GPIOB_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<1))
#define GPIOC_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<2))
#define GPIOD_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<3))
#define GPIOE_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<4))
#define GPIOF_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<5))
#define GPIOG_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<6))
#define GPIOH_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<7))
#define GPIOI_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<8))
#define GPIOJ_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<9))
#define GPIOK_PCLK_ENABLE() 			(RCC->AHB1ENR |=  (1<<10))

//ENABLE I2C Peri Clock
#define I2CP1_PCLK_ENABLE()				(RCC->APB1ENR  |= (1<<21))
#define I2CP2_PCLK_ENABLE()				(RCC->APB1ENR  |= (1<<22))
#define I2CP3_PCLK_ENABLE()				(RCC->APB1ENR  |= (1<<23))

//Enable SPI Peri Clock
#define SPI1_PCLK_ENABLE()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<15))

//Enable USART Peri Clock
#define USART1_PCLK_ENABLE()			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_ENABLE()			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_ENABLE()			(RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_ENABLE()			(RCC->APB2ENR |= (1<<5))

//Enable UART Peri Clock
#define UART4_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<20))
#define UART7_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<30))
#define UART8_PCLK_ENABLE()				(RCC->APB1ENR |= (1<<31))

//Enable SYSCFG Peri Clock
#define SYSCFG_PCLK_ENABLE()			(RCC->APB2ENR |= (1<<14))


/*----------------DISABLE RCC PERIPHERAL CLOCK MACROS-------------------------*/

//Disable GPIO Peri Clock
#define GPIOA_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<0))
#define GPIOB_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<1))
#define GPIOC_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<2))
#define GPIOD_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<3))
#define GPIOE_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<4))
#define GPIOF_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<5))
#define GPIOG_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<6))
#define GPIOH_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<7))
#define GPIOI_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<8))
#define GPIOJ_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<9))
#define GPIOK_PCLK_DISABLE() 			(RCC->AHB1ENR &=  ~(1<<10))

//Disable I2C Peri Clock
#define I2CP1_PCLK_DISABLE()			(RCC->APB1ENR  &= ~(1<<21))
#define I2CP2_PCLK_DISABLE()			(RCC->APB1ENR  &= ~(1<<22))
#define I2CP3_PCLK_DISABLE()			(RCC->APB1ENR  &= ~(1<<23))

//Disable SPI Peri Clock
#define SPI1_PCLK_DISABLE()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DISABLE()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DISABLE()				(RCC->APB1ENR &= ~(1<<15))

//Disable USART Peri Clock
#define USART1_PCLK_DISABLE()			(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DISABLE()			(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DISABLE()			(RCC->APB1ENR &= ~(1<<18))
#define USART6_PCLK_DISABLE()			(RCC->APB2ENR &= ~(1<<5))

//Disable UART Peri Clock
#define UART4_PCLK_DISABLE()			(RCC->APB1ENR &= (1<<19))
#define UART5_PCLK_DISABLE()			(RCC->APB1ENR &= (1<<20))
#define UART7_PCLK_DISABLE()			(RCC->APB1ENR &= (1<<30))
#define UART8_PCLK_DISABLE()			(RCC->APB1ENR &= (1<<31))

//Disable SYSCFG Peri Clock
#define SYSCFG_PCLK_DISABLE()			(RCC->APB2ENR &= ~(1<<14))

//Reset GPIO Peripherals
#define GPIOA_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));} while(0)
#define GPIOJ_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9));} while(0)
#define GPIOK_REG_RESET()				do{(RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1<<10));} while(0)



/*------------------INTERRPUT REQUEST(IRQ) NUMBER MACROS---------------------------------------*/
#define IRQ_NO_EXTI0	6
#define IRQ_NO_EXTI1	7
#define IRQ_NO_EXTI2	8
#define IRQ_NO_EXTI3	9
#define IRQ_NO_EXTI4	10
#define IRQ_NO_EXTI9_5	23
#define IRQ_NO_EXTI5_10	40

#endif /* INC_STM32F429XX_H_ */
