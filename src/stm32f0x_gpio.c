/**
  ******************************************************************************
  * @file    Template_2/stm32f0_gpio.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   GPIO functions.
  ******************************************************************************
  * @attention
  *
  * Use this functions to configure global inputs/outputs.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0x_gpio.h"
#include "hard.h"



//--- Private typedef ---//
//--- Private define ---//
//--- Private macro ---//
//--- Private variables ---//
//--- Private function prototypes ---//
//--- Private functions ---//

//-------------------------------------------//
// @brief  GPIO configure.
// @param  None
// @retval None
//------------------------------------------//
void GPIO_Config (void)
{
	unsigned long temp;

	//--- MODER ---//
	//00: Input mode (reset state)
	//01: General purpose output mode
	//10: Alternate function mode
	//11: Analog mode

	//--- OTYPER ---//
	//These bits are written by software to configure the I/O output type.
	//0: Output push-pull (reset state)
	//1: Output open-drain

	//--- ORSPEEDR ---//
	//These bits are written by software to configure the I/O output speed.
	//x0: Low speed.
	//01: Medium speed.
	//11: High speed.
	//Note: Refer to the device datasheet for the frequency.

	//--- PUPDR ---//
	//These bits are written by software to configure the I/O pull-up or pull-down
	//00: No pull-up, pull-down
	//01: Pull-up
	//10: Pull-down
	//11: Reserved

#ifdef VER_1_0
	//--- GPIO A ---//
	if (!GPIOA_CLK)
		GPIOA_CLK_ON;

	temp = GPIOA->MODER;	//2 bits por pin
	temp &= 0x3C030000;		//PA0 - PA1 analog input; PA2 - PA3 alternate function; PA4 - PA5 input; PA6 alternate function; PA7 out open drain;
//	temp |= 0x416860AF;		//PA9 PA10 alternative; PA11 PA12 PA15 out open drain
//	temp |= 0x4168606F;		//pruebo pin3
	temp |= 0x416860A3;		//PA1 input para pruebas
	GPIOA->MODER = temp;

	temp = GPIOA->OTYPER;	//1 bit por pin
	temp &= 0xFFFF7F7F;
//	temp |= 0x00000080;		//PA7 open drain
	temp |= 0x00008080;		//PA15 y PA7 open drain
	GPIOA->OTYPER = temp;

	temp = GPIOA->OSPEEDR;	//2 bits por pin
	temp &= 0x3C030F0F;
	temp |= 0x00000000;		//low speed
	GPIOA->OSPEEDR = temp;

	temp = GPIOA->PUPDR;	//2 bits por pin
	temp &= 0xFFFFFFF3;
	temp |= 0x00000008;		//pull down pin1 para pruebas
	GPIOA->PUPDR = temp;



	//Alternate Fuction
	//GPIOA->AFR[0] = 0x11000000;	//PA7 -> AF1; PA6 -> AF1

	//--- GPIO B ---//
	if (!GPIOB_CLK)
		GPIOB_CLK_ON;

	temp = GPIOB->MODER;	//2 bits por pin
	temp &= 0xFFFF0F3C;		//PB0 analog input; PB3 input PB6 PB7 alternative
	temp |= 0x0000A003;
	GPIOB->MODER = temp;

	temp = GPIOB->OTYPER;	//1 bit por pin
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOB->OTYPER = temp;

	temp = GPIOB->OSPEEDR;	//2 bits por pin
	temp &= 0xFFFF0FFF;
	temp |= 0x00000000;		//low speed
	GPIOB->OSPEEDR = temp;

	temp = GPIOB->PUPDR;	//2 bits por pin
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOB->PUPDR = temp;

	//Alternate Fuction
	//GPIOB->AFR[0] = 0x11000000;	//PA7 -> AF1; PA6 -> AF1
#endif

#ifdef GPIOF_ENABLE

	//--- GPIO F ---//
	if (!GPIOF_CLK)
		GPIOF_CLK_ON;

	temp = GPIOF->MODER;
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOF->MODER = temp;

	temp = GPIOF->OTYPER;
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOF->OTYPER = temp;

	temp = GPIOF->OSPEEDR;
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOF->OSPEEDR = temp;

	temp = GPIOF->PUPDR;
	temp &= 0xFFFFFFFF;
	temp |= 0x00000000;
	GPIOF->PUPDR = temp;

#endif


//	//Interrupt en PB8
//	if (!SYSCFG_CLK)
//		SYSCFG_CLK_ON;
//
//	SYSCFG->EXTICR[2] = 0x00000001; //Select Port B & Pin 8 external interrupt
//	EXTI->IMR |= 0x0100; 			//Corresponding mask bit for interrupts
//	EXTI->EMR |= 0x0000; 			//Corresponding mask bit for events
//	EXTI->RTSR |= 0x0100; 			//Pin 8 Interrupt line on rising edge
//	EXTI->FTSR |= 0x0100; 			//Pin 8 Interrupt line on falling edge
//
//	NVIC_EnableIRQ(EXTI4_15_IRQn);
//	NVIC_SetPriority(EXTI4_15_IRQn, 6);


}

inline void EXTIOff (void)
{
	EXTI->IMR &= ~0x00000100;
}

inline void EXTIOn (void)
{
	EXTI->IMR |= 0x00000100;
}

//--- end of file ---//
