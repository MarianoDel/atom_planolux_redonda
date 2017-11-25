/**
  ******************************************************************************
  * @file    Template_2/stm32f0_uart.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   UART functions.
  ******************************************************************************
  * @attention
  *
  * Use this functions to configure serial comunication interface (UART).
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "stm32f0xx.h"
#include "uart.h"

#include "gps_vktel.h"

#include <string.h>




//--- Private typedef ---//
//--- Private define ---//
//--- Private macro ---//

//#define USE_USARTx_TIMEOUT



//--- Externals variables ---//

//--- Externals del GPS ---//
extern volatile unsigned char usart1_mini_timeout;
extern volatile unsigned char usart1_pckt_ready;
extern volatile unsigned char usart1_have_data;

#ifdef USE_GSM_GATEWAY
extern volatile unsigned char usart2_mini_timeout;
extern volatile unsigned char usart2_pckt_ready;
extern volatile unsigned char usart2_have_data;
#endif

//#define data512		data1		//en rx es la trama recibida; en tx es la trama a enviar
//#define data256		data		//en rx son los valores del channel elegido
//volatile unsigned char * pdmx;

extern volatile unsigned char tx1buff[];
extern volatile unsigned char rx1buff[];

extern volatile unsigned char tx2buff[];
extern volatile unsigned char rx2buff[];

//--- Private variables ---//
volatile unsigned char * ptx1;
volatile unsigned char * ptx1_pckt_index;
volatile unsigned char * prx1;

volatile unsigned char * ptx2;
volatile unsigned char * ptx2_pckt_index;
volatile unsigned char * prx2;

volatile unsigned char pckt_gps_ready = 0;
volatile unsigned char usart_mode = USART_GPS_MODE;

//Reception buffer.

//Transmission buffer.

//--- Private function prototypes ---//
//--- Private functions ---//

//cambio de modo al USART del GPS al GSM
//le paso el modo o le pregunto
//responde modo
unsigned char Usart1Mode (unsigned char new_mode)
{
	unsigned int temp_int;
	unsigned int temp_gpio;

	temp_int = USART1->CR1;
	USART1->CR1 &= 0xFFFFFF6F;		//limpio flags IE

	if (new_mode == USART_GPS_MODE)
	{
		//config gpio A to Input
		temp_gpio = GPIOA->MODER;		//2 bits por pin
		temp_gpio &= 0xFFC3FFFF;		//PA9 PA10 input
		temp_gpio |= 0x00000000;		//
		GPIOA->MODER = temp_gpio;

		GPIOA->AFR[1] &= 0xFFFFF00F;	//PA9 -> AF0 A10 -> AF0

		//config gpio B to Alternative
		temp_gpio = GPIOB->MODER;		//2 bits por pin
		temp_gpio &= 0xFFFF0FFF;		//PB6 PB7 alternative
		temp_gpio |= 0x0000A000;		//
		GPIOB->MODER = temp_gpio;

		GPIOB->AFR[0] &= 0x00FFFFFF;	//PB7 -> AF0 PB6 -> AF0

		usart_mode = USART_GPS_MODE;
	}

	if (new_mode == USART_GSM_MODE)
	{
		//config gpio B to Input
		temp_gpio = GPIOB->MODER;		//2 bits por pin
		temp_gpio &= 0xFFFF0FFF;		//PB6 PB7 input
		temp_gpio |= 0x00000000;		//
		GPIOB->MODER = temp_gpio;

		GPIOB->AFR[0] &= 0x00FFFFFF;	//PB7 -> AF0 PB6 -> AF0

		//config gpio A to Alternative
		temp_gpio = GPIOA->MODER;		//2 bits por pin
		temp_gpio &= 0xFFC3FFFF;		//PA9 PA10 alternative
		temp_gpio |= 0x00280000;		//
		GPIOA->MODER = temp_gpio;

		GPIOA->AFR[1] |= 0x00000110;	//PA10 -> AF1 PA9 -> AF1

		usart_mode = USART_GSM_MODE;
	}

	USART1->CR1 = temp_int;
	return usart_mode;
}

unsigned char ReadUsart1Buffer (unsigned char * bout, unsigned short max_len)
{
	unsigned int len;

	len = prx1 - rx1buff;

	if (len < max_len)
	{
		//el prx1 siempre llega adelantado desde la int, lo corto con un 0
		*prx1 = '\0';
		prx1++;
		len += 1;
		memcpy(bout, (unsigned char *) rx1buff, len);
	}
	else
	{
		memcpy(bout, (unsigned char *) rx1buff, len);
		len = max_len;
	}

	//ajusto punteros de rx luego de la copia
	prx1 = rx1buff;

	return (unsigned char) len;
}

void USART1_IRQHandler(void)
{
	unsigned char dummy;

	/* USART in mode Receiver --------------------------------------------------*/
	if (USART1->ISR & USART_ISR_RXNE)
	{
		dummy = USART1->RDR & 0x0FF;

		//RX GPS & GSM
		if ((usart_mode == USART_GPS_MODE) || (usart_mode == USART_GSM_MODE))
		{
			if (prx1 < &rx1buff[SIZEOF_DATA])
			{
				*prx1 = dummy;
				prx1++;
				usart1_have_data = 1;
			}
			usart1_mini_timeout = TT_GSM_MINI;
		}

//		//RX GSM
//		if (usart_mode == USART_GSM_MODE)
//		{
//			if (prx1 < &rx1buff[SIZEOF_DATA])
//			{
//				*prx1 = dummy;
//				prx1++;
//				gsm_have_data = 1;
//			}
//			gsm_mini_timeout = TT_GSM_MINI;
//		}
	}

	/* USART in mode Transmitter -------------------------------------------------*/

	if (USART1->CR1 & USART_CR1_TXEIE)
	{
		if (USART1->ISR & USART_ISR_TXE)
		{
			if ((ptx1 < &tx1buff[SIZEOF_DATA]) && (ptx1 < ptx1_pckt_index))
			{
				USART1->TDR = *ptx1;
				ptx1++;
			}
			else
			{
				ptx1 = tx1buff;
				ptx1_pckt_index = tx1buff;
				USART1->CR1 &= ~USART_CR1_TXEIE;
			}
		}
	}

	if ((USART1->ISR & USART_ISR_ORE) || (USART1->ISR & USART_ISR_NE) || (USART1->ISR & USART_ISR_FE))
	{
		USART1->ICR |= 0x0e;
		dummy = USART1->RDR;
	}
}

void USART2_IRQHandler(void)
{
	unsigned char dummy;

	/* USART in mode Receiver --------------------------------------------------*/
	if (USART2->ISR & USART_ISR_RXNE)
	{
		//RX WIFI
		dummy = USART2->RDR & 0x0FF;

#ifdef USE_GSM_GATEWAY
		if (prx2 < &rx2buff[SIZEOF_DATA])
		{
			*prx2 = dummy;
			prx2++;
			usart2_have_data = 1;
		}
			usart2_mini_timeout = TT_GPS_MINI;
#endif

	}

	/* USART in mode Transmitter -------------------------------------------------*/

	if (USART2->CR1 & USART_CR1_TXEIE)
	{
		if (USART2->ISR & USART_ISR_TXE)
		{
			if ((ptx2 < &tx2buff[SIZEOF_DATA]) && (ptx2 < ptx2_pckt_index))
			{
				USART2->TDR = *ptx2;
				ptx2++;
			}
			else
			{
				ptx2 = tx2buff;
				ptx2_pckt_index = tx2buff;
				USART2->CR1 &= ~USART_CR1_TXEIE;
			}
		}
	}

	if ((USART2->ISR & USART_ISR_ORE) || (USART2->ISR & USART_ISR_NE) || (USART2->ISR & USART_ISR_FE))
	{
		USART2->ICR |= 0x0e;
		dummy = USART2->RDR;
	}
}

void Usart2Send (char * send)
{
	unsigned char i;

	i = strlen(send);
	Usart2SendUnsigned((unsigned char *) send, i);
}

void Usart2SendUnsigned(unsigned char * send, unsigned char size)
{
	if ((ptx2_pckt_index + size) < &tx2buff[SIZEOF_DATA])
	{
		memcpy((unsigned char *)ptx2_pckt_index, send, size);
		ptx2_pckt_index += size;
		USART2->CR1 |= USART_CR1_TXEIE;
	}
}

void Usart2SendSingle(unsigned char tosend)
{
	Usart2SendUnsigned(&tosend, 1);
}

unsigned char ReadUsart2Buffer (unsigned char * bout, unsigned short max_len)
{
	unsigned int len;

	len = prx2 - rx2buff;

	if (len < max_len)
		memcpy(bout, (unsigned char *) rx2buff, len);
	else
	{
		memcpy(bout, (unsigned char *) rx2buff, len);
		len = max_len;
	}

	//ajusto punteros de rx luego de la copia
	prx2 = rx2buff;

	return (unsigned char) len;
}

void Usart1Send (char * send)
{
	unsigned char i;

	i = strlen(send);
	Usart1SendUnsigned((unsigned char *) send, i);
}

void Usart1SendUnsigned(unsigned char * send, unsigned char size)
{
	if ((ptx1_pckt_index + size) < &tx1buff[SIZEOF_DATA])
	{
		memcpy((unsigned char *)ptx1_pckt_index, send, size);
		ptx1_pckt_index += size;
		USART1->CR1 |= USART_CR1_TXEIE;
	}
}

void Usart1SendSingle(unsigned char tosend)
{
	Usart1SendUnsigned(&tosend, 1);
}


void USART2Config(void)
{
	unsigned int temp;

	if (!USART2_CLK)
		USART2_CLK_ON;

	temp = GPIOA->AFR[0];
	temp &= 0xFFF00FF;
	temp |= 0x0001100;	//PA2 -> AF1 PA3 -> AF1
	GPIOA->AFR[0] = temp;

	ptx2 = tx2buff;
	ptx2_pckt_index = tx2buff;
	prx2 = rx2buff;

	USART2->BRR = USART_9600;
	USART2->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 7);
}

void USART1Config(void)
{
	unsigned int temp;

	if (!USART1_CLK)
		USART1_CLK_ON;

#ifdef VER_1_0
	//para empezar con el GPS
	//GPIOB->AFR[0] |= 0x00000000;	//PB7 -> AF0 PB6 -> AF0
	GPIOB->AFR[0] &= 0x00FFFFFF;	//PB7 -> AF0 PB6 -> AF0
	//para empezar con el GSM
	//GPIOA->AFR[1] |= 0x00000110;	//PA10 -> AF1 PA9 -> AF1

	// temp = GPIOA->AFR[0];
	// temp &= 0xFFF00FF;
	// temp |= 0x0001100;	//PA2 -> AF1 PA3 -> AF1
	// GPIOA->AFR[0] = temp;

#endif

	ptx1 = tx1buff;
	ptx1_pckt_index = tx1buff;
	prx1 = rx1buff;

	USART1->BRR = USART_9600;
//	USART1->CR2 |= USART_CR2_STOP_1;	//2 bits stop
//	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
//	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE;	//SIN TX
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;	//para pruebas TX

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 5);
}


//--- end of file ---//
