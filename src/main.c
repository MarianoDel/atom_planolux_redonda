/**
  ******************************************************************************
  * @file    Template_2/main.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Use this template for new projects with stm32f0xx family.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "tim.h"
#include "uart.h"
#include "hard.h"

#include "core_cm0.h"
#include "adc.h"
#include "flash_program.h"
#include "main_menu.h"
#include "synchro.h"


#include "tcp_transceiver.h"

//Para MQTT
#ifdef USE_GPS
#include "MQTTPacket.h"
#include "MQTTConnect.h"
#endif

//Para Hardware de GPS
#ifdef USE_GPS
#include "gps_vktel.h"
#endif

//Para Hardware de GSM
#if (defined USE_GSM) || (defined USE_GSM_GATEWAY)
#include "sim900_800.h"
#endif

//--- VARIABLES EXTERNAS ---//



// ------- Externals del Puerto serie  -------
volatile unsigned char tx2buff[SIZEOF_DATA];
volatile unsigned char rx2buff[SIZEOF_DATA];

volatile unsigned char tx1buff[SIZEOF_DATA];
volatile unsigned char rx1buff[SIZEOF_DATA];

//
//volatile unsigned char data1[SIZEOF_DATA1];
////static unsigned char data_back[10];
//volatile unsigned char data[SIZEOF_DATA];

// ------- Externals de los timers -------
//volatile unsigned short prog_timer = 0;
//volatile unsigned short mainmenu_timer = 0;
volatile unsigned short show_select_timer = 0;
volatile unsigned char switches_timer = 0;
volatile unsigned char acswitch_timer = 0;

volatile unsigned short scroll1_timer = 0;
volatile unsigned short scroll2_timer = 0;

volatile unsigned short standalone_timer;
volatile unsigned short standalone_enable_menu_timer;
//volatile unsigned short standalone_menu_timer;
volatile unsigned char grouped_master_timeout_timer;
volatile unsigned short take_temp_sample = 0;
volatile unsigned char timer_wifi_bright = 0;

#ifdef USE_REDONDA_BASIC
volatile unsigned short timer_relay = 0;			//para relay default (si no hay synchro)

volatile unsigned short tt_take_photo_sample;
volatile unsigned short tt_relay_on_off;
#endif


unsigned char saved_mode;


// ------- Externals de los switches -------
unsigned short s1;
unsigned short s2;
unsigned short sac;
unsigned char sac_aux;

// ------- Externals de los switches -------
#ifdef ADC_WITH_INT
volatile unsigned short adc_ch[3];
volatile unsigned char seq_ready = 0;
unsigned short zero_current;
#endif

// ------- Externals del GPS & GSM -------
volatile unsigned char usart1_mini_timeout;
volatile unsigned char usart1_pckt_ready;
volatile unsigned char usart1_have_data;
unsigned char usart1_pckt_bytes;

#define gps_mini_timeout	usart1_mini_timeout
#define gps_pckt_ready		usart1_pckt_ready
#define gps_have_data		usart1_have_data
#define gps_pckt_bytes		usart1_pckt_bytes

#ifdef USE_GPS
unsigned char gps_buff [SIZEOF_GPSBUFF];
volatile unsigned char usart2_mini_timeout;
volatile unsigned char usart2_pckt_ready;
volatile unsigned char usart2_have_data;
unsigned char usart2_pckt_bytes;

#endif

// ------- Externals del GSM -------
#if (defined USE_GSM) || (defined USE_GSM_GATEWAY)
#define gsm_mini_timeout	usart1_mini_timeout
#define gsm_pckt_ready		usart1_pckt_ready
#define gsm_have_data		usart1_have_data
#define gsm_pckt_bytes		usart1_pckt_bytes

volatile unsigned char usart2_mini_timeout;
volatile unsigned char usart2_pckt_ready;
volatile unsigned char usart2_have_data;
unsigned char usart2_pckt_bytes;

//unsigned char AlertasReportar[5] = {0,0,0,0,0};
//unsigned char ActDact = 0;
//unsigned char claveAct[5] = {0,0,0,0,0};
//volatile char USERCODE[8] = "123456";
extern volatile char buffUARTGSMrx2[];
#endif

//--- VARIABLES GLOBALES ---//
parameters_typedef param_struct;

// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
volatile unsigned short tcp_kalive_timer;
//volatile unsigned char display_timer;
volatile unsigned char filter_timer;

//volatile unsigned char door_filter;
//volatile unsigned char take_sample;
//volatile unsigned char move_relay;
volatile unsigned short secs = 0;
volatile unsigned char hours = 0;
volatile unsigned char minutes = 0;

#define SIZEOF_POWER_VECT		10

unsigned short power_vect [SIZEOF_POWER_VECT];

//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);

// ------- del DMX -------
extern void EXTI4_15_IRQHandler(void);
#define DMX_TIMEOUT	20

//--- FILTROS DE SENSORES ---//
#define LARGO_FILTRO 16
#define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
unsigned short vtemp [LARGO_FILTRO + 1];
unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//

// #define KW			0.009721
// #define KW			0.00945
// #define KW			0.00959
#define KW			0.01013


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
	unsigned char i, ii;
	unsigned char bytes_remain, bytes_read, need_ack = 0;
	unsigned char resp = RESP_CONTINUE;
	unsigned short power_int, power_dec;
	unsigned short power;
	float fcalc = 1.0;
	unsigned int zero_current_loc = 0;

#ifdef USE_REDONDA_BASIC
	main_state_t main_state = MAIN_INIT;
#ifdef WITH_HYST
	unsigned short hyst;
#endif
#ifdef WITH_1_TO_10_VOLTS
	unsigned char one_to_ten;
#endif

#else		//USE_REDONDA_BASIC
	unsigned char main_state = 0;
#endif
	char s_lcd [20];
	enum TcpMessages tcp_msg = NONE_MSG;
	unsigned char new_room = 0;
	unsigned char new_lamp = 0;
	unsigned char last_bright = 0;
	unsigned char show_ldr = 0;
	int dummy_resp = 0;
	unsigned char pps_one = 0;

#ifdef USE_PROD_PROGRAM
	unsigned char jump_the_menu = 0;
#endif
	parameters_typedef * p_mem_init;
	//!< At this stage the microcontroller clock setting is already configured,
    //   this is done through SystemInit() function which is called from startup
    //   file (startup_stm32f0xx.s) before to branch to application main.
    //   To reconfigure the default setting of SystemInit() function, refer to
    //   system_stm32f0xx.c file

	//GPIO Configuration.
	GPIO_Config();


	//ACTIVAR SYSTICK TIMER
	if (SysTick_Config(48000))
	{
		while (1)	/* Capture error */
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			for (i = 0; i < 255; i++)
			{
				asm (	"nop \n\t"
						"nop \n\t"
						"nop \n\t" );
			}
		}
	}


	//ADC Configuration
//	AdcConfig();

	//TIM Configuration.
	// TIM_3_Init();
//	TIM_14_Init();
//	TIM_16_Init();		//para OneShoot() cuando funciona en modo master
//	TIM_17_Init();		//lo uso para el ADC de Igrid

//	EXTIOff ();

	// while (1)
	// {
	// 	// if (SYNCP)
	// 	// {
	// 	// 	SYNCP_OFF;
	// 	// 	LED_OFF;
	// 	// }
	// 	// else
	// 	// {
	// 	// 	SYNCP_ON;
	// 	// 	LED_ON;
	// 	// }
	// 	// Wait_ms(10);
	//
	// 	// for (i = 0; i < 255; i++)
	// 	// {
	// 	// 	Update_TIM3_CH1 (i);
	// 	// 	Wait_ms (10);
	// 	// }
	// }

//		while (1)
//		{
//			PIN3_OFF;
//			Wait_ms (10);
//			PIN3_ON;
//			Wait_ms (10);
//		}

	//--- Welcome code ---//
	LED_OFF;
//	EN_GPS_OFF;
	EN_GPS_ON;
	//RELAY_ON;
	RELAY_OFF;

	USART1Config();
	USART2Config();

	EXTIOff();

#ifdef USE_REDONDA_BASIC
//---------- Inicio Programa de Produccion Redonda Basic --------//
	// USART1Config();
	AdcConfig();		//recordar habilitar sensor en adc.h

#ifdef WITH_1_TO_10_VOLTS
	TIM_3_Init ();					//lo tuilizo para 1 a 10V y para synchro ADC
#endif

	TIM_16_Init();					//o tuilizo para synchro de relay
	TIM16Enable();

	Usart2Send((char *) (const char *) "\r\nKirno Placa Redonda - Basic V1.0\r\n");
	Usart2Send((char *) (const char *) "  Features:\r\n");
	#ifdef WITH_1_TO_10_VOLTS
	Usart2Send((char *) (const char *) "  Dimmer 1 to 10V\r\n");
	#endif
	#ifdef WITH_HYST
	Usart2Send((char *) (const char *) "  Night Hysteresis\r\n");
	#endif
	#ifdef WITH_TEMP_CONTROL
	Usart2Send((char *) (const char *) "  Temp Control\r\n");
	#endif
	#ifdef USE_WITH_SYNC
	Usart2Send((char *) (const char *) "  Sync by Edges\r\n");
	#else
	Usart2Send((char *) (const char *) "  Sync by ADC\r\n");
	#endif


	for (i = 0; i < 8; i++)
	{
		if (LED)
			LED_OFF;
		else
			LED_ON;

		Wait_ms (250);
	}

	timer_standby = 2000;

//--- Programa de pruebas 1 a 10V -----
	// while (1)
	// {
	// 	for (i = 0; i < 255; i++)
	// 	{
	// 		Update_TIM3_CH1(i);
	// 		Wait_ms(10);
	// 	}
	//
	// 	for (i = 255; i >= 0; i--)
	// 	{
	// 		Update_TIM3_CH1(i);
	// 		Wait_ms(10);
	// 	}
	// }
//--- FIN Programa de pruebas 1 a 10V -----

//--- Programa de pruebas synchro de Relay -----
	// i = 0;
	// while (1)
	// {
	// 	switch (i)
	// 	{
	// 		case 0:
	// 			RelayOn();
	// 			timer_standby = 50;
	// 			LED_ON;
	// 			i++;
	// 			break;
	//
	// 		case 1:
	// 			if (!timer_standby)
	// 			{
	// 				RelayOff();
	// 				LED_OFF;
	// 				i++;
	// 				timer_standby = 10000;
	// 			}
	// 			break;
	//
	// 		case 2:
	// 			if (!timer_standby)
	// 			{
	// 				i = 0;
	// 			}
	// 			break;
	// 	}
	//
	// 	UpdateRelay ();
	// }
//--- FIN Programa de pruebas synchro de Relay -----


//--- Programa de pruebas I meas -----
	while (1)
	{
		switch (main_state)
		{
			case MAIN_INIT:
				Update_TIM3_CH1 (10);		//lo uso para ver diff entre synchro adc con led
				main_state = SYNCHRO_ADC;
				ADC1->CR |= ADC_CR_ADSTART;
				seq_ready = 0;
				break;

			case SYNCHRO_ADC:
				if (seq_ready)
				{
					main_state = SET_ZERO_CURRENT;
					Usart2Send((char *) (const char *) "Getted\r\n");
					Usart2Send((char *) (const char *) "Waiting GSM Startup and zero current\r\n");
					timer_standby = 60000;
					zero_current_loc = 0;
					i = 0;
				}
				break;

			case SET_ZERO_CURRENT:
				if (!timer_standby)
				{
					if (i < 32)
					// if (i < 4)
					{
						if (seq_ready)
						{
							seq_ready = 0;
							zero_current_loc += I_Sense;
							i++;
							timer_standby = 2;	//cargo valor zero_current en 64ms
						}
					}
					else
					{
						zero_current_loc >>= 5;
						// zero_current_loc >>= 2;
						zero_current = zero_current_loc;
						main_state = LAMP_ON;
						RELAY_ON;
						i = 0;
					}
				}
				break;

			case LAMP_OFF:
				break;

			case LAMP_ON:
				if (!timer_standby)
				{
					if (i < SIZEOF_POWER_VECT)
					{
						power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
						i++;
					}
					else
					{
						//termine de cargar el vector, muestro info
						power = PowerCalcMean8(power_vect);
						power_2secs_acum += power;
						power_2secs_index++;
						fcalc = power;
						fcalc = power * KW;
						power_int = (unsigned short) fcalc;
						fcalc = fcalc - power_int;
						fcalc = fcalc * 100;
						power_dec = (unsigned short) fcalc;
						sprintf(s_lcd, "p: %3d.%02d d: %d\r\n", power_int, power_dec, power);

						// sprintf(s_lcd, "z: %d, v: %d, i: %d\r\n", zero_current, V_Sense, I_Sense);
						// sprintf(s_lcd, "z: %d, v: %d, i: %d\r\n", zero_current, GetVGrid(), GetIGrid());
						//sprintf(s_lcd, "temp: %d, photo: %d\r\n", GetTemp(), ReadADC1_SameSampleTime (ADC_CH1));
						Usart2Send(s_lcd);
						i = 0;

						if (power_2secs_index >= 30)	//1 a 30 es el contador
						{
							power_2secs_index = 0;
							power_minutes += power;		//TODO: supongo todo el minuto el mismo consumo ver power_2secs_acum
							power_minutes_index++;
						}

						if (power_minutes_index >= 60)	//1 a 60 es el contador
						{
							power_minutes_index = 0;
							power_hours += power_minutes;		//TODO: OJO se supuso todo el minuto el mismo consumo
							power_minutes = 0;
						}

						
					}
					timer_standby = 200;		//10 veces 200ms

					// fcalc = voltage;
					// fcalc = fcalc * KV;
					// volt_int = (short) fcalc;
					// fcalc = fcalc - volt_int;
					// fcalc = fcalc * 100;
					// volt_dec = (short) fcalc;
					//
					// sprintf(str, "%2d.%02d", volt_int, volt_dec);
					//
					// //sprintf(str, "%4d        ", voltage);
					// LCDTransmitStr(str);

				}

				break;

			default:
				main_state = MAIN_INIT;
				break;
		}

		//Cosas que dependen de las muestras
		if (seq_ready)
		{
			seq_ready = 0;
			UpdateVGrid ();
			UpdateIGrid ();
		}

		//Cosas que no dependen del estado del programa
		UpdateRelay ();
		// UpdatePhotoTransistor();
	}	//end while 1

//--- FIN Programa de pruebas I meas -----

	while (1)
	{
		switch (main_state)
		{
			case MAIN_INIT:
				RelayOff();
				LED_OFF;
				FillPhotoBuffer();
#ifdef WITH_TEMP_CONTROL
				FillTempBuffer();
#endif
#ifdef WITH_1_TO_10_VOLTS
				Update_TIM3_CH1 (0);
#endif
				main_state = SYNCHRO_ADC;
#ifdef ADC_WITH_INT
				seq_ready = 0;
#endif
				break;

			case SYNCHRO_ADC:
#ifdef ADC_WITH_INT
				if (seq_ready)
#endif
				{
					main_state = SET_ZERO_CURRENT;
				}
				break;

			case SET_ZERO_CURRENT:
				main_state = LAMP_OFF;
				break;

			case LAMP_OFF:
				if (!tt_relay_on_off)
				{
					if (GetPhoto() > VOLTAGE_PHOTO_ON)
					{
						main_state = LAMP_ON;
						tt_relay_on_off = 10000;
	#ifdef WITH_1_TO_10_VOLTS
						Update_TIM3_CH1 (PWM_MIN);
	#endif

						RelayOn();
						LED_ON;
	#ifdef WITH_HYST
						hours = 0;
	#endif
					}
				}
				break;

			case LAMP_ON:
				if (!tt_relay_on_off)
				{
	#ifdef WITH_HYST		//con Hysteresis apaga casi en el mismo punto en el que prende
					hyst = GetHysteresis (hours);
					if (GetPhoto() < (VOLTAGE_PHOTO_ON - hyst))
	#else
					if (GetPhoto() < VOLTAGE_PHOTO_OFF)
	#endif
					{
						main_state = LAMP_OFF;
	#ifdef WITH_1_TO_10_VOLTS
						Update_TIM3_CH1 (0);
	#endif
						tt_relay_on_off = 10000;
						RelayOff();
						LED_OFF;
					}
				}

	#ifdef WITH_1_TO_10_VOLTS
				if (main_state == LAMP_ON)
				{
					one_to_ten = GetNew1to10 (GetPhoto());
					Update_TIM3_CH1 (one_to_ten);
				}
	#endif
				break;

			default:
				main_state = MAIN_INIT;
				break;
		}

		if (!timer_standby)
		{
#ifdef WITH_TEMP_CONTROL
			sprintf(s_lcd, "temp: %d, photo: %d\r\n", GetTemp(), GetPhoto());
#else
			sprintf(s_lcd, "photo: %d\r\n", GetPhoto());
#endif
			//sprintf(s_lcd, "temp: %d, photo: %d\r\n", GetTemp(), ReadADC1_SameSampleTime (ADC_CH1));
			Usart2Send(s_lcd);
			timer_standby = 2000;
		}

		//Cosas que no dependen del estado del programa
		UpdateRelay ();
#ifdef WITH_TEMP_CONTROL
		UpdateTemp();
#endif
		UpdatePhotoTransistor();
	}	//end while 1
//---------- Fin Programa de Procduccion Redonda Basic--------//
#endif	//USE_REDONDA_BASIC


#ifdef USE_MQTT_LIB
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	int rc = 0;
	char buf[200];
	MQTTString topicString = MQTTString_initializer;
	char* payload = "mypayload";
	int payloadlen = strlen(payload);int buflen = sizeof(buf);
	int len = 0;

	data.clientID.cstring = "me";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	len = MQTTSerialize_connect(buf, buflen, &data); /* 1 */

	topicString.cstring = "mytopic";
	len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, topicString, payload, payloadlen); /* 2 */

	len += MQTTSerialize_disconnect(buf + len, buflen - len); /* 3 */
	//falta abrir puerto
	//falta enviar al socket
	//falta cerrar socket
#endif

//	//---------- Prueba USART2 --------//
//
//    while( 1 )
//    {
//    	Usart2Send((char *) (const char *) "Kirno debug placa redonda\r\n");
//        Wait_ms(3000);
//    }
//
//    //---------- Fin Prueba USART2 --------//

	//---------- Prueba con GPS --------//
#ifdef USE_GPS
	Usart2SendSingle('M');
	Usart2Send((char *) (const char *) "Kirno debug placa redonda\r\n");
	Wait_ms(1000);

	Usart1Mode (USART_GPS_MODE);

	//mando reset al gps
	Usart2Send((char *) (const char *) "Reset de GPS\r\n");
	GPSStartResetSM ();
	while (GPSStart() != RESP_OK);

	//mando conf al gps
	Usart2Send((char *) (const char *) "Config al GPS\r\n");
	GPSConfigResetSM ();
	while (GPSConfig() != RESP_OK);

//	//mando reset factory al gps
//	Usart2Send((char *) (const char *) "GPS a Factory Default\r\n");
//	GPSResetFactoryResetSM ();
//	while (GPSResetFactory() != RESP_OK);

	Usart2Send((char *) (const char *) "Espero datos de posicion\r\n");
//	timer_standby = 60000;
//	while( timer_standby )
	while( 1 )
	{
		if (gps_pckt_ready)
		{
			gps_pckt_ready = 0;
			//Usart2SendSingle('P');
			Usart2Send("\r\nP:\r\n");
			Usart2SendUnsigned(gps_buff, gps_pckt_bytes);
		}

		GPSProcess();
	}
#endif
	//---------- Fin Prueba con GPS --------//

	//---------- Prueba con GSM --------//
#ifdef USE_GSM
	Usart2Send((char *) (const char *) "Cambio a GSM\r\n");

	Usart1Mode (USART_GSM_MODE);


	//Pruebo USART1
//	while (1)
//	{
//			Usart1SendUnsigned((unsigned char *) (const char *) "Test OK\r\n", sizeof("Test OK\r\n"));
//			Wait_ms(50);
//	}


	//mando start al gsm
	Usart2Send((char *) (const char *) "Reset y Start GSM\r\n");
	//GPSStartResetSM ();
	timer_standby = 60000;		//doy 1 minuto para prender modulo
	while (timer_standby)
	{
		i = GSM_Start();
		if (i == 2)
		{
			Usart2Send((char *) (const char *) "Start OK\r\n");
			timer_standby = 0;
		}

		if (i == 4)
			Usart2Send((char *) (const char *) "Start NOK\r\n");
	}

	//mando conf al gsm
	Usart2Send((char *) (const char *) "Config al GSM\r\n");
	//GPSConfigResetSM ();

	i = 0;
	while (i == 0)
	{
		ii = GSM_Config(1000);

		if (ii == 2)
			i = 0;
		else if (ii > 2)
		{
			Usart2Send((const char*) "Error en configuracion\r\n");
			while (1);
		}

		GSMProcess();
		GSMReceive ();

		if (gsm_pckt_ready)
		{
			gsm_pckt_ready = 0;
			Usart2SendUnsigned(buffUARTGSMrx2, gsm_pckt_bytes);
		}

		if (LIGHT)
			LED_ON;
		else
			LED_OFF;
	}


	while( 1 )
	{
		if (gsm_pckt_ready)
		{
			gsm_pckt_ready = 0;
			Usart2SendUnsigned(buffUARTGSMrx2, gsm_pckt_bytes);
		}

		GSMProcess();

		if (LIGHT)
			LED_ON;

	}
#endif

	//---------- Pruebas con GSM GATEWAY --------//
#ifdef USE_GSM_GATEWAY
	LED_OFF;
	for (i = 0; i < 6; i++)
	{
		if (LED)
			LED_OFF;
		else
			LED_ON;

		Wait_ms (300);
	}
	Wait_ms (3000);

	Usart2Send((char *) (const char *) "GSM GATEWAY.. Cambio a GSM\r\n");

	Usart1Mode (USART_GSM_MODE);


	//mando start al gsm
	Usart2Send((char *) (const char *) "Reset y Start GSM\r\n");
	//GPSStartResetSM ();
	timer_standby = 60000;		//doy 1 minuto para prender modulo
	while (timer_standby)
	{
		i = GSM_Start();
		if (i == 2)
		{
			Usart2Send((char *) (const char *) "Start OK\r\n");
			timer_standby = 0;
		}
		else

		if (i == 4)
			Usart2Send((char *) (const char *) "Start NOK\r\n");
	}

	Usart2Send((char *) (const char *) "GSM GATEWAY Listo para empezar\r\n");

	while (1)
	{
		GSMProcess();

		if (usart2_pckt_ready)	//deja paquete en buffUARTGSMrx2
		{
			usart2_pckt_ready = 0;
			Usart1SendUnsigned((unsigned char *) buffUARTGSMrx2, usart2_pckt_bytes);
		}

		if (gsm_pckt_ready)		//deja paquete en buffUARTGSMrx2
		{
			gsm_pckt_ready = 0;
			Usart2SendUnsigned((unsigned char *) buffUARTGSMrx2, gsm_pckt_bytes);
		}

		if (LIGHT)
			LED_ON;
		else
			LED_OFF;
	}
#endif

	//---------- Fin Prueba con GSM GATEWAY --------//



	//---------- Prueba temp --------//
	/*
	while (1)
	{
		local_meas = GetTemp();
		if (local_meas != local_meas_last)
		{
			LED_ON;
			local_meas_last = local_meas;
			LCD_2DO_RENGLON;
			LCDTransmitStr((const char *) "Brd Temp:       ");
			local_meas = ConvertTemp(local_meas);
			sprintf(s_lcd, "%d", local_meas);
			Lcd_SetDDRAM(0x40 + 10);
			LCDTransmitStr(s_lcd);
			LED_OFF;
		}

		UpdateTemp();
	}
	*/
	//---------- Fin prueba temp --------//

	//---------- Prueba 1 to 10V --------//
	/*
	local_meas = 0;
	while (1)
	{
		LCD_2DO_RENGLON;
		LCDTransmitStr((const char *) "1 to 10V:       ");
		fcalc = local_meas;
		fcalc = fcalc * K_1TO10;
		one_int = (short) fcalc;
		fcalc = fcalc - one_int;
		fcalc = fcalc * 10;
		one_dec = (short) fcalc;

		sprintf(s_lcd, "%02d.%01d V", one_int, one_dec);
		Lcd_SetDDRAM(0x40 + 10);
		LCDTransmitStr(s_lcd);

		Wait_ms (1000);
		if (local_meas <= 255)
			local_meas = 0;
		else
			local_meas++;
	}
	*/
	//---------- Fin prueba 1 to 10V --------//


	//---------- Fin Programa de Procduccion --------//

	return 0;
}

//--- End of Main ---//




void prepare_json_pkt (uint8_t * buffer)
{
      int32_t d1 = 1, d2 = 2, d3 = 3, d4 = 4, d5 = 5, d6 = 6;
      char tempbuff[40];
      volatile float HUMIDITY_Value;
      volatile float TEMPERATURE_Value;
      volatile float PRESSURE_Value;



      strcpy((char *)buffer,"{\"d\":{\"myName\":\"Nucleo\"");
//      BSP_HUM_TEMP_GetTemperature((float *)&TEMPERATURE_Value);
//      floatToInt(TEMPERATURE_Value, &d1, &d2, 2);
      sprintf(tempbuff, ",\"A_Temperature\":%lu.%lu",d1, d2);
      strcat((char *)buffer,tempbuff);

//      BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
//      floatToInt(HUMIDITY_Value, &d3, &d4, 2);
      sprintf(tempbuff, ",\"A_Humidity\":%lu.%lu",d3,d4 );
      strcat(  (char *)buffer,tempbuff);

//      BSP_PRESSURE_GetPressure((float *)&PRESSURE_Value);
//      floatToInt(PRESSURE_Value, &d5, &d6, 2);
      sprintf(tempbuff, ",\"A_Pressure\":%lu.%lu",d5,d6 );
      strcat((char *)buffer,tempbuff);


      strcat((char *)buffer,"}}");

      return;
}

//void EXTI4_15_IRQHandler(void)
//{
//	unsigned short aux;
//
////--- SOLO PRUEBA DE INTERRUPCIONES ---//
////	if (DMX_INPUT)
////		LED_ON;
////	else
////		LED_OFF;
////
////	EXTI->PR |= 0x0100;
//
//	if(EXTI->PR & 0x0100)	//Line8
//	{
//
//		//si no esta con el USART detecta el flanco	PONER TIMEOUT ACA?????
//		if ((dmx_receive_flag == 0) || (dmx_timeout_timer == 0))
//		//if (dmx_receive_flag == 0)
//		{
//			switch (signal_state)
//			{
//				case IDLE:
//					if (!(DMX_INPUT))
//					{
//						//Activo timer en Falling.
//						TIM14->CNT = 0;
//						TIM14->CR1 |= 0x0001;
//						signal_state++;
//					}
//					break;
//
//				case LOOK_FOR_BREAK:
//					if (DMX_INPUT)
//					{
//						//Desactivo timer en Rising.
//						aux = TIM14->CNT;
//
//						//reviso BREAK
//						//if (((tim_counter_65ms) || (aux > 88)) && (tim_counter_65ms <= 20))
//						if ((aux > 87) && (aux < 210))	//Consola STARLET 6
//						//if ((aux > 87) && (aux < 2000))		//Consola marca CODE tiene break 1.88ms
//						{
//							LED_ON;
//							//Activo timer para ver MARK.
//							//TIM2->CNT = 0;
//							//TIM2->CR1 |= 0x0001;
//
//							signal_state++;
//							//tengo el break, activo el puerto serie
//							DMX_channel_received = 0;
//							//dmx_receive_flag = 1;
//
//							dmx_timeout_timer = DMX_TIMEOUT;		//activo el timer cuando prendo el puerto serie
//							//USARTx_RX_ENA;
//						}
//						else	//falso disparo
//							signal_state = IDLE;
//					}
//					else	//falso disparo
//						signal_state = IDLE;
//
//					TIM14->CR1 &= 0xFFFE;
//					break;
//
//				case LOOK_FOR_MARK:
//					if ((!(DMX_INPUT)) && (dmx_timeout_timer))	//termino Mark after break
//					{
//						//ya tenia el serie habilitado
//						//if ((aux > 7) && (aux < 12))
//						dmx_receive_flag = 1;
//					}
//					else	//falso disparo
//					{
//						//termine por timeout
//						dmx_receive_flag = 0;
//						//USARTx_RX_DISA;
//					}
//					signal_state = IDLE;
//					LED_OFF;
//					break;
//
//				default:
//					signal_state = IDLE;
//					break;
//			}
//		}
//
//		EXTI->PR |= 0x0100;
//	}
//}

void TimingDelay_Decrement(void)
{
	if (wait_ms_var)
		wait_ms_var--;

//	if (display_timer)
//		display_timer--;

	if (timer_standby)
		timer_standby--;

	if (acswitch_timer)
		acswitch_timer--;

	if (tt_take_photo_sample)
		tt_take_photo_sample--;

	if (tt_relay_on_off)
		tt_relay_on_off--;

#ifdef ADC_WITH_TEMP_SENSE
	if (tt_take_temp_sample)
		tt_take_temp_sample--;
#endif

	if (take_temp_sample)
		take_temp_sample--;

	if (filter_timer)
		filter_timer--;

	//cuenta de a 1 minuto
	if (secs > 59999)	//pasaron 1 min
	{
		minutes++;
		secs = 0;
	}
	else
		secs++;

	if (minutes > 60)
	{
		hours++;
		minutes = 0;
	}


#ifdef USE_MQTT_LIB
	//timer del MQTT
	SysTickIntHandler();
#endif

#if (defined USE_GPS) || (defined USE_GSM) || (defined USE_GSM_GATEWAY)
	if (usart1_mini_timeout)
		usart1_mini_timeout--;
	if (usart2_mini_timeout)
		usart2_mini_timeout--;
#endif
#ifdef USE_GPS
	GPSTimeoutCounters ();
#endif

#if (defined USE_GSM) || (defined USE_GSM_GATEWAY)
	GSMTimeoutCounters ();
#endif
}

//------ EOF -------//
