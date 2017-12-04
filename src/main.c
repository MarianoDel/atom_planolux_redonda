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
#include "funcs_gsm.h"
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
// volatile unsigned char switches_timer = 0;
//
//
// volatile unsigned short scroll1_timer = 0;
// volatile unsigned short scroll2_timer = 0;
//
// volatile unsigned short standalone_timer;
// volatile unsigned short standalone_enable_menu_timer;
// //volatile unsigned short standalone_menu_timer;
// volatile unsigned char grouped_master_timeout_timer;
volatile unsigned short take_temp_sample = 0;
volatile unsigned short timer_rep = 0;
// volatile unsigned char timer_wifi_bright = 0;

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

extern volatile char buffUARTGSMrx2[];
#endif

parameters_typedef param_struct;

//--- VARIABLES GLOBALES ---//


//para las mediciones
// unsigned int power_2secs_acum = 0;
// unsigned char power_2secs_index = 0;
// unsigned short power_minutes = 0;
// unsigned char power_minutes_index = 0;
// unsigned short power_hours = 0;

//para los msjs GSM
char gsmNUM [20];
char gsmMSG [180];






// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
volatile unsigned short tcp_kalive_timer;
//volatile unsigned char display_timer;
volatile unsigned char filter_timer;

//volatile unsigned char door_filter;
//volatile unsigned char take_sample;
//volatile unsigned char move_relay;
#ifdef WITH_HYST
volatile unsigned short secs = 0;
volatile unsigned char hours = 0;
volatile unsigned char minutes = 0;
#endif

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

//--- Private Definitions ---//
#define num_tel_rep		param_struct.num_reportar


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
	unsigned short wh_int, wh_dec;
	unsigned short power, last_power;
	float fcalc = 1.0;
	unsigned int zero_current_loc = 0;

	unsigned short acum_secs_index;
	unsigned int acum_secs, acum_hours;
	unsigned char show_power_index = 0;
	unsigned char show_power = 0;

#ifdef USE_REDONDA_BASIC
	main_state_t main_state = MAIN_INIT;
	unsigned char reportar_SMS = 0;
	unsigned char sended = 0;
	lamp_on_state_t lamp_on_state = init_airplane0;
	unsigned char counters_mode = 0;
	unsigned char meas_end = 0;

#ifdef WITH_HYST
	unsigned short hyst;
#endif
#ifdef WITH_1_TO_10_VOLTS
	unsigned char one_to_ten;
#endif

#else		//USE_REDONDA_BASIC
	unsigned char main_state = 0;
#endif
	char s_lcd [100];		//lo agrando porque lo uso tambien para enviar SMS
	// enum TcpMessages tcp_msg = NONE_MSG;
	// unsigned char new_room = 0;
	// unsigned char new_lamp = 0;
	// unsigned char last_bright = 0;
	// unsigned char show_ldr = 0;
	// int dummy_resp = 0;
	// unsigned char pps_one = 0;

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

	//--- Leo los parametros de memoria ---//
#ifdef USE_REDONDA_BASIC
	param_struct.acumm_historico = ((parameters_typedef *) (unsigned int *) PAGE63)->acumm_historico;
	if (param_struct.acumm_historico != 0xFFFFFFFF)
	{
		//memoria no vacia
		strncpy( param_struct.num_reportar,
					((parameters_typedef *) (char *) PAGE63)->num_reportar,
					sizeof(param_struct.num_reportar));


		param_struct.timer_reportar = ((parameters_typedef *) (unsigned int *) PAGE63)->timer_reportar;
		reportar_SMS = 1;
	}
	else
	{
		//memoria vacia
		param_struct.acumm_historico = 0;
		param_struct.timer_reportar = 2;
		reportar_SMS = 0;
		strcpy( param_struct.num_reportar, "1149867843");	//segund asim de claro
	}
#endif


	//--- Welcome code ---//
	LED_OFF;
//	EN_GPS_OFF;
	EN_GPS_ON;
	//RELAY_ON;
	RELAY_OFF;

	USART1Config();
	USART2Config();

	EXTIOff();



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
		if (i == 1)
		{
			Usart2Send((char *) (const char *) "Start OK\r\n");
			timer_standby = 0;
		}
		else

		if (i > 1)
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
	#ifdef USE_GSM
	Usart2Send((char *) (const char *) "  Uses GSM for SMS data\r\n");
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
	FuncsGSMReset();
	Usart1Mode(USART_GSM_MODE);

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
				if (seq_ready)					//TODO ojo aca seq_ready se usa fuera del main switch
				{
					Usart2Send((char *) (const char *) "Getted\r\n");

#ifdef USE_GSM
					Usart2Send((char *) (const char *) "Waiting GSM Startup and zero current\r\n");
					main_state = SET_ZERO_CURRENT;
					timer_standby = 0;
					zero_current_loc = 0;
					i = 0;
#else
					main_state = SET_ZERO_CURRENT;
					timer_standby = 60000;
					zero_current_loc = 0;
					i = 0;
#endif
				}
				break;

			case SET_ZERO_CURRENT:
				if (!timer_standby)
				{
					if (i < 32)
					{
						if (seq_ready)		//TODO ojo aca seq_ready se usa fuera del main switch
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
						zero_current = zero_current_loc;
						main_state = SET_COUNTERS_AND_PHONE0;
						RELAY_ON;
						i = 0;
					}
				}
				break;

			case SET_COUNTERS_AND_PHONE0:
				acum_secs = 0;
				acum_secs_index = 0;
				acum_hours = 0;

				counters_mode = 0;

				//espero que el telefono este libre
				//TODO: timeout aca
				if (FuncsGSMStateAsk() == gsm_state_ready)
				{
					Usart2Send((char *) (const char *) "Reports by SMS\r\n");
					main_state = SET_COUNTERS_AND_PHONE1;

					//pido imei
					s_lcd[0] = '\0';
					FuncsGSMCommandAnswer ("AT+GSN\r\n" , s_lcd);
				}
				break;

			case SET_COUNTERS_AND_PHONE1:
				//espero que el telefono este libre
				if (FuncsGSMStateAsk() == gsm_state_ready)
				{
					i = strlen(s_lcd);
					strncpy(param_struct.imei, s_lcd, (i - 2));
					Usart2Send("IMEI: ");
					Usart2Send(param_struct.imei);
					Usart2Send("\r\n");

					//mando SMS con mi info
					strcpy(s_lcd, "IMEI: ");
					strcat(s_lcd, param_struct.imei);
					strcat(s_lcd, ", ACTIVO");

					FuncsGSMSendSMS(s_lcd, param_struct.num_reportar);
					// FuncsGSMSendSMS("forro", param_struct.num_reportar);
					main_state = LAMP_OFF;
				}
				break;

			case LAMP_OFF:
				if (!tt_relay_on_off)
				{
					if (GetPhoto() > VOLTAGE_PHOTO_ON)
					{
#ifdef WITH_1_TO_10_VOLTS
						Update_TIM3_CH1 (PWM_MIN);
#else
						Update_TIM3_CH1 (PWM_MAX);
#endif
						// RelayOn();
						main_state = LAMP_ON;
						tt_relay_on_off = 10000;
						Usart2Send("PRENDIDO\r\n");
						FuncsGSMSendSMS("PRENDIDO", param_struct.num_reportar);

						LED_ON;
#ifdef WITH_HYST
						hours = 0;
#endif
					}
				}
				break;

			case LAMP_ON:
#if defined REPORTS_AIRPLANE_MODE
				switch (lamp_on_state)
				{
					case init_airplane0:
						if (FuncsGSMStateAsk() == gsm_state_ready)
						{
							//lo paso a modo avion
							s_lcd[0] = '\0';
							FuncsGSMCommandAnswer ("AT+CFUN=4\r\n" , s_lcd);
							lamp_on_state = init_airplane1;
						}
						break;

					case init_airplane1:
						if (!strncmp(s_lcd, "OK", sizeof("OK") - 1))
						{
							//en modo avion, prendo y mido
							RelayOn();
							lamp_on_state = meas_init;
						}
						break;

					case meas_init:
						if (RelayIsOn())
						{
							lamp_on_state = meas_meas;
							counters_mode = 1;
						}
						break;

					case meas_meas:
						if (meas_end)
						{
							meas_end = 0;

							if (!tt_relay_on_off)
							{
#ifdef WITH_HYST			//con Hysteresis apaga casi en el mismo punto en el que prende
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
									lamp_on_state = init_airplane0;
									counters_mode = 0;
									Usart2Send("APAGADO\r\n");
									FuncsGSMSendSMS("APAGADO", param_struct.num_reportar);
									tt_relay_on_off = 10000;
									RelayOff();
									LED_OFF;
								}
#ifdef WITH_1_TO_10_VOLTS
								else
								{
									one_to_ten = GetNew1to10 (GetPhoto());
									Update_TIM3_CH1 (one_to_ten);
								}
#endif
							}

							if (!timer_rep)
							{
								timer_rep = param_struct.timer_reportar;
								counters_mode = 2;
								lamp_on_state = meas_reporting0;
							}
						}
						break;

					case meas_reporting0:
						//lo saco de modo avion
						s_lcd[0] = '\0';
						FuncsGSMCommandAnswer ("AT+CFUN=1\r\n" , s_lcd);
						lamp_on_state = meas_reporting1;
						break;

					case meas_reporting1:
						if (!strncmp(s_lcd, "OK", sizeof("OK") - 1))
						{
							if (FuncsGSMStateAsk() == gsm_state_ready)
							{
								// fcalc = power;
								fcalc = power * KW;
								power_int = (unsigned short) fcalc;
								fcalc = fcalc - power_int;
								fcalc = fcalc * 100;
								power_dec = (unsigned short) fcalc;

								fcalc = (acum_hours + acum_secs / 1800) * KW;
								wh_int = (unsigned short) fcalc;
								fcalc = fcalc - wh_int;
								fcalc = fcalc * 10;
								wh_dec = (unsigned short) fcalc;

								sprintf(s_lcd, "pi: %3d.%02d wh: %3d.%01d\r\n", power_int, power_dec, wh_int, wh_dec);

								//TODO: para debug no envio datos
								Usart2Send(s_lcd);
								FuncsGSMSendSMS(s_lcd, param_struct.num_reportar);
							}
							lamp_on_state = init_airplane0;
						}
						break;

					default:
						lamp_on_state = init_airplane0;
						break;
				}

#elif defined REPORTS_NORMAL_MODE
				switch (lamp_on_state)
				{
					case init_airplane0:
						lamp_on_state++;
						break;

					case init_airplane1:
						lamp_on_state++;
						break;

					case meas_init:
						RelayOn();
						lamp_on_state = meas_meas;
						counters_mode = 1;
						break;

					case meas_meas:
						if (meas_end)		//termino una vuelta de mediciones, generalmente 2 segundos
						{
							meas_end = 0;

							if (!tt_relay_on_off)
							{
#ifdef WITH_HYST			//con Hysteresis apaga casi en el mismo punto en el que prende
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
									lamp_on_state = init_airplane0;
									counters_mode = 0;
									Usart2Send("APAGADO");
									FuncsGSMSendSMS("APAGADO", param_struct.num_reportar);
									tt_relay_on_off = 10000;
									RelayOff();
									LED_OFF;
								}
#ifdef WITH_1_TO_10_VOLTS
								else
								{
									one_to_ten = GetNew1to10 (GetPhoto());
									Update_TIM3_CH1 (one_to_ten);
								}
#endif
							}

							if (!timer_rep)
							{
								timer_rep = param_struct.timer_reportar;
								// counters_mode = 2;		//sigo midiendo normalmente
								lamp_on_state = meas_reporting0;
							}
						}
						break;

					case meas_reporting0:

						// fcalc = power;
						fcalc = power * KW;
						power_int = (unsigned short) fcalc;
						fcalc = fcalc - power_int;
						fcalc = fcalc * 100;
						power_dec = (unsigned short) fcalc;

						fcalc = (acum_hours + acum_secs / 1800) * KW;
						wh_int = (unsigned short) fcalc;
						fcalc = fcalc - wh_int;
						fcalc = fcalc * 10;
						wh_dec = (unsigned short) fcalc;

						sprintf(s_lcd, "pi: %3d.%02d wh: %3d.%01d\r\n", power_int, power_dec, wh_int, wh_dec);

						//TODO: para debug no envio datos
						Usart2Send(s_lcd);
						FuncsGSMSendSMS(s_lcd, param_struct.num_reportar);
						lamp_on_state = meas_meas;
						break;

					case meas_reporting1:
						break;

					default:
						lamp_on_state = init_airplane0;
						break;
				}

#else
#error "Debe elegir la forma de reportar Normal / Airplane"
#endif

				if (counters_mode)	//si esta activo el modo de contadores mido
				{
					if (!timer_standby)	//update cada 200ms
					{
						if (i < SIZEOF_POWER_VECT)
						{
							power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
							i++;
						}
						else
						{		//termine de cargar el vector, guardo muestro info
							i = 0;

							if (counters_mode == 1)	//mido normalmente
							{
								power = PowerCalcMean8(power_vect);
								last_power = power;
							}

							if (counters_mode == 2)	//no mido solo update de lo viejo
								power = last_power;

							acum_secs += power;
							acum_secs_index++;
							show_power_index++;

							if (acum_secs_index >= 1800)
							{
								acum_hours += (acum_secs / 1800);	//lo convierto a Wh, para no perder bits en cada cuenta
								acum_secs = 0;
								acum_secs_index = 0;
							}

							//cuando termino una medicion completa aviso con meas_end
							meas_end = 1;


							// if (show_power_index >= 30)
							// {
							// 	show_power = 1;
							// 	show_power_index = 0;
							// }
							//
							// if (show_power)
							// {
							// 	// fcalc = power;
							// 	fcalc = power * KW;
							// 	power_int = (unsigned short) fcalc;
							// 	fcalc = fcalc - power_int;
							// 	fcalc = fcalc * 100;
							// 	power_dec = (unsigned short) fcalc;
							//
							// 	fcalc = (acum_hours + acum_secs / 1800) * KW;
							// 	wh_int = (unsigned short) fcalc;
							// 	fcalc = fcalc - wh_int;
							// 	fcalc = fcalc * 10;
							// 	wh_dec = (unsigned short) fcalc;
							//
							// 	sprintf(s_lcd, "pi: %3d.%02d wh: %3d.%01d\r\n", power_int, power_dec, wh_int, wh_dec);
							//
							// 	//TODO: para debug no envio datos
							// 	Usart2Send(s_lcd);
							//
							// 	show_power = 0;
							// }
						}
						timer_standby = 200;		//10 veces 200ms
					}
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
		UpdatePhotoTransistor();
#ifdef USE_GSM
		FuncsGSM();
#endif
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

void TimingDelay_Decrement(void)
{
	if (wait_ms_var)
		wait_ms_var--;

	if (timer_standby)
		timer_standby--;

#ifdef USE_REDONDA_BASIC
	if (tt_take_photo_sample)
		tt_take_photo_sample--;

	if (tt_relay_on_off)
		tt_relay_on_off--;
#endif

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

		if (timer_rep)
			timer_rep--;	//timer de reportes de a 1 minuto
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
