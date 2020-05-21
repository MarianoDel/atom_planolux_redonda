//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### MAIN.C #######################################
//----------------------------------------------------

// Includes --------------------------------------------------------------------
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
#include "dma.h"
#include "flash_program.h"
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
#if (defined USE_GSM) || (defined USE_GSM_GATEWAY) || (defined USE_ONLY_POWER_SENSE)
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

// ------- Externals para filtros -------
unsigned short mains_voltage_filtered;
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
#define timer_for_reports (mem_conf.timer_reportar * 30)
#define timer_for_debugs (mem_conf.timer_pruebas * 30)
// volatile unsigned char timer_wifi_bright = 0;

#if (defined USE_REDONDA_BASIC) || (defined USE_ONLY_POWER_SENSE)
volatile unsigned short timer_relay = 0;			//para relay default (si no hay synchro)

volatile unsigned short tt_take_photo_sample;
#endif


unsigned char saved_mode;

// ------- Externals del o para el ADC -------
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];
volatile unsigned char seq_ready = 0;
unsigned short zero_current;

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

// ------- Externals de la Memoria y los modos -------
parameters_typedef * pmem = (parameters_typedef *) (unsigned int *) FLASH_PAGE_FOR_BKP;	//en flash
parameters_typedef mem_conf;


//--- VARIABLES GLOBALES ---//
unsigned short show_power_index = 0;	//lo uso como timer sincronizado con la medicion, tick 2 secs.
unsigned short show_power_index_debug = 0;

//para los msjs GSM
char gsmNUM [20];
char gsmMSG [180];


// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
volatile unsigned short tcp_kalive_timer;
//volatile unsigned char display_timer;
volatile unsigned char timer_meas;

#ifdef WITH_HYST
volatile unsigned short secs = 0;
volatile unsigned char hours = 0;
volatile unsigned char minutes = 0;
#endif

#ifdef POWER_MEAS_WITH_SAMPLES

#ifdef POWER_MEAS_WITH_SAMPLES_MA32
#define SIZEOF_POWER_VECT		32			//aunque en realidad es MA32
#define SIZEOF_POWER_VECT_INDEX		10		//este lo dejo igual porque quiere mediciones cada 2 segundos
#endif

#ifdef POWER_MEAS_WITH_SAMPLES_BESTOF_10
#define SIZEOF_POWER_VECT		10
#define SIZEOF_POWER_VECT_INDEX		10
#endif

unsigned short power_vect [SIZEOF_POWER_VECT];
#endif

#ifdef POWER_MEAS_PEAK_TO_PEAK
#define SIZEOF_POWER_VECT		10
unsigned short power_vect [SIZEOF_POWER_VECT];
#endif



//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);

// ------- del DMX -------
extern void EXTI4_15_IRQHandler(void);
#define DMX_TIMEOUT	20

//--- FILTROS DE SENSORES ---//
// #define LARGO_FILTRO 16
// #define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
// unsigned short vtemp [LARGO_FILTRO + 1];
// unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//


//--- Private Definitions ---//


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    unsigned char i, ii;
    unsigned char resp = resp_continue;
    unsigned char need_to_save = 0;
    // unsigned short wh_int, wh_dec;
    // float fcalc = 1.0;
    unsigned short power, last_power;
    unsigned int zero_current_loc = 0;

    unsigned short acum_secs_index;
    unsigned int acum_secs, acum_hours;

#if (defined USE_REDONDA_BASIC) || (defined USE_ONLY_POWER_SENSE)
    main_state_t main_state = MAIN_INIT;
    lamp_on_state_t lamp_on_state = init_airplane0;
    unsigned char counters_mode = 0;
    unsigned char meas_end = 0;

#ifdef WITH_HYST
    unsigned short hyst;
#endif
#ifdef WITH_1_TO_10_VOLTS
    unsigned short one_to_ten;
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

    HARD_Initial_Setup();


    //ADC Configuration
//	AdcConfig();

    //TIM Configuration.
    // TIM_3_Init();
//	TIM_14_Init();
//	TIM_16_Init();		//para OneShoot() cuando funciona en modo master
//	TIM_17_Init();		//lo uso para el ADC de Igrid

//	EXTIOff ();


//		while (1)
//		{
//			PIN3_OFF;
//			Wait_ms (10);
//			PIN3_ON;
//			Wait_ms (10);
//		}

    //--- Leo los parametros de memoria ---//
#ifdef USE_REDONDA_BASIC
    memcpy(&mem_conf, pmem, sizeof(parameters_typedef));
    if (mem_conf.acumm_wh == 0xFFFFFFFF)
    {
        //memoria vacia
        mem_conf.acumm_wh = 0;
        mem_conf.acumm_w2s = 0;
        mem_conf.acumm_w2s_index = 0;
        mem_conf.timer_reportar = 2;
        mem_conf.timer_pruebas = 10;
        //el timer a reportar esta n minutos, yo tengo tick cada 2 segundos

        strcpy( mem_conf.num_reportar, "1149867843");	//segunda sim de claro
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
    Usart2Send("GSM GATEWAY.. Cambio a GSM\r\n");
    Usart1Mode (USART_GSM_MODE);

    //mando start al gsm
    Usart2Send("Reset y Start GSM\r\n");
    //GPSStartResetSM ();
    timer_standby = 60000;		//doy 1 minuto para prender modulo
    while (timer_standby)
    {
        i = GSM_Start();
        if (i == 1)
        {
            Usart2Send("Start OK\r\n");
            timer_standby = 0;
        }

        if (i > 1)
        {
            Usart2Send("Start NOK\r\n");
            Usart2Send("Please reboot!\r\n");
        }
    }

    Usart2Send("GSM GATEWAY Listo para empezar\r\n");

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

    // Prendo todos los Timers
    TIM_3_Init ();					//lo utilizo para 1 a 10V y para synchro ADC
    TIM_16_Init();					//o utilizo para synchro de relay
    TIM16Enable();
    
    //ADC and DMA configuration
    AdcConfig();
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    //end of ADC & DMA
        
    Usart2Send("\r\nKirno Placa Redonda - Basic V1.0\r\n");
    Usart2Send("  Features:\r\n");
#ifdef WITH_1_TO_10_VOLTS
    Usart2Send("  Dimmer 1 to 10V\r\n");
#endif
#ifdef WITH_HYST
    Usart2Send("  Night Hysteresis\r\n");
#endif
#ifdef WITH_TEMP_CONTROL
    Usart2Send("  Temp Control\r\n");
#endif
#ifdef USE_WITH_SYNC
    Usart2Send("  Sync by Edges\r\n");
#else
    Usart2Send("  Sync by ADC\r\n");
#endif
#ifdef USE_GSM
    Usart2Send("  Uses GSM for SMS data\r\n");
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

    // while (1)
    // {
    // 	if (SYNCP)
    // 	{
    // 		SYNCP_OFF;
    // 		LED_OFF;
    // 	}
    // 	else
    // 	{
    // 		SYNCP_ON;
    // 		LED_ON;
    // 	}
    // 	Wait_ms(10);
    //
    // 	// for (i = 0; i < 255; i++)
    // 	// {
    // 	// 	Update_TIM3_CH1 (i);
    // 	// 	Wait_ms (10);
    // 	// }
    // }


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
    // 			//habilitar y esperar el final de la secuencia
    // 			ADC1->CR |= ADC_CR_ADSTART;
    // 			seq_ready = 0;
    // 			i++;
    // 			break;
    //
    // 		case 1:
    // 			if (seq_ready)		//esoty sincronizado
    // 				i++;
    // 			break;
    //
    // 		case 2:
    // 			RelayOn();
    // 			timer_standby = 50;
    // 			LED_ON;
    // 			i++;
    // 			break;
    //
    // 		case 3:
    // 			if (!timer_standby)
    // 			{
    // 				RelayOff();
    // 				LED_OFF;
    // 				i++;
    // 				timer_standby = 10000;
    // 			}
    // 			break;
    //
    // 		case 4:
    // 			if (!timer_standby)
    // 			{
    // 				i = 2;
    // 			}
    // 			break;
    // 	}
    //
    // 	UpdateRelay ();
    // }

    // ADC1->CR |= ADC_CR_ADSTART;
    // while (1)
    // {
    // 	if (!timer_standby)
    // 	{
    // 		timer_standby = 2000;
    // 		UpdatePhotoTransistor();
    // 		one_to_ten = GetNew1to10 (GetPhoto());
    // 		sprintf (s_lcd, "pwm%d photo%d\r\n", one_to_ten, GetPhoto());
    // 		Usart2Send(s_lcd);
    // 		Update_TIM3_CH1 (one_to_ten);
    // 	}
    // }

//--- FIN Programa de pruebas synchro de Relay -----


//--- Programa de Redonda Basic - Produccion - -----

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
            if ((!timer_standby) && (mains_voltage_filtered > CONNECT_VOLTAGE))
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
                    main_state = SET_COUNTERS_AND_PHONE;
                    i = 0;
                }
            }
            break;

        case SET_COUNTERS_AND_PHONE:
            //cargo contadores desde la flash
            acum_secs = mem_conf.acumm_w2s;
            acum_secs_index = mem_conf.acumm_w2s_index;
            acum_hours = mem_conf.acumm_wh;

            counters_mode = 0;
            LED_OFF;

            Update_TIM3_CH1 (0);		//fin de la prueba de synchro
            main_state = LAMP_OFF;
            break;

        case LAMP_OFF:
            if (GetPhoto() > VOLTAGE_PHOTO_ON)
            {
#ifdef WITH_1_TO_10_VOLTS
                Update_TIM3_CH1 (PWM_MIN);
#else
                Update_TIM3_CH1 (PWM_MAX);
#endif
                // RelayOn();
                main_state = LAMP_ON;
                lamp_on_state = init_airplane0;
                Usart2Send("PRENDIDO\r\n");
                FuncsGSMSendSMS("PRENDIDO", mem_conf.num_reportar);
                LED_ON;
#ifdef WITH_HYST
                hours = 0;
#endif
            }

            //me piden prender forzado
            if (diag_prender)
            {
                Update_TIM3_CH1 (PWM_MAX);
                main_state = LAMP_ON;
                lamp_on_state = init_airplane0;
                Usart2Send("PRENDIDO\r\n");
                //TODO: ver de enviar solo si no hay cola de recepcion!!!!, meterlo directo en la funcion FuncsGSMSendSMS
                FuncsGSMSendSMS("PRENDIDO", mem_conf.num_reportar);
                LED_ON;
#ifdef WITH_HYST
                hours = 0;
#endif
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

#ifdef WITH_HYST			//con Hysteresis apaga casi en el mismo punto en el que prende
                    hyst = GetHysteresis (hours);
                    if (GetPhoto() < (VOLTAGE_PHOTO_ON - hyst))
#else
                        if (GetPhoto() < VOLTAGE_PHOTO_OFF)
#endif
                        {
                            main_state = LAMP_OFF;
                            Update_TIM3_CH1 (0);

                            lamp_on_state = init_airplane0;
                            counters_mode = 0;
                            Usart2Send("APAGADO\r\n");
                            FuncsGSMSendSMS("APAGADO", mem_conf.num_reportar);
                            RelayOff();
                            LED_OFF;
                        }
                        else
                        {
                            //No apago, tengo que reportar?
                            if ((timer_for_reports != 0) && (show_power_index >= timer_for_reports))
                            {
                                show_power_index = 0;
                                counters_mode = 2;		//paso al modo memoria de medicion
                                lamp_on_state = meas_reporting0;
                            }
                            else if ((send_energy) && (!SMSLeft()))
                            {
                                send_energy_reset;
                                counters_mode = 2;		//paso al modo memoria de medicion
                                lamp_on_state = meas_reporting0;
                            }
#ifdef WITH_1_TO_10_VOLTS
                            one_to_ten = GetNew1to10 (GetPhoto());
                            Update_TIM3_CH1 (one_to_ten);
#endif
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
                        ShowPower(s_lcd, power, acum_hours, acum_secs);
                        Usart2Send(s_lcd);
                        FuncsGSMSendSMS(s_lcd, mem_conf.num_reportar);
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
                show_power_index = 0;
                show_power_index_debug = 0;
                timer_meas = 200;		//le doy 200ms de buffer a la medicion
						//esto en realidad es un indice de 2 segundos de tick, la info esta en minutos
                break;

            case meas_meas:
                if (meas_end)		//termino una vuelta de mediciones, generalmente 2 segundos
                {
                    meas_end = 0;

#ifdef WITH_HYST			//con Hysteresis apaga casi en el mismo punto en el que prende
                    hyst = GetHysteresis (hours);
                    if ((GetPhoto() < (VOLTAGE_PHOTO_ON - hyst))  && (!diag_prender))		//NO ESTOY EN DIAGNOSTICO
#else
                        if ((GetPhoto() < VOLTAGE_PHOTO_OFF) && (!diag_prender))		//NO ESTOY EN DIAGNOSTICO
#endif
                        {
                            main_state = LAMP_OFF;
                            Update_TIM3_CH1 (0);

                            lamp_on_state = init_airplane0;
                            counters_mode = 0;
                            Usart2Send("APAGADO\r\n");
                            FuncsGSMSendSMS("APAGADO", mem_conf.num_reportar);
                            RelayOff();
                            LED_OFF;
                        }
                        else
                        {
                            //No apago, tengo que reportar? o estoy en DIAGNOSTICO
                            if (diag_prender)		//en diagnostico
                            {
                                if ((timer_for_debugs != 0) && (show_power_index_debug >= timer_for_debugs))
                                {
                                    //termine el diagnostico
                                    diag_prender_reset;
                                    main_state = LAMP_OFF;
                                    Update_TIM3_CH1 (0);
                                    lamp_on_state = init_airplane0;
                                    counters_mode = 0;
                                    Usart2Send("APAGADO\r\n");
                                    FuncsGSMSendSMS("APAGADO", mem_conf.num_reportar);
                                    RelayOff();
                                    LED_OFF;
                                }
                            }

                            if ((timer_for_reports != 0) && (show_power_index >= timer_for_reports))
                            {
                                show_power_index = 0;
                                counters_mode = 2;		//paso al modo memoria de medicion
                                ShowPower(s_lcd, power, acum_hours, acum_secs);
                                lamp_on_state = meas_reporting0;
                                LED_OFF;
                            }
                            else if ((send_energy) && (!SMSLeft()))
                            {
                                send_energy_reset;
                                counters_mode = 2;		//paso al modo memoria de medicion
                                ShowPower(s_lcd, power, acum_hours, acum_secs);
                                lamp_on_state = meas_reporting0;
                                LED_OFF;
                            }
                            else if ((diag_apagar) && (!SMSLeft()))
                            {
                                diag_apagar_reset;
                                diag_prender_reset;
                                main_state = LAMP_OFF;
                                Update_TIM3_CH1 (0);
                                lamp_on_state = init_airplane0;
                                counters_mode = 0;
                                Usart2Send("APAGADO\r\n");
                                FuncsGSMSendSMS("APAGADO", mem_conf.num_reportar);
                                RelayOff();
                                LED_OFF;
                            }
                            else if ((send_sms_ok) && (!SMSLeft()))
                            {
                                send_sms_ok_reset;
                                counters_mode = 2;		//paso al modo memoria de medicion
                                strcpy(s_lcd, "Conf OK!");
                                lamp_on_state = meas_reporting0;
                                LED_OFF;
                            }

#ifdef WITH_1_TO_10_VOLTS
                            if (!diag_prender)		//si estoy en diagnostico no dimmerizo
                            {
                                one_to_ten = GetNew1to10 (GetPhoto());
                                Update_TIM3_CH1 (one_to_ten);
                            }
#endif
                        }
                }
                break;

            case meas_reporting0:	//mando msj en s_lcd al numero de reporte
                Usart2Send(s_lcd);
                resp = FuncsGSMSendSMS(s_lcd, mem_conf.num_reportar);
                if (resp == resp_gsm_error)
                    lamp_on_state = meas_reporting1;
                else
                    lamp_on_state = meas_reporting2;

                break;

            case meas_reporting1:
                if (meas_end)		//me sincronizo nuevamente con la medicion
                {
                    meas_end = 0;
                    LED_ON;
                    counters_mode = 1;
                    lamp_on_state = meas_meas;
                }
                break;

            case meas_reporting2:
                if (meas_end)		//me sincronizo nuevamente con la medicion
                {
                    meas_end = 0;

                    if (FuncsGSMStateAsk() == gsm_state_ready)
                    {
                        LED_ON;
                        counters_mode = 1;
                        lamp_on_state = meas_meas;
                    }
                }
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
                if (!timer_meas)	//update cada 200ms
                {
#ifdef POWER_MEAS_PEAK_TO_PEAK
                    if (i < (SIZEOF_POWER_VECT - 1))
                    {
                        power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
                        i++;
                    }
                    else
                    {
                        //cargo la ultima posicion del vector y muestro info
                        power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
                        i = 0;

                        if (counters_mode == 1)	//mido normalmente
                        {
                            power = PowerCalcMean8(power_vect);
                            // power = 9871;	//100w forzados para evaluar contadores
                            //9871 * KW = 100
                            if (power < MIN_SENSE_POWER)	//minimo de medicion
                                power = 0;

                            last_power = power;
                        }

                        if (counters_mode == 2)	//no mido solo update de lo viejo
                            power = last_power;

                        acum_secs += power;
                        acum_secs_index++;
                        show_power_index++;
                        show_power_index_debug++;
                        need_to_save = 1;			//aviso que en algun momento hay que guardar

                        if (acum_secs_index >= 1800)
                        {
                            acum_hours += (acum_secs / 1800);	//lo convierto a Wh, para no perder bits en cada cuenta
                            acum_secs = 0;
                            acum_secs_index = 0;
                        }
                        //cuando termino una medicion completa aviso con meas_end
                        meas_end = 1;
                    }
#endif

#ifdef POWER_MEAS_WITH_SAMPLES
                    //pasaron los 200ms
#ifdef POWER_MEAS_WITH_SAMPLES_MA32
                    power = MAFilter32(PowerCalcWithSamples(), power_vect);
#endif
#ifdef POWER_MEAS_WITH_SAMPLES_BESTOF_10
                    power_vect[i] = PowerCalcWithSamples();
#endif
                    if (i < (SIZEOF_POWER_VECT_INDEX - 1))
                    {
                        i++;
                    }
                    else
                    {
#ifdef POWER_MEAS_WITH_SAMPLES_BESTOF_10
                        power = PowerCalcMean8(power_vect);
#endif
                        i = 0;
                        if (counters_mode == 1)	//mido normalmente
                        {
                            if (power < MIN_SENSE_POWER)	//minimo de medicion
                                power = 0;

                            last_power = power;
                        }

                        if (counters_mode == 2)	//no mido solo update de lo viejo
                            power = last_power;

                        acum_secs += power;
                        acum_secs_index++;
                        show_power_index++;
                        show_power_index_debug++;
                        need_to_save = 1;			//aviso que en algun momento hay que guardar

                        if (acum_secs_index >= 1800)
                        {
                            acum_hours += (acum_secs / 1800);	//lo convierto a Wh, para no perder bits en cada cuenta
                            acum_secs = 0;
                            acum_secs_index = 0;
                        }
                        //cuando termino una medicion completa aviso con meas_end
                        meas_end = 1;
#ifdef DEBUG_MEAS_ON
                        ShowPower(s_lcd, power, acum_hours, acum_secs);
                        Usart2Send(s_lcd);
#endif
                    }
#endif
                    timer_meas = 200;		//10 veces 200ms
                }
            }
            break;	//termina LAMP_ON

        case GO_TO_MAINS_FAILURE:
            //apago el gsm
            FuncsGSMShutdown ();
            main_state = MAINS_FAILURE;
            Update_TIM3_CH1 (0);

            //espero 10 segundos como minimo
            timer_standby = 10000;
            Usart2Send("LOW MAINS VOLTAGE\r\n");
            break;

        case MAINS_FAILURE:
            if ((!timer_standby) && (mains_voltage_filtered > CONNECT_VOLTAGE))
            {
                main_state = MAIN_INIT;
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

        //reviso si hay problemas de alimentacion
        if ((main_state > SET_ZERO_CURRENT) && (main_state < GO_TO_MAINS_FAILURE))		//TODO: si es un glitch deberia grabar y seguir
        {
            if (Mains_Glitch() || (mains_voltage_filtered < DISCONNECT_VOLTAGE))
            {
                if (need_to_save)
                {
                    RelayOffFast ();
                    //update de memoria
                    mem_conf.acumm_w2s = acum_secs;
                    mem_conf.acumm_w2s_index = acum_secs_index;
                    mem_conf.acumm_wh = acum_hours;
                    mem_conf.send_energy_flag = 0;	//limpio flags

                    if (WriteConfigurations())
                        Usart2Send("Saved OK!\r\n");
                    else
                        Usart2Send("Mem Error!\r\n");
                    need_to_save = 0;
                }
                if (Mains_Glitch())
                    Usart2Send("Mains Glitch\r\n");
                main_state = GO_TO_MAINS_FAILURE;
            }
        }

        //respuestas cuando tengo lampara apagada (las otras salen sincronizadas con la medicion)
        if (main_state != LAMP_ON)
        {
            //reviso si me pidieron reportar
            if ((send_energy) && (!SMSLeft()))
            {
                send_energy_reset;
                ShowPower(s_lcd, 0, acum_hours, acum_secs);	//si entre por aca la pi es 0
                Usart2Send(s_lcd);
                FuncsGSMSendSMS(s_lcd, mem_conf.num_reportar);
            }

            //reviso si me pidieron reportar un OK de configuracion
            if ((send_sms_ok) && (!SMSLeft()))
            {
                send_sms_ok_reset;
                strcpy(s_lcd, "Conf OK!");
                Usart2Send(s_lcd);
                FuncsGSMSendSMS(s_lcd, mem_conf.num_reportar);
            }
        }

        //Cosas que no dependen del estado del programa
        UpdateRelay ();
        UpdatePhotoTransistor();
#ifdef USE_GSM
        FuncsGSM();
#endif
    }	//end while 1

//---------- Fin Programa de Produccion Redonda Basic--------//
#endif	//USE_REDONDA_BASIC



//--- Programa Solo medicion de potencia -----
#ifdef USE_ONLY_POWER_SENSE
    // USART1Config();
    AdcConfig();		//recordar habilitar sensor en adc.h

    TIM_3_Init ();					//lo utilizo para 1 a 10V y para synchro ADC

    TIM_16_Init();					//o utilizo para synchro de relay
    TIM16Enable();

    Usart2Send((char *) (const char *) "\r\nKirno Placa Redonda - Only Power\r\n");

#ifdef POWER_MEAS_PEAK_TO_PEAK
    Usart2Send((char *) (const char *) "Power Meas Peak to Peak\r\n");
#endif

#ifdef POWER_MEAS_WITH_SAMPLES
    Usart2Send((char *) (const char *) "Power Meas Taking Samples\r\n");
#endif

    for (i = 0; i < 16; i++)
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
                main_state = SET_ZERO_CURRENT;
                timer_standby = 8000;
                zero_current_loc = 0;
                i = 0;
            }
            break;

        case SET_ZERO_CURRENT:
            // if ((!timer_standby) && (mains_voltage_filtered > CONNECT_VOLTAGE))
            // {
            // 	if (i < 32)
            // 	{
            // 		if (seq_ready)		//TODO ojo aca seq_ready se usa fuera del main switch
            // 		{
            // 			seq_ready = 0;
            // 			zero_current_loc += I_Sense;
            // 			i++;
            // 			timer_standby = 2;	//cargo valor zero_current en 64ms
            // 		}
            // 	}
            // 	else
            // 	{
            // 		zero_current_loc >>= 5;
            // 		zero_current = zero_current_loc;
            // FuncsGSMShutdownAlways();
            // Usart2Send("Wait Phone Shutdown\r\n");
            // main_state = SET_COUNTERS_AND_PHONE;
            // i = 0;
            // 	}
            // }

            if (!timer_standby)
            {
                FuncsGSMShutdownAlways();
                Usart2Send("Wait Phone Shutdown\r\n");
                main_state = SET_COUNTERS_AND_PHONE;
                i = 0;
            }
            break;

        case SET_COUNTERS_AND_PHONE:
            //cargo contadores desde la flash
            acum_secs = mem_conf.acumm_w2s;
            acum_secs_index = mem_conf.acumm_w2s_index;
            acum_hours = mem_conf.acumm_wh;

            counters_mode = 0;
            LED_OFF;

            //espero que el telefono este libre
            if (FuncsGSMStateAsk() == gsm_state_stop_always)
            {
                Usart2Send((char *) (const char *) "Phone Shutdown\r\n");
                Usart2Send("PRENDIDO\r\n");
                Update_TIM3_CH1 (PWM_MAX);
                main_state = LAMP_ON;
                RelayOn();
                counters_mode = 1;
                LED_ON;
            }
            break;

        case LAMP_ON:
            if (counters_mode)	//si esta activo el modo de contadores mido
            {
                if (!timer_meas)	//update cada 200ms
                {
#ifdef POWER_MEAS_PEAK_TO_PEAK
                    if (i < (SIZEOF_POWER_VECT - 1))
                    {
                        power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
                        i++;
                    }
                    else
                    {
                        //cargo la ultima posicion del vector y muestro info
                        power_vect[i] = PowerCalc (GetVGrid(), GetIGrid());
                        i = 0;

                        if (counters_mode == 1)	//mido normalmente
                        {
                            power = PowerCalcMean8(power_vect);
                            // power = 9871;	//100w forzados para evaluar contadores
                            //9871 * KW = 100
                            if (power < MIN_SENSE_POWER)	//minimo de medicion
                                power = 0;

                            last_power = power;
                        }

                        if (counters_mode == 2)	//no mido solo update de lo viejo
                            power = last_power;

                        acum_secs += power;
                        acum_secs_index++;
                        show_power_index++;
                        need_to_save = 1;			//aviso que en algun momento hay que guardar

                        if (acum_secs_index >= 1800)
                        {
                            acum_hours += (acum_secs / 1800);	//lo convierto a Wh, para no perder bits en cada cuenta
                            acum_secs = 0;
                            acum_secs_index = 0;
                        }
                        //cuando termino una medicion completa aviso con meas_end
                        meas_end = 1;
                        ShowPower(s_lcd, power, acum_hours, acum_secs);
                        Usart2Send(s_lcd);
                    }
#endif

#ifdef POWER_MEAS_WITH_SAMPLES
                    //pasaron los 200ms
#ifdef POWER_MEAS_WITH_SAMPLES_MA32
                    power = MAFilter32(PowerCalcWithSamples(), power_vect);
#endif
#ifdef POWER_MEAS_WITH_SAMPLES_BESTOF_10
                    power_vect[i] = PowerCalcWithSamples();
#endif
                    if (i < (SIZEOF_POWER_VECT_INDEX - 1))
                    {
                        i++;
                    }
                    else
                    {
#ifdef POWER_MEAS_WITH_SAMPLES_BESTOF_10
                        power = PowerCalcMean8(power_vect);
#endif
                        i = 0;
                        if (counters_mode == 1)	//mido normalmente
                        {
                            if (power < MIN_SENSE_POWER)	//minimo de medicion
                                power = 0;

                            last_power = power;
                        }

                        if (counters_mode == 2)	//no mido solo update de lo viejo
                            power = last_power;

                        acum_secs += power;
                        acum_secs_index++;
                        show_power_index++;
                        need_to_save = 1;			//aviso que en algun momento hay que guardar

                        if (acum_secs_index >= 1800)
                        {
                            acum_hours += (acum_secs / 1800);	//lo convierto a Wh, para no perder bits en cada cuenta
                            acum_secs = 0;
                            acum_secs_index = 0;
                        }
                        //cuando termino una medicion completa aviso con meas_end
                        meas_end = 1;
                        ShowPower(s_lcd, power, acum_hours, acum_secs);
                        Usart2Send(s_lcd);
                    }
#endif

                    timer_meas = 200;		//10 veces 200ms
                }
            }
            break;	//termina LAMP_ON

        case GO_TO_MAINS_FAILURE:
            break;

        case MAINS_FAILURE:
            break;

        default:
            main_state = MAIN_INIT;
            break;
        }	//fin switch main_state

		//Cosas que dependen de las muestras
		// if (seq_ready)
		// {
		// 	seq_ready = 0;
		// // 	UpdateVGrid ();
		// // 	UpdateIGrid ();
		// }

		//Cosas que no dependen del estado del programa
        UpdateRelay ();
#ifdef USE_GSM
        FuncsGSM();
#endif
    }	//end while 1

//--- FIN Programa de pruebas I meas -----
#endif

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
    Usart2Send("Reset y Start GSM\r\n");
    //GPSStartResetSM ();
    timer_standby = 60000;		//doy 1 minuto para prender modulo
    while (timer_standby)
    {
        i = GSM_Start();
        if (i == 2)
        {
            Usart2Send("Start OK\r\n");
            timer_standby = 0;
        }

        if (i == 4)
            Usart2Send("Start NOK\r\n");
    }

    //mando conf al gsm
    Usart2Send("Config al GSM\r\n");
    //GPSConfigResetSM ();

    i = 0;
    while (i == 0)
    {
        ii = GSM_Config(1000);

        if (ii == 2)
            i = 0;
        else if (ii > 2)
        {
            Usart2Send("Error en configuracion\r\n");
            while (1);
        }

        GSMProcess();
        GSMReceive ();

        if (gsm_pckt_ready)
        {
            gsm_pckt_ready = 0;
            Usart2SendUnsigned((unsigned char *)buffUARTGSMrx2, gsm_pckt_bytes);
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
            Usart2SendUnsigned((unsigned char *)buffUARTGSMrx2, gsm_pckt_bytes);
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
    // volatile float HUMIDITY_Value, TEMPERATURE_Value, PRESSURE_Value;


    strcpy((char *)buffer,"{\"d\":{\"myName\":\"Nucleo\"");
    // BSP_HUM_TEMP_GetTemperature((float *)&TEMPERATURE_Value);
    // floatToInt(TEMPERATURE_Value, &d1, &d2, 2);
    sprintf(tempbuff, ",\"A_Temperature\":%lu.%lu",d1, d2);
    strcat((char *)buffer,tempbuff);

    // BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
    // floatToInt(HUMIDITY_Value, &d3, &d4, 2);
    sprintf(tempbuff, ",\"A_Humidity\":%lu.%lu",d3,d4 );
    strcat(  (char *)buffer,tempbuff);

    // BSP_PRESSURE_GetPressure((float *)&PRESSURE_Value);
    // floatToInt(PRESSURE_Value, &d5, &d6, 2);
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

#endif

#ifdef ADC_WITH_TEMP_SENSE
	if (tt_take_temp_sample)
		tt_take_temp_sample--;
#endif

	if (take_temp_sample)
		take_temp_sample--;

	if (timer_meas)
		timer_meas--;

	// //cuenta de a 1 minuto
	// if (secs > 59999)	//pasaron 1 min
	// {
	// 	minutes++;
	// 	secs = 0;
	// }
	// else
	// 	secs++;
	//
	// if (minutes > 60)
	// {
	// 	hours++;
	// 	minutes = 0;
	// }

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
