/*
 * hard.h
 *
 *  Created on: 28/11/2013
 *      Author: Mariano
 */

#ifndef HARD_H_
#define HARD_H_


//----------- Defines For Configuration -------------
//----------- Hardware Board Version -------------
#define VER_1_0
#define VER_1_1		//mismo pinout que VER_1_0

//-------- Type of Program ----------------
//#define USE_MQTT_LIB
//#define USE_GPS
//#define USE_GSM
#define USE_GSM_GATEWAY

//#define WIFI_TO_MQTT_BROKER
//#define USE_CERT_PROGRAM
//#define USE_PROD_PROGRAM

//-------- End Of Defines For Configuration ------

#ifdef VER_1_0
//GPIOA pin0	V_Sense
//GPIOA pin1	Light_Sense
#define LIGHT ((GPIOA->IDR & 0x0002) != 0)

//GPIOA pin2
//GPIOA pin3	usart2 tx rx (para debug)
#define PIN3_ON	GPIOA->BSRR = 0x00000008
#define PIN3_OFF GPIOA->BSRR = 0x00080000


//GPIOA pin4
#define NETLIGHT	((GPIOA->IDR & 0x0010) != 0)

//GPIOA pin5
#define STATUS		((GPIOA->IDR & 0x0020) != 0)

//GPIOA pin6	para PWM_CH1

//GPIOA pin7
#define PWRKEY ((GPIOA->ODR & 0x0080) != 0)
#define PWRKEY_ON	GPIOA->BSRR = 0x00000080
#define PWRKEY_OFF GPIOA->BSRR = 0x00800000

//GPIOB pin0 I_Sense

//GPIOB pin1

//GPIOA pin8

//GPIOA pin9
//GPIOA pin10	usart1 tx rx (para el SIM)

//GPIOA pin11
#define RELAY ((GPIOA->ODR & 0x0800) != 0)
#define RELAY_ON	GPIOA->BSRR = 0x00000800
#define RELAY_OFF GPIOA->BSRR = 0x08000000

//GPIOA pin12
#define LED ((GPIOA->ODR & 0x1000) != 0)
#define LED_ON	GPIOA->BSRR = 0x00001000
#define LED_OFF GPIOA->BSRR = 0x10000000

//GPIOA pin13
//GPIOA pin14

//GPIOA pin15
#define EN_GPS 		((GPIOA->ODR & 0x8000) != 0)
#define EN_GPS_ON	GPIOA->BSRR = 0x00008000
#define EN_GPS_OFF	GPIOA->BSRR = 0x80000000


//GPIOB pin3
#define PPS ((GPIOB->IDR & 0x0008) == 0)

//GPIOB pin4
//GPIOB pin5

//GPIOB pin6
//GPIOB pin7	usart1 tx rx (para el GPS)

#endif	//


//#define DOOR_ROOF	200
//#define DOOR_THRESH	180


//ESTADOS DEL PROGRAMA PRINCIPAL
#define MAIN_INIT				0
#define MAIN_INIT_1				1
#define MAIN_SENDING_CONF		2
#define MAIN_WAIT_CONNECT_0		3
#define MAIN_WAIT_CONNECT_1		4
#define MAIN_WAIT_CONNECT_2		5
#define MAIN_READING_TCP		6
#define MAIN_TRANSPARENT		7
#define MAIN_AT_CONFIG_2B		8
#define MAIN_ERROR				9

#define MAIN_STAND_ALONE		10
#define MAIN_GROUPED			11
#define MAIN_NETWORKED			12
#define MAIN_NETWORKED_1		13
#define MAIN_IN_MAIN_MENU		14

//ESTADOS DEL PROGRAMA PRINCIPAL EN MODO MQTT
#ifdef WIFI_TO_MQTT_BROKER
typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_sending_conf,
  wifi_state_wait_ip,
  wifi_state_wait_ip1,
  wifi_state_idle,
  wifi_state_connecting,
  wifi_state_connected,
  wifi_state_disconnected,
  wifi_state_error,
  wifi_state_socket_close,
  mqtt_socket_create,
  client_conn,
  mqtt_connect,
  mqtt_sub,
  mqtt_pub,
  mqtt_device_control,
  wifi_undefine_state       = 0xFF,
} wifi_state_t;
#endif

//---- Temperaturas en el LM335
//37	2,572
//40	2,600
//45	2,650
//50	2,681
//55	2,725
//60	2,765
#define TEMP_IN_30		3226
#define TEMP_IN_35		3279
#define TEMP_IN_50		3434
#define TEMP_IN_65		3591
#define TEMP_DISCONECT		4000

//ESTADOS DEL DISPLAY EN RGB_FOR_CAT
#define SHOW_CHANNELS	0
#define SHOW_NUMBERS	1

#define SWITCHES_TIMER_RELOAD	10
#define AC_SWITCH_TIMER_RELOAD	22

#define SWITCHES_THRESHOLD_FULL	300		//3 segundos
#define SWITCHES_THRESHOLD_HALF	100		//1 segundo
#define SWITCHES_THRESHOLD_MIN	5		//50 ms

#define AC_SWITCH_THRESHOLD_ROOF	255		//techo del integrador
#define AC_SWITCH_THRESHOLD_FULL	136		//3 segundos
#define AC_SWITCH_THRESHOLD_HALF	45		//1 segundo
#define AC_SWITCH_THRESHOLD_MIN		2		//50 ms

#define TTIMER_FOR_CAT_DISPLAY			2000	//tiempo entre que dice canal y el numero
#define TIMER_STANDBY_TIMEOUT_REDUCED	2000	//reduced 2 segs
#define TIMER_STANDBY_TIMEOUT			6000	//6 segundos
#define DMX_DISPLAY_SHOW_TIMEOUT		30000	//30 segundos

#define S_FULL		10
#define S_HALF		3
#define S_MIN		1
#define S_NO		0

#define FUNCTION_DMX	1
#define FUNCTION_MAN	2
#define FUNCTION_CAT	FUNCTION_MAN

#define SIZEOF_DATA1	512
#define SIZEOF_DATA		256
#define SIZEOF_DATA512	SIZEOF_DATA1
#define SIZEOF_DATA256	SIZEOF_DATA
#define SIZEOF_BUFFTCP	SIZEOF_DATA





// ------- de los switches -------
//void UpdateSwitches (void);
//unsigned char CheckS1 (void);
//unsigned char CheckS2 (void);
//void UpdateACSwitch (void);
//unsigned char CheckACSw (void);


#endif /* HARD_H_ */

