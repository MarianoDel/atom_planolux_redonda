//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### HARD.H #######################################
//----------------------------------------------------

// Define to prevent recursive inclusion ---------------------------------------
#ifndef HARD_H_
#define HARD_H_


//----------- Defines For Configuration -------------
//----------- Hardware Board Version -------------
// #define VER_1_0
#define VER_1_1		//mismo pinout que VER_1_0

//-------- Type of Program ----------------
// #define USE_GPS    //TODO: hay que mejorar la inclusion, redefine MAIN_INIT
// #define USE_GSM_GATEWAY
//OJO --- los dos que siguen van juntos
#define USE_GSM
#define USE_REDONDA_BASIC
//OJO --- los dos que siguen van juntos
// #define USE_GSM
// #define USE_ONLY_POWER_SENSE


//#define WIFI_TO_MQTT_BROKER
//#define USE_CERT_PROGRAM
//#define USE_PROD_PROGRAM
#define DEBUG_ON
#define DEBUG_MEAS_ON

#if (defined USE_REDONDA_BASIC) || (defined USE_ONLY_POWER_SENSE)
//-------- Voltage Conf ------------------------
#define VOLTAGE_PHOTO_OFF	3322
#define VOLTAGE_PHOTO_ON	3722

//-------- Type of Program and Features ----------------
#define WITH_1_TO_10_VOLTS
#define WITH_HYST
// #define WITH_TEMP_CONTROL

//-------- Type of Sync with Mains ----------------
// #define USE_WITH_EDGE_SYNC				//usa circuito sync por flanco (modificacion en Hardware)
#define USE_WITH_ADC_SYNC				//usa al adc para determinar flancos

//-------- Kind of Reports Sended ----------------
#define REPORTS_NORMAL_MODE
// #define REPORTS_AIRPLANE_MODE

//-------- Type of Power Measurement ----------------
// #define POWER_MEAS_PEAK_TO_PEAK
#define POWER_MEAS_WITH_SAMPLES
// #define POWER_MEAS_WITH_SAMPLES_MA32
#define POWER_MEAS_WITH_SAMPLES_BESTOF_10

//-------- Others Configurations depending on the formers ------------
//-------- Hysteresis Conf ------------------------
#ifdef WITH_HYST
#define HYST_MAX	400
#define HYST_2		340
#define HYST_4		240
#define HYST_6		140
#define HYST_MIN	40
#endif

//-------- PWM Conf ------------------------
#ifdef POWER_MEAS_PEAK_TO_PEAK
#define PWM_MAX	255
#define PWM_80		204
#define PWM_60		153
#define PWM_40		102
#define PWM_20		52
#define PWM_MIN	26
#endif

#ifdef POWER_MEAS_WITH_SAMPLES
#define PWM_MAX	1000
#define PWM_80		800
#define PWM_60		600
#define PWM_40		400
#define PWM_20		200
#define PWM_MIN	100
#endif

#endif	//USE_REDONDA_BASIC

//-------- End Of Defines For Configuration ------

#if (defined VER_1_0 || defined VER_1_1)
#ifdef USE_WITH_SYNC
//GPIOA pin0	lo uso como SYNC
#define SYNC ((GPIOA->IDR & 0x0001) != 0)
#else
//GPIOA pin0	V_Sense
#endif
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
#define EN_GPS_ON		GPIOA->BSRR = 0x00008000
#define EN_GPS_OFF	GPIOA->BSRR = 0x80000000
#ifndef USE_GPS
#define SYNCP			EN_GPS
#define SYNCP_ON		EN_GPS_ON
#define SYNCP_OFF		EN_GPS_OFF
#endif

//GPIOB pin3
#define PPS ((GPIOB->IDR & 0x0008) == 0)

//GPIOB pin4
//GPIOB pin5

//GPIOB pin6
//GPIOB pin7	usart1 tx rx (para el GPS)

#endif	//


//ESTADOS DEL PROGRAMA PRINCIPAL
// #if (defined USE_GSM_GATEWAY) || (defined USE_GSM) || (defined USE_GPS)
#if (defined USE_GSM_GATEWAY) || (defined USE_GPS)
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
#endif

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

//ESTADOS DEL PROGRAMA PRINCIPAL USE_REDONDA_BASIC
#if defined USE_REDONDA_BASIC || defined USE_ONLY_POWER_SENSE
typedef enum
{
	MAIN_INIT = 0,
	SYNCHRO_ADC,
	SET_ZERO_CURRENT,
	SET_COUNTERS_AND_PHONE,
  	LAMP_OFF,
	START_GSM,
	CONFIG_GSM,
	WELCOME_GSM,
	LAMP_ON,
	GO_TO_MAINS_FAILURE,
	MAINS_FAILURE

} main_state_t;

//Estados cuando la lampara esta prendida
typedef enum
{
	init_airplane0 = 0,
	init_airplane1,
	meas_init,
	meas_meas,
	meas_reporting0,
	meas_reporting1,
	meas_reporting2,
  	meas_go_airplane

} lamp_on_state_t;

#endif


//--- Temas con el sync de relay
//#define TT_DELAYED_OFF		5600		//para relay metab
//#define TT_DELAYED_ON		6560		//para relay metab
#ifdef USE_WITH_SYNC
#define TT_DELAYED_OFF		5260		//para relay placa redonda
#define TT_DELAYED_ON		5400		//para relay placa redonda
#else
#define TT_DELAYED_OFF		6660		//para relay placa redonda
#define TT_DELAYED_ON		7000		//para relay placa redonda

#define ADC_THRESHOLD		1024		//0.825V threshold para deteccion de flanco
#define ADC_NOISE				50		//0.04V threshold para ruido de ADC
#endif
#define TT_RELAY			60		//timeout de espera antes de pegar o despegar el relay por default

enum Relay_State {

	ST_OFF = 0,
	ST_WAIT_ON,
	ST_DELAYED_ON,
	ST_ON,
	ST_WAIT_OFF,
	ST_DELAYED_OFF

};

//--- Temas de la medicion de potencia
#ifdef POWER_MEAS_WITH_SAMPLES
#define KW			0.01246		//midiendo a 52W en MUESTRA B con programa USE_ONLY_POWER_SENSE 3-1-18
//midiendo a 300W en MUESTRA B con programa USE_ONLY_POWER_SENSE 3-1-18
#define MIN_SENSE_POWER		160		//2W con KW
#endif

#ifdef POWER_MEAS_PEAK_TO_PEAK
// #define KW			0.01013		//R originales en OPAMP
// #define KW			0.01992		//con los cambos en las R y ajustado en 300W MUESTRA A
// #define KW			0.02131		//midiendo desde 50 a 300W en MUESTRA A ajustado con python "ajuste_potencia.py" 19-12-17
#define KW			0.02119		//midiendo a 300W en MUESTRA A con programa USE_ONLY_POWER_SENSE 19-12-17

// #define MIN_SENSE_POWER		753		//15W con KW
#define MIN_SENSE_POWER		1506		//30W con KW
#endif

//--- Temas con la medicion de tension
//MUESTRA A (BV)
// #define GLITCH_VOLTAGE			764		//equivale a 100V 0.616V V_Sense
// #define DISCONNECT_VOLTAGE		1786		//equivale 160V 1.44V V_Sense
// #define CONNECT_VOLTAGE			2233		//equivale 180V 1.8V V_Sense

//MUESTRA B KIRNO	15-12-2017
#define CONNECT_VOLTAGE			1898		//equivale 180V 1.58V V_Sense
// #define DISCONNECT_VOLTAGE		1662		//equivale 160V 1.34V V_Sense
#define DISCONNECT_VOLTAGE		1500		//equivale 160V 1.34V V_Sense
#define GLITCH_VOLTAGE			636		//equivale a 100V 0.512V V_Sense


//------------ Lock for Samples -----------
#define LOCK_STANDBY							0
#define LOCK_READY_TO_TAKE_SAMPLES		1
#define LOCK_SAMPLES_TAKEN					2
#define LOCK_PROCESSING						3


typedef enum {
    resp_ok = 0,
    resp_continue,
    resp_selected,
    resp_change,
    resp_change_all_up,
    resp_working,
    resp_error,
    resp_need_to_save,
    resp_finish,
    resp_nok,
    resp_timeout,
    resp_ready,
    resp_no_answer,
    resp_save

} resp_t;


// Module Exported Functions ---------------------------------------------------
void RelayOn (void);
void RelayOff (void);
void RelayOffFast (void);
void UpdateRelay (void);
unsigned char RelayIsOn (void);
unsigned char RelayIsOff (void);
unsigned short GetHysteresis (unsigned char);
unsigned short GetNew1to10 (unsigned short);
void UpdateVGrid (void);
void UpdateIGrid (void);
unsigned short GetVGrid (void);
unsigned short GetIGrid (void);
unsigned short PowerCalc (unsigned short, unsigned short);
unsigned short PowerCalcMean8 (unsigned short * p);
void ShowPower (char *, unsigned short, unsigned int, unsigned int);
unsigned char Mains_Glitch (void);

void UpdatePhotoTransistor(void);
unsigned short GetPhoto (void);
void FillTempBuffer (void);
void FillPhotoBuffer (void);
unsigned short PowerCalcWithSamples (void);
void StartSampling (void);
void HARD_Initial_Setup (void);

#endif /* HARD_H_ */
