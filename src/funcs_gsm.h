//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### FUNCS_GSM.H ##################################
//----------------------------------------------------

// Define to prevent recursive inclusion ---------------------------------------
#ifndef _FUNCS_GSM_H_
#define _FUNCS_GSM_H_

// Includes for Configurations -------------------------------------------------
#include "hard.h"
#include "stm32f0xx.h"


// Exported Types --------------------------------------------------------------
typedef enum {
	gsm_state_reset = 0,
	gsm_state_verify_at,
	gsm_state_wait_cpin,
	gsm_state_echo_disable,
	gsm_state_sms_mode,
	gsm_state_sms_character,
	gsm_state_verify_fully_func,
	gsm_state_wait_reg,
	gsm_state_verify_reg,
	gsm_state_dell_all,
	gsm_state_get_imei,
	gsm_state_ready,
	gsm_state_sending_conf,
	gsm_state_sending_sms,
	gsm_state_command_answer,
	gsm_state_idle,
	gsm_state_connecting,
	gsm_state_connected,
	gsm_state_reading_sms,
	gsm_state_disconnected,
	gsm_state_shutdown,
	gsm_state_stop_wait,
	gsm_state_shutdown_always,
	gsm_state_stop_always

} t_GsmState;

typedef enum {
	resp_gsm_continue = 0,
	resp_gsm_ok,
	resp_gsm_error,
	resp_gsm_timeout

} t_RespGsm;

// Exported Constants ----------------------------------------------------------
//FLAGS de string del gsm
#define GSM_RESET_FLAG		0x8000
#define GSM_SET_CALL			0x0001
#define GSM_SET_SMS			0x0002
#define GSM_SET_CPIN			0x0004
#define GSM_SET_POWER_DOWN	0x4000



// Exported Functions ----------------------------------------------------------
void FuncsGSM (void);
void FuncsGSMReset (void);
unsigned char FuncsGSMReady (void);
unsigned char FuncsGSMStateAsk (void);
void FuncsGSMShutdown (void);
void FuncsGSMShutdownAlways (void);
unsigned char FuncsGSMSendSMS (char *, char *);
void FuncsGSMMessageFlags (unsigned short);
unsigned short FuncsGSMMessageFlagsAsk (void);
unsigned char FuncsGSMCommandAnswer (char * , char * );



#endif
//--- end of file ---//
