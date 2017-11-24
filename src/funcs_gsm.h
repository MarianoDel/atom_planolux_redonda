
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FUNCS_GSM_H_
#define _FUNCS_GSM_H_

#include "hard.h"
#include "stm32f0xx.h"

//--- Exported types ---//
typedef enum {
	gsm_state_reset = 0,
	gsm_state_verify_at,
	gsm_state_ready_wait,
	gsm_state_ready,
	gsm_state_sending_conf,
	gsm_state_sending_sms,
	gsm_state_wait_ip1,
	gsm_state_idle,
	gsm_state_connecting,
	gsm_state_connected,
	gsm_state_disconnected,
	gsm_state_shutdown,
	gsm_state_stop_wait

} t_GsmState;

typedef enum {
	resp_gsm_continue = 0,
	resp_gsm_ok,
	resp_gsm_error,
	resp_gsm_timeout

} t_RespGsm;

//--- Exported macro ---//


//--- Exported functions ---//
void FuncsGSM (void);
void FuncsGSMReset (void);
unsigned char FuncsGSMReady (void);
void FuncsGSMShutdown (void);
unsigned char FuncsGSMSendSMS (char *, char *);



#endif
//--- end of file ---//
