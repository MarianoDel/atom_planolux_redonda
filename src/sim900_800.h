//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### SIM900_800.H #################################
//----------------------------------------------------

// Define to prevent recursive inclusion ---------------------------------------
#ifndef _SIM900_800_H_
#define _SIM900_800_H_

// Includes for Configuration --------------------------------------------------
#include "hard.h"
#include "stm32f0xx.h"


//#define buffUARTGSMrx_dimension 512
#define buffUARTGSMrx_dimension 256

//--- Definicion de pines de hardware, los que no se conecten en la placa utilizar 1 (o lo que corresponda)
//--- se relacionanan con los nombres de hard.h

//--- NRESET - salida al modulo ---//
#define GSM_NRESET
#define GSM_NRESET_ON	asm ("nop \n\t")
#define GSM_NRESET_OFF	asm ("nop \n\t")

//--- POWER KEY - salida al modulo ---//
#define GSM_PWRKEY 		PWRKEY
#define GSM_PWRKEY_ON	PWRKEY_ON
#define GSM_PWRKEY_OFF	PWRKEY_OFF

//--- STATUS - entrada desde el modulo---//
#define GSM_STATUS 		STATUS

//--- NETLIGHT - entrada desde el modulo---//
#define GSM_NETLIGHT 	NETLIGHT

//--- LED NETLIGHT - led de indicaciones ---//
#define LED_NETLIGHT_ON 	LED_ON
#define LED_NETLIGHT_OFF 	LED_OFF
//#define LED_NETLIGHT_ON 	asm ("nop \n\t")
//#define LED_NETLIGHT_OFF 	asm ("nop \n\t")

//--- Module Function Declaration ---//
unsigned char GSM_Start(void);
unsigned char GSM_Stop(void);
void GSM_Start_Stop_ResetSM (void);
unsigned char GSM_Delay (unsigned short);

void GSMReceive (void);
//void GSMReceive (unsigned char * pAlertasReportar, char * puserCode, unsigned char * pclaveAct, unsigned char * pActDact);
char GSMSendCommand (char *ptrCommand, unsigned short timeOut, unsigned char rta,char *ptrRta);
char GSM_Config(unsigned short timeOut);
char GSMSendSMS (char *ptrMSG, char *ptrNUM, unsigned short timeOut);
char GSMConfigGPRS (char sim, char *ptrAPN, char *ptrUSER, char *ptrKEY , char *ptrIPAdd, char *ptrIPremote, char *ptrPORTremote,unsigned short timeOut);
char GSM_SetSIM (unsigned char sim);
char GSMSendIP (char *ptrMSG, unsigned short timeOut);
void GSMTimeoutCounters (void);
//void GSMPrestador(unsigned char * prestadorSim1, unsigned char * prestadorSim2);
void GSMPrestador(unsigned char * pGSMHWstatus, unsigned char * prestadorSim1, unsigned char * prestadorSim2, char * pCONFIGURACIONgprsAPN1, char * pCONFIGURACIONgprsUsuario1, char * pCONFIGURACIONgprsClave1, char * pCONFIGURACIONgprsProveedor1, char * pCONFIGURACIONgprsAPN2, char * pCONFIGURACIONgprsUsuario2, char * pCONFIGURACIONgprsClave2, char * pCONFIGURACIONgprsProveedor2, char * pCONFIGURACIONIPREMOTE, char * pCONFIGURACIONPORTREMOTE);
void GSMrxSMS(unsigned char * pAlertasReportar, char * puserCode, unsigned char * pclaveAct, unsigned char * pActDact, char * pGSMReadSMStel);
char GSMCloseIP(void);
char GSMConfigPDPGPRS (char sim, char *ptrAPN, char *ptrUSER, char *ptrKEY , char *ptrIPAdd, char *ptrIPremote, char *ptrPORTremote,unsigned short timeOut);


void GSMProcess (void);
void GSMReceivSMS (void);
#define UARTGSMSend(X)	Usart1Send(X)
unsigned char SMSLeft (void);

#endif
