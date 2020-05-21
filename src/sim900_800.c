//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### SIM900_800.C #################################
//----------------------------------------------------

// Includes --------------------------------------------------------------------
#include "sim900_800.h"
#include "uart.h"
#include "tim.h"
#include "ESP8266.h"
#include "funcs_gsm.h"		//para string flags
#include "flash_program.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>


// Externals -------------------------------------------------------------------
extern parameters_typedef mem_conf;
//Config SIM900.
extern char SIM900APNSIM1[64];
extern char SIM900USUARIOSIM1[20];
extern char SIM900CLAVESIM1[20];

extern char SIM900APNSIM2[64];
extern char SIM900USUARIOSIM2[20];
extern char SIM900CLAVESIM2[20];
extern char SIM900IPREMOTE[20];
extern char SIM900PORTREMOTE[20];

extern volatile unsigned char usart1_mini_timeout;
extern volatile unsigned char usart1_pckt_ready;
extern volatile unsigned char usart1_have_data;
extern unsigned char usart1_pckt_bytes;

#ifdef USE_GSM_GATEWAY
extern volatile unsigned char usart2_mini_timeout;
extern volatile unsigned char usart2_pckt_ready;
extern volatile unsigned char usart2_have_data;
extern unsigned char usart2_pckt_bytes;
#endif


// Globals ---------------------------------------------------------------------
//UART GSM.
//RX.
volatile char buffUARTGSMrx2[buffUARTGSMrx_dimension];
volatile char PacketReadyUARTGSM;
//TX.

//GSM Start.
unsigned char GSMStartState = 0;
volatile unsigned short GSMStartTime;
volatile unsigned short GSMGeneralTimeOut;

//GSM SendCommand.
char GSMSendCommandState = 0;
char GSMSendCommandFlag = 0;
volatile unsigned short GSMSendCommandTimeOut;


//GSM SendSMS
char GSMSendSMSState = 0;
char GSMSendSMSFlag = 0;
volatile unsigned short GSMSendSMSTimeOut;
char GSMSendSMSbuffAux[32];

//GSM Config.
char GSMConfigState = 0;
volatile unsigned short GSMConfigTimeOut;
volatile unsigned short GSMConfigTime;

//GPRS Config.
unsigned short GSMConfigGPRSTimeOut;
char GSMConfigGPRSState = 0;
char GSMConfigGPRSflag = 0;
char GSMbuffStatus[32];
char GSMbuffGPRSCommand[64];
char GSMIPadd[16];
unsigned char GPRSrssi = 99;

//GSM SendIP
char GSMSendIPState = 0;
char GSMSendIPFlag = 0;
volatile unsigned short GSMSendIPTimeOut;
char GSMSendIPbuffAux[32];

char GSMbuffRtaCommand[buffUARTGSMrx_dimension];


const char GSM_OK[] 	= "OK";
const char GSM_ERR[] 	= "ERROR";

const char GSM_CMGF[] = "+CMGF:";
const char GSM_IPSTATE[] = "STATE:";

//Estados GPRS.
const char GSM_IPINITIAL[] 		= "IP INITIAL";
const char GSM_IPSTART[] 		= "IP START";
const char GSM_IPGPRSACT[] 		= "IP GPRSACT";
const char GSM_IPSTATUS[] 		= "IP STATUS";
const char GSM_IPCONNECTING[] 	= "TCP CONNECTING";
const char GSM_IPCONNOK[] 		= "CONNECT OK";
const char GSM_IPCONNFAIL[] 	= "CONNECT FAIL";
const char GSM_IPCONNCLOSE[] 	= "TCP CLOSED";

const char GSM_SIM1[] = "SIM1";
const char GSM_SIM2[] = "SIM2";
const char GSM_SENDOK[] = "SEND OK";
// const char GSM_SENDOK[] = "000: ACK";
unsigned char prestadorSimState = 0;
unsigned char prestadorSimTime = 250;


//Recepcion de SMS.
unsigned char GSMCantSMS = 0;
// unsigned char GSMCantSMS2 = 0;
unsigned char GSMnumSMS = 1;
char GSMReadSMSState = 0;
char GSMReadSMScommand[32];
char GSMReadSMSrepIn[32];
// unsigned char GSMrxSMSState = 0;
unsigned char prestadorSimSelect = 0;
unsigned char flagCloseIP = 0;


#define gsm_mini_timeout	usart1_mini_timeout
#define gsm_pckt_ready		usart1_pckt_ready
#define gsm_have_data		usart1_have_data
#define gsm_pckt_bytes		usart1_pckt_bytes


// Module Functions ------------------------------------------------------------
//Procesa respuestas del modulo GSM esperando el final de linea por timeout
//carga el buffer buffUARTGSMrx2 y avisa con el flag PacketReadyUARTGSM
void GSMProcess (void)
{
    if ((gsm_have_data) && (!gsm_mini_timeout))
    {
        gsm_have_data = 0;
        gsm_pckt_ready = 1;
        gsm_pckt_bytes = ReadUsart1Buffer((unsigned char *) buffUARTGSMrx2, sizeof(buffUARTGSMrx2));
        PacketReadyUARTGSM = 1;
//		GSMReceive (unsigned char * pAlertasReportar, char * puserCode, unsigned char * pclaveAct, unsigned char * pActDact);
    }

#ifdef USE_GSM_GATEWAY
    if ((usart2_have_data) && (!usart2_mini_timeout))
    {
        usart2_have_data = 0;
        usart2_pckt_ready = 1;
        usart2_pckt_bytes = ReadUsart2Buffer((unsigned char *) buffUARTGSMrx2, sizeof(buffUARTGSMrx2));
    }
#endif
}


//------------------------------------//
//
//Wait: 1
//OK: 	2
//ERR:	3
//TO:	4
//------------------------------------//
/*
unsigned char GSM_Start(void)
{

	if (!GSM_STATUS || (GSMStartState > 1))
	{
		switch(GSMStartState)
		{
			case 0:
				GSMStartTime = 40;
				GSM_PWRKEY_ON;
				LED_NETLIGHT_ON;
				GSMStartState++;
				break;
			case 1:
				if(GSMStartTime == 0) //Espera 4 segugundos.
				{
					LED_NETLIGHT_OFF;
					GSM_PWRKEY_OFF;
					GSMStartTime = 100; //10 segundos.
					GSMStartState++;
				}
				break;
			case 2:
				if(GSM_STATUS)
				{
					//Encendio.
					LED_NETLIGHT_ON;
					GSMStartState++;
					GSMStartTime = 30;
				}
				if(GSMStartTime == 0)
				{
					//Se agoto el tiempo de espera.
					GSMStartState = 0;
					return 4;
				}
				break;
			case 3:
				if(GSMStartTime == 0)
				{
					//1 segundo mas.
					LED_NETLIGHT_OFF;
					GSM_PWRKEY_ON;
					GSMStartState = 0;
					return 2;
				}
				break;
		}
	}
	if (GSM_STATUS || (GSMStartState == 0))
	{
		return 2;
	}
	return 1;
}
*/

//GSM_Start() contesta:
//0 trabajando
//1 terminado OK
//2 ERROR
//3 Timeout
unsigned char GSM_Start (void)
{
    switch(GSMStartState)
    {
    case 0:
        //Levanto PWRKEY
        LED_NETLIGHT_ON;
        GSMStartTime = 100;
        GSM_PWRKEY_ON;
        GSMStartState++;
        break;

    case 1:
        //Bajo PWRKEY
        if(GSMStartTime == 0) //Espera 100 mseg
        {
            LED_NETLIGHT_OFF;
            GSM_PWRKEY_OFF;
            GSMStartTime = 4000; //hasta 4 segundos.
            GSMStartState++;
        }
        break;

    case 2:
        //Levanto PWRKEY
        if (GSM_STATUS)
        {
            GSMStartTime = 1000;
            GSMStartState++;
            LED_NETLIGHT_ON;
            GSM_PWRKEY_ON;
        }
        else if(GSMStartTime == 0) //Espera hasta 4 segs
        {
            return 3;		//TimeOut
        }
        break;

    case 3:
        if(GSMStartTime == 0)	//Espero 1 segundo mas y reviso GSM_STATUS
        {
            if (GSM_STATUS)
                return 1;		//OK
            else
                return 2;		//Error
        }
        break;

    default:
        GSMStartState = 0;
        break;
    }
    return 0;						//trabajando
}

void GSM_Start_Stop_ResetSM (void)
{
    GSMStartState = 0;
}

//GSM_Stop() contesta:
//0 trabajando
//1 terminado OK
//2 ERROR
//3 Timeout
unsigned char GSM_Stop(void)
{
    unsigned char resp = 0;

    switch(GSMStartState)
    {
    case 0:
        //Levanto PWRKEY
        GSM_PWRKEY_OFF;
        GSMStartTime = 250;	//espero 250ms
        GSMGeneralTimeOut = 4000;	//timeout de 4 segs
        GSMStartState++;
        break;

    case 1:
        if ((!GSMStartTime) && (!GSM_STATUS))
        {
            GSMStartTime = 1000;	//espero 1seg
            GSMStartState++;
        }
        break;

    case 2:
        if (!GSMStartTime)
        {
            GSM_PWRKEY_ON;
            resp = 1;
        }
        break;
    }

    if (!GSMGeneralTimeOut)
        resp = 3;				//timeout

    return resp;
}

//TODO: OJO utiliza GSMStartState
//in: delay en ms
//out: 0 working; 1 finish
unsigned char GSM_Delay (unsigned short tim)
{
    unsigned char resp = 0;

    switch(GSMStartState)
    {
    case 0:
        GSMGeneralTimeOut = tim;
        GSMStartState++;
        break;

    case 1:
        if (!GSMGeneralTimeOut)
        {
            GSMStartState = 0;
            resp = 1;
        }
        break;

    default:
        GSMStartState = 0;
        break;
    }
    return resp;
}

//GSMPrestador(&PrestadorSim1, &PrestadorSim2, &CONFIGURACIONgprsAPN1[0], &CONFIGURACIONgprsUsuario1[0], &CONFIGURACIONgprsClave1[0], &CONFIGURACIONgprsProveedor1[0], &CONFIGURACIONgprsAPN2[0], &CONFIGURACIONgprsUsuario2[0], &CONFIGURACIONgprsClave2[0], &CONFIGURACIONgprsProveedor2[0])
void GSMPrestador(unsigned char * pGSMHWstatus,
                  unsigned char * prestadorSim1,
                  unsigned char * prestadorSim2,
                  char * pCONFIGURACIONgprsAPN1,
                  char * pCONFIGURACIONgprsUsuario1,
                  char * pCONFIGURACIONgprsClave1,
                  char * pCONFIGURACIONgprsProveedor1,
                  char * pCONFIGURACIONgprsAPN2,
                  char * pCONFIGURACIONgprsUsuario2,
                  char * pCONFIGURACIONgprsClave2,
                  char * pCONFIGURACIONgprsProveedor2,
                  char * pCONFIGURACIONIPREMOTE,
                  char * pCONFIGURACIONPORTREMOTE)
{
    unsigned char i;

    if (prestadorSimTime == 0)
    {
        switch(prestadorSimSelect)
        {
        case 0:
            if (*pGSMHWstatus & 0x01)
                prestadorSimSelect++;
            break;
        case 1:
            if (*prestadorSim1 == 0)
            {
                switch(prestadorSimState)
                {
                case 0:
                    prestadorSimState++;
                    break;
                case 1:

                    i = GSM_SetSIM (1);
                    if (i == 2)
                        prestadorSimState++;
                    if (i > 2)
                    {
                        prestadorSimState = 0;
                        //prestadorSimTime = 150;
                        prestadorSimSelect++;
                    }

                    if (i == 2)
                        prestadorSimState++;
                    if (i > 2)
                    {
                        prestadorSimState = 0;
                        //prestadorSimTime = 150;
                        prestadorSimSelect++;
                    }

                    break;
                case 2:
                    i = GSMSendCommand ("AT+CSPN?\r\n", 15, 1, &GSMbuffRtaCommand[0]);

                    if (i == 2)
                    {
                        if (!strncmp ((const char *)&GSMbuffRtaCommand[0],(const char *) "+CSPN", sizeof("+CSPN") - 1))
                        {

                            if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) pCONFIGURACIONgprsProveedor1, sizeof(pCONFIGURACIONgprsProveedor1) - 1))
                            {
                                *prestadorSim1 = 1;

                                strcpy((char *)SIM900APNSIM1, 		(const char *)pCONFIGURACIONgprsAPN1);
                                strcpy((char *)SIM900USUARIOSIM1, 	(const char *)pCONFIGURACIONgprsUsuario1);
                                strcpy((char *)SIM900CLAVESIM1, 	(const char *)pCONFIGURACIONgprsClave1);
                                strcpy((char *)SIM900IPREMOTE, 		(const char *)pCONFIGURACIONIPREMOTE);
                                strcpy((char *)SIM900PORTREMOTE, 	(const char *)pCONFIGURACIONPORTREMOTE);
                            }

                            if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) &pCONFIGURACIONgprsProveedor2[0], sizeof(&pCONFIGURACIONgprsProveedor2[0]) - 1))
                            {
                                *prestadorSim1 = 2;

                                strcpy((char *)SIM900APNSIM1, 		(const char *)pCONFIGURACIONgprsAPN2);
                                strcpy((char *)SIM900USUARIOSIM1, 	(const char *)pCONFIGURACIONgprsUsuario2);
                                strcpy((char *)SIM900CLAVESIM1, 	(const char *)pCONFIGURACIONgprsClave2);
                                strcpy((char *)SIM900IPREMOTE, 		(const char *)pCONFIGURACIONIPREMOTE);
                                strcpy((char *)SIM900PORTREMOTE, 	(const char *)pCONFIGURACIONPORTREMOTE);
                            }

/*								if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Personal", sizeof("Personal") - 1))
								{
                                                                *prestadorSim1 = 1;
                                                                LCDTransmitSMStr("\r      SIM 1     \n    Personal    ");
								}
								if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Movistar", sizeof("Movistar") - 1))
								{
                                                                *prestadorSim1 = 2;
                                                                LCDTransmitSMStr("\r      SIM 1     \n    Movistar    ");
								}
								if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Claro", sizeof("Claro") - 1))
								{
                                                                *prestadorSim1 = 3;
                                                                LCDTransmitSMStr("\r      SIM 1     \n      Claro     ");
								}
*/
                            prestadorSimState = 0;
                        }
                        //prestadorSimTime = 150;
                        prestadorSimSelect++;
                    }
                    if (i > 2)
                    {
                        prestadorSimState = 0;
                        //prestadorSimTime = 150;
                        prestadorSimSelect++;

                    }
                    break;

                default:
                    prestadorSimState = 0;
                    break;
                }
            }
        case 2:
            if (*prestadorSim2 == 0)
            {
                switch(prestadorSimState)
                {
                case 0:
                    if (*pGSMHWstatus & 0x02);
                    prestadorSimState++;
                    break;
                case 1:
                    i = GSM_SetSIM (2);

                    if (i == 2)
                        prestadorSimState++;
                    if (i > 2)
                    {
                        prestadorSimState = 0;
                        prestadorSimSelect = 0;
                        prestadorSimTime = 150;
                    }

                    break;
                case 2:
                    i = GSMSendCommand ("AT+CSPN?\r\n", 15, 1, &GSMbuffRtaCommand[0]);

                    if (i == 2)
                    {
                        if (!strncmp ((const char *)&GSMbuffRtaCommand[0],(const char *) "+CSPN", sizeof("+CSPN") - 1))
                        {

                            if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) pCONFIGURACIONgprsProveedor1, sizeof(pCONFIGURACIONgprsProveedor1) - 1))
                            {
                                *prestadorSim2 = 1;

                                strcpy((char *)SIM900APNSIM2, 		(const char *)pCONFIGURACIONgprsAPN1);
                                strcpy((char *)SIM900USUARIOSIM2, 	(const char *)pCONFIGURACIONgprsUsuario1);
                                strcpy((char *)SIM900CLAVESIM2, 	(const char *)pCONFIGURACIONgprsClave1);
                                strcpy((char *)SIM900IPREMOTE, 		(const char *)pCONFIGURACIONIPREMOTE);
                                strcpy((char *)SIM900PORTREMOTE, 	(const char *)pCONFIGURACIONPORTREMOTE);
                            }

                            if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) pCONFIGURACIONgprsProveedor2, sizeof(pCONFIGURACIONgprsProveedor2) - 1))
                            {
                                *prestadorSim2 = 2;

                                strcpy((char *)SIM900APNSIM2, 		(const char *)pCONFIGURACIONgprsAPN2);
                                strcpy((char *)SIM900USUARIOSIM2, 	(const char *)pCONFIGURACIONgprsUsuario2);
                                strcpy((char *)SIM900CLAVESIM2, 	(const char *)pCONFIGURACIONgprsClave2);
                                strcpy((char *)SIM900IPREMOTE, 		(const char *)pCONFIGURACIONIPREMOTE);
                                strcpy((char *)SIM900PORTREMOTE, 	(const char *)pCONFIGURACIONPORTREMOTE);
                            }

/*									if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Personal", sizeof("Personal") - 1))
									{
                                                                        *prestadorSim2 = 1;
                                                                        LCDTransmitSMStr("\r      SIM 2     \n    Personal    ");
									}
									if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Movistar", sizeof("Movistar") - 1))
									{
                                                                        *prestadorSim2 = 2;
                                                                        LCDTransmitSMStr("\r      SIM 2     \n    Movistar    ");
									}
									if (!strncmp ((const char *)&GSMbuffRtaCommand[8], (const char *) "Claro", sizeof("Claro") - 1))
									{
                                                                        *prestadorSim2 = 3;
                                                                        LCDTransmitSMStr("\r      SIM 2     \n      Claro     ");
									}
*/
                            prestadorSimState = 0;

                        }
                        prestadorSimSelect = 0;
                        prestadorSimTime = 150;
                    }
                    if (i > 2)
                    {
                        prestadorSimState = 0;
                        prestadorSimSelect = 0;
                        prestadorSimTime = 150;
                    }
                    break;

                default:
                    prestadorSimState = 0;
                    break;
                }
            }
            break;

        default:
            prestadorSimSelect = 0;
            break;


        }

    }
}

char GSMCloseIP(void)
{
    unsigned char i = 0;

    if (flagCloseIP == 1)
    {
        i = GSMSendCommand ("AT+CIPSHUT\r\n", 15, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            flagCloseIP = 0;
            return i;
        }
        if (i > 2)
        {
            flagCloseIP = 0;
            return i;
        }

        return 1;
    }

    return 0;
}

//Procesa respuestas del modulo GSM ubicadas en el buffer buffUARTGSMrx2 con flag PacketReadyUARTGSM
//revisa los flag de estados GSMConfigGPRSflag GSMSendCommandFlag y revisa respuestas no esperadas
//para respuestas no esperadas revisa SMS y avisa la cantidad con GSMCantSMS
//void GSMReceive (unsigned char * pAlertasReportar, char * puserCode, unsigned char * pclaveAct, unsigned char * pActDact)
void GSMReceive (void)
{
    //---- Comunicacion con modulo GSM ----//
    if (PacketReadyUARTGSM)
    {
        ESPPreParser2((unsigned char *)buffUARTGSMrx2, (unsigned char *)buffUARTGSMrx2);
#ifdef DEBUG_ON
        Usart2Send((char *)buffUARTGSMrx2);
        Usart2Send("\r\n");
#endif

        if (GSMSendCommandFlag)
        {
            if (GSMSendCommandFlag == 3)	//no espera respuesta
            {
                if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)&GSM_SENDOK[0], strlen((const char *)&GSM_SENDOK[0])))
                    GSMSendCommandFlag = 4;
                if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"> ", (sizeof("> ") - 1)))
                    GSMSendCommandFlag = 4;
                if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"ATOK", (sizeof("ATOK") - 1)))
                    GSMSendCommandFlag = 4;
                if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"ATE0OK", (sizeof("ATE0OK") - 1)))
                    GSMSendCommandFlag = 4;
                if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"OK", (sizeof("OK") - 1)))
                    GSMSendCommandFlag = 4;
            }

            if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *) "ERROR", (sizeof("ERROR") - 1)))
                GSMSendCommandFlag = 5;

            if (GSMSendCommandFlag == 1)	//espera respuestas
            {
                //autorizo la copia siempre
                // if(buffUARTGSMrx2[0] == '+')		//lo que sigue es la respuesta al comando
                GSMSendCommandFlag = 2;
            }
        }

        if(GSMConfigGPRSflag == 1)
        {
            if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)&GSM_IPSTATE[0], strlen((const char *)&GSM_IPSTATE[0])))
            {
                strcpy((char *)&GSMbuffStatus[0],(const char *)&buffUARTGSMrx2[sizeof(GSM_IPSTATE)]);
                GSMConfigGPRSflag = 2;
            }
        }

        if(GSMConfigGPRSflag == 3)
        {
            if ((buffUARTGSMrx2[0] > 47) && (buffUARTGSMrx2[0] < 59) && (buffUARTGSMrx2[1] > 47) && (buffUARTGSMrx2[1] < 59) && (buffUARTGSMrx2[2] > 47) && (buffUARTGSMrx2[2] < 59) && buffUARTGSMrx2[3] == '.')
            {
                strncpy((char *)&GSMIPadd[0],(const char *)&buffUARTGSMrx2[0], 16);
                GSMSendCommandFlag = 4;
            }
            if ((buffUARTGSMrx2[0] > 47) && (buffUARTGSMrx2[0] < 59) && (buffUARTGSMrx2[1] > 47) && (buffUARTGSMrx2[1] < 59) && buffUARTGSMrx2[2] == '.')
            {
                strncpy((char *)&GSMIPadd[0],(const char *)&buffUARTGSMrx2[0], 16);
                GSMSendCommandFlag = 4;
            }
            if ((buffUARTGSMrx2[0] > 47) && (buffUARTGSMrx2[0] < 59) && buffUARTGSMrx2[1] == '.')
            {
                strncpy((char *)&GSMIPadd[0],(const char *)&buffUARTGSMrx2[0], 16);
                GSMSendCommandFlag = 4;
            }
        }

        //respuestas no esperadas
        //primero reviso flags
        if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"+CPIN: ", (sizeof("+CPIN: ") - 1)))
        {
            FuncsGSMMessageFlags (GSM_RESET_FLAG);
            FuncsGSMMessageFlags (GSM_SET_CPIN);
        }

        if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"Call Ready", (sizeof("Call Ready") - 1)))
            FuncsGSMMessageFlags (GSM_SET_CALL);

        if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"SMS Ready", (sizeof("SMS Ready") - 1)))
            FuncsGSMMessageFlags (GSM_SET_SMS);

        if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"NORMAL POWER DOWN", (sizeof("NORMAL POWER DOWN") - 1)))
        {
            FuncsGSMMessageFlags (GSM_RESET_FLAG);
            FuncsGSMMessageFlags (GSM_SET_POWER_DOWN);
        }

        // if (!strncmp((char *)&buffUARTGSMrx2[0], (const char *)"+CMTIDS: \"SM\",", sizeof ("+CMTIDS: \"SM\",") -1))
        // {
        // 	GSMCantSMS2 = buffUARTGSMrx2[14] - 48;
        // }
        if (!strncmp((char *)&buffUARTGSMrx2[0], (const char *)"+CMTI: \"SM\",", sizeof ("+CMTI: \"SM\",") -1))
        {
            //TODO: modificar a dos bytes
            GSMCantSMS = buffUARTGSMrx2[12] - 48;
        }

        if(!strncmp((const char *)&buffUARTGSMrx2[0], (const char *)"CLOSED", strlen((const char *)"CLOSED")))
        {
            flagCloseIP = 1;
        }

        PacketReadyUARTGSM = 0;
    }
}

//-----------------------------------------------------//
//char GSMSendCommand (char *ptrCommand, unsigned char timeOut, unsigned char rta, char *ptrRta)
//----------------------------------------------------//
//ptrCommand: Comando a enviar.
//ptrRta: Respuesta al comando desde sim900.
//rta: 0: No espera rta. 1: Espera respuesta antes de OK.
//TO: time out. en ms
//----------------------------------------------------//
//Wait: 	1
//OK: 		2
//ERR:		3
//TO:
//NO OK:	4
//NO RTA: 	5
//----------------------------------------------------//
char GSMSendCommand (char *ptrCommand, unsigned short timeOut, unsigned char rta, char *ptrRta)
{
    switch(GSMSendCommandState)
    {
    case 0:
        GSMSendCommandFlag = 0;
        GSMSendCommandTimeOut = timeOut;
        GSMSendCommandState++;
        break;

    case 1:
        UARTGSMSend(&ptrCommand[0]);

        if (rta)
        {
            GSMSendCommandState = 2;
            GSMSendCommandFlag = 1;
        }
        else
        {
            GSMSendCommandState = 3;
            GSMSendCommandFlag = 3;
        }
        break;

    case 2:
        //Espera rta.
        if (GSMSendCommandFlag == 2)
        {
            //Rta obtenida.
            strcpy((char *)ptrRta, (const char *)&buffUARTGSMrx2[0]);
            //OK pegado
            GSMSendCommandFlag = 0;
            GSMSendCommandState = 0;
            return 2;
        }
        break;

    case 3:
        //Espera OK.
        if (GSMSendCommandFlag == 4)
        {
            //OK recibido.
            GSMSendCommandFlag = 0;
            GSMSendCommandState = 0;
            return 2;
        }
        break;

    default:
        GSMSendCommandState = 0;
        break;
    }

    if (GSMSendCommandFlag == 5)	//"ERROR"
    {
        GSMSendCommandState = 0;
        return 3;
    }

    if (!GSMSendCommandTimeOut)
    {
        GSMSendCommandState = 0;
        return 4;
    }

    return 1;
}

//---------------------------------------------------------------//
//Configuracion del modulo GSM.
//---------------------------------------------------------------//
//
//Wait: 	0
//OK:		1
//ERR:	2
//TO:		3
//---------------------------------------------------------------//
char GSM_Config(unsigned short timeOut)
{
    unsigned char i;

    switch(GSMConfigState)
    {
    case 0:
        GSM_NRESET_ON;
        GSM_PWRKEY_ON;
        GSMConfigTime = 0;
        GSMConfigTimeOut = timeOut;
        GSMConfigState++;
        break;

    case 1:

        if (GSM_STATUS)
        {
            //i = GSMSendCommand ("AT\r\n", 15, 0, &GSMbuffRtaCommand[0]);
            i = GSMSendCommand ("AT\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

            if ((i == 2) || (i == 3))
            {
                GSMConfigState += 4;
                GSM_NRESET_ON;
            }

            if (i == 4)
            {
                GSMConfigState += 1;
                GSM_NRESET_ON;
            }

        }
        else
        {
            GSMConfigState += 2;
            GSM_NRESET_ON;
        }
        break;

    case 2:
        //Reinicio del modulo.
        GSM_NRESET_OFF;
        if(!GSM_STATUS)
        {
            GSMConfigState++;
            GSM_NRESET_ON;
        }
        break;

    case 3:

        //Encendido del modulo.
        i = GSM_Start();

        if (i == 2)
        {
            GSMConfigState++;
            GSMConfigTime = 20;
        }

        if (i == 4)
        {
            GSMConfigState = 1;
            return 2;
        }
        break;

    case 4:

        if (GSMConfigTime == 0)
            GSMConfigState = 1;
        break;

    case 5:

        //Comandos para configurar.
        //ATE0.
        //i = GSMSendCommand ("ATE0\r\n", 15, 0, &GSMbuffRtaCommand[0]);
        i = GSMSendCommand ("ATE0\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigState++;
        }
        if (i > 2)
        {
            GSMConfigState = 1;
            return 2;
        }

        break;

    case 6:
        //AT+CMGF=1
        //i = GSMSendCommand ("AT+CMGF=1\r\n", 15, 0, &GSMbuffRtaCommand[0]);
        i = GSMSendCommand ("AT+CMGF=1\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigState++;
        }
        if (i > 2)
        {
            GSMConfigState = 1;
            return 2;
        }
        break;
    case 7:
        //AT+CSCS="GSM"
        //i = GSMSendCommand ("AT+CSCS=\"GSM\"\r\n", 15, 0, &GSMbuffRtaCommand[0]);
        i = GSMSendCommand ("AT+CSCS=\"GSM\"\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigState++;
        }
        if (i > 2)
        {
            GSMConfigState = 1;
            return 2;
        }
        break;
    case 8:
        //AT+CSMP=49,255,0,241
        //i = GSMSendCommand ("AT+CSMP=49,255,0,241\r\n", 15, 0, &GSMbuffRtaCommand[0]);
        i = GSMSendCommand ("AT+CSMP=49,255,0,241\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigState++;
        }
        if (i > 2)
        {
            GSMConfigState = 1;
            return 2;
        }
        break;
    case 9:
        //AT&W
        //i = GSMSendCommand ("AT&W\r\n", 15, 0, &GSMbuffRtaCommand[0]);
        i = GSMSendCommand ("AT&W\r\n", GSMConfigTimeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigState = 1;
            return 1;
        }
        if (i > 2)
        {
            GSMConfigState = 1;
            return 2;
        }
        break;
    default:
        GSMConfigState = 0;
        break;

    }

    if (GSMConfigTimeOut == 0)
    {
        GSMConfigState = 0;
        return 3;
    }

    return 0;
}

//-----------------------------------------------------------------------//
//char GSMSendSMS (char *ptrMSG, char *ptrNUM, unsigned char timeOut)
//-----------------------------------------------------------------------//
//String de mensaje: 	"..."
//timeOut en mS
//String Numero: 		"----------"
//Wait: 	0
//OK:		1
//ERR:	2
//TO:		3
//-----------------------------------------------------------------------//
char GSMSendSMS (char *ptrMSG, char *ptrNUM, unsigned short timeOut)
{
    unsigned char i;

    switch(GSMSendSMSState)
    {
    case 0:
        GSMSendSMSFlag = 0;
        GSMSendSMSTimeOut = timeOut;
        sprintf(&GSMSendSMSbuffAux[0], "AT+CMGS=\"");
        strcat(&GSMSendSMSbuffAux[0], ptrNUM);
        strcat(&GSMSendSMSbuffAux[0], "\"\r\n");
        GSMSendSMSState = 1;
        break;

    case 1:
        i = GSMSendCommand(&GSMSendSMSbuffAux[0], timeOut, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMSendSMSbuffAux[0] = 0;
            strcat(&GSMSendSMSbuffAux[0], ptrMSG);
            strcat(&GSMSendSMSbuffAux[0], "\032");
            GSMSendSMSState++;
        }

        if (i > 2)
        {
            GSMSendSMSState = 0;
            return 2;
        }
        break;

    case 2:
        i = GSMSendCommand(&GSMSendSMSbuffAux[0], timeOut, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMSendSMSState = 0;

            if (!strncmp(GSMbuffRtaCommand, "+CMGS: ", sizeof("+CMGS: ") - 1))
                return 1;
            else
                return 2;
        }

        if (i > 2)
        {
            GSMSendSMSState = 0;
            return 2;
        }
        break;

    default:
        GSMSendSMSState = 0;
        break;
    }

    if (GSMSendSMSTimeOut == 0)
    {
        GSMSendSMSState = 0;
        return 3;
    }

    return 0;
}

//-----------------------------------------------------------------------//
//char GSMConfigGPRS (char *ptrAPN, char *ptrUSER, char *ptrKEY , unsigned char timeOut)
//-----------------------------------------------------------------------//
//char *ptrAPN "gprs.personal.com"
//char *ptrUSER ""
//char *ptrKEY  ""
//char *ptrIPAdd Para devolver la ip asignada.
//unsigned char timeOut x100mS
//char sim
//-----------------------------------------------------------------------//
//Wait:			1
//OK:			2
//ERR:			3
//TO:			4
//NO SIGNAL:	20
//-----------------------------------------------------------------------//
char GSMConfigGPRS (char sim,
                    char *ptrAPN,
                    char *ptrUSER,
                    char *ptrKEY ,
                    char *ptrIPAdd,
                    char *ptrIPremote,
                    char *ptrPORTremote,
                    unsigned short timeOut)
{
    unsigned char i;

    switch(GSMConfigGPRSState)
    {

    case 0:
        GSMConfigGPRSTimeOut = timeOut;
        GSMConfigGPRSState++;
        GSMConfigGPRSflag = 1;
        break;

    case 1:
        i = GSMSendCommand("000: KAL,123456,100\r\n", 20, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            //Respondio ACK.
            //Conectado.
            GSMConfigGPRSState = 0;
            return 2;
        }

        if (i > 2)
        {
            //No conectado.
            //Seleccionar Sim.
            GSMConfigGPRSState = 15;
        }

        break;
    case 2:

        i = GSMSendCommand("AT+CIPSTATUS\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 3:
        if(GSMConfigGPRSflag == 2)
        {
            //const char GSM_IPINITIAL[] 		= "IP INITIAL";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPINITIAL[0], strlen((const char *)&GSM_IPINITIAL[0])))
            {
                GSMConfigGPRSState += 1;
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPSTART[] 		= "IP START";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPSTART[0], strlen((const char *)&GSM_IPSTART[0])))
            {
                GSMConfigGPRSState += 8;
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPGPRSACT[] 		= "IP GPRSACT";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPGPRSACT[0], strlen((const char *)&GSM_IPGPRSACT[0])))
            {
                GSMConfigGPRSState += 9;
                GSMConfigGPRSflag = 1;
            }

            //const char GSM_IPSTATUS[] 	= "IP STATUS";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPSTATUS[0], strlen((const char *)&GSM_IPSTATUS[0])))
            {
                GSMConfigGPRSState += 10;
                sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CIPSTART=\"TCP\",\"");
                strcat(&GSMbuffGPRSCommand[0], ptrIPremote);
                strcat(&GSMbuffGPRSCommand[0], "\",\"");
                strcat(&GSMbuffGPRSCommand[0], ptrPORTremote);
                strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPCONNECTING[] 	= "TCP CONNECTING";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNECTING[0], strlen((const char *)&GSM_IPCONNECTING[0])))
            {
                GSMConfigGPRSState -= 1;
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPCONNOK[] 		= "CONNECT OK";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNOK[0], strlen((const char *)&GSM_IPCONNOK[0])))
            {
                GSMConfigGPRSState = 0;
                return 2;
            }
/*				//const char GSM_IPCONNFAIL[] 	= "CONNECT FAIL";
				if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNFAIL[0], strlen((const char *)&GSM_IPCONNFAIL[0])))
				{
                                GSMConfigGPRSState += 11;
                                GSMConfigGPRSflag = 1;
				}
*/
            //const char GSM_IPCONNCLOSE[] 	= "TCP CLOSED";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNCLOSE[0], strlen((const char *)&GSM_IPCONNCLOSE[0])))
            {
                GSMConfigGPRSState += 10;
                sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CIPSTART=\"TCP\",\"");
                strcat(&GSMbuffGPRSCommand[0], ptrIPremote);
                strcat(&GSMbuffGPRSCommand[0], "\",\"");
                strcat(&GSMbuffGPRSCommand[0], ptrPORTremote);
                strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
                GSMConfigGPRSflag = 1;
            }

            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)"CONNECTED", sizeof("CONNECTED") - 1))
            {
                GSMConfigGPRSState += 11;
                GSMConfigGPRSflag = 1;
            }

            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)"PDP DEACT", sizeof("PDP DEACT") - 1))
            {
                GSMConfigGPRSState += 13;
                GSMConfigGPRSflag = 1;
            }
        }
        break;

    case 4:
        i = GSMSendCommand("AT+CPIN?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if ((!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CPIN: READY", sizeof("+CPIN: READY") - 1)) || (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CPINDS: READY", sizeof("+CPINDS: READY") - 1)))
            {
                GSMConfigGPRSState++;
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 5:
        i = GSMSendCommand("AT+CREG?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CREG: 0,1", sizeof("+CREG: 0,1") - 1))
            {
                GSMConfigGPRSState++;
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 6:
        i = GSMSendCommand("AT+CSQ\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CSQ:", sizeof("+CSQ:") - 1))
            {
                GPRSrssi = ((GSMbuffRtaCommand[6] - 48) * 10 + ((GSMbuffRtaCommand[7] - 48)));

                if (GPRSrssi != 99)
                    GSMConfigGPRSState++;
                else
                {
                    GSMConfigGPRSState = 0;
                    return 20;
                }
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 7:

        i = GSMSendCommand("AT+CIPMODE=1\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 8:

        i = GSMSendCommand("AT+CGATT?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if(!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CGATT:", sizeof("+CGATT:") - 1))
            {
                if (GSMbuffRtaCommand[8] == 49)
                {
                    GSMConfigGPRSState += 1;
                    sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CGDCONT=1,\"IP\",\"");
                    strcat(&GSMbuffGPRSCommand[0], ptrAPN);
                    strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
                }
                else if (GSMbuffRtaCommand[8] == 48)
                {
                    GSMConfigGPRSState = 0;
                    return 3;
                    //El modulo no se encuentra listo para conectar GPRS.
                }
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        break;

    case 9:

        i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
            sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CSTT=\"");
            strcat(&GSMbuffGPRSCommand[0], ptrAPN);
            strcat(&GSMbuffGPRSCommand[0], "\",\"");
            strcat(&GSMbuffGPRSCommand[0], ptrUSER);
            strcat(&GSMbuffGPRSCommand[0], "\",\"");
            strcat(&GSMbuffGPRSCommand[0], ptrKEY);
            strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 10:

        i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 2;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 11:
        i = GSMSendCommand("AT+CIICR\r\n", 200, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 2;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 12:

        GSMConfigGPRSflag = 3;
        i = GSMSendCommand("AT+CIFSR\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 2;
            GSMConfigGPRSflag = 1;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 13:

        i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 3;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 14:
        GSMConfigGPRSState = 0;
        return 2;
        break;

    case 15:
        if ((sim == 1) || (sim == 2))
        {
            i = GSM_SetSIM(sim);

            if (i == 2)
            {
                GSMConfigGPRSState = 2;
            }

            if (i>2)
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }
        else
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        break;
    case 16:

        i = GSMSendCommand("AT+CIPSHUT\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    default:
        GSMConfigGPRSState = 0;
        break;

    }

    if (GSMConfigGPRSTimeOut == 0)
    {
        GSMConfigGPRSState = 0;
        return 4;
    }

    return 1;
}


char GSM_SetSIM (unsigned char sim)
{

    unsigned char i;

    if (sim == 1)
        i = GSMSendCommand("AT+CDSDS=1\r\n", 50, 0, &GSMbuffRtaCommand[0]);
    else if (sim == 2)
        i = GSMSendCommand("AT+CDSDS=2\r\n", 50, 0, &GSMbuffRtaCommand[0]);
    else
        return 3;

    if (i == 2)
    {
        return 2;
    }
    if (i > 2)
    {
        return i;
    }

    return 1;
}


char GSMSendIP (char *ptrMSG, unsigned short timeOut)
{
    unsigned char i;

    switch(GSMSendIPState)
    {
    case 0:
        GSMSendIPFlag = 0;
        GSMSendIPTimeOut = timeOut;
        GSMSendIPState++;
        break;

    case 1:

        i = GSMSendCommand("AT+CIPSEND\r\n", 80, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {

            GSMSendIPbuffAux[0] = 0;
            strcat(&GSMSendIPbuffAux[0], ptrMSG);
            strcat(&GSMSendIPbuffAux[0], "\032");
            GSMSendIPState++;
        }

        if (i > 2)
        {
            GSMSendIPState = 0;
            return 3;
        }
        break;
    case 2:

        i = GSMSendCommand(&GSMSendIPbuffAux[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMSendIPState = 0;
            return 2;
        }

        if (i > 2)
        {
            GSMSendIPState = 0;
            return 3;
        }
        break;

    default:
        GSMSendIPState = 0;
        break;
    }

    if (GSMSendIPTimeOut == 0)
    {
        GSMSendIPState = 0;
        return 4;
    }

    return 1;
}


void GSMTimeoutCounters (void)
{
    if (GSMStartTime)
        GSMStartTime--;

    if(GSMSendCommandTimeOut)
        GSMSendCommandTimeOut--;

    if(GSMConfigTimeOut)
        GSMConfigTimeOut--;

    if(GSMConfigTime)
        GSMConfigTime--;

    if(GSMSendSMSTimeOut)
        GSMSendSMSTimeOut--;

    if(GSMConfigGPRSTimeOut)
        GSMConfigGPRSTimeOut--;

    if(GSMSendIPTimeOut)
        GSMSendIPTimeOut--;

    if (prestadorSimTime)
        prestadorSimTime--;

    if(GSMGeneralTimeOut)
        GSMGeneralTimeOut--;

}


void GSMReceivSMS (void)
{
    unsigned char i, j, colon_index;
    char * pToAnswer;

    if (GSMCantSMS)	//me avisan que hay un SMS para leer, este es el index
    {
        switch(GSMReadSMSState)
        {
        case 0:
            sprintf(&GSMReadSMScommand[0], (const char *)"AT+CMGR=%d\r\n", GSMCantSMS);
            GSMReadSMSState++;
            break;

        case 1:
            i = GSMSendCommand (&GSMReadSMScommand[0], 6000, 1, &GSMbuffRtaCommand[0]);

            if (i == 2)
            {
                if (!strncmp((char *)&GSMbuffRtaCommand[0], (const char *)"+CMGR:", sizeof("+CMGR:") - 1))
                {
                    //mensajes al modulo
                    colon_index = 0;
                    for (j = 0; j < 222; j++)		//222 	160bytes para SMS + 62 bytes header
                    {
                        if (GSMbuffRtaCommand[j] == '"')
                        {
                            colon_index++;
                            if (colon_index == 8)
                            {
                                pToAnswer = (char *) &GSMbuffRtaCommand [j+1];
                                j = 222;
                            }
                        }
                    }

                    //fin del for, en pToAnswer debo tener la respuesta (payload del SMS)
                    if (colon_index == 8)
                    {
                        //Son todos payloads correctos REVISO RESPUESTAS
                        if (!strncmp(pToAnswer, (const char *)"REPORTAR:", sizeof ("REPORTAR:") -1))
                        {
                            strcpy(num_tel_rep, (pToAnswer + 9));
                            //quito el OK pegado
                            colon_index	= strlen(num_tel_rep);
                            if (colon_index > 2)
                                num_tel_rep[colon_index - 2] = '\0';

                            send_sms_ok_set;

                        }

                        if (!strncmp(pToAnswer, (const char *)"TIMER:", sizeof ("TIMER:") -1))
                        {
                            colon_index = 0;
                            colon_index += (*(pToAnswer + 6) - 48) * 10;
                            colon_index += *(pToAnswer + 7) - 48;

                            if ((*(pToAnswer + 6) == 'F') && (*(pToAnswer + 7) == 'F'))
                            {
                                timer_rep = 0;
                                send_sms_ok_set;
                            }
                            else if ((colon_index > 1) && (colon_index <= 60))
                            {
                                timer_rep = colon_index;
                                send_sms_ok_set;
                            }
                        }

                        if (!strncmp(pToAnswer, (const char *)"TIMERD:", sizeof ("TIMERD:") -1))
                        {
                            colon_index = 0;
                            colon_index += (*(pToAnswer + 7) - 48) * 10;
                            colon_index += *(pToAnswer + 8) - 48;

                            if ((*(pToAnswer + 7) == 'F') && (*(pToAnswer + 8) == 'F'))
                            {
                                timer_debug = 0;
                                send_sms_ok_set;
                            }
                            else if ((colon_index > 1) && (colon_index <= 60))
                            {
                                timer_debug = colon_index;
                                send_sms_ok_set;
                            }
                        }

                        if (!strncmp(pToAnswer, (const char *)"PRENDER:", sizeof ("PRENDER:") -1))
                            diag_prender_set;

                        if (!strncmp(pToAnswer, (const char *)"APAGAR:", sizeof ("APAGAR:") -1))
                            diag_apagar_set;

                        if (!strncmp(pToAnswer, (const char *)"ENERGIA:", sizeof ("ENERGIA:") -1))
                            send_energy_set;

                    }

                    //me fijo si tengo mas SMS
                    if (GSMCantSMS > 1)
                    {
                        GSMReadSMSState = 0;
                        GSMCantSMS--;
                    }
                    else
                        GSMReadSMSState++;
                }
            }

            if (i > 2)
                GSMReadSMSState = 0;

            break;

        case 2:
            i = GSMSendCommand ("AT+CMGDA=\"DEL READ\"\r\n", 25000, 0, &GSMbuffRtaCommand[0]);

            //modificacion 19-12-17, si no logre borrar igual dejo vacios sms
            //ademas uso GSMCantSMS como flag para seguir trabajando
            if (i != 1)
            {
                GSMReadSMSState = 0;
                GSMCantSMS = 0;
            }
            break;

        default:
            GSMReadSMSState = 0;
            break;
        }
    }
}

//---------------------------------------------------------//
//void GSMrxSMS(char * ptrMSG, char *ptrNumTel, char flagSMSin)
//---------------------------------------------------------//
// void GSMrxSMS(unsigned char * pAlertasReportar, char * puserCode, unsigned char * pclaveAct, unsigned char * pActDact, char * pGSMReadSMStel)
// {
// 	unsigned char i;
// 	unsigned char flag;
//
// 	if (GSMCantSMS)
// 	{
// 		switch(GSMrxSMSState)
// 		{
//
// 			case 0:
// 				GSMrxSMSState++;
// 				break;
// 			case 1:
// 				i = GSM_SetSIM (1);
//
// 				if (i == 2)
// 					GSMrxSMSState++;
// 				if (i > 2)
// 					GSMrxSMSState=0;
// 				break;
//
// 			case 2:
// 					//Verifico que no hayan sido leidos los SMS.
// 					if (GSMnumSMS <= GSMCantSMS)
// 					{
// 						switch(GSMReadSMSState)
// 						{
// 							case 0:
// 								GSMReadSMSrepIn[0] = 0;
// 								sprintf(&GSMReadSMScommand[0], (const char *)"AT+CMGR=%d\r\n", GSMnumSMS);
// 								GSMReadSMSState++;
// 								break;
//
// 							case 1:
//
// 								i = GSMSendCommand (&GSMReadSMScommand[0], 15, 1, &GSMbuffRtaCommand[0]);
//
// 								if (i == 2)
// 								{
// 									if (!strncmp((char *)&GSMbuffRtaCommand[0], (const char *)"+CMGR:", sizeof("+CMGR:") - 1))
// 									{
//
// 										if (!strncmp((char *)&GSMReadSMSrepIn[0], (const char *)"000:", sizeof ("000:") -1))
// 										{
// 											i = 0;
// 											flag = 0;
// 											while (GSMbuffRtaCommand[i] != 0)
// 											{
// 												if ((GSMbuffRtaCommand[i] == ',') && (GSMbuffRtaCommand[i+1] == '"') && (flag == 0))
// 												{
// 													i += 2;
// 													flag = i;
// 													while (GSMbuffRtaCommand[i] != '"')
// 													{
// 														i++;
// 													}
// 													strncpy(pGSMReadSMStel, &GSMbuffRtaCommand[flag], (i - flag));
// 													//strcat(pGSMReadSMStel, (const char *)"\r\n");
// 													//UARTDBGSend(pGSMReadSMStel);
// 													//GSMReadSMStel[0]  = 0;
// 												}
// 												i++;
// 											}
//
// 											GSMReadSMSrepIn[19] = 0;
// 											if(!strncmp((const char *)&GSMReadSMSrepIn[0], (const char *)"000: ARM", sizeof("000: ARM") - 1))
// 											{
// 												if (!strncmp((const char *)&GSMReadSMSrepIn[9], (const char *)puserCode, strlen((const char *)puserCode)))
// 												{
// 													strncpy((char *) pclaveAct, (const char *)&buffUARTGSMrx2[16], 4);
//
// 													*pActDact |= 0x40; //Armar
// 													*pActDact |= 0x01; //SendOK.
// 												}
// 											}
//
// 											if(!strncmp((const char *)&GSMReadSMSrepIn[0], (const char *)"000: DRM", sizeof("000: DRM") - 1))
// 											{
// 												if (!strncmp((const char *)&GSMReadSMSrepIn[9], (const char *)puserCode, strlen((const char *)puserCode)))
// 												{
// 													strncpy((char *) pclaveAct, (const char *)&buffUARTGSMrx2[16], 4);
//
// 													*pActDact |= 0x80; //Desrmar
// 													*pActDact |= 0x01; //SendOK.
// 												}
// 											}
// 											//strcat(&GSMReadSMSrepIn[0], (const char *)"\r\n");
// 											//UARTDBGSend(&GSMReadSMSrepIn[0]);
// 											//Led4Toggle();
// 										}
//
// 										GSMReadSMSState = 0;
// 										GSMnumSMS++;
// 									}
// 								}
//
// 								if (i > 2)
// 								{
// 									GSMReadSMSState = 0;
// 								}
// 							break;
//
// 						default:
// 							GSMReadSMSState = 0;
// 							break;
// 						}
// 			}
// 			else
// 			{
//
// 				i = GSMSendCommand ("AT+CMGDA=\"DEL READ\"\r\n", 15, 0, &GSMbuffRtaCommand[0]);
//
// 				if (i == 2)
// 				{
// 					GSMnumSMS = 1;
// 					GSMrxSMSState = 0;
// 					GSMReadSMSState = 0;
// 					GSMCantSMS = 0;
// 				}
//
// 				if (i > 2)
// 				{
// 					GSMReadSMSState = 0;
// 				}
// 			}
// 			break;
//
// 		default:
// 			GSMrxSMSState = 0;
// 			break;
// 		}
// 	}
// 	else if (GSMCantSMS2)
// 	{
// 		switch(GSMrxSMSState)
// 		{
//
// 			case 0:
// 				GSMrxSMSState++;
// 				break;
// 			case 1:
// 				i = GSM_SetSIM (2);
//
// 				if (i == 2)
// 					GSMrxSMSState++;
// 				if (i > 2)
// 					GSMrxSMSState=0;
// 				break;
//
// 			case 2:
// 					//Verifico que no hayan sido leidos los SMS.
// 					if (GSMnumSMS <= GSMCantSMS2)
// 					{
// 						switch(GSMReadSMSState)
// 						{
// 							case 0:
// 								GSMReadSMSrepIn[0] = 0;
// 								sprintf(&GSMReadSMScommand[0], (const char *)"AT+CMGR=%d\r\n", GSMnumSMS);
// 								GSMReadSMSState++;
// 								break;
//
// 							case 1:
//
// 								i = GSMSendCommand (&GSMReadSMScommand[0], 15, 1, &GSMbuffRtaCommand[0]);
//
// 								if (i == 2)
// 								{
// 									if (!strncmp((char *)&GSMbuffRtaCommand[0], (const char *)"+CMGR:", sizeof("+CMGR:") - 1))
// 									{
//
// 										if (!strncmp((char *)&GSMReadSMSrepIn[0], (const char *)"000:", sizeof ("000:") -1))
// 										{
// 											i = 0;
// 											flag = 0;
// 											while (*(GSMbuffRtaCommand+i) != 0)
// 											{
// 												if ((*(GSMbuffRtaCommand+i) == ',') && (*(GSMbuffRtaCommand+i+1) == '"') && (flag == 0))
// 												{
// 													i += 2;
// 													flag = i;
// 													while (*(GSMbuffRtaCommand+i) != '"')
// 													{
// 														i++;
// 													}
// 													strncpy((char *)pGSMReadSMStel, (const char *)&GSMbuffRtaCommand[flag], (i - flag));
// 													//strcat(pGSMReadSMStel, (const char *)"\r\n");
// 													//UARTDBGSend(pGSMReadSMStel);
// 													//*pGSMReadSMStel  = 0;
// 												}
// 												i++;
// 											}
//
// 											GSMReadSMSrepIn[19] = 0;
// 											if(!strncmp((const char *)&GSMReadSMSrepIn[0], (const char *)"000: ARM", sizeof("000: ARM") - 1))
// 											{
// 												if (!strncmp((const char *)&GSMReadSMSrepIn[9], (const char *)puserCode, strlen((const char *)puserCode)))
// 												{
// 													strncpy((char *) pclaveAct, (const char *)&buffUARTGSMrx2[16], 4);
//
// 													*pActDact |= 0x40; //Armar
// 													*pActDact |= 0x02; //SendOK.
// 												}
// 											}
//
// 											if(!strncmp((const char *)&GSMReadSMSrepIn[0], (const char *)"000: DRM", sizeof("000: DRM") - 1))
// 											{
// 												if (!strncmp((const char *)&GSMReadSMSrepIn[9], (const char *)puserCode, strlen((const char *)puserCode)))
// 												{
// 													strncpy((char *) pclaveAct, (const char *)&buffUARTGSMrx2[16], 4);
//
// 													*pActDact |= 0x80; //Desrmar
// 													*pActDact |= 0x02; //SendOK.
// 												}
// 											}
// 											//strcat(&GSMReadSMSrepIn[0], (const char *)"\r\n");
// 											//UARTDBGSend(&GSMReadSMSrepIn[0]);
// 											//Led4Toggle();
// 										}
//
// 										GSMReadSMSState = 0;
// 										GSMnumSMS++;
// 									}
// 								}
//
// 								if (i > 2)
// 								{
// 									GSMReadSMSState = 0;
// 									//GSMrxSMSState = 0;
// 								}
// 							break;
//
// 						default:
// 							GSMReadSMSState = 0;
// 							break;
// 						}
// 			}
// 			else
// 			{
//
// 				i = GSMSendCommand ("AT+CMGDA=\"DEL READ\"\r\n", 15, 0, &GSMbuffRtaCommand[0]);
//
// 				if (i == 2)
// 				{
// 					GSMnumSMS = 1;
// 					GSMReadSMSState = 0;
// 					GSMrxSMSState = 0;
// 					GSMCantSMS2 = 0;
// 				}
//
// 				if (i > 2)
// 				{
// 					GSMrxSMSState = 0;
// 					GSMReadSMSState = 0;
// 				}
// 			}
// 			break;
// 		default:
// 			GSMrxSMSState = 0;
// 			break;
// 		}
// 	}
// }

char GSMConfigPDPGPRS (char sim,
                       char *ptrAPN,
                       char *ptrUSER,
                       char *ptrKEY ,
                       char *ptrIPAdd,
                       char *ptrIPremote,
                       char *ptrPORTremote,
                       unsigned short timeOut)
{
    unsigned char i;

    switch(GSMConfigGPRSState)
    {

    case 0:
        GSMConfigGPRSTimeOut = timeOut;
        GSMConfigGPRSState++;
        GSMConfigGPRSflag = 1;
        break;

    case 1:
        GSMConfigGPRSState = 15;
/*
  i = GSMSendCommand("000: KAL,123456,100\r\n", 50, 0, &GSMbuffRtaCommand[0]);

  if (i == 2)
  {
  //Respondio ACK.
  //Conectado.
  GSMConfigGPRSState = 0;
  return 2;
  }

  if (i > 2)
  {
  //No conectado.
  //Seleccionar Sim.
  GSMConfigGPRSState = 15;
  }
*/
        break;
    case 2:

        i = GSMSendCommand("AT+CIPSTATUS\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 3:
        if(GSMConfigGPRSflag == 2)
        {
            //const char GSM_IPINITIAL[] 		= "IP INITIAL";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPINITIAL[0], strlen((const char *)&GSM_IPINITIAL[0])))
            {
                GSMConfigGPRSState += 1;
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPSTART[] 		= "IP START";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPSTART[0], strlen((const char *)&GSM_IPSTART[0])))
            {
                GSMConfigGPRSState += 8;
                GSMConfigGPRSflag = 1;
            }
            //const char GSM_IPGPRSACT[] 		= "IP GPRSACT";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPGPRSACT[0], strlen((const char *)&GSM_IPGPRSACT[0])))
            {
                GSMConfigGPRSState += 9;
                GSMConfigGPRSflag = 1;
            }

            //const char GSM_IPSTATUS[] 	= "IP STATUS";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPSTATUS[0], strlen((const char *)&GSM_IPSTATUS[0])))
            {
/*					GSMConfigGPRSState += 10;
					sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CIPSTART=\"TCP\",\"");
					strcat(&GSMbuffGPRSCommand[0], ptrIPremote);
					strcat(&GSMbuffGPRSCommand[0], "\",\"");
					strcat(&GSMbuffGPRSCommand[0], ptrPORTremote);
					strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
					GSMConfigGPRSflag = 1;
*/
                GSMConfigGPRSState = 0;
                return 2;
            }
            //const char GSM_IPCONNECTING[] 	= "TCP CONNECTING";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNECTING[0], strlen((const char *)&GSM_IPCONNECTING[0])))
            {
                //GSMConfigGPRSState -= 1;
                //GSMConfigGPRSflag = 1;
                GSMConfigGPRSState = 0;
                return 2;
            }
            //const char GSM_IPCONNOK[] 		= "CONNECT OK";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNOK[0], strlen((const char *)&GSM_IPCONNOK[0])))
            {
                GSMConfigGPRSState = 0;
                return 2;
            }
/*				//const char GSM_IPCONNFAIL[] 	= "CONNECT FAIL";
				if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNFAIL[0], strlen((const char *)&GSM_IPCONNFAIL[0])))
				{
                                GSMConfigGPRSState += 11;
                                GSMConfigGPRSflag = 1;
				}
*/
            //const char GSM_IPCONNCLOSE[] 	= "TCP CLOSED";
            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)&GSM_IPCONNCLOSE[0], strlen((const char *)&GSM_IPCONNCLOSE[0])))
            {
                /*	GSMConfigGPRSState += 10;
                        sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CIPSTART=\"TCP\",\"");
                        strcat(&GSMbuffGPRSCommand[0], ptrIPremote);
                        strcat(&GSMbuffGPRSCommand[0], "\",\"");
                        strcat(&GSMbuffGPRSCommand[0], ptrPORTremote);
                        strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
                        GSMConfigGPRSflag = 1;
                */
                GSMConfigGPRSState = 0;
                return 2;
            }

            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)"CONNECTED", sizeof("CONNECTED") - 1))
            {
                /*GSMConfigGPRSState += 11;
                  GSMConfigGPRSflag = 1;
                */
                GSMConfigGPRSState = 0;
                return 2;
            }

            if(!strncmp((const char *)&GSMbuffStatus[0], (const char *)"PDP DEACT", sizeof("PDP DEACT") - 1))
            {
                GSMConfigGPRSState += 13;
                GSMConfigGPRSflag = 1;
            }
        }
        break;

    case 4:
        i = GSMSendCommand("AT+CPIN?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CPIN: READY", sizeof("+CPIN: READY") - 1))
            {
                GSMConfigGPRSState++;
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 5:
        i = GSMSendCommand("AT+CREG?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CREG: 0,1", sizeof("+CREG: 0,1") - 1))
            {
                GSMConfigGPRSState++;
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 6:
        i = GSMSendCommand("AT+CSQ\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if (!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CSQ:", sizeof("+CSQ:") - 1))
            {
                GPRSrssi = ((GSMbuffRtaCommand[6] - 48) * 10 + ((GSMbuffRtaCommand[7] - 48)));

                if (GPRSrssi != 99)
                    GSMConfigGPRSState++;
                else
                {
                    GSMConfigGPRSState = 0;
                    return 20;
                }
            }
            else
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 7:

        i = GSMSendCommand("AT+CIPMODE=1\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 8:

        i = GSMSendCommand("AT+CGATT?\r\n", 50, 1, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            if(!strncmp((const char *)&GSMbuffRtaCommand[0], (const char *)"+CGATT:", sizeof("+CGATT:") - 1))
            {
                if (GSMbuffRtaCommand[8] == 49)
                {
                    GSMConfigGPRSState += 1;
                    sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CGDCONT=1,\"IP\",\"");
                    strcat(&GSMbuffGPRSCommand[0], ptrAPN);
                    strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
                }
                else if (GSMbuffRtaCommand[8] == 48)
                {
                    GSMConfigGPRSState = 0;
                    return 3;
                    //El modulo no se encuentra listo para conectar GPRS.
                }
            }
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        break;

    case 9:

        i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState++;
            sprintf(&GSMbuffGPRSCommand[0], (const char*)"AT+CSTT=\"");
            strcat(&GSMbuffGPRSCommand[0], ptrAPN);
            strcat(&GSMbuffGPRSCommand[0], "\",\"");
            strcat(&GSMbuffGPRSCommand[0], ptrUSER);
            strcat(&GSMbuffGPRSCommand[0], "\",\"");
            strcat(&GSMbuffGPRSCommand[0], ptrKEY);
            strcat(&GSMbuffGPRSCommand[0], "\"\r\n");
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 10:

        i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 2;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 11:
        i = GSMSendCommand("AT+CIICR\r\n", 200, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 2;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;

    case 12:

        GSMConfigGPRSflag = 3;
        i = GSMSendCommand("AT+CIFSR\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 14;
            GSMConfigGPRSflag = 1;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    case 13:
        GSMConfigGPRSState = 2;
/*
  i = GSMSendCommand(&GSMbuffGPRSCommand[0], 50, 0, &GSMbuffRtaCommand[0]);

  if (i == 2)
  {
  GSMConfigGPRSState = 3;
  }

  if (i > 2)
  {
  GSMConfigGPRSState = 0;
  return 3;
  }
*/
        break;

    case 14:
        GSMConfigGPRSState = 0;
        return 2;
        break;

    case 15:
        if ((sim == 1) || (sim == 2))
        {
            i = GSM_SetSIM(sim);

            if (i == 2)
            {
                GSMConfigGPRSState = 2;
            }

            if (i>2)
            {
                GSMConfigGPRSState = 0;
                return 3;
            }
        }
        else
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        break;
    case 16:

        i = GSMSendCommand("AT+CIPSHUT\r\n", 50, 0, &GSMbuffRtaCommand[0]);

        if (i == 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }

        if (i > 2)
        {
            GSMConfigGPRSState = 0;
            return 3;
        }
        break;
    default:
        GSMConfigGPRSState = 0;
        break;

    }

    if (GSMConfigGPRSTimeOut == 0)
    {
        GSMConfigGPRSState = 0;
        return 4;
    }

    return 1;
}


unsigned char SMSLeft (void)
{
    return GSMCantSMS;
}


//--- end of file ---//
