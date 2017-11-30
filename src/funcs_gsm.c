

/* Includes ------------------------------------------------------------------*/
#include "funcs_gsm.h"
#include "sim900_800.h"



//--- Externals variables ---//


//--- Global variables ---//
t_GsmState gsm_state = gsm_state_reset;
unsigned char gsm_sms_error_counter = 0;
char * p_MSG;
char * p_NUM;
char * p_CMD;
char * p_RESP;

//flags
unsigned short GSMFlags = 0;


//--- Exported functions ---//

//Procesa toda la pila del GSM (por lo menos para los SMS)
//los comandos que necesita el modulo se envian por otras funciones
void FuncsGSM (void)
{
	t_RespGsm resp = resp_gsm_continue;
	char dummy [2];

	switch (gsm_state)
	{
		case gsm_state_reset:
			resp = GSM_Start();

			if (resp == resp_gsm_ok)
				gsm_state = gsm_state_verify_at;

			if ((resp == resp_gsm_error) || (resp == resp_gsm_timeout))
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_shutdown;
			}
			break;

		case gsm_state_verify_at:
			resp = GSMSendCommand ("AT\r\n", 1000, 0, &dummy[0]);

			if (resp == 2)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_ready_wait;
			}

			if (resp > 2)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_shutdown;
			}
			break;

		case gsm_state_ready_wait:
			resp = GSM_Delay (8000);	//8 segundos de espera

			if (resp == resp_gsm_ok)
				gsm_state = gsm_state_ready;

			break;

		case gsm_state_ready:

		//TODO: reviar aca contador de errores
			break;

		case gsm_state_sending_sms:
			resp = GSMSendSMS (p_MSG, p_NUM, 4000);

			if (resp == resp_gsm_ok)
			{
				if (gsm_sms_error_counter)
					gsm_sms_error_counter--;

				gsm_state = gsm_state_ready;
			}

			if ((resp == resp_gsm_error) || (resp == resp_gsm_timeout))
			{
				gsm_sms_error_counter++;
				gsm_state = gsm_state_ready;
			}
			break;

		case gsm_state_command_answer:
			resp = GSMSendCommand (p_CMD, 4000, 1, p_RESP);

			if (resp != 1)
			{
				gsm_state = gsm_state_ready;
			}
			break;

		case gsm_state_shutdown:
			resp = GSM_Stop();

			if (resp == resp_gsm_ok)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_stop_wait;
			}
			break;

		case gsm_state_stop_wait:
			resp = GSM_Delay (1000);	//1 segundos de espera antes de prender

			if (resp == resp_gsm_ok)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_reset;
			}
			break;

		default:
			GSM_Start_Stop_ResetSM ();
			gsm_state = gsm_state_reset;
			break;
	}

	GSMProcess ();		//lee bytes del puerto serie y avisa con flag la terminacion del msj
	GSMReceive ();		//usa el flag para analizar las respuestas

	GSMReceivSMS ();	//si existen SMS los leo aca!

}

void FuncsGSMReset (void)
{
	GSM_Start_Stop_ResetSM ();
	gsm_state = gsm_state_reset;
}

unsigned char FuncsGSMSendSMS (char *ptrMSG, char *ptrNUM)
{
	if (gsm_state != gsm_state_ready)
		return resp_gsm_error;

	gsm_state = gsm_state_sending_sms;
	p_MSG = ptrMSG;
	p_NUM = ptrNUM;

	return resp_gsm_ok;
}

void FuncsGSMShutdown (void)
{
	GSM_Start_Stop_ResetSM ();
	gsm_state = gsm_state_shutdown;
}

unsigned char FuncsGSMReady (void)
{
	if (gsm_state == gsm_state_ready)
		return resp_gsm_ok;
	else
		return resp_gsm_error;
}

unsigned char FuncsGSMStateAsk (void)
{
	return gsm_state;
}

void FuncsGSMMessageFlags (unsigned short flag)
{
	unsigned short temp;

	//veo si es un reset flag
	if (flag & GSM_RESET_FLAG)
		GSMFlags &= flag;
	else			//set flags
		GSMFlags |= flag;

}

unsigned short FuncsGSMMessageFlagsAsk (void)
{
	return GSMFlags;
}

unsigned char FuncsGSMCommandAnswer (char * pCMD, char * pIMEI)
{
	if (gsm_state != gsm_state_ready)
		return resp_gsm_error;

	gsm_state = gsm_state_command_answer;
	p_RESP = pIMEI;
	p_CMD = pCMD;

	return resp_gsm_ok;
}
//--- Private function prototypes ---//
//--- Private functions ---//





//--- end of file ---//
