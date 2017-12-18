

/* Includes ------------------------------------------------------------------*/
#include "funcs_gsm.h"
#include "sim900_800.h"
#include "uart.h"
#include "flash_program.h"
#include <string.h>


#define MAX_STARTUP_ERRORS		5

//--- Externals variables ---//
extern parameters_typedef param_struct;

//--- Global variables ---//
t_GsmState gsm_state = gsm_state_reset;
unsigned char gsm_sms_error_counter = 0;
unsigned char gsm_error_counter = 0;
char * p_MSG;
char * p_NUM;
char * p_CMD;
char * p_RESP;

char s_msg [100];

//flags
unsigned short GSMFlags = 0;


//--- Exported functions ---//

//Procesa toda la pila del GSM (por lo menos para los SMS)
//los comandos que necesita el modulo se envian por otras funciones
void FuncsGSM (void)
{
	t_RespGsm resp = resp_gsm_continue;
	unsigned short flags = 0;
	unsigned char i = 0;

	switch (gsm_state)
	{
		case gsm_state_reset:
			resp = GSM_Start();

			if (resp == resp_gsm_ok)
			{
				gsm_error_counter = 0;
				gsm_state = gsm_state_verify_at;
			}

			if ((resp == resp_gsm_error) || (resp == resp_gsm_timeout))
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_shutdown;
			}
			break;

		case gsm_state_verify_at:
			resp = GSMSendCommand ("AT\r\n", 1000, 0, &s_msg[0]);

			if (resp == 2)
				gsm_state = gsm_state_wait_cpin;

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		//espero cpin o call ready
		case gsm_state_wait_cpin:
			flags = FuncsGSMMessageFlagsAsk();

			if ((flags == GSM_SET_CPIN) || (flags == GSM_SET_CALL))
				gsm_state = gsm_state_echo_disable;

			resp = GSM_Delay (8000);	//8 segundos de espera

			if (resp == resp_gsm_ok)
				gsm_state = gsm_state_echo_disable;

			break;

		case gsm_state_echo_disable:
			resp = GSMSendCommand ("ATE0\r\n", 1000, 0, &s_msg[0]);

			if (resp == 2)
				gsm_state = gsm_state_sms_mode;

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_sms_mode:
			resp = GSMSendCommand ("AT+CMGF=1\r\n", 1000, 0, &s_msg[0]);

			if (resp == 2)
				gsm_state = gsm_state_sms_character;

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_sms_character:
			resp = GSMSendCommand ("AT+CSCS=\"GSM\"\r\n", 1000, 0, &s_msg[0]);

			if (resp == 2)
				gsm_state = gsm_state_verify_fully_func;

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_verify_fully_func:
			resp = GSMSendCommand ("AT+CFUN=1\r\n", 10000, 0, &s_msg[0]);

			if (resp == 2)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_wait_reg;
			}

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_wait_reg:
			resp = GSM_Delay (8000);	//8 segundos de espera

			if (resp == resp_gsm_ok)
				gsm_state = gsm_state_verify_reg;

			break;

		case gsm_state_verify_reg:
			resp = GSMSendCommand ("AT+CREG?\r\n", 1000, 1, &s_msg[0]);

			if (resp == 2)
			{
				if (!strncmp(s_msg, "+CREG: 0,1", sizeof("+CREG: 0,1") - 1))
					gsm_state = gsm_state_get_imei;		//equipo registrado
				if (!strncmp(s_msg, "+CREG: 0,5", sizeof("+CREG: 0,5") - 1))
					gsm_state = gsm_state_get_imei;		//equipo registrado con roaming

				if (!strncmp(s_msg, "+CREG: 0,2", sizeof("+CREG: 0,2") - 1))
					resp = 3;		//equipo buscando nueva empresa
				if (!strncmp(s_msg, "+CREG: 0,3", sizeof("+CREG: 0,3") - 1))
					resp = 3;		//equipo mal o no registrado

				//ver como resolverlo a futuro
				if (gsm_state == gsm_state_ready)
					gsm_error_counter = 0;
			}

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
				{
					gsm_error_counter++;
					gsm_state = gsm_state_wait_reg;
				}
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_get_imei:
			resp = GSMSendCommand ("AT+GSN\r\n", 1000, 1, &s_msg[0]);

			if (resp == 2)
			{
				i = strlen(s_msg);
				strncpy(param_struct.imei, s_msg, (i - 2));
				Usart2Send("IMEI: ");
				Usart2Send(param_struct.imei);
				Usart2Send("\r\n");

				//mando SMS con mi info
				strcpy(s_msg, "IMEI: ");
				strcat(s_msg, param_struct.imei);
				strcat(s_msg, ", ACTIVO");

				p_MSG = s_msg;
				p_NUM = param_struct.num_reportar;
				gsm_state = gsm_state_sending_sms;
			}

			if (resp > 2)
			{
				if (gsm_error_counter < MAX_STARTUP_ERRORS)
					gsm_error_counter++;
				else
				{
					GSM_Start_Stop_ResetSM ();
					gsm_state = gsm_state_shutdown;
				}
			}
			break;

		case gsm_state_ready:

		//TODO: reviar aca contador de errores
			break;

		case gsm_state_sending_sms:
			resp = GSMSendSMS (p_MSG, p_NUM, 60000);

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
			resp = GSMSendCommand (p_CMD, 10000, 1, p_RESP);	//la mayoria de los comandos no tarda mas de 10 secs

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
			resp = GSM_Delay (10000);	//10 segundos de espera antes de prender

			if (resp == resp_gsm_ok)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_reset;
			}
			break;

		case gsm_state_shutdown_always:
			resp = GSM_Stop();

			if (resp == resp_gsm_ok)
			{
				GSM_Start_Stop_ResetSM ();
				gsm_state = gsm_state_stop_always;
			}
			break;

		case gsm_state_stop_always:
			break;

		default:
			GSM_Start_Stop_ResetSM ();
			gsm_state = gsm_state_reset;
			break;
	}

	GSMProcess ();		//lee bytes del puerto serie y avisa con flag la terminacion del msj
	GSMReceive ();		//usa el flag para analizar las respuestas

	GSMReceivSMS ();	//si existen SMS los leo aca! TODO: por ahora

	// if (gsm_error_counter > 4)
	// {
	// 	Usart2Send("Error counter overflow -> SHUTTING DOWN\r\n");
	// 	GSM_Start_Stop_ResetSM ();
	// 	gsm_state = gsm_state_shutdown;
	// }
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

void FuncsGSMShutdownAlways (void)
{
	GSM_Start_Stop_ResetSM ();
	gsm_state = gsm_state_shutdown_always;
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
