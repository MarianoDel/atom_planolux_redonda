//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### GPS_VKTEK.H ##################################
//----------------------------------------------------

// Includes --------------------------------------------------------------------
#include "gps_vktel.h"
#include "uart.h"
#include "hard.h"


// Reused Constants ------------------------------------------------------------
#define gps_mini_timeout	usart1_mini_timeout
#define gps_pckt_ready		usart1_pckt_ready
#define gps_have_data		usart1_have_data
#define gps_pckt_bytes		usart1_pckt_bytes


// Externals -------------------------------------------------------------------
extern volatile unsigned char gps_mini_timeout;
extern volatile unsigned char gps_pckt_ready;
extern volatile unsigned char gps_have_data;
extern unsigned char gps_pckt_bytes;
extern unsigned char gps_buff [];


// Globals ---------------------------------------------------------------------
GPSState gps_state = GPS_INIT;
volatile unsigned short gps_timeout;
unsigned char gps_buff [SIZEOF_GPSBUFF];

// ------- String de configuracion del GPS -------
unsigned char str_conf1 [] = One_Output_Ten_Secs_String_Cmd;
//unsigned char str_conf1 [] = One_Output_One_Secs_String_Cmd;



//--- PRUEBA ALL ENABLE ---//
//unsigned char str_conf2 [] = Enable_GPDTM_String_Cmd;
//unsigned char str_conf3 [] = Enable_GPGBS_String_Cmd;
//unsigned char str_conf4 [] = Enable_GPGGA_String_Cmd;
//unsigned char str_conf5 [] = Enable_GPGLL_String_Cmd;
//unsigned char str_conf6 [] = Enable_GPGRS_String_Cmd;
//
//unsigned char str_conf7 [] = Enable_GPGSA_String_Cmd;
//unsigned char str_conf8 [] = Enable_GPGST_String_Cmd;
//unsigned char str_conf9 [] = Enable_GPGSV_String_Cmd;
//unsigned char str_conf10 [] = Enable_GPRMC_String_Cmd;
//unsigned char str_conf11 [] = Enable_GPVTG_String_Cmd;
//
//unsigned char str_conf12 [] = Enable_GPZDA_String_Cmd;

//--- PRUEBA SOLO GPGLL ENABLE ---//
unsigned char str_conf2 [] = Disable_GPDTM_String_Cmd;
unsigned char str_conf3 [] = Disable_GPGBS_String_Cmd;
unsigned char str_conf4 [] = Disable_GPGGA_String_Cmd;
unsigned char str_conf5 [] = Enable_GPGLL_String_Cmd;
unsigned char str_conf6 [] = Disable_GPGRS_String_Cmd;

unsigned char str_conf7 [] = Disable_GPGSA_String_Cmd;
unsigned char str_conf8 [] = Disable_GPGST_String_Cmd;
unsigned char str_conf9 [] = Disable_GPGSV_String_Cmd;
unsigned char str_conf10 [] = Disable_GPRMC_String_Cmd;
unsigned char str_conf11 [] = Disable_GPVTG_String_Cmd;

unsigned char str_conf12 [] = Disable_GPZDA_String_Cmd;

unsigned char str_factory [] = Set_Factory_String_Cmd;
unsigned char str_save [] = Save_Changes_String_Cmd;


// Module Functions ------------------------------------------------------------
//Inicio del GPS
//contesta RESP_CONTNUE o resp_ok
unsigned char GPSStart(void)
{
    unsigned char resp = resp_continue;

    switch (gps_state)
    {
    case GPS_INIT:
        GPS_PIN_DISA;
        gps_timeout = 100;
        gps_state++;
        break;

    case GPS_INIT2:
        if (!gps_timeout)
        {
            GPS_PIN_ENA;
            gps_timeout = 300;	//300ms startup
            gps_state++;
        }
        break;

    case GPS_INIT3:
        if (!gps_timeout)
        {
            unsigned char to_send = 0xFF;
            Usart1SendUnsigned(&to_send, 1);
            gps_timeout = 2;
            gps_state++;
        }
        break;

    case GPS_INIT4:
        if (!gps_timeout)
        {
            resp = resp_ok;
            gps_state = GPS_INIT;
        }
        break;

    default:
        gps_state = GPS_INIT;
        break;
    }
    return resp;
}


//Timers internos para el GPS
void GPSTimeoutCounters (void)
{
    if (gps_timeout)
        gps_timeout--;

//	if (gps_mini_timeout)	//descuenta general
//		gps_mini_timeout--;
//
}


void GPSProcess (void)
{
    if ((gps_have_data) && (!gps_mini_timeout))
    {
        gps_have_data = 0;
        gps_pckt_ready = 1;
        gps_pckt_bytes = ReadUsart1Buffer(gps_buff, SIZEOF_GPSBUFF);

    }
}


void GPSConfigResetSM(void)
{
    gps_state = GPS_INIT;
}


//Configuracion inicial del GPS
//contesta RESP_CONTNUE o resp_ok
unsigned char GPSConfig(void)
{
    unsigned char resp = resp_continue;

    switch (gps_state)
    {
    case GPS_INIT:
        Usart1SendUnsigned(str_conf1, sizeof(str_conf1));
        gps_timeout = 100;
        gps_state++;
        break;

    case GPS_INIT2:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf2, sizeof(str_conf2));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT3:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf3, sizeof(str_conf3));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT4:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf4, sizeof(str_conf4));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT5:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf5, sizeof(str_conf5));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT6:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf6, sizeof(str_conf6));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT7:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf7, sizeof(str_conf7));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT8:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf8, sizeof(str_conf8));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT9:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf9, sizeof(str_conf9));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT10:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf10, sizeof(str_conf10));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT11:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf11, sizeof(str_conf11));
            gps_timeout = 100;
            gps_state++;
        }
        break;

    case GPS_INIT12:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_conf12, sizeof(str_conf12));
            gps_timeout = 100;
            gps_state = GPS_SAVE;
        }
        break;

    case GPS_SAVE:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_save, sizeof(str_save));
//				gps_timeout = 100;
//				gps_state++;
            resp = resp_ok;
            gps_state = GPS_INIT;
        }
        break;

    default:
        gps_state = GPS_INIT;
        break;
    }

    return resp;
}


//Configuracion de Factoty Default GPS
//contesta RESP_CONTNUE o resp_ok
unsigned char GPSResetFactory(void)
{
    unsigned char resp = resp_continue;

    switch (gps_state)
    {
    case GPS_INIT:
        Usart1SendUnsigned(str_factory, sizeof(str_factory));
        gps_timeout = 100;
        gps_state = GPS_SAVE;
        break;

    case GPS_SAVE:
        if (!gps_timeout)
        {
            Usart1SendUnsigned(str_save, sizeof(str_save));
//				gps_timeout = 100;
//				gps_state++;
            resp = resp_ok;
            gps_state = GPS_INIT;
        }
        break;

    default:
        gps_state = GPS_INIT;
        break;
    }

    return resp;
}


//--- end of file ---//
