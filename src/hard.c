//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### HARD.C #######################################
//----------------------------------------------------

// Includes --------------------------------------------------------------------
#include "hard.h"
#include "tim.h"
#include "stm32f0xx.h"
#include "adc.h"
#include "dsp.h"
#include "uart.h"

#include <stdio.h>


// Externals -------------------------------------------------------------------
extern unsigned short timer_relay;
extern volatile unsigned short adc_ch[];
extern unsigned short zero_current;
extern unsigned short mains_voltage_filtered;
extern volatile unsigned short tt_take_photo_sample;

// Globals ---------------------------------------------------------------------
//unsigned char relay_state = 0;
enum Relay_State relay_state = ST_OFF;
unsigned char last_edge;
unsigned char mains_with_glitch = 0;
ma8_u16_data_obj_t mains_filter;

unsigned short max_igrid_last, max_igrid, min_igrid_last, min_igrid;
unsigned short max_vgrid_last, max_vgrid, min_vgrid_last, min_vgrid;
unsigned short igrid_update_samples;
unsigned short vgrid_update_samples;


// ------- del PhotoTransistor -------
#define SIZEOF_PHOTO_TRANS		32
#define DIVISOR_PHOTO			5
unsigned short VoltagePhoto [SIZEOF_PHOTO_TRANS];
unsigned char photo_index = 0;
unsigned short last_photo = 0;
unsigned char new_photo_sample = 0;

// ------- de los filtros y mediciones -------
#ifdef POWER_MEAS_WITH_SAMPLES
#define VECT_SAMPLES		80
unsigned short isense [VECT_SAMPLES];
unsigned short vsense [VECT_SAMPLES];

int power_aux [VECT_SAMPLES];
volatile unsigned char lock_vect = LOCK_STANDBY;
volatile unsigned char power_vector_index = 0;
#endif


// Module Functions ------------------------------------------------------------
// Initial setup for filters and pointers
void HARD_Initial_Setup (void)
{
    MA8_U16Circular_Reset(&mains_filter);
}


void RelayOn (void)
{
    if (!RelayIsOn())
    {
        relay_state = ST_WAIT_ON;
        timer_relay = TT_RELAY;
    }
}


void RelayOff (void)
{
    if (!RelayIsOff())
    {
        relay_state = ST_WAIT_OFF;
        timer_relay = TT_RELAY;
    }
}


void RelayOffFast (void)
{
    RELAY_OFF;
    relay_state = ST_OFF;
}


unsigned char RelayIsOn (void)
{
    if ((relay_state == ST_WAIT_ON) ||
        (relay_state == ST_DELAYED_ON) ||
        (relay_state == ST_ON))
        return 1;
    else
        return 0;
}


unsigned char RelayIsOff (void)
{
    if ((relay_state == ST_WAIT_OFF) ||
        (relay_state == ST_DELAYED_OFF) ||
        (relay_state == ST_OFF))
        return 1;
    else
        return 0;
}


void UpdateRelay (void)
{
    unsigned char edge = 0;
#ifdef USE_WITH_ADC_SYNC
    unsigned short sample = 0;
#endif

#ifdef USE_WITH_EDGE_SYNC
    if ((!last_edge) && (SYNC))		//flanco ascendente detector
    {									//senoidal arriba
//		edge = 1;
        last_edge = 1;
        SYNCP_ON;
    }

    if ((last_edge) && (!SYNC))		//flanco descendente detector
    {									//senoidal abajo
        edge = 1;
        last_edge = 0;
        SYNCP_OFF;
    }
#endif

#ifdef USE_WITH_ADC_SYNC
    //sample = ReadADC1_SameSampleTime(ADC_CH0);
    sample = V_Sense;			//tengo los samples en adc_ch[]

    if (!last_edge)	//flanco ascendente detector, senoidal arriba
    {
        if (sample > (ADC_THRESHOLD + ADC_NOISE))
        {
            last_edge = 1;
            SYNCP_ON;
        }
    }

    if (last_edge)		//flanco descendente detector, senidal abajo
    {
        if (sample < (ADC_THRESHOLD - ADC_NOISE))
        {
            edge = 1;
            last_edge = 0;
            SYNCP_OFF;
#ifdef POWER_MEAS_WITH_SAMPLES
            StartSampling();
#endif
        }
    }
#endif

    switch (relay_state)
    {
    case ST_OFF:

        break;

    case ST_WAIT_ON:
        if (edge)
        {
            edge = 0;
            relay_state = ST_DELAYED_ON;
            TIM16->CNT = 0;
        }

        if (!timer_relay)		//agoto el timer y no encontro sincro, pega igual
        {
            RELAY_ON;
            relay_state = ST_ON;
        }
        break;

    case ST_DELAYED_ON:
        if (TIM16->CNT > TT_DELAYED_ON)
        {
            RELAY_ON;
            relay_state = ST_ON;
        }
        break;

    case ST_ON:

        break;

    case ST_WAIT_OFF:
        if (edge)
        {
            edge = 0;
            relay_state = ST_DELAYED_OFF;
            TIM16->CNT = 0;
        }

        if (!timer_relay)		//agoto el timer y no encontro sincro, despega igual
        {
            RELAY_OFF;
            relay_state = ST_OFF;
        }

        break;

    case ST_DELAYED_OFF:
        if (TIM16->CNT > TT_DELAYED_OFF)
        {
            RELAY_OFF;
            relay_state = ST_OFF;
        }
        break;

    default:
        RELAY_OFF;
        relay_state = ST_OFF;
        break;
    }
}

#ifdef WITH_HYST
unsigned short GetHysteresis (unsigned char hours_past)
{
    if (hours_past > 8)
        return HYST_MIN;
    else if (hours_past > 6)
        return HYST_6;
    else if (hours_past > 4)
        return HYST_4;
    else if (hours_past > 2)
        return HYST_2;
    else
        return HYST_MAX;
}
#endif

#ifdef WITH_1_TO_10_VOLTS
unsigned short GetNew1to10 (unsigned short light)	//prendo en 3722 a 4095 tengo 373 puntos
{
    unsigned short new_light = 0;

#ifdef POWER_MEAS_PEAK_TO_PEAK
    if (light > VOLTAGE_PHOTO_ON)
    {
        new_light = light - VOLTAGE_PHOTO_ON;
    }
    new_light += PWM_MIN;

    if (new_light > 255)
        new_light = 255;
#endif

#ifdef POWER_MEAS_WITH_SAMPLES
    //uso 373 puntos dividido en 6 segmentos 62 puntos
    if (light > 4033)
        new_light = PWM_MAX;
    else if (light > 3971)
        new_light = PWM_80;
    else if (light > 3909)
        new_light = PWM_60;
    else if (light > 3847)
        new_light = PWM_40;
    else if (light > 3785)
        new_light = PWM_20;
    else
        new_light = PWM_MIN;

#endif

    // if (light < VOLTAGE_PHOTO_ON)
    // 	new_light = PWM_MIN;
    // else
    // {
    // 	new_light = light - VOLTAGE_PHOTO_ON;
    // 	new_light += PWM_MIN;
    // }

    return new_light;
}
#endif

//Hay que llamarla sincronizado con las muestras, entran 312 en un ciclo
void UpdateVGrid (void)
{
    //miro la ultima medicion
#ifdef POWER_MEAS_WITH_SAMPLES
    if (vgrid_update_samples < 90)    //80 un ciclo de 20ms
#endif

#ifdef POWER_MEAS_PEAK_TO_PEAK
    if (vgrid_update_samples < 350)    //312 un ciclo de 20ms
#endif
    {
        //reviso si es un maximo
        if (V_Sense > max_vgrid)
            max_vgrid = V_Sense;

        //   //reviso si es un minimo
        //   if (V_Sense < min_vgrid)
        //       min_vgrid = V_Sense;

        vgrid_update_samples++;
    }
    else
    {
        //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
        max_vgrid_last = max_vgrid;
        min_vgrid_last = min_vgrid;
        max_vgrid = 0;
        min_vgrid = 0;
        //   max_vgrid = 2048;
        //   min_vgrid = 2048;
        vgrid_update_samples = 0;

        //reviso si es un glitch
        if (max_vgrid_last < GLITCH_VOLTAGE)
            mains_with_glitch = 1;
        else
            mains_with_glitch = 0;

        //filtro de alimentacion
        mains_voltage_filtered = MA8_U16Circular(&mains_filter, max_vgrid_last);
    }
}


unsigned char Mains_Glitch (void)
{
    return mains_with_glitch;
}

//Hay que llamarla sincronizado con las muestras, entran 312 en un ciclo
void UpdateIGrid (void)
{
    //miro la ultima medicion
    if (igrid_update_samples < 350)    //20 es toda la senoidal 23 es un ciclo y un octavo
    {
        //reviso si es un maximo
        if (I_Sense > max_igrid)
            max_igrid = I_Sense;

        //reviso si es un minimo
        if (I_Sense < min_igrid)
            min_igrid = I_Sense;

        igrid_update_samples++;
    }
    else
    {
        //if (LED2)
        //    LED2_OFF;
        //else
        //    LED2_ON;

        //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
        max_igrid_last = max_igrid;
        min_igrid_last = min_igrid;
        max_igrid = zero_current;
        min_igrid = zero_current;
        igrid_update_samples = 0;
    }
}


unsigned short GetVGrid (void)
{
    if (max_vgrid_last > min_vgrid_last)
        return max_vgrid_last - min_vgrid_last;    //valor de tension pico a pico
    else
        return 0;
}


unsigned short GetIGrid (void)
{
    if (max_igrid_last > min_igrid_last)
        return max_igrid_last - min_igrid_last;    //valor de corriente pico a pico
    else
        return 0;
}


unsigned short PowerCalc (unsigned short a, unsigned short b)
{
    unsigned int temp;
    temp = a * b;
    temp >>= 8;
    return (unsigned short) temp;
}


unsigned short PowerCalcMean8 (unsigned short * p)
{
    unsigned char i, j, ii, max_index, min_index;
    unsigned short power = 0;

    //busco el maximo
    for (i = 0; i < 10; i++)
    {
        if (power < *(p+i))
        {
            power = *(p+i);
            max_index = i;
        }
    }

    //busco el minimo
    power = 4095;
    for (i = 0; i < 10; i++)
    {
        if (power > *(p+i))
        {
            power = *(p+i);
            min_index = i;
        }
    }

    //descarto extremos
    j = 0;
    if (min_index == max_index)	//revisar min_index != max_index
        ii = 9;
    else
        ii = 10;

    for (i = 0; i < ii; i++)
    {
        if ((j != min_index) && (j != max_index))
        {
            *(p+j) = *(p+i);
            j++;
        }
    }

    //TODO: esto es un vector de 10 donde descarto extremos y calculo el promedio
    //rehacer el filtro
    // power = MAFilter8 (p);
    return power;
}


void ShowPower (char * pstr, unsigned short pi, unsigned int e_acum_hours, unsigned int e_acum_secs)
{
    unsigned short power_int, power_dec;
    unsigned short wh_int, wh_dec;
    float fcalc = 1.0;

    fcalc = pi * KW;
    power_int = (unsigned short) fcalc;
    fcalc = fcalc - power_int;
    fcalc = fcalc * 100;
    power_dec = (unsigned short) fcalc;

    fcalc = (e_acum_hours + e_acum_secs / 1800) * KW;
    wh_int = (unsigned short) fcalc;
    fcalc = fcalc - wh_int;
    fcalc = fcalc * 10;
    wh_dec = (unsigned short) fcalc;

    sprintf(pstr, "pi: %3d.%02d wh: %d.%01d\r\n", power_int, power_dec, wh_int, wh_dec);

}

void UpdatePhotoTransistor(void)
{
    //hago update cada 1 seg
    if (!tt_take_photo_sample)
    {
        tt_take_photo_sample = 1000;

        // VoltagePhoto [photo_index] = ReadADC1_SameSampleTime(ADC_CH1);
        VoltagePhoto [photo_index] = Light_Sense;

        if (photo_index < (SIZEOF_PHOTO_TRANS - 1))
            photo_index++;
        else
            photo_index = 0;

        new_photo_sample = 1;
    }
}


void FillPhotoBuffer (void)
{
    unsigned char i;
    unsigned short dummy;

    // dummy = ReadADC1_SameSampleTime(ADC_CH1);
    dummy = Light_Sense;

    for (i = 0; i < SIZEOF_PHOTO_TRANS; i++)
        VoltagePhoto[i] = dummy;

}

//devuelve el valor promedio del PhotoTransistor
//si existen nuevas muestras hace la cuenta, sino contesta el ultimo valor calculado
unsigned short GetPhoto (void)
{
    unsigned char i;
    unsigned int t = 0;

    if (new_photo_sample)
    {
        for (i = 0; i < SIZEOF_PHOTO_TRANS; i++)
        {
            t += VoltagePhoto[i];
        }

        last_photo = t >> DIVISOR_PHOTO;
        new_photo_sample = 0;
    }

    return last_photo;
}

#ifdef POWER_MEAS_WITH_SAMPLES
//devuelve la potencia activa de los vectores de mustras isense y vsense
unsigned short PowerCalcWithSamples (void)
{
#ifdef DEBUG_MEAS_ON
	char s [10];
#endif	
	unsigned char i;
	int aux1 = 0;

	lock_vect = LOCK_PROCESSING;
	//promedio isense para conocer zero current
	for (i = 0; i < VECT_SAMPLES; i++)
		aux1 += isense[i];

	aux1 = aux1 / VECT_SAMPLES;
	// aux1 = 2048;

#ifdef DEBUG_MEAS_ON
	sprintf(s, "z%d ", aux1);
	Usart2Send(s);
#endif

	//en power_aux pongo la corriente sin offset
	for (i = 0; i < VECT_SAMPLES; i++)
		power_aux[i] = isense[i] - (unsigned short) aux1;

	//multiplico para conocer pact
	for (i = 0; i < VECT_SAMPLES; i++)
	{
		aux1 = power_aux[i] * vsense[i];
		// aux1 >>= 8;
		aux1 >>= 5;
		// aux1 = aux1 / 32;
		power_aux[i] = aux1;
	}

	//integro pact
	aux1 = 0;
	for (i = 0; i < VECT_SAMPLES; i++)
		aux1 += power_aux[i];

	aux1 = aux1 / VECT_SAMPLES;

	if (aux1 < 0)		//recorto errores negativos
		aux1 = 0;

	lock_vect = LOCK_STANDBY;

#ifdef DEBUG_MEAS_ON
	// sprintf(s, "p%d ", aux1);
	// Usart2Send(s);
		// Usart2Send("n ");
#endif

	return (unsigned short) aux1;
}


void StartSampling (void)
{
    if (lock_vect == LOCK_STANDBY)
    {
// #ifdef DEBUG_MEAS_ON
// 			// sprintf(s, "z%d ", aux1);
// 			// Usart2Send(s);
        // Usart2Send("s");
// #endif
        lock_vect = LOCK_READY_TO_TAKE_SAMPLES;
    }
}

#endif

//--- end of file ---//
