//----------------------------------------------------
// #### PROYECTO KIRNO REDONDA GSM - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### SYNCHRO.C ####################################
//----------------------------------------------------

// Includes --------------------------------------------------------------------
#include "synchro.h"
#include "hard.h"
#include "gpio.h"
#include "adc.h"
#include "stm32f0xx.h"


// Externals -------------------------------------------------------------------
extern volatile unsigned char igrid_timer;
extern volatile unsigned char vgrid_timer;


// Globals ---------------------------------------------------------------------
unsigned short max_vgrid_last = 0;
unsigned short max_vgrid = 0;
unsigned short vgrid_update_samples = 0;
unsigned short max_igrid_last = 0;
unsigned short min_igrid_last = 0;
unsigned short max_igrid = 0;
unsigned short min_igrid = 0;
unsigned char igrid_update_samples = 0;


// Module Functions ------------------------------------------------------------
void UpdateIGrid (void)
{
    unsigned short medida = 0;

    if (igrid_timer)	//cada 400us
    {
        igrid_timer = 0;
        if (igrid_update_samples < IGRID_SAMPLES_RESET)	//50 es toda la senoidal 60 es un ciclo y un octavo
        {
#ifdef VER_1_3
            medida = ReadADC1_SameSampleTime(ADC_Channel_0);
#endif
#ifdef VER_1_2
            medida = ReadADC1_SameSampleTime(ADC_Channel_8);
#endif

            //reviso si es un maximo
            if (medida > max_igrid)
                max_igrid = medida;

            //reviso si es un minimo
            if (medida < min_igrid)
                min_igrid = medida;

            igrid_update_samples++;
        }
        else
        {
            //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
            max_igrid_last = max_igrid;
            min_igrid_last = min_igrid;
            max_igrid = 2048;
            min_igrid = 2048;

            //ajusto para referencia en 1.81V
            //max_igrid = 2256;
            //min_igrid = 2256;

            igrid_update_samples = 0;
        }
    }
}


unsigned short GetIGrid (void)
{
    unsigned short medida;

    if (max_igrid_last > min_igrid_last)
    {
        medida = max_igrid_last - min_igrid_last;	//valor de corriente pico a pico
    }
    else
        medida = 0;

    return medida;
}


void UpdateVGrid (void)		//es una senial rectificada solo positiva
{
    unsigned short medida;

    if (vgrid_timer)	//cada 400us
    {
        vgrid_timer = 0;

        medida = ReadADC1_SameSampleTime(ADC_Channel_1);
        //medida = MAFilterFast(V_GRID_SENSE, v_vgrid);

        if (vgrid_update_samples < VGRID_SAMPLES_RESET)	//512 es toda la senoidal 576 es un ciclo y un octavo
        {
            //reviso si es un maximo
            if (medida > max_vgrid)
                max_vgrid = medida;

            vgrid_update_samples++;
        }
        else
        {
            //paso un ciclo y un octavo completo, seguro tengo maximo y minimos cargados
            max_vgrid_last = max_vgrid;
            max_vgrid = 0;
            vgrid_update_samples = 0;
        }
    }
}

unsigned short GetVGrid (void)
{
    return max_vgrid_last;
}

//--- end of file ---//


