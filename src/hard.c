/*
 * hard.c
 *
 *  Created on: 28/03/2017
 *      Author: Mariano
 */

#include "hard.h"
#include "tim.h"
#include "stm32f0xx.h"

/* Externals variables ---------------------------------------------------------*/
extern unsigned short timer_relay;


/* Global variables ------------------------------------------------------------*/
//unsigned char relay_state = 0;
enum Relay_State relay_state = ST_OFF;
unsigned char last_edge;

/* Module Functions ------------------------------------------------------------*/

//Pide conectar el relay
void RelayOn (void)
{
	if (!RelayIsOn())
	{
		relay_state = ST_WAIT_ON;
		timer_relay = TT_RELAY;
	}
}

//Pide desconectar el relay
void RelayOff (void)
{
	if (!RelayIsOff())
	{
		relay_state = ST_WAIT_OFF;
		timer_relay = TT_RELAY;
	}
}

//Revisa el estado del relay
unsigned char RelayIsOn (void)
{
	if ((relay_state == ST_WAIT_ON) ||
			(relay_state == ST_DELAYED_ON) ||
			(relay_state == ST_ON))
		return 1;
	else
		return 0;
}

//Revisa el estado del relay
unsigned char RelayIsOff (void)
{
	if ((relay_state == ST_WAIT_OFF) ||
			(relay_state == ST_DELAYED_OFF) ||
			(relay_state == ST_OFF))
		return 1;
	else
		return 0;
}

//chequeo continuo del estado del relay
void UpdateRelay (void)
{
	unsigned char edge = 0;

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
unsigned char GetNew1to10 (unsigned short light)	//prendo en 3722 a 4095 tengo 373 puntos
{
	unsigned short new_light = 0;

	if (light > VOLTAGE_PHOTO_ON)
	{
		new_light = light - VOLTAGE_PHOTO_ON;
	}
	new_light += PWM_MIN;

	if (new_light > 255)
		new_light = 255;

	// if (light < VOLTAGE_PHOTO_ON)
	// 	new_light = PWM_MIN;
	// else
	// {
	// 	new_light = light - VOLTAGE_PHOTO_ON;
	// 	new_light += PWM_MIN;
	// }

	return (unsigned char) new_light;
}
#endif
