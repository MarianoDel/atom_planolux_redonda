/*
 * adc.c
 *
 *  Created on: 04/05/2015
 *      Author: Mariano
 */
#include "adc.h"
#include "stm32f0xx.h"
#include "hard.h"

#ifdef DEBUG_ON
#include "uart.h"
#include <stdio.h>
#endif


//--- VARIABLES EXTERNAS ---//
extern volatile unsigned short adc_ch [];
extern volatile unsigned short tt_take_photo_sample;

#ifdef ADC_WITH_INT
extern volatile unsigned char seq_ready;
#endif

#ifdef ADC_WITH_TEMP_SENSE
extern volatile unsigned short tt_take_temp_sample;
#endif

//--- VARIABLES GLOBALES ---//
#ifdef ADC_WITH_INT
volatile unsigned short * p_channel;
#endif

#ifdef ADC_WITH_TEMP_SENSE
// ------- del sensor de Temperatura -------
unsigned short board_temp [SIZEOF_BOARD_TEMP];
unsigned short last_temp = 0;
unsigned char board_temp_index = 0;
unsigned char new_temp_sample = 0;
#endif

// ------- del PhotoTransistor -------
#define SIZEOF_PHOTO_TRANS		32
#define DIVISOR_PHOTO			5
unsigned short VoltagePhoto [SIZEOF_PHOTO_TRANS];
unsigned char photo_index = 0;
unsigned short last_photo = 0;
unsigned char new_photo_sample = 0;

// ------- de los filtros y mediciones -------
#ifdef POWER_MEAS_WITH_SAMPLES
unsigned short isense [VECT_SAMPLES];
unsigned short vsense [VECT_SAMPLES];

int power_aux [VECT_SAMPLES];
volatile unsigned char lock_vect = LOCK_STANDBY;
volatile unsigned char power_vector_index = 0;
#endif



//Single conversion mode (CONT=0)
//In Single conversion mode, the ADC performs a single sequence of conversions,
//converting all the channels once.

//Continuous conversion mode (CONT=1)
//In continuous conversion mode, when a software or hardware trigger event occurs,
//the ADC performs a sequence of conversions, converting all the channels once and then
//automatically re-starts and continuously performs the same sequence of conversions

//Discontinuous mode (DISCEN)
//In this mode (DISCEN=1), a hardware or software trigger event is required to start
//each conversion defined in the sequence. Only with (CONT=0)

void AdcConfig (void)
{
	if (!RCC_ADC_CLK)
		RCC_ADC_CLK_ON;

	// preseteo los registros a default, la mayoria necesita tener ADC apagado
	ADC1->CR = 0x00000000;
	ADC1->IER = 0x00000000;
	ADC1->CFGR1 = 0x00000000;
	ADC1->CFGR2 = 0x00000000;
	ADC1->SMPR = 0x00000000;
	ADC1->TR = 0x0FFF0000;
	ADC1->CHSELR = 0x00000000;

	//set clock
	ADC1->CFGR2 = ADC_ClockMode_SynClkDiv4;

	//set resolution, trigger & Continuos or Discontinuous
	ADC1->CFGR1 |= ADC_Resolution_12b | ADC_ExternalTrigConvEdge_Rising | ADC_ExternalTrigConv_T3_TRGO;	//recordar ADC1->CR |= ADC_CR_ADSTART
	//ADC1->CFGR1 |= ADC_Resolution_12b | ADC_ExternalTrigConvEdge_Rising | ADC_ExternalTrigConv_T1_TRGO;
	//ADC1->CFGR1 |= ADC_Resolution_12b | ADC_CFGR1_DISCEN;
	// ADC1->CFGR1 |= ADC_Resolution_12b;

	//DMA Config
	//ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;

	//set sampling time
	ADC1->SMPR |= ADC_SampleTime_239_5Cycles;
	// ADC1->SMPR |= ADC_SampleTime_71_5Cycles;
	// ADC1->SMPR |= ADC_SampleTime_55_5Cycles;
	// ADC1->SMPR |= ADC_SampleTime_41_5Cycles;		//17.39 son SP 420
	// ADC1->SMPR |= ADC_SampleTime_28_5Cycles;		//17.39 son SP 420
	//ADC1->SMPR |= ADC_SampleTime_7_5Cycles;		//17.36 de salida son SP 420 pero a veces pega
													//las dos int (usar DMA?) y pierde el valor intermedio
	//ADC1->SMPR |= ADC_SampleTime_1_5Cycles;			//20.7 de salida son SP 420 (regula mal)

#ifdef ADC_WITH_INT
	//set channel selection
	ADC1->CHSELR |= ADC_Channel_0 | ADC_Channel_1 | ADC_Channel_8;

	//set interrupts
	ADC1->IER |= ADC_IT_EOC;

	//set pointer
	p_channel = &adc_ch[0];

	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	NVIC_SetPriority(ADC1_COMP_IRQn, 3);
#endif

#ifdef ADC_WITH_TEMP_SENSE
	ADC->CCR |= ADC_CCR_TSEN;
#endif

	//calibrar ADC
	ADCGetCalibrationFactor();

#ifdef POWER_MEAS_WITH_SAMPLES
	power_vector_index = 0;
#endif
	// Enable ADC1
	ADC1->CR |= ADC_CR_ADEN;
}

#ifdef ADC_WITH_INT
void ADC1_COMP_IRQHandler (void)
{
	if (ADC1->ISR & ADC_IT_EOC)
	{
		if (ADC1->ISR & ADC_IT_EOSEQ)	//seguro que es channel8 en posicion 3
		{
			p_channel = &adc_ch[2];
			*p_channel = ADC1->DR;
			p_channel = &adc_ch[0];
			seq_ready = 1;
		}
		else
		{
			*p_channel = ADC1->DR;		//
			if (p_channel < &adc_ch[2])
				p_channel++;

			seq_ready = 0;		//lo agrego por si el main no blanquea la secuencia
		}

#ifdef POWER_MEAS_WITH_SAMPLES
		if ((seq_ready) && (lock_vect == LOCK_READY_TO_TAKE_SAMPLES))
		{
			LED_ON;
			//cargo 2 vectores, V e I de 80 posiciones cada uno (circular) sampleado de 4KHz
			isense[power_vector_index] = I_Sense;
			vsense[power_vector_index] = V_Sense;

			if (power_vector_index < (VECT_SAMPLES - 1))
				power_vector_index++;
			else
			{
				LED_OFF;
				//termine de cargar el vector, aviso que esta listo
				lock_vect = LOCK_SAMPLES_TAKEN;
				power_vector_index = 0;
/////////
				// Usart2Send("y");


			}

		}
#endif

		//clear pending
		ADC1->ISR |= ADC_IT_EOC | ADC_IT_EOSEQ;
	}
}
#endif


//Setea el sample time en el ADC
void SetADC1_SampleTime (void)
{
	uint32_t tmpreg = 0;

	/* Clear the Sampling time Selection bits */
	tmpreg &= ~ADC_SMPR1_SMPR;

	/* Set the ADC Sampling Time register */
	tmpreg |= (uint32_t)ADC_SampleTime_239_5Cycles;

	/* Configure the ADC Sample time register */
	ADC1->SMPR = tmpreg ;
}


//lee el ADC sin cambiar el sample time anterior
unsigned short ReadADC1_SameSampleTime (unsigned int channel)
{
	// Configure the ADC Channel
	ADC1->CHSELR = channel;

	// Start the conversion
	ADC1->CR |= (uint32_t)ADC_CR_ADSTART;

	// Wait until conversion completion
	while((ADC1->ISR & ADC_ISR_EOC) == 0);

	// Get the conversion value
	return (uint16_t) ADC1->DR;
}

unsigned short ReadADC1Check (unsigned char channel)
{
	if (ADC1->CR & 0x01)			//reviso ADEN
		return 0xFFFF;

	//espero que este listo para convertir
	while ((ADC1->ISR & 0x01) == 0);	//espero ARDY = 1

	if ((ADC1->CFGR1 & 0x00010000) == 0)			//reviso DISCONTINUOS = 1
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00002000)					//reviso CONT = 0
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000C00)					//reviso TRIGGER = 00
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000020)					//reviso ALIGN = 0
		return 0xFFFF;

	if (ADC1->CFGR1 & 0x00000018)					//reviso RES = 00
		return 0xFFFF;

	//espero que no se este convirtiendo ADCSTART = 0
	while ((ADC1->CR & 0x02) != 0);	//espero ADCSTART = 0

	ADC1->CHSELR = 0x00000001;	//solo convierto CH0

	return 1;
}

unsigned int ADCGetCalibrationFactor (void)
{
  uint32_t tmpreg = 0, calibrationcounter = 0, calibrationstatus = 0;

  /* Set the ADC calibartion */
  ADC1->CR |= (uint32_t)ADC_CR_ADCAL;

  /* Wait until no ADC calibration is completed */
  do
  {
    calibrationstatus = ADC1->CR & ADC_CR_ADCAL;
    calibrationcounter++;
  } while((calibrationcounter != CALIBRATION_TIMEOUT) && (calibrationstatus != 0x00));

  if((uint32_t)(ADC1->CR & ADC_CR_ADCAL) == RESET)
  {
    /*Get the calibration factor from the ADC data register */
    tmpreg = ADC1->DR;
  }
  else
  {
    /* Error factor */
    tmpreg = 0x00000000;
  }
  return tmpreg;
}

#ifdef ADC_WITH_TEMP_SENSE
void UpdateTemp(void)
{
	//hago update cada 1 seg
	if (!tt_take_temp_sample)
	{
		tt_take_temp_sample = 1000;

		board_temp [board_temp_index] = ReadADC1_SameSampleTime(ADC_CH16);
		//board_temp [0] = ReadADC1_SameSampleTime(ADC_CH16);

		if (board_temp_index < (SIZEOF_BOARD_TEMP - 1))
			board_temp_index++;
		else
			board_temp_index = 0;

		new_temp_sample = 1;
	}
}

//devuelve el valor promedio de la temperatura
//si existen nuevas muestras hace la cuenta, sino contesta el ultimo valor calculado
unsigned short GetTemp (void)
{
    unsigned char i;
    unsigned int t = 0;

    if (new_temp_sample)
    {
        for (i = 0; i < SIZEOF_BOARD_TEMP; i++)
            t += board_temp[i];

        last_temp = t >> 3;
        new_temp_sample = 0;
    }

    return ConvertTemp(last_temp);
}

void FillTempBuffer (void)
{
	unsigned char i;
	unsigned short dummy;

	dummy = ReadADC1_SameSampleTime(ADC_CH16);

	for (i = 0; i < SIZEOF_BOARD_TEMP; i++)
		 board_temp[i] = dummy;

}

short ConvertTemp (unsigned short adc_temp)
{
	int32_t temperature; /* will contain the temperature in degree Celsius */
	//temperature = (((int32_t) ADC1->DR * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );
	temperature = (int32_t) *TEMP30_CAL_ADDR - adc_temp;
	temperature *= 1000;
	temperature = temperature / 5336;	//4.3mV / °C
	temperature = temperature + 30;

	return (short) temperature;
}
#endif //ADC_WITH_TEMP_SENSE

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
	char s [10];
	unsigned char i;
	int aux1 = 0;

	lock_vect = LOCK_PROCESSING;
	//promedio isense para conocer zero current
	for (i = 0; i < VECT_SAMPLES; i++)
		aux1 += isense[i];

	aux1 = aux1 / VECT_SAMPLES;
	// aux1 = 2048;

	sprintf(s, "z%d ", aux1);
	Usart2Send(s);

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

#ifdef DEBUG_ON
	// sprintf(s, "p%d ", aux1);
	// Usart2Send(s);
		// Usart2Send("n ");
#endif

	return (unsigned short) aux1;
}

void ADCStartSampling (void)
{
	if (lock_vect == LOCK_STANDBY)
	{
// #ifdef DEBUG_ON
// 			// sprintf(s, "z%d ", aux1);
// 			// Usart2Send(s);
		// Usart2Send("s");
// #endif
		lock_vect = LOCK_READY_TO_TAKE_SAMPLES;
	}
}

#endif
