/* Copyright 2016, XXXX
 *
 *  * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "04_ADC_bm.h"       /* Aqui el nuevo mio  */

#define LED_1  1
#define LED_2  2
#define LED_3  3
#define LED_RGB_ROJO  4
#define LED_RGB_VERDE 5
#define LED_RGB_AZUL  6

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

uint16_t senial[] = {512,544,576,608,639,670,700,730,759,786,813,838,862,885,907,926,944,961,975,

988,999,1008,1015,1020,1023,1024,1023,1020,1015,1008,999,988,975,961,944,926,907,885,862,838,813,786,759,

730,700,670,639,608,576,544,512,480,448,416,385,354,324,294,265,238,211,186,162,139,117,98,80,63,49,36,25,

16,9,4,1,0,1,4,9,16,25,36,49,63,80,98,117,139,162,186,211,238,265,294,324,354,385,416,448,480};

uint16_t	Dato_DAC[100];


#define  PWM_MAX  100

uint8_t  pwm_cont = PWM_MAX;
uint8_t  val = 50;
uint8_t  pwm_val = 50;


void PWM_timerIsr()        // @ 100 Hz
{
      if(!pwm_val--)
    	  ApagarLed(LED_3);    // Cortar Potencia

      if(!pwm_cont--) {
    	  // fin del ciclo
        pwm_cont = PWM_MAX;
        pwm_val = val;

        if(pwm_val)
        	EncenderLed(LED_3);    // Subir Potencia
      }
}



void delay( uint64_t i)
{
	for( ; i>0; i--)
	{
		;
	}
}

void Actualiza_DAC( void)
{
	static uint8_t indice;

	if( indice < 100)
		indice++;
	else{
		indice = 0;
		InvertirLed(LED_2);
	}

	update_DAC_value( senial[indice]);     // Dato_DAC[indice]

}

int main(void)
{
	uint8_t		TeclaActual;
	uint16_t	Valor_ADC;
	//uint32_t	PeriodoActual = 10;
	//uint16_t	i;

   /* inicializaciones */
   
	InicioLeds( );
	Init_Switches( );
	//init_DAC_EDUCIAA( );
	init_ADC_EDUCIAA( );

	ADC_Sel( CH1);
    //#define  CH1 ADC_CH1

	timer0Init( 100, &PWM_timerIsr);


	PulsosLed(LED_1, 5);
	delay( 2000000L);


   /* mi programa principal */
   

    while(1)
    {
      TeclaActual = 0;

      TeclaActual = Read_Switches();
      switch( TeclaActual)
      {
      case TEC1:

      	  break;

      case TEC2:

      	  break;

      case TEC3:

      	  break;

      case TEC4:

    	  break;

      }
      Valor_ADC = read_ADC_value_pooling();    // leer nuevo valor

      val = Valor_ADC / 10;

      if( (Valor_ADC < 200) | (Valor_ADC > 800) )
      {
    	  EncenderLed(LED_2);
      }
      else
      {
    	  ApagarLed(LED_2);
      }

    };
    
	return 0;


}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

