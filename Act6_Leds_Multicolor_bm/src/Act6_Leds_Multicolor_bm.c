/* Copyright 2016, Ing. Daniel Steiner   dhsteiner@gmail.com
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

/** \brief Act6_Leds_Multicolor_bm
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
#include "Act6_Leds_Multicolor_bm.h"       /* Aqui el nuevo mio  */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/


#define  PWM_MAX  100

uint8_t  pwm_cont = PWM_MAX;
uint8_t  valR = 50;
uint8_t  valV = 50;
uint8_t  valA = 50;
//
uint8_t  pwm_valR = 0;
uint8_t  pwm_valV = 0;
uint8_t  pwm_valA = 0;

typedef struct rva {     // Modelo de color, 100 niveles de cada LED Rojo, verde, azul
	uint8_t  mR;
	uint8_t  mV;
	uint8_t  mA;
} RVA;

RVA  Paleta[] = {	{10, 0, 0},
					{10, 10, 0},
					{10, 0, 10},
					{20, 0, 90},
					{20, 50, 0},
					{20, 0, 50},
					{30, 0, 70},
					{30, 10, 0},
					{30, 0, 60},
					{40, 20, 0},
					{40, 0, 30},
					{60, 30, 30},
					{60, 10, 90},
					{80, 0, 20},
					{80, 80, 0},
					{0, 50, 50} }    // 16 colores para el LED RVA (RGB en ingles)
;

void delay( uint64_t i)
{
	for( ; i>0; i--)
	{
		;
	}
}

void PWM_timerIsr()        // @ 100 Hz
{
      if(!pwm_valR--)
    	  ApagarLed(LED_RGB_ROJO);    //

      if(!pwm_valV--)
    	  ApagarLed(LED_RGB_VERDE);    //

      if(!pwm_valA--)
    	  ApagarLed(LED_RGB_AZUL);    //

      if(!pwm_cont--) {
    	  // fin del ciclo
        pwm_cont = PWM_MAX;
        pwm_valR = valR;
        pwm_valV = valV;
        pwm_valA = valA;

        if(pwm_valR)
        	EncenderLed(LED_RGB_ROJO);    //
        if(pwm_valR)
            EncenderLed(LED_RGB_VERDE);   //
        if(pwm_valR)
            EncenderLed(LED_RGB_AZUL);    //
      }
}

void Color_RVA( uint8_t  R, uint8_t  V, uint8_t  A){
	valR = R;
	valV = V;
	valA = A;
}

// Al energizar la placa deben parpadear alternadamente los leds Rojo y Verde,
// 250ms encendido cada led.
void Actualiza_LED( void)
{
		InvertirLed(LED_1);
		InvertirLed(LED_3);
}
/** \brief Main function
 *
 * This is the main entry point of the software
 *
 * Actividad 6: Led Multicolor
 *
Al energizar la placa deben parpadear alternadamente los leds Rojo y Verde,
250ms encendido cada led.

Al presionar la TECLA 1, se debe variar el color del led RGB en, al menos,
16 colores diferentes.

La temporización de los leds se debe realizar a través del RIT Timer.
(ver Temporizadores e Interrupciones)
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
	uint8_t	 TeclaActual;
	uint8_t	 LedActual = LED_2;
	uint64_t RetardoActual = 3000000L;
	uint8_t  i = 0;

   /* inicializaciones */
   
	InicioLeds( );
	Init_Switches( );

	//EncenderLed( LED_RGB_ROJO);
	EncenderLed( LED_1);

	timerInit( 250, &Actualiza_LED);

	timer0Init( 100, &PWM_timerIsr);

	Color_RVA(100, 100, 0);      // ROJO, VERDE, AZUL
   
   /* mi programa principal */
   
    while(1)
    {
      TeclaActual = 0;
      delay( 1000000);

      TeclaActual = Read_Switches();
      switch( TeclaActual)
      {
      case TEC1:
    	  ApagarLed(LED_1);
    	  ApagarLed(LED_3);
    	  // Ver como deshabilitar Interrupcion Timer
    	  timerInit( 10000, &Actualiza_LED);   // 10 segundos

      	  if(i < 16 ) i++;
      	Color_RVA(Paleta[i].mR, Paleta[i].mV, Paleta[i].mA);      // ROJO, VERDE, AZUL
      	  break;

      }

    };
    
	return 0;


}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

