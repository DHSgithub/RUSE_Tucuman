/* Copyright 2016, Ing. Daniel Steiner   dhsteiner@gmail.com
 *
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

/** \brief Control de Leds
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
#include "Act5_Ctrl_Leds_bm.h"       /* Aqui el nuevo mio  */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
void delay( uint64_t i)
{
	for( ; i>0; i--)
	{
		;
	}
}
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/




/** \brief Main function
 *
 * This is the main entry point of the software
 * Corresponde a la actividad 5 Control de Leds
 *
 * TEC1 derecha
 * TEC2 izquierda
 *
 *En clase se agrega esta funcionalidad a la original:
 *
 * TEC3 menor frecuencia de parpadeo
 * TEC4 mayor frecuencia
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

   /* inicializaciones */
   
	InicioLeds( );
	Init_Switches( );
   
   /* mi programa principal */
   
    while(1)
    {
      TeclaActual = 0;
      delay( 10000);

      TeclaActual = Read_Switches();
      switch( TeclaActual)
      {
      case TEC1:
      	  if(LedActual > 1 ) LedActual--;
      	  break;

      case TEC2:
    	  if(LedActual < 3 ) LedActual++;
      	  break;

      case TEC3:
    	  if( RetardoActual < 6000000L ) RetardoActual += 100000L;
      	  break;

      case TEC4:
    	  if( RetardoActual > 100000L )  RetardoActual -= 100000L;
      	  break;

      }
    	EncenderLed( LedActual);
    	delay( RetardoActual);
    	ApagarLed( LedActual);
    	delay( RetardoActual);
    };
    
	return 0;


}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

