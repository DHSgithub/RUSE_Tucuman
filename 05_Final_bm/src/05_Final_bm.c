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

/** Organizador Separador de Billetes  source file
 **
 ** This is a  example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * grupo de trabajo:

   Ing. Telmo Moya
   Ing. Daniel Steiner

 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
//#include <stdio.h>
#include "05_Final_bm.h"       /* Aqui el nuevo mio  */

#define MOTOR_CINTA  1
#define PUERTA_GUIA  2
#define LOTE_LISTO   3

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


uint8_t  ContBilletes  = 0;
uint8_t	 BilleteActual;
uint8_t	 BilleteEsperado = 20;


uint8_t  LeerBillete( )
{
  uint8_t  CodigoBillete = 20;

	//
	PulsosLed(LED_RGB_AZUL, 2);
	return( CodigoBillete);
}

void delay( uint64_t tiempo)
{
	while( tiempo--);

}

void Actualiza_DAC( void)
{

}

int main(void)
{
	uint8_t		TeclaActual;


   /* inicializaciones */
   
	InicioLeds( );
	Init_Switches( );
	init_UART_FTDI_EDUCIAA();    // 9600 / por defecto 115200,8,n,1

   /* mi programa principal */

      TeclaActual = 0;

      ApagarLed( MOTOR_CINTA);
      //
      // display info inicial
      //

      do
      {
    	  TeclaActual = Read_Switches();
      }
      while( TeclaActual != TEC1);     //  ESPERA BOTON DE INICIO CICLO

      // iniciando un ciclo de 100 billetes

      while( ContBilletes <= 6)
      {
    	  delay( 10000000);    // 1 seg... aprox

    	  if( ( BilleteActual = LeerBillete() ) != 0 )    // Cero mientras no pasen un billete
    	  {
    		  EncenderLed( MOTOR_CINTA);

    		  if( BilleteActual == BilleteEsperado)  // PUERTA_GUIA
    		  {
    			  EncenderLed( PUERTA_GUIA); // PUERTA_GUIA a Derecha
    			  ContBilletes++;
    		  }
    		  else
    		  {
    			  ApagarLed( PUERTA_GUIA); // PUERTA_GUIA a Izquierda
    		  }
    	  }
      }

      ApagarLed( MOTOR_CINTA);        // Detiene cinta
      EncenderLed( LOTE_LISTO);       // Aviso de ciclo listo

      while(1)
      { };
    
	return 0;


}

/*==================[end of file]============================================*/

