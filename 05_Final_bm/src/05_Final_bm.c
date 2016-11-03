/* Copyright 2016,
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

/** Organizador Separador de Billetes  source file Ing. Daniel Steiner
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

   Ing. Telmo Moya         Interfz serie con lector, display GLCD
   Ing. Daniel Steiner     Cuerpo del controlador, sensores, actuadores

 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/

#include "05_Final_bm.h"       /*   */

#define MOTOR_CINTA  1
#define PUERTA_GUIA  2
#define LOTE_LISTO   3

#define LED_RGB_ROJO  4
#define LED_RGB_VERDE 5
#define LED_RGB_AZUL  6

/*==================[internal data definition]===============================*/
uint8_t  ContBilletes  = 0;
uint8_t	 BilleteActual;
uint8_t	 BilleteEsperado = 20;

/*==================[internal functions definition]==========================*/
uint8_t  LeerBillete( )
{
   uint8_t  CodigoBillete = 20;

	PulsosLed(LED_RGB_AZUL, 2);
	return( CodigoBillete);
}

void delay( uint64_t tiempo)
{
	while( tiempo--);
}

/** \brief Main function
 * This is the main entry point of the software.
 * \returns 0
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
	uint8_t		TeclaActual;


   /* inicializaciones */
   
	InicioLeds( );
	Init_Switches( );
	init_UART_FTDI_EDUCIAA();    // VER 9600 / por defecto 115200,8,n,1

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

      while( ContBilletes < 6)   // DS_DEBUG 100
      {
    	  delay( 10000000);    // DS_DEBUG - sacar

    	  if( ( BilleteActual = LeerBillete() ) != 0 ) // Cero mientras no pasen un billete
    	  {
    		  EncenderLed( MOTOR_CINTA);

    		  if( BilleteActual == BilleteEsperado)  // Actualiza PUERTA_GUIA
    		  {
    			  EncenderLed( PUERTA_GUIA); // PUERTA_GUIA a Derecha OK
    			  ContBilletes++;
    		  }
    		  else
    		  {
    			  ApagarLed( PUERTA_GUIA); // PUERTA_GUIA a Izquierda RECHAZADO
    		  }
    	  }
      }

      // Fin del ciclo de 100 billetes
      //
      ApagarLed( MOTOR_CINTA);        // Detiene cinta
      EncenderLed( LOTE_LISTO);       // Aviso de ciclo listo

      while(1)     // Fin del programa, requiere RESET
      { };
    
	return 0;


}

/*==================[end of file]============================================*/

