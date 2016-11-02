/* Copyright 2016, XXXXXXXXX  
 * All rights reserved.
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

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/
/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal LED Driver
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

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif
#include "led.h"

/*==================[macros and definitions]=================================*/
#define LED1_MUX_GROUP     2
#define LED1_MUX_PIN       10
#define LED1_GPIO_PORT     0
#define LED1_GPIO_PIN      14

#define LED2_MUX_GROUP     2
#define LED2_MUX_PIN       11
#define LED2_GPIO_PORT     1
#define LED2_GPIO_PIN      11

#define LED3_MUX_GROUP     2
#define LED3_MUX_PIN       12
#define LED3_GPIO_PORT     1
#define LED3_GPIO_PIN      12

#define LED_RGB_R_MUX_GROUP   2
#define LED_RGB_R_MUX_PIN     0
#define LED_RGB_R_GPIO_PORT   5
#define LED_RGB_R_GPIO_PIN    0

#define LED_RGB_G_MUX_GROUP   2
#define LED_RGB_G_MUX_PIN     1
#define LED_RGB_G_GPIO_PORT   5
#define LED_RGB_G_GPIO_PIN    1

#define LED_RGB_B_MUX_GROUP   2
#define LED_RGB_B_MUX_PIN     2
#define LED_RGB_B_GPIO_PORT   5
#define LED_RGB_B_GPIO_PIN    2

//#define LED3_R_BIT      0

#define DIRECCION_SALIDA   1
#define DIRECCION_ENTRADA  0
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


void InicioLeds( void)    //  los leds de CIAA
{
	Chip_GPIO_Init(LPC_GPIO_PORT);

	Chip_SCU_PinMux( LED1_MUX_GROUP, LED1_MUX_PIN, MD_PUP, FUNC0);
	Chip_SCU_PinMux( LED2_MUX_GROUP, LED2_MUX_PIN, MD_PUP, FUNC0);
	Chip_SCU_PinMux( LED3_MUX_GROUP, LED3_MUX_PIN, MD_PUP, FUNC0); /*  pull up */
//	A continuacion, el modo (entrada o salida) de cada pin con la funcion:

	Chip_GPIO_SetDir( LPC_GPIO_PORT, LED1_GPIO_PORT, 1 << LED1_GPIO_PIN, DIRECCION_SALIDA);
	Chip_GPIO_SetDir( LPC_GPIO_PORT, LED2_GPIO_PORT, 1 << LED2_GPIO_PIN, DIRECCION_SALIDA);
	Chip_GPIO_SetDir( LPC_GPIO_PORT, LED3_GPIO_PORT, 1 << LED3_GPIO_PIN, DIRECCION_SALIDA);

	// led RGB
	Chip_SCU_PinMux(LED_RGB_R_MUX_GROUP,LED_RGB_R_MUX_PIN,MD_PUP,FUNC4);
	Chip_SCU_PinMux(LED_RGB_R_MUX_GROUP,LED_RGB_G_MUX_PIN,MD_PUP,FUNC4);
	Chip_SCU_PinMux(LED_RGB_R_MUX_GROUP,LED_RGB_B_MUX_PIN,MD_PUP,FUNC4);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, LED_RGB_R_GPIO_PORT,(1<<LED_RGB_R_GPIO_PIN)|(1<<LED_RGB_G_GPIO_PIN)|(1<<LED_RGB_B_GPIO_PIN),1);

	Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED1_GPIO_PORT,1<<LED1_GPIO_PIN);
	Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED2_GPIO_PORT,1<<LED2_GPIO_PIN);
	Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED3_GPIO_PORT,1<<LED3_GPIO_PIN);
	//
	Chip_GPIO_ClearValue(LPC_GPIO_PORT, LED_RGB_R_GPIO_PORT,(1<<LED_RGB_R_GPIO_PIN)|(1<<LED_RGB_G_GPIO_PIN)|(1<<LED_RGB_B_GPIO_PIN));

}

void EncenderLed(uint8_t  led)
{
	if (led == 1)
	{
		Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LED1_GPIO_PORT, LED1_GPIO_PIN);
	}

	if (led == 2)
	{
		Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LED2_GPIO_PORT, LED2_GPIO_PIN);
	}

	if (led == 3)
	{
		Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LED3_GPIO_PORT, LED3_GPIO_PIN);
	}

	if (led == 4)
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,LED_RGB_R_GPIO_PORT,LED_RGB_R_GPIO_PIN);
	}

	if (led == 5)
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,LED_RGB_G_GPIO_PORT,LED_RGB_G_GPIO_PIN);
	}

	if (led == 6)
	{
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,LED_RGB_B_GPIO_PORT,LED_RGB_B_GPIO_PIN);
	}

}

void ApagarLed(uint8_t  led)
{
	if (led == 1)
	{
		Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LED1_GPIO_PORT, LED1_GPIO_PIN);
	}

	if (led == 2)
	{
		Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LED2_GPIO_PORT, LED2_GPIO_PIN);
	}

	if (led == 3)
	{
		Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LED3_GPIO_PORT, LED3_GPIO_PIN);
	}

	if (led == 4)
	{
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,LED_RGB_R_GPIO_PORT,LED_RGB_R_GPIO_PIN);
	}

	if (led == 5)
	{
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,LED_RGB_G_GPIO_PORT,LED_RGB_G_GPIO_PIN);
	}

	if (led == 6)
	{
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,LED_RGB_B_GPIO_PORT,LED_RGB_B_GPIO_PIN);
	}

}

void InvertirLed(uint8_t  led)
{

	if (led == 1)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED1_GPIO_PORT, LED1_GPIO_PIN);
	}

	if (led == 2)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED2_GPIO_PORT, LED2_GPIO_PIN);
	}

	if (led == 3)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED3_GPIO_PORT, LED3_GPIO_PIN);
	}

	if (led == 4)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_R_GPIO_PORT,LED_RGB_R_GPIO_PIN);
	}

	if (led == 5)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_G_GPIO_PORT,LED_RGB_G_GPIO_PIN);
	}

	if (led == 6)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_B_GPIO_PORT,LED_RGB_B_GPIO_PIN);
	}

}

void PulsosLed(uint8_t  led, int  CantPulsos)
{
 uint64_t i;

 CantPulsos = CantPulsos * 2;

 while( CantPulsos > 0 )
 {
	if (led == 1)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED1_GPIO_PORT, LED1_GPIO_PIN);
	}

	if (led == 2)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED2_GPIO_PORT, LED2_GPIO_PIN);
	}

	if (led == 3)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, LED3_GPIO_PORT, LED3_GPIO_PIN);
	}

	if (led == 4)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_R_GPIO_PORT,LED_RGB_R_GPIO_PIN);
	}

	if (led == 5)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_G_GPIO_PORT,LED_RGB_G_GPIO_PIN);
	}

	if (led == 6)
	{
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT,LED_RGB_B_GPIO_PORT,LED_RGB_B_GPIO_PIN);
	}

	//
	CantPulsos--;

	for( i=3000000; i>0; i--)
	{
		;
	}
 }
}

//	Para setear y resetear los pines, existen numerosas funciones, entre ellas:
//	Chip GPIO ClearValue();
//	Chip GPIO SetValue();
//	Chip GPIO SetPinOutLow();
//	Chip GPIO SetPinOutHigh();
//	Chip GPIO SetPortOutHigh();
//	Chip GPIO SetPinToggle();
//	Chip GPIO SetPortToggle();


/*==================[end of file]============================================*/

