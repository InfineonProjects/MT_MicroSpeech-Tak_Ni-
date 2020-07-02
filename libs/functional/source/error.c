/*
 * error.c
 *
 *  Created on: 20 квіт. 2020 р.
 *      Author: yabe
 */

#include "error.h"

/*******************************************************************************
* Function Name: reboot
***************************************
* Summary:
* 	Blinks red LED for 2.5 sec and than reboot the CPU.
*
*******************************************************************************/
void reboot(){
	blinking_led(BUTTON3, 5, 500);
	turn_led_on_off(BUTTON3, 1);
	NVIC_SystemReset();
}


/*******************************************************************************
* Function Name: reboot_with_error
***************************************
* Summary:
* 	Show error massage, blinks red LED for 2.5 sec and than reboot the CPU.
*
* Parameters:
* 	*base	-	address of the UART component;
* 	massage	-	massage with description of the error;
*
*******************************************************************************/
void reboot_with_error(char massage[]){
	printf(	"\n\r*********** ERROR **********"
			"\n\rReboot the CPU"
			"\n%s\n", massage);
	blinking_led(BUTTON3, 5, 500);
	turn_led_on_off(BUTTON3, 1);
	NVIC_SystemReset();
}


/*******************************************************************************
* Function Name: halt
***************************************
* Summary:
* 	Blinks red LED for 2.5 sec and than halt the CPU.
*
*******************************************************************************/
void halt(){
	blinking_led(BUTTON4, 5, 500);
	turn_led_on_off(BUTTON4, 1);
	CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: halt_with_error
***************************************
* Summary:
* 	Show error massage, blinks red LED for 2.5 sec and than halt the CPU.
*
* Parameters:
* 	*base	-	address of the UART component;
* 	massage	-	massage with description of the error;
*
*******************************************************************************/
void halt_with_error(char massage[]){
	printf(	"\n\r*********** ERROR **********"
			"\n\rHalt the CPU"
			"\n%s\n", massage);
	blinking_led(BUTTON4, 5, 500);
	turn_led_on_off(BUTTON4, 1);
	CY_ASSERT(0);
}

