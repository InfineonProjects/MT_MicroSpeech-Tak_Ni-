/*
 * led.c
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#include "led.h"

/*******************************************************************************
* Function Name: initialize_led
***************************************
* Summary:
* 	Set the default parameters for the LEDs.
* 		BUTTON2 (Blue RGB) is inverse:
* 			brightness_passive 	-	is the max value;
* 			brightness_active 	-	is the min value;
*
* Parameters:
*
* Return:
*	cy_rslt_t - error status. Returns CY_RSLT_SUCCESS if succeeded.
*
*******************************************************************************/
 void initialize_led(){
	cy_rslt_t result;

    led[BUTTON0].pin = CYBSP_LED_RGB_RED;
   	led[BUTTON1].pin = CYBSP_LED_RGB_GREEN;
   	led[BUTTON2].pin = CYBSP_LED_RGB_BLUE;
   	led[BUTTON3].pin = CYBSP_LED9;
   	led[BUTTON4].pin = CYBSP_LED8;
   	led[BUTTON0].status = 0;
   	led[BUTTON1].status = 0;
   	led[BUTTON2].status = 0;
   	led[BUTTON3].status = 0;
   	led[BUTTON4].status = 0;
	led[BUTTON0].brightness_passive = 80;
   	led[BUTTON1].brightness_passive = 99;
   	led[BUTTON2].brightness_passive = 75;
   	led[BUTTON3].brightness_passive = 95;
   	led[BUTTON4].brightness_passive = 95;
   	led[BUTTON0].brightness_active = 10;
   	led[BUTTON1].brightness_active = 80;
   	led[BUTTON2].brightness_active = 100;
   	led[BUTTON3].brightness_active = 80;
   	led[BUTTON4].brightness_active = 80;

	for (int i=0; i<NUM_LEDS; i++){
		result = cyhal_pwm_init(&led[i].pwm_control, led[i].pin, NULL);
		if(result != CY_RSLT_SUCCESS)
			halt_with_error("\tLED -> initialize_led() ->"
							"\n\r\t\t\t->tcyhal_pwm_init()");

		/* Set the PWM output frequency and duty cycle */
		result = cyhal_pwm_set_duty_cycle(&led[i].pwm_control, led[i].brightness_passive, FREQ);
		if(result != CY_RSLT_SUCCESS)
			halt_with_error("\tLED -> initialize_led() ->"
							"\n\r\t\t\t->cyhal_pwm_set_duty_cycle()");

		/* Start the PWM */
		result = cyhal_pwm_start(&led[i].pwm_control);
		if(result != CY_RSLT_SUCCESS)
			halt_with_error("\tLED -> initialize_led() ->"
							"\n\r\t\t\t->cyhal_pwm_start()");
	}

	/* Set the PWM output frequency and duty cycle */
	result = cyhal_pwm_set_duty_cycle(&led[BUTTON4].pwm_control, led[BUTTON4].brightness_active, FREQ);
	if(result != CY_RSLT_SUCCESS)
		halt_with_error("\tLED -> initialize_led() ->"
						"\n\r\t\t\t->cyhal_pwm_set_duty_cycle()");
	// To check LEDs.
	Cy_SysLib_Delay(300);

	for (int i=0; i<NUM_LEDS; i++){
		/* Set the PWM output frequency and duty cycle */
		result = cyhal_pwm_set_duty_cycle(&led[i].pwm_control, 100, FREQ);
		if(result != CY_RSLT_SUCCESS)
			halt_with_error("\tLED -> initialize_led() ->"
							"\n\r\t\t\t->cyhal_pwm_set_duty_cycle()");
	}
}


 /*******************************************************************************
 * Function Name: change_led_status
 ***************************************
 * Summary:
 * 	Change led status (0, 1) and brightness (passive, active).
 *
 * Parameters:
 *	led_pos	-	position of led which status and brightness will change.
 *
 *******************************************************************************/
  void change_led_status(uint8_t led_pos){
 	change_led_duty_cycle(BUTTON3, 100);

 	if (0 == led[led_pos].status){
 		led[led_pos].status = 1;
 		change_led_duty_cycle(led_pos, led[led_pos].brightness_active);
 	}
 	else{
 		led[led_pos].status = 0;
 		change_led_duty_cycle(led_pos, led[led_pos].brightness_passive);
 	}
 }


 /*******************************************************************************
 * Function Name: change_led_duty_cycle
 ***************************************
 * Summary:
 * 	Change duty cycle, i.e. brightness of led.
 *
 * Parameters:
 *	led_pos		-	position of led which brightness will change.
 *	duty_cycle	-	the int number in range [0, 100].
 *
 *******************************************************************************/
 void change_led_duty_cycle(uint8_t led_pos, uint32_t duty_cycle){
	cy_rslt_t result = cyhal_pwm_set_duty_cycle(&led[led_pos].pwm_control, duty_cycle, FREQ);
	if(result != CY_RSLT_SUCCESS)
				halt_with_error("\tLED -> change_led_duty_cycle() ->"
								"\n\r\t\t\t->cyhal_pwm_set_duty_cycle()");
}


/******************************************************************************
 * Function Name: turn_led_on_off
 ***************************************
 * Summary:
 * 	Turn led on/off.
 *
 * Parameters:
 *	led_pos	-	position of the LED  which will be turn on/off.
 *	state	-	can be on - 1, and off - 0.
 *
 * Return:
 *
 *****************************************************************************/
 void turn_led_on_off(uint8_t led_pos, uint8_t state){
	cy_rslt_t result;
	if (1 == state)
		result = cyhal_pwm_start(&led[led_pos].pwm_control);
	else
		result = cyhal_pwm_stop(&led[led_pos].pwm_control);
	if(result != CY_RSLT_SUCCESS)
				halt_with_error("\tLED -> turn_led_on_off() ->"
								"\n\r\t\t\t->cyhal_pwm_start()");
}



/******************************************************************************
 * Function Name: blinking_led
 ***************************************
 * Summary:
 *	Turns off all LEDs except set by user (blinking_led).
 *		This led will be blink with set parameters.
 *
 * Parameters:
 *	blinking_led	-	led which will be blink.
 *	cycle_num		-	number of blink cycle.
 *	time_ms			-	time of one half of full cycle.
 *
 *****************************************************************************/
void blinking_led(uint8_t blinking_led, uint8_t cycle_num, uint16_t time_ms){
	 change_led_duty_cycle(blinking_led, 0);
	 for (uint8_t i=0; i<NUM_LEDS; i++)
		 turn_led_on_off(blinking_led, 0);

	 cycle_num = cycle_num*2 + 1;
	 while (cycle_num--){
		 turn_led_on_off(blinking_led, cycle_num % 2);
		 Cy_SysLib_Delay(time_ms);
	 }
 }

 uint8_t uart_enable(){
	 return 1 == led[BUTTON4].status;
 }
