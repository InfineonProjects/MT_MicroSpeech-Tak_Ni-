/*
 * led.h
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#ifndef FUNCTIONAL_HEADERS_LED_H_
	#define FUNCTIONAL_HEADERS_LED_H_

	#include "cyhal.h"
	#include "cybsp.h"

	#include "error.h"

	// Leds number
	#define NUM_LEDS (5u)

	#define FREQ (100.0f)

	/**************************************
	 * 	led[0] -> CYBSP_LED_RGB_RED
	 * 	led[1] -> CYBSP_LED_RGB_GREEN
	 * 	led[2] -> CYBSP_LED_RGB_BLUE
	 * 	led[3] -> CYBSP_LED9 -> RED
	 * 	led[4] -> CYBSP_LED8 -> ORANGE
	 *************************************/

	#define BUTTON0 0
	#define BUTTON1 1
	#define BUTTON2 2
	#define BUTTON3 3
	#define BUTTON4 4

	struct led_info{
		cyhal_pwm_t pwm_control;
		cyhal_gpio_t pin;
		uint8_t brightness_active;
		uint8_t brightness_passive;
		uint8_t status;
	};
	struct led_info led[NUM_LEDS];

	/*******************************************************************************
	 * Function Prototypes
	********************************************************************************/
	void initialize_led();
	void change_led_status(uint8_t led_pos);
	void change_led_duty_cycle(uint8_t led_pos, uint32_t new_range);
	void turn_led_on_off(uint8_t led_pos, uint8_t state);
	void blinking_led(uint8_t blinking_led, uint8_t cycle_num, uint16_t time);

	uint8_t uart_enable();



#endif /* FUNCTIONAL_HEADERS_LED_H_ */










