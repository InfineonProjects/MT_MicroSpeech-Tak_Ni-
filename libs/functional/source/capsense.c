/*
 * capsense.c
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#include "capsense.h"


/******************************************************************************
* Function Name: initialize_capsense
***************************************
* Summary:
*  This function initializes the CapSense and configure
*  	the CapSense interrupt.
*
******************************************************************************/
void initialize_capsense()
{
    cy_status status;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if(CYRET_SUCCESS != status)
    	halt_with_error("\tCapsense.c -> initialize_capsense() ->"
						"\n\r\n\r\t\t\t-> Cy_CapSense_Init()");

	/* Initialize CapSense interrupt */
	Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
	NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
	NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

	/* Initialize the CapSense firmware modules. */
	status = Cy_CapSense_Enable(&cy_capsense_context);

	if(CYRET_SUCCESS != status)
	    	halt_with_error("\tCapsense.c -> initialize_capsense() ->"
							"\n\r\t\t\t-> Cy_CapSense_Enable()");
}


/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
void capsense_isr()
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}


/*******************************************************************************
* Function Name: touch_control
********************************************************************************
* Summary:
*  Change LEDs and Buttons state according to the user`s actions.
*  		1. Fixes the pressed Button.
*  		2. Wait for long press.
*  		3. Change brightness of led[*active_button] according
*  			to the User`s actions.
*
*  This example has 4 CSD buttons:
*  		0. BTN0 (CSD) 										->	Button0	->	led[0]	->	Red		(RGB)
*  		1. BTN1 (CSD) 										->	Button1	->	led[1]	->	Green	(RGB)
*  		2. Short touch on Button0 and Button1 				-> 	Button2	->	led[2]	->	Blue	(RGB)
*  		2. Long touch (1 sec) on Button0 and Button1 		-> 	Button3	->	led[3]	->	Red
  		3. Very long touch (3 sec) on Button0 and Button1	->	Button4	->	led[4]	->	Orange
*
* Parameters:
* 	*active_button	-	position of the LED which parameters will change.
*
*******************************************************************************/
void touch_control(uint8_t *active_button){
	// State of the Buttons and Slider at this moment
	uint32_t button0_state;
    uint32_t button1_state;
    uint32_t button2_state;	// button2 -> button0 + button1 together
    uint16_t slider_pos;

    uint8_t slider_touch_state;
    uint8_t brightness;
    cy_stc_capsense_touch_t* slider_touch_info;

    // State of the Buttons and Slider at previous moment
    static uint32_t button0_state_prev;
    static uint32_t button1_state_prev;
    static uint32_t button2_state_prev;
    static uint16_t slider_pos_prev;

	static uint32_t button_pressed_time = 0;
	static uint32_t button_pressed_time_for_UART = 0;


	/* Process all widgets */
	Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

    /* Get Button0 state */
    button0_state = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON0_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get Button1 state */
    button1_state = Cy_CapSense_IsSensorActive(
                                CY_CAPSENSE_BUTTON1_WDGT_ID,
                                CY_CAPSENSE_BUTTON0_SNS0_ID,
                                &cy_capsense_context);

    /* Get Button2 state */
    button2_state = 0;
    if (button0_state == button1_state)
    	button2_state = button0_state;

    /* Get Slider state */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
            			CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_state = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;


    /* Detect new touch on Button0 */
    if((0u != button0_state) &&
       (0u == button0_state_prev)){
    		button_pressed_time = 0;
    		*active_button = 0;
    		change_led_duty_cycle(BUTTON0, led[BUTTON0].brightness_passive);
    		change_led_duty_cycle(BUTTON1, 100);
    		change_led_duty_cycle(BUTTON2, 100);
    		change_led_duty_cycle(BUTTON3, 100);
    }

    /* Detect new touch on Button1 */
    if((0u != button1_state) &&
       (0u == button1_state_prev)){
    		button_pressed_time = 0;
    		*active_button = 1;
    		change_led_duty_cycle(BUTTON0, 100);
    		change_led_duty_cycle(BUTTON1, led[BUTTON1].brightness_passive);
    		change_led_duty_cycle(BUTTON2, 100);
    		change_led_duty_cycle(BUTTON3, 100);
    }

    /* Detect new touch on Button2 */
    if (0u != button2_state &&
		0u == button2_state_prev){
    		button_pressed_time = 0;
    		*active_button = 2;
    		change_led_duty_cycle(BUTTON0, 100);
    		change_led_duty_cycle(BUTTON1, 100);
			change_led_duty_cycle(BUTTON2, led[BUTTON2].brightness_passive);
    		change_led_duty_cycle(BUTTON3, 100);
	}

    /***************************************
     * Detect new long touch on some Button
     ***************************************
     * 	If detect long touch (1 sec) on Button0 or Button1 then led[3] is activate
     * 		and you can change "brightness_active" for led[0] or led[1].
     *
     * 	If detect long touch (1 sec) on Button2 then led[2] is activate too
     * 		and you can change "brightness_active" for led[2].
     *
     *	If detect long long touch (3 sec) on Button3 (the same combination as for Button2)
     *		then led[4] is activate. This mean that UART is inaccessible (led[4].state = 0).
     *		Of course you can change brightness for led[4], exactly after touch.
     *
     **************************************/
    if ((button0_state == button0_state_prev && 0u != button0_state) ||
		(button1_state == button1_state_prev && 0u != button1_state) ||
		(button2_state == button2_state_prev && 0u != button2_state)){
			button_pressed_time++;
			button_pressed_time_for_UART++;
    }

    if (button_pressed_time > 1000 &&
    	BUTTON4 != *active_button){
    	// If detect long touch on Button0 and Button1
		if (1 == button2_state){
			*active_button = 3;
			change_led_duty_cycle(BUTTON0, 100);
			change_led_duty_cycle(BUTTON1, 100);
			change_led_duty_cycle(BUTTON2, 100);
		}
		// Else detect long touch on Button0 or Button1
		else{
			button_pressed_time = 0;
			change_led_duty_cycle(BUTTON3, led[BUTTON3].brightness_active);
		}

		change_led_duty_cycle(*active_button, led[*active_button].brightness_active);
    }

    // If detect long touch on Button3
    if (button_pressed_time_for_UART > 3000 &&
		3 == *active_button){
			change_led_duty_cycle(BUTTON3, 100);
			if (0 == led[BUTTON4].status){
				led[4].status = 1;
				change_led_duty_cycle(BUTTON4, led[BUTTON4].brightness_passive);
				*active_button = 1;
				change_led_duty_cycle(BUTTON1, led[*active_button].brightness_passive);
			}
			else{
				change_led_duty_cycle(BUTTON4, led[BUTTON4].brightness_active);
				*active_button = 4;
				led[4].status = 0;

				// comment
				change_led_duty_cycle(BUTTON1, led[BUTTON1].brightness_passive);
			}
			button_pressed_time = 0;
			button_pressed_time_for_UART = 0;
    }

    // Detect the new touch on slider and change brightness for led[*active_button].
    if((0 != slider_touch_state) &&
       (slider_pos != slider_pos_prev)){
        	brightness = 10 + (slider_pos * 100) / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        	if (brightness > 100)
        		brightness = 100;
        	// If short touch then change passive brightness,
        	//	else, i.e. long touch, change active brightess.
        	if (button_pressed_time > 1000){
        		led[*active_button].brightness_active = brightness;
        		change_led_duty_cycle(*active_button, led[*active_button].brightness_active);
        	}
        	else{
        		led[*active_button].brightness_passive = brightness;
        		change_led_duty_cycle(*active_button, led[*active_button].brightness_passive);
        	}

    }

    /* Update the previous touch state */
    button0_state_prev = button0_state;
    button1_state_prev = button1_state;
    button2_state_prev = button2_state;
    slider_pos_prev = slider_pos;


	/* Establishes synchronized operation between the CapSense
	 * middleware and the CapSense Tuner tool.
	 */
	Cy_CapSense_RunTuner(&cy_capsense_context);

	/* Start next scan */
	Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
}


/*******************************************************************************
* Function Name: detect_touch
***************************************
* Summary:
*  	Check CSD Buttons and Slider on touch.
*		If detect touch return True (1), else False (0).
*		Generally, the function "Cy_CapSense_IsBusy()" is incorrect,
*			because return True if CapSense is active
*			(not CSD Buttons, but CapSense).
*
* Parameters:
*
* Return:
*	cy_rslt_t - error status. Returns CY_RSLT_SUCCESS if succeeded.
*
*******************************************************************************/
uint8_t detect_touch(){
	return CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context);
}
