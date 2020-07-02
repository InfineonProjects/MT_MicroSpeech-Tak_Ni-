/*
 * audio.h
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#ifndef LIBS_FUNCTIONAL_HEADERS_AUDIO_H_
	#define LIBS_FUNCTIONAL_HEADERS_AUDIO_H_

	#include "cy_pdl.h"
	#include "cyhal.h"
	#include "cybsp.h"

	#include "ak4954a.h"
	#include "led.h"

	/* Size of the recorded buffer */
	#define BUFFER_SIZE     32000u // 8 msec = 128 // 32768

	/* Number of channels (Stereo) */
	#define NUM_CHANNELS    2u

	/* Master I2C Timeout */
	#define MI2C_TIMEOUT    10u         /* in ms */

	/* Master Clock (MCLK) Frequency for the audio codec */
	#define MCLK_FREQ       2042000   /* in Hz */

	/* Duty cycle for the MCLK PWM */
	#define MCLK_DUTY_CYCLE 50.0f       /* in %  */

	/* DMA Maximum loop transfer size */
	#define DMA_LOOP_SIZE   256u


	/* Master I2C variables */
	cyhal_i2c_t mi2c;

	void initialize_audio(int16_t *recorded_data);
	void mi2c_transmit(uint8_t reg_adrr, uint8_t data);
	void record_audio(uint8_t active_button);
	void play_record();

#endif /* LIBS_FUNCTIONAL_HEADERS_AUDIO_H_ */
