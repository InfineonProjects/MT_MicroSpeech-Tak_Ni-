/*
 * capsense.h
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#ifndef FUNCTIONAL_HEADERS_CAPSENSE_H_
	#define FUNCTIONAL_HEADERS_CAPSENSE_H_

	#include "cy_pdl.h"
	#include "cyhal.h"
	#include "cycfg_capsense.h"

	#include "led.h"
	#include "error.h"

	#define CAPSENSE_INTR_PRIORITY  (7u)


	void initialize_capsense();
	void capsense_isr();
	void touch_control(uint8_t *active_button);
	uint8_t detect_touch();



#endif /* FUNCTIONAL_HEADERS_CAPSENSE_H_ */
