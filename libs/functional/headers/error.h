/*
 * error.h
 *
 *  Created on: 20 квіт. 2020 р.
 *      Author: yabe
 */

#ifndef LIBS_FUNCTIONAL_HEADERS_ERROR_H_
	#define LIBS_FUNCTIONAL_HEADERS_ERROR_H_

	#include "cybsp.h"

	#include "led.h"

	void reboot();
	void reboot_with_error(char massage[]);
	void halt();
	void halt_with_error(char massage[]);


#endif /* LIBS_FUNCTIONAL_HEADERS_ERROR_H_ */
