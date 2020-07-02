/*
 * fft.h
 *
 *  Created on: 16 квіт. 2020 р.
 *      Author: yabe
 */

#ifndef FFT_H_
	#define FFT_H_

	#ifndef ARM_MATH_CM4
		#define ARM_MATH_CM4
	#endif // ARM_MATH_CM4

	//#define ARM_MATH_CM4
	#ifndef __FPU_PRESENT
		#define __FPU_PRESENT 1U
	#endif // __FPU_PRESENT

	#include "arm_math.h"
	#include "arm_const_structs.h"

	#include "error.h"


	/* ------------------------------------------------------------------
	* Global variables for FFT Bin Example
	* ------------------------------------------------------------------- */

//	uint16_t fftSize_ = SAMPLE_LENGTH;
//	uint8_t ifftFlag_ = 0;
//	uint8_t doBitReverse_ = 1;
	//arm_status status;

	int8_t number_of_bits_to_upscale(uint16_t frame_size);
	void fft_q15				(const int16_t *input_data, int16_t * magnitude, uint16_t frame_num, uint16_t frame_size);
	void fft_q15_sound			(const int16_t *input_data, int16_t *output_data, uint16_t frame_num, uint16_t frame_size);
	void fft_q15_hamming_sound	(const int16_t *input_data, int16_t * magnitude, uint16_t frame_num, uint16_t frame_size);
	void fft_q31				(const int16_t *input_data, int16_t * magnitude, uint16_t frame_num, uint16_t frame_size);
	void fft_q31_sound			(const int16_t *input_data, int16_t *output_data, uint16_t frame_num, uint16_t frame_size);
	void fft_q31_hamming_sound	(const int16_t *input_data, int16_t * magnitude, uint16_t frame_num, uint16_t frame_size);
	void fft_q15_test_1khz_fft	(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size);
	void fft_q15_test_1khz_sound(int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size);
//	void fft_float(const int16_t *input_data, int16_t * output_data, uint16_t frame_num, uint16_t frame_size);



#endif /* FFT_H_ */











