/*
 * fft.c
 *
 *  Created on: 16 квіт. 2020 р.
 *      Author: yabe
 */


#include "fft.h"

/*******************************************************************************
* Function Name: get_number_of_bits_to_upscale
***************************************
* Summary:
*	FFT work only with array that has next size:
*		32, 64, 128, 256, 512, 1024, 2048, 4096, 8192.
*
*	If we use FFT_q15 than each value in spectrogram must be upscale in some
*		number of bits. Detail: http://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html#ga00e615f5db21736ad5b27fb6146f3fc5
*
*	This function check input frame size.
*		If the size is incorrect than will be call "halt_with_error(...)".
*		Else, it return number of bits to upscale.
*
* Parameters:
*	frame_size	- 	number of sound bits for FFT.
*
* Return:
*	int8_t 		-	number of bits to upscale.
*
*******************************************************************************/
int8_t get_number_of_bits_to_upscale(uint16_t frame_size){
	uint16_t available_frames_size[9] = {32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};
	for (int i=0; i<9; i++){
		if (available_frames_size[i] == frame_size)
			return i + 4;
	}
	char massage[60];
	sprintf(massage, 	"fft_q31\n"
						"Bad number of bits.\n"
						"It can not be equal to %d",
						frame_size);
	halt_with_error(massage);

	// Never gets here.
	return 0;
}

/*******************************************************************************
* Function Name: fft_q15
***************************************
* Summary:
*	Made FFT transform with set frame_size.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*magnitude		- 	array into which the spectrogram will be recorded;
*	frame_num		-	number FFT frames
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q15(const int16_t *input_data, int16_t *magnitude, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q15 FFT problem (overflow). Not necessary.
	q15_t buffer_out_complex[frame_size * 2];
	q15_t magnitude_real[sample_lenght];
	q15_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q15 Instance_q15_fft;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q15(&Instance_q15_fft, frame_size, 0, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Copy TI's data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++)
			buffer_in_real[i][j] = (q15_t)input_data[j + frame_size*i];
	}

	for (i=0; i<frame_num; i++){
			//Compute real FFT
			arm_rfft_q15(&Instance_q15_fft, buffer_in_real[i], buffer_out_complex);

			// Upscale value in some number of bits
			for (j= 0; j < frame_size * 2; j++)
				buffer_out_complex[j] <<= number_of_bits_to_upscale;

			// Compute magnitude.
			//		FFT output has real and imaginary parts, that is way
			//		magnitude is equal sqrt(r^2 + i^2). This is optimized
			//		function to compute magnitude.
			arm_cmplx_mag_q15(buffer_out_complex, magnitude_real, frame_size);

			// Record compute magnitude like 2-dimension array
			for (j = 0; j < frame_size; j++)
				magnitude[j + frame_size*i] = (int16_t)magnitude_real[j];

//		// in one cycle (not correct)
//		q15_t real, imag;                              /* Temporary input variables */
//		q31_t acc0, acc1;                              /* Accumulators */
//		for (j=0; j<frame_size*2; j+=2){
//			buffer_out_complex[j] <<= number_of_bits_to_upscale;
//			buffer_out_complex[j+1] <<= number_of_bits_to_upscale;
//
//			real = buffer_out_complex[j];
//			imag = buffer_out_complex[j+1];
//			acc0 = ((q31_t) real * real);
//			acc1 = ((q31_t) imag * imag);
//
//			/* store result in 2.14 format in destination buffer. */
//			arm_sqrt_q15((q15_t) (((q63_t) acc0 + acc1) >> 17), &magnitude[j>>1]);
//			// To return magnitude to q15_t format.
//			magnitude[j>>1 + frame_size*i] <<= 1;
//		}
	}
}


/*******************************************************************************
* Function Name: fft_q15_sound
***************************************
* Summary:
* 	This function shows how FFT and IFFT work and gives some estimates:
* 		* What is the quality of the output signal;
* 		* Which noises are added;
* 		* How much time spends to made FFT and IFFT;
*
* 	IFFT has problem with backward transform, first bit in each frame gets noise.
*		To avoid this, set the first bit to the average of the previous and next bits.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*data_output	- 	array into which the IFFT sound will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q15_sound(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q15 FFT problem (overflow). Not necessary.
	q15_t buffer_out_complex[frame_size * 2];
	q15_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q15 Instance_q15_fft;
	arm_rfft_instance_q15 Instance_q15_ifft;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q15(&Instance_q15_fft, frame_size, 0, 1);
	status = arm_rfft_init_q15(&Instance_q15_ifft, frame_size, 1, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Copy input data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++)
			buffer_in_real[i][j] = (q15_t)input_data[j + frame_size*i];
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT
		arm_rfft_q15(&Instance_q15_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		//Compute inverse real FFT (IFFT) using the data from FFT
		arm_rfft_q15(&Instance_q15_ifft, buffer_out_complex, buffer_in_real[i]);

		for (j = 1; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j]<<1;

		// IFFT has problem with backward transform, first bit in each frame get noise.
		//		To avoid this, set the first bit to the average of the previous and next bits.
		if (i == 0)
			data_output[frame_size*i] = (int16_t)(0.5*(int32_t)data_output[frame_size*i+1]);
		else
			data_output[frame_size*i] = (int16_t)(((int32_t)data_output[frame_size*i - 1] + (int32_t)data_output[frame_size*i + 1])/2);
	}
}


/*******************************************************************************
* Function Name: fft_q15_hamming_sound
***************************************
* Summary:
* 	This function shows how FFT and IFFT work with pre-processing using the
* 		Hamming window and gives some estimates:
* 			* What is the quality of the output signal;
* 			* Which noises are added;
* 			* How much time spends to made FFT and IFFT;
*
* 	IFFT has problem with backward transform, first bit in each frame gets noise.
*		To avoid this, set the first bit to the average of the previous and next bits.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*data_output	- 	array into which the IFFT sound will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q15_hamming_sound(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;
	float32_t temp;

	arm_status status;
	arm_rfft_instance_q15 Instance_q15_fft;
	arm_rfft_instance_q15 Instance_q15_ifft;

	// q31 variables to fix q15 FFT problem (overflow). Not necessary.
	q15_t buffer_out_complex[frame_size * 2];
	q15_t buffer_in_real[frame_num][frame_size];

	float32_t hamming[frame_size];

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q15(&Instance_q15_fft, frame_size, 0, 1);
	status = arm_rfft_init_q15(&Instance_q15_ifft, frame_size, 1, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Compute coefficient of Hamming window
	for (j=0; j<frame_size; j++)
		hamming[j] = 0.54 - 0.46*sinf((2*PI*j)/(frame_size-1));

	// Copy TI's data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++){
			temp = input_data[j + frame_size*i] * hamming[j];
			buffer_in_real[i][j] = (q15_t)(temp + (q31_t)input_data[j + frame_size*i]);
		}
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT using the completed data buffer
		arm_rfft_q15(&Instance_q15_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		//Compute inverse real FFT using the data from fft
		arm_rfft_q15(&Instance_q15_ifft, buffer_out_complex, buffer_in_real[i]);


		for (j = 0; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j];

		// IFFT has problem with backward transform, first bit in each frame get noise.
		//		To avoid this, set the first bit to the average of the previous and next bits.
		if (i == 0)
			data_output[frame_size*i] = (int16_t)(0.5*(int32_t)data_output[frame_size*i+1]);
		else
			data_output[frame_size*i] = (int16_t)(((int32_t)data_output[frame_size*i - 1] + (int32_t)data_output[frame_size*i + 1])/2);
	}
}



/*******************************************************************************
* Function Name: fft_q31
***************************************
* Summary:
*	Made FFT transform with set frame_size.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*magnitude		- 	array into which the spectrogram will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q31(const int16_t *input_data, int16_t *magnitude, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q31 FFT problem (overflow). Not necessary.
	q31_t buffer_out_complex[frame_size * 2];
	q31_t magnitude_real[sample_lenght];
	q31_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q31 Instance_q31_fft;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q31(&Instance_q31_fft, frame_size, 0, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Copy TI's data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++)
			buffer_in_real[i][j] = (q31_t)input_data[j + frame_size*i];
	}

	for (i=0; i<frame_num; i++){
			//Compute real FFT
			arm_rfft_q31(&Instance_q31_fft, buffer_in_real[i], buffer_out_complex);

			// Upscale value in some number of bits
			for (j= 0; j < frame_size * 2; j++)
				buffer_out_complex[j] <<= number_of_bits_to_upscale;

			// Compute magnitude.
			//		FFT output has real and imaginary parts, that is way
			//		magnitude is equal sqrt(r^2 + i^2). This is optimized
			//		function to compute magnitude.
			arm_cmplx_mag_q31(buffer_out_complex, magnitude_real, frame_size);

			// Record compute magnitude like 2-dimension array
			for (j = 0; j < frame_size; j++)
				magnitude[j + frame_size*i] = (int16_t)magnitude_real[j];

//		// in one cycle (not correct)
//		q31_t real, imag;                              /* Temporary input variables */
//		q31_t acc0, acc1;                              /* Accumulators */
//		for (j=0; j<frame_size*2; j+=2){
//			buffer_out_complex[j] <<= number_of_bits_to_upscale;
//			buffer_out_complex[j+1] <<= number_of_bits_to_upscale;
//
//			real = buffer_out_complex[j];
//			imag = buffer_out_complex[j+1];
//			acc0 = ((q31_t) real * real);
//			acc1 = ((q31_t) imag * imag);
//
//			/* store result in 2.14 format in destination buffer. */
//			arm_sqrt_q31((q31_t) (((q63_t) acc0 + acc1) >> 17), &magnitude[j>>1]);
//			// To return magnitude to q31_t format.
//			magnitude[j>>1 + frame_size*i] <<= 1;
//		}
	}
}


/*******************************************************************************
* Function Name: fft_q31_sound
***************************************
* Summary:
* 	This function shows how FFT and IFFT work and gives some estimates:
* 		* What is the quality of the output signal;
* 		* Which noises are added;
* 		* How much time spends to made FFT and IFFT;
*
* 	IFFT has problem with backward transform, first bit in each frame gets noise.
*		To avoid this, set the first bit to the average of the previous and next bits.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*data_output	- 	array into which the IFFT sound will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q31_sound(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q31 FFT problem (overflow). Not necessary.
	q31_t buffer_out_complex[frame_size * 2];
	q31_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q31 Instance_q31_fft;
	arm_rfft_instance_q31 Instance_q31_ifft;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q31(&Instance_q31_fft, frame_size, 0, 1);
	status = arm_rfft_init_q31(&Instance_q31_ifft, frame_size, 1, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Copy input data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++)
			buffer_in_real[i][j] = (q31_t)input_data[j + frame_size*i];
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT
		arm_rfft_q31(&Instance_q31_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		//Compute inverse real FFT (IFFT) using the data from FFT
		arm_rfft_q31(&Instance_q31_ifft, buffer_out_complex, buffer_in_real[i]);

		for (j = 1; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j]<<1;

		// IFFT has problem with backward transform, first bit in each frame get noise.
		//		To avoid this, set the first bit to the average of the previous and next bits.
		if (i == 0)
			data_output[frame_size*i] = (int16_t)(0.5*(int32_t)data_output[frame_size*i+1]);
		else
			data_output[frame_size*i] = (int16_t)(((int32_t)data_output[frame_size*i - 1] + (int32_t)data_output[frame_size*i + 1])/2);
	}
}


/*******************************************************************************
* Function Name: fft_q31_hamming_sound
***************************************
* Summary:
* 	This function shows how FFT and IFFT work with pre-processing using the
* 		Hamming window and gives some estimates:
* 			* What is the quality of the output signal;
* 			* Which noises are added;
* 			* How much time spends to made FFT and IFFT;
*
* 	IFFT has problem with backward transform, first bit in each frame gets noise.
*		To avoid this, set the first bit to the average of the previous and next bits.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*data_output	- 	array into which the IFFT sound will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q31_hamming_sound(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;
	float32_t temp;

	arm_status status;
	arm_rfft_instance_q31 Instance_q31_fft;
	arm_rfft_instance_q31 Instance_q31_ifft;

	// q31 variables to fix q31 FFT problem (overflow). Not necessary.
	q31_t buffer_out_complex[frame_size * 2];
	q31_t buffer_in_real[frame_num][frame_size];

	float32_t hamming[frame_size];

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q31(&Instance_q31_fft, frame_size, 0, 1);
	status = arm_rfft_init_q31(&Instance_q31_ifft, frame_size, 1, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Compute coefficient of Hamming window
	for (j=0; j<frame_size; j++)
		hamming[j] = 0.54 - 0.46*sinf((2*PI*j)/(frame_size-1));

	// Copy TI's data arrays to the new FFT working buffers
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++){
			temp = input_data[j + frame_size*i] * hamming[j];
			buffer_in_real[i][j] = temp + input_data[j + frame_size*i];
		}
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT using the completed data buffer
		arm_rfft_q31(&Instance_q31_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		//Compute inverse real FFT using the data from fft
		arm_rfft_q31(&Instance_q31_ifft, buffer_out_complex, buffer_in_real[i]);


		for (j = 0; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j];

		// IFFT has problem with backward transform, first bit in each frame get noise.
		//		To avoid this, set the first bit to the average of the previous and next bits.
		if (i == 0)
			data_output[frame_size*i] = (int16_t)(0.5*(int32_t)data_output[frame_size*i+1]);
		else
			data_output[frame_size*i] = (int16_t)(((int32_t)data_output[frame_size*i - 1] + (int32_t)data_output[frame_size*i + 1])/2);
	}
}


/*******************************************************************************
* Function Name: fft_q15_test_1khz_sound
***************************************
* Summary:
* 	This function shows how FFT and IFFT work with one-wave signal
* 		and gives some estimates:
* 			* What is the quality of the output signal;
*	 		* Which noises are added;
* 			* How much time spends to made FFT and IFFT;
*
* 	IFFT has problem with backward transform, first bit in each frame gets noise.
*		To avoid this, set the first bit to the average of the previous and next bits.
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*data_output	- 	array into which the IFFT sound will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q15_test_1khz_sound(int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q15 FFT problem (overflow). Not necessary.
	q15_t buffer_out_complex[frame_size * 2];
	q15_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q15 Instance_q15_fft;
	arm_rfft_instance_q15 Instance_q15_ifft;

	uint16_t freq = 1000;
	uint16_t freq_d = 16384;
	uint32_t t = 0;
	float32_t temp;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q15(&Instance_q15_fft, frame_size, 0, 1);
	status = arm_rfft_init_q15(&Instance_q15_ifft, frame_size, 1, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Generate a 1000 Hz sine wave
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++){
			t = j + frame_size*i;
			temp = 1000.0 * sinf(6.283185308 * t * (float)freq / (float)freq_d);
			buffer_in_real[i][j] = (q15_t)temp;
			input_data[t] = (uint16_t)buffer_in_real[i][j];
		}
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT using the completed data buffer
		arm_rfft_q15(&Instance_q15_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		//Compute inverse real FFT using the data from fft
		arm_rfft_q15(&Instance_q15_ifft, buffer_out_complex, buffer_in_real[i]);

		for (j = 0; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j];

		// IFFT has problem with backward transform, first bit in each frame get noise.
		//		To avoid this, set the first bit to the average of the previous and next bits.
		if (i == 0)
			data_output[frame_size*i] = (int16_t)(0.5*(int32_t)data_output[frame_size*i+1]);
		else
			data_output[frame_size*i] = (int16_t)(((int32_t)data_output[frame_size*i - 1] + (int32_t)data_output[frame_size*i + 1])/2);
	}
}


/*******************************************************************************
* Function Name: fft_q15_test_1khz_fft
***************************************
* Summary:
*	This function shows how FFT work with one-wave signal
*
* Parameters:
* 	*input_data		-	array with input sound bits;
*	*magnitude		- 	array into which the spectrogram will be recorded;
*	frame_num		-	number FFT frames;
*	frame_size		-	number of sound bits for FFT.
*
*******************************************************************************/
void fft_q15_test_1khz_fft(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
	uint32_t i, j;
	uint16_t sample_lenght = frame_num * frame_size;
	int8_t number_of_bits_to_upscale;

	// q31 variables to fix q15 FFT problem (overflow). Not necessary.
	q15_t buffer_out_complex[frame_size * 2];
	q15_t magnitude_real[sample_lenght];
	q15_t buffer_in_real[frame_num][frame_size];

	arm_status status;
	arm_rfft_instance_q15 Instance_q15_fft;

	uint16_t freq = 1000;
	uint16_t freq_d = 16384;
	uint32_t t = 0;
	float32_t temp;

	status = ARM_MATH_SUCCESS;
	status = arm_rfft_init_q15(&Instance_q15_fft, frame_size, 0, 1);

	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);

	// Generate a 1000 Hz sine wave
	for (i = 0; i < frame_num; i++){
		for (j=0; j<frame_size; j++){
			t = j + frame_size*i;
			temp = 1000.0 * sinf(6.283185308 * t * (float)freq / (float)freq_d);
			buffer_in_real[i][j] = (q15_t)temp;
		}
	}

	for (i=0; i<frame_num; i++){
		//Compute real FFT using the completed data buffer
		arm_rfft_q15(&Instance_q15_fft, buffer_in_real[i], buffer_out_complex);

		// Upscale value in some number of bits
		for (j= 0; j < frame_size * 2; j++)
			buffer_out_complex[j] <<= number_of_bits_to_upscale;

		// Compute magnitude.
		//		FFT output has real and imaginary parts, that is way
		//		magnitude is equal sqrt(r^2 + i^2). This is optimized
		//		function to compute magnitude.
		arm_cmplx_mag_q15(buffer_out_complex, magnitude_real, frame_size);

		// Record compute magnitude like 2-dimension array
		for (j = 0; j < frame_size; j++)
			data_output[j + frame_size*i] = (int16_t)magnitude_real[j];
	}
}

//void fft_float(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
//	uint32_t i, j;
//	uint16_t sample_lenght = frame_num * frame_size;
//	int8_t number_of_bits_to_upscale;
//	arm_status status;
//	arm_rfft_instance_q15 Instance_q15;
//
//	frame_num = sample_lenght/frame_size;
//
//	// q15 variables to fix q15 FFT problem
////	float32_t magnitude[frame_size];
//	float32_t buffer_in_complex[frame_num][frame_size*2];
//	float32_t buffer_out_complex[frame_size*2];
//
//
//	status = ARM_MATH_SUCCESS;
//	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);
//
//	// Copy TI's data arrays to the new FFT working buffers
//	for (i = 0; i < frame_num; i++){
//		for (j=0; j<frame_size; j++)
//			buffer_in_complex[i][j*2] = ((float32_t)input_data[j + frame_size*i])/32768;
//	}
//
//
//	for (i=0; i<frame_num; i++){
//		//Compute real FFT using the completed data buffer
//		arm_rfft_fast_instance_f32 Instance_q15;
//		status = arm_rfft_fast_init_f32(&Instance_q15, frame_size);
//		arm_rfft_fast_f32(&Instance_q15, buffer_in_complex[i], buffer_out_complex, 0);
//
////		for (j= 0; j < frame_size * 2; j++)
////			buffer_out_complex[j] <<= number_of_bits_to_upscale;
//
//		//Compute inverse real FFT using the data from fft
//		arm_rfft_fast_instance_f32 Instance_q15;
//		status = arm_rfft_fast_init_f32(&Instance_q15, frame_size);
//		arm_rfft_fast_f32(&Instance_q15, buffer_out_complex, buffer_in_complex[i], 1);
//
//
////		arm_cmplx_mag_f32(buffer_in_complex[i], buffer_out_complex, frame_size);
//
//		for (j = 0; j < frame_size; j++)
//			data_output[j + frame_size*i] = (int16_t)buffer_in_complex[i][j*2+1];
//	}
//
//}


//void fft_test(const int16_t *input_data, int16_t *data_output, uint16_t frame_num, uint16_t frame_size){
//	uint32_t i, j;
//	uint16_t sample_lenght = frame_num * frame_size;
//	int8_t number_of_bits_to_upscale;
//	arm_status status;
//	arm_rfft_instance_q15 Instance_q15;
//
//	frame_num = sample_lenght/frame_size;
//
//	// q15 variables to fix q15 FFT problem
//	q15_t buffer_out_complex[frame_size * 2];
//	q15_t buffer_in_real[frame_num][frame_size*2];
//
//
//	status = ARM_MATH_SUCCESS;
//	number_of_bits_to_upscale = get_number_of_bits_to_upscale(frame_size);
//
//	// Copy TI's data arrays to the new FFT working buffers
//	for (i = 0; i < frame_num; i++){
//		for (j=0; j<frame_size; j++)
//			buffer_in_real[i][j*2] = (q15_t)input_data[j + frame_size*i];
//	}
//
//
//
//
//	for (i=0; i<frame_num; i++){
//		//Compute real FFT using the completed data buffer
//		const arm_cfft_instance_q15 *Instance_q15SS;
//		Instance_q15 = &arm_cfft_sR_q15_len128;
//		status = arm_cfft_init_q15(&Instance_q15, frame_size);
//		arm_cfft_q15(&Instance_q15, buffer_in_real[i], 0, 1);
//
////		for (j= 0; j < frame_size * 2; j++)
////			buffer_in_real[i][j] <<= number_of_bits_to_upscale;
//
//		//Compute inverse real FFT using the data from fft
//		const arm_cfft_instance_q15 *Instance_q15;
//		Instance_q15 = &arm_cfft_sR_q15_len128;
//		status = arm_cfft_init_q15(&Instance_q15, frame_size);
//		arm_cfft_q15(&Instance_q15, buffer_in_real[i], 1, 1);
//
//
//
//		arm_cmplx_mag_q15(buffer_in_real[i], buffer_in_real[i], (uint32_t)frame_size);
//		for (j = 0; j < frame_size; j++){
//			data_output[j + frame_size*i] = (int16_t)buffer_in_real[i][j];
//		}
//	}
//
//}

















