/*
 * audio.c
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: yabe
 */

#include "audio.h"


/*******************************************************************************
* Function Name: initialize_audio
***************************************
* Summary:
* 	Initialize all necessary  components for audio:
*   	1. Master Clock with a PWM.
*   	2. I2C Master.
*   	3. I2S block.
*   	4. PDM/PCM block.
*   	5. AK494A codec.
*   	6. DMAs and their descriptor addresses.
*
* Parameters:
*	*recorded_data	- array into which the sound will be recorded.
*
* Return:
*	cy_rslt_t - error status. Returns CY_RSLT_SUCCESS if succeeded.
*
*******************************************************************************/
void initialize_audio(int16_t *recorded_data){
	cy_rslt_t result;

	const cyhal_i2c_cfg_t mi2c_cfg = {
		.is_slave        = false,
		.address         = 0,
		.frequencyhal_hz = 400000
	};

	/* Master Clock PWM */
	cyhal_pwm_t mclk_pwm;


	/* Initialize the Master Clock with a PWM */
	cyhal_pwm_init(&mclk_pwm, (cyhal_gpio_t) P5_0, NULL);
	result = cyhal_pwm_set_duty_cycle(&mclk_pwm, MCLK_DUTY_CYCLE, MCLK_FREQ);
	if(result != CY_RSLT_SUCCESS)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> cyhal_pwm_set_duty_cycle()");

	result = cyhal_pwm_start(&mclk_pwm);
	if(result != CY_RSLT_SUCCESS)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> cyhal_pwm_start()");


	/* Wait for a bit the MCLK to clock the audio codec */
	Cy_SysLib_Delay(1);


	/* Initialize the I2C Master */
	result = cyhal_i2c_init(&mi2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	if(result != CY_RSLT_SUCCESS)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> cyhal_i2c_init()");

	result = cyhal_i2c_configure(&mi2c, &mi2c_cfg);
	if(result != CY_RSLT_SUCCESS)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> cyhal_i2c_configure()");


	/* Initialize the I2S block */
	cy_en_i2s_status_t result_I2S;
	result_I2S = Cy_I2S_Init(CYBSP_I2S_HW, &CYBSP_I2S_config);
	if (CY_I2S_SUCCESS != result_I2S)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> Cy_I2S_Init()");

	Cy_I2S_ClearTxFifo(CYBSP_I2S_HW);
	/* Put at least one frame into the Tx FIFO */
	Cy_I2S_WriteTxData(CYBSP_I2S_HW, 0UL);
	Cy_I2S_WriteTxData(CYBSP_I2S_HW, 0UL);


	/* Initialize PDM/PCM block */
	cy_en_pdm_pcm_status_t result_PDM;
	result_PDM = Cy_PDM_PCM_Init(CYBSP_PDM_HW, &CYBSP_PDM_config);
	if (CY_PDM_PCM_SUCCESS != result_PDM)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> Cy_PDM_PCM_Init()");


	/* Configure the AK494A codec and enable it */
	uint32_t result_ak4954a;
	result_ak4954a = ak4954a_init(mi2c_transmit);
	if (0 != result_ak4954a)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> init audio codec");

	result_ak4954a = ak4954a_activate();
	if (0 != result_ak4954a)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> activate audio codec");


	/* Initialize the DMAs and their descriptor addresses */
	cy_en_dma_status_t result_DMA;
	result_DMA = Cy_DMA_Descriptor_Init(&CYBSP_DMA_PDM_Descriptor_0, &CYBSP_DMA_PDM_Descriptor_0_config);
	if (CY_DMA_SUCCESS != result_DMA)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> Cy_DMA_Descriptor_Init()");
	Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_PDM_Descriptor_0, BUFFER_SIZE*NUM_CHANNELS/DMA_LOOP_SIZE);
	Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_PDM_Descriptor_0, (void *) &CYBSP_PDM_HW->RX_FIFO_RD);
	Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_PDM_Descriptor_0, (void *) recorded_data);
	Cy_DMA_Channel_Init(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL, &CYBSP_DMA_PDM_channelConfig);
	Cy_DMA_Enable(CYBSP_DMA_PDM_HW);

	result_PDM = Cy_DMA_Descriptor_Init(&CYBSP_DMA_I2S_Descriptor_0, &CYBSP_DMA_I2S_Descriptor_0_config);
	if (CY_DMA_SUCCESS != result_DMA)
		halt_with_error("\tAudio.c -> initialize_audio() ->"
						"\n\r\t\t\t-> Cy_DMA_Descriptor_Init()");
	Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_I2S_Descriptor_0, BUFFER_SIZE*NUM_CHANNELS/DMA_LOOP_SIZE);
	Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_DMA_I2S_Descriptor_0, (void *) recorded_data);
	Cy_DMA_Descriptor_SetDstAddress(&CYBSP_DMA_I2S_Descriptor_0, (void *) &CYBSP_I2S_HW->TX_FIFO_WR);
	Cy_DMA_Channel_Init(CYBSP_DMA_I2S_HW, CYBSP_DMA_I2S_CHANNEL, &CYBSP_DMA_I2S_channelConfig);
	Cy_DMA_Enable(CYBSP_DMA_I2S_HW);

	/* Enable Audio Subsystem */
	Cy_I2S_EnableTx(CYBSP_I2S_HW);
	Cy_PDM_PCM_Enable(CYBSP_PDM_HW);

	// BUTTON2 is inverse (see led initialization).
	change_led_status(BUTTON2);
	// To avoid noise in first record.
	record_audio(BUTTON2);

	/* Reset the channel index for the next recording */
	CYBSP_DMA_PDM_HW->CH_STRUCT[CYBSP_DMA_PDM_CHANNEL].CH_IDX = 0;
}


/*******************************************************************************
* Function Name: mi2c_transmit
********************************************************************************
* Summary:
*  I2C Master function to transmit data to the given address.
*
* Parameters:
*  reg_addr: address to be updated
*  data: 8-bit data to be written in the register
*
* Return:
*  cy_rslt_t - I2C master transaction error status.
*              Returns CY_RSLT_SUCCESS if succeeded.
*
*******************************************************************************/
void  mi2c_transmit(uint8_t reg_addr, uint8_t data)
{
    cy_rslt_t result;
    uint8_t buffer[AK4954A_PACKET_SIZE];

    buffer[0] = reg_addr;
    buffer[1] = data;

    /* Send the data over the I2C */
    result = cyhal_i2c_master_write(&mi2c, AK4954A_I2C_ADDR, buffer, 2, MI2C_TIMEOUT, true);

    if (CY_DMA_SUCCESS != result)
    		halt_with_error("\tAudio.c -> mi2c_transmit() ->"
    						"\n\r\t\t\t-> cyhal_i2c_master_write()");
}


/*******************************************************************************
* Function Name: record_audio
***************************************
* Summary:
* 	Enable DMA_Channel i.e. start audio recording
*
* Parameters:
*	active_button	-	necessary to activate status "brightness_active"
*						in led that is active. It can be led[0] or led[1],
*						i.e. Red or Green led.
*
* Return:
*
*******************************************************************************/
void record_audio(uint8_t active_button){
	change_led_status(active_button);

	/* Enable DMA to record from the microphone */
	Cy_DMA_Channel_Enable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);

	while (CYBSP_DMA_PDM_HW->CH_STRUCT[CYBSP_DMA_PDM_CHANNEL].CH_IDX != BUFFER_SIZE){}

	// Stop record
	change_led_status(active_button);
	Cy_DMA_Channel_Disable(CYBSP_DMA_PDM_HW, CYBSP_DMA_PDM_CHANNEL);
}


/*******************************************************************************
* Function Name: play_record
***************************************
* Summary:
*	Enable the I2S DMA channel.
*
* Parameters:
*
* Return:
*
*******************************************************************************/
void play_record(){
	/* Reset the channel index for the next recording */
	CYBSP_DMA_PDM_HW->CH_STRUCT[CYBSP_DMA_PDM_CHANNEL].CH_IDX = 0;

	/* Set up the DMAs to play the recorded data */
	Cy_DMA_Descriptor_SetYloopDataCount(&CYBSP_DMA_I2S_Descriptor_0, BUFFER_SIZE/DMA_LOOP_SIZE);

	/* Start playing the recorded data by enabling the I2S DMA */
	Cy_DMA_Channel_Enable(CYBSP_DMA_I2S_HW, CYBSP_DMA_I2S_CHANNEL);
}
