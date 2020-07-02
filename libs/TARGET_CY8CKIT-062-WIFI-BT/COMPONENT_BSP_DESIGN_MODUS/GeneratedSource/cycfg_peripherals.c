/*******************************************************************************
* File Name: cycfg_peripherals.c
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.1.0.1266
* psoc6pdl 1.5.2.3446
* personalities_2.0 2.0.0.0
* udd 1.2.0.136
*
********************************************************************************
* Copyright 2020 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_peripherals.h"

const cy_stc_i2s_config_t CYBSP_I2S_config = 
{
	.txEnabled = true,
	.rxEnabled = false,
	.txDmaTrigger = true,
	.rxDmaTrigger = false,
	.clkDiv = 12,
	.extClk = false,
	.txMasterMode = 1,
	.txAlignment = CY_I2S_I2S_MODE,
	.txWsPulseWidth = CY_I2S_WS_ONE_CHANNEL_LENGTH,
	.txWatchdogEnable = false,
	.txWatchdogValue = 4294967295,
	.txSdoLatchingTime = 0,
	.txSckoInversion = 0,
	.txSckiInversion = 0,
	.txChannels = 2,
	.txChannelLength = CY_I2S_LEN16,
	.txWordLength = CY_I2S_LEN16,
	.txOverheadValue = CY_I2S_OVHDATA_ONE,
	.txFifoTriggerLevel = 128,
	.rxMasterMode = 1,
	.rxAlignment = CY_I2S_I2S_MODE,
	.rxWsPulseWidth = CY_I2S_WS_ONE_CHANNEL_LENGTH,
	.rxWatchdogEnable = false,
	.rxWatchdogValue = 4294967295,
	.rxSdiLatchingTime = 0,
	.rxSckoInversion = 0,
	.rxSckiInversion = 0,
	.rxChannels = 2,
	.rxChannelLength = CY_I2S_LEN16,
	.rxWordLength = CY_I2S_LEN16,
	.rxSignExtension = 0,
	.rxFifoTriggerLevel = 0,
};
#if defined (CY_USING_HAL)
	const cyhal_resource_inst_t CYBSP_I2S_obj = 
	{
		.type = CYHAL_RSC_I2S,
		.block_num = 0U,
		.channel_num = 0U,
	};
#endif //defined (CY_USING_HAL)
const cy_stc_pdm_pcm_config_t CYBSP_PDM_config = 
{
	.clkDiv = CY_PDM_PCM_CLK_DIV_1_4,
	.mclkDiv = CY_PDM_PCM_CLK_DIV_1_4,
	.ckoDiv = 2U,
	.ckoDelay = 0U,
	.sincDecRate = 32U,
	.chanSelect = CY_PDM_PCM_OUT_STEREO,
	.chanSwapEnable = false,
	.highPassFilterGain = 8U,
	.highPassDisable = false,
	.softMuteCycles = CY_PDM_PCM_SOFT_MUTE_CYCLES_96,
	.softMuteFineGain = 1UL,
	.softMuteEnable = false,
	.wordLen = CY_PDM_PCM_WLEN_16_BIT,
	.signExtension = false,
	.gainLeft = CY_PDM_PCM_GAIN_10_5_DB,
	.gainRight = CY_PDM_PCM_GAIN_10_5_DB,
	.rxFifoTriggerLevel = 0U,
	.dmaTriggerEnable = true,
	.interruptMask = 0UL,
};
#if defined (CY_USING_HAL)
	const cyhal_resource_inst_t CYBSP_PDM_obj = 
	{
		.type = CYHAL_RSC_PDM,
		.block_num = 0U,
		.channel_num = 0U,
	};
#endif //defined (CY_USING_HAL)
cy_stc_csd_context_t cy_csd_0_context = 
{
	.lockKey = CY_CSD_NONE_KEY,
};


void init_cycfg_peripherals(void)
{
#if defined (CY_USING_HAL)
	cyhal_hwmgr_reserve(&CYBSP_I2S_obj);
#endif //defined (CY_USING_HAL)

#if defined (CY_USING_HAL)
	cyhal_hwmgr_reserve(&CYBSP_PDM_obj);
#endif //defined (CY_USING_HAL)

	Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_8_BIT, 3U);
}
