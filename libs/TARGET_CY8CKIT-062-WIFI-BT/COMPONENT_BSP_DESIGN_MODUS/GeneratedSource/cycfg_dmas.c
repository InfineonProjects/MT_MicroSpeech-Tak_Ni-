/*******************************************************************************
* File Name: cycfg_dmas.c
*
* Description:
* DMA configuration
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

#include "cycfg_dmas.h"

const cy_stc_dma_descriptor_config_t CYBSP_DMA_PDM_Descriptor_0_config = 
{
	.retrigger = CY_DMA_RETRIG_16CYC,
	.interruptType = CY_DMA_DESCR_CHAIN,
	.triggerOutType = CY_DMA_DESCR,
	.channelState = CY_DMA_CHANNEL_DISABLED,
	.triggerInType = CY_DMA_1ELEMENT,
	.dataSize = CY_DMA_HALFWORD,
	.srcTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
	.dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
	.descriptorType = CY_DMA_2D_TRANSFER,
	.srcAddress = NULL,
	.dstAddress = NULL,
	.srcXincrement = 0,
	.dstXincrement = 1,
	.xCount = 256,
	.srcYincrement = 0,
	.dstYincrement = 256,
	.yCount = 256,
	.nextDescriptor = &CYBSP_DMA_PDM_Descriptor_0,
};
cy_stc_dma_descriptor_t CYBSP_DMA_PDM_Descriptor_0 = 
{
	.ctl = 0UL,
	.src = 0UL,
	.dst = 0UL,
	.xCtl = 0UL,
	.yCtl = 0UL,
	.nextPtr = 0UL,
};
const cy_stc_dma_channel_config_t CYBSP_DMA_PDM_channelConfig = 
{
	.descriptor = &CYBSP_DMA_PDM_Descriptor_0,
	.preemptable = false,
	.priority = 3,
	.enable = false,
	.bufferable = false,
};
#if defined (CY_USING_HAL)
	const cyhal_resource_inst_t CYBSP_DMA_PDM_obj = 
	{
		.type = CYHAL_RSC_DMA,
		.block_num = 0U,
		.channel_num = CYBSP_DMA_PDM_CHANNEL,
	};
#endif //defined (CY_USING_HAL)
const cy_stc_dma_descriptor_config_t CYBSP_DMA_I2S_Descriptor_0_config = 
{
	.retrigger = CY_DMA_RETRIG_16CYC,
	.interruptType = CY_DMA_DESCR_CHAIN,
	.triggerOutType = CY_DMA_1ELEMENT,
	.channelState = CY_DMA_CHANNEL_DISABLED,
	.triggerInType = CY_DMA_1ELEMENT,
	.dataSize = CY_DMA_HALFWORD,
	.srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
	.dstTransferSize = CY_DMA_TRANSFER_SIZE_WORD,
	.descriptorType = CY_DMA_2D_TRANSFER,
	.srcAddress = NULL,
	.dstAddress = NULL,
	.srcXincrement = 1,
	.dstXincrement = 0,
	.xCount = 256,
	.srcYincrement = 256,
	.dstYincrement = 0,
	.yCount = 256,
	.nextDescriptor = &CYBSP_DMA_I2S_Descriptor_0,
};
cy_stc_dma_descriptor_t CYBSP_DMA_I2S_Descriptor_0 = 
{
	.ctl = 0UL,
	.src = 0UL,
	.dst = 0UL,
	.xCtl = 0UL,
	.yCtl = 0UL,
	.nextPtr = 0UL,
};
const cy_stc_dma_channel_config_t CYBSP_DMA_I2S_channelConfig = 
{
	.descriptor = &CYBSP_DMA_I2S_Descriptor_0,
	.preemptable = false,
	.priority = 3,
	.enable = false,
	.bufferable = false,
};
#if defined (CY_USING_HAL)
	const cyhal_resource_inst_t CYBSP_DMA_I2S_obj = 
	{
		.type = CYHAL_RSC_DMA,
		.block_num = 0U,
		.channel_num = CYBSP_DMA_I2S_CHANNEL,
	};
#endif //defined (CY_USING_HAL)


void init_cycfg_dmas(void)
{
#if defined (CY_USING_HAL)
	cyhal_hwmgr_reserve(&CYBSP_DMA_PDM_obj);
#endif //defined (CY_USING_HAL)

#if defined (CY_USING_HAL)
	cyhal_hwmgr_reserve(&CYBSP_DMA_I2S_obj);
#endif //defined (CY_USING_HAL)
}
