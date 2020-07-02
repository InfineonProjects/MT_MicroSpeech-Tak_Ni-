/*
 * Copyright (c) 2018 Cypress
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief UART driver for Cypress PSoC6 MCU family.
 *
 * Note:
 * - Error handling is not implemented.
 * - The driver works only in polling mode, interrupt mode is not implemented.
 */

#include "cy_syslib.h"
#include "cy_sysclk.h"
#include "cy_gpio.h"
#include "cy_scb_uart.h"
#include "cycfg_peripherals.h"
#include "uart_psoc6.h"

#define DT_UART_PSOC6_UART_12_BASE_ADDRESS SCB6
#define DT_UART_PSOC6_UART_12_PORT         P12_0_PORT
#define DT_UART_PSOC6_UART_12_RX_NUM       P12_0_NUM
#define DT_UART_PSOC6_UART_12_TX_NUM       P12_1_NUM
#define DT_UART_PSOC6_UART_12_RX_VAL       P12_0_SCB6_UART_RX
#define DT_UART_PSOC6_UART_12_TX_VAL       P12_1_SCB6_UART_TX
#define DT_UART_PSOC6_UART_12_CLOCK        PCLK_SCB6_CLOCK

struct cypress_psoc6_config {
	CySCB_Type *base;
	GPIO_PRT_Type *port;
	uint32_t rx_num;
	uint32_t tx_num;
	en_hsiom_sel_t rx_val;
	en_hsiom_sel_t tx_val;
	en_clk_dst_t scb_clock;
};

static const struct cypress_psoc6_config cypress_psoc6_uart12_config = {
	.base = DT_UART_PSOC6_UART_12_BASE_ADDRESS,
	.port = DT_UART_PSOC6_UART_12_PORT,
	.rx_num = DT_UART_PSOC6_UART_12_RX_NUM,
	.tx_num = DT_UART_PSOC6_UART_12_TX_NUM,
	.rx_val = DT_UART_PSOC6_UART_12_RX_VAL,
	.tx_val = DT_UART_PSOC6_UART_12_TX_VAL,
	.scb_clock = DT_UART_PSOC6_UART_12_CLOCK,
};

void uart_psoc6_poll_out(unsigned char c)
{
	const struct cypress_psoc6_config *config = &cypress_psoc6_uart12_config;

	while (Cy_SCB_UART_Put(config->base, (uint32_t)c) != 1UL)
		;
}

void DebugLog(const char* s)
{
//        static uint8_t is_initialized = 0;
    	const struct cypress_psoc6_config *config = &cypress_psoc6_uart12_config;
        
//        if (is_initialized == 0)
//        {
//        	/* Configure UART to operate */
//        	(void) Cy_SCB_UART_Init(config->base, config, NULL);
//        	Cy_SCB_UART_Enable(config->base);
//            is_initialized = 1;
//        }


        for (uint32_t i = 0; s[i] != '\0'; i++){
            uart_psoc6_poll_out(s[i]);
        }



}



