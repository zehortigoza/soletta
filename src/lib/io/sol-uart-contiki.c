/*
 * This file is part of the Soletta Project
 *
 * Copyright (C) 2015 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <stdlib.h>

#include <dev/uart.h>

#define SOL_LOG_DOMAIN &_log_domain
#include "sol-log-internal.h"
#include "sol-event-handler-contiki.h"
#include "sol-uart.h"
#include "sol-util.h"

SOL_LOG_INTERNAL_DECLARE_STATIC(_log_domain, "uart");

struct sol_uart {
    unsigned int id;
    struct {
        const unsigned char *buffer;
        void (*user_cb)(void *data, struct sol_uart *uart, unsigned char *tx, int status);
        const void *user_data;
    } tx;
    struct {
        void (*user_cb)(void *user_data, struct sol_uart *uart, unsigned char read_char);
        const void *user_data;
        unsigned char buffer[32];
    } rx;
};

static void
uart_rx_cb(unsigned int dev, void *data, unsigned char *buffer, unsigned int count)
{
    struct sol_uart *uart = data;
    unsigned int i;

    if (!uart->rx.user_cb)
        return;

    for (i = 0; i < count; i++)
        uart->rx.user_cb((void *)uart->rx.user_data, uart, (char)buffer[i]);
}

static void
uart_tx_dispatch(struct sol_uart *uart, int status)
{
    unsigned char *tx = (unsigned char *)uart->tx.buffer;

    uart->tx.buffer = NULL;
    if (!uart->tx.user_cb)
        return;
    uart->tx.user_cb((void *)uart->tx.user_data, uart, tx, status);
}

static void
tx_handler_cb(void *user_data, process_event_t ev, process_data_t ev_data)
{
    struct sol_uart *uart = user_data;
    uart_write_event_data_t *write_event = ev_data;

    uart_tx_dispatch(uart, write_event->status);
}

SOL_API struct sol_uart *
sol_uart_open(const char *port_name, const struct sol_uart_config *config)
{
    struct sol_uart *uart;
    const uart_baud_rate_t baud_rate_table[] = {
        [SOL_UART_BAUD_RATE_9600] = UART_BAUD_RATE_9600,
        [SOL_UART_BAUD_RATE_19200] = UART_BAUD_RATE_19200,
        [SOL_UART_BAUD_RATE_38400] = UART_BAUD_RATE_38400,
        [SOL_UART_BAUD_RATE_57600] = UART_BAUD_RATE_57600,
        [SOL_UART_BAUD_RATE_115200] = UART_BAUD_RATE_115200
    };
    const uart_parity_t parity_table[] = {
        [SOL_UART_PARITY_NONE] = UART_PARITY_DISABLE,
        [SOL_UART_PARITY_EVEN] = UART_PARITY_ENABLE_EVEN,
        [SOL_UART_PARITY_ODD] = UART_PARITY_ENABLE_ODD,
    };
    uart_data_bits_t data_bits;
    uart_stop_bits_t stop_bits;
    int ret;

    SOL_LOG_INTERNAL_INIT_ONCE;

    if (unlikely(config->api_version != SOL_UART_CONFIG_API_VERSION)) {
        SOL_WRN("Couldn't open UART that has unsupported version '%u', "
            "expected version is '%u'",
            config->api_version, SOL_UART_CONFIG_API_VERSION);
        return NULL;
    }

    SOL_EXP_CHECK(config->data_bits == SOL_UART_DATA_BITS_6, NULL);
    SOL_EXP_CHECK(config->data_bits == SOL_UART_DATA_BITS_5, NULL);
    SOL_EXP_CHECK(config->flow_control, NULL);
    SOL_NULL_CHECK(port_name, NULL);

    uart = malloc(sizeof(struct sol_uart));
    SOL_NULL_CHECK(uart, NULL);

    uart->id = strtol(port_name, NULL, 10);

    data_bits = config->data_bits == SOL_UART_DATA_BITS_8 ?
        UART_DATA_BITS_8 : UART_DATA_BITS_7;

    stop_bits = config->stop_bits == SOL_UART_STOP_BITS_TWO ?
        UART_STOP_BITS_TWO : UART_STOP_BITS_ONE;

    ret = uart_init(uart->id, baud_rate_table[config->baud_rate], data_bits,
        parity_table[config->parity], stop_bits, uart_rx_cb, uart,
        uart->rx.buffer, sizeof(uart->rx.buffer));
    SOL_INT_CHECK_GOTO(ret, != 0, fail);

    sol_mainloop_contiki_event_handler_add(uart_write_event, NULL,
        tx_handler_cb, uart);

    uart->rx.user_cb = config->rx_cb;
    uart->rx.user_data = config->rx_cb_user_data;
    uart->tx.buffer = NULL;

    return uart;

fail:
    free(uart);
    return NULL;
}

SOL_API void
sol_uart_close(struct sol_uart *uart)
{
    SOL_NULL_CHECK(uart);

    sol_mainloop_contiki_event_handler_del(uart_write_event, NULL,
        tx_handler_cb, uart);
    if (uart->tx.buffer)
        uart_tx_dispatch(uart, -1);
    uart_shutdown(uart->id);
    free(uart);
}

SOL_API bool
sol_uart_write(struct sol_uart *uart, const unsigned char *tx, unsigned int length, void (*tx_cb)(void *data, struct sol_uart *uart, unsigned char *tx, int status), const void *data)
{
    SOL_NULL_CHECK(uart, false);
    SOL_EXP_CHECK(uart->tx.buffer, false);
    SOL_NULL_CHECK(tx, false);
    SOL_EXP_CHECK(!length, false);

    uart->tx.buffer = tx;
    uart->tx.user_cb = tx_cb;
    uart->tx.user_data = data;

    if (uart_write(uart->id, uart->tx.buffer, length) != 0) {
        uart->tx.buffer = NULL;
        return false;
    }

    return true;
}
