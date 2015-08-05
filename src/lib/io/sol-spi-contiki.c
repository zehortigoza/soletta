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

#include <stdlib.h>

#include <contiki.h>
#include <dev/gpio.h>
#include <dev/spi-new.h>

#define SOL_LOG_DOMAIN &_log_domain
#include "sol-log-internal.h"
#include "sol-event-handler-contiki.h"
#include "sol-mainloop.h"
#include "sol-spi.h"
#include "sol-util.h"

SOL_LOG_INTERNAL_DECLARE_STATIC(_log_domain, "spi");

/* Flags */
#define INTERN_ALLOCATED_TX_BUFFER (1 << 0)
#define INTERN_ALLOCATED_RX_BUFFER (1 << 1)
#define BUSY (1 << 2)

struct sol_spi {
    unsigned int bus;
    unsigned int cs_pin;
    struct {
        void (*transfer_cb)(void *cb_data, struct sol_spi *spi, const uint8_t *tx, uint8_t *rx, ssize_t status);
        const void *cb_data;
        const uint8_t *tx;
        uint8_t *rx;
        size_t count;
        uint8_t flags;
    } transfer;
};

static spi_speed_t
uint32_to_spi_speed_enum(uint32_t freq)
{
    if (freq >= 10000000)
        return SPI_SPEED_10M;
    if (freq >= 5000000)
        return SPI_SPEED_5M;
    if (freq >= 1000000)
        return SPI_SPEED_1M;
    if (freq >= 400000)
        return SPI_SPEED_400K;
    return SPI_SPEED_100K;
}

static void
spi_transfer_cb(void *user_data, process_event_t ev, process_data_t ev_data)
{
    struct sol_spi *spi = user_data;
    spi_event_data_t *spi_event = ev_data;
    ssize_t status;

    gpio_write(spi->cs_pin, 1);

    if (spi->transfer.flags & INTERN_ALLOCATED_TX_BUFFER) {
        free((uint8_t *)spi->transfer.tx);
        spi->transfer.tx = NULL;
    }
    if (spi->transfer.flags & INTERN_ALLOCATED_RX_BUFFER) {
        free(spi->transfer.rx);
        spi->transfer.rx = NULL;
    }
    spi->transfer.flags = 0;

    if (!spi->transfer.transfer_cb)
        return;

    status = spi_event->status < 0 ? -1 : spi->transfer.count;
    spi->transfer.transfer_cb((void *)spi->transfer.cb_data, spi,
        spi->transfer.tx, spi->transfer.rx, status);
}

SOL_API struct sol_spi *
sol_spi_open(unsigned int bus, const struct sol_spi_config *config)
{
    struct sol_spi *spi;
    spi_speed_t speed;

    SOL_LOG_INTERNAL_INIT_ONCE;

    if (unlikely(config->api_version != SOL_SPI_CONFIG_API_VERSION)) {
        SOL_WRN("Couldn't open SPI that has unsupported version '%u', "
            "expected version is '%u'",
            config->api_version, SOL_SPI_CONFIG_API_VERSION);
        return NULL;
    }

    SOL_EXP_CHECK(config->bits_per_word != 8, NULL);

    spi = malloc(sizeof(struct sol_spi));
    SOL_NULL_CHECK(spi, NULL);

    speed = uint32_to_spi_speed_enum(config->frequency);
    SOL_EXP_CHECK_GOTO(spi_init_master(bus, speed, config->mode) != 0, error);

    SOL_EXP_CHECK_GOTO(gpio_init(spi->cs_pin, GPIO_MODE_OUTPUT, GPIO_NO_PULL) != 0, error);
    gpio_write(spi->cs_pin, 1);

    SOL_EXP_CHECK_GOTO(
        !sol_mainloop_contiki_event_handler_add(spi_master_transfer_event, NULL,
        spi_transfer_cb, spi), error);

    spi->bus = bus;
    spi->cs_pin = config->chip_select;
    spi->transfer.flags = 0;

    return spi;

error:
    SOL_WRN("%u,%u: Unable to setup SPI", bus, config->chip_select);
    free(spi);
    return NULL;
}

SOL_API bool
sol_spi_transfer(struct sol_spi *spi, const uint8_t *tx, uint8_t *rx, size_t count, void (*transfer_cb)(void *cb_data, struct sol_spi *spi, const uint8_t *tx, uint8_t *rx, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(spi, false);
    SOL_EXP_CHECK(spi->transfer.flags & BUSY, false);

    if (tx == NULL) {
        tx = calloc(count, sizeof(uint8_t));
        SOL_NULL_CHECK(tx, false);
        spi->transfer.flags |= INTERN_ALLOCATED_TX_BUFFER;
    }
    if (rx == NULL) {
        rx = calloc(count, sizeof(uint8_t));
        SOL_NULL_CHECK_GOTO(rx, rx_alloc_fail);
        spi->transfer.flags |= INTERN_ALLOCATED_RX_BUFFER;
    }

    gpio_write(spi->cs_pin, 0);
    SOL_EXP_CHECK_GOTO(spi_master_transceive(spi->bus, tx, rx, count), transceive_fail);

    spi->transfer.flags |= BUSY;
    spi->transfer.transfer_cb = transfer_cb;
    spi->transfer.cb_data = cb_data;
    spi->transfer.tx = tx;
    spi->transfer.rx = rx;
    spi->transfer.count = count;
    return true;

transceive_fail:
    gpio_write(spi->cs_pin, 1);
    if (spi->transfer.flags & INTERN_ALLOCATED_RX_BUFFER) {
        free(rx);
        spi->transfer.flags &= ~INTERN_ALLOCATED_RX_BUFFER;
    }
rx_alloc_fail:
    if (spi->transfer.flags & INTERN_ALLOCATED_TX_BUFFER) {
        free((uint8_t *)tx);
        spi->transfer.flags &= ~INTERN_ALLOCATED_TX_BUFFER;
    }
    return false;
}

SOL_API void
sol_spi_close(struct sol_spi *spi)
{
    SOL_NULL_CHECK(spi);

    sol_mainloop_contiki_event_handler_del(spi_master_transfer_event, NULL,
        spi_transfer_cb, spi);

    if (spi->transfer.flags & BUSY) {
        spi_event_data_t spi_event_data;
        spi_event_data.dev = spi->bus;
        spi_event_data.status = -1;
        spi_transfer_cb(spi, spi_master_transfer_event, &spi_event_data);
    }

    spi_shutdown(spi->bus);
    free(spi);
}
