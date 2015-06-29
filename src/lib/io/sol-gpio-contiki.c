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

#include <contiki.h>
#include <dev/gpio.h>

#define SOL_LOG_DOMAIN &_log_domain
#include "sol-log-internal.h"
#include "sol-event-handler-contiki.h"
#include "sol-gpio.h"
#include "sol-mainloop.h"
#include "sol-util.h"

SOL_LOG_INTERNAL_DECLARE_STATIC(_log_domain, "gpio");

struct sol_gpio {
    int pin;
    bool active_low;
    struct {
        void (*cb)(void *data, struct sol_gpio *gpio);
        const void *data;
        void *int_handler;
    } irq;
};

static void
event_handler_cb(void *user_data, process_event_t ev, process_data_t ev_data)
{
    struct sol_gpio *gpio = user_data;

    gpio->irq.cb((void *)gpio->irq.data, gpio);
}

SOL_API struct sol_gpio *
sol_gpio_open_raw(int pin, const struct sol_gpio_config *config)
{
    struct sol_gpio *gpio;
    const unsigned int drive_table[] = {
        [SOL_GPIO_DRIVE_NONE] = GPIO_NO_PULL,
        [SOL_GPIO_DRIVE_PULL_UP] = GPIO_PULL_UP,
        [SOL_GPIO_DRIVE_PULL_DOWN] = GPIO_PULL_DOWN
    };
    gpio_pull_resistor_t pull;

    SOL_LOG_INTERNAL_INIT_ONCE;

    if (unlikely(config->api_version != SOL_GPIO_CONFIG_API_VERSION)) {
        SOL_WRN("Couldn't open gpio that has unsupported version '%u'"
            "expected version is '%u'",
            config->api_version, SOL_GPIO_CONFIG_API_VERSION);
        return NULL;
    }

    gpio = malloc(sizeof(struct sol_gpio));
    SOL_NULL_CHECK(gpio, NULL);

    gpio->pin = pin;
    gpio->active_low = config->active_low;

    if (config->dir == SOL_GPIO_DIR_OUT) {
        if (gpio_init(gpio->pin, GPIO_MODE_OUTPUT, GPIO_NO_PULL) < 0)
            goto error;
        sol_gpio_write(gpio, config->out.value);
    } else {
        pull = drive_table[config->drive_mode];
        if (config->in.trigger_mode == SOL_GPIO_EDGE_NONE) {
            if (gpio_init(gpio->pin, GPIO_MODE_INPUT, pull) < 0)
                goto error;
        } else {
            gpio_interruption_trigger_t trigger;
            const unsigned int trigger_table[] = {
                [SOL_GPIO_EDGE_RISING] = GPIO_RISING,
                [SOL_GPIO_EDGE_FALLING] = GPIO_FALLING,
                [SOL_GPIO_EDGE_BOTH] = GPIO_BOTH
            };

            trigger = trigger_table[config->in.trigger_mode];
            gpio->irq.cb = config->in.cb;
            gpio->irq.data = config->in.user_data;
            if (gpio_init_input_interruption(gpio->pin, pull, trigger) < 0)
                goto error;
            sol_mainloop_contiki_event_handler_add(gpio_input_event,
                (void *)(long)pin, event_handler_cb, gpio);
        }
    }

    return gpio;
error:
    free(gpio);
    return NULL;
}

SOL_API void
sol_gpio_close(struct sol_gpio *gpio)
{
    SOL_NULL_CHECK(gpio);
    if (gpio->irq.cb)
        sol_mainloop_contiki_event_handler_del(gpio_input_event,
            (void *)(long)gpio->pin, event_handler_cb, gpio);
    gpio_disable(gpio->pin);
    free(gpio);
}

SOL_API bool
sol_gpio_write(struct sol_gpio *gpio, bool value)
{
    SOL_NULL_CHECK(gpio, false);
    gpio_write(gpio->pin, gpio->active_low ^ value);
    return true;
}

SOL_API int
sol_gpio_read(struct sol_gpio *gpio)
{
    SOL_NULL_CHECK(gpio, -EINVAL);
    return gpio->active_low ^ !!gpio_read(gpio->pin);
}
