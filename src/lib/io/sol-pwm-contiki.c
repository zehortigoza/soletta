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
#include <dev/pwm.h>

#define SOL_LOG_DOMAIN &_log_domain
#include "sol-log-internal.h"
#include "sol-pwm.h"
#include "sol-util.h"

SOL_LOG_INTERNAL_DECLARE_STATIC(_log_domain, "pwm");

struct dev_ref {
    unsigned int dev;
    uint16_t ref;/* number of channels depending on it */
    uint16_t channels_enabled;/* number of channels generating PWM */
};
static struct sol_vector pwm_dev_vector = SOL_VECTOR_INIT(struct dev_ref);

struct sol_pwm {
    uint32_t period, duty;
    unsigned int dev;
    unsigned int channel;
    bool enabled;
};

static struct dev_ref *
_pwm_dev_get(unsigned int dev, uint16_t *index)
{
    uint16_t i;
    struct dev_ref *dev_ref;

    SOL_VECTOR_FOREACH_IDX (&pwm_dev_vector, dev_ref, i) {
        if (dev_ref->dev == dev) {
            if (index)
                *index = i;
            return dev_ref;
        }
    }

    return NULL;
}

SOL_API struct sol_pwm *
sol_pwm_open_raw(int device, int channel, const struct sol_pwm_config *config)
{
    struct sol_pwm *pwm;
    struct dev_ref *dev_ref;

    SOL_LOG_INTERNAL_INIT_ONCE;

    if (unlikely(config->api_version != SOL_PWM_CONFIG_API_VERSION)) {
        SOL_WRN("Couldn't open pwm that has unsupported version '%u', "
            "expected version is '%u'",
            config->api_version, SOL_PWM_CONFIG_API_VERSION);
        return NULL;
    }

    SOL_EXP_CHECK(config->alignment != SOL_PWM_ALIGNMENT_LEFT, NULL);
    SOL_EXP_CHECK(config->polarity != SOL_PWM_POLARITY_NORMAL, NULL);

    pwm = calloc(1, sizeof(struct sol_pwm));
    SOL_NULL_CHECK(pwm, NULL);

    pwm->dev = device;
    pwm->channel = channel;

    dev_ref = _pwm_dev_get(pwm->dev, NULL);
    if (!dev_ref) {
        dev_ref = sol_vector_append(&pwm_dev_vector);
        SOL_NULL_CHECK_GOTO(dev_ref, error_append_dev_ref);
        dev_ref->dev = device;
        dev_ref->channels_enabled = dev_ref->ref = 0;
        pwm_init(dev_ref->dev);
    }
    dev_ref->ref++;

    if (config->period_ns != -1)
        sol_pwm_set_period(pwm, config->period_ns);
    if (config->duty_cycle_ns != -1)
        sol_pwm_set_duty_cycle(pwm, config->duty_cycle_ns);
    sol_pwm_set_enabled(pwm, config->enabled);

    return pwm;

error_append_dev_ref:
    free(pwm);
    return NULL;
}

SOL_API void
sol_pwm_close(struct sol_pwm *pwm)
{
    struct dev_ref *dev_ref;
    uint16_t index;

    SOL_NULL_CHECK(pwm);

    sol_pwm_set_enabled(pwm, false);
    dev_ref = _pwm_dev_get(pwm->dev, &index);
    dev_ref->ref--;
    if (!dev_ref->ref) {
        sol_vector_del(&pwm_dev_vector, index);
        pwm_stop(pwm->dev);
    }

    free(pwm);
}

SOL_API bool
sol_pwm_set_enabled(struct sol_pwm *pwm, bool enable)
{
    struct dev_ref *dev_ref;

    SOL_NULL_CHECK(pwm, false);

    if (pwm->enabled == enable)
        return true;
    pwm->enabled = enable;

    dev_ref = _pwm_dev_get(pwm->dev, NULL);

    if (pwm->enabled) {
        pwm_set(pwm->dev, pwm->channel, pwm->period, pwm->duty);
        if (!dev_ref->channels_enabled)
            pwm_start(pwm->dev);
        dev_ref->channels_enabled++;
    } else {
        pwm_set(pwm->dev, pwm->channel, pwm->period, 0);
        dev_ref->channels_enabled--;
        if (!dev_ref->channels_enabled)
            pwm_stop(pwm->dev);
    }

    return true;
}

SOL_API bool
sol_pwm_get_enabled(const struct sol_pwm *pwm)
{
    SOL_NULL_CHECK(pwm, false);
    return pwm->enabled;
}

SOL_API bool
sol_pwm_set_period(struct sol_pwm *pwm, uint32_t period_ns)
{
    SOL_NULL_CHECK(pwm, false);

    pwm->period = period_ns;
    if (!pwm->enabled)
        return true;

    return pwm_set(pwm->dev, pwm->channel, pwm->period, pwm->duty) == 0;
}

SOL_API int32_t
sol_pwm_get_period(const struct sol_pwm *pwm)
{
    SOL_NULL_CHECK(pwm, -1);
    return (int32_t)pwm->period;
}

SOL_API bool
sol_pwm_set_duty_cycle(struct sol_pwm *pwm, uint32_t duty_cycle_ns)
{
    SOL_NULL_CHECK(pwm, false);

    pwm->duty = duty_cycle_ns;
    if (!pwm->enabled)
        return true;

    return pwm_update(pwm->dev, pwm->channel, pwm->duty) == 0;
}

SOL_API int32_t
sol_pwm_get_duty_cycle(const struct sol_pwm *pwm)
{
    SOL_NULL_CHECK(pwm, -1);
    return (int32_t)pwm->duty;
}
