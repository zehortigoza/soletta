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
#include <dev/i2c.h>

#define SOL_LOG_DOMAIN &_log_domain
#include "sol-log-internal.h"
#include "sol-event-handler-contiki.h"
#include "sol-mainloop.h"
#include "sol-i2c.h"
#include "sol-util.h"

SOL_LOG_INTERNAL_DECLARE_STATIC(_log_domain, "i2c");

#define FLAG_BUSY (1 << 0)
#define FLAG_OPERATION_READ (1 << 1)
#define FLAG_OPERATION_WRITE (1 << 2)
#define FLAG_OPERATION_WITH_REG (1 << 3)
#define FLAG_OPERATION_MULTIPLE (1 << 4)
#define FLAG_PENDING_CANCEL (1 << 4)

struct sol_i2c {
    uint8_t bus;
    uint8_t slave_addr;
    struct {
        const void *cb_data;
        uint8_t *data;
        void (*dispatch)(struct sol_i2c *i2c);
        union {
            struct {
                void (*cb)(void *cb_data, struct sol_i2c *i2c, uint8_t *data, ssize_t status);
            } read_write_cb;
            struct {
                void (*cb)(void *cb_data, struct sol_i2c *i2c, uint8_t reg, uint8_t *data, ssize_t status);
            } read_write_reg_cb;
        };
        size_t count;
        ssize_t status;
        uint8_t reg;
        uint8_t flags;
        uint8_t times; // Only used on read_register_multiple()
        uint8_t times_done; // Only used on read_register_multiple()
    } async;
};

static void
i2c_pending_cancel_handle(struct sol_i2c *i2c)
{
    i2c->async.flags &= ~FLAG_PENDING_CANCEL;

    switch (i2c->async.flags) {
    case FLAG_OPERATION_READ:
        sol_i2c_read(i2c, i2c->async.data, i2c->async.count,
            i2c->async.read_write_cb.cb, i2c->async.cb_data);
        break;
    case FLAG_OPERATION_READ | FLAG_OPERATION_WITH_REG:
        sol_i2c_read_register(i2c, i2c->async.reg, i2c->async.data,
            i2c->async.count, i2c->async.read_write_reg_cb.cb, i2c->async.cb_data);
        break;
    case FLAG_OPERATION_READ | FLAG_OPERATION_WITH_REG | FLAG_OPERATION_MULTIPLE:
        sol_i2c_read_register_multiple(i2c, i2c->async.reg, i2c->async.data,
            i2c->async.count, i2c->async.times, i2c->async.read_write_reg_cb.cb,
            i2c->async.cb_data);
        break;
    case FLAG_OPERATION_WRITE:
        sol_i2c_write(i2c, i2c->async.data, i2c->async.count,
            i2c->async.read_write_cb.cb, i2c->async.cb_data);
        break;
    case FLAG_OPERATION_WRITE | FLAG_OPERATION_WITH_REG:
        sol_i2c_write_register(i2c, i2c->async.reg, i2c->async.data,
            i2c->async.count, i2c->async.read_write_reg_cb.cb, i2c->async.cb_data);
        break;
    default:
        SOL_WRN("Unexpected flags=%d", i2c->async.flags);
    }
}

static void
i2c_write_cb(void *user_data, process_event_t ev, process_data_t ev_data)
{
    i2c_event_data_t *i2c_event = ev_data;
    struct sol_i2c *i2c = user_data;

    if (i2c->async.flags & FLAG_PENDING_CANCEL) {
        i2c_pending_cancel_handle(i2c);
        return;
    }

    if (i2c_event->status < 0) {
        i2c->async.dispatch(i2c);
        return;
    }

    if (i2c->async.flags & FLAG_OPERATION_READ) {
        /* handling a read register, multiple or single. */
        uint8_t *data = i2c->async.data + (i2c->async.count * i2c->async.times_done);
        if (i2c_master_read(i2c->bus, i2c->slave_addr, data, i2c->async.count)) {
            SOL_WRN("Error reading from I2C after write the register.");
            i2c->async.dispatch(i2c);
        }
        return;
    }

    if (i2c->async.flags & FLAG_OPERATION_WITH_REG) {
        i2c->async.flags &= ~FLAG_OPERATION_WITH_REG;
        if (i2c_master_write(i2c->bus, i2c->slave_addr, i2c->async.data,
            i2c->async.count)) {
            SOL_WRN("Error writing to I2C bus:%d", i2c->bus);
            i2c->async.dispatch(i2c);
        }
        return;
    }

    i2c->async.status = i2c->async.count;
    i2c->async.dispatch(i2c);
}

static void
i2c_read_cb(void *user_data, process_event_t ev, process_data_t ev_data)
{
    i2c_event_data_t *i2c_event = ev_data;
    struct sol_i2c *i2c = user_data;

    if (i2c->async.flags & FLAG_PENDING_CANCEL) {
        i2c_pending_cancel_handle(i2c);
        return;
    }

    if (i2c_event->status < 0) {
        i2c->async.dispatch(i2c);
        return;
    }

    /* is a single read? Doesn't matter if is with reg or not */
    if (!(i2c->async.flags & FLAG_OPERATION_MULTIPLE)) {
        i2c->async.status = i2c->async.count;
        i2c->async.dispatch(i2c);
        return;
    }

    /* multiple read */
    i2c->async.times_done++;
    if (i2c->async.times_done == i2c->async.times) {
        i2c->async.status = i2c->async.times * i2c->async.count;
        i2c->async.dispatch(i2c);
    } else {
        if (i2c_master_write(i2c->bus, i2c->slave_addr, &i2c->async.reg, 1)) {
            SOL_WRN("Error writing to I2C.");
            i2c->async.dispatch(i2c);
        }
    }
}

SOL_API struct sol_i2c *
sol_i2c_open_raw(uint8_t bus, enum sol_i2c_speed speed)
{
    struct sol_i2c *i2c;
    const uint8_t table[] = {
        [SOL_I2C_SPEED_10KBIT] = I2C_SPEED_100K,
        [SOL_I2C_SPEED_100KBIT] = I2C_SPEED_100K,
        [SOL_I2C_SPEED_400KBIT] = I2C_SPEED_400K,
        [SOL_I2C_SPEED_1MBIT] = I2C_SPEED_1MB,
        [SOL_I2C_SPEED_3MBIT_400KBIT] = I2C_SPEED_3_2MB
    };

    SOL_LOG_INTERNAL_INIT_ONCE;

    i2c = malloc(sizeof(struct sol_i2c));
    SOL_NULL_CHECK(i2c, NULL);

    SOL_EXP_CHECK_GOTO(i2c_init_master(bus, table[speed]), init_error);

    SOL_EXP_CHECK_GOTO(
        sol_mainloop_contiki_event_handler_add(i2c_master_write_event, NULL,
        i2c_write_cb, i2c), init_error);
    SOL_EXP_CHECK_GOTO(
        sol_mainloop_contiki_event_handler_add(i2c_master_read_event, NULL,
        i2c_read_cb, i2c), add_read_handler_error);

    i2c->bus = bus;
    i2c->async.flags = 0;

    return i2c;

add_read_handler_error:
    sol_mainloop_contiki_event_handler_del(i2c_master_write_event, NULL,
        i2c_write_cb, i2c);
init_error:
    free(i2c);
    return NULL;
}

SOL_API void
sol_i2c_close_raw(struct sol_i2c *i2c)
{
    SOL_NULL_CHECK(i2c);

    if (i2c->async.flags & FLAG_BUSY)
        sol_i2c_pending_cancel(i2c, (struct sol_i2c_pending *)&i2c->async.flags);

    sol_mainloop_contiki_event_handler_del(i2c_master_write_event, NULL,
        i2c_write_cb, i2c);
    sol_mainloop_contiki_event_handler_del(i2c_master_read_event, NULL,
        i2c_read_cb, i2c);

    i2c_shutdown(i2c->bus);
    free(i2c);
}

SOL_API bool
sol_i2c_set_slave_address(struct sol_i2c *i2c, uint8_t slave_address)
{
    SOL_NULL_CHECK(i2c, false);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, false);
    i2c->slave_addr = slave_address;
    return true;
}

SOL_API uint8_t
sol_i2c_get_slave_address(struct sol_i2c *i2c)
{
    SOL_NULL_CHECK(i2c, 0);
    return i2c->slave_addr;
}

SOL_API struct sol_i2c_pending *
sol_i2c_write_quick(struct sol_i2c *i2c, bool rw, void (*write_quick_cb)(void *cb_data, struct sol_i2c *i2c, ssize_t status), const void *cb_data)
{
    SOL_CRI("Unsupported");
    return NULL;
}

static void
_i2c_read_write_dispatch(struct sol_i2c *i2c)
{
    i2c->async.flags = 0;
    if (!i2c->async.read_write_cb.cb) return;
    i2c->async.read_write_cb.cb((void *)i2c->async.cb_data, i2c,
        i2c->async.data, i2c->async.status);
}

SOL_API struct sol_i2c_pending *
sol_i2c_read(struct sol_i2c *i2c, uint8_t *data, size_t count, void (*read_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t *data, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(i2c, NULL);
    SOL_NULL_CHECK(data, NULL);
    SOL_INT_CHECK(count, == 0, NULL);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, NULL);

    i2c->async.flags |= FLAG_BUSY | FLAG_OPERATION_READ;
    i2c->async.read_write_cb.cb = read_cb;
    i2c->async.dispatch = _i2c_read_write_dispatch;
    i2c->async.cb_data = cb_data;
    i2c->async.data = data;
    i2c->async.count = count;
    i2c->async.status = -1;
    i2c->async.times_done = 0;

    if (i2c->async.flags & FLAG_PENDING_CANCEL)
        return (struct sol_i2c_pending *)&i2c->async.flags;

    SOL_EXP_CHECK(i2c_master_read(i2c->bus, i2c->slave_addr, data, count), NULL);

    return (struct sol_i2c_pending *)&i2c->async.flags;
}

SOL_API struct sol_i2c_pending *
sol_i2c_write(struct sol_i2c *i2c, uint8_t *data, size_t count, void (*write_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t *data, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(i2c, NULL);
    SOL_NULL_CHECK(data, NULL);
    SOL_INT_CHECK(count, == 0, NULL);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, NULL);

    i2c->async.flags |= FLAG_BUSY | FLAG_OPERATION_WRITE;
    i2c->async.read_write_cb.cb = write_cb;
    i2c->async.dispatch = _i2c_read_write_dispatch;
    i2c->async.cb_data = cb_data;
    i2c->async.data = data;
    i2c->async.count = count;
    i2c->async.status = -1;
    i2c->async.times_done = 0;

    if (i2c->async.flags & FLAG_PENDING_CANCEL)
        return (struct sol_i2c_pending *)&i2c->async.flags;

    SOL_EXP_CHECK(i2c_master_write(i2c->bus, i2c->slave_addr, data, count), NULL);

    return (struct sol_i2c_pending *)&i2c->async.flags;
}

static void
_i2c_read_write_reg_dispatch(struct sol_i2c *i2c)
{
    i2c->async.flags = 0;
    if (!i2c->async.read_write_reg_cb.cb) return;
    i2c->async.read_write_reg_cb.cb((void *)i2c->async.cb_data, i2c,
        i2c->async.reg, i2c->async.data, i2c->async.status);
}

static struct sol_i2c_pending *
_i2c_read_write_register(struct sol_i2c *i2c, uint8_t reg, uint8_t *data, size_t count, void (*read_write_reg_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t reg, uint8_t *data, ssize_t status), const void *cb_data)
{
    i2c->async.read_write_reg_cb.cb = read_write_reg_cb;
    i2c->async.dispatch = _i2c_read_write_reg_dispatch;
    i2c->async.cb_data = cb_data;
    i2c->async.data = data;
    i2c->async.count = count;
    i2c->async.status = -1;
    i2c->async.times_done = 0;
    i2c->async.reg = reg;

    if (i2c->async.flags & FLAG_PENDING_CANCEL)
        return (struct sol_i2c_pending *)&i2c->async.flags;

    SOL_EXP_CHECK(i2c_master_write(i2c->bus, i2c->slave_addr, &i2c->async.reg, 1), NULL);

    return (struct sol_i2c_pending *)&i2c->async.flags;
}


SOL_API struct sol_i2c_pending *
sol_i2c_read_register(struct sol_i2c *i2c, uint8_t reg, uint8_t *data, size_t count, void (*read_reg_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t reg, uint8_t *data, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(i2c, NULL);
    SOL_NULL_CHECK(data, NULL);
    SOL_INT_CHECK(count, == 0, NULL);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, NULL);

    i2c->async.flags |= FLAG_BUSY | FLAG_OPERATION_READ | FLAG_OPERATION_WITH_REG;
    return _i2c_read_write_register(i2c, reg, data, count, read_reg_cb, cb_data);
}

SOL_API struct sol_i2c_pending *
sol_i2c_write_register(struct sol_i2c *i2c, uint8_t reg, const uint8_t *data, size_t count, void (*write_reg_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t reg, uint8_t *data, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(i2c, NULL);
    SOL_NULL_CHECK(data, NULL);
    SOL_INT_CHECK(count, == 0, NULL);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, NULL);

    i2c->async.flags |= FLAG_BUSY | FLAG_OPERATION_WRITE | FLAG_OPERATION_WITH_REG;
    return _i2c_read_write_register(i2c, reg, (uint8_t *)data, count, write_reg_cb, cb_data);
}

SOL_API struct sol_i2c_pending *
sol_i2c_read_register_multiple(struct sol_i2c *i2c, uint8_t reg, uint8_t *data, size_t count, uint8_t times, void (*read_reg_multiple_cb)(void *cb_data, struct sol_i2c *i2c, uint8_t reg, uint8_t *data, ssize_t status), const void *cb_data)
{
    SOL_NULL_CHECK(i2c, NULL);
    SOL_NULL_CHECK(data, NULL);
    SOL_INT_CHECK(count, == 0, NULL);
    SOL_EXP_CHECK(i2c->async.flags & FLAG_BUSY, NULL);

    i2c->async.flags |= FLAG_BUSY | FLAG_OPERATION_READ;
    i2c->async.flags |= FLAG_OPERATION_WITH_REG | FLAG_OPERATION_MULTIPLE;
    i2c->async.times = times;
    return _i2c_read_write_register(i2c, reg, data, count, read_reg_multiple_cb, cb_data);
}

SOL_API bool
sol_i2c_busy(struct sol_i2c *i2c)
{
    SOL_NULL_CHECK(i2c, true);
    return i2c->async.flags & FLAG_BUSY;
}

SOL_API uint8_t
sol_i2c_bus_get(const struct sol_i2c *i2c)
{
    SOL_NULL_CHECK(i2c, 0);
    return i2c->bus;
}

SOL_API void
sol_i2c_pending_cancel(struct sol_i2c *i2c, struct sol_i2c_pending *pending)
{
    SOL_NULL_CHECK(i2c);
    SOL_NULL_CHECK(pending);

    if ((struct sol_i2c_pending *)&i2c->async.flags != pending || !(i2c->async.flags & FLAG_BUSY)) {
        SOL_WRN("Invalid I2C pending handle.");
        return;
    }

    i2c->async.dispatch(i2c);
    i2c->async.flags = FLAG_PENDING_CANCEL;
}
