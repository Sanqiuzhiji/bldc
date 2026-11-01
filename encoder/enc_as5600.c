/*
	Copyright 2026

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#include "enc_as5600.h"

#include "ch.h"
#include "hal.h"
#include "hw.h"

#include <stdio.h>
#include <string.h>

#define AS5600_ADDRESS                  0x36
#define AS5600_REG_STATUS               0x0B
#define AS5600_REG_RAW_ANGLE_H          0x0C

#define AS5600_STATUS_MAGNET_DETECTED   (1U << 5)
#define AS5600_STATUS_MAGNET_TOO_WEAK   (1U << 4)
#define AS5600_STATUS_MAGNET_TOO_STRONG (1U << 3)

#define AS5600_SAMPLE_RATE_HZ           1000
#define AS5600_STATUS_INTERVAL          100
#define AS5600_I2C_TIMEOUT_MS           2
#define AS5600_MAX_CONSECUTIVE_ERRORS   5
#define AS5600_DATA_TIMEOUT_MS          10

typedef struct
{
    volatile float angle_deg;
    volatile uint16_t raw_angle;
    volatile uint8_t status;
    volatile bool valid;
    volatile bool stop_now;
    volatile bool is_running;
    volatile uint32_t update_count;
    volatile uint32_t error_count;
    volatile uint32_t consecutive_errors;
    volatile systime_t last_update;
} AS5600_state_t;

static AS5600_state_t m_state;
static THD_WORKING_AREA(as5600_thread_wa, 384);
static THD_FUNCTION(as5600_thread, arg);

static bool read_registers(uint8_t reg, uint8_t* data, size_t len)
{
    msg_t res;
    systime_t timeout = MS2ST(AS5600_I2C_TIMEOUT_MS);

    i2cAcquireBus(&HW_I2C_DEV);
    res = i2cMasterTransmitTimeout(&HW_I2C_DEV, AS5600_ADDRESS,
                                   &reg, 1, data, len, timeout);
    i2cReleaseBus(&HW_I2C_DEV);

    return res == MSG_OK;
}

static bool read_angle(void)
{
    uint8_t data[2];
    if (!read_registers(AS5600_REG_RAW_ANGLE_H, data, sizeof(data)))
    {
        m_state.error_count++;
        m_state.consecutive_errors++;
        return false;
    }

    uint16_t raw = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    m_state.raw_angle = raw;
    m_state.angle_deg = (float)raw * (360.0f / 4096.0f);
    m_state.last_update = chVTGetSystemTimeX();
    m_state.update_count++;
    m_state.consecutive_errors = 0;
    m_state.valid = true;

    return true;
}

static bool read_status(void)
{
    uint8_t status;
    if (read_registers(AS5600_REG_STATUS, &status, 1))
    {
        m_state.status = status;
        return true;
    }
    else
    {
        m_state.error_count++;
        return false;
    }
}

bool enc_as5600_init(void)
{
    memset(&m_state, 0, sizeof(m_state));
    hw_start_i2c();

    // Require one valid angle before exposing the encoder to the motor control.
    if (!read_angle())
    {
        hw_stop_i2c();
        return false;
    }

    if (!read_status())
    {
        m_state.valid = false;
        hw_stop_i2c();
        return false;
    }
    m_state.stop_now = false;
    m_state.is_running = true;
    chThdCreateStatic(as5600_thread_wa, sizeof(as5600_thread_wa),
                      NORMALPRIO + 5, as5600_thread, NULL);

    return true;
}

void enc_as5600_deinit(void)
{
    if (m_state.is_running)
    {
        m_state.stop_now = true;
        while (m_state.is_running)
        {
            chThdSleepMilliseconds(1);
        }
    }

    m_state.valid = false;
    hw_stop_i2c();
}

float enc_as5600_read_deg(void)
{
    return m_state.angle_deg;
}

bool enc_as5600_has_fault(void)
{
    if (!m_state.valid ||
        m_state.consecutive_errors >= AS5600_MAX_CONSECUTIVE_ERRORS)
    {
        return true;
    }

    if (chVTTimeElapsedSinceX(m_state.last_update) > MS2ST(AS5600_DATA_TIMEOUT_MS))
    {
        return true;
    }

    return !(m_state.status & AS5600_STATUS_MAGNET_DETECTED) ||
    (m_state.status & (AS5600_STATUS_MAGNET_TOO_WEAK |
        AS5600_STATUS_MAGNET_TOO_STRONG));
}

char* enc_as5600_print_info(void)
{
    static char info[160];
    snprintf(info, sizeof(info),
             "AS5600 raw: %u, angle: %.2f deg, updates: %lu, errors: %lu, magnet: %s%s%s",
             (unsigned)m_state.raw_angle,
             (double)m_state.angle_deg,
             (unsigned long)m_state.update_count,
             (unsigned long)m_state.error_count,
             (m_state.status & AS5600_STATUS_MAGNET_DETECTED) ? "detected" : "missing",
             (m_state.status & AS5600_STATUS_MAGNET_TOO_WEAK) ? ", weak" : "",
             (m_state.status & AS5600_STATUS_MAGNET_TOO_STRONG) ? ", strong" : "");
    return info;
}

static THD_FUNCTION(as5600_thread, arg)
{
    (void)arg;
    chRegSetThreadName("AS5600");

    systime_t next = chVTGetSystemTimeX();
    uint32_t status_count = 0;

    while (!m_state.stop_now)
    {
        read_angle();

        status_count++;
        if (status_count >= AS5600_STATUS_INTERVAL)
        {
            status_count = 0;
            read_status();
        }

        systime_t previous = next;
        next += CH_CFG_ST_FREQUENCY / AS5600_SAMPLE_RATE_HZ;
        systime_t now = chVTGetSystemTimeX();
        if (!chVTIsTimeWithinX(now, previous, next))
        {
            next = now + CH_CFG_ST_FREQUENCY / AS5600_SAMPLE_RATE_HZ;
        }
        chThdSleepUntil(next);
    }

    m_state.is_running = false;
}
