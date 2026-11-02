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

// AS5600使用固定的7位I2C地址。角度由0x0C、0x0D两个连续寄存器组成，
// 状态寄存器用于检查磁铁是否存在以及磁场强度是否合适。
#define AS5600_ADDRESS                  0x36
#define AS5600_REG_STATUS               0x0B
#define AS5600_REG_RAW_ANGLE_H          0x0C

#define AS5600_STATUS_MAGNET_DETECTED   (1U << 5)
#define AS5600_STATUS_MAGNET_TOO_WEAK   (1U << 4)
#define AS5600_STATUS_MAGNET_TOO_STRONG (1U << 3)

// 角度以1 kHz刷新；磁铁状态变化较慢，因此每100个角度样本检查一次。
// 连续通信失败和数据长时间未更新都会被上层视为编码器故障。
#define AS5600_SAMPLE_RATE_HZ           1000
#define AS5600_STATUS_INTERVAL          100
#define AS5600_I2C_TIMEOUT_MS           2
#define AS5600_MAX_CONSECUTIVE_ERRORS   5
#define AS5600_DATA_TIMEOUT_MS          10

// 采样线程与电机控制线程共享这些状态，因此使用volatile保证每次都从内存读取。
// angle_deg是供电机控制使用的机械角度，raw_angle和计数器用于终端诊断。
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

// 在一次I2C事务中先写入寄存器地址，再从该地址开始连续读取len个字节。
// 总线锁保证AS5600访问不会与系统中的其他I2C事务交叉。
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

    // RAW ANGLE是12位数值：高寄存器的低4位为bit[11:8]，低寄存器为bit[7:0]。
    // 4096个量化位置对应一整圈，将原始值换算为0~360度的机械角度。
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

    // 启动采样线程前先完成一次角度和状态读取，确认器件能够正常响应，
    // 避免将尚未初始化的数据暴露给电机控制逻辑。
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
    // 先通知采样线程退出并等待其释放I2C总线，再关闭I2C外设。
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
    // 数据无效、连续通信失败或数据过期都说明当前角度不再可信。
    if (!m_state.valid ||
        m_state.consecutive_errors >= AS5600_MAX_CONSECUTIVE_ERRORS)
    {
        return true;
    }

    if (chVTTimeElapsedSinceX(m_state.last_update) > MS2ST(AS5600_DATA_TIMEOUT_MS))
    {
        return true;
    }

    // AS5600能够通信并不代表磁铁安装正确，还需要检查磁铁存在和磁场强度标志。
    return !(m_state.status & AS5600_STATUS_MAGNET_DETECTED) ||
    (m_state.status & (AS5600_STATUS_MAGNET_TOO_WEAK |
        AS5600_STATUS_MAGNET_TOO_STRONG));
}

char* enc_as5600_print_info(void)
{
    // 返回一份静态诊断文本，供encoder终端命令打印当前角度、通信和磁铁状态。
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
        // 每个周期读取角度；每AS5600_STATUS_INTERVAL个周期额外读取一次状态。
        read_angle();

        status_count++;
        if (status_count >= AS5600_STATUS_INTERVAL)
        {
            status_count = 0;
            read_status();
        }

        // 使用绝对唤醒时间减少周期漂移。如果本次I2C访问已经错过目标周期，
        // 则以当前时间为基准重新安排下一次采样。
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
