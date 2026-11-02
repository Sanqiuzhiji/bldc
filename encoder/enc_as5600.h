/*
	Copyright 2026

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
*/

#ifndef ENC_AS5600_H_
#define ENC_AS5600_H_

#include <stdbool.h>

// 启动I2C、验证AS5600通信并创建后台角度采样线程。
bool enc_as5600_init(void);

// 停止采样线程并释放AS5600使用的I2C资源。
void enc_as5600_deinit(void);

// 返回采样线程最近一次缓存的机械角度，单位为度。
float enc_as5600_read_deg(void);

// 检查通信超时、连续读取错误以及磁铁状态。
bool enc_as5600_has_fault(void);

// 返回供终端命令显示的AS5600诊断信息。
char *enc_as5600_print_info(void);

#endif /* ENC_AS5600_H_ */
