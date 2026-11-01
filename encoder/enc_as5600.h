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

bool enc_as5600_init(void);
void enc_as5600_deinit(void);
float enc_as5600_read_deg(void);
bool enc_as5600_has_fault(void);
char *enc_as5600_print_info(void);

#endif /* ENC_AS5600_H_ */
