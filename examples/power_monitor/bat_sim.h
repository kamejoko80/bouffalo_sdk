/*
 * Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BAT_SIM_H_
#define _BAT_SIM_H_

#include "board.h"
#include "ina229.h"

/*
 * Vout = DAC_VCC x data / 4096
 * Where DAC_VCC ~ 4.75 volts (form USB cable)
 * data is in range [0 - 4095]
 * we limit the output voltage is 4.2 volts
 *
 * DATA_MAX_4P2 = 4096 * 4.2 / DAC_VCC
 */

#define DATA_MAX_4P2 (3622)
#define DATA_3P8     (3350)

void bat_sim_read_config_data_code_epprom(void);
void bat_sim_fast_mode_write(uint16_t data);

#endif /* _BAT_SIM_H_ */