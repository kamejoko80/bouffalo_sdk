/*
 * Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#ifndef _TCA9534_H_
#define _TCA9534_H_

#include "board.h"

#define I_PORT 0x00 // Input port
#define O_PORT 0x01 // Output port
#define P_PORT 0x02 // Polarity inversion
#define C_PORT 0x03 // Configuration

typedef enum {
    FPGA_MODE0 = 0,
    FPGA_VDDIO,
    FPGA_VCORE,
    VOL_MEASURE,
    BAT_SIM_ENA,
    FPGA_OSC25M_ENA
} power_ctrl_t;

void tca9534_init(void);
void tca9534_pin_control(power_ctrl_t ctrl, uint8_t set);

#endif /* _TCA9534_H_ */