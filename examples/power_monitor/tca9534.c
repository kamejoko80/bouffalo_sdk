/*
 * Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "tca9534.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static struct bflb_device_s *i2c0;

#define DEVICE_ADDR 0x38 // A0 = A1 = A2 = 0

void i2c0_init(void)
{
    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 400000);
}

uint8_t tca9534_read_reg(uint8_t addr)
{
    struct bflb_i2c_msg_s msgs[2];
    uint8_t reg_addr = addr;
    uint8_t reg_value = 0;

    msgs[0].addr = DEVICE_ADDR;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = &reg_addr;
    msgs[0].length = 1;

    msgs[1].addr = DEVICE_ADDR;
    msgs[1].flags = I2C_M_READ;
    msgs[1].buffer = &reg_value;
    msgs[1].length = 1;

    bflb_i2c_transfer(i2c0, msgs, 2);

    return reg_value;
}

void tca9534_write_reg(uint8_t addr, uint8_t value)
{
    struct bflb_i2c_msg_s msgs[1];
    uint8_t addr_value[2];

    addr_value[0] = addr;
    addr_value[1] = value;

    msgs[0].addr = DEVICE_ADDR;
    msgs[0].flags = I2C_M_WRITE;
    msgs[0].buffer = addr_value;
    msgs[0].length = 2;

    bflb_i2c_transfer(i2c0, msgs, 1);
}

void tca9534_init(void)
{
    i2c0_init();

    /* before port output configuration,
       set the default output value first
       P7(NA)                = 0 // Not use -> set input
       P6(NA)                = 0 // Not use -> set input
       P5(25MHZ_ENA)         = 0 // Not use -> set input
       P4(BAT_SIM_ENA)       = 0
       P3(VOL_MEASURE_ENA_N) = 1
       P2(FPGA_VCORE_ENA)    = 0
       P1(FPGA_VDDIO_ENA_N)  = 1
       P0(FPGA_MODE0)        = 0
    */

    tca9534_write_reg(O_PORT, 0x0A);
    tca9534_write_reg(C_PORT, 0xE0);

/*  uint8_t value;
    value = tca9534_read_reg(0);
    printf("Reg 0 = %X\r\n", value);
    value = tca9534_read_reg(1);
    printf("Reg 1 = %X\r\n", value);
    value = tca9534_read_reg(2);
    printf("Reg 2 = %X\r\n", value);
    value = tca9534_read_reg(3);
    printf("Reg 3 = %X\r\n", value); */
}

void tca9534_pin_control(power_ctrl_t ctrl, uint8_t set)
{
    uint8_t value;
    value = tca9534_read_reg(O_PORT);

    /* check for inverted polorization */
    if((ctrl == FPGA_VDDIO) || (ctrl == VOL_MEASURE))
    {
        if(set)
        {
            value &= ~(1<<ctrl);
        }
        else
        {
            value |= 1<<ctrl;
        }
    }
    else
    {
        if(set)
        {
            value |= 1<<ctrl;
        }
        else
        {
            value &= ~(1<<ctrl);
        }
    }

    tca9534_write_reg(O_PORT, value);
}
