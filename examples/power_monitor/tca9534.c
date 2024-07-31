#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "tca9534.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern struct bflb_device_s *i2c0;
extern void cdc_acm_printf(const char *format, ...);

#define DEVICE_ADDR 0x20 // A0 = A1 = A2 = 0

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

void tca9534_test(void)
{
    uint8_t value;

    value = tca9534_read_reg(0);
    cdc_acm_printf("Reg 0 = %X\r\n", value);
    value = tca9534_read_reg(1);
    cdc_acm_printf("Reg 1 = %X\r\n", value);
    value = tca9534_read_reg(2);
    cdc_acm_printf("Reg 2 = %X\r\n", value);
}
