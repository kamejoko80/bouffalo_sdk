#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "tca9534.h"
#include "bat_sim.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static struct bflb_device_s *i2c0;
extern void cdc_acm_printf(const char *format, ...);

#define DEVICE_ADDR 0x60 // A0 = A1 = A2 = 0

void bat_sim_read_config_data_code_epprom(void)
{
    struct bflb_i2c_msg_s msgs[1];
    uint8_t p_rx[5] = {0, 0, 0, 0, 0};
    uint16_t temp;

    i2c0 = bflb_device_get_by_name("i2c0");

    msgs[0].addr = DEVICE_ADDR;
    msgs[0].flags = I2C_M_READ;
    msgs[0].buffer = p_rx;
    msgs[0].length = 5;

    bflb_i2c_transfer(i2c0, msgs, 1);

    printf("Config data:\r\n");
    printf("RDY     = %X\r\n", (p_rx[0]>>7));
    printf("POR     = %X\r\n", (p_rx[0]>>6)&0x01);
    printf("PD1/PD0 = %X\r\n", (p_rx[0]>>1)&0x03);
    printf("DAC data:\r\n");
    temp = (uint16_t)((p_rx[1] << 8) | p_rx[2]);
    temp = temp >> 4;
    printf("DAC_H   = %X\r\n", (temp >> 8) & 0xFF);
    printf("DAC_L   = %X\r\n", (temp & 0xFF));
    printf("EEPROM data:\r\n");
    printf("PD1/PD0 = %X\r\n", (p_rx[3]>>5)&0x03);
    printf("EPR_H   = %X\r\n", (p_rx[3] | 0x0F));
    printf("EPR_L   = %X\r\n", (p_rx[4]));
}

void bat_sim_fast_mode_write(uint16_t data)
{
    struct bflb_i2c_msg_s msgs[1];
    uint8_t p_tx[2];
    uint16_t value;

    /* we limit the output voltage at 4.2 volts */
    if(data >= DATA_MAX_4P2)
    {
        value = DATA_MAX_4P2;
    }
    else
    {
        value = data;
    }

    i2c0 = bflb_device_get_by_name("i2c0");
    p_tx[0] = (uint8_t)((value >> 8) & 0x0F);
    p_tx[1] = (uint8_t)value;

    msgs[0].addr = DEVICE_ADDR;
    msgs[0].flags = I2C_M_WRITE;
    msgs[0].buffer = p_tx;
    msgs[0].length = 2;

    printf("Set voltage output: %f\r\n", (value * VCC)/4096);

    bflb_i2c_transfer(i2c0, msgs, 1);
}