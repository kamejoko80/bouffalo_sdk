/*
 * Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "bat_sim.h"
#include "ina229.h"
#include "tca9534.h"
#include "cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern uint8_t *p_wr_buf;
extern ina229_config_t ina229_config;
extern ina229_lsb_param_t ina229_lsb;

extern void cdc_acm_cmd_response_send(void);
extern void status_led_on(void);
extern void status_led_off(void);

bool check_ina229_config_param(cmd_t *p_cmd)
{
    if((p_cmd->config.cnv_time < CONV_TIME_280uS) || (p_cmd->config.cnv_time > CONV_TIME_4120uS))
    {
        printf("Invalid ina229 conversion time param\r\n");
        return false;
    }

    if(p_cmd->config.avg_num > AVG_NUM_1024)
    {
        printf("Invalid ina229 average number param\r\n");
        return false;
    }

    if(p_cmd->config.adc_range > ADC_RANGE_1)
    {
        printf("Invalid ina229 adc range param\r\n");
        return false;
    }

    if(p_cmd->config.avg_alert > AVG_ALERT_YES)
    {
        printf("Invalid ina229 alert on AVG param\r\n");
        return false;
    }

    return true;
}

void cmd_process(uint8_t *cmd_buff, uint32_t len)
{
    cmd_t *p_cmd = NULL;
    response_t *resp = (response_t *)p_wr_buf;

    if(len < 4)
    {
        printf("Invalid command len\r\n");
        return;
    }

    p_cmd = (cmd_t *)cmd_buff;
    memset((void *)resp, 0, sizeof(response_t));
    resp->response = p_cmd->cmd;

    switch (p_cmd->cmd)
    {
        case CMD_NOP:
            printf("CMD nop\r\n");
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        case CMD_RESET_INA229:
            printf("CMD reset ina229\r\n");
            ina229_reset();
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        case CMD_WRITE_CONFIG_PARAM:
            printf("CMD write ina229 config params\r\n");
            if(len < 8)
            {
                printf("Invalid command len\r\n");
            }
            else if(check_ina229_config_param(p_cmd))
            {
                resp->result = 1;
                resp->config = p_cmd->config;
                resp->hw_config.vcc = DAC_VCC;
                resp->hw_config.rshunt = RSHUNT;
                ina229_config = p_cmd->config;
            }
            else
            {
                resp->result = 0;
            }
            cdc_acm_cmd_response_send();
            break;
        case CMD_READ_CONFIG_PARAM:
            printf("CMD read ina229 config params\r\n");
            resp->result = 1;
            resp->config = ina229_config;
            resp->hw_config.vcc = DAC_VCC;
            resp->hw_config.rshunt = RSHUNT;
            cdc_acm_cmd_response_send();
            break;
        case CMD_CONFIGURE_INA229:
            printf("CMD configure the ina229\r\n");
            ina229_param_config(&ina229_config);
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        case CMD_SET_BAT_SIM_VOLT:
            printf("CMD set battery simulator voltage output\r\n");
            bat_sim_fast_mode_write((uint16_t)((p_cmd->param_1 <<8) | p_cmd->param_2));
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        case CMD_BAT_SIM_OUTPUT:
            if(p_cmd->param_1 == 0x01)
            {
                printf("CMD set bat sim voltage output on\r\n");
                status_led_on();
                tca9534_pin_control(BAT_SIM_ENA, 1);
                resp->result = 1;
            }
            else if (p_cmd->param_1 == 0x00)
            {
                printf("CMD set bat sim voltage output off\r\n");
                status_led_off();
                tca9534_pin_control(BAT_SIM_ENA, 0);
                resp->result = 1;
            }
            else
            {
                printf("CMD set bat sim voltage output is invalid\r\n");
            }
            cdc_acm_cmd_response_send();
            break;
        case CMD_START_MEASURE:
            printf("CMD start measuring\r\n");
            ina229_start_measure();
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        case CMD_STOP_MEASURE:
            printf("CMD stop measuring\r\n");
            ina229_stop_measure();
            resp->result = 1;
            cdc_acm_cmd_response_send();
            break;
        default:
            printf("Unknown cmd type\r\n");
            resp->result = 0;
            cdc_acm_cmd_response_send();
            break;
    }
}