#include "bflb_gpio.h"
#include "bflb_i2c.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "ina229.h"
#include "cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern uint8_t *p_wr_buf;
extern ina229_config_t ina229_config;

extern void cdc_acm_data_send(uint32_t len);

void cmd_process(uint8_t *cmd_buff, uint32_t len)
{
    cmd_t *p_cmd = NULL;
    response_t *resp = (response_t *)p_wr_buf;

    if(len != sizeof(cmd_t))
    {
        printf("Invalid command len\r\n");
        return;
    }

    p_cmd = (cmd_t *)cmd_buff;
    resp->response = p_cmd->cmd;

    switch (p_cmd->cmd) {
        case CMD_NOP:
            printf("CMD nop\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_RESET_INA229:
            printf("CMD reset ina229\r\n");
            ina229_reset();
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_CONVERSION_TIME:
            printf("CMD set conversion time\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_SET_ADCRANGE:
            printf("CMD set ADC range\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_SET_ALERT_COMP_AVG:
            printf("CMD set alert comparison on the AVG value\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_CONFIG_INA229:
            printf("CMD configure the ina229\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_SET_BAT_SIM_VOLT:
            printf("CMD set battery simulator voltage output\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_BAT_SIM_OUTPUT:
            printf("CMD set bat sim voltage output on/off\r\n");
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_START_MESURE:
            printf("CMD start measuring\r\n");
            ina229_start_measure(&ina229_config);
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        case CMD_STOP_MESURE:
            printf("CMD stop measuring\r\n");
            ina229_stop_measure(&ina229_config);
            resp->result = 1;
            cdc_acm_data_send(sizeof(response_t));
            break;
        default:
            printf("Unknown cmd type\r\n");
            resp->result = 0;
            cdc_acm_data_send(sizeof(response_t));
            break;
    }
}