#ifndef _CMD_H_
#define _CMD_H_

#include "board.h"

/*************************************************************************************************
 *                 COMMUNICATION COMMAND DESCRIPTION
 *
 * Command format:
 *
 *      [B0]    [B1]       [B2]        [B3]
 *      [cmd]   [param 0]  [param 1]   [param 2]
 *
 * Command list:
 *
 * [idx][cmd]   [param 0]  [param 1]   [param 2]
 * --------------------------------------------------------------------------------------------
 *  [0] 0x00     0x00     0x00        0x00      : NOP
 * --------------------------------------------------------------------------------------------
 *  [0] 0x01     0x00     0x00        0x00      : Reset INA229
 *  [0] 0x01     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x02     CT       AVG         0x00      : Set VBUSCT & VSHCT conversion time(CT), AVG count
 *  [0] 0x02     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x03     ADCR     0x00        0x00      : Set ADCRANGE 0/1
 *  [0] 0x03     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x04     ACOMP    0x00        0x00      : Set ALERT comparison on the averaged value (ACOMP = 0/1)
 *  [0] 0x04     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x05     0x00     0x00        0x00      : Configure the INA229 with the above parameters
 *  [0] 0x05     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x06     VAL_H    VAL_L       0x00      : Set battery simulator volatge
 *  [0] 0x06     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x07     0x01     0x00        0x00      : Battery simulator volatge output enable
 *  [0] 0x07     0x00     0x00        0x00      : Battery simulator volatge output disable
 *  [0] 0x07     0x0/1    0x00        0x00      : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x08     0x00     0x00        0x00      : Start measuring
 *                                              : No response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x09     0x00     0x00        0x00      : Stop measuring
 *                                              : No response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x0A     LEN_H    LEN_L       0x00      : Data streaming report (LEN = 2048 bytes)
 *  [1] V[3]     V[2]     V[1]        V[0]      : Voltage data [V] (first half is voltage)
 *  [2] V[3]     V[2]     V[1]        V[0]      :
 *  .....................................       :
 *  [n] I[3]     I[2]     I[1]        I[0]      : Current [mA] (second half is current)
 *  [m] I[3]     I[2]     I[1]        I[0]      :
 * --------------------------------------------------------------------------------------------
 *
 *************************************************************************************************/

typedef struct {
    uint8_t cmd;
    uint8_t param_1;
    uint8_t param_2;
    uint8_t param_3;
} cmd_t;

typedef struct {
    uint8_t response;
    uint8_t result;
    uint8_t reserve_1;
    uint8_t reserve_2;
} response_t;

typedef enum {
    CMD_NOP = 0,
    CMD_RESET_INA229 = 0x01,
    CMD_CONVERSION_TIME = 0x02,
    CMD_SET_ADCRANGE = 0x03,
    CMD_SET_ALERT_COMP_AVG = 0x04,
    CMD_CONFIG_INA229 = 0x05,
    CMD_SET_BAT_SIM_VOLT = 0x06,
    CMD_BAT_SIM_OUTPUT = 0x07,
    CMD_START_MESURE = 0x08,
    CMD_STOP_MESURE = 0x09
} cmd_code_t;

void cmd_process(uint8_t *cmd_buff, uint32_t len);

#endif /* _CMD_H_ */
