#ifndef _CMD_H_
#define _CMD_H_

#include "board.h"
#include "ina229.h"

/*************************************************************************************************
 *                 COMMUNICATION COMMAND DESCRIPTION
 *
 * Command format:
 *
 *      [B0]     [B1]        [B2]        [B3]
 *      [cmd]    [param 0]   [param 1]   [param 2]
 *
 * Command list:
 *
 * [idx][cmd]    [param 0]   [param 1]   [param 2]
 * --------------------------------------------------------------------------------------------
 *  [0] 0x00      0x00       0x00        0x00         : NOP
 * --------------------------------------------------------------------------------------------
 *  [0] 0x01      0x00       0x00        0x00         : Reset INA229
 *  [0] 0x01      0x0/1      0x00        0x00         : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x02      0x00       0x00        0x00         : Write INA229 config params
 *  [1] cnv_time  avg_num    adc_range   avg_alert    : Config params
 *  [0] 0x02      0x0/1      0x00        0x00         : Response
 *  [1] cnv_time  avg_num    adc_range   avg_alert    : INA229 config param return
 *  [2] vcc[0]    vcc[1]     vcc[2]      vcc[3]       : ADC VCC param return [V]
 *  [3] rsh[0]    rsh[1]     rsh[2]      rsh[3]       : Rshunt param return [Ω]
 * --------------------------------------------------------------------------------------------
 *  [0] 0x03      0x00       0x00        0x00         : Read INA229's config params
 *  [0] 0x03      0x0/1      0x00        0x00         : Response result 0/1
 *  [1] cnv_time  avg_num    adc_range   avg_alert    : INA229 config param return
 *  [2] vcc[0]    vcc[1]     vcc[2]      vcc[3]       : ADC VCC param return [V]
 *  [3] rsh[0]    rsh[1]     rsh[2]      rsh[3]       : Rshunt param return [Ω]
 * --------------------------------------------------------------------------------------------
 *  [0] 0x04      0x00       0x00        0x00         : Configure the INA229 with the above parameters
 *  [0] 0x04      0x0/1      0x00        0x00         : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x05      VAL_L      VAL_H       0x00         : Set battery simulator volatge
 *  [0] 0x05      0x0/1      0x00        0x00         : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x06      0x01       0x00        0x00         : Battery simulator volatge output enable
 *  [0] 0x06      0x00       0x00        0x00         : Battery simulator volatge output disable
 *  [0] 0x06      0x0/1      0x00        0x00         : Response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x07      0x00       0x00        0x00         : Start measuring
 *                                                    : No response
 * --------------------------------------------------------------------------------------------
 *  [0] 0x08      0x00       0x00        0x00         : Stop measuring
 *                                                    : No response
 * --------------------------------------------------------------------------------------------
 *  [0] sign[0]   sign[1]    sign[2]     sign[3]      : Data streaming report (LEN = 2048 bytes)
 *  [1] idl[0]    idl[1]     idl[2]      idl[3]       : ID low word
 *  [2] idh[0]    idh[1]     idh[2]      idh[3]       : ID high word
 *  [3] v[0]      v[1]       v[2]        v[3]         : Voltage data [V] (first half is voltage)
 *  [4] v[0]      v[1]       v[2]        v[3]         :
 *  .........................................         :
 *  [n] i[0]      i[1]       i[2]        i[3]         : Current [mA] (second half is current)
 *  [m] i[0]      i[1]       i[2]        i[3]         :
 * --------------------------------------------------------------------------------------------
 *
 *************************************************************************************************/

typedef struct {
    uint8_t cmd;
    uint8_t param_1;
    uint8_t param_2;
    uint8_t param_3;
    ina229_config_t config;
} cmd_t;

typedef struct {
    uint8_t response;
    uint8_t result;
    uint8_t reserve_1;
    uint8_t reserve_2;
    ina229_config_t config;
    ina229_hw_param_t hw_config;
} response_t;

typedef enum {
    CMD_NOP                = 0x00,
    CMD_RESET_INA229       = 0x01,
    CMD_WRITE_CONFIG_PARAM = 0x02,
    CMD_READ_CONFIG_PARAM  = 0x03,
    CMD_CONFIGURE_INA229   = 0x04,
    CMD_SET_BAT_SIM_VOLT   = 0x05,
    CMD_BAT_SIM_OUTPUT     = 0x06,
    CMD_START_MEASURE      = 0x07,
    CMD_STOP_MEASURE       = 0x08
} cmd_code_t;

void cmd_process(uint8_t *cmd_buff, uint32_t len);

#endif /* _CMD_H_ */
