#ifndef _INA229_H_
#define _INA229_H_

#include "board.h"

#define CONFIG             (0x00)
#define ADC_CONFIG         (0x01)
#define SHUNT_CAL          (0x02)
#define SHUNT_TEMPCO       (0x03)
#define VSHUNT             (0x04)
#define VBUS               (0x05)
#define DIETEMP            (0x06)
#define CURRENT            (0x07)
#define POWER              (0x08)
#define ENERGY             (0x09)
#define CHARGE             (0x0A)
#define DIAG_ALRT          (0x0B)
#define SOVL               (0x0C)
#define SUVL               (0x0D)
#define BOVL               (0x0E)
#define BUVL               (0x0F)
#define TEMP_LIMIT         (0x10)
#define PWR_LIMIT          (0x11)
#define MANUFACTURER_ID    (0x3E)
#define DEVICE_ID          (0x3F)

/*
 * Full scale ranges:
 *    Shunt voltage:
 *        ±163.84 mV (ADCRANGE = 0)  312.5 nV/LSB
 *        ±40.96 mV  (ADCRANGE = 1) 78.125 nV/LSB
 *    Bus voltage:
 *        0V to 85V 195.3125 µV/LSB
 *    Temperature:
 *        –40°C to +125°C 7.8125 m°C/LSB
 *
 *    SHUNT_CAL = 13107.2 x 10^6 x CURRENT_LSB x RSHUNT
 *    the value of SHUNT_CAL must be multiplied by 4 for ADCRANGE = 1
 *    CURRENT_LSB = Maximum Expected Current / 2^19
 *
 *    Current [A] = CURRENT_LSB x CURRENT
 *    CURRENT is the value read from the CURRENT register
 *
 *    Power [W] = 3.2 x CURRENT_LSB x POWER
 *    POWER is the value read from the POWER register
 *
 *    Energy [J] = 16 x 3.2 x CURRENT_LSB x ENERGY
 *    Charge [C] = CURRENT_LSB x CHARGE
 *
 *    In case ADCRANGE = 0:
 *        Imax = 163.84mV / 50mΩ = 3.2768A
 *        CURRENT_LSB = 3.2768 / 2^19 = 0.00000625A = 6.25µA
 *        SHUNT_CAL = 13107.2 x 10^6 x 0.00000625 x 0.05 = 4096
 *
 *    In case ADCRANGE = 1:
 *        Imax = 40.96mV / 50mΩ = 0.8192A
 *        CURRENT_LSB = 0.8192 / 2^19 = 0.0000015625A = 1.5625µA
 *        SHUNT_CAL = (13107.2 x 10^6 x 0.0000015625 x 0.05) x 4 = 4096
 *
 */

/* LSB in case ADCRANGE = 0 */
#define CURRENT_LSB_1 0.00000625
#define VSHUNT_LSB_1  0.0000003125
#define VBUS_LSB_1    0.0001953125

void ina229_init(void);
void ina229_enable_alert_interrupt(void);
void ina229_disable_alert_interrupt(void);

#endif /* _INA229_H_ */