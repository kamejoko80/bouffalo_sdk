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

void ina229_init(void);
void ina229_enable_alert_interrupt(void);
void ina229_disable_alert_interrupt(void);

#endif /* _INA229_H_ */