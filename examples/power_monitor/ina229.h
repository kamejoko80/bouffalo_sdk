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

#define ADC_VCC (4.75)
#define RSHUNT  (0.05)

/* LSB in case ADCRANGE = 0 */
#define CURRENT_LSB_0 (0.00000625)
#define VSHUNT_LSB_0  (0.0000003125)
#define VBUS_LSB_0    (0.0001953125)

/* LSB in case ADCRANGE = 1 */
#define CURRENT_LSB_1 (0.0000015625)
#define VSHUNT_LSB_1  (0.000000078125)
#define VBUS_LSB_1    (0.0001953125)

/* Data report sample number */
#define DATA_RPT_SAMPLE_SIZE (1024/4)

typedef enum {
    CONV_TIME_280uS  = 0x3,
    CONV_TIME_540uS  = 0x4,
    CONV_TIME_1052uS = 0x5,
    CONV_TIME_2074uS = 0x6,
    CONV_TIME_4120uS = 0x7
} ina229_conv_time_t;

typedef enum {
    AVG_NUM_1    = 0x00,
    AVG_NUM_4    = 0x01,
    AVG_NUM_16   = 0x02,
    AVG_NUM_64   = 0x03,
    AVG_NUM_128  = 0x04,
    AVG_NUM_256  = 0x05,
    AVG_NUM_512  = 0x06,
    AVG_NUM_1024 = 0x07
} ina229_avg_num_t;

typedef enum {
    ADC_RANGE_0  = 0x00, /* vshunt range: ±163.84 mV */
    ADC_RANGE_1  = 0x01  /* vshunt range: ±40.96 mV  */
} ina229_adc_range_t;

typedef enum {
    AVG_ALERT_NO  = 0x00,
    AVG_ALERT_YES = 0x01
} ina229_avg_alert_t;

typedef struct {
    uint8_t cnv_time;
    uint8_t avg_num;
    uint8_t adc_range;
    uint8_t avg_alert;
} ina229_config_t;

typedef struct {
    float vcc;    /* ADC VCC voltage [volts] */
    float rshunt; /* Rhunt value [Ω] */
} ina229_hw_param_t;

typedef struct {
    float current_lsb;
    float vshunt_lsb;
    float vbus_lsb;
} ina229_lsb_param_t;

typedef struct {
   uint32_t sign;
   uint64_t id;
   int32_t voltage[DATA_RPT_SAMPLE_SIZE]; /* Voltage [V] */
   int32_t current[DATA_RPT_SAMPLE_SIZE]; /* Current [mA] */
} ina229_data_report_t;

void ina229_reset(void);
void ina229_init(void);
void ina229_interface_bus_init(void);
void ina229_enable_alert_interrupt(void);
void ina229_disable_alert_interrupt(void);
void ina229_param_config(ina229_config_t *config);
void ina229_start_measure(void);
void ina229_stop_measure(void);

#endif /* _INA229_H_ */