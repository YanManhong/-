#ifndef INA219AIDCNR_HELPER_H
#define INA219AIDCNR_HELPER_H

#include "pinctrl.h"
#include "soc_osal.h"
#include "i2c.h"
#include "osal_debug.h"
#include "cmsis_os2.h"
#include "app_init.h"
#include <stdio.h>
#include "gpio.h"
#include "hal_gpio.h"

#define INA219_ADDRESS                             (0x40)
#define MAX_SHUNT_RANGE                            (0x0FA0)

/* Register */
#define INA219_REG_CONFIG                          (0x00)
#define INA219_REG_SHUNT_VOLTAGE                   (0x01)
#define INA219_REG_BUS_VOLTAGE                     (0x02)
#define INA219_REG_POWER                           (0x03)
#define INA219_REG_CURRENT                         (0x04)
#define INA219_REG_CALIBRATION                     (0x05)

#define INA219_CONFIG_RESET                        (0x8000)
#define INA219_CONFIG_VOLTAGE_RANGE_16V            (0x0000)
#define INA219_CONFIG_VOLTAGE_RANGE_32V            (0x2000)

#define INA219_CONFIG_GAIN_1_40MV                  (0x0000)
#define INA219_CONFIG_GAIN_2_80MV                  (0x0800)
#define INA219_CONFIG_GAIN_4_160MV                 (0x1000)
#define INA219_CONFIG_GAIN_8_320MV                 (0x1800)

#define INA219_CONFIG_BADCRES_9BIT                 (0x0000)
#define INA219_CONFIG_BADCRES_10BIT                (0x0080)
#define INA219_CONFIG_BADCRES_11BIT                (0x0100)
#define INA219_CONFIG_BADCRES_12BIT                (0x0180)
#define INA219_CONFIG_BADCRES_12BIT_2S_1060US      (0x0480)
#define INA219_CONFIG_BADCRES_12BIT_4S_2130US      (0x0500)
#define INA219_CONFIG_BADCRES_12BIT_8S_4260US      (0x0580)
#define INA219_CONFIG_BADCRES_12BIT_16S_8510US     (0x0600)
#define INA219_CONFIG_BADCRES_12BIT_32S_17MS       (0x0680)
#define INA219_CONFIG_BADCRES_12BIT_64S_34MS       (0x0700)
#define INA219_CONFIG_BADCRES_12BIT_128S_69MS      (0x0780)

#define INA219_CONFIG_SADCRES_9BIT_1S_84US         (0x0000)
#define INA219_CONFIG_SADCRES_10BIT_1S_148US       (0x0008)
#define INA219_CONFIG_SADCRES_11BIT_1S_276US       (0x0010)
#define INA219_CONFIG_SADCRES_12BIT_1S_532US       (0x0018)
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US      (0x0048)
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US      (0x0050)
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US      (0x0058)
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US     (0x0060)
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS       (0x0068)
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS       (0x0070)
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS      (0x0078)

#define INA219_CONFIG_MODE_MASK                    0x07
#define INA219_CONFIG_MODE_POWERDOWN               0x00
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED         0x01
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED         0x02
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED     0x03
#define INA219_CONFIG_MODE_ADCOFF                  0x04
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS        0x05
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS        0x06
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS    0x07

typedef struct {
    uint32_t bus;        // I2C 总线编号
    uint16_t address;    // 设备地址
} INA219_t;

uint8_t INA219_Init(INA219_t *ina219, uint32_t bus, uint16_t address);
uint16_t INA219_ReadBusVoltage(INA219_t *ina219);
uint16_t INA219_ReadCurrent_mA(INA219_t *ina219);
uint16_t INA219_ReadCurrent_raw(INA219_t *ina219);
uint16_t INA219_ReadShuntVoltage_mV(INA219_t *ina219);
uint16_t INA219_ReadDataForRegister_16Bits(INA219_t *ina219, uint8_t registerAddress);
uint16_t INA219_GetConfigInfo(INA219_t *ina219);

void INA219_Reset(INA219_t *ina219);
void INA219_SetCalibration(INA219_t *ina219, uint16_t calibrationData);
void INA219_SetConfig(INA219_t *ina219, uint16_t configData);
void INA219_SetCalibration_16V_8A(INA219_t *ina219);
void INA219_WriteDataToRegister_16Bits(INA219_t *ina219, uint8_t registerAddress, uint16_t value);

#endif // INA219AIDCNR_HELPER_H
