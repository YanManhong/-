#include "INA219.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "i2c.h"
#include "osal_debug.h"
#include "cmsis_os2.h"
#include "app_init.h"
#include <stdio.h>
#include "gpio.h"
#include "hal_gpio.h"

uint16_t ina219_calibrationValue;
uint16_t ina219_currentDivider_mA;
float ina219_powerMultiplier_mW;

// 读取寄存器（16位）
uint16_t INA219_ReadDataForRegister_16Bits(INA219_t *ina219, uint8_t regAddr)
{
    uint8_t recvBuf[2] = {0};
    i2c_data_t data = {
        .send_buf = &regAddr,
        .send_len = 1,
        .receive_buf = recvBuf,
        .receive_len = 2
    };

    errcode_t ret = uapi_i2c_master_writeread(ina219->bus, ina219->address, &data);
    if (ret != 0) {
        osal_printk("INA219: I2C read failed: reg 0x%02X\n", regAddr);
        return 0xFFFF;
    }
    return (recvBuf[0] << 8) | recvBuf[1];
}

// 写寄存器（16位）
void INA219_WriteDataToRegister_16Bits(INA219_t *ina219, uint8_t regAddr, uint16_t value)
{
    uint8_t buf[3] = {regAddr, (uint8_t)(value >> 8), (uint8_t)value};
    i2c_data_t data = {
        .send_buf = buf,
        .send_len = 3,
        .receive_buf = NULL,
        .receive_len = 0
    };

    errcode_t ret = uapi_i2c_master_write(ina219->bus, ina219->address, &data);
    if (ret != 0) {
        osal_printk("INA219: I2C write failed: reg 0x%02X\n", regAddr);
    }
}

// 读取总线电压（单位：mV）
uint16_t INA219_ReadBusVoltage(INA219_t *ina219)
{
    uint16_t raw = INA219_ReadDataForRegister_16Bits(ina219, INA219_REG_BUS_VOLTAGE);
    return (raw >> 3) * 4;  // LSB = 4mV
}

// 读取原始电流值（无缩放）
uint16_t INA219_ReadCurrent_raw(INA219_t *ina219)
{
    return INA219_ReadDataForRegister_16Bits(ina219, INA219_REG_CURRENT);
}

// 读取实际电流（单位：mA）
// 注意：这里改用int16_t以处理负电流

    uint16_t INA219_ReadCurrent_mA(INA219_t *ina219)
{
    int16_t raw = (int16_t)INA219_ReadCurrent_raw(ina219);
    if (ina219_currentDivider_mA <= 0) {
        osal_printk("Error: ina219_currentDivider_mA invalid: %d\n", ina219_currentDivider_mA);
        return 0;
    }
    if (raw < 0) {
        return 0; // 负电流直接返回0
    }
    uint16_t current_mA = (uint16_t)(raw / ina219_currentDivider_mA);
    if (current_mA > 2000) {  // 合理上限
        osal_printk("Warning: current value too large: %u\n", current_mA);
        return 0;
    }
    return current_mA;
}



    

// 读取分流电压（单位：mV）
uint16_t INA219_ReadShuntVoltage_mV(INA219_t *ina219)
{
    int16_t raw = (int16_t)INA219_ReadDataForRegister_16Bits(ina219, INA219_REG_SHUNT_VOLTAGE);
    // 这里直接转换为带符号整数，按 INA219 数据手册
    return raw / 100;  // 每位 = 10uV，除以100变为mV
}

// 复位
void INA219_Reset(INA219_t *ina219)
{
    INA219_WriteDataToRegister_16Bits(ina219, INA219_REG_CONFIG, INA219_CONFIG_RESET);
    osal_msleep(1);  // 延迟1ms，替换原usleep(1000)
}

// 写校准寄存器
void INA219_SetCalibration(INA219_t *ina219, uint16_t calibrationData)
{
    INA219_WriteDataToRegister_16Bits(ina219, INA219_REG_CALIBRATION, calibrationData);
}

// 读取配置寄存器
uint16_t INA219_GetConfigInfo(INA219_t *ina219)
{
    return INA219_ReadDataForRegister_16Bits(ina219, INA219_REG_CONFIG);
}

// 设置配置寄存器
void INA219_SetConfig(INA219_t *ina219, uint16_t configData)
{
    INA219_WriteDataToRegister_16Bits(ina219, INA219_REG_CONFIG, configData);
}

// 设置 16V 8A 校准值
void INA219_SetCalibration_16V_8A(INA219_t *ina219)
{
    uint16_t configInfo = INA219_CONFIG_VOLTAGE_RANGE_16V |
                          INA219_CONFIG_GAIN_2_80MV |
                          INA219_CONFIG_BADCRES_12BIT |
                          INA219_CONFIG_SADCRES_12BIT_1S_532US |
                          INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    ina219_calibrationValue = 20480;
    ina219_currentDivider_mA = 5;
    ina219_powerMultiplier_mW = 0.25f;

    INA219_SetCalibration(ina219, ina219_calibrationValue);
    INA219_SetConfig(ina219, configInfo);
}

// 初始化 INA219
uint8_t INA219_Init(INA219_t *ina219, uint32_t bus, uint16_t address)
{
    ina219->bus = bus;
    ina219->address = address;

    ina219_currentDivider_mA = 0;
    ina219_powerMultiplier_mW = 0;

    INA219_Reset(ina219);
    INA219_SetCalibration_16V_8A(ina219);

    osal_printk("INA219: INA219 initialized on bus %u, addr 0x%02X\n", bus, address);
    return 1;
}
