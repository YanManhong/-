#include "bh1750.h"
#include "i2c.h"
#include "pinctrl.h"
#include "common_def.h"
#include "soc_osal.h"
#include <stdio.h>

#define BH1750_ADDR              0x46
#define BH1750_POWER_OFF         0x00
#define BH1750_POWER_ON          0x01
#define BH1750_MODULE_RESET      0x07
#define BH1750_CONTINUE_H_MODE   0x10
#define BH1750_RES               1
#define MEASURE_MODE             BH1750_CONTINUE_H_MODE

#define I2C_SET_BAUDRATE         400000
#define I2C_MASTER_ADDR          0x0
#define I2C_MASTER_BUS_ID        1

static uint16_t bh1750_buff_H = 0;
static uint16_t bh1750_buff_L = 0;

static void i2c(void) {
    uapi_pin_set_mode(15, 2);
    uapi_pin_set_mode(16, 2);
    uint32_t baudrate = I2C_SET_BAUDRATE;
    uint32_t hscode = I2C_MASTER_ADDR;
    errcode_t ret = uapi_i2c_master_init(I2C_MASTER_BUS_ID, baudrate, hscode);
    if (ret != 0) {
        printf("I2C init failed! error code: %d\r\n", ret);
    }
}

void bh1750_SendCMD(uint8_t cmd){
    uint8_t buffer[] = {cmd};
    i2c_data_t data = {
        .send_buf = buffer,
        .send_len = sizeof(buffer)
    };
    errcode_t ret = uapi_i2c_master_write(I2C_MASTER_BUS_ID, BH1750_ADDR >> 1, &data);
    if (ret != 0) {
        printf("BH1750:I2cWriteCMD(%02X) failed, %0X!\n", cmd, ret);
    }
}

void bh1750_ReadData(void){
    uint8_t buffer[2] = {0};
    i2c_data_t data = {
        .receive_buf = buffer,
        .receive_len = sizeof(buffer)
    };
    errcode_t ret = uapi_i2c_master_read(I2C_MASTER_BUS_ID, BH1750_ADDR >> 1, &data);
    if (ret != 0) {
        printf("BH1750:I2cRead(len:%d) failed, %0X!\n", data.receive_len, ret);
        return;
    }
    bh1750_buff_H = data.receive_buf[0];
    bh1750_buff_L = data.receive_buf[1];
}

uint16_t bh1750_GetLightIntensity(void){
    bh1750_ReadData();
    uint16_t data = (bh1750_buff_H << 8) | bh1750_buff_L;
    return (data * BH1750_RES * 10) / 12;
}

void bh1750_init(void){
    bh1750_SendCMD(BH1750_POWER_ON);
    bh1750_SendCMD(BH1750_MODULE_RESET);
    bh1750_SendCMD(MEASURE_MODE);
    osal_msleep(200);
    osal_printk("BH1750 Init SUCC!\r\n");
}
