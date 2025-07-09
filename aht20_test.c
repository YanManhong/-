/*
 * Copyright (c) 2024 HiSilicon Technologies CO., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pinctrl.h"
#include "i2c.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "aht20.h"
#include "app_init.h"
#include "aht20_test.h"

#define CONFIG_I2C_SCL_MASTER_PIN 15
#define CONFIG_I2C_SDA_MASTER_PIN 16
#define CONFIG_I2C_MASTER_PIN_MODE 2
#define I2C_MASTER_ADDR 0x0
#define I2C_SET_BANDRATE 400000

void app_i2c_init_pin(void)
{
    uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);
    uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);
}

void aht20_init(void)
{
    uint32_t baudrate = I2C_SET_BANDRATE;
    uint32_t hscode = I2C_MASTER_ADDR;

    app_i2c_init_pin();
    errcode_t ret = uapi_i2c_master_init(1, baudrate, hscode);
    if (ret != 0) {
        printf("i2c init failed, ret = 0x%X\r\n", ret);
    }

    while (aht20_calibrate() != 0) {
        printf("AHT20 sensor init failed!\r\n");
        osal_mdelay(100); // 100ms delay for sensor retry
    }
}

void aht20_test_task(environment_msg *msg)
{
    errcode_t retval = aht20_start_measure();
    retval = aht20_get_measure_result(&msg->temperature, &msg->humidity);
    if (retval != 0) {
        printf("get humidity data failed!\r\n");
    } else {
        printf("Temperature: %.2f °C\n", msg->temperature);
        printf("Humidity: %.2f %%\n", msg->humidity);
    }
}
