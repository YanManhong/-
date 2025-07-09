

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTClient.h"
#include "osal_debug.h"
#include "los_memory.h"
#include "los_task.h"
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
#include "MQTTClient.h"
#include "wifi_connect.h"
#include "sle_low_latency.h"
#include "i2c.h"
#include "INA219.h"
#include "aht20.h"
#include "aht20_test.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#include "bh1750.h"
#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
#define SLE_UART_TASK_STACK_SIZE            0x600
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

#define SLE_UART_TASK_PRIO                  28
#define SLE_UART_TASK_DURATION_MS           2000
#define SLE_UART_BAUDRATE                   115200
#define SLE_UART_TRANSFER_SIZE              512
#define I2C_MASTER_ADDR            0x0
#define I2C_SET_BAUDRATE           400000
#define I2C_MASTER_BUS_ID 1

static INA219_t ina219;



static void i2c(void) {
    uapi_pin_set_mode(15, 2);
    uapi_pin_set_mode(16, 2);
    uint32_t baudrate = I2C_SET_BAUDRATE;
    uint32_t hscode = I2C_MASTER_ADDR;
    errcode_t ret = uapi_i2c_master_init(1, baudrate, hscode);
    if (ret != 0) {
        printf("I2C init failed! error code: %d\r\n", ret);
        return;
    }
}

static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };

static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

static void uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }else if (CONFIG_SLE_UART_BUS == 1) {
        uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
        uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);       
    }
}

static void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };

    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uapi_uart_deinit(CONFIG_SLE_UART_BUS);
    uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);

}

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)

#define SLE_UART_SERVER_DELAY_COUNT         5

#define SLE_UART_TASK_STACK_SIZE            0x1200
#define SLE_ADV_HANDLE_DEFAULT              1
#define SLE_UART_SERVER_MSG_QUEUE_LEN       5
#define SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE  32
#define SLE_UART_SERVER_QUEUE_DELAY         0xFFFFFFFF
#define SLE_UART_SERVER_BUFF_MAX_SIZE       800
extern MQTTClient client;  

unsigned long g_sle_uart_server_msgqueue_id;
#define SLE_UART_SERVER_LOG                 "[sle uart server]"
static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}
static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps write request callback cbk server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, write_cb_para->handle, status);
    if ((write_cb_para->length > 0) && write_cb_para->value) {
        osal_printk("\n sle uart recived data : %s\r\n", write_cb_para->value);
        uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)write_cb_para->value, write_cb_para->length, 0);
    }
}

void ble_send_intensity(uint16_t light, uint16_t vbus, uint16_t current, float temp, float humi) {
    char payload[128] = {0};
    int len = snprintf(payload, sizeof(payload), 
                       "light:%u\nvbus:%u\ncurrent:%u\ntemp:%.2f\nhumi:%.2f", 
                       light, vbus, current, temp, humi);
    
    if (len <= 0) {
        osal_printk("[BLE] Format sensor data failed!\r\n");
        return;
    }

    errcode_t ret = sle_uart_server_send_report_by_handle((uint8_t *)payload, len);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("[BLE] Send sensor data failed: %x\r\n", ret);
    } else {
        osal_printk("[BLE] Sent:\n%s\r\n", payload);
    }
}




static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    if (sle_uart_client_is_connected()) {
    sle_uart_server_send_report_by_handle(buffer, length);
    } else {
        osal_printk("%s sle client is not connected! \r\n", SLE_UART_SERVER_LOG);
    }
}

float read_temp_sensor(void)
{
    float temp = 35.0f;
    float humi;

    if (aht20_start_measure() != 0) {
        return 35.0f;
    }

    osDelay(75); // 等待测量完成（单位ms）

    if (aht20_get_measure_result(&temp, &humi) != 0) {
        return 35.0f;
    }

    return temp;
}




static void sle_uart_server_create_msgqueue(void)
{
    if (osal_msg_queue_create("sle_uart_server_msgqueue", SLE_UART_SERVER_MSG_QUEUE_LEN, \
        (unsigned long *)&g_sle_uart_server_msgqueue_id, 0, SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE) != OSAL_SUCCESS) {
        osal_printk("^%s sle_uart_server_create_msgqueue message queue create failed!\n", SLE_UART_SERVER_LOG);
    }
}

static void sle_uart_server_delete_msgqueue(void)
{
    osal_msg_queue_delete(g_sle_uart_server_msgqueue_id);
}

static void sle_uart_server_write_msgqueue(uint8_t *buffer_addr, uint16_t buffer_size)
{
    osal_msg_queue_write_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                              (uint32_t)buffer_size, 0);
}

static int32_t sle_uart_server_receive_msgqueue(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    return osal_msg_queue_read_copy(g_sle_uart_server_msgqueue_id, (void *)buffer_addr, \
                                    buffer_size, SLE_UART_SERVER_QUEUE_DELAY);
}
static void sle_uart_server_rx_buf_init(uint8_t *buffer_addr, uint32_t *buffer_size)
{
    *buffer_size = SLE_UART_SERVER_MSG_QUEUE_MAX_SIZE;
    (void)memset_s(buffer_addr, *buffer_size, 0, *buffer_size);
}

static void *sle_uart_server_task(const char *arg)
{
    unused(arg);

    // 初始化 I2C 和电源监控传感器 INA219
    i2c();
    INA219_Init(&ina219, 1, 0x40);

    // 设置 I2C 引脚
    uapi_pin_set_ds(16, 0); 
    uapi_pin_set_ds(15, 0); 
    uapi_pin_set_mode(15, 2);
    uapi_pin_set_mode(16, 2);

    // 风扇相关引脚设置
    uapi_pin_set_mode(12, 0);      
    uapi_gpio_set_dir(12, 1);      
    uapi_gpio_set_val(12, 1);      

    // 继电器相关引脚设置（GPIO11）
    uapi_pin_set_mode(11, 0);       
    uapi_gpio_set_dir(11, 1);      
    uapi_gpio_set_val(11, 0);  // 默认低电平

    // 初始化 BLE UART 服务
    sle_uart_server_create_msgqueue();
    sle_uart_server_register_msg(sle_uart_server_write_msgqueue);
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);

    // 初始化 UART
    uart_init_pin();
    uart_init_config();
    uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, sle_uart_server_read_int_handler);
    if (ret != ERRCODE_SUCC) {
        osal_printk("%s Register uart callback fail.[%x]\r\n", SLE_UART_SERVER_LOG, ret);
        return NULL;
    }

    // 初始化光照传感器
    bh1750_init();

    while (1) {
        uint32_t light = bh1750_GetLightIntensity();
        uint16_t vbus = INA219_ReadBusVoltage(&ina219);
        uint16_t current = INA219_ReadCurrent_mA(&ina219);
        float temperature = 0.0f;
        float humidity = 0.0f;

        if (aht20_start_measure() == 0) {
            osDelay(75);
            if (aht20_get_measure_result(&temperature, &humidity) != 0) {
                osal_printk("读取温湿度失败\r\n");
                temperature = 0.0f;
                humidity = 0.0f;
            }
        } else {
            osal_printk("启动测量失败\r\n");
            temperature = 0.0f;
            humidity = 0.0f;
        }

        // 光照控制继电器逻辑
        if (light > 3000) {
            uapi_gpio_set_val(11, 1);
            osal_printk("[继电器] 光照强，打开继电器 (GPIO11 = 1)\r\n");
        } else {
            uapi_gpio_set_val(11, 0);
            osal_printk("[继电器] 光照弱，关闭继电器 (GPIO11 = 0)\r\n");
        }

        // --- 打印并发送数据 ---
        osal_printk("光照强度: %lu lx\r\n", light);
        osal_printk("电压: %u mV\r\n", vbus);
        osal_printk("电流: %u mA\r\n", current);
        osal_printk("温度: %.2f °C\r\n", temperature);
        osal_printk("湿度: %.2f %%\r\n", humidity);
        ble_send_intensity((uint16_t)light, vbus, current, temperature, humidity);
        osDelay(1000); // 延时 1 秒
    }

    return NULL;
}



#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)


void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{




    unused(client_id);
    unused(conn_id);
    unused(status);

    // 确保字符串以 null 结尾
    char recv_buf[128] = {0};  // 扩大缓冲区，防止数据过长截断
    size_t copy_len = data->data_len < sizeof(recv_buf) - 1 ? data->data_len : sizeof(recv_buf) - 1;
    memcpy(recv_buf, data->data, copy_len);

    osal_printk("\n sle uart received data : %s\r\n", recv_buf);

    // 清空屏幕
    ssd1306_Fill(Black);

    // 分行处理并显示（最多 5 行，支持湿度行）
    char *line = strtok(recv_buf, "\n");
    uint8_t line_num = 0;

    while (line != NULL && line_num < 5) {
        ssd1306_SetCursor(0, line_num * 12);  // 每行高度12像素
        ssd1306_DrawString(line, Font_7x10, White);
        line = strtok(NULL, "\n");
        line_num++;
    }

    ssd1306_UpdateScreen();

    // UART 也发送原始数据（未分割）
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)recv_buf, strlen(recv_buf), 0);
}




void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle uart recived data : %s\r\n", data->data);
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)(data->data), data->data_len, 0);
}

static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    unused(error);
    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();
    sle_uart_send_param->data_len = length;
    sle_uart_send_param->data = (uint8_t *)buffer;
    ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
}

static void *sle_uart_client_task(const char *arg)
{
    unused(arg); // 
    uart_init_pin();

    uart_init_config();
    i2c();
    ssd1306_Init();//第一步
    ssd1306_Fill(Black);//第二步：屏幕全黑

    while (1) {
            uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    
    // 注册新的UART接收回调函数
    // 当UART缓冲区满或空闲时触发回调，调用sle_uart_client_read_int_handler处理接收到的数据
    errcode_t ret = uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS,
                                                   UART_RX_CONDITION_FULL_OR_IDLE,
                                                   1, // 触发回调的最小字节数
                                                   sle_uart_client_read_int_handler);
    
    /*---------------------- SLE UART客户端初始化 ----------------------*/
    // 初始化SLE UART客户端，注册通知和指示回调函数
    // 当接收到服务器通知或指示时，分别调用sle_uart_notification_cb和sle_uart_indication_cb处理
    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    osal_msleep(1000);
    /*---------------------- 错误处理 ----------------------*/
    // 检查UART回调注册是否成功
    if (ret != ERRCODE_SUCC) {
        osal_printk("Register uart callback fail.");
        return NULL; // 注册失败，终止任务
    }

   
    }
    

    return NULL; // 任务应保持运行，此处返回值无实际意义
}
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

static void sle_uart_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* Run the sle_uart_entry. */
app_run(sle_uart_entry);