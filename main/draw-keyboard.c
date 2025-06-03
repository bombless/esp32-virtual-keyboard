#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"

// 日志标签
static const char *TAG = "LED_OLED";

// GPIO定义
#define LED_GPIO        GPIO_NUM_2
#define I2C_MASTER_SCL  GPIO_NUM_22
#define I2C_MASTER_SDA  GPIO_NUM_21

// I2C配置
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// SSD1315 OLED参数
#define OLED_ADDR           0x3C
#define OLED_CMD_MODE       0x00
#define OLED_DATA_MODE      0x40
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// 闪烁控制
#define BLINK_PERIOD_MS     3500
static bool led_state = false;
static bool oled_state = false;

// SSD1315初始化命令序列
static const uint8_t ssd1315_init_cmds[] = {
    0xAE,        // Display OFF
    0x20, 0x00,  // Set Memory Addressing Mode (Horizontal)
    0xB0,        // Set Page Start Address
    0xC8,        // Set COM Output Scan Direction
    0x00,        // Set low column address
    0x10,        // Set high column address
    0x40,        // Set start line address
    0x81, 0xFF,  // Set contrast control register
    0xA1,        // Set segment re-map 0 to 127
    0xA6,        // Set normal display
    0xA8, 0x3F,  // Set multiplex ratio(1 to 64)
    0xA4,        // Output RAM to Display
    0xD3, 0x00,  // Set display offset
    0xD5, 0xF0,  // Set display clock divide ratio/oscillator frequency
    0xD9, 0x22,  // Set pre-charge period
    0xDA, 0x12,  // Set com pins hardware configuration
    0xDB, 0x20,  // Set vcomh
    0x8D, 0x14,  // Set DC-DC enable
    0xAF         // Display ON
};

// 简单的5x8字体数据 (只包含数字和字母A-Z)
static const uint8_t font_5x8[][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

// I2C初始化
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 
                              I2C_MASTER_RX_BUF_DISABLE, 
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

// 向SSD1315发送命令
static esp_err_t ssd1315_write_cmd(uint8_t cmd)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, OLED_CMD_MODE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_stop(cmd_handle);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_handle, 
                                         I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

// 向SSD1315发送数据
static esp_err_t ssd1315_write_data(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, OLED_DATA_MODE, true);
    i2c_master_write(cmd_handle, data, len, true);
    i2c_master_stop(cmd_handle);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_handle, 
                                         I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

// SSD1315初始化
static esp_err_t ssd1315_init(void)
{
    for (size_t i = 0; i < sizeof(ssd1315_init_cmds); i++) {
        esp_err_t ret = ssd1315_write_cmd(ssd1315_init_cmds[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SSD1315初始化失败: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGI(TAG, "SSD1315初始化成功");
    return ESP_OK;
}

// 清空显示
static void ssd1315_clear(void)
{
    uint8_t clear_data[OLED_WIDTH] = {0};
    
    for (int page = 0; page < 8; page++) {
        ssd1315_write_cmd(0xB0 + page);  // 设置页地址
        ssd1315_write_cmd(0x00);         // 设置低列地址
        ssd1315_write_cmd(0x10);         // 设置高列地址
        ssd1315_write_data(clear_data, OLED_WIDTH);
    }
}

// 在指定位置显示字符
static void ssd1315_show_plain_char(uint8_t x, uint8_t y, char c)
{
    uint8_t char_index;
    
    if (c >= '0' && c <= '9') {
        char_index = c - '0';
    } else if (c >= 'A' && c <= 'Z') {
        char_index = c - 'A' + 10;
    } else {
        return; // 不支持的字符
    }
    
    // 设置页地址和列地址
    ssd1315_write_cmd(0xB0 + y);           // 页地址
    ssd1315_write_cmd(0x00 + (x & 0x0F));  // 低列地址
    ssd1315_write_cmd(0x10 + (x >> 4));    // 高列地址
    
    // 发送字符数据
    ssd1315_write_data((uint8_t*)font_5x8[char_index], 5);
}

// 在指定位置显示字符
static void ssd1315_show_char(uint8_t x, uint8_t y, char c, uint8_t width)
{
    uint8_t buffer[width];
    uint8_t char_index;
    
    if (c >= '0' && c <= '9') {
        char_index = c - '0';
    } else if (c >= 'A' && c <= 'Z') {
        char_index = c - 'A' + 10;
    } else {
        return; // 不支持的字符
    }
    
    // 设置页地址和列地址
    ssd1315_write_cmd(0xB0 + y);           // 页地址
    ssd1315_write_cmd(0x00 + (x & 0x0F));  // 低列地址
    ssd1315_write_cmd(0x10 + (x >> 4));    // 高列地址

    uint8_t bar = 255;
    uint8_t empty = 0;

    uint8_t *p = buffer;
    
    // 发送字符数据
    *p++ = bar;

    int left_space = (width - 2 - 5) / 2;
    int right_space = width - 2 - 5 - left_space;
    for (int i = 0; i < left_space; i += 1) {
        *p++ = empty;
    }
    memcpy(p, font_5x8[char_index], 5);
    p += 5;
    for (int i = 0; i < right_space; i += 1) {
        *p++ = empty;
    }
    *p++ = bar;
    ssd1315_write_data(buffer, width);
}

static void ssd1315_show_line(uint8_t x_start, uint8_t x_end, uint8_t y, uint8_t value)
{
    // 设置页地址和列地址
    ssd1315_write_cmd(0xB0 + y); // 页地址
    ssd1315_write_cmd(0x00);     // 低列地址0
    ssd1315_write_cmd(0x10);     // 高列地址0
    uint8_t buffer[128];

    for (int i = 0; i < 128; i += 1) {
        if (i < x_start || i >= x_end) {
            buffer[i] = 0x00;
        } else {
            buffer[i] = value;
        }
    }
    
    // 发送字符数据
    ssd1315_write_data(buffer, 128);
}



// 显示字符串
static void ssd1315_show_keyboard()
{
    uint8_t x = 8;
    for (const char *p = "QWERTYUIOP"; *p; p++) {
        ssd1315_show_char(x, 0, *p, 12);
        x += 12;
    }

    ssd1315_show_line(8, 127, 1, 1);
    ssd1315_show_line(13, 120, 2, 128);

    x = 13;
    for (const char *p = "ASDFGHJKL"; *p; p++) {
        ssd1315_show_char(x, 3, *p, 12);
        x += 12;
    }

    ssd1315_show_line(13, 120, 4, 1);
    ssd1315_show_line(27, 110, 5, 128);
    
    x = 15;
    for (const char *p = "tZXCVBNM"; *p; p++) {
        ssd1315_show_char(x, 6, *p, 12);
        x += 12;
    }
    ssd1315_show_line(27, 110, 7, 1);
}

// LED和OLED控制任务
static void blink_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t blink_delay = pdMS_TO_TICKS(BLINK_PERIOD_MS);
    
    while (1) {
        // 切换LED状态
        led_state = !led_state;
        gpio_set_level(LED_GPIO, led_state);
        
        // 切换OLED显示状态
        oled_state = !oled_state;

        if (oled_state) {
            ssd1315_show_keyboard();
        } else {
            ssd1315_clear();
            int x = 0;
            for (const char *p = "HELLO WORLD"; *p; p++) {
                x += 9;
                ssd1315_show_plain_char(x, 4, *p);
            }            
        }


        // 输出状态日志
        ESP_LOGI(TAG, "LED: %s | OLED: %s", 
                 led_state ? "ON" : "OFF", 
                 oled_state ? "VISIBLE" : "BLANK");
        
        // 等待下一次闪烁
        vTaskDelayUntil(&last_wake_time, blink_delay);
    }
}

// GPIO初始化
static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 LED和SSD1315 OLED闪烁程序启动");
    
    // 初始化GPIO
    gpio_init();
    ESP_LOGI(TAG, "GPIO初始化完成");
    
    // 初始化I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C初始化完成");
    
    // 短暂延时确保硬件稳定
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 初始化SSD1315
    ESP_ERROR_CHECK(ssd1315_init());
    
    // 创建闪烁任务
    xTaskCreate(blink_task, "blink_task", 4096, NULL, 5, NULL);
    // ssd1315_clear();
    ssd1315_show_keyboard();
    // ssd1315_show_char(0, 4, 'A');
    // ssd1315_show_lines(0, 5);
    // ssd1315_show_char(0, 6, 'B');
    // ssd1315_show_lines(0, 7);
    // ssd1315_show_lines(0, 1);
    // ssd1315_show_lines(10, 3);
    
    ESP_LOGI(TAG, "闪烁任务已启动");
}
