#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Docs Reference
// (I2C) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
// (I2C Code) https://github.com/ThomasGeor/esp32_tc74_temperature_sensor/blob/master/main/app_main.c

// Temperature sensor:
// SCLK -> OUTPUT (GPIO02)
// SDA -> INPUT (GPIO04)

#define I2C_MASTER_SCL_IO GPIO_NUM_2
#define I2C_MASTER_SDA_IO GPIO_NUM_4
#define I2C_MASTER_NUM  I2C_NUM_0                   // 2C port number for master dev 
#define TC74_SLAVE_ADDR_A5  0x4D                    // default slave address for TC74 sensor
#define TC74_SLAVE_ADDR TC74_SLAVE_ADDR_A5          // slave address for TC74 sensor 
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define READ_WRITE_CONFIG_REGISTER  0x01
#define NACK_VAL 0x1                                // I2C nack value 
#define SET_NORM_OP_VALUE 0x00                      // sets the 7th bit of configuration register to normal mode
#define READ_TEMP_REGISTER 0x00
#define SET_STANBY_VALUE 0x80                       // sets the 7th bit of configuration register to standby mode
#define I2C_MASTER_FREQ_HZ 100000                   // I2C master clock frequency 

void config_i2c(){
    // I2C COnfiguration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                    // I2C master mode
        .sda_io_num = I2C_MASTER_SDA_IO,            // GPIO number for I2C sda signal
        .scl_io_num = I2C_MASTER_SCL_IO,            // GPIO number for I2C scl signal
        .sda_pullup_en = GPIO_PULLUP_ENABLE,        // Internal GPIO pull mode for I2C sda signal
        .scl_pullup_en = GPIO_PULLUP_ENABLE,        // Internal GPIO pull mode for I2C scl signal
        .master.clk_speed = I2C_MASTER_FREQ_HZ,     // select frequency specific to your project
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);

    // Install driver
    // slv_rx_buf_len: Receiving buffer size. Only slave mode will use this value
    // slv_tx_buf_len: Sending buffer size. Only slave mode will use this value
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_master_read_tc74_config(uint8_t *mode){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, READ_WRITE_CONFIG_REGISTER, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, mode, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 300 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_master_set_tc74_mode(uint8_t mode){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, READ_WRITE_CONFIG_REGISTER, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mode, SET_NORM_OP_VALUE);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 300 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_master_read_temp(uint8_t *temperature_value){
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, READ_TEMP_REGISTER, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, temperature_value, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 300 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void){
    static const char *TAG = "TC74";

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    // Config I2C
    config_i2c();

    // Master read configuration
    uint8_t operation_mode;

    // signed integer value of 8 bits
    uint8_t temperature_value;

    while(1){
        i2c_master_read_tc74_config(&operation_mode);
        i2c_master_set_tc74_mode(SET_NORM_OP_VALUE);
        
        // Delay for 250ms
        vTaskDelay(250 / portTICK_RATE_MS);

        i2c_master_read_temp(&temperature_value);
        
        ESP_LOGI(TAG,"Temperature is : %d",temperature_value);
        i2c_master_read_tc74_config(&operation_mode);
        i2c_master_set_tc74_mode(SET_STANBY_VALUE);

        // Delay for 1500ms
        vTaskDelay(1500 / portTICK_RATE_MS);
    }
}
