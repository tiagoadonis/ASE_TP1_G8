#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Doc Reference
// (Manual) https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
// (GPIO) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
// (GPIO Code) https://github.com/espressif/esp-idf/blob/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
// (I2C) https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/i2c.html
// (I2C Code) https://github.com/espressif/esp-idf/blob/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/i2c/i2c_simple/main/i2c_simple_main.c
// (PWM - LEDC)  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
// (PWM - LEDC Code) https://github.com/espressif/esp-idf/tree/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/ledc

static const char *TAG = "I2C_PWM";

#define I2C_MASTER_SCL_IO GPIO_NUM_22               /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_21               /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                            /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ 100000                   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0
#define TC74_SLAVE_ADDR_A5 0x4D                     /*!< default slave address for TC74 sensor */
#define READ_TEMP_REGISTER 0x00
#define TC74_SLAVE_ADDR TC74_SLAVE_ADDR_A5          /*!< slave address for TC74 sensor */
#define READ_BIT I2C_MASTER_READ                    /*!< I2C master read */
#define WRITE_BIT I2C_MASTER_WRITE                  /*!< I2C master write */
#define ACK_CHECK_EN 0x1                            /*!< I2C master will check ack from slave*/
#define NACK_VAL 0x1                                /*!< I2C nack value */

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO (5)
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO (4)
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1
#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TEST_CH_NUM (2)
#define LEDC_TEST_DUTY (4000)
#define LEDC_TEST_FADE_TIME (3000)

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_master_read_temp(i2c_port_t i2c_num, uint8_t *tmprt) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, READ_TEMP_REGISTER, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, TC74_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, tmprt, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 300 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void i2c_pwm_task(void *arg){
    uint8_t temp_val;
    int ch;

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
            {   .channel    = LEDC_HS_CH0_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_HS_CH0_GPIO,
                .speed_mode = LEDC_HS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_HS_TIMER,
                .flags.output_invert = 0 },
            
            {   .channel    = LEDC_HS_CH1_CHANNEL,
                .duty       = 0,
                .gpio_num   = LEDC_HS_CH1_GPIO,
                .speed_mode = LEDC_HS_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_HS_TIMER,
                .flags.output_invert = 0 },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while(1) {
        i2c_master_read_temp(I2C_MASTER_NUM,&temp_val);
        ESP_LOGI(TAG,"Temperature is : %d",temp_val);
        
        if(temp_val < 30) {
            ch = 0;             // green led (Channel 0 - GPIO 5)
        }
        else {
            ch = 1;             // red led (Channel 1 - GPIO 4)
        }
        
        // Fade up/increase intensity
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);

        // Fade down/decrease intensity
        ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
        ledc_fade_start(ledc_channel[ch].speed_mode,
                ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    ledc_fade_func_uninstall();
}

void app_main(void) {
    xTaskCreate(i2c_pwm_task, "i2c_pwm_task", 1024 * 2, (void *)0, 10, NULL);
}