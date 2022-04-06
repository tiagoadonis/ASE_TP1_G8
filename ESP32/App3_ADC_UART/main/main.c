#include <stdio.h>
#include <driver/adc.h>
#include "driver/uart.h"
#include <esp_adc_cal.h>
#include <string.h>

// Doc Reference
// (ADC) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
// (ADC Code) https://github.com/espressif/esp-idf/blob/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/adc/single_read/adc/main/adc1_example_main.c
// (GPIO) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html
// (GPIO Code) https://github.com/espressif/esp-idf/blob/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
// (UART) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#uart-api-setting-communication-parameters
// (UART Code) https://github.com/espressif/esp-idf/blob/4350e6fef859b94f3efae6ceced751911c9d2cbc/examples/peripherals/uart/uart_echo/main/uart_echo_example_main.c

// GPIOs must be used in all 3 applications.

// Over-the-Air update capabilities must be implemented and used to swap between 
// two applications of your choice.

// Power saving/Power management features must be included in one application (the one that, 
// from your point of view, is more appropriate). It would be great if you can use an application 
// that provides the current supplied by the USB port connected to the kit.

#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   25          // Multisampling

void config_UART(){
    const uart_port_t uart_num = UART_NUM_0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,      // enable hardware flow control
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, 18, 19));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    // uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, queue_size, uart_queue, intr_alloc_flags)
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // Write data to UART.
    char* test_str = "This is a test string.\n";
    uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
}


// To configure the ADC
void config_ADC(){
    // ADC1: 
    // 8 channels: GPIO32 - GPIO39

    //ADC1 channel 0 is GPIO36
    adc_channel_t channel = ADC1_CHANNEL_0; 

    // ADC capture width is 12Bit.
    adc_bits_width_t width = ADC_WIDTH_BIT_12;

    // ADC attenuation is 0 dB
    adc_atten_t atten = ADC_ATTEN_DB_11;

    // Configure precision
    adc1_config_width(width);

    // To calculate the voltage based on the ADC raw results, this formula can be used:
    // Vout = Dout * Vmax / Dmax 

    // ADC unit - SAR ADC 1
    adc_unit_t unit = ADC_UNIT_1;

    // Configure attenuation
    // +----------+-------------+-----------------+
    // |          | attenuation | suggested range |
    // |    SoC   |     (dB)    |      (mV)       |
    // +==========+=============+=================+
    // |          |       0     |    100 ~  950   |
    // |          +-------------+-----------------+
    // |          |       2.5   |    100 ~ 1250   |
    // |   ESP32  +-------------+-----------------+
    // |          |       6     |    150 ~ 1750   |
    // |          +-------------+-----------------+
    // |          |      11     |    150 ~ 2450   |
    // +----------+-------------+-----------------+
    adc1_config_channel_atten(channel, atten);

     //Characterize ADC at particular atten
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    // To read ADC conversion result
    while(1){
        uint32_t raw = 0;
        
        for(int i = 0; i < NO_OF_SAMPLES; i++){
            // Get raw value
            raw += adc1_get_raw(channel);
        }
        raw /= NO_OF_SAMPLES;

        // Get voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
        // Pass voltage to V
        float toVolts = (float) voltage / (float) 1000;

        // Print the info
        printf("Raw: %d\tVoltage: %.2fV\n", raw, toVolts);
    }
}

void app_main(void){
    //config_ADC();

    config_UART();
}
