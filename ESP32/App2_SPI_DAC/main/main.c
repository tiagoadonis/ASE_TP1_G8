#include <stdio.h>
#include <string.h>
#include <driver/dac.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi.h"

// Doc Reference
// (SPI) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html
// (SPI Code) https://github.com/espressif/esp-idf/blob/8b4e032255532d0d318b278dd670365e2b16f7a3/examples/peripherals/spi_master/hd_eeprom/main/spi_eeprom_main.c
// (DAC) https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/dac.html

void print_eeprom_data(uint8_t* rx_buf, int n){
	uint16_t column = 0;
	uint8_t data;
	uint32_t first_addr = 0, last_addr = n-1;

	uint32_t addr;
	for (addr = first_addr; addr <= last_addr; addr++) {
		data = rx_buf[addr];

		if (column == 0) {
			printf("%05x: ",addr);
		}

		printf("%02x ",data);
		column++;
		
		if (column == 16) {
			printf("\n");
			column = 0;
		}
	}
}

/*-------------------------------------------- MAIN APP FUNCTION -------------------------------------------*/
void app_main(void){        
    eeprom_t eeprom;

	// Initialize the SPI configuration
	spi_init(&eeprom);

	// Get Status Register
	uint8_t reg;
	readStatusReg(&eeprom, &reg);

	// Data to write
	uint8_t data[512];

	// Buffer with the data read
	uint8_t rx_buf[512];

	// Write Byte
	for (int i = 0; i < 512; i++) {
		data[i] = 0xff - i;	
	}  

	for (int addr = 0; addr < 512; addr++) {
		writeByte(&eeprom, addr, data[addr]);
	}
	
	// DAC
	dac_output_enable(DAC_CHANNEL_2);
	
	for(uint8_t i=0; i < 255; i++){
		//Set output voltage and generate delay
		dac_output_voltage(DAC_CHANNEL_2, i);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	for(uint8_t i=255; i > 0; i--){
		//Set output voltage and generate delay
		dac_output_voltage(DAC_CHANNEL_2, i);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	dac_output_disable(DAC_CHANNEL_2);

	memset(rx_buf, 0, 512);
	read_n_bytes(&eeprom, 0, rx_buf, 512);
	
	print_eeprom_data(rx_buf, 512);
}                                               
