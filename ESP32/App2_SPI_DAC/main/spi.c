#include <driver/spi_master.h>
#include "esp_log.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi.h"
#include <string.h>

// Initialize the SPI configurations
void spi_init(eeprom_t* eeprom){
	esp_err_t ret;

    // Bus configuration
	spi_bus_config_t bus_config = {
		.sclk_io_num = PIN_NUM_SCLK,        // GPIO pin for SPI Clock signal
		.mosi_io_num = PIN_NUM_MOSI,        // GPIO pin for Master Out Slave In
		.miso_io_num = PIN_NUM_MISO,        // GPIO pin for Master In Slave Out
		.quadwp_io_num = -1,                // GPIO pin for WP (Write Protect) signal, or -1 if not used
		.quadhd_io_num = -1                 // GPIO pin for HD (Hold) signal, or -1 if not used.
	};

    // Initialize the SPI bus
    // SPI_DMA_CH_AUTO -> to let the driver to allocate the DMA channel
	spi_bus_initialize(HSPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
	
    // Set the atributes of the eeprom_t structure
	eeprom->totalBytes = 512;                                       // EEPROM 25LC040A Organization
	eeprom->addressBits = 9;                                        // 2^9 = 512
	eeprom->pageSize = 16;                                          // EEPROM 25LC040A Page Size
	eeprom->lastPage = (eeprom->totalBytes / eeprom->pageSize)-1;   // 32 pages -> [0..31]

    // Device configuration structure (configuration for a SPI slave)
	spi_device_interface_config_t dev_config;
    
    // memset -> fill a block of memory with the given value (dev_config)
    // to save the 25LC040A SPI EEPROM configuration 
	memset(&dev_config, 0, sizeof(spi_device_interface_config_t));

	dev_config.clock_speed_hz = SPI_MASTER_FREQ_8M;     // Clock speed -> 8MHz
	dev_config.spics_io_num = PIN_NUM_CS;               // CS GPIO pin for this device
	dev_config.queue_size = 1;                          // how many transactions can exist at the same time
	dev_config.mode = 0;                                // SPI mode, a pair of (CPOL, CPHA)
                                                        // CPOL: Clock polarity | CPHA: clock phase

    // Handle for a device on a SPI bus
	spi_device_handle_t handle;
    // Add the device previously configured to the bus
	spi_bus_add_device(HSPI_HOST, &dev_config, &handle);
	
    // set the handler
    eeprom->handle = handle;
}

// Read the status register -> to check the actual status
void readStatusReg(eeprom_t* eeprom, uint8_t* reg){
    // Transaction configuration structure
	spi_transaction_t transaction;

    // Instruction Set and register needed
	uint8_t data[2];

    // Instruction Set-> Read Status Register
	data[0] = EEPROM_RDSR;
	
    // to save the transaction configuration
    memset(&transaction, 0, sizeof(spi_transaction_t));

	transaction.length = 2 * 8;             // Total data length, in bits
	transaction.tx_buffer = data;           // Pointer to transmit buffer
	transaction.rx_buffer = data;           // Pointer to receive buffer

    // Send the SPI transaction previously configured
    spi_device_transmit(eeprom->handle, &transaction);
	*reg = data[1];
}

// Set write enable
void writeEnable(eeprom_t* eeprom){
    // Create the transaction
	spi_transaction_t transaction;

    // Only the Instruction Set code is needed
	uint8_t data[1];

    // Write Enable code
	data[0] = EEPROM_WREN;

    // to save the transaction configuration
	memset(&transaction, 0, sizeof(spi_transaction_t));

    // Configure the transaction
	transaction.length = 1 * 8;         // Total data length, in bits
	transaction.tx_buffer = data;       // Pointer to transmit buffer
	transaction.rx_buffer = data;       // Pointer to receive buffer
	
    // Send the SPI transaction previously configured
    spi_device_transmit(eeprom->handle, &transaction);
}

// Checks if the bus is busy
bool isBusy(eeprom_t* eeprom){
	// Create the transaction
    spi_transaction_t transaction;

    // Instruction Set and bus status are needed
	uint8_t data[2];

    // Read Status Register code
	data[0] = EEPROM_RDSR;
	
    // to save the transaction configuration
    memset(&transaction, 0, sizeof(spi_transaction_t));

	transaction.length = 2 * 8;             // Total data length, in bits
	transaction.tx_buffer = data;           // Pointer to transmit buffer
	transaction.rx_buffer = data;           // Pointer to receive buffer

    // Send the SPI transaction previously configured
	esp_err_t ret = spi_device_transmit(eeprom->handle, &transaction);

    // If the transactions fails -> the bus is busy
	if (ret != ESP_OK){
        return false;
    }

    // The write in progress is on the bus -> the write operation has begun, like we previously set (WREN)
	if((data[1] & EEPROM_STATUS_WIP) == EEPROM_STATUS_WIP){
        return true;
    }

    // For any other reason is busy
	return false;
}

// To write some bytes on the EEPROM (read the 25LC040A SPI EEPROM Datasheet to help)
void writeByte(eeprom_t* eeprom, uint16_t addr, uint8_t data){
	// Create the transaction
	spi_transaction_t transaction;

	// Set write enable -> must be done to write something on the EEPROM
	writeEnable(eeprom);
	
	// Check if the bus is busy -> if it's busy wait
	while(isBusy(eeprom)) {
		vTaskDelay(1);
	}

    // Instruction + Address MSb + Lower Address Byte +Data Byte
	uint8_t new_data[4];
	int16_t index = 0;

    // Write code
	new_data[0] = EEPROM_WRITE;

    if (eeprom->addressBits == 9 && addr > 0xff) {
        new_data[0] = new_data[0] | 0x08;
    }

    if (eeprom->addressBits <= 9) {
		new_data[1] = (addr & 0xFF);
		new_data[2] = data;
		index = 2;
	} else {
		new_data[1] = (addr >> 8) & 0xFF;
		new_data[2] = addr & 0xFF;
		new_data[3] = data;
		index = 3;
	}

    // to save the transaction configuration
	memset(&transaction, 0, sizeof(spi_transaction_t));

	transaction.length = (index+1) * 8;             // Total data length, in bits
	transaction.tx_buffer = new_data;               // Pointer to transmit buffer
	transaction.rx_buffer = new_data;               // Pointer to receive buffer

    // Send the SPI transaction previously configured
	spi_device_transmit(eeprom->handle, &transaction);
	
	// Wait for idle
	while(isBusy(eeprom)) {
		vTaskDelay(1);
	}
}

void read_n_bytes(eeprom_t* eeprom, uint16_t addr, uint8_t* buf, int16_t n){ 
	// Create the transaction
	spi_transaction_t transaction;

    // Instruction + Address MSb + Lower Address Byte + Data Out
	uint8_t data[4];
	
    int16_t index = 0;

    // Read de n bytes
	for (int i = 0; i < n; i++) {
		uint16_t new_addr = addr + i;

        // Read code
		data[0] = EEPROM_READ;
		
        if(eeprom->addressBits == 9 && addr > 0xff){
            data[0] = data[0] | 0x08;
        }

		if(eeprom->addressBits <= 9){
			data[1] = (new_addr & 0xFF);
			index = 2;
		} 
        else{
			data[1] = (new_addr >> 8) & 0xFF;
			data[2] = new_addr & 0xFF;
			index = 3;
		}

        // to save the transaction configuration
		memset(&transaction, 0, sizeof(spi_transaction_t));

		transaction.length = (index+1) * 8;             // Total data length, in bits
		transaction.tx_buffer = data;                   // Pointer to transmit buffer
		transaction.rx_buffer = data;                   // Pointer to receive buffer
		
        // Send the SPI transaction previously configured
        spi_device_transmit(eeprom->handle, &transaction);
		
        buf[i] = data[index];
	}
}