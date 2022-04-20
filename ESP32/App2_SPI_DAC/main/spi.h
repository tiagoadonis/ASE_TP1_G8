#include "driver/spi_master.h"

/* GPIO configuration */
#define PIN_NUM_CS 18                       // CS -> GPIO18
#define PIN_NUM_MISO 19                     // MISO -> GPIO19
#define PIN_NUM_MOSI 25                     // MOSI -> GPIO25
#define PIN_NUM_SCLK 27                     // SCLK -> GPIO27

/* SPI commands */
/* (Instruction Set: 25LC040A SPI EEPROM Datasheet -> page8) */
#define EEPROM_RDSR 0x05                    // Read Status Register
#define EEPROM_WREN 0x06                    // Write Enable
#define EEPROM_WRITE 0x02                   // Write to Memory Array  
#define EEPROM_READ 0x03                    // Read from Memory Array

/* SPI status */
#define EEPROM_STATUS_WIP 0x01              // Write in Progress

// New struct with some data needed
typedef struct {
	int32_t	totalBytes;
	int16_t	addressBits;
	int16_t	pageSize;
	int16_t	lastPage;
	spi_device_handle_t handle;
} eeprom_t;

void spi_init(eeprom_t* eeprom);
void readStatusReg(eeprom_t* eeprom, uint8_t* reg);
void writeByte(eeprom_t* eeprom, uint16_t addr, uint8_t data);
void writeEnable(eeprom_t* eeprom);
bool isBusy(eeprom_t* eeprom);
void read_n_bytes(eeprom_t* eeprom, uint16_t addr, uint8_t* buf, int16_t n);