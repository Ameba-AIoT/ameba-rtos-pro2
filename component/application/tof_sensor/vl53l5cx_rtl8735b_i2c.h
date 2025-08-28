#include <string.h>
#include "PinNames.h"
#include "basic_types.h"
#include "diag.h"
#include "i2c_api.h"
#include "pinmap.h"
#include "ex_api.h"
#include "freertos_service.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef VL53L5CX_RTL8735B_I2C_H_
#define VL53L5CX_RTL8735B_I2C_H_

#define MBED_I2C_MTR_SDA PE_6
#define MBED_I2C_MTR_SCL PE_5

#define MBED_I2C_BUS_CLK		400000
#define I2C_DATA_LENGTH			128
#define I2C_DEVICE_ADDRESS		0x29
#define I2C_BUFFER_SIZE			32

#define lowByte(w)  ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

extern const uint8_t REVISION_ID;
extern const uint8_t DEVICE_ID;
extern i2c_t i2cmaster;

// TX Buffer
extern uint8_t txBuffer[I2C_DATA_LENGTH];
extern uint8_t txAddress;
extern uint8_t txBufferLength;

// RX Buffer
extern uint8_t rxBuffer[I2C_DATA_LENGTH];
extern uint8_t rxBufferIndex;
extern uint8_t rxBufferLength;

void begin_transmission(uint8_t address);
uint8_t end_transmission();
uint8_t end_transmission_non_send_stop();

int available(void);
size_t write_data(uint8_t data);
int read_data(void);

uint8_t request_from(uint8_t address, uint8_t quantity);
uint8_t write_single_byte(uint16_t registerAddress, const uint8_t value);
uint8_t read_single_byte(uint16_t registerAddress);
uint8_t write_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize);
uint8_t read_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize);

#endif
