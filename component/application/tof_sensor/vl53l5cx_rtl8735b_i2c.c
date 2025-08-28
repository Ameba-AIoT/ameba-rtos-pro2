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
#include "vl53l5cx_rtl8735b_i2c.h"

uint8_t wireMaxPacketSize = I2C_BUFFER_SIZE;
int flag = 0;

uint8_t txBuffer[I2C_DATA_LENGTH];
uint8_t txAddress = 0;
uint8_t txBufferLength = 0;

uint8_t rxBuffer[I2C_DATA_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

const uint8_t REVISION_ID = 0x02;
const uint8_t DEVICE_ID = 0xf0;

i2c_t i2cmaster;

void i2c_callback_set_flag(void *userdata)
{
	flag = 1;
}

void begin_transmission(uint8_t address)
{
	if (txAddress != address) {
		txAddress = address;
		//	delay(50);
	}
	txBufferLength = 0;
}

uint8_t end_transmission()
{
	uint8_t sendStop = 0x1;
	int length;
	uint8_t error = 0;

	if (txBufferLength == 0) {
		txBuffer[0] = 0x00;  // Default dummy byte
		txBufferLength = 1;
	}

	i2c_reset(&i2cmaster);
	// toggle flag to normal
	if (flag == 1) {
		flag = 0;
	}

	i2c_init(&i2cmaster, MBED_I2C_MTR_SDA, MBED_I2C_MTR_SCL);
	i2c_frequency(&i2cmaster, MBED_I2C_BUS_CLK);

	i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_callback_set_flag);
	if (sendStop == false) {
		i2c_restart_enable(&i2cmaster);
	}
	//printf("Writing to 0x%02X with %d byte(s)\n", txAddress, txBufferLength);
	length = i2c_write(&i2cmaster, (int)txAddress, (char *)txBuffer, (int)txBufferLength, (int)sendStop);
	hal_delay_us(txBufferLength * 200);
	if (sendStop == false) {
		i2c_restart_disable(&i2cmaster);
	}
	if ((txBufferLength > 0) && (length <= 0)) {
		error = 1;
	}
	if (flag == 0) {
		error = -1;	// Error for wrong slave address
	} else {
		error = 0;
	}
	txBufferLength = 0;	// empty buffer

	return error;
}

uint8_t end_transmission_non_send_stop()
{
	uint8_t sendStop = 0x0;
    int length;
    uint8_t error = 0;

	if (txBufferLength == 0) {
		txBuffer[0] = 0x00;  // Default dummy byte
		txBufferLength = 1;
	}

	i2c_reset(&i2cmaster);
	// toggle flag to normal
	if (flag == 1) {
		flag = 0;
	}

	i2c_init(&i2cmaster, MBED_I2C_MTR_SDA, MBED_I2C_MTR_SCL);
	i2c_frequency(&i2cmaster, MBED_I2C_BUS_CLK);

	i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_callback_set_flag);
	if (sendStop == false) {
		i2c_restart_enable(&i2cmaster);
	}

	length = i2c_write(&i2cmaster, (int)txAddress, (char *)txBuffer, (int)txBufferLength, (int)sendStop);
	hal_delay_us(txBufferLength * 200);
	if (sendStop == false) {
		i2c_restart_disable(&i2cmaster);
	}
	if ((txBufferLength > 0) && (length <= 0)) {
		error = 1;
	}
	if (flag == 0) {
		error = -1;    // Error for wrong slave address
	} else {
		error = 0;
	}
	txBufferLength = 0;    // empty buffer

	return error;
}

size_t write_data(uint8_t data)
{
	if (txBufferLength >= I2C_DATA_LENGTH) {
		return 0;
	}
	txBuffer[txBufferLength++] = data;

	return 1;
}

int read_data(void)
{
	if (rxBufferIndex < rxBufferLength) {
		return rxBuffer[rxBufferIndex++];
	}
	return -1;
}

uint8_t request_from(uint8_t address, uint8_t quantity)
{
	uint8_t sendStop = 0x1;
	if (quantity > I2C_DATA_LENGTH) {
		quantity = I2C_DATA_LENGTH;
	}

	// perform blocking read into buffer
	i2c_read(&i2cmaster, ((int)address), (char *)(rxBuffer), ((int)quantity), ((int)sendStop));

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = quantity;

	return quantity;
}

int available(void)
{
	return (rxBufferLength - rxBufferIndex);
}

uint8_t write_single_byte(uint16_t registerAddress, const uint8_t value)
{
	begin_transmission(I2C_DEVICE_ADDRESS);
	write_data(highByte(registerAddress));
	write_data(lowByte(registerAddress));
	write_data(value);
	return end_transmission();
}

uint8_t read_single_byte(uint16_t registerAddress)
{
	begin_transmission(I2C_DEVICE_ADDRESS);
	write_data(highByte(registerAddress));
	write_data(lowByte(registerAddress));
	end_transmission();
	request_from(I2C_DEVICE_ADDRESS, 1U);
	hal_delay_ms(10);
	return read_data();
}

// Must be able to write 32,768 bytes at a time
uint8_t write_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
	// Chunk I2C transactions into limit of 32 bytes (or wireMaxPacketSize)
	uint8_t i2cError = 0;
	uint32_t startSpot = 0;
	uint32_t bytesToSend = bufferSize;
	while (bytesToSend > 0) {
		uint32_t len = bytesToSend;
		if (len > (wireMaxPacketSize - 2)) {	// Allow 2 byte for register address
			len = (wireMaxPacketSize - 2);
		}

		begin_transmission(I2C_DEVICE_ADDRESS);
		write_data(highByte(registerAddress));
		write_data(lowByte(registerAddress));

		for (uint16_t x = 0; x < len; x++) {
		   write_data(buffer[startSpot + x]);	// Write a portion of the payload to the bus
		}

		i2cError = end_transmission();	// Release bus because we are writing the address each time
		if (i2cError != 0) {
			return (i2cError);	// Sensor did not ACK
		}

		startSpot += len;	// Move the pointer forward
		bytesToSend -= len;
		registerAddress += len;	// Move register address forward
	}
	return (i2cError);
}

uint8_t read_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
	uint8_t i2cError = 0;

	// Write address to read from
	begin_transmission(I2C_DEVICE_ADDRESS);
	write_data(highByte(registerAddress));
	write_data(lowByte(registerAddress));
	i2cError =  end_transmission_non_send_stop();	// Do not release bus
	if (i2cError != 0) {
		return (i2cError);
	}

	// Read bytes up to max transaction size
	uint16_t bytesToReadRemaining = bufferSize;
	uint16_t offset = 0;
	while (bytesToReadRemaining > 0) {
		// Limit to 32 bytes or whatever the buffer limit is for given platform
		uint16_t bytesToRead = bytesToReadRemaining;
		if (bytesToRead > wireMaxPacketSize) {
			bytesToRead = wireMaxPacketSize;
		}

		request_from(I2C_DEVICE_ADDRESS, (uint8_t)bytesToRead);
		hal_delay_ms(10);
		if (available()) {
			for (uint16_t x = 0; x < bytesToRead; x++) {
				buffer[offset + x] = read_data();
			}
		} else {
			return (false);	// Sensor did not respond
		}

		offset += bytesToRead;
		bytesToReadRemaining -= bytesToRead;
	}

	return (0);	// Success
}
