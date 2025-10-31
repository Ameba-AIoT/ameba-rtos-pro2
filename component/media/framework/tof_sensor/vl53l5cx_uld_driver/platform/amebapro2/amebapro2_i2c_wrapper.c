#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "basic_types.h"
#include "diag.h"
#include "amebapro2_i2c_wrapper.h"

static tof_i2c_config_t *g_cfg = NULL;  // read-only config
static tof_i2c_ctx_t    *g_ctx = NULL;  // mutable runtime state

void tof_i2c_init(tof_i2c_config_t *cfg, tof_i2c_ctx_t *ctx)
{
	g_cfg = cfg;
	g_ctx = ctx;

	i2c_init(&g_cfg->i2cmaster, g_cfg->sda, g_cfg->scl);
	i2c_frequency(&g_cfg->i2cmaster, g_cfg->bus_clk_hz);
}

void tof_i2c_deinit(tof_i2c_config_t *cfg)
{
	g_cfg = cfg;

	i2c_reset(&g_cfg->i2cmaster);
}

void tof_i2c_callback_set_tx_complete_flag(void *userdata)
{
	g_ctx->tx_complete_flag = 1;
}

void tof_i2c_begin_transmission(uint8_t address)
{
	if (g_ctx->tx_address != address) {
		g_ctx->tx_address = address;
		//	delay(50);
	}
	g_ctx->tx_buffer_length = 0;
}

uint8_t tof_i2c_end_transmission(void)
{
	uint8_t sendStop = 0x1;
	int length;
	uint8_t error = 0;

	if (g_ctx->tx_buffer_length == 0) {
		g_ctx->tx_buffer[0] = 0x00;  // Default dummy byte
		g_ctx->tx_buffer_length = 1;
	}

	i2c_reset(&g_cfg->i2cmaster);
	// toggle flag to normal
	if (g_ctx->tx_complete_flag == 1) {
		g_ctx->tx_complete_flag = 0;
	}

	i2c_init(&g_cfg->i2cmaster, g_cfg->sda, g_cfg->scl);
	i2c_frequency(&g_cfg->i2cmaster, g_cfg->bus_clk_hz);

	i2c_set_user_callback(&g_cfg->i2cmaster, I2C_TX_COMPLETE, tof_i2c_callback_set_tx_complete_flag);
	if (sendStop == false) {
		i2c_restart_enable(&g_cfg->i2cmaster);
	}
	//printf("Writing to 0x%02X with %d byte(s)\n", g_ctx->tx_address, g_ctx->tx_buffer_length);
	length = i2c_write(&g_cfg->i2cmaster, (int)g_ctx->tx_address, (char *)g_ctx->tx_buffer, (int)g_ctx->tx_buffer_length, (int)sendStop);
	hal_delay_us(g_ctx->tx_buffer_length * 200);
	if (sendStop == false) {
		i2c_restart_disable(&g_cfg->i2cmaster);
	}
	if ((g_ctx->tx_buffer_length > 0) && (length <= 0)) {
		error = 1;
	}
	if (g_ctx->tx_complete_flag == 0) {
		error = -1;	// Error for wrong slave address
	} else {
		error = 0;
	}
	g_ctx->tx_buffer_length = 0;	// empty buffer

	return error;
}

uint8_t tof_i2c_end_transmission_non_send_stop(void)
{
	uint8_t sendStop = 0x0;
	int length;
	uint8_t error = 0;

	if (g_ctx->tx_buffer_length == 0) {
		g_ctx->tx_buffer[0] = 0x00;  // Default dummy byte
		g_ctx->tx_buffer_length = 1;
	}

	i2c_reset(&g_cfg->i2cmaster);
	// toggle flag to normal
	if (g_ctx->tx_complete_flag == 1) {
		g_ctx->tx_complete_flag = 0;
	}

	i2c_init(&g_cfg->i2cmaster, g_cfg->sda, g_cfg->scl);
	i2c_frequency(&g_cfg->i2cmaster, g_cfg->bus_clk_hz);

	i2c_set_user_callback(&g_cfg->i2cmaster, I2C_TX_COMPLETE, tof_i2c_callback_set_tx_complete_flag);
	if (sendStop == false) {
		i2c_restart_enable(&g_cfg->i2cmaster);
	}

	length = i2c_write(&g_cfg->i2cmaster, (int)g_ctx->tx_address, (char *)g_ctx->tx_buffer, (int)g_ctx->tx_buffer_length, (int)sendStop);
	hal_delay_us(g_ctx->tx_buffer_length * 200);
	if (sendStop == false) {
		i2c_restart_disable(&g_cfg->i2cmaster);
	}
	if ((g_ctx->tx_buffer_length > 0) && (length <= 0)) {
		error = 1;
	}
	if (g_ctx->tx_complete_flag == 0) {
		error = -1;    // Error for wrong slave address
	} else {
		error = 0;
	}
	g_ctx->tx_buffer_length = 0;    // empty buffer

	return error;
}

size_t tof_i2c_write_data(uint8_t data)
{
	if (g_ctx->tx_buffer_length >= g_cfg->i2c_data_length) {
		return 0;
	}
	g_ctx->tx_buffer[g_ctx->tx_buffer_length++] = data;

	return 1;
}

int tof_i2c_read_data(void)
{
	if (g_ctx->rx_buffer_index < g_ctx->rx_buffer_length) {
		return g_ctx->rx_buffer[g_ctx->rx_buffer_index++];
	}
	return -1;
}

uint8_t tof_i2c_request_from(uint8_t address, uint8_t quantity)
{
	uint8_t sendStop = 0x1;
	if (quantity >  g_cfg->i2c_data_length) {
		quantity =  g_cfg->i2c_data_length;
	}

	// perform blocking read into buffer
	i2c_read(&g_cfg->i2cmaster, ((int)address), (char *)(g_ctx->rx_buffer), ((int)quantity), ((int)sendStop));

	// set rx buffer iterator vars
	g_ctx->rx_buffer_index = 0;
	g_ctx->rx_buffer_length = quantity;

	return quantity;
}

int tof_i2c_available(void)
{
	return (g_ctx->rx_buffer_length - g_ctx->rx_buffer_index);
}

uint8_t tof_i2c_write_single_byte(uint16_t registerAddress, const uint8_t value)
{
	tof_i2c_begin_transmission(g_cfg->i2c_addr);
	tof_i2c_write_data(highByte(registerAddress));
	tof_i2c_write_data(lowByte(registerAddress));
	tof_i2c_write_data(value);
	return tof_i2c_end_transmission();
}

uint8_t tof_i2c_read_single_byte(uint16_t registerAddress)
{
	tof_i2c_begin_transmission(g_cfg->i2c_addr);
	tof_i2c_write_data(highByte(registerAddress));
	tof_i2c_write_data(lowByte(registerAddress));
	tof_i2c_end_transmission();
	tof_i2c_request_from(g_cfg->i2c_addr, 1U);
	hal_delay_ms(10);
	return tof_i2c_read_data();
}

// Must be able to write 32,768 bytes at a time
uint8_t tof_i2c_write_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
	// Chunk I2C transactions into limit of 32 bytes (or wireMaxPacketSize)
	uint8_t i2cError = 0;
	uint32_t startSpot = 0;
	uint32_t bytesToSend = bufferSize;
	while (bytesToSend > 0) {
		uint32_t len = bytesToSend;
		if (len > (g_cfg->i2c_max_packet_size - 2)) {	// Allow 2 byte for register address
			len = (g_cfg->i2c_max_packet_size - 2);
		}

		tof_i2c_begin_transmission(g_cfg->i2c_addr);
		tof_i2c_write_data(highByte(registerAddress));
		tof_i2c_write_data(lowByte(registerAddress));

		for (uint16_t x = 0; x < len; x++) {
			tof_i2c_write_data(buffer[startSpot + x]);	// Write a portion of the payload to the bus
		}

		i2cError = tof_i2c_end_transmission();	// Release bus because we are writing the address each time
		if (i2cError != 0) {
			return (i2cError);	// Sensor did not ACK
		}

		startSpot += len;	// Move the pointer forward
		bytesToSend -= len;
		registerAddress += len;	// Move register address forward
	}
	return (i2cError);
}

uint8_t tof_i2c_read_multiple_bytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
	uint8_t i2cError = 0;

	// Write address to read from
	tof_i2c_begin_transmission(g_cfg->i2c_addr);
	tof_i2c_write_data(highByte(registerAddress));
	tof_i2c_write_data(lowByte(registerAddress));
	i2cError =  tof_i2c_end_transmission_non_send_stop();	// Do not release bus
	if (i2cError != 0) {
		return (i2cError);
	}

	// Read bytes up to max transaction size
	uint16_t bytesToReadRemaining = bufferSize;
	uint16_t offset = 0;
	while (bytesToReadRemaining > 0) {
		// Limit to 32 bytes or whatever the buffer limit is for given platform
		uint16_t bytesToRead = bytesToReadRemaining;
		if (bytesToRead > g_cfg->i2c_max_packet_size) {
			bytesToRead = g_cfg->i2c_max_packet_size;
		}

		tof_i2c_request_from(g_cfg->i2c_addr, (uint8_t)bytesToRead);
		hal_delay_ms(10);
		if (tof_i2c_available()) {
			for (uint16_t x = 0; x < bytesToRead; x++) {
				buffer[offset + x] = tof_i2c_read_data();
			}
		} else {
			return (false);	// Sensor did not respond
		}

		offset += bytesToRead;
		bytesToReadRemaining -= bytesToRead;
	}

	return (0);	// Success
}