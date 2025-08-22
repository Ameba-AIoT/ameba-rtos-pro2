#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include <platform_opts.h>
#include "i2c_api.h"
#include "ex_api.h"
#include "imu/inv_imu_driver.h"
#include "icm42670p_hal.h"

static i2c_t i2cmaster;
static int icm42670p_inited = 0;
#define ICM42670P_I2C_ADDR 0x68
#define ICM42670P_I2C_MTR_SDA PF_2 //PE_4
#define ICM42670P_I2C_MTR_SCL PF_1 //PE_3
#define ICM42670P_I2C_BUS_CLK 400000

static xSemaphoreHandle i2c_rx_done_sema = NULL;
static xSemaphoreHandle i2c_tx_done_sema = NULL;

static void i2c_master_rxc_callback(void *userdata)
{
	xSemaphoreGiveFromISR(i2c_rx_done_sema, NULL);
}

static void i2c_master_txc_callback(void *userdata)
{
	xSemaphoreGiveFromISR(i2c_tx_done_sema, NULL);
}

static struct inv_imu_serif icm_serif = {0};
static struct inv_imu_device icm_driver = {0};
static inv_imu_interrupt_parameter_t int1_config = {0};

extern void i2c_set_user_callback(i2c_t *obj, I2CCallback i2ccb, void(*i2c_callback)(void *));
static int icm42670p_i2c_read(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);
static int icm42670p_i2c_write(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);
uint8_t icm42670p_i2c_init(void)
{
	if (icm42670p_inited == 1) {
		return 0;
	}
	i2c_rx_done_sema = xSemaphoreCreateBinary();
	i2c_tx_done_sema = xSemaphoreCreateBinary();
	i2c_init(&i2cmaster, ICM42670P_I2C_MTR_SDA, ICM42670P_I2C_MTR_SCL);
	i2c_frequency(&i2cmaster, ICM42670P_I2C_BUS_CLK);
	i2c_set_user_callback(&i2cmaster, I2C_RX_COMPLETE, i2c_master_rxc_callback);
	i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_master_txc_callback);
	icm_serif.serif_type = UI_I2C;
	icm_serif.read_reg  = icm42670p_i2c_read;
	icm_serif.write_reg = icm42670p_i2c_write;
	icm_serif.context = &i2cmaster;
	icm_serif.max_read = 2560; //TBD
	icm_serif.max_write = 2560; //TBD

	int rc = inv_imu_init(&icm_driver, &icm_serif, NULL);
	if (rc != INV_ERROR_SUCCESS) {
		printf("ICM42670P or ICM42607P: inv_imu_init failed.\r\n");
		return rc;
	}

	/* Check WHOAMI */
	uint8_t who_am_i = 0xff;
	rc = inv_imu_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != 0) {
		printf("ICM42670P or ICM42607P: inv_imu_get_who_am_i failed.\r\n");
		return -2;
	}
	if (who_am_i != INV_IMU_WHOAMI_ICM42670 && who_am_i != INV_IMU_WHOAMI_ICM42607) {
		printf("ICM42670P(0x%02x) or ICM42607P(0x%02x): WHOAMI mismatch.\r\n", INV_IMU_WHOAMI_ICM42670, INV_IMU_WHOAMI_ICM42607);
		return -3;
	}

	/*
	* Configure interrupts pins
	* - Polarity High
	* - Pulse mode
	* - Push-Pull drive
	*/
	inv_imu_int1_pin_config_t int1_pin_config;
	int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
	int1_pin_config.int_mode     = INT_CONFIG_INT1_MODE_PULSED;
	int1_pin_config.int_drive    = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
	inv_imu_set_pin_config_int1(&icm_driver, &int1_pin_config);

	// Enable FIFO
	inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_ENABLED);

	// Set Gyro mode
	rc = inv_imu_set_gyro_fsr(&icm_driver, GYRO_CONFIG0_FS_SEL_2000dps);
	rc |= inv_imu_set_gyro_frequency(&icm_driver, GYRO_CONFIG0_ODR_800_HZ);
	rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver);
	if (rc != 0) {
		printf("ICM42670P or ICM42607P: set Gyro mode failed.\r\n");
		return rc;
	}

	icm42670p_inited = 1;
	return 0;
}

uint8_t icm42670p_i2c_init_with_pin(uint32_t sda, uint32_t scl)
{
	if (icm42670p_inited == 1) {
		return 0;
	}
	i2c_rx_done_sema = xSemaphoreCreateBinary();
	i2c_tx_done_sema = xSemaphoreCreateBinary();
	i2c_init(&i2cmaster, sda, scl);
	i2c_frequency(&i2cmaster, ICM42670P_I2C_BUS_CLK);
	i2c_set_user_callback(&i2cmaster, I2C_RX_COMPLETE, i2c_master_rxc_callback);
	i2c_set_user_callback(&i2cmaster, I2C_TX_COMPLETE, i2c_master_txc_callback);
	icm_serif.serif_type = UI_I2C;
	icm_serif.read_reg  = icm42670p_i2c_read;
	icm_serif.write_reg = icm42670p_i2c_write;
	icm_serif.context = &i2cmaster;
	icm_serif.max_read = 2560; //TBD
	icm_serif.max_write = 2560; //TBD

	int rc = inv_imu_init(&icm_driver, &icm_serif, NULL);
	if (rc != INV_ERROR_SUCCESS) {
		printf("ICM42670P or ICM42607P: inv_imu_init failed.\r\n");
		return rc;
	}

	/* Check WHOAMI */
	uint8_t who_am_i = 0xff;
	rc = inv_imu_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != 0) {
		printf("ICM42670P or ICM42607P: inv_imu_get_who_am_i failed.\r\n");
		return -2;
	}
	if (who_am_i != INV_IMU_WHOAMI_ICM42670 && who_am_i != INV_IMU_WHOAMI_ICM42607) {
		printf("ICM42670P(0x%02x) or ICM42607P(0x%02x): WHOAMI mismatch.\r\n", INV_IMU_WHOAMI_ICM42670, INV_IMU_WHOAMI_ICM42607);
		return -3;
	}

	/*
	* Configure interrupts pins
	* - Polarity High
	* - Pulse mode
	* - Push-Pull drive
	*/
	inv_imu_int1_pin_config_t int1_pin_config;
	int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
	int1_pin_config.int_mode     = INT_CONFIG_INT1_MODE_PULSED;
	int1_pin_config.int_drive    = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
	inv_imu_set_pin_config_int1(&icm_driver, &int1_pin_config);

	// Enable FIFO
	inv_imu_configure_fifo(&icm_driver, INV_IMU_FIFO_ENABLED);

	// Set Gyro mode
	rc = inv_imu_set_gyro_fsr(&icm_driver, GYRO_CONFIG0_FS_SEL_2000dps);
	rc |= inv_imu_set_gyro_frequency(&icm_driver, GYRO_CONFIG0_ODR_800_HZ);
	rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver);
	if (rc != 0) {
		printf("ICM42670P or ICM42607P: set Gyro mode failed.\r\n");
		return rc;
	}

	icm42670p_inited = 1;
	return 0;
}

int icm42670p_read_fifo(icm32670p_event_cb event)
{
	if (icm42670p_inited == 0) {
		printf("ICM42670P or ICM42607P: not inited.\r\n");
		return -1;
	}
	icm_driver.sensor_event_cb = event;
	return inv_imu_get_data_from_fifo(&icm_driver);
}

int icm42670p_reset_fifo(void)
{
	if (icm42670p_inited == 0) {
		printf("ICM42670P or ICM42607P: not inited.\r\n");
		return -1;
	}
	return inv_imu_reset_fifo(&icm_driver);
}

static int icm42670p_i2c_read(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	i2c_write(&i2cmaster, ICM42670P_I2C_ADDR, (char const *)&reg, 1, 1);
	xSemaphoreTake(i2c_tx_done_sema, portMAX_DELAY);

	i2c_read(&i2cmaster, ICM42670P_I2C_ADDR, (char *)rbuffer, rlen, 1);
	xSemaphoreTake(i2c_rx_done_sema, portMAX_DELAY);

	return 0;
}

static int icm42670p_i2c_write(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	uint8_t data[64] = {0};
	data[0] = reg;
	if (wlen > 63) {
		printf("icm42670p or icm42607p i2c_write: write length is over 63 bytes.\r\n");
		return -1;
	}
	for (int i = 0; i < wlen; i++) {
		data[i + 1] = wbuffer[i];
	}

	i2c_write(&i2cmaster, ICM42670P_I2C_ADDR, (char const *)data, wlen + 1, 1);
	xSemaphoreTake(i2c_tx_done_sema, portMAX_DELAY);
	return 0;
}
