#include "imu/inv_imu_driver.h"

typedef void (*icm32670p_event_cb)(inv_imu_sensor_event_t *event);

uint8_t icm42670p_i2c_init(void);
uint8_t icm42670p_i2c_init_with_pin(uint32_t sda, uint32_t scl);
int icm42670p_read_fifo(icm32670p_event_cb event);
int icm42670p_reset_fifo(void);
