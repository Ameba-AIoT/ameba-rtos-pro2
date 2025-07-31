#include "device.h"
#include "main.h"
#include "spi_api.h"
#include "spi_ex_api.h"

#define SPI_IS_AS_MASTER 0

// SPI0 (S0)
#define SPI0_MOSI  PE_3
#define SPI0_MISO  PE_2
#define SPI0_SCLK  PE_1
#define SPI0_CS    PE_4

extern void hal_ssi_toggle_between_frame(phal_ssi_adaptor_t phal_ssi_adaptor, u8 ctl);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	int TestingTimes = 10;
	int Counter      = 0;
	int TestData     = 0;

	dbg_printf("\r\n   SPI Twoboard DEMO   \r\n");

#if SPI_IS_AS_MASTER
	spi_t spi_master;

	spi_init(&spi_master, SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
	spi_format(&spi_master, DfsSixteenBits, ((int)SPI_SCLK_IDLE_LOW | (int)SPI_SCLK_TOGGLE_MIDDLE), 0);
	spi_frequency(&spi_master, 1000000);
	hal_ssi_toggle_between_frame(&spi_master.hal_ssi_adaptor, ENABLE);

	dbg_printf("--------------------------------------------------------\n\r");
	for (Counter = 0, TestData = 0xFF; Counter < TestingTimes; Counter++) {
		spi_master_write(&spi_master, TestData);
		dbg_printf("Master write: %02X\n\r", TestData);
		TestData--;
	}
	spi_free(&spi_master);

#else
	spi_t spi_slave;

	spi_init(&spi_slave, SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
	spi_format(&spi_slave, DfsSixteenBits, ((int)SPI_SCLK_IDLE_LOW | (int)SPI_SCLK_TOGGLE_MIDDLE), 1);
	hal_ssi_toggle_between_frame(&spi_slave.hal_ssi_adaptor, ENABLE);

	dbg_printf("--------------------------------------------------------\n\r");
	for (Counter = 0, TestData = 0xFF; Counter < TestingTimes; Counter++) {
		dbg_printf("Slave  read : %02X\n\r", spi_slave_read(&spi_slave));
		TestData--;
	}
	spi_free(&spi_slave);
#endif

	dbg_printf("SPI Demo finished.\n\r");
	for (;;);
}
