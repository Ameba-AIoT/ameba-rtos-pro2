
#include "device.h"
#include "diag.h"
#include "main.h"
#include "spi_api.h"

#define FakeMbedAPI  1

// SPI0 (S0)
#define SPI0_MOSI  PE_3
#define SPI0_MISO  PE_2
#define SPI0_SCLK  PE_1
#define SPI0_CS    PE_4

// SPI1 (S1)
#define SPI1_MOSI  PF_7
#define SPI1_MISO  PF_5
#define SPI1_SCLK  PF_6
#define SPI1_CS    PF_8


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
spi_t spi_master;
spi_t spi_slave;

void main(void)
{
#if FakeMbedAPI

	/* SPI0 is as Slave */
	//SPI0_IS_AS_SLAVE = 1;

	spi_init(&spi_master, SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_CS);
	spi_format(&spi_master, 8, 0, 0);
	spi_frequency(&spi_master, 200000);
	hal_ssi_toggle_between_frame(&(spi_master.hal_ssi_adaptor), ENABLE);

	spi_init(&spi_slave,  SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
	spi_format(&spi_slave, 8, 0, 1);
	spi_frequency(&spi_slave, 200000);
	hal_ssi_toggle_between_frame(&(spi_slave.hal_ssi_adaptor), ENABLE);

	int TestingTimes = 10;
	int Counter      = 0;
	int TestData     = 0;
	int ReadData     = 0;

	int result = 1;

	/**
	 * Master read/write, Slave read/write
	 */
	dbg_printf("SPI Demo Start.\n\r");
	dbg_printf("--------------------------------------------------------\n\r");

	for (Counter = 0, TestData = 0x01; Counter < TestingTimes; Counter++) {
		spi_slave_write(&spi_slave, TestData);
		ReadData = spi_master_write(&spi_master, TestData);
		DBG_SSI_INFO("Master write: %02X, read: %02X\n\r", TestData, ReadData);
		if (TestData != ReadData) {
			result = 0;
		}

		TestData++;

		ReadData = spi_slave_read(&spi_slave);
		DBG_SSI_INFO(ANSI_COLOR_CYAN"Slave  write: %02X, read: %02X\n\r"ANSI_COLOR_RESET, (TestData - 1), ReadData);
		if (TestData - 1 != ReadData) {
			result = 0;
		}
		TestData++;
	}

	/**
	 * Master write, Slave read
	 */
	dbg_printf("--------------------------------------------------------\n\r");
	for (Counter = 0, TestData = 0xFF; Counter < TestingTimes; Counter++) {
		spi_master_write(&spi_master, TestData);
		ReadData = spi_slave_read(&spi_slave);
		dbg_printf("Master write: %02X\n\r", TestData);
		DBG_SSI_INFO(ANSI_COLOR_CYAN"Slave  read : %02X\n\r"ANSI_COLOR_RESET, ReadData);
		if (TestData != ReadData) {
			result = 0;
		}
		TestData--;
	}

	spi_free(&spi_master);
	spi_free(&spi_slave);

	dbg_printf("SPI Demo finished.\n\r");
	dbg_printf("\r\nResult is %s\r\r", (result) ? "success" : "fail");

	for (;;);

#else  // mbed SPI API emulation

#endif

}

