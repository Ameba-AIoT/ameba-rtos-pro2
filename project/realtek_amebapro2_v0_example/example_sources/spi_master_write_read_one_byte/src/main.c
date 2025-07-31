#include "device.h"
#include "diag.h"
#include "main.h"
#include "spi_api.h"

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

	/* SPI3 is as Slave */
	spi_init(&spi_slave,  SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
	spi_format(&spi_slave, 8, 3, 1);

	spi_init(&spi_master, SPI1_MOSI, SPI1_MISO, SPI1_SCLK, SPI1_CS);
	spi_format(&spi_master, 8, 3, 0);
	spi_frequency(&spi_master, 4000000);

	int TestingTimes = 16;
	int TestData     = 0;
	int ReadData     = 0;

	for (TestData = 0x01; TestData < TestingTimes; TestData++) {
		spi_slave_write(&spi_slave, TestData);
		ReadData = spi_master_write(&spi_master, ~TestData);
		dbg_printf("Slave write: %02X, Master read: %02X\n\r", TestData, ReadData);
		ReadData = spi_slave_read(&spi_slave);
		dbg_printf("Master  write: %02X, Slave read: %02X\n\r", ~TestData, ReadData);
	}

	spi_free(&spi_master);
	spi_free(&spi_slave);

	dbg_printf("SPI Demo finished.\n\r");

	for (;;);

}

