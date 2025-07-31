Example Description

This example describes how to use SPI read/write DMA mode by MBED API.

The SPI Interface provides a "Serial Peripheral Interface" Master.

This interface can be used for communication with SPI Slave devices, such as FLASH memory, LCD screens, and other modules or integrated circuits.

In this example, we use config SPI_IS_AS_Master to decide if the device is Master or Slave.
    If SPI_IS_AS_Master is 1, then the device is Master.
    If SPI_IS_AS_Master is 0, then the device is Slave.

Connections:
    Master board                <---------->       Slave board
    Master's MOSI (PE_3)        <---------->       Slave's MOSI (PE_3)
    Master's MISO (PE_2)        <---------->       Slave's MISO (PE_2)
    Master's SCLK (PE_1)        <---------->       Slave's SCLK (PE_1)
    Master's CS   (PE_4)        <---------->       Slave's CS   (PE_4)

This example shows Master sends data to Slave in DMA mode.
We bootup Slave first, and then bootup Master.
Then log will present that Master sending data to Slave.

