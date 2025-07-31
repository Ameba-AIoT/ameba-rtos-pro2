Example Description

This example describes how to use SPI read/write by mbed api.


The SPI Interface provides a "Serial Peripheral Interface" Master.

This interface can be used for communication with SPI slave devices,
such as modules or integrated circuits.

In this example, it use 2 sets of SPI. One is master, the other is slave.
By default it use SPI0 as slave, and use SPI1 as master.
So we connect them as below:
    Connect SPI0_MOSI (PE_3) to SPI1_MOSI (PF_7)
    Connect SPI0_MISO (PE_2) to SPI1_MISO (PF_5)
    Connect SPI0_SCLK (PE_1) to SPI1_SCLK (PF_6)
    Connect SPI0_CS   (PE_4) to SPI1_CS   (PF_8)

This example aims to demonstrate how the spi master writes one byte data to the slave while receving one byte 
data from the slave.
The slave device keeps the data in its fifo until the master device writes a data to the slave.
The write operation of the master would provide clock to the slave so that the slave can push data in its fifo to the master.