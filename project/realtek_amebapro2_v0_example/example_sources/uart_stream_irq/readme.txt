Example Description

This example describes how to use UART to communicate with PC.

Required Components:
    USBtoTTL adapter

Connect to PC
    1, Connect Ground: connect to GND pin via USBtoTTL adapter
    2, Use UART2
        1), PE_2 as UART_RX connect to TX of USBtoTTL adapter
        2), PE_1 as UART_TX connect to RX of USBtoTTL adapter

Open Super terminal or Teraterm and set baud rate to 38400, 1 stopbit, no parity, no flow control.

This example shows:
    The RX data ready interrupt service routine is used to receive 8 characters from the PC, and then loopback them to the PC.
