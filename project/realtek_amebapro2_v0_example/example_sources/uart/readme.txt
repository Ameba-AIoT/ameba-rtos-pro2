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
User input will be received by the demo board, and demo board will loopback the received character with a prompt string "8735B$".


