Example Description:
	This example demonstrates the function of UART Auto Flow control.
	To run this example 2 boards have to be connected via UART.

Details:
	In this example, UART 2 will be enabled as the UART channel between the both boards.

	The first board that is powered on will be the TX side, the second board that is powered on will be the RX side. 
	The difference between the power on times must not be longer than 5 seconds.
	The RX side will make some delay every 16-byte received, by this way we can trigger the flow control mechanism.

Required Components:
	1. 2 EV boards
	2. jumper cables

Setup:
	UART and pin mapping:
		UART 2:
			PE2: UART 2  RX
			PE1: UART 2  TX
			PE3: UART 2  CTS
			PE4: UART 2  RTS

	Connection between the 2 boards:
	Board1                      <---------->        Board2
	UART2  CTS    PE3          <---------->        PE4  UART2  RTS
	UART2  RTS    PE4          <---------->        PE3  UART2  CTS
	UART2  RX     PE2          <---------->        PE1  UART2  TX
	UART2  TX     PE1          <---------->        PE2  UART2  RX
	GND                         <---------->        GND



