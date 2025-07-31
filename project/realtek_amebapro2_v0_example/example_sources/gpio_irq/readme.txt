Example Description

This example describes how to use GPIO read/write by MBED API.

Requirement Components:
    a LED
    a push button
    a resister

Pin name PE_0 and PA_3 map to GPIOE_0 and GPIOA_3:
    1, PA_3 as input with internal pull-high, connect a resistor to ground in series. PA_3 also connect to a push button and the other side of the push button connected to 3.3V.
    2, PE_0 as output, connect a LED to GND in series.

In this example, push the button to trigger interrupt to turn on/off the LED.

