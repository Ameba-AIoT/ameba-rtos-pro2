Example Description

This example describes how to use Power Mode API for DeepSleep.

Requirement Components:
    1, wake up by Stimer
        NONE
    2, wake up by GPIO
        a push button
        a resister

In this example,
    When wake up by Sitmer,
        1, The system will enter DeepSleep mode by 5s and then reboot the system.
    When wake up by GPIO,
        a), if there is configured GPIO interrupt.
            1, The system will enter DeepSleep mode by 5s.
            2, if the GPIO interrupt has been activated the system will be rebooted.
        b), if there is no configured GPIO interrupt.
            1, PA_2 connect to a resistor series to 3.3V. PA_2 also connect to a push button then series to GND.
            2, The system will enter DeepSleep mode by 5s.
            3, Please set PA_2 as GPIO interrupt for waking up the system. When the push button is pressed, the system will be rebooted.

Note
    1, Supported GPIO pins, group A.