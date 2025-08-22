Example Description

This example describes how to implement a high/low level trigger on 1 GPIO pin.

Pin name PA_3 and PE_0 map to GPIOA_3 and GPIOE_0:
Connect PA_3 and PE_0 
    1, PA_3 as GPIO input high/low level trigger.
    2, PE_0 as GPIO output

In this example, PE_0 is a signal source that changes the level to high and low periodically.

PA_3 setup to listen to low level events in initial.
When PA_3 catch low level events, it disables the IRQ to avoid receiving duplicate events.
(NOTE: the level events will keep invoked if level keeps in the same level)

Then PA_3 is configured to listen to high level events and enable IRQ.
As PA_3 catches high level events, it changes back to listen to low level events.

Thus PA_3 can handle both high/low level events.

In this example, you will see a log that prints high/low level event periodically.

