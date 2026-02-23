##################################################################################
#                                                                                #
#                           example_pir_sensor                                 	 #
#                                                                                #
##################################################################################

Table of Contents
~~~~~~~~~~~~~~~~~
 - Description
 - Setup Guide
 - Result description
 - Supported List

 
Description
~~~~~~~~~~~
        This example demonstrates how to interface with a PIR sensor via I2C. It runs a persistent loop to continuously poll the PIR sensor registers to retrieve detection data.


Setup Guide
~~~~~~~~~~~
        1. cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DSCENARIO=pir_sensor
        2. Connect the PIR sensor SDA (PE_4) and SCL (PE_3) to the AmebaPro2 I2C interface.
        3. Download and burn the firmware into the EVB board.


Result description
~~~~~~~~~~~~~~~~~~
        The console log will continuously show the raw values or detection status read from the PIR sensor.


Supported List
~~~~~~~~~~~~~~
[Supported List]
	Supported :
	    AmebaPro2

