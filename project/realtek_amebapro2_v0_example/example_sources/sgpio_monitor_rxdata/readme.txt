Example Description

This example describes how to monitor the decoded data. 

Required Components:
 - Connect SGPIO TX and SGPIO RX  
 
Output Waveform:

	    Bit 0		          Bit1
--------            ---------      ------      --------------
       |            |  			|      |	
       |<---60us--->|                   |<10us>|		
       |            |	                |      |
       --------------                   --------
       <-------70us------->             <-------70us------->
       
This example shows:
 1. Initilize SGPIO. 
 2. Initilize SGPIO TX to generate the encoded waveform.
 3. Initilize SGPIO RX to decode data.
 4. Set the monitored data 0xBBAA. When RX data matches 0xBBAA, generate the interrupt. 
 5. Send data 0xDDCCBBEE. Only print "0xddccbbee TX END..".
 6. Send data 0xDDCCBBAA. Print "0xddccbbaa TX END.." and "Match the monitor data". 