Example Description

This example describes how to decode data by using the sampling function. 
Distinguish between bit0 and bit1 according to sampling high or low.

Required Components:
 - Connect SGPIO TX and SGPIO RX   
 
Output Waveform:

	    Bit 0		          Bit1
	    
       <-------70us------->             <-------70us------->
--------            ---------      ------      --------------
       |            |  			|      |	
       |<---60us--->|                   |<10us>|		
       |            |	                |      |
       --------------                   --------
       <--35us-->                       <--35us-->
                ^                                ^
                Sampling "0"                     Sampling "1"  
       
This example shows:
 1. Initilize SGPIO. 
 2. Initilize SGPIO RX to decode data.
 3. Initilize SGPIO TX to generate the encoded waveform.
 4. Show the decode data. Print "rxdata[0]: 0xddccbbaa, rxdata[1]: 0x44332211". 