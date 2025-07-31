Example Description

This example describes how to use the counter mode.
When give external triggers, multc counts value. 
 
Required Components:
 - Connect SGPIO TX and SGPIO RX 

Output Waveform:
 
   -----   -----   -----   -----   -----   -----   -----   -----    
   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
 ---   -----   -----   -----   -----   -----   -----   -----   -----  
   ^       ^       ^       ^       ^       ^       ^       ^
   1       2       3       4       5->0    1       2       3
 
This example shows:
 1. Initilize SGPIO. 
 2. Set the counter mode. When multc is 5, happen the interrupt and reset multc. 
 3. Give 8 rising edge triggers. Happen the match interrupt and reset multc.  
 4. Print "multc match event, multc is 5.". 
 5. Print "Because the match event happened, the count value is 3.".     