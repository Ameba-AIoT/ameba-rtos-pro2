Example Description

This example describes how to use i2s by comparing rx data with tx pattern.

Using loopback mode.
Check rx data and tx pattern are the same value. 
(Maybe the data order shift because the tx and rx don't work at the same time.)

Print:

  Set TX Pattern
    1     2     3     4     5     6     7     8
    9     a     b     c     d     e     f    10
   11    12    13    14    15    16    17    18
   19    1a    1b    1c    1d    1e    1f    20
   21    22    23    24    25    26    27    28
   29    2a    2b    2c    2d    2e    2f    30
   31    32    33    34    35    36    37    38
   39    3a    3b    3c    3d    3e    3f    40

  Check RX Data
    1     2     3     4     5     6     7     8
    9     a     b     c     d     e     f    10
   11    12    13    14    15    16    17    18
   19    1a    1b    1c    1d    1e    1f    20
   21    22    23    24    25    26    27    28
   29    2a    2b    2c    2d    2e    2f    30
   31    32    33    34    35    36    37    38
   39    3a    3b    3c    3d    3e    3f    40