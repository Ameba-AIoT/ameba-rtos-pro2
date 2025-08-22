# Nor flash
- fw1_address --- 0x60000
- ISP_address_PT_ISP_IQ --- 0x460000
- NN_address_PT_NN_MDL --- 0x530000

## erase flash

>  ./uartfwburn.exe -p serial_port -b 2000000 -e chip -U -x 32

>  ./uartfwburn.exe -p serial_port -b 2000000 -e chip -U -x 32 -r

## system file bin, partial flash
> ./uartfwburn.exe -p serial_port -f system_files.bin -b 2000000 -x 32

> ./uartfwburn.exe -p serial_port -f system_files.bin -b 2000000 -U -x 32

## upload nn bin, partial flash
> ./uartfwburn.exe -p serial_port -f firmware.bin -b 2000000 -s fw1_address -x 32

> ./uartfwburn.exe -p serial_port -f firmware_isp_iq.bin -b 2000000 -s ISP_address_PT_ISP_IQ -x 32

> ./uartfwburn.exe -p serial_port -f nn_model.bin -b 2000000 -s NN_address_PT_NN_MDL -x 32 -r

## update isp bin, partial flash
> ./uartfwburn.exe -p serial_port -f firmware.bin -b 2000000 -s fw1_address -x 32

> ./uartfwburn.exe -p serial_port -f firmware_isp_iq.bin -b 2000000 -s ISP_address_PT_ISP_IQ -x 32 -r

## update flash bin
> ./uartfwburn.exe -p COM5 -f flash_ntz.nn.bin -b 2000000 -U -x 32

> ./uartfwburn.exe -p COM5 -f flash_ntz.nn.bin -b 2000000 -U -x 32 -r


### Nand flash
TBD