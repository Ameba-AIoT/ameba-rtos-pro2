
.PHONY: all clean mksubdir build_lib post_process obj_copy

MKDIR= mkdir

ROOT_DIR= $(PWD)/..

ifeq ($(shell uname), Linux)
ASDK_DIR=/toolchain/asdk-10.3.0/linux/newlib
else
ASDK_DIR=/asdk-10.3.0/mingw32/newlib
endif

OBJ_DIR	= $(PWD)/cmsis/voe/source/obj
OUTPUT_DIR = $(PWD)/cmsis/voe/source/output
export ASDK_DIR ROOT_DIR OBJ_DIR

CC	= $(ASDK_DIR)/bin/arm-none-eabi-gcc  
AR		= $(ASDK_DIR)/bin/arm-none-eabi-ar
AS		= $(ASDK_DIR)/bin/arm-none-eabi-gcc
LD		= $(ASDK_DIR)/bin/arm-none-eabi-gcc 
NM		= $(ASDK_DIR)/bin/arm-none-eabi-nm
STRIP	= $(ASDK_DIR)/bin/arm-none-eabi-strip
OBJDUMP	= $(ASDK_DIR)/bin/arm-none-eabi-objdump
OBJCOPY	= $(ASDK_DIR)/bin/arm-none-eabi-objcopy
CC_SIZE	= $(ASDK_DIR)/bin/arm-none-eabi-size

export CC AR AS LD NM STRIP OBJDUMP OBJCOPY CC_SIZE


OUTLIB = 
all:
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm voe.bin $(OBJCOPYDIR)/voe.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm voe_fcs.bin $(OBJCOPYDIR)/voe_fcs.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm fcs_data.bin $(OBJCOPYDIR)/fcs_data.o;
	#$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor.bin $(OBJCOPYDIR)/sensor.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_gc2053.bin $(OBJCOPYDIR)/sensor_gc2053.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_gc4023.bin $(OBJCOPYDIR)/sensor_gc4023.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_fixp.bin $(OBJCOPYDIR)/sensor_fixp.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_sc2310.bin $(OBJCOPYDIR)/sensor_sc2310.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_sc301.bin $(OBJCOPYDIR)/sensor_sc301.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_fixp2K.bin $(OBJCOPYDIR)/sensor_fixp2K.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_fixp5M.bin $(OBJCOPYDIR)/sensor_fixp5M.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_fixp4M169.bin $(OBJCOPYDIR)/sensor_fixp4M169.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551c.bin $(OBJCOPYDIR)/sensor_rs0551c.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551c2K.bin $(OBJCOPYDIR)/sensor_rs0551c2K.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551c4M.bin $(OBJCOPYDIR)/sensor_rs0551c4M.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551c60fps.bin $(OBJCOPYDIR)/sensor_rs0551c60fps.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551c1360X768.bin $(OBJCOPYDIR)/sensor_rs0551c1360X768.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551cYUV.bin $(OBJCOPYDIR)/sensor_rs0551cYUV.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_gc4653.bin $(OBJCOPYDIR)/sensor_gc4653.o;	
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_ps5258.bin $(OBJCOPYDIR)/sensor_ps5258.o;	
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_ps5270.bin $(OBJCOPYDIR)/sensor_ps5270.o;	
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_imx307.bin $(OBJCOPYDIR)/sensor_imx307.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_imx307hdr.bin $(OBJCOPYDIR)/sensor_imx307hdr.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_rs0551cft2.bin $(OBJCOPYDIR)/sensor_rs0551cft2.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_k05.bin $(OBJCOPYDIR)/sensor_k05.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_sc2336.bin $(OBJCOPYDIR)/sensor_sc2336.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm sensor_imx327.bin $(OBJCOPYDIR)/sensor_imx327.o;	
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm iq.bin $(OBJCOPYDIR)/iq.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm iq_ext.bin $(OBJCOPYDIR)/iq_ext.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm iq_linear.bin $(OBJCOPYDIR)/iq_linear.o;		
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm voeiram.bin $(OBJCOPYDIR)/voeiram.o;
	$(OBJCOPY) -I binary -O elf32-littlearm -B arm fcs_data_ccut.bin $(OBJCOPYDIR)/fcs_data_ccut.o;	

