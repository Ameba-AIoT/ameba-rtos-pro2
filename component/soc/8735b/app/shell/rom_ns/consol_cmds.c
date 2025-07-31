/**************************************************************************//**
 * @file     consol_cmds.c
 * @brief    Some commands implementation for the shell command. It provides
 *           some basic memory write and dump commands.
 * @version  V1.00
 * @date     2022-06-09
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
#include "cmsis.h"
#include "shell.h"
#include "diag.h"

#define SECTION_CCMD_TEXT         SECTION(".ccmd.text")
#define SECTION_CCMD_DATA         SECTION(".ccmd.data")
#define SECTION_CCMD_RODATA       SECTION(".ccmd.rodata")
#define SECTION_CCMD_BSS          SECTION(".ccmd.bss")

extern u32 _strtoul(const char *nptr, char **endptr, int base);
extern void dump_for_one_bytes(u8 *pdata, u32 len);

s32 cmd_dump_byte(u32 argc, u8 *argv[]);
s32 cmd_dump_helfword(u32 argc, u8 *argv[]);
s32 cmd_dump_word(u32 argc, u8 *argv[]);
s32 cmd_write_byte(u32 argc, u8 *argv[]);
s32 cmd_write_word(u32 argc, u8 *argv[]);

#if 1
s32 cmd_bit_read(u32 argc, u8 *argv[]);
s32 cmd_bit_write(u32 argc, u8 *argv[]);
#endif

SECTION_CCMD_RODATA
const shell_command_entry_t rom_cmd_table[] = {
#if CONFIG_FPGA && (CONFIG_SIMU_ASIC_FLOW_EN == 0) // FPGA  
	{
		"ROM", (const char *)"DB", (shell_program_t)cmd_dump_byte, (const char *)"DB <Address, Hex> <Len, Dec>: \r\n"
		"\t\t\t\tDump memory byte or Read Hw byte register"
	},
	{
		"ROM", (const char *)"DHW", (shell_program_t)cmd_dump_helfword, (const char *)"DHW <Address, Hex> <Len, Dec>: \r\n"
		"\t\t\t\tDump memory helf-word or Read Hw helf-word register;"
	},
	{
		"ROM", (const char *)"DW", (shell_program_t)cmd_dump_word, (const char *)"DW <Address, Hex> <Len, Dec>:\r\n"
		"\t\t\t\tDump memory word or Read Hw word register; "
	},
	{
		"ROM", (const char *)"EB", (shell_program_t)cmd_write_byte, (const char *)"EB <Address, Hex> <Value, Hex>: \r\n"
		"\t\t\t\tWrite memory byte or Write Hw byte register \r\n"
		"\t\t\t\tSupports multiple byte writting by a single command \r\n"
		"\t\t\t\tEx: EB Address Value0 Value1"
	},
	{
		"ROM", (const char *)"EW", (shell_program_t)cmd_write_word, (const char *)"EW <Address, Hex> <Value, Hex>: \r\n"
		"\t\t\t\tWrite memory word or Write Hw word register \r\n"
		"\t\t\t\tSupports multiple word writting by a single command \r\n"
		"\t\t\t\tEx: EW Address Value0 Value1"
	},
	{"ROM", (const char *)"BR", (shell_program_t)cmd_bit_read, (const char *)"BR <Address, Hex> <Bit2, Dec>  <Bit1, Dec> [:] [exp data]"},
	{"ROM", (const char *)"BW", (shell_program_t)cmd_bit_write, (const char *)"BW <Address, Hex> <Bit2, Dec>  <Bit1, Dec> <Value, Hex>"},
#else   // PXP & ASIC
	{
		"ROM", (const char *)"DB", (shell_program_t)cmd_dump_byte, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"DHW", (shell_program_t)cmd_dump_helfword, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"DW", (shell_program_t)cmd_dump_word, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"EB", (shell_program_t)cmd_write_byte, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"EW", (shell_program_t)cmd_write_word, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"BR", (shell_program_t)cmd_bit_read, (const char *)" ???\r\n"
	},
	{
		"ROM", (const char *)"BW", (shell_program_t)cmd_bit_write, (const char *)" ???\r\n"
	},
#endif
	{"ROM", (const char *)NULL, (shell_program_t)NULL, (const char *)NULL}    // end of table
};

SECTION_CCMD_RODATA
const u32 mem_access_abandon_range[] = {
	/*  start addr      end addr    */
//    0x40000810,     0x40000814,
//    0x50000810,     0x50000814,

	/* end of list */
	0xFFFFFFFF,     0xFFFFFFFF
};

SECTION_CCMD_TEXT
s32 _cmd_mem_range_validate(u32 addr, u32 len)
{
	u32 i;
	u32 end_addr;
	s32 valid = 1;

	i = 0;
	if (len <= 4) {
		while (mem_access_abandon_range[i] != 0xFFFFFFFF) {
			if ((addr >= mem_access_abandon_range[i]) && (addr <= mem_access_abandon_range[i + 1])) {
				valid = 0;
				break;
			}
			i = i + 2;
		}
	} else {
		end_addr = addr + len;
		while (mem_access_abandon_range[i] != 0xFFFFFFFF) {
			if (((addr >= mem_access_abandon_range[i]) && (addr <= mem_access_abandon_range[i + 1])) ||
				((end_addr >= mem_access_abandon_range[i]) && (end_addr <= mem_access_abandon_range[i + 1]))) {
				valid = 0;
				break;
			}
			i = i + 2;
		}
	}

	return valid;
}

SECTION_CCMD_TEXT
s32 cmd_dump_byte(u32 argc, u8 *argv[])
{
	u32 src;
	u32 len;

	if (argc < 1) {
		dbg_printf("Wrong argument number!\r\n");
		return FALSE;
	}

	src = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	if (argc > 1) {
		if (!argv[1]) {
			len = 16;
		} else {
			len = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
		}
	} else {
		len = 16;
	}

	dump_for_one_bytes((u8 *)src, len);

	return _TRUE ;
}

SECTION_CCMD_TEXT
s32 cmd_dump_helfword(u32 argc, u8  *argv[])
{
	u32 src;
	u32 len;
	u32 i;

	if (argc < 1) {
		dbg_printf("Wrong argument number!\r\n");
		return _FALSE;
	}

	if (argv[0]) {
		src = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	} else {
		dbg_printf("Wrong argument number!\r\n");
		return _FALSE;
	}

	if (argc > 1) {
		if (!argv[1]) {
			len = 1;
		} else {
			len = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
		}
	} else {
		len = 1;
	}

	while ((src) & 0x01) {
		src++;
	}

	for (i = 0; i < len; i += 4, src += 16) {
		dbg_printf("%08X:  %04X    %04X    %04X    %04X    %04X    %04X    %04X    %04X\r\n",
				   src, *(u16 *)(src), *(u16 *)(src + 2),
				   *(u16 *)(src + 4), *(u16 *)(src + 6),
				   *(u16 *)(src + 8), *(u16 *)(src + 10),
				   *(u16 *)(src + 12), *(u16 *)(src + 14));
	}

	return _TRUE;

}

SECTION_CCMD_TEXT
s32 cmd_dump_word(u32 argc, u8  *argv[])
{
	u32 src;
	u32 len;
	u32 i;

	if (argc < 1) {
		dbg_printf("Wrong argument number!\r\n");
		return _FALSE;
	}

	if (argv[0]) {
		src = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	} else {
		dbg_printf("Wrong argument number!\r\n");
		return _FALSE;
	}

	if (argc > 1) {
		if (!argv[1]) {
			len = 1;
		} else {
			len = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
		}
	} else {
		len = 1;
	}

	while ((src) & 0x03) {
		src++;
	}

	dbg_printf("\r\n");
	for (i = 0; i < len; i += 4, src += 16) {
		dbg_printf("%08X:    %08X", src, *(u32 *)(src));
		dbg_printf("    %08X", *(u32 *)(src + 4));
		dbg_printf("    %08X", *(u32 *)(src + 8));
		dbg_printf("    %08X\r\n", *(u32 *)(src + 12));
	}

	return _TRUE;
}

SECTION_CCMD_TEXT
s32 cmd_write_byte(u32 argc, u8  *argv[])
{
	u32 src, i;
	u8 value;

	src = _strtoul((const char *)(argv[0]), (char **)NULL, 16);

	for (i = 0; i < argc - 1; i++, src++) {
		if (_cmd_mem_range_validate(src, 1)) {
			value = _strtoul((const char *)(argv[i + 1]), (char **)NULL, 16);
			dbg_printf("0x%08X = 0x%02X\r\n", src, value);
			*(volatile u8 *)(src) = value;
		} else {
			dbg_printf("Addr(0x%x). in abandon range!\r\n", src);
		}
	}

	return 0;
}

SECTION_CCMD_TEXT
s32 cmd_write_word(u32 argc, u8  *argv[])
{
	u32 src;
	u32 value, i;

	src = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	while ((src) & 0x03) {
		src++;
	}

	for (i = 0; i < (argc - 1); i++, src += 4) {
		if (_cmd_mem_range_validate(src, 4)) {
			value = _strtoul((const char *)(argv[i + 1]), (char **)NULL, 16);
			dbg_printf("0x%08X = 0x%08X\r\n", src, value);
			*(volatile u32 *)(src) = value;
		} else {
			dbg_printf("Addr(0x%x). in abandon range!\r\n", src);
		}
	}

	return 0;
}

#if 1
SECTION_CCMD_TEXT
s32 cmd_bit_read(u32 argc, u8  *argv[])
{
	u32 addr = 0x80;
	u32 b2 = 31, b1 = 0;
	u32 chk = 0;
	u32 exp = 0;

	if (argc < 1) {
		dbg_printf("Ex: br <addr> <msb> <lsb> [:] [expect]\r\n");
		dbg_printf("Ex: br 0x80 31 15\r\n");
		dbg_printf("Ex: br 0x80 31 15 : 0x1234\r\n");
		return _FALSE;
	}

	if (argc > 0) {
		addr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	}
	if (argc > 1) {
		b2 = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}
	if (argc > 2) {
		b1 = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}
	if (argc > 3) {
		//chk= _strtoul((const char*)(argv[3]), (char **)NULL, 16);
		if (*argv[3] == ':') {
			chk = 1;
		}
	}

	if (argc > 4) {
		exp = _strtoul((const char *)(argv[4]), (char **)NULL, 16);
	}


	unsigned int tmp;
	tmp =  *(u32 *)(addr);
	tmp = (tmp << (31 - b2)) >> (31 - b2);
	tmp = tmp >> b1;

	if (chk) {
		if (tmp == exp) {
			dbg_printf("...pass\r\n");
		} else {
			dbg_printf("...fail, read_data=%x, exp_data=%x\r\n", tmp, exp);
		}

	} else {
		dbg_printf("%x\r\n", tmp);
	}

	return _TRUE;
}

SECTION_CCMD_TEXT
s32 cmd_bit_write(u32 argc, u8  *argv[])
{
	u32 addr = 0x80;
	u32 b2 = 31, b1 = 0;
	u32 wdata = 0;

	if (argc < 1) {
		dbg_printf("Ex: bw <addr> <msb> <lsb> <data>\r\n");
		dbg_printf("Ex: bw 0x80 31 15 0x1234\r\n");
		return _FALSE;
	}

	if (argc > 0) {
		addr = _strtoul((const char *)(argv[0]), (char **)NULL, 16);
	}
	if (argc > 1) {
		b2 = _strtoul((const char *)(argv[1]), (char **)NULL, 10);
	}
	if (argc > 2) {
		b1 = _strtoul((const char *)(argv[2]), (char **)NULL, 10);
	}
	if (argc > 3) {
		wdata = _strtoul((const char *)(argv[3]), (char **)NULL, 16);
	}


	unsigned int rdata, tmp;
	rdata =  *(u32 *)(addr);
	tmp = (rdata << (31 - b2)) >> (31 - b2);
	tmp = ((tmp >> b1) << b1);

	tmp = (rdata ^ tmp) | ((wdata) << b1);

	*(u32 *)(addr) = tmp;

	return 0;
}
#endif

