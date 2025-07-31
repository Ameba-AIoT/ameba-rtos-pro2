#include "PinNames.h"
#include "basic_types.h"
#include <osdep_api.h>

#include "i2c_api.h"
#include "i2c_ex_api.h"
#include "pinmap.h"
#include "wait_api.h"
#include "alc5680.h"

#define I2C_ALC5680_ADDR	        (0X5A/2)
#if defined(CONFIG_PLATFORM_8195A)
#define I2C_MTR_SDA			PB_3
#define I2C_MTR_SCL			PB_2
#elif defined(CONFIG_PLATFORM_8711B)
#define I2C_MTR_SDA			PA_30
#define I2C_MTR_SCL			PA_29
#endif
#define I2C_BUS_CLK			100000  //100K HZ

//i2c_t   alc5680_i2c;
#if defined (__ICCARM__)
i2c_t   alc5680_i2c;
#else
volatile i2c_t   alc5680_i2c;
#define printf  DBG_8195A
#endif

#define HELLO_BLUE_GENIE 0X03
#define ALEXA 0X04
#define SESAMEE 0X05
#define BAIDU 0X76

#define OLD_VERSION 0
#define NEW_VERSION 1

#define ALC_FW_START_ADDR 0X70000000

#if OLD_VERSION
#define header_offset 32 //[addr1][len1][addr2]][len2][addr3]][len3][size][checksum]
typedef struct _alc5680_fw {
	unsigned int alc_addr[3];
	unsigned int alc_len[3];
	unsigned int size;
	unsigned int checksum;
	unsigned int alc_addr_offset;
	unsigned int alc_len_offset;
	unsigned int alc_len_index;
} alc5680_fw;
#endif

#if NEW_VERSION
#define header_offset 12 //[size][checksum][version]
typedef struct _alc5680_fw {
	unsigned int version;
	unsigned int size;
	unsigned int checksum;
	unsigned int alc_addr_offset;
	unsigned int alc_len_offset;
} alc5680_fw;
#endif

static alc5680_fw alc5680_info;

static void alc5680_delay(void)
{
	int i;

	i = 10000;

	while (i) {
		i--;
		asm volatile("nop\n\t");
	}
}

u8 alc5680_reg_write_16(u16 reg, u16 val)
{
	int length = 0;
	char buf[4];
	buf[0] = (char)(reg >> 8);
	buf[1] = (char)(reg & 0xff);
	buf[2] = (char)(val >> 8);
	buf[3] = (char)(val & 0xff);

	length = i2c_write(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 4, 1);
	//alc5680_delay();
	return (length == 4) ? 0 : 1;
}

u8 alc5680_reg_write_32(u32 reg, u32 val)
{
	int tmp;
	char *buf = (char *)&tmp;
	u16 val1;
	u8 ret = 0;

	ret = alc5680_reg_write_16(0x0001, reg & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0002, reg >> 16);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0003, val & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0004, val >> 16);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0000, 0x0003);
	if (ret != 0) {
		printf("error\r\n");
	}

	return 0;

}//0x1800C02C

u8 alc5680_reg_write_16b(u32 reg, u16 val)
{
	int tmp;
	char *buf = (char *)&tmp;
	u16 val1;
	u8 ret = 0;

	ret = alc5680_reg_write_16(0x0001, reg & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0002, reg >> 16);
	if (ret != 0) {
		printf("error\r\n");
	}

	ret = alc5680_reg_write_16(0x0003, val & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}


	ret = alc5680_reg_write_16(0x0000, 0x0001);
	if (ret != 0) {
		printf("error\r\n");
	}

	return 0;

}

u8 alc5680_reg_read_32(u32 reg, u32 *val)
{
	unsigned int tmp;
	char *buf = (char *)&tmp;
	u16 val1;
	u8 ret = 0;

	ret = alc5680_reg_write_16(0x0001, reg & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}
	ret = alc5680_reg_write_16(0x0002, reg >> 16);
	if (ret != 0) {
		printf("error\r\n");
	}
	ret = alc5680_reg_write_16(0x0000, 0x0002);
	if (ret != 0) {
		printf("error\r\n");
	}

	buf[0] = 0x0003 >> 8;
	buf[1] = 0x0003 & 0xff;

	if (i2c_write(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) != 2) {
		DBG_8195A("alc5680_reg_read(): write register addr fail\n");
		ret = 1;
	}
	//alc5680_delay();

	buf[0] = 0xaa;
	buf[1] = 0xaa;

	if (i2c_read(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) < 2) {
		printf("alc5680_reg_read(): read register value fail\n");
		ret = 1;
	} else {
		*val = ((buf[1] & 0xFF) << 24) | ((buf[0] & 0xFF) << 16);
	}


	buf[0] = 0x0004 >> 8;
	buf[1] = 0x0004 & 0xff;

	if (i2c_write(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) != 2) {
		DBG_8195A("alc5680_reg_read(): write register addr fail\n");
		ret = 1;
	}

	if (i2c_read(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) < 2) {
		printf("alc5680_reg_read(): read register value fail\n");
		ret = 1;
	} else {
		*val |= ((buf[1] & 0xFF) << 8) | (buf[0] & 0xFF);
	}


	//alc5680_delay();
	return ret;

}

u8 alc5680_reg_read_16(u32 reg, u16 *val)
{
	int tmp;
	char *buf = (char *)&tmp;
	u16 val1;
	u8 ret = 0;

	ret = alc5680_reg_write_16(0x0001, reg & 0xffff);
	if (ret != 0) {
		printf("error\r\n");
	}
	ret = alc5680_reg_write_16(0x0002, reg >> 16);
	if (ret != 0) {
		printf("error\r\n");
	}
	ret = alc5680_reg_write_16(0x0000, 0x0002);
	if (ret != 0) {
		printf("error\r\n");
	}

	buf[0] = 0x0003 >> 8;
	buf[1] = 0x0003 & 0xff;

	if (i2c_write(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) != 2) {
		DBG_8195A("alc5680_reg_read(): write register addr fail\n");
		ret = 1;
	}
	//alc5680_delay();

	buf[0] = 0xaa;
	buf[1] = 0xaa;

	if (i2c_read(&alc5680_i2c, I2C_ALC5680_ADDR, &buf[0], 2, 1) < 2) {
		printf("alc5680_reg_read(): read register value fail\n");
		ret = 1;
	} else {
		*val = ((buf[0] & 0xFF) << 8) | (buf[1] & 0xFF);
	}
	//alc5680_delay();
	return ret;

}

void alc5680_i2c_init(void)
{
	i2c_init(&alc5680_i2c, I2C_MTR_SDA, I2C_MTR_SCL);
	i2c_frequency(&alc5680_i2c, I2C_BUS_CLK);
	memset(&alc5680_info, 0, sizeof(alc5680_fw));
}

void alc5680_erase()
{
	unsigned short value = 0;
	alc5680_reg_write_32(0x18009508, 0x00000001); //erase command
	while (1) {
		vTaskDelay(1000);
		alc5680_reg_read_16(0x18009508, &value);
		if ((value & 0x01) == 0x00) {
			printf("Erase finish %d\r\n", value);
			break;
		} else {
			printf("Erase ..... %x\r\n", value);
		}
		vTaskDelay(1000);
	}
}

void alc5680_write_data_to_flash(const unsigned char *data_value, unsigned int len, unsigned int addr)
{
	int i = 0;
	unsigned int value = 0;
	const unsigned char *ptr = data_value;
	for (int i = 0; i < len / 4; i++) {
		value = (ptr[0]) | (ptr[1] << 8) | (ptr[2] << 16) | (ptr[3] << 24);
		alc5680_reg_write_32(addr + i * 4, value);
		ptr += 4;
	}
}

int alc5680_get_version()
{
	unsigned int value = 0;
	alc5680_reg_read_32(0x7000A004, &value);
	value = value >> 24;
	if (value == HELLO_BLUE_GENIE) {
		printf("VERSION:HELLO_BLUE_GENIE\r\n");
	} else if (value == ALEXA) {
		printf("VERSION:ALEXA\r\n");
	} else if (value == SESAMEE) {
		printf("VERSION:Zhima kaimen\r\n");
	} else if (value == BAIDU) {
		printf("VERSION:Xiao du xiao du\r\n");
	} else {
		printf("NON-DEFINE VERSION ADDR=0X7000A004 %x\r\n", value);
	}
	return value;
}

void set_alc5680_volume(unsigned char left, unsigned char right)
{
	alc5680_reg_write_16b(0x1800C02C, left << 8 | right);
}
void get_alc5680_volume(unsigned short *value)
{
	alc5680_reg_read_16(0x1800C02C, value);
}

void alc5680_status()
{
	unsigned int value = 0;
	alc5680_reg_read_16(0x1800c1fe, &value);
	printf("codec id = %x\r\n", value);
	alc5680_reg_read_16(0x1800C0CA, &value);
	printf("status state = %x\r\n", value);
	alc5680_get_version();
}

/////////////////////////////////////////////////////

#if OLD_VERSION
void alc5680_get_info(unsigned char *buffer)
{
	alc5680_info.size = buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3];
	alc5680_info.checksum = buffer[4] << 24 | buffer[5] << 16 | buffer[6] << 8 | buffer[7];
	alc5680_info.version = buffer[8] << 24 | buffer[9] << 16 | buffer[10] << 8 | buffer[11];
	printf("codec size = %x checksum = %x version = %x\r\n", alc5680_info.size, alc5680_info.version);
}

void alc5680_handler_function(unsigned char *buffer, int len, unsigned int index)
{
	unsigned char *buf_index = buffer;
	unsigned int buf_len = len;

	if (index == 1) {
		alc5680_get_info(buf_index);
		buf_index += header_offset;
		buf_len = buf_len - header_offset;
		alc5680_info.alc_len_offset = 0;
		alc5680_info.alc_len_index = 0;
		alc5680_info.alc_addr_offset = ALC_FW_START_ADDR;
	}

	alc5680_write_data_to_flash(buf_index, buf_len, alc5680_info.alc_addr_offset);
	alc5680_info.alc_addr_offset += buf_len;
	alc5680_info.alc_len_offset += buf_len;

	if (len != 512) {
		printf(" alc_addr_offset = %x alc_len_offset = %x \r\n", alc5680_info.alc_addr_offset, alc5680_info.alc_len_offset);
	}
}


void alc5680_check_sum()
{
	unsigned int alc_checksum = 0;
	unsigned int i = 0, j = 0;
	unsigned int value = 0;
	unsigned int count = 0;
	unsigned int percent = alc5680_info.size / 100;
	unsigned int percent_count = 0;
	if (percent % 4) {
		percent = percent + (4 - (percent % 4));
	}
	printf("checksum start\r\n");

	for (i = ALC_FW_START_ADDR; i < ALC_FW_START_ADDR + alc5680_info.size; i += 4) {
		alc5680_reg_read_32(i, &value);
		alc_checksum += ((value & 0xff) + ((value & 0xff00) >> 8) + ((value & 0xff0000) >> 16) + ((value & 0xff000000) >> 24));
		count += 4;
		if (count % percent == 0) {
			printf("%d%%\r", percent_count);
			percent_count++;
		}
	}

	if (alc_checksum != alc5680_info.checksum) {
		printf("Checksum error\r\n");
	} else {
		printf("Checksum successful\r\n");
	}
	printf("bin_checksum %x alc_checksum = %x count = %d\r\n", alc5680_info.checksum, alc_checksum, count);
}
#endif

#if NEW_VERSION
void alc5680_get_info(unsigned char *buffer)
{
	alc5680_info.size = buffer[0] << 24 | buffer[1] << 16 | buffer[2] << 8 | buffer[3];
	alc5680_info.checksum = buffer[4] << 24 | buffer[5] << 16 | buffer[6] << 8 | buffer[7];
	alc5680_info.version = buffer[8] << 24 | buffer[9] << 16 | buffer[10] << 8 | buffer[11];
	printf("codec size = %x checksum = %x version = %x\r\n", alc5680_info.size, alc5680_info.checksum, alc5680_info.version);
}

void alc5680_handler_function(unsigned char *buffer, int len, unsigned int index)
{
	unsigned char *buf_index = buffer;
	unsigned int buf_len = len;

	if (index == 1) {
		alc5680_get_info(buf_index);
		buf_index += header_offset;
		buf_len = buf_len - header_offset;
		alc5680_info.alc_len_offset = 0;
		alc5680_info.alc_addr_offset = ALC_FW_START_ADDR;
	}

	alc5680_write_data_to_flash(buf_index, buf_len, alc5680_info.alc_addr_offset);
	alc5680_info.alc_addr_offset += buf_len;
	alc5680_info.alc_len_offset += buf_len;

	if (len != 512) {
		printf(" alc_addr_offset = %x alc_len_offset = %x \r\n", alc5680_info.alc_addr_offset, alc5680_info.alc_len_offset);
	}
}


void alc5680_check_sum()
{
	unsigned int alc_checksum = 0;
	unsigned int i = 0, j = 0;
	unsigned int value = 0;
	unsigned int count = 0;
	unsigned int percent = alc5680_info.size / 100;
	unsigned int percent_count = 0;
	if (percent % 4) {
		percent = percent + (4 - (percent % 4));
	}
	printf("checksum start\r\n");

	for (i = ALC_FW_START_ADDR; i < ALC_FW_START_ADDR + alc5680_info.size; i += 4) {
		alc5680_reg_read_32(i, &value);
		alc_checksum += ((value & 0xff) + ((value & 0xff00) >> 8) + ((value & 0xff0000) >> 16) + ((value & 0xff000000) >> 24));
		count += 4;
		if (count % percent == 0) {
			printf("%d%%\r", percent_count);
			percent_count++;
		}
	}

	if (alc_checksum != alc5680_info.checksum) {
		printf("Checksum error\r\n");
	} else {
		printf("Checksum successful\r\n");
	}
	printf("bin_checksum %x alc_checksum = %x count = %d\r\n", alc5680_info.checksum, alc_checksum, count);
}
#endif



