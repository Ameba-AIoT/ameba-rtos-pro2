/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * 20170309 refined by RTK
 */
#include "sgtl5000.h"
#include "PinNames.h"
#include "basic_types.h"
#include <osdep_api.h>

#include "i2c_api.h"
#include "i2c_ex_api.h"
#include "pinmap.h"

#define I2C_MTR_SDA            PB_3
#define I2C_MTR_SCL            PB_2
#define I2C_BUS_CLK            100000  //100K HZ

static i2c_t   sgtl5000_i2c;

static uint16_t ana_ctrl;
static uint8_t i2c_addr;

static uint8_t muted;

uint8_t sgtl5000_reg_write(uint16_t reg, uint16_t val)
{
	int length = 0;
	uint8_t buf[4];

	if (reg == CHIP_ANA_CTRL) {
		ana_ctrl = val;
	}

	buf[0] = (reg >> 8);
	buf[1] = (reg & 0xff);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xff);

	length = i2c_write(&sgtl5000_i2c, i2c_addr, (char *)buf, 4, 1);
	return (length == 4) ? 0 : 1;
}

uint8_t sgtl5000_reg_read(uint16_t reg, uint16_t *val)
{
	uint8_t buf[2];
	uint8_t ret = 0;

	do {
		buf[0] = (reg >> 8);
		buf[1] = (reg & 0xff);

		if (i2c_write(&sgtl5000_i2c, i2c_addr, (char *)buf, 2, 1) != 2) {
			ret = 1;
			break;
		}

		if (i2c_read(&sgtl5000_i2c, i2c_addr, &buf[0], 2, 1) < 2) {
			ret = 1;
			break;
		}

		*val = ((buf[0] & 0xFF) << 8) | (buf[1] & 0xFF);
	} while (0);

	return ret;
}

void sgtl5000_setAddress(uint8_t level)
{
	if (level == 0) {
		i2c_addr = SGTL5000_I2C_ADDR_CS_LOW;
	} else {
		i2c_addr = SGTL5000_I2C_ADDR_CS_HIGH;
	}
}

static void i2c_master_err_callback(void *userdata)
{
	DBG_8195A("I2C AckAddr:%d Err:%x",
			  sgtl5000_i2c.SalI2CHndPriv.SalI2CHndPriv.I2CAckAddr,
			  sgtl5000_i2c.SalI2CHndPriv.SalI2CHndPriv.ErrType
			 );
}

uint8_t sgtl5000_enable(void)
{
	uint16_t val = 0;
	uint8_t ret = 0;
	muted = 1;

	memset(&sgtl5000_i2c, 0x00, sizeof(sgtl5000_i2c));
	i2c_init(&sgtl5000_i2c, I2C_MTR_SDA, I2C_MTR_SCL);
	i2c_frequency(&sgtl5000_i2c, I2C_BUS_CLK);
	i2c_set_user_callback(&sgtl5000_i2c, I2C_ERR_OCCURRED, i2c_master_err_callback);

	// set I2C address
	sgtl5000_setAddress(0);  // CTRL_ADR0_CS is tied to GND
	vTaskDelay(5);

	ret = sgtl5000_reg_read(CHIP_ID, &val);
	if (ret == 0) {
		DBG_8195A("SGTL5000 CHIP ID:0x%04X\n", val);
	} else {
		DBG_8195A("Get SGTL5000 CHIP ID fail\n");
	}

	sgtl5000_reg_write(CHIP_ANA_POWER,     0x4060); // VDDD is externally driven with 1.8V
	sgtl5000_reg_write(CHIP_LINREG_CTRL,   0x006C); // VDDA & VDDIO both over 3.1V
	sgtl5000_reg_write(CHIP_REF_CTRL,      0x01F2); // VAG=1.575, normal ramp, +12.5% bias current
	sgtl5000_reg_write(CHIP_LINE_OUT_CTRL, 0x0F22); // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
	sgtl5000_reg_write(CHIP_SHORT_CTRL,    0x4446); // allow up to 125mA
	sgtl5000_reg_write(CHIP_ANA_CTRL,      0x0137); // enable zero cross detectors
	sgtl5000_reg_write(CHIP_ANA_POWER,     0x40FF); // power up: lineout, hp, adc, dac
	sgtl5000_reg_write(CHIP_DIG_POWER,     0x0073); // power up all digital stuff
	vTaskDelay(400);
	sgtl5000_reg_write(CHIP_LINE_OUT_VOL,  0x1D1D); // default approx 1.3 volts peak-to-peak
	sgtl5000_reg_write(CHIP_CLK_CTRL,      0x0004); // 44.1 kHz, 256*Fs
	sgtl5000_reg_write(CHIP_I2S_CTRL,      0x0130); // SCLK=32*Fs, 16bit, I2S format
	// default signal routing is ok?
	sgtl5000_reg_write(CHIP_SSS_CTRL,      0x0010); // ADC->I2S, I2S->DAC
	sgtl5000_reg_write(CHIP_ADCDAC_CTRL,   0x0000); // disable dac mute
	sgtl5000_reg_write(CHIP_DAC_VOL,       0x3C3C); // digital gain, 0dB
	sgtl5000_reg_write(CHIP_ANA_HP_CTRL,   0x7F7F); // set volume (lowest level)
	sgtl5000_reg_write(CHIP_ANA_CTRL,      0x0036);  // enable zero cross detectors

	return 0;
}

uint8_t sgtl5000_muteHeadphone(void)
{
	return sgtl5000_reg_write(CHIP_ANA_CTRL, ana_ctrl | (1 << 4));
}

uint8_t sgtl5000_unmuteHeadphone(void)
{
	return sgtl5000_reg_write(CHIP_ANA_CTRL, ana_ctrl & ~(1 << 4));
}

uint8_t sgtl5000_muteLineout(void)
{
	return sgtl5000_reg_write(CHIP_ANA_CTRL, ana_ctrl | (1 << 8));
}

uint8_t sgtl5000_unmuteLineout(void)
{
	return sgtl5000_reg_write(CHIP_ANA_CTRL, ana_ctrl & ~(1 << 8));
}

uint8_t sgtl5000_inputSelect(uint8_t n)
{
	uint8_t ret = 0;

	if (n == SGTL5000_AUDIO_INPUT_LINEIN) {
		ret =  sgtl5000_reg_write(CHIP_ANA_ADC_CTRL, 0x055);               // +7.5dB gain (1.3Vp-p full scale)
		ret &= sgtl5000_reg_write(CHIP_ANA_CTRL,     ana_ctrl | (1 << 2)); // enable linein
	} else if (n == SGTL5000_AUDIO_INPUT_MIC) {
		ret  = sgtl5000_reg_write(CHIP_MIC_CTRL,     0x0173);              // mic preamp gain = +40dB
		ret &= sgtl5000_reg_write(CHIP_ANA_ADC_CTRL, 0x088);               // input gain +12dB (is this enough?)
		ret &= sgtl5000_reg_write(CHIP_ANA_CTRL,     ana_ctrl & ~(1 << 2)); // enable mic
	}

	return ret;
}

uint8_t sgtl5000_setVolume(float val)
{
	int volumeInt = 0;

	volumeInt = (int)(val * 129 + 0.499);

	if (volumeInt == 0) {
		muted = 1;
		sgtl5000_reg_write(CHIP_ANA_HP_CTRL, 0x7F7F);
		return sgtl5000_muteHeadphone();
	} else if (volumeInt > 0x80) {
		volumeInt = 0;
	} else {
		volumeInt = 0x80 - volumeInt;
	}
	if (muted) {
		muted = 0;
		sgtl5000_unmuteHeadphone();
	}
	volumeInt = volumeInt | (volumeInt << 8);

	return sgtl5000_reg_write(CHIP_ANA_HP_CTRL, volumeInt);    // set volume
}

uint8_t sgtl5000_micGain(uint16_t dB)
{
	uint8_t ret;
	uint16_t preamp_gain, input_gain;

	if (dB >= 40) {
		preamp_gain = 3;
		dB -= 40;
	} else if (dB >= 30) {
		preamp_gain = 2;
		dB -= 30;
	} else if (dB >= 20) {
		preamp_gain = 1;
		dB -= 20;
	} else {
		preamp_gain = 0;
	}
	input_gain = (dB * 2) / 3;
	if (input_gain > 15) {
		input_gain = 15;
	}

	ret = sgtl5000_reg_write(CHIP_MIC_CTRL, 0x0170 | preamp_gain);
	ret &= sgtl5000_reg_write(CHIP_ANA_ADC_CTRL, (input_gain << 4) | input_gain);

	return ret;
}

