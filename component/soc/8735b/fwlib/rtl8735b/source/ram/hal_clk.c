#include "cmsis.h"
#include "hal.h"

enum rts_isp_sensor_clock {
	CLK_NONE = 0,
	CLK_12M = 12000000,
	CLK_24M = 24000000,
	CLK_27M = 27000000,
	CLK_37M125 = 37125000,
	CLK_54M = 54000000,
	CLK_74M25 = 74250000,
};

void hclk_set_rate(uint32_t rate)
{
	SYSON_S_TypeDef *syson_s = SYSON_S;
	volatile uint32_t val;
	uint32_t start_time = 0;

	switch (rate) {
	case CLK_NONE:
		//5000_0834[29]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0834[11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0834[12]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[31]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		return;
#if 0
	case CLK_12M:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'000111
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (7 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'100
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (4 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'1100110011010
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0x199A << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1111
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (0x0F << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[12]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
#endif
	case CLK_24M:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'000111
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (7 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'100
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (4 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'1100110011010
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0x199A << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1110
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (0x0E << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[12]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
#if 0
	case CLK_37M125:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'000101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (5 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'011
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (3 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'0110011001101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0xCCD << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1100
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (0x0C << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[12]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
	case CLK_74M25:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'000101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (5 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'011
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (3 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'0110011001101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0xCCD << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (8 << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[11]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_D2_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
#endif
	case CLK_27M:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'001000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (8 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'110
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (6 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'0110011001101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0xCCD << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1110
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (0x0E << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[12]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
#if 0
	case CLK_54M:
		//5000_0834[12:11]=0
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_REG_CK_EN_D2_ISP | SYSON_S_BIT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[10:5]=6b'001000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_DIVN_SDM_ISP);
		val |= (8 << SYSON_S_SHIFT_DIVN_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0840[18:16]=3b'110
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0N_SDM_ISP);
		val |= (6 << SYSON_S_SHIFT_F0N_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0840[31:19]=13b'0110011001101
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL3;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_F0F_SDM_ISP);
		val |= (0xCCD << SYSON_S_SHIFT_F0F_SDM_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL3 = val;

		//5000_0838[4:1]=4b'0000
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		HAL_CLEAR_BIT(val, SYSON_S_MASK_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[31]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_ERC_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		hal_delay_us(1);

		//5000_0834[29]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_POW_PLL_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0838[4:1]=4b'1100
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL1;
		val |= (0x0C << SYSON_S_SHIFT_REG_CK_OUT_SEL_ISP1);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL1 = val;

		//5000_0834[12]=1
		val = syson_s->SYSON_S_REG_ISP_PLL_CTRL0;
		val |= (1 << SYSON_S_SHIFT_REG_CK_EN_ISP);
		syson_s->SYSON_S_REG_ISP_PLL_CTRL0 = val;

		//5000_0804[0]=0
		val = syson_s->SYSON_S_REG_SYS_ISO_CTRL;
		HAL_CLEAR_BIT(val, SYSON_S_BIT_R3_SET_ISP);
		syson_s->SYSON_S_REG_SYS_ISO_CTRL = val;
		break;
#endif
	default:
		printf("ERROR clock setting\n\r");
		return;
	}

	start_time = hal_read_curtime_us();
#if 0
	//polling pll ready
	while (!(syson_s->SYSON_S_REG_SYS_CLK_CTRL & SYSON_S_BIT_ISPPLL_RDY)) {
		if (hclk_timeout_chk(start_time) == 1) {
			printf("%s timeout\n\r", __func__);
			break;
		}

		hal_delay_us(1);
	}
#endif


}