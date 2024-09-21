/*
 * Copyright (c) 2024, MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Bouffalo Lab RISC-V MCU series initialization code
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <soc.h>
#include <clic.h>

/* Set Embedded Flash Pullup */
static void system_bor_init(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(HBN_BASE + HBN_MISC_OFFSET);
	/* borThreshold = 1 */
	tmpVal = (tmpVal & HBN_BOR_VTH_UMSK) | ((uint32_t)(1) << HBN_BOR_VTH_POS);
	/* enablePorInBor true*/
	tmpVal = (tmpVal & HBN_BOR_SEL_UMSK) | ((uint32_t)(1) << HBN_BOR_SEL_POS);
	/* enableBor true*/
	tmpVal = (tmpVal & HBN_PU_BOR_UMSK) | ((uint32_t)(1) << HBN_PU_BOR_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_MISC_OFFSET);


	/* enableBorInt false */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	tmpVal = tmpVal & HBN_IRQ_BOR_EN_UMSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);
}

static uint32_t mtimer_get_clk_src_div(void)
{
	uint32_t bclk_div = -1;

	bclk_div = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	bclk_div = (bclk_div & GLB_REG_BCLK_DIV_MSK) >> GLB_REG_BCLK_DIV_POS;
	return ((sys_read32(CORECLOCKREGISTER) / (bclk_div + 1)) / 1000 / 1000 - 1);
}

static void system_clock_settle(void)
{
	__asm__ volatile (".rept 15 ; nop ; .endr");
}


static void system_clock_delay_32M_ms(uint32_t ms)
{
	uint32_t count = 0;

	do
	{
		__asm__ volatile (".rept 32 ; nop ; .endr");
		count++;
	} while (count < ms);
}

static uint32_t is_pds_busy(void)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	if (tmpVal & EF_CTRL_EF_IF_0_BUSY_MSK) {
		return 1;
	}
	return 0;
}

/* /!\ only use when running on 32Mhz Oscillator Clock
 * (system_set_root_clock(0);
 * system_set_root_clock_dividers(0, 0);
 * sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);)
 * Only Use with IRQs off
 * returns 0 when error
 */
static uint32_t system_efuse_read_32M(void)
{
	uint32_t tmpVal = 0;
	uint32_t tmpVal2 = 0;
	uint32_t *pefuse_start = (uint32_t *)(BFLB_EF_CTRL_BASE);
	uint32_t timeout = 0;

	do
	{
		system_clock_delay_32M_ms(1);
		timeout++;
	} while (timeout < EF_CTRL_DFT_TIMEOUT_VAL && is_pds_busy() > 0);

	/* do a 'ahb clock' setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_SAHB_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);

	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	system_clock_settle();

	/* clear PDS cache registry */
	for (uint32_t i = 0; i < EF_CTRL_EFUSE_R0_SIZE / 4; i++) {
		pefuse_start[i] = 0;
	}

	/* Load efuse region0 */
	/* not ahb clock setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_EF_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);
	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);

	/* trigger read */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_EF_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(1 << EF_CTRL_EF_IF_0_TRIG_POS);
	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	system_clock_delay_32M_ms(5);

	/* wait for read to complete */
	do
	{
		system_clock_delay_32M_ms(1);
		tmpVal = sys_read32(BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);
	} while ((tmpVal & EF_CTRL_EF_IF_0_BUSY_MSK) ||
		!(tmpVal && EF_CTRL_EF_IF_0_AUTOLOAD_DONE_MSK));

	/* do a 'ahb clock' setup */
	tmpVal =	EF_CTRL_EFUSE_CTRL_PROTECT |
			(EF_CTRL_OP_MODE_AUTO << EF_CTRL_EF_IF_0_MANUAL_EN_POS) |
			(EF_CTRL_PARA_DFT << EF_CTRL_EF_IF_0_CYC_MODIFY_POS) |
			(EF_CTRL_SAHB_CLK << EF_CTRL_EF_CLK_SAHB_DATA_SEL_POS) |
			(1 << EF_CTRL_EF_IF_AUTO_RD_EN_POS) |
			(0 << EF_CTRL_EF_IF_POR_DIG_POS) |
			(1 << EF_CTRL_EF_IF_0_INT_CLR_POS) |
			(0 << EF_CTRL_EF_IF_0_RW_POS) |
			(0 << EF_CTRL_EF_IF_0_TRIG_POS);

	sys_write32(tmpVal, BFLB_EF_CTRL_BASE + EF_CTRL_EF_IF_CTRL_0_OFFSET);

	/* get trim
	 * .name = "rc32m",
	 * .en_addr = 0x0C * 8 + 19,
	 * .parity_addr = 0x0C * 8 + 18,
	 * .value_addr = 0x0C * 8 + 10,
	 * .value_len = 8,
	 */
	tmpVal2 = 0x0c * 8 + 19;
	tmpVal = sys_read32(BFLB_EF_CTRL_BASE + (tmpVal2 / 32) * 4);
	if (!(tmpVal & (1 << (tmpVal2 % 32)))) {
		return 0;
	}
	tmpVal2 = 0x0C * 8 + 10;
	tmpVal = sys_read32(BFLB_EF_CTRL_BASE + (tmpVal2 / 32) * 4);
	tmpVal = tmpVal >> (tmpVal2 % 32);
	return tmpVal & ((1 << 8) - 1);
	/* TODO: check trim parity */
}

/* /!\ only use when running on 32Mhz Oscillator Clock
 */
static void system_clock_trim_32M(void)
{
	uint32_t tmpVal = 0;
	uint32_t trim = 0;

	trim = system_efuse_read_32M();
	tmpVal = sys_read32(PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	tmpVal = (tmpVal & PDS_RC32M_EXT_CODE_EN_UMSK) | 1 << PDS_RC32M_EXT_CODE_EN_POS;
	tmpVal = (tmpVal & PDS_RC32M_CODE_FR_EXT_UMSK) | trim << PDS_RC32M_CODE_FR_EXT_POS;
	sys_write32(tmpVal, PDS_BASE + PDS_RC32M_CTRL0_OFFSET);
	system_clock_settle();
}

/* 32 Mhz Oscillator: 0
 * crystal: 1
 * PLL and 32M: 2
 * PLL and crystal: 3
 */
static void system_set_root_clock(uint32_t clock)
{
	uint32_t tmpVal = 0;

	/* invalid value, fallback to internal 32M */
	if (clock < 0 || clock > 3)
	{
		clock = 0;
	}
	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_ROOT_CLK_SEL_UMSK) | (clock << HBN_ROOT_CLK_SEL_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	system_clock_settle();
}

static void system_set_root_clock_dividers(uint32_t hclk_div, uint32_t bclk_div)
{
	uint32_t tmpVal = 0;

	/* set dividers */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_HCLK_DIV_UMSK) | (hclk_div << GLB_REG_HCLK_DIV_POS);
	tmpVal = (tmpVal & GLB_REG_BCLK_DIV_UMSK) | (bclk_div << GLB_REG_BCLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* do something undocumented, probably acknowledging clock change by disabling then
	 * reenabling bclk
	 */
	sys_write32(0x00000001, 0x40000FFC);
	sys_write32(0x00000000, 0x40000FFC);

	/* set core clock ?*/
	sys_write32(sys_read32(CORECLOCKREGISTER) / (hclk_div + 1), CORECLOCKREGISTER);

	system_clock_settle();

	/* enable clocks */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	system_clock_settle();
}

/* HCLK: 0
 * PLL120M: 1
 */

static void system_set_PKA_clock(uint32_t pka_clock)
{
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_SWRST_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_PKA_CLK_SEL_UMSK) | (pka_clock << GLB_PKA_CLK_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SWRST_CFG2_OFFSET);
}


static void system_set_machine_timer_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1)
	{
		enable = 1;
	}
	tmpVal = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmpVal = (tmpVal & GLB_CPU_RTC_EN_UMSK) | (enable << GLB_CPU_RTC_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
}

/* clock:
 * 0: BCLK
 * 1: 32Khz Oscillator (RC32*K*)
 */

static void system_set_machine_timer_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (divider > 0x1FFFF)
	{
		divider = 0x1FFFF;
	}
	if (clock > 1)
	{
		clock = 1;
	}
	/* disable mtime clock */
	system_set_machine_timer_clock_enable(0);


	tmpVal = sys_read32(GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);
	tmpVal = (tmpVal & GLB_CPU_RTC_SEL_UMSK) | (clock << GLB_CPU_RTC_SEL_POS);
	tmpVal = (tmpVal & GLB_CPU_RTC_DIV_UMSK) | (divider << GLB_CPU_RTC_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CPU_CLK_CFG_OFFSET);

	system_set_machine_timer_clock_enable(enable);
}


/* No Crystal: 0
 * 32M: 1
 * 32MHz Oscillator : 32
 */

/* TODO: function is very long, consider splitting. pds power down and power up good candidate? */
/* TODO: Is PLL just called DLL on BL702? Or is it actually a different system? Why even?
 * at least it's simpler i guess
 */
static void system_setup_DLL(uint32_t crystal)
{
	uint32_t tmpVal = 0;

	/* DLL Off */

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PPU_DLL_UMSK) | ((uint32_t)(0) << GLB_PPU_DLL_POS);
	tmpVal = (tmpVal & GLB_PU_DLL_UMSK) | ((uint32_t)(0) << GLB_PU_DLL_POS);
	tmpVal = (tmpVal & GLB_DLL_RESET_UMSK) | ((uint32_t)(1) << GLB_DLL_RESET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	if (crystal == 32)
	{
		/* make sure we are on RC32M before trim */
		system_set_root_clock(0);
		system_set_root_clock_dividers(0, 0);
		sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);

		/* Trim RC32M */
		system_clock_trim_32M();

		tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
		tmpVal = (tmpVal & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(0) <<
GLB_DLL_REFCLK_SEL_POS);
		sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	}
	else
	{
		tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
		tmpVal = (tmpVal & GLB_DLL_REFCLK_SEL_UMSK) | ((uint32_t)(1) <<
GLB_DLL_REFCLK_SEL_POS);
		sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);
	}


	/* init sequence */

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_PRECHG_SEL_UMSK) | ((uint32_t)(1) << GLB_DLL_PRECHG_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PPU_DLL_UMSK) | ((uint32_t)(1) << GLB_PPU_DLL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_PU_DLL_UMSK) | ((uint32_t)(1) << GLB_PU_DLL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_RESET_UMSK) | ((uint32_t)(0) << GLB_DLL_RESET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);

	system_clock_settle();

	/* enable all DLL clocks*/
	tmpVal = sys_read32(GLB_BASE + GLB_DLL_OFFSET);
	tmpVal = (tmpVal & GLB_DLL_CLK_57P6M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_57P6M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_96M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_96M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_144M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_144M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_288M_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_288M_EN_POS);
	tmpVal = (tmpVal & GLB_DLL_CLK_MMDIV_EN_UMSK) | ((uint32_t)(1) << GLB_DLL_CLK_MMDIV_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_DLL_OFFSET);
	return;
}


static uint32_t system_uart_bflb_get_crystal_frequency(void)
{
	return (32 * 1000 * 1000);
}

/* Frequency Source:
 * No Crystal: 0
 * 32M: 1
 * 32MHz Oscillator: 32
 *
 * /!\ When Frequency Source is 32M, we do not power crystal
 *
 * Clock Frequency:
 * Crystal: 0
 * DLL 57.6MHz: 1
 * DLL 96Mhz: 2
 * DLL 144Mhz: 3
 * ? DLL 288 Mhz: 4 ?
 * 32MHz Oscillator : 32
 *
 *  /!\ When Clock Frequency is 32M, we do not power crystal or PLL
 */
static void system_init_root_clock(uint32_t crystal, uint32_t clock_frequency_source)
{
	uint32_t tmpVal = 0;
	uint32_t xtal_power_timeout = 0;


	/* make sure all clocks are enabled */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_BCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_BCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_HCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_HCLK_EN_POS);
	tmpVal = (tmpVal & GLB_REG_FCLK_EN_UMSK) | ((uint32_t)(1) << GLB_REG_FCLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set clock to internal 32MHz Oscillator as failsafe */
	system_set_root_clock(0);
	system_set_root_clock_dividers(0, 0);
	sys_write32(32 * 1000 * 1000, CORECLOCKREGISTER);

	if (clock_frequency_source == 32)
	{
		return;
	}

	if (crystal != 32)
	{
		/* power crystal */
		tmpVal = sys_read32(AON_BASE + AON_RF_TOP_AON_OFFSET);
		tmpVal = (tmpVal & AON_PU_XTAL_AON_UMSK) | ((uint32_t)(1) << AON_PU_XTAL_AON_POS);
		tmpVal = (tmpVal & AON_PU_XTAL_BUF_AON_UMSK) | ((uint32_t)(1) <<
AON_PU_XTAL_BUF_AON_POS);
		sys_write32(tmpVal, AON_BASE + AON_RF_TOP_AON_OFFSET);

		/* wait for crystal to be powered on */
		/* TODO: figure way to communicate this failed */
		do {
			system_clock_delay_32M_ms(1);
			tmpVal = sys_read32(AON_BASE + AON_TSEN_OFFSET);
			xtal_power_timeout++;
		} while (!(tmpVal & AON_XTAL_RDY_MSK) && xtal_power_timeout < 120);
	}

	/* power PLL
	 * This code path only when PLL!
	 */
	system_setup_DLL(crystal);
	/* Big settle, 55us in SDK */
	system_clock_delay_32M_ms(10);


	/* glb enable pll actual? */
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_EN_UMSK) | ((uint32_t)(1) << GLB_REG_PLL_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set PLL to use in GLB*/
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_UMSK) | ((uint32_t)(clock_frequency_source - 1) <<
GLB_REG_PLL_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG0_OFFSET);

	/* set root clock settings */
	switch (clock_frequency_source) {
	case 32:
		/* we are not supposed to be here */
	break;

	case 0:
		system_set_root_clock(1);
		sys_write32(system_uart_bflb_get_crystal_frequency(), CORECLOCKREGISTER);
	break;

	case 1:
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(57 * 1000 * 1000 + 6 * 100 * 1000, CORECLOCKREGISTER);
	break;

	case 2:
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(96 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	case 3:
		/* TODO: enable rom access 2T*/
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(144 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	case 4:
		/* TODO: enable rom access 2T*/
		system_set_root_clock_dividers(0, 1);
		system_set_root_clock(crystal == 32 ? 2 : 3);
		sys_write32(288 * 1000 * 1000, CORECLOCKREGISTER);
	break;

	default:
	break;
	}

	/* settle */
	system_clock_delay_32M_ms(10);

	return;
}

static void uart_set_clock_enable(uint32_t enable)
{
	uint32_t tmpVal = 0;

	if (enable > 1)
	{
		enable = 1;
	}
	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_EN_UMSK) | (enable << GLB_UART_CLK_EN_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG2_OFFSET);
}

/* Clock:
 * FCLK: 0
 * 160 Mhz PLL: 1
 * When using PLL root clock, we can use either setting, when using the 32Mhz Oscillator with a
 * uninitialized PLL, only FCLK will be available.
 */
static void uart_set_clock(uint32_t enable, uint32_t clock, uint32_t divider)
{
	uint32_t tmpVal = 0;

	if (divider > 0x7)
	{
		divider = 0x7;
	}
	if (clock > 1)
	{
		clock = 1;
	}
	/* disable uart clock */
	uart_set_clock_enable(0);


	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	tmpVal = (tmpVal & GLB_UART_CLK_DIV_UMSK) | (divider << GLB_UART_CLK_DIV_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_CLK_CFG2_OFFSET);

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_UART_CLK_SEL_UMSK) | (clock << HBN_UART_CLK_SEL_POS);
	sys_write32(tmpVal, HBN_BASE + HBN_GLB_OFFSET);

	uart_set_clock_enable(enable);
}


/* TODO: should take crystal type, defaults to 40Mhz crystal for BL602 as seems the most common */
static void system_clock_init(void)
{
#if 0
	system_init_root_clock(0, 32);
	system_set_root_clock_dividers(0, 0);
	system_clock_trim_32M();
#else
	system_init_root_clock(1, 3);
	system_set_root_clock_dividers(0, 1);
#endif
	system_set_machine_timer_clock(1, 0, mtimer_get_clk_src_div());
}

static void peripheral_clock_init(void)
{

	uint32_t regval = sys_read32(0x40000024);
	/* enable UART0 clock routing */
	regval |= (1 << 16);
	/* enable I2C0 clock routing */
	regval |= (1 << 19);
	sys_write32(regval, 0x40000024);
	uart_set_clock(1, 0, 0);
}


#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void)
{
	extern uint32_t __global_pointer$;
	return (ulong_t)&__global_pointer$;
}
#endif

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */

static int bl_riscv_init(void)
{
	uint32_t key;
	uint32_t *p;
	uint32_t i = 0;
	uint32_t tmpVal = 0;

	key = irq_lock();


	/* disable hardware_pullup_pull_down (reg_en_hw_pu_pd = 0) */
	tmpVal = sys_read32(HBN_BASE + HBN_IRQ_MODE_OFFSET);
	/* "BL_CLR_REG_BIT" */
	tmpVal = tmpVal & HBN_REG_EN_HW_PU_PD_UMSK;
	sys_write32(tmpVal, HBN_BASE + HBN_IRQ_MODE_OFFSET);

	/* 'seam' 0kb, undocumented */
	tmpVal = sys_read32(GLB_BASE + GLB_SEAM_MISC_OFFSET);
	tmpVal = (tmpVal & GLB_EM_SEL_UMSK) | ((uint32_t)(0) << GLB_EM_SEL_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_SEAM_MISC_OFFSET);


	/* GLB_UART_Sig_Swap_Set(UART_SIG_SWAP_NONE); */
	tmpVal = sys_read32(GLB_BASE + GLB_PARM_OFFSET);
	tmpVal = (tmpVal & GLB_UART_SWAP_SET_UMSK) | ((uint32_t)(0) <<
GLB_UART_SWAP_SET_POS);
	sys_write32(tmpVal, GLB_BASE + GLB_PARM_OFFSET);

	/* CLear all interrupt */
	p = (uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIE);

	for (i = 0; i < (IRQn_LAST + 3) / 4; i++) {
		p[i] = 0;
	}

	p = (uint32_t *)(CLIC_HART0_ADDR + CLIC_INTIP);

	for (i = 0; i < (IRQn_LAST + 3) / 4; i++) {
		p[i] = 0;
	}

	/* init bor for all platform */
	system_bor_init();

	system_clock_init();
	peripheral_clock_init();

	irq_unlock(key);

	/* wait 10 ms for peripherals to be ready */
	k_timepoint_t end_timeout = sys_timepoint_calc(K_MSEC(10));

	while (!sys_timepoint_expired(end_timeout))
	{
	}

	return 0;
}


SYS_INIT(bl_riscv_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
