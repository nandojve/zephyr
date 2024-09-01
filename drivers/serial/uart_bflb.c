/*
 * Copyright (c) 2021-2024 ATL Electronics
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bflb_bl_uart

/**
 * @brief UART driver for Bouffalo Lab MCU family
 */
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>

#include <glb_reg.h>
#include <hbn_reg.h>
#include <pds_reg.h>
#include <uart_reg.h>
#include <bflb_uart.h>

struct bflb_config {
	const struct pinctrl_dev_config *pincfg;
	uint32_t baudrate;
	uint8_t direction;
	uint8_t data_bits;
	uint8_t stop_bits;
	uint8_t parity;
	uint8_t bit_order;
	uint8_t flow_ctrl;
	uint8_t tx_fifo_threshold;
	uint8_t rx_fifo_threshold;
	uint32_t base_reg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct bflb_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static void uart_bflb_enabled(const struct device *dev, uint32_t enable)
{
	const struct bflb_config *cfg = dev->config;
	uint32_t rxt = 0;
	uint32_t txt = 0;

	if (enable > 1)
	{
		enable = 1;
	}

	txt = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	txt = (txt & ~UART_CR_UTX_EN) | enable;
	rxt = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
	rxt = (rxt & ~UART_CR_URX_EN) | enable;
	sys_write32(rxt, cfg->base_reg + UART_URX_CONFIG_OFFSET);
	sys_write32(txt, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
}

#ifdef CONFIG_SOC_SERIES_BL60X

static uint32_t uart_bflb_get_crystal_frequency(void)
{
	uint32_t tmpVal;

	/* get clkpll_sdmin */
	tmpVal = sys_read32(PDS_BASE + PDS_CLKPLL_SDM_OFFSET);
	tmpVal = (tmpVal & PDS_CLKPLL_SDMIN_MSK) >> PDS_CLKPLL_SDMIN_POS;

	switch (tmpVal) {
	case 0x500000:
	/* 24m */
	return (24 * 1000 * 1000);

	case 0x3C0000:
	/* 32m */
	return (32 * 1000 * 1000);

	case 0x320000:
	/* 38.4m */
	return (384 * 100 * 1000);

	case 0x300000:
	/* 40m */
	return (40 * 1000 * 1000);

	case 0x49D39D:
	/* 26m */
	return (26 * 1000 * 1000);

	default:
	/* 32m */
	return (32 * 1000 * 1000);
	}
}

static uint32_t uart_bflb_get_PLL_frequency(void)
{
	uint32_t tmpVal;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;

	if (tmpVal == 0) {
		/* pll 48m */
		return (48 * 1000 * 1000);
	} else if (tmpVal == 1) {
		/* pll 120m */
		return (120 * 1000 * 1000);
	} else if (tmpVal == 2) {
		/* pll 160m */
		return (160 * 1000 * 1000);
	} else if (tmpVal == 3) {
		/* pll 192m */
		return (192 * 1000 * 1000);
	} else {
		return 0;
	}
}


static uint32_t uart_bflb_get_clock(void)
{
	uint32_t tmpVal = 0;
	uint32_t uart_divider = 0;
	uint32_t hclk_divider = 0;


	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	uart_divider = (tmpVal & GLB_UART_CLK_DIV_MSK) >> GLB_UART_CLK_DIV_POS;


	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_UART_CLK_SEL_MSK) >> HBN_UART_CLK_SEL_POS;


	if (tmpVal == 0)
	{
		/* FCLK */
		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		hclk_divider = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		tmpVal = (tmpVal & GLB_HBN_ROOT_CLK_SEL_MSK) >> GLB_HBN_ROOT_CLK_SEL_POS;

		if (tmpVal == 0)
		{
			/* RC32M clock */
			tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		}
		else if (tmpVal == 1)
		{
			/* Crystal clock */
			tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		}
		else if (tmpVal > 1)
		{
			/* PLL Clock */
			tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));

		}
	}
	else
	{
		/* UART PLL 160 */
		return ((160 * 1000 * 1000) / (uart_divider + 1));
	}
	return 0;
}

#elif defined(CONFIG_SOC_SERIES_BL70X)

static uint32_t uart_bflb_get_crystal_frequency(void)
{
	return (32 * 1000 * 1000);
}

static uint32_t uart_bflb_get_PLL_frequency(void)
{
	uint32_t tmpVal;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
	tmpVal = (tmpVal & GLB_REG_PLL_SEL_MSK) >> GLB_REG_PLL_SEL_POS;

	if (tmpVal == 0) {
		return (57 * 1000 * 1000 + 6 * 100 * 1000);
	} else if (tmpVal == 1) {
		return (96 * 1000 * 1000);
	} else if (tmpVal == 2) {
		return (144 * 1000 * 1000);
	} else if (tmpVal == 3) {
		return (288 * 1000 * 1000);
	} else {
		return 0;
	}
}

static uint32_t uart_bflb_get_clock(void)
{
	uint32_t tmpVal = 0;
	uint32_t uart_divider = 0;
	uint32_t hclk_divider = 0;

	tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG2_OFFSET);
	uart_divider = (tmpVal & GLB_UART_CLK_DIV_MSK) >> GLB_UART_CLK_DIV_POS;

	tmpVal = sys_read32(HBN_BASE + HBN_GLB_OFFSET);
	tmpVal = (tmpVal & HBN_UART_CLK_SEL_MSK) >> HBN_UART_CLK_SEL_POS;

	if (tmpVal == 0) {
		/* FCLK */
		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		hclk_divider = (tmpVal & GLB_REG_HCLK_DIV_MSK) >> GLB_REG_HCLK_DIV_POS;

		tmpVal = sys_read32(GLB_BASE + GLB_CLK_CFG0_OFFSET);
		tmpVal = (tmpVal & GLB_HBN_ROOT_CLK_SEL_MSK) >> GLB_HBN_ROOT_CLK_SEL_POS;

		if (tmpVal == 0) {
			/* RC32M clock */
			tmpVal = (32 * 1000 * 1000) / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		} else if (tmpVal == 1) {
			/* Crystal clock */
			tmpVal = uart_bflb_get_crystal_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));
		} else if (tmpVal > 1) {
			/* PLL Clock */
			tmpVal = uart_bflb_get_PLL_frequency() / (hclk_divider + 1);
			return (tmpVal / (uart_divider + 1));

		}
	} else {
		/* UART DLL 96 */
		return ((96 * 1000 * 1000) / (uart_divider + 1));
	}
	return 0;
}

#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_bflb_err_check(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t status = 0;
	uint32_t tmpVal = 0;
	int errors = 0;

	status = sys_read32(cfg->base_reg + UART_INT_STS_OFFSET);
	tmpVal = sys_read32(cfg->base_reg + UART_INT_CLEAR_OFFSET);

	if (status & UART_URX_FER_INT) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (status & UART_UTX_FER_INT) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (status & UART_URX_PCE_INT) {
		tmpVal |= UART_CR_URX_PCE_CLR;

		errors |= UART_ERROR_PARITY;
	}

	sys_write32(tmpVal, cfg->base_reg + UART_INT_CLEAR_OFFSET);

	return errors;
}

int uart_bflb_irq_tx_ready(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);
	return (tmpVal & UART_TX_FIFO_CNT_MASK) > 0;
}

int uart_bflb_irq_rx_ready(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);
	return (tmpVal & UART_RX_FIFO_CNT_MASK) > 0;
}

int uart_bflb_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct bflb_config *const cfg = dev->config;
	uint8_t num_tx = 0U;

	while (num_tx < len && uart_bflb_irq_tx_ready(dev) > 0) {
		sys_write32(tx_data[num_tx++], cfg->base_reg + UART_FIFO_WDATA_OFFSET);
	}

	return num_tx;
}

int uart_bflb_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct bflb_config *const cfg = dev->config;
	uint8_t num_rx = 0U;

	while ((num_rx < size) && uart_bflb_irq_rx_ready(dev) > 0) {
		rx_data[num_rx++] = sys_read32(cfg->base_reg + UART_FIFO_RDATA_OFFSET);
	}

	return num_rx;
}

void uart_bflb_irq_tx_enable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal & ~UART_CR_UTX_FIFO_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}

void uart_bflb_irq_tx_disable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal | UART_CR_UTX_FIFO_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}


int uart_bflb_irq_tx_complete(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;
	int rc = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_STATUS_OFFSET);
	rc = tmpVal & UART_STS_UTX_BUS_BUSY;
	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);
	rc += tmpVal & UART_TX_FIFO_CNT_MASK;
	return (rc > 0 ? 1 : 0);
}

void uart_bflb_irq_rx_enable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal & ~UART_CR_URX_FIFO_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}

void uart_bflb_irq_rx_disable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal | UART_CR_URX_FIFO_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}

void uart_bflb_irq_err_enable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal & ~UART_CR_URX_PCE_MASK;
	tmpVal = tmpVal & ~UART_CR_UTX_FER_MASK;
	tmpVal = tmpVal & ~UART_CR_URX_FER_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}

void uart_bflb_irq_err_disable(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	tmpVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);
	tmpVal = tmpVal | UART_CR_URX_PCE_MASK;
	tmpVal = tmpVal | UART_CR_UTX_FER_MASK;
	tmpVal = tmpVal | UART_CR_URX_FER_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_MASK_OFFSET);
}

int uart_bflb_irq_is_pending(const struct device *dev)
{
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = sys_read32(cfg->base_reg + UART_INT_STS_OFFSET);
	uint32_t maskVal = sys_read32(cfg->base_reg + UART_INT_MASK_OFFSET);

	/* only first 8 bits are not reserved */
	return (((tmpVal & ~maskVal) & 0xFF) != 0 ? 1 : 0);
}

int uart_bflb_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

void uart_bflb_irq_callback_set(const struct device *dev,
			      uart_irq_callback_user_data_t cb,
			      void *user_data)
{
	struct bflb_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}

static void uart_bflb_isr(const struct device *dev)
{
	struct bflb_data *const data = dev->data;
	const struct bflb_config *const cfg = dev->config;
	uint32_t tmpVal = 0;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
	/* clear interrupts that require ack*/
	tmpVal = sys_read32(cfg->base_reg + UART_INT_CLEAR_OFFSET);
	tmpVal = tmpVal | UART_CR_URX_RTO_CLR;
	sys_write32(tmpVal, cfg->base_reg + UART_INT_CLEAR_OFFSET);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_bflb_configure(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;
	uint32_t tx_cfg = 0;
	uint32_t rx_cfg = 0;
	uint32_t divider = 0;
	uint32_t tmpVal = 0;

	divider = (uart_bflb_get_clock() * 10 / cfg->baudrate + 5) / 10;
	if (divider >= 0xFFFF)
	{
		divider = 0xFFFF - 1;
	}

	uart_bflb_enabled(dev, 0);

	sys_write32(((divider - 1) << 0x10) | ((divider - 1) & 0xFFFF), cfg->base_reg
+ UART_BIT_PRD_OFFSET);


	/* Configure Parity */
	tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);

	switch (cfg->parity) {
	case UART_PARITY_NONE:
		tx_cfg &= ~UART_CR_UTX_PRT_EN;
		rx_cfg &= ~UART_CR_URX_PRT_EN;
		break;
	case UART_PARITY_ODD:
		tx_cfg |= UART_CR_UTX_PRT_EN;
		tx_cfg |= UART_CR_UTX_PRT_SEL;
		rx_cfg |= UART_CR_URX_PRT_EN;
		rx_cfg |= UART_CR_URX_PRT_SEL;
		break;
	case UART_PARITY_EVEN:
		tx_cfg |= UART_CR_UTX_PRT_EN;
		tx_cfg &= ~UART_CR_UTX_PRT_SEL;
		rx_cfg |= UART_CR_URX_PRT_EN;
		rx_cfg &= ~UART_CR_URX_PRT_SEL;
		break;
	default:
		break;
	}

	/* Configure data bits */
	tx_cfg &= ~UART_CR_UTX_BIT_CNT_D_MASK;
	tx_cfg |= (cfg->data_bits + 4) << UART_CR_UTX_BIT_CNT_D_SHIFT;
	rx_cfg &= ~UART_CR_URX_BIT_CNT_D_MASK;
	rx_cfg |= (cfg->data_bits + 4) << UART_CR_URX_BIT_CNT_D_SHIFT;

	/* Configure tx stop bits */
	tx_cfg &= ~UART_CR_UTX_BIT_CNT_P_MASK;
	tx_cfg |= cfg->stop_bits << UART_CR_UTX_BIT_CNT_P_SHIFT;

	/* Configure tx cts flow control function */
	if (cfg->flow_ctrl & UART_FLOWCTRL_CTS) {
		tx_cfg |= UART_CR_UTX_CTS_EN;
	} else {
		tx_cfg &= ~UART_CR_UTX_CTS_EN;
	}

	/* disable de-glitch function */
	rx_cfg &= ~UART_CR_URX_DEG_EN;

	/* Write config */
	sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);

	/* enable hardware control RTS */
	#if defined(CONFIG_SOC_SERIES_BL60X)
	tmpVal = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
	tmpVal &= ~UART_CR_URX_RTS_SW_MODE;
	sys_write32(tmpVal, cfg->base_reg + UART_URX_CONFIG_OFFSET);
	#else
	tmpVal = sys_read32(cfg->base_reg + UART_SW_MODE_OFFSET);
	tmpVal &= ~UART_CR_URX_RTS_SW_MODE;
	sys_write32(tmpVal, cfg->base_reg + UART_SW_MODE_OFFSET);
	#endif

	/* disable inversion */
	tmpVal = sys_read32(cfg->base_reg + UART_DATA_CONFIG_OFFSET);
	tmpVal &= ~UART_CR_UART_BIT_INV;
	sys_write32(tmpVal, cfg->base_reg + UART_DATA_CONFIG_OFFSET);


	/* TX free run enable */
	tmpVal = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
	tmpVal |= UART_CR_UTX_FRM_EN;
	sys_write32(tmpVal, cfg->base_reg + UART_UTX_CONFIG_OFFSET);


	/* Configure FIFO thresholds */
	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);
	tmpVal &= ~UART_TX_FIFO_TH_MASK;
	tmpVal &= ~UART_RX_FIFO_TH_MASK;
	tmpVal |= (cfg->tx_fifo_threshold << UART_TX_FIFO_TH_SHIFT) & UART_TX_FIFO_TH_MASK;
	tmpVal |= (cfg->rx_fifo_threshold << UART_RX_FIFO_TH_SHIFT) & UART_RX_FIFO_TH_MASK;
	sys_write32(tmpVal, cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET);

	/* Clear FIFO */
	tmpVal = sys_read32(cfg->base_reg + UART_FIFO_CONFIG_0_OFFSET);
	tmpVal |= UART_TX_FIFO_CLR;
	tmpVal |= UART_RX_FIFO_CLR;
	tmpVal &= ~UART_DMA_TX_EN;
	tmpVal &= ~UART_DMA_RX_EN;
	sys_write32(tmpVal, cfg->base_reg + UART_FIFO_CONFIG_0_OFFSET);

	return 0;
}

static int uart_bflb_init(const struct device *dev)
{
	const struct bflb_config *cfg = dev->config;
	int rc = 0;

	pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	rc = uart_bflb_configure(dev);
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* disable all irqs */
	sys_write32(0x0, cfg->base_reg + UART_INT_EN_OFFSET);
	/* clear all IRQS */
	sys_write32(0xFF, cfg->base_reg + UART_INT_CLEAR_OFFSET);
	/* mask all IRQs */
	sys_write32(0xFFFFFFFF, cfg->base_reg + UART_INT_MASK_OFFSET);
	/* unmask necessary irqs */
	uart_bflb_irq_rx_enable(dev);
	uart_bflb_irq_err_enable(dev);
	/* enable all irqs */
	sys_write32(0xFF, cfg->base_reg + UART_INT_EN_OFFSET);
	cfg->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
	uart_bflb_enabled(dev, 1);
	return rc;
}

static int uart_bflb_poll_in(const struct device *dev, unsigned char *c)
{
	const struct bflb_config *cfg = dev->config;

	if ((sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_RX_FIFO_CNT_MASK) != 0) {
		*c = sys_read8(cfg->base_reg + UART_FIFO_RDATA_OFFSET);
		return 0;
	}

	return -1;
}

static void uart_bflb_poll_out(const struct device *dev, unsigned char c)
{
	const struct bflb_config *cfg = dev->config;

	while ((sys_read32(cfg->base_reg + UART_FIFO_CONFIG_1_OFFSET) & UART_TX_FIFO_CNT_MASK) == 0)
	{
	}
	sys_write8(c, cfg->base_reg + UART_FIFO_WDATA_OFFSET);
	return;
}



#ifdef CONFIG_PM_DEVICE
static int uart_bflb_pm_control(const struct device *dev,
			      enum pm_device_action action)
{
	const struct bl_config *cfg = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		(void)pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		const struct bflb_config *cfg = dev->config;
		uint32_t tx_cfg;
		uint32_t rx_cfg;

		tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg |= UART_CR_UTX_EN;
		rx_cfg |= UART_CR_URX_EN;
		sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		if (pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_SLEEP)) {
			return -134;
		}
		const struct bflb_config *cfg = dev->config;
		uint32_t tx_cfg;
		uint32_t rx_cfg;

		tx_cfg = sys_read32(cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		rx_cfg = sys_read32(cfg->base_reg + UART_URX_CONFIG_OFFSET);
		tx_cfg &= ~UART_CR_UTX_EN;
		rx_cfg &= ~UART_CR_URX_EN;
		sys_write32(tx_cfg, cfg->base_reg + UART_UTX_CONFIG_OFFSET);
		sys_write32(rx_cfg, cfg->base_reg + UART_URX_CONFIG_OFFSET);
		break;
	default:
		return -134;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct uart_driver_api uart_bflb_driver_api = {
	.poll_in = uart_bflb_poll_in,
	.poll_out = uart_bflb_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.err_check = uart_bflb_err_check,
	.fifo_fill = uart_bflb_fifo_fill,
	.fifo_read = uart_bflb_fifo_read,
	.irq_tx_enable = uart_bflb_irq_tx_enable,
	.irq_tx_disable = uart_bflb_irq_tx_disable,
	.irq_tx_ready = uart_bflb_irq_tx_ready,
	.irq_tx_complete = uart_bflb_irq_tx_complete,
	.irq_rx_enable = uart_bflb_irq_rx_enable,
	.irq_rx_disable = uart_bflb_irq_rx_disable,
	.irq_rx_ready = uart_bflb_irq_rx_ready,
	.irq_err_enable = uart_bflb_irq_err_enable,
	.irq_err_disable = uart_bflb_irq_err_disable,
	.irq_is_pending = uart_bflb_irq_is_pending,
	.irq_update = uart_bflb_irq_update,
	.irq_callback_set = uart_bflb_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define BFLB_UART_IRQ_HANDLER_DECL(instance)					\
	static void uart_bflb_config_func_##instance(const struct device *dev);
#define BFLB_UART_IRQ_HANDLER_FUNC(instance)					\
	.irq_config_func = uart_bflb_config_func_##instance
#define BFLB_UART_IRQ_HANDLER(instance)						\
	static void uart_bflb_config_func_##instance(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(instance),				\
			    DT_INST_IRQ(instance, priority),			\
			    uart_bflb_isr,					\
			    DEVICE_DT_INST_GET(instance),			\
			    0);							\
		irq_enable(DT_INST_IRQN(instance));				\
	}
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define BFLB_UART_IRQ_HANDLER_DECL(instance)
#define BFLB_UART_IRQ_HANDLER_FUNC(instance)
#define BFLB_UART_IRQ_HANDLER(instance)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


#define BL_UART_INIT(instance)							\
	PINCTRL_DT_INST_DEFINE(instance);					\
	BFLB_UART_IRQ_HANDLER_DECL(instance)					\
	static struct bflb_data uart##instance##_bflb_data;			\
	static const struct bflb_config uart##instance##_bflb_config = {	\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(instance),		\
		.base_reg = DT_INST_REG_ADDR(instance),				\
										\
		.baudrate = DT_INST_PROP(instance, current_speed),		\
		.data_bits = UART_DATA_BITS_8,					\
		.stop_bits = UART_STOP_BITS_1,					\
		.parity = UART_PARITY_NONE,					\
		.bit_order = UART_MSB_FIRST,					\
		.flow_ctrl = UART_FLOWCTRL_NONE,				\
		/* overflow interupt thresold, size is 32 bytes*/		\
		.tx_fifo_threshold = 8,						\
		.rx_fifo_threshold = 0,						\
		BFLB_UART_IRQ_HANDLER_FUNC(instance)				\
	};									\
	DEVICE_DT_INST_DEFINE(instance, &uart_bflb_init,			\
			      uart_bflb_pm_control,				\
			      &uart##instance##_bflb_data,			\
			      &uart##instance##_bflb_config, PRE_KERNEL_1,	\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &uart_bflb_driver_api);				\
										\
	BFLB_UART_IRQ_HANDLER(instance)

DT_INST_FOREACH_STATUS_OKAY(BL_UART_INIT)
