/*
 * Copyright (c) 2020 Linaro Ltd.
 * Copyright (c) 2020 ATL Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Cypress PSoC-6 MCU family devicetree helper macros
 */

#ifndef _CYPRESS_PSOC6_DT_H_
#define _CYPRESS_PSOC6_DT_H_

#include <devicetree.h>

/* Devicetree macros related to interrupt */
#ifdef CONFIG_CPU_CORTEX_M0PLUS
#define CY_PSOC6_DT_NVIC_MUX_INSTALL(n, isr, label)	    \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, interrupt_parent),\
		(CY_PSOC6_NVIC_MUX_INSTALL(n, isr, label)))
#define CY_PSOC6_DT_NVIC_MUX_IRQN(n) DT_IRQN(DT_INST_PHANDLE_BY_IDX(n,\
						interrupt_parent, 0))

/*
 * DT_INST_PROP_BY_IDX should be used instead DT_INST_IRQN.  DT_INST_IRQN
 * return IRQ with level translation since it uses interrupt-parent to update
 * Cortex-M0 NVIC multiplexers. See multi-level-interrupt-handling.
 */
#define CY_PSOC6_DT_NVIC_MUX_MAP(n) Cy_SysInt_SetInterruptSource( \
					DT_IRQN(DT_INST_PHANDLE_BY_IDX(n,\
						interrupt_parent, 0)), \
					DT_INST_PROP_BY_IDX(n, interrupts, 0))
#else
#define CY_PSOC6_DT_NVIC_MUX_INSTALL(n, isr, label) \
	CY_PSOC6_NVIC_MUX_INSTALL(n, isr, label)
#define CY_PSOC6_DT_NVIC_MUX_IRQN(n) DT_INST_IRQN(n)
#define CY_PSOC6_DT_NVIC_MUX_MAP(n)
#endif

#define CY_PSOC6_NVIC_MUX_INSTALL(n, isr, label)	  \
		do {					  \
		IRQ_CONNECT(CY_PSOC6_DT_NVIC_MUX_IRQN(n), \
			DT_INST_IRQ(n, priority),	  \
			isr, DEVICE_GET(label), 0);	  \
		CY_PSOC6_DT_NVIC_MUX_MAP(n);		  \
		irq_enable(CY_PSOC6_DT_NVIC_MUX_IRQN(n)); \
		} while (0)

/* Devicetree related macros to construct pin mux config data */

/* Get a node id from a pinctrl-0 prop at index 'i' */
#define NODE_ID_FROM_PINCTRL_0(inst, i) \
	DT_INST_PHANDLE_BY_IDX(inst, pinctrl_0, i)

/* Get GPIO register address associated with pinctrl-0 pin at index 'i' */
#define CY_PSOC6_PIN_TO_GPIO_REG_ADDR(inst, i) \
	DT_REG_ADDR(DT_PHANDLE(NODE_ID_FROM_PINCTRL_0(inst, i), cypress_pins))

/* Get PIN associated with pinctrl-0 pin at index 'i' */
#define CY_PSOC6_PIN(inst, i) \
	DT_PHA(NODE_ID_FROM_PINCTRL_0(inst, i), cypress_pins, pin)

/* Get HSIOM value associated with pinctrl-0 pin at index 'i' */
#define CY_PSOC6_PIN_HSIOM(inst, i) \
	DT_PHA(NODE_ID_FROM_PINCTRL_0(inst, i), cypress_pins, hsiom)

/* Helper function for CY_PSOC6_PIN_FLAGS */
#define CY_PSOC6_PIN_FLAG(inst, i, flag) \
	DT_PROP(NODE_ID_FROM_PINCTRL_0(inst, i), flag)

/* Convert DT flags to SoC flags */
#define CY_PSOC6_PIN_FLAGS(inst, i) \
	(CY_PSOC6_PIN_FLAG(inst, i, bias_pull_up) << \
		SOC_GPIO_PULLUP_POS | \
	 CY_PSOC6_PIN_FLAG(inst, i, bias_pull_down) << \
		SOC_GPIO_PULLUP_POS | \
	 CY_PSOC6_PIN_FLAG(inst, i, drive_open_drain) << \
		SOC_GPIO_OPENDRAIN_POS | \
	 CY_PSOC6_PIN_FLAG(inst, i, drive_open_source) << \
		SOC_GPIO_OPENSOURCE_POS | \
	 CY_PSOC6_PIN_FLAG(inst, i, drive_push_pull) << \
		SOC_GPIO_PUSHPULL_POS | \
	 CY_PSOC6_PIN_FLAG(inst, i, input_enable) << \
		SOC_GPIO_INPUTENABLE_POS)

/* Construct a soc_pio_pin element for pin cfg */
#define CY_PSOC6_DT_PIN(inst, idx)					\
	{								\
		(GPIO_PRT_Type *)CY_PSOC6_PIN_TO_GPIO_REG_ADDR(inst, idx), \
		CY_PSOC6_PIN(inst, idx),				\
		CY_PSOC6_PIN_HSIOM(inst, idx) << SOC_GPIO_FUNC_POS |	\
		CY_PSOC6_PIN_FLAGS(inst, idx)				\
	}

/* Get the number of pins for pinctrl-0 */
#define CY_PSOC6_DT_NUM_PINS(inst) DT_INST_PROP_LEN(inst, pinctrl_0)

/* internal macro to structure things for use with UTIL_LISTIFY */
#define CY_PSOC6_DT_PIN_ELEM(idx, inst) CY_PSOC6_DT_PIN(inst, idx),

/* Construct an array intializer for soc_gpio_pin for a device instance */
#define CY_PSOC6_DT_PINS(inst)				\
	{ UTIL_LISTIFY(CY_PSOC6_DT_NUM_PINS(inst),	\
		       CY_PSOC6_DT_PIN_ELEM, inst)	\
	}

#endif /* _CYPRESS_PSOC6_SOC_DT_H_ */
