/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Board configuration macros
 *
 * This header file is used to specify and describe board-level aspects
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <zephyr/sys/util.h>
#include <../common/soc_common.h>

#ifndef _ASMLANGUAGE

/* Add include for DTS generated information */
#include <zephyr/devicetree.h>

/* Addresses */

#include <bouffalolab/bl70x/bflb_soc.h>
#include <bouffalolab/bl70x/aon_reg.h>
#include <bouffalolab/bl70x/glb_reg.h>
#include <bouffalolab/bl70x/hbn_reg.h>
#include <bouffalolab/bl70x/l1c_reg.h>
#include <bouffalolab/bl70x/pds_reg.h>
#include <bouffalolab/bl70x/tzc_sec_reg.h>
#include <bouffalolab/bl70x/ef_ctrl_reg.h>
#include <bouffalolab/bl70x/extra_defines.h>

/* RISC-V Machine Timer configuration */
#define RISCV_MTIME_BASE             0x0200BFF8
#define RISCV_MTIMECMP_BASE          0x02004000

/* lib-c hooks required RAM defined variables */
#define RISCV_RAM_BASE               DT_SRAM_BASE_ADDRESS
#define RISCV_RAM_SIZE               KB(DT_SRAM_SIZE)

#define SOC_BOUFFALOLAB_BL_PLL160_FREQ_HZ (160000000)
#define SOC_BOUFFALOLAB_BL_PLL96_FREQ_HZ (96000000)
#define SOC_BOUFFALOLAB_BL_HCLK_FREQ_HZ	\
	DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

#ifdef CONFIG_RISCV_GP
ulong_t __soc_get_gp_initial_value(void);
#endif

#ifdef CONFIG_CODE_DATA_RELOCATION
#define ITCMF __attribute__((section(".itcm")))
#else
#define ITCMF
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
