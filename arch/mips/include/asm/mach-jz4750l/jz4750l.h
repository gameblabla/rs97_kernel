/*
 *  linux/include/asm-mips/mach-jz4750l/jz4750l.h
 *
 *  JZ4750 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_H__
#define __ASM_JZ4750_H__

#include <asm/mach-jz4750l/regs.h>
#include <asm/mach-jz4750l/ops.h>
#include <asm/mach-jz4750l/dma.h>
#include <asm/mach-jz4750l/misc.h>

/*------------------------------------------------------------------
 * Platform definitions
 */
#define JZ_SOC_NAME "JZ4750L"

#ifdef CONFIG_JZ4750L_F4750L
#include <asm/mach-jz4750l/board-f4750l.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4750l/clock.h>
#include <asm/mach-jz4750l/serial.h>

#endif /* __ASM_JZ4750_H__ */
