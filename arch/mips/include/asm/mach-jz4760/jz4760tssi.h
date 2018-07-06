/*
 * jz4760tssi.h
 * JZ4760 TSSI register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: whxu@ingenic.cn
 */

#ifndef __JZ4760TSSI_H__
#define __JZ4760TSSI_H__


/*
 * TS slave interface(TSSI) address definition
 */
#define	TSSI_BASE	0xb0073000


/*
 * TS registers offset address definition
 */
#define TSSI_TSENA_OFFSET	(0x00)	/* rw,  8, 0x08 */
#define TSSI_TSCFG_OFFSET	(0x04)	/* rw, 16, 0x04ff */
#define TSSI_TSCTRL_OFFSET	(0x08)	/* rw,  8, 0x03 */
#define TSSI_TSSTAT_OFFSET	(0x0c)	/* rw,  8, 0x00 */
#define TSSI_TSFIFO_OFFSET	(0x10)	/* rw, 32, 0x???????? */
#define TSSI_TSPEN_OFFSET	(0x14)	/* rw, 32, 0x00000000 */
#define TSSI_TSNUM_OFFSET	(0x18)	/* rw,  8, 0x00 */
#define TSSI_TSDTR_OFFSET	(0x1c)	/* rw,  8, 0x7f */
#define TSSI_TSPID_OFFSET	(0x20)	/* rw, 32, 0x00000000 */


/*
 * TS registers address definition
 */
#define TSSI_TSENA	(TSSI_BASE + TSSI_TSENA_OFFSET)
#define TSSI_TSCFG	(TSSI_BASE + TSSI_TSCFG_OFFSET)
#define TSSI_TSCTRL	(TSSI_BASE + TSSI_TSCTRL_OFFSET)
#define TSSI_TSSTAT	(TSSI_BASE + TSSI_TSSTAT_OFFSET)
#define TSSI_TSFIFO	(TSSI_BASE + TSSI_TSFIFO_OFFSET)
#define TSSI_TSPEN	(TSSI_BASE + TSSI_TSPEN_OFFSET)
#define TSSI_TSNUM	(TSSI_BASE + TSSI_TSNUM_OFFSET)
#define TSSI_TSDTR	(TSSI_BASE + TSSI_TSDTR_OFFSET)
#define TSSI_TSPID(n)	(TSSI_BASE + TSSI_TSPID_OFFSET + (n)*4)	/* max n is 15 */


/*
 * TS registers common define
 */

/* TSSI enable register(TSENA) */
#define TSENA_RESET		BIT7
#define TSENA_FAIL		BIT4
#define TSENA_PEN0		BIT3
#define TSENA_PIDEN		BIT2
#define TSENA_DMAEN		BIT1
#define TSENA_ENA		BIT0

/* TSSI configure register(TSCFG) */
#define TSCFG_EDNWD		BIT9
#define TSCFG_EDNBT		BIT8
#define TSCFG_TSDI_HIGH		BIT7
#define TSCFG_USE0		BIT6
#define TSCFG_TSCLK		BIT5
#define TSCFG_PARAL		BIT4
#define TSCFG_CLKP		BIT3
#define TSCFG_FRM_HIGH		BIT2
#define TSCFG_STR_HIGH		BIT1
#define TSCFG_FAIL_HIGH		BIT0

#define TSCFG_TRIGV_LSB		14
#define TSCFG_TRIGV_MASK	BIT_H2L(15, TSCFG_TRIGV_LSB)
#define TSCFG_TRIGV(n)		(((n)/8) << TSCFG_TRIGV_LSB)	/* n = 4, 8, 16 */

#define TSCFG_TRANSMD_LSB	10
#define TSCFG_TRANSMD_MASK	BIT_H2L(11, TSCFG_TRANSMD_LSB)

/* TSSI control register(TSCTRL) */
#define TSCTRL_FDTRM		BIT2
#define TSCTRL_FOVERUNM		BIT1
#define TSCTRL_FTRIGM		BIT0

/* TSSI state register(TSSTAT) */
#define TSSTAT_FDTR		BIT2
#define TSSTAT_FOVERUN		BIT1
#define TSSTAT_FTRIG		BIT0


#ifndef __MIPS_ASSEMBLER

#define REG_TSSI_TSENA		REG8(TSSI_TSENA)
#define REG_TSSI_TSCFG          REG16(TSSI_TSCFG)
#define REG_TSSI_TSCTRL         REG8(TSSI_TSCTRL)
#define REG_TSSI_TSSTAT         REG8(TSSI_TSSTAT)
#define REG_TSSI_TSFIFO         REG32(TSSI_TSFIFO)
#define REG_TSSI_TSPEN          REG32(TSSI_TSPEN)
#define REG_TSSI_TSNUM          REG8(TSSI_TSNUM)
#define REG_TSSI_TSDTR          REG8(TSSI_TSDTR)
#define REG_TSSI_TSPID(n)       REG32(TSSI_TSPID(n))


#endif /* __MIPS_ASSEMBLER */

#endif /* __JZ4760TSSI_H__ */

