/*
 *  Jz4760 On-Chip ethernet driver.
 *
 *  Copyright (C) 2005 - 2007  Ingenic Semiconductor Inc. Jason<xwang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __JZ4760_ETH_H__
#define __JZ4760_ETH_H__


// ----------------------------------------------------------------------------------
// 			PE-MACMII registers (16-bit)
// ----------------------------------------------------------------------------------

#define ETH_MAC_MCR1			(ETHC_BASE + 0x00 )// MAC configuration register #1
#define ETH_MAC_MCR2			(ETHC_BASE + 0x04 )// MAC configuration register #2
#define ETH_MAC_IPGR			(ETHC_BASE + 0x08 )// Back-to-Back Inter-Packet-Gap register
#define ETH_MAC_NIPGR			(ETHC_BASE + 0x0c )// Non-Back-to-Back Inter-Packet-Gap register
#define ETH_MAC_CWR			(ETHC_BASE + 0x10 )// Collision Window / Retry register
#define ETH_MAC_MFR			(ETHC_BASE + 0x14 )// Maximum Frame register
#define ETH_MAC_PSR			(ETHC_BASE + 0x18 )// PHY Support register (SMII / RMII / PMD / ENDEC)
#define ETH_MAC_TR			(ETHC_BASE + 0x1c )// Test register
#define ETH_MAC_MCFGR			(ETHC_BASE + 0x20 )// MII Mgmt Configuration
#define ETH_MAC_MCMDR			(ETHC_BASE + 0x24 )// MII Mgmt Command
#define ETH_MAC_MADRR			(ETHC_BASE + 0x28 )// MII Mgmt Address
#define ETH_MAC_MWTDR			(ETHC_BASE + 0x2c )// MII Mgmt Write Data
#define ETH_MAC_MRDDR			(ETHC_BASE + 0x30 )// MII Mgmt Read Data
#define ETH_MAC_MINDR			(ETHC_BASE + 0x34 )// MII Mgmt Indicators

#define ETH_MAC_SA0			(ETHC_BASE + 0x40 )// Station Address
#define ETH_MAC_SA1			(ETHC_BASE + 0x44 )// Station Address
#define ETH_MAC_SA2			(ETHC_BASE + 0x48 )// Station Address

// Constants for ETH_MAC_MCR1 register
#define MCR1_SOFTRST			(1 << 15)	// Soft reset
#define MCR1_SMLTRST			(1 << 14)	// Simulation reset
#define MCR1_MCSRRST			(1 << 11)	// MAC Control Sublayer / Rx domain logic reset
#define MCR1_RFUNRST			(1 << 10)	// Rx Function logic reset
#define MCR1_MCSTRST			(1 << 9)	// MAC Control Sublayer / Tx domain logic reset
#define MCR1_TFUNRST			(1 << 8)	// Tx Function logic reset
#define MCR1_LB				(1 << 4)	// Tx interface loop back to Rx interface
#define MCR1_TFC			(1 << 3)	// Enable Tx flow control
#define MCR1_RFC			(1 << 2)	// Enable Rx flow control
#define MCR1_PARF			(1 << 1)	// Pass all receive frames
#define MCR1_RE				(1 << 0)	// Enable receive

// Constants for ETH_MAC_MCR2 register
#define MCR2_ED				(1 << 14)	// Defer to carrier indefinitely as per the Standard
#define MCR2_BPNB			(1 << 13)	// Back Pressure / No Backoff
#define MCR2_NB				(1 << 12)	// No Backoff
#define MCR2_LPE			(1 << 9)	// Long Preamble Enforcement
#define MCR2_PPE			(1 << 8)	// Pure Preamble Enforcement
#define MCR2_ADPE			(1 << 7)	// Auto-Detect Pad Enable
#define MCR2_VPE			(1 << 6)	// VLAN Pad Enable
#define MCR2_PCE			(1 << 5)	// Pad / CRC Enable
#define MCR2_CE				(1 << 4)	// CRC Enable
#define MCR2_DC				(1 << 3)	// Delayed CRC
#define MCR2_HFE			(1 << 2)	// Huge Frame Enable
#define MCR2_FLC			(1 << 1)	// Frame Length Checking
#define MCR2_FD				(1 << 0)	// Full-Duplex

// Contants for ETH_MAC_IPGR register
#define IPGR_MASK			0x007f		// In Full-Duplex the recommended value is 0x15(21d)

// Contants for ETH_MAC_NIPGR register
#define NIPGR_P1_MASK			0x7f00		// The recommended value is 0xC(12d)
#define NIPGR_P2_MASK			0x007f		// The recommended value is 0x12(18d)

// Contants for ETH_MAC_CWR register
#define CWR_CW_MASK			0x3f00		// Collision window, Default value is 0x37(55d)
#define CWR_RM_MASK			0x000f		// Retry time, default & standard is 0xF(15d)

// Contants for ETH_MAC_PSR register
#define PSR_RIM				(1 << 15)	// Reset Interface Module
#define PSR_PM				(1 << 12)	// PHY Module
#define PSR_RPERMII			(1 << 11)	// Reset PERMII
#define PSR_OS				(1 << 8)	// Operating Speed
#define PSR_RPE100M			(1 << 7)	// Reset PE100M module
#define PSR_FQ				(1 << 6)	// Force Quiet
#define PSR_NC				(1 << 5)	// No Cipher
#define PSR_DLF				(1 << 4)	// Disable Link Fail
#define PSR_RPE10T			(1 << 3)	// Reset PE10T module
#define PSR_EJP				(1 << 1)	// Enable Jabber Protection
#define PSR_BM				(1 << 0)	// Bit Mode

// Contants for ETH_MAC_TR register
#define TR_TB				(1 << 2)	// Test Backpressure
#define TR_TP				(1 << 1)	// Test Pause
#define TR_SPQ				(1 << 0)	// Shortcut Pause Quanta

// Contants for ETH_MAC_MCFGR register
#define MCFGR_RST			(1 << 15)	// Reset MII Mgmt
#define MCFGR_CS_MASK			0x001c		// Clock Reset
#define MCFGR_SP			(1 << 1)	// Suppress Preamble
#define MCFGR_SI			(1 << 0)	// Scan Increment

// Contants for ETH_MAC_MCMDR register
#define MCMDR_SCAN			(1 << 1)	// Cause MII Mgmt module to perform Read cycles continuously
#define MCMDR_READ			(1 << 0)	// Cause MII Mgmt module to perform single Read cycles
#define MII_SCAN			(1 << 1)
#define MII_NO_SCAN			0

// Contants for ETH_MAC_MADRR register
#define MADRR_PA_MASK			0x1f00		// PHY Address
#define MADRR_RA_MASK			0x001f		// Register Address

// Contants for ETH_MAC_MINDR register
#define MINDR_LF			(1 << 3)	// Link Fail
#define MINDR_NV			(1 << 2)	// Not Valid
#define MINDR_S				(1 << 1)	// Scanning
#define MINDR_BUSY			(1 << 0)	// Busy

// ----------------------------------------------------------------------------------
// 			M-AHBE DMA control and status registers (32-bit)
// ----------------------------------------------------------------------------------

#define ETH_DMA_TCR			(ETHC_BASE + 0x180)// Transmit control
#define ETH_DMA_TDR			(ETHC_BASE + 0x184)// Pointer to Transmit Descriptor
#define ETH_DMA_TSR			(ETHC_BASE + 0x188)// Transmit Status
#define ETH_DMA_RCR			(ETHC_BASE + 0x18c)// Receive Control
#define ETH_DMA_RDR			(ETHC_BASE + 0x190)// Pointer to Receive Descriptor
#define ETH_DMA_RSR			(ETHC_BASE + 0x194)// Receive Status
#define ETH_DMA_IMR			(ETHC_BASE + 0x198)// Interrupt Mask
#define ETH_DMA_IR			(ETHC_BASE + 0x19c)// Interrupts
/*  ETH_DMA_IR = ETH_DMA_IMR _AND_ corresponding status flags which in ETH_DMA_TSR and ETH_DMA_RSR
 *
 *  ETH_DMA_IR is read only, so the way to disable all interrupt is clear ETH_DMA_IMR or ETH_DMA_T(R)SR.
 *  Clear ETH_DMA_IMR is better. Then read status and call handle routine.
 *  Reset ETH_DMA_IMR after all interrupt-enabled status flags be cleared.
 */

// Constants for ETH_DMA_TCR register
#define TCR_ENABLE			(1 << 0)	// Enable DMA tx packet transfers

// Constants for ETH_DMA_TSR register
#define TSR_PKTSENT			(1 << 0)	// Indicates packet(s) have(has) been successfully txed
#define TSR_UNDERRUN			(1 << 1)	// Set whenever DMA controller reads a Empty Flag
#define TSR_BUSERR			(1 << 3)	// Indicates that DMA controller found tx bus error
#define TSR_PKTCNT_MASK			(0xFF << 16)	// Bit mask of tx packet counter
#define TSR_FLAGS			(TSR_PKTSENT | TSR_UNDERRUN | TSR_BUSERR)

// Constants for ETH_DMA_RCR register
#define RCR_ENABLE			(1 << 0)	// Enable DMA rx packet transfers

// Constants for ETH_DMA_RSR register
#define RSR_PKTRECV			(1 << 0)	// Indicates packet(s) have(has) been successfully rxed
#define RSR_OVERFLOW			(1 << 2)	// Set whenever DMA controller reads a Non-Empty Flag
#define RSR_BUSERR			(1 << 3)	// Indicates that DMA controller found rx bus error
#define RSR_PKTCNT_MASK			(0xFF << 16)	// Bit mask of rx packet counter
#define RSR_FLAGS			(RSR_PKTRECV | RSR_OVERFLOW | RSR_BUSERR)

// Constants for ETH_DMA_IMR register
#define IMR_PKTSENT			(1 << 0)	// Packet sent interrupt mask
#define IMR_UNDERRUN			(1 << 1)	// Underrun interrupt mask
#define IMR_TBUSERR			(1 << 3)	// Tx bus error interrupt mask
#define IMR_PKTRECV			(1 << 4)	// Packet received interrupt mask
#define IMR_OVERFLOW			(1 << 6)	// Overflow interrupt mask
#define IMR_RBUSERR			(1 << 7)	// Rx bus error interrupt mask
#define IMR_ALL_IRQ			(IMR_UNDERRUN | IMR_TBUSERR | IMR_OVERFLOW | IMR_RBUSERR | IMR_PKTRECV)
//#define IMR_ALL_IRQ			(IMR_UNDERRUN | IMR_TBUSERR | IMR_OVERFLOW | IMR_RBUSERR | IMR_PKTSENT | IMR_PKTRECV)

#define BURST_LEN_LO			(1 << 27)
#define BURST_LEN_HI			(1 << 28)
// Burst length configuration.
// 	HI	LO	Burst length
// 	0	0	4
//	0	1	8
//	1	0	16
//	1	1	4
#define BURST_LEN_4			(0 << 27)
#define BURST_LEN_8			(1 << 27)
#define BURST_LEN_16			(2 << 27)

// ----------------------------------------------------------------------------------
// 			M-MIIFIF (32-bits) registers (32-bit)
// ----------------------------------------------------------------------------------

#define ETH_FIFO_CR0			(ETHC_BASE + 0x3c )// Configuration Register 0
#define ETH_FIFO_CR1			(ETHC_BASE + 0x4c )// Configuration Register 1
#define ETH_FIFO_CR2			(ETHC_BASE + 0x50 )// Configuration Register 2
#define ETH_FIFO_CR3			(ETHC_BASE + 0x54 )// Configuration Register 3
#define ETH_FIFO_CR4			(ETHC_BASE + 0x58 )// Configuration Register 4
#define ETH_FIFO_CR5			(ETHC_BASE + 0x5c )// Configuration Register 5

#define ETH_FIFO_RAR0			(ETHC_BASE + 0x60 )// RAM Access Register 0
#define ETH_FIFO_RAR1			(ETHC_BASE + 0x64 )// RAM Access Register 1
#define ETH_FIFO_RAR2			(ETHC_BASE + 0x68 )// RAM Access Register 2
#define ETH_FIFO_RAR3			(ETHC_BASE + 0x6c )// RAM Access Register 3
#define ETH_FIFO_RAR4			(ETHC_BASE + 0x70 )// RAM Access Register 4
#define ETH_FIFO_RAR5			(ETHC_BASE + 0x74 )// RAM Access Register 5
#define ETH_FIFO_RAR6			(ETHC_BASE + 0x78 )// RAM Access Register 6
#define ETH_FIFO_RAR7			(ETHC_BASE + 0x7c )// RAM Access Register 7

// Constants for ETH_FIFO_CR0 register
#define FTF_EN_RPLY			(1 << 20)	// Indicate whether mmiitfif_fab module is enabled
#define STF_EN_RPLY			(1 << 19)	// Indicate whether mmiitfif_sys module is enabled
#define FRF_EN_RPLY			(1 << 18)	// Indicate whether mmiirfif_fab module is enabled
#define SRF_EN_RPLY			(1 << 17)	// Indicate whether mmiirfif_sys module is enabled
#define WTM_EN_RPLY			(1 << 16)	// Indicate whether mmiitfif_wtm module is enabled
#define ALL_EN_RPLY			(FTF_EN_RPLY | STF_EN_RPLY | FRF_EN_RPLY | SRF_EN_RPLY | WTM_EN_RPLY)

#define FTF_EN_REQ			(1 << 12)	// Request enable/disable -ing the mmiitfif_fab module
#define STF_EN_REQ			(1 << 11)	// Request enable/disable -ing the mmiitfif_sys module
#define FRF_EN_REQ			(1 << 10)	// Request enable/disable -ing the mmiirfif_fab module
#define SRF_EN_REQ			(1 << 9)	// Request enable/disable -ing the mmiirfif_sys module
#define WTM_EN_REQ			(1 << 8)	// Request enable/disable -ing the mmiitfif_wtm module
#define ALL_EN_REQ			(FTF_EN_REQ | STF_EN_REQ | FRF_EN_REQ | SRF_EN_REQ | WTM_EN_REQ)

#define FTF_RST				(1 << 4)	// Reset mmiitfif_fab module
#define STF_RST				(1 << 3)	// Reset mmiitfif_sys module
#define FRT_RST				(1 << 2)	// Reset mmiirfif_fab module
#define SRF_RST				(1 << 1)	// Reset mmiirfif_sys module
#define WTM_RST				(1 << 0)	// Reset mmiitfif_wtm module
#define ALL_RST				(FTF_RST | STF_RST | FRT_RST | SRF_RST | WTM_RST)

// Constants for ETH_FIFO_CR1 register
#define CFG_FR_TH_MASK			(0x0fff << 16)	// Fabric Receive Threshold mask
#define CFG_XOFF_RTX_MASK		0xffff		// ??????????????

// Constants for ETH_FIFO_CR2 register
#define CFG_H_WM_MASK			(0x1fff << 16)	// Receive RAM high watermark mask
#define CFG_L_WM_MASK			0x1fff		// Receive RAM low watermark mask

// Constants for ETH_FIFO_CR3 register
#define CFG_H_WM_FT_MASK		(0x0fff << 16)	// ??????????????
#define CFG_FT_TH_MASK			0xffff		// Fabric Transmit Threshold mask

// Constants for ETH_FIFO_CR4 register
#define HST_FLT_RFRM_MASK		0xffff		// Indicate drop condition

// Constants for ETH_FIFO_CR5 register
#define CFG_H_DPLX			(1 << 22)	// Enable Half-duplex backpressure as flow control mchm
#define CFG_SR_FULL			(1 << 21)	// Indicate whether FIFO storage has been met or exceeded
#define CFG_SR_FULL_CLR			(1 << 20)	// Clear CFG_SR_FULL bit
#define CLK_EN_MODE			(1 << 19)	// 
#define HST_DR_LT_64			(1 << 18)	// Drop the frame which less than 16 bit length
#define HST_FLT_RFRMDC_MASK		(0xffff)	// Indicate drop condition ... which don't care

// Constants for ETH_FIFO_RAR0 register
#define RAR0_HT_W_REQ			(1 << 31)	// Host Tx RAM write request
#define RAR0_HT_W_ACK			(1 << 30)	// Host Tx RAM write acknowledge
#define RAR0_HT_W_CD_MASK		(0xff << 16)	// Host Tx RAM write control data
#define RAR0_HT_W_ADDR_MASK		(0x3ff << 0)	// Host Tx RAM write address, [9:0] 10 bit, (4k RAM / 4 = 1024)

// Constants for ETH_FIFO_RAR2 register
#define RAR2_HT_R_REQ			(1 << 31)	// Host Tx RAM read request
#define RAR2_HT_R_ACK			(1 << 30)	// Host Tx RAM read acknowledge
#define RAR2_HT_R_CD_MASK		(0xff << 16)	// Host Tx RAM read control data
#define RAR2_HT_R_ADDR_MASK		(0x3ff << 0)	// Host Tx RAM read address, [9:0] 10 bit

// Constants for ETH_FIFO_RAR4 register
#define RAR4_HR_W_REQ			(1 << 31)	// Host Rx RAM write request
#define RAR4_HR_W_ACK			(1 << 30)	// Host Rx RAM write acknowledge
#define RAR4_HR_W_CD_MASK		(0xf << 16)	// Host Rx RAM write control data
#define RAR4_HR_W_ADDR_MASK		(0x3ff << 0)	// Host Rx RAM write address

// Constants for ETH_FIFO_RAR6 register
#define RAR4_HR_R_REQ			(1 << 31)	// Host Rx RAM read request
#define RAR4_HR_R_ACK			(1 << 30)	// Host Rx RAM read acknowledge
#define RAR4_HR_R_CD_MASK		(0xf << 16)	// Host Rx RAM read control data
#define RAR4_HR_R_ADDR_MASK		(0x3ff << 0)	// Host Rx RAM read address

// ----------------------------------------------------------------------------------
// 			PE-MSTAT registers (32-bit)
// ----------------------------------------------------------------------------------

// Several control signal
#define ETH_STAT_CR			ETH_DMA_IMR	// Use high 3 bits of ETH_DMA_IMR
#define STAT_EN				(1 << 31)	// Enable PE-MSTAT module
#define STAT_CL				(1 << 30)	// Clear counters, need voltage jump
#define STAT_AZ				(1 << 29)	// Enable Auto-Zero after read

// Combined transmit and receive counters. [17 : 0] available
#define ETH_STAT_TR64			(ETHC_BASE + 0x80 )// Tx & Rx 64          bytes frame counter
#define ETH_STAT_TR127			(ETHC_BASE + 0x84 )// Tx & Rx 65 ~ 127    bytes frame counter
#define ETH_STAT_TR255			(ETHC_BASE + 0x88 )// Tx & Rx 128 ~ 255   bytes frame counter
#define ETH_STAT_TR511			(ETHC_BASE + 0x8c )// Tx & Rx 256 ~ 511   bytes frame counter
#define ETH_STAT_TR1K			(ETHC_BASE + 0x90 )// Tx & Rx 512 ~ 1023  bytes frame counter
#define ETH_STAT_TRMAX			(ETHC_BASE + 0x94 )// Tx & Rx 1024 ~ 1518 bytes frame counter
#define ETH_STAT_TRMGV			(ETHC_BASE + 0x98 )// Tx & Rx 1519 ~ 1522 bytes good VLAN frame counter

// Receive counters
#define ETH_STAT_RBYT			(ETHC_BASE + 0x9c )// Rx Byte counter				[23 : 0]
#define ETH_STAT_RPKT			(ETHC_BASE + 0xa0 )// Rx Packet counter				[17 : 0]
#define ETH_STAT_RFCS			(ETHC_BASE + 0xa4 )// Rx FCS Error counter			[11 : 0]
#define ETH_STAT_RMCA			(ETHC_BASE + 0xa8 )// Rx Multicast packet counter		[17 : 0]
#define ETH_STAT_RBCA			(ETHC_BASE + 0xac )// Rx Broadcast packet counter		[21 : 0]
#define ETH_STAT_RXCF			(ETHC_BASE + 0xb0 )// Rx Control frame packet counter		[17 : 0]
#define ETH_STAT_RXPF			(ETHC_BASE + 0xb4 )// Rx PAUSE frame packet counter		[11 : 0]
#define ETH_STAT_RXUO			(ETHC_BASE + 0xb8 )// Rx Unkown OP code counter			[11 : 0]
#define ETH_STAT_RALN			(ETHC_BASE + 0xbc )// Rx Alignment Error counter		[11 : 0]
#define ETH_STAT_RFLR			(ETHC_BASE + 0xc0 )// Rx Frame Length Error counter		[15 : 0]
#define ETH_STAT_RCDE			(ETHC_BASE + 0xc4 )// Rx Code Error counter			[11 : 0]
#define ETH_STAT_RCSE			(ETHC_BASE + 0xc8 )// Rx Carrier Sense Error counter		[11 : 0]
#define ETH_STAT_RUND			(ETHC_BASE + 0xcc )// Rx Undersize packet counter		[11 : 0]
#define ETH_STAT_ROVR			(ETHC_BASE + 0xd0 )// Rx Oversize packet counter		[11 : 0]
#define ETH_STAT_RFRG			(ETHC_BASE + 0xd4 )// Rx Fragments counter			[11 : 0]
#define ETH_STAT_RJBR			(ETHC_BASE + 0xd8 )// Rx Jabber counter				[11 : 0]
#define ETH_STAT_RDRP			(ETHC_BASE + 0xdc )// Rx Drop					[11 : 0]

// Transmit counters
#define ETH_STAT_TBYT			(ETHC_BASE + 0xe0 )// Tx Byte counter				[23 : 0]
#define ETH_STAT_TPKT			(ETHC_BASE + 0xe4 )// Tx Packet counter				[17 : 0]
#define ETH_STAT_TMCA			(ETHC_BASE + 0xe8 )// Tx Multicast packet counter		[17 : 0]
#define ETH_STAT_TBCA			(ETHC_BASE + 0xec )// Tx Broadcast packet counter		[17 : 0]
#define ETH_STAT_TXPF			(ETHC_BASE + 0xf0 )// Tx PAUSE frame packet counter		[11 : 0]
#define ETH_STAT_TDFR			(ETHC_BASE + 0xf4 )// Tx Deferral packet counter		[11 : 0]
#define ETH_STAT_TEDF			(ETHC_BASE + 0xf8 )// Tx Excessive Deferral packet counter	[11 : 0]
#define ETH_STAT_TSCL			(ETHC_BASE + 0xfc )// Tx Single Collision packet counter	[11 : 0]
#define ETH_STAT_TMCL			(ETHC_BASE + 0x100)// Tx Multiple Collision packet counte	[11 : 0]
#define ETH_STAT_TLCL			(ETHC_BASE + 0x104)// Tx Late Collision packet counter		[11 : 0]
#define ETH_STAT_TXCL			(ETHC_BASE + 0x108)// Tx Excessive Collision packet counter	[11 : 0]
#define ETH_STAT_TNCL			(ETHC_BASE + 0x10c)// Tx Total Collision packet counter		[12 : 0]
#define ETH_STAT_TPFH			(ETHC_BASE + 0x110)// Tx PAUSE frames Honored counter		[11 : 0]
#define ETH_STAT_TDRP			(ETHC_BASE + 0x114)// Tx Drop frame counter			[11 : 0]
#define ETH_STAT_TJBR			(ETHC_BASE + 0x118)// Tx Jabber frame counter			[11 : 0]
#define ETH_STAT_TFCS			(ETHC_BASE + 0x11c)// Tx FCS Error counter			[11 : 0]
#define ETH_STAT_TXCF			(ETHC_BASE + 0x120)// Tx Control frame counter			[11 : 0]
#define ETH_STAT_TOVR			(ETHC_BASE + 0x124)// Tx Oversize frame counter			[11 : 0]
#define ETH_STAT_TUND			(ETHC_BASE + 0x128)// Tx Undersize frame counter		[11 : 0]
#define ETH_STAT_TFRG			(ETHC_BASE + 0x12c)// Tx Fragments frame counter		[11 : 0]

// Carry registers
#define ETH_STAT_CAR1			(ETHC_BASE + 0x130)// Carry Register 1
#define ETH_STAT_CAR2			(ETHC_BASE + 0x134)// Carry Register 2
#define ETH_STAT_CARM1			(ETHC_BASE + 0x138)// Carry Mask Register 1
#define ETH_STAT_CARM2			(ETHC_BASE + 0x13c)// Carry Mask Register 2

// Constants for ETH_STAT_CAR*
#define CAR1_TR64			(1 << 31)
#define CAR1_TR127			(1 << 30)
#define CAR1_TR255			(1 << 29)
#define CAR1_TR511			(1 << 28)
#define CAR1_TR1K			(1 << 27)
#define CAR1_TRMAX			(1 << 26)
#define CAR1_TRMGV			(1 << 25)
#define CAR1_RBY			(1 << 16)
#define CAR1_RPK			(1 << 15)
#define CAR1_RFC			(1 << 14)
#define CAR1_RMC			(1 << 13)
#define CAR1_RBC			(1 << 12)
#define CAR1_RXC			(1 << 11)
#define CAR1_RXP			(1 << 10)
#define CAR1_RXU			(1 << 9)
#define CAR1_RAL			(1 << 8)
#define CAR1_RFL			(1 << 7)
#define CAR1_RCD			(1 << 6)
#define CAR1_RCS			(1 << 5)
#define CAR1_RUN			(1 << 4)
#define CAR1_ROV			(1 << 3)
#define CAR1_RFR			(1 << 2)
#define CAR1_RJB			(1 << 1)
#define CAR1_RDR			(1 << 0)

#define CAR2_TJB			(1 << 19)
#define CAR2_TFC			(1 << 18)
#define CAR2_TCF			(1 << 17)
#define CAR2_TOV			(1 << 16)
#define CAR2_TUN			(1 << 15)
#define CAR2_TFG			(1 << 14)
#define CAR2_TBY			(1 << 13)
#define CAR2_TPK			(1 << 12)
#define CAR2_TMC			(1 << 11)
#define CAR2_TBC			(1 << 10)
#define CAR2_TPF			(1 << 9)
#define CAR2_TDF			(1 << 8)
#define CAR2_TED			(1 << 7)
#define CAR2_TSC			(1 << 6)
#define CAR2_TMA			(1 << 5)
#define CAR2_TLC			(1 << 4)
#define CAR2_TXC			(1 << 3)
#define CAR2_TNC			(1 << 2)
#define CAR2_TPH			(1 << 1)
#define CAR2_TDP			(1 << 0)

#define CARM1_TR64			(1 << 31)
#define CARM1_TR127			(1 << 30)
#define CARM1_TR255			(1 << 29)
#define CARM1_TR511			(1 << 28)
#define CARM1_TR1K			(1 << 27)
#define CARM1_TRMAX			(1 << 26)
#define CARM1_TRMGV			(1 << 25)
#define CARM1_RBY			(1 << 16)
#define CARM1_RPK			(1 << 15)
#define CARM1_RFC			(1 << 14)
#define CARM1_RMC			(1 << 13)
#define CARM1_RBC			(1 << 12)
#define CARM1_RXC			(1 << 11)
#define CARM1_RXP			(1 << 10)
#define CARM1_RXU			(1 << 9)
#define CARM1_RAL			(1 << 8)
#define CARM1_RFL			(1 << 7)
#define CARM1_RCD			(1 << 6)
#define CARM1_RCS			(1 << 5)
#define CARM1_RUN			(1 << 4)
#define CARM1_ROV			(1 << 3)
#define CARM1_RFR			(1 << 2)
#define CARM1_RJB			(1 << 1)
#define CARM1_RDR			(1 << 0)

#define CARM2_TJB			(1 << 19)
#define CARM2_TFC			(1 << 18)
#define CARM2_TCF			(1 << 17)
#define CARM2_TOV			(1 << 16)
#define CARM2_TUN			(1 << 15)
#define CARM2_TFG			(1 << 14)
#define CARM2_TBY			(1 << 13)
#define CARM2_TPK			(1 << 12)
#define CARM2_TMC			(1 << 11)
#define CARM2_TBC			(1 << 10)
#define CARM2_TPF			(1 << 9)
#define CARM2_TDF			(1 << 8)
#define CARM2_TED			(1 << 7)
#define CARM2_TSC			(1 << 6)
#define CARM2_TMA			(1 << 5)
#define CARM2_TLC			(1 << 4)
#define CARM2_TXC			(1 << 3)
#define CARM2_TNC			(1 << 2)
#define CARM2_TPH			(1 << 1)
#define CARM2_TDP			(1 << 0)

// ----------------------------------------------------------------------------------
// 			PE-SAL registers (32-bit)
// ----------------------------------------------------------------------------------

#define ETH_SAL_AFR			(ETHC_BASE + 0x1a0)// Address Filter Register
#define ETH_SAL_HT1			(ETHC_BASE + 0x1a4)// Hash Table [64 : 32]
#define ETH_SAL_HT2			(ETHC_BASE + 0x1a8)// Hash Table [31 : 0 ]

// Constants for ETH_SAL_AFR
#define AFR_PRO				(1 << 3)	// Promiscuous mode
#define AFR_PRM				(1 << 2)	// Accept Multicast
#define AFR_AMC				(1 << 1)	// Accept Multicast qualified
#define AFR_ABC				(1 << 0)	// Accept Broadcast

// ----------------------------------------------------------------------------------
// 			MISC definition
// ----------------------------------------------------------------------------------

// Receive Status Vector
#define RSV_RVTD			(1 << 30)	// Receive VLAN Type detected
#define RSV_RUO				(1 << 29)	// Receive Unsupported Op-code
#define RSV_RPCF			(1 << 28)	// Receive Pause Control Frame
#define RSV_RCF				(1 << 27)	// Receive Control Frame
#define RSV_DN				(1 << 26)	// Dribble Nibble
#define RSV_BP				(1 << 25)	// Broadcast Packet
#define RSV_MP				(1 << 24)	// Multicast Packet
#define RSV_OK				(1 << 23)	// Receive OK
#define RSV_LOR				(1 << 22)	// Length Out of Range
#define RSV_LCE				(1 << 21)	// Length Check Error
#define RSV_CRCE			(1 << 20)	// CRC Error
#define RSV_RCV				(1 << 19)	// Receive Code Violation
#define RSV_CEPS			(1 << 18)	// Carrier Event Previously Seen
#define RSV_REPS			(1 << 17)	// RXDV Event Previously Seen
#define RSV_PPI				(1 << 16)	// Packet Previously Ignored

// ----------------------------------------------------------------------------------
// 			MII Registers and Definitions
// ----------------------------------------------------------------------------------

#define MII_BMCR	0x00		/* MII Management Control Register */
#define MII_BMSR	0x01		/* MII Management Status Register */
#define MII_ID1		0x02		/* PHY Identifier Register 0 */
#define MII_ID2		0x03		/* PHY Identifier Register 1 */
#define MII_ANAR	0x04		/* Auto Negotiation Advertisement */
#define MII_ANLPAR	0x05		/* Auto Negotiation Link Partner Ability */
#define MII_ANER	0x06		/* Auto Negotiation Expansion */
#define MII_ANP		0x07		/* Auto Negotiation Next Page TX */
#define MII_DSCR 	0x10		/* Davicom Specified Configration Register */
#define MII_DSCSR	0x11		/* Davicom Specified Configration/Status Register */
#define MII_10BTCSR	0x12		/* 10base-T Specified Configration/Status Register */

#define MII_PREAMBLE	0xffffffff	/* MII Management Preamble */
#define MII_TEST	0xaaaaaaaa	/* MII Test Signal */
#define MII_STRD	0x06		/* Start of Frame+Op Code: use low nibble */
#define MII_STWR	0x0a		/* Start of Frame+Op Code: use low nibble */

// MII Management Control Register
#define MII_CR_RST	0x8000         /* RESET the PHY chip */
#define MII_CR_LPBK	0x4000         /* Loopback enable */
#define MII_CR_SPD	0x2000         /* 0: 10Mb/s; 1: 100Mb/s */
#define MII_CR_10	0x0000         /* Set 10Mb/s */
#define MII_CR_100	0x2000         /* Set 100Mb/s */
#define MII_CR_ASSE	0x1000         /* Auto Speed Select Enable */
#define MII_CR_PD	0x0800         /* Power Down */
#define MII_CR_ISOL	0x0400         /* Isolate Mode */
#define MII_CR_RAN	0x0200         /* Restart Auto Negotiation */
#define MII_CR_FDM	0x0100         /* Full Duplex Mode */
#define MII_CR_CTE	0x0080         /* Collision Test Enable */

// MII Management Status Register
#define MII_SR_T4C	0x8000         /* 100BASE-T4 capable */
#define MII_SR_TXFD	0x4000         /* 100BASE-TX Full Duplex capable */
#define MII_SR_TXHD	0x2000         /* 100BASE-TX Half Duplex capable */
#define MII_SR_TFD	0x1000         /* 10BASE-T Full Duplex capable */
#define MII_SR_THD	0x0800         /* 10BASE-T Half Duplex capable */
#define MII_SR_ASSC	0x0020         /* Auto Speed Selection Complete*/
#define MII_SR_RFD	0x0010         /* Remote Fault Detected */
#define MII_SR_ANC	0x0008         /* Auto Negotiation capable */
#define MII_SR_LKS	0x0004         /* Link Status */
#define MII_SR_JABD	0x0002         /* Jabber Detect */
#define MII_SR_XC	0x0001         /* Extended Capabilities */

// MII Management Auto Negotiation Advertisement Register
#define MII_ANAR_TAF	0x03e0         /* Technology Ability Field */
#define MII_ANAR_T4AM	0x0200         /* T4 Technology Ability Mask */
#define MII_ANAR_TXAM	0x0180         /* TX Technology Ability Mask */
#define MII_ANAR_FDAM	0x0140         /* Full Duplex Technology Ability Mask */
#define MII_ANAR_HDAM	0x02a0         /* Half Duplex Technology Ability Mask */
#define MII_ANAR_100M	0x0380         /* 100Mb Technology Ability Mask */
#define MII_ANAR_10M	0x0060         /* 10Mb Technology Ability Mask */
#define MII_ANAR_CSMA	0x0001         /* CSMA-CD Capable */

// MII Management Auto Negotiation Remote End Register
#define MII_ANLPAR_NP	0x8000         /* Next Page (Enable) */
#define MII_ANLPAR_ACK	0x4000         /* Remote Acknowledge */
#define MII_ANLPAR_RF	0x2000         /* Remote Fault */
#define MII_ANLPAR_TAF	0x03e0         /* Technology Ability Field */
#define MII_ANLPAR_T4AM	0x0200         /* T4 Technology Ability Mask */
#define MII_ANLPAR_TXAM	0x0180         /* TX Technology Ability Mask */
#define MII_ANLPAR_FDAM	0x0140         /* Full Duplex Technology Ability Mask */
#define MII_ANLPAR_HDAM	0x02a0         /* Half Duplex Technology Ability Mask */
#define MII_ANLPAR_100M	0x0380         /* 100Mb Technology Ability Mask */
#define MII_ANLPAR_10M	0x0060         /* 10Mb Technology Ability Mask */
#define MII_ANLPAR_CSMA	0x0001         /* CSMA-CD Capable */

// Media / mode state machine definitions
// User selectable:
#define TP              0x0040	       /* 10Base-T (now equiv to _10Mb)        */
#define TP_NW           0x0002         /* 10Base-T with Nway                   */
#define BNC             0x0004         /* Thinwire                             */
#define AUI             0x0008         /* Thickwire                            */
#define BNC_AUI         0x0010         /* BNC/AUI on DC21040 indistinguishable */
#define _10Mb           0x0040         /* 10Mb/s Ethernet                      */
#define _100Mb          0x0080         /* 100Mb/s Ethernet                     */
#define AUTO            0x4000         /* Auto sense the media or speed        */

// Internal states
#define NC              0x0000         /* No Connection                        */
#define ANS             0x0020         /* Intermediate AutoNegotiation State   */
#define SPD_DET         0x0100         /* Parallel speed detection             */
#define INIT            0x0200         /* Initial state                        */
#define EXT_SIA         0x0400         /* External SIA for motherboard chip    */
#define ANS_SUSPECT     0x0802         /* Suspect the ANS (TP) port is down    */
#define TP_SUSPECT      0x0803         /* Suspect the TP port is down          */
#define BNC_AUI_SUSPECT 0x0804         /* Suspect the BNC or AUI port is down  */
#define EXT_SIA_SUSPECT 0x0805         /* Suspect the EXT SIA port is down     */
#define BNC_SUSPECT     0x0806         /* Suspect the BNC port is down         */
#define AUI_SUSPECT     0x0807         /* Suspect the AUI port is down         */
#define MII             0x1000         /* MII on the 21143                     */

// ----------------------------------------------------------------------------------
// 			Device data and structure
// ----------------------------------------------------------------------------------

#define READ_COMMAND		(SIOCDEVPRIVATE+4)
#define WRITE_COMMAND		(SIOCDEVPRIVATE+5)
#define GETDRIVERINFO		(SIOCDEVPRIVATE+6)

#define ETH_TX_TIMEOUT		(6*HZ)

#define RX_BUF_SIZE		(1536 + 64)

#define NUM_RX_DESCS		64
#define NUM_TX_DESCS		16

#define MAX_WAIT		2000

#define STAT_INTERVAL		HZ * 3

// Constants for DMA descriptor
#define EMPTY_FLAG_MASK			(0x01 << 31)
#define FTPP_FLAGS_MASK			(0x1f << 16)
#define FTCFRM_MASK			(0x01 << 20)
#define FTPP_PADMODE_MASK		(0x03 << 18)
#define FTPP_GENFCS_MASK		(0x01 << 17)
#define FTPP_EN_MASK			(0x01 << 16)

#define PKT_SIZE_MASK			(0x0FFF)

/*
static const char *media_types[] = {
	"10BaseT-HD ", "10BaseT-FD ","100baseTx-HD ", 
	"100baseTx-FD", "100baseT4", 0
};
*/

typedef struct {
	unsigned int pkt_addr;	// phy addr
	unsigned int pkt_size;	// include empty flag
	unsigned int next_desc;	// phy addr

	unsigned int for_align; // for 4-word alignment
} jz_desc_t;


enum counters{
//	CNT_TR64,
//	CNT_TR127,
//	CNT_TR255,
//	CNT_TR511,
//	CNT_TR1K,
//	CNT_TRMAX,
//	CNT_TRMGV,
	CNT_RBYT,
	CNT_RPKT,
	CNT_RFCS,
	CNT_RMCA,
//	CNT_RBCA,
//	CNT_RXCF,
//	CNT_RXPF,
//	CNT_RXUO,
//	CNT_RALN,
//	CNT_RFLR,
//	CNT_RCDE,
//	CNT_RCSE,
//	CNT_RUND,
//	CNT_ROVR,
//	CNT_RFRG,
//	CNT_RJBR,
	CNT_RDRP,
	CNT_TBYT,
	CNT_TPKT,
//	CNT_TMCA,
//	CNT_TBCA,
//	CNT_TXPF,
//	CNT_TDFR,
//	CNT_TEDF,
//	CNT_TSCL,
//	CNT_TMCL,
//	CNT_TLCL,
//	CNT_TXCL,
	CNT_TNCL,
//	CNT_TPFH,
	CNT_TDRP,
//	CNT_TJBR,
	CNT_TFCS,
//	CNT_TXCF,
//	CNT_TOVR,
//	CNT_TUND,
//	CNT_TFRG,

	STAT_CNT_NUM
};


/* Allocate by alloc_etherdev, which could ensure the struct was 32-byte align */
struct jz_eth_private {

	jz_desc_t tx_ring[NUM_TX_DESCS];	/* transmit descriptors, 4-word align */
	jz_desc_t rx_ring[NUM_RX_DESCS];	/* receive descriptors, 4-word align */

	dma_addr_t dma_tx_ring;                 /* bus address of tx ring */
	dma_addr_t dma_rx_ring;                 /* bus address of rx ring */
	dma_addr_t dma_rx_buf;			/* DMA address of rx buffer */
	unsigned int vaddr_rx_buf;		/* virtual address of rx buffer */

	unsigned int rx_head;			/* first rx descriptor */
	unsigned int tx_head;			/* first tx descriptor */
	unsigned int tx_tail;  			/* last unacked transmit packet */
	unsigned int tx_full;			/* transmit buffers are full */
	struct sk_buff *tx_skb[NUM_TX_DESCS];	/* skbuffs for packets to transmit */

	struct net_device_stats stats;
	unsigned int carry_counters[STAT_CNT_NUM];

	spinlock_t lock;

	int media;				/* Media (eg TP), mode (eg 100B)*/
	int full_duplex;			/* Current duplex setting */
	int link_state;
	char phys[32];				/* List of attached PHY devices */
	char valid_phy;				/* Current linked phy-id with MAC */
	int mii_phy_cnt;
	int phy_type;				/* 1-RTL8309,0-DVCOM */
	struct ethtool_cmd ecmds[32];
	u16 advertising;			/* NWay media advertisement */

	struct task_struct *thread;		/* Link cheak thread */
	int thread_die;
	struct completion thr_exited;
	wait_queue_head_t thr_wait;

	struct pm_dev *pmdev;
};

// ----------------------------------------------------------------------------------
// 			Operation defination
// ----------------------------------------------------------------------------------

// Operations of ETH DMA
#define __eth_dma_tx_enable()					\
do {								\
	REG32(ETH_DMA_TCR) |= TCR_ENABLE;			\
} while(0)

#define __eth_dma_tx_disable()					\
do {								\
	REG32(ETH_DMA_TCR) &= ~TCR_ENABLE;			\
} while(0)

#define __eth_dma_set_burst_len(len_macro)			\
do {								\
	REG32(ETH_DMA_IMR) &= ~(BURST_LEN_LO | BURST_LEN_HI);	\
	REG32(ETH_DMA_IMR) |= (len_macro);			\
} while(0)

#define __eth_set_tx_desc_addr(desc_addr)			\
do {								\
	REG32(ETH_DMA_TDR) = desc_addr;				\
} while(0)

#define __eth_get_flag_pkt_sent()		(REG32(ETH_DMA_TSR) & TSR_PKTSENT)
#define __eth_get_flag_underrun()		(REG32(ETH_DMA_TSR) & TSR_UNDERRUN)
#define __eth_get_flag_tx_bus_err()		(REG32(ETH_DMA_TSR) & TSR_BUSERR)
#define __eth_get_tx_pkt_cnt()			((REG32(ETH_DMA_TSR) & TSR_PKTCNT_MASK) >> 0x10)

#define __eth_reduce_pkt_sent_cnt()		(REG32(ETH_DMA_TSR) |= TSR_PKTSENT)
#define __eth_clear_flag_underrun()		(REG32(ETH_DMA_TSR) |= TSR_UNDERRUN)
#define __eth_clear_flag_tx_bus_err()		(REG32(ETH_DMA_TSR) |= TSR_BUSERR)
#define __eth_clear_tx_pkt_cnt()				\
do {								\
	unsigned int tmp_cnt = __eth_get_tx_pkt_cnt();		\
	for (; tmp_cnt > 0; tmp_cnt--)				\
		__eth_reduce_pkt_sent_cnt();			\
} while(0)



#define __eth_dma_rx_enable()					\
do {								\
	REG32(ETH_DMA_RCR) |= RCR_ENABLE;			\
} while(0)

#define __eth_dma_rx_disable()					\
do {								\
	REG32(ETH_DMA_RCR) &= ~RCR_ENABLE;			\
} while(0)

#define __eth_set_rx_desc_addr(desc_addr)			\
do {								\
	REG32(ETH_DMA_RDR) = desc_addr;				\
} while(0)

#define __eth_get_flag_pkt_recv()		(REG32(ETH_DMA_RSR) & RSR_PKTRECV)
#define __eth_get_flag_overflow()		(REG32(ETH_DMA_RSR) & RSR_OVERFLOW)
#define __eth_get_flag_rx_bus_err()		(REG32(ETH_DMA_RSR) & RSR_BUSERR)
#define __eth_get_rx_pkt_cnt()			((REG32(ETH_DMA_RSR) & RSR_PKTCNT_MASK) >> 0x10)

#define __eth_reduce_pkt_recv_cnt()		(REG32(ETH_DMA_RSR) |= RSR_PKTRECV)
#define __eth_clear_flag_overflow()		(REG32(ETH_DMA_RSR) |= RSR_OVERFLOW)
#define __eth_clear_flag_rx_bus_err()		(REG32(ETH_DMA_RSR) |= RSR_BUSERR)
#define __eth_clear_rx_pkt_cnt()				\
do {								\
	unsigned int tmp_cnt = __eth_get_rx_pkt_cnt();		\
	for (; tmp_cnt > 0; tmp_cnt--)				\
		__eth_reduce_pkt_recv_cnt();			\
} while(0)

#define __eth_clear_rx_flags()					\
do {								\
	/*__eth_clear_rx_pkt_cnt();*/				\
	REG32(ETH_DMA_RSR) = RSR_OVERFLOW | RSR_BUSERR;		\
} while(0)

#define __eth_clear_tx_flags()					\
do {								\
	/*__eth_clear_tx_pkt_cnt();*/				\
	REG32(ETH_DMA_TSR) = TSR_UNDERRUN | TSR_BUSERR;		\
} while(0)

#define __eth_enable_irq()					\
do {								\
	REG32(ETH_DMA_IMR) |= IMR_ALL_IRQ;			\
} while(0)

#define __eth_disable_irq()					\
do {								\
	REG32(ETH_DMA_IMR) &= ~IMR_ALL_IRQ;			\
} while(0)

#define __eth_enable()						\
do {								\
	__eth_dma_tx_enable();					\
	__eth_dma_rx_enable();					\
} while(0)

#define __eth_disable()						\
do {								\
	__eth_dma_tx_disable();					\
	__eth_dma_rx_disable();					\
} while(0)

#define __eth_set_mac_address(b1, b2, b3, b4, b5, b6)		\
do {								\
	REG16(ETH_MAC_SA0) = ((b1) << 8) | ((b2) & 0xff);	\
	REG16(ETH_MAC_SA1) = ((b3) << 8) | ((b4) & 0xff);	\
	REG16(ETH_MAC_SA2) = ((b5) << 8) | ((b6) & 0xff);	\
} while(0)


// Operations of ETH MAC registers
#define __mac_reset()						\
do {								\
	REG16(ETH_MAC_MCR1) |= MCR1_SOFTRST;			\
	udelay(1000);						\
	REG16(ETH_MAC_MCR1) &= ~MCR1_SOFTRST;			\
} while(0)

#define __mac_get_IPGR()			(REG16(ETH_MAC_IPGR) & IPGR_MASK)

#define __mac_set_IPGR(ipgt)					\
do {								\
	REG16(ETH_MAC_IPGR) = ipgt;				\
} while(0)

#define __mac_get_NIPGR1()			((REG16(ETH_MAC_NIPGR) & NIPGR_P1_MASK) >> 8)

#define __mac_set_NIPGR1(v1)					\
do {								\
	REG16(ETH_MAC_NIPGR) |= ((v1) << 8) & NIPGR_P1_MASK;	\
} while(0)

#define __mac_get_NIPGR2()			(REG16(ETH_MAC_NIPGR) & NIPGR_P2_MASK)

#define __mac_set_NIPGR2(v2)					\
do {								\
	REG16(ETH_MAC_NIPGR) |= (v2) & NIPGR_P2_MASK;		\
} while(0)

#define __mac_get_collision_window()		((REG16(ETH_MAC_CWR) & CWR_CW_MASK) >> 8)

#define __mac_set_collision_window(cw)				\
do {								\
	REG16(ETH_MAC_CWR) |= ((cw) << 8) & CWR_CW_MASK;	\
} while(0)

#define __mac_get_retx_maximum()		(REG16(ETH_MAC_CWR) & CWR_RM_MASK)

#define __mac_set_retx_maximum(rm)				\
do {								\
	REG16(ETH_MAC_CWR) |= (rm) & CWR_RM_MASK;		\
} while(0)

#define __mac_get_max_frame_length()		(REG16(ETH_MAC_MFR))

#define __mac_set_max_frame_length(len)				\
do {								\
	REG16(ETH_MAC_MFR) = len;				\
} while(0)

#define __mac_set_mii_clk(_clkdiv)					\
do {									\
	REG16(ETH_MAC_MCFGR) |= ((_clkdiv) << 2) & MCFGR_CS_MASK;	\
} while(0)

#define __mac_set_mii_address(pa, ra)					\
do {									\
	REG16(ETH_MAC_MADRR) =	 					\
		(((pa) << 8)& MADRR_PA_MASK)|((ra) & MADRR_RA_MASK);	\
} while(0)


#define __mac_send_mii_read_cmd(pa, ra, scan)				\
do {									\
	__mac_set_mii_address(pa, ra);					\
	REG16(ETH_MAC_MCMDR) &= ~MCMDR_SCAN & ~MCMDR_READ;		\
	REG16(ETH_MAC_MCMDR) |=	MCMDR_READ | ((scan) & MCMDR_SCAN);	\
} while(0)

#define __mac_send_mii_write_cmd(pa, ra, wdata)				\
do {									\
	__mac_set_mii_address(pa, ra);					\
	REG16(ETH_MAC_MCMDR) &= ~MCMDR_SCAN & ~MCMDR_READ;		\
	__mac_mii_write_data(wdata);					\
} while(0)

#define __mac_mii_write_data(_2byte)		(REG16(ETH_MAC_MWTDR) = _2byte)

#define __mac_mii_read_data()			REG16(ETH_MAC_MRDDR)

#define __mac_mii_is_busy()			(REG16(ETH_MAC_MINDR) & MINDR_BUSY)

// Operations of ETH FIFO registers
#define __fifo_enable_all_modules()					\
do {									\
	REG32(ETH_FIFO_CR0) |= ALL_EN_REQ;				\
} while(0)

#define __fifo_enable_all_modules_finish()				\
do {									\
	REG32(ETH_FIFO_CR0) &= ~ALL_EN_REQ;				\
} while(0)

// All enabled mean all reply bits were set except SRF_EN_RPLY...
#define __fifo_all_enabled()	((REG32(ETH_FIFO_CR0) & (ALL_EN_RPLY & ~SRF_EN_RPLY)) == (ALL_EN_RPLY & ~SRF_EN_RPLY))

#define __fifo_reset_all()					\
do {								\
	REG32(ETH_FIFO_CR0) |= ALL_RST;				\
	REG32(ETH_FIFO_CR0) &= ~ALL_RST;			\
} while(0)

#define __fifo_set_fr_threshold(_th)				\
do {								\
	REG32(ETH_FIFO_CR1) &= ~CFG_FR_TH_MASK;			\
	REG32(ETH_FIFO_CR1) |= ((_th) << 16) & CFG_FR_TH_MASK;	\
} while(0)

#define __fifo_set_XOFF_RTX(_th)				\
do {								\
	REG32(ETH_FIFO_CR1) &= ~CFG_XOFF_RTX_MASK;		\
	REG32(ETH_FIFO_CR1) |= (_th) & CFG_XOFF_RTX_MASK;	\
} while(0)

#define __fifo_set_high_wm(_th)					\
do {								\
	REG32(ETH_FIFO_CR2) &= ~CFG_H_WM_MASK;			\
	REG32(ETH_FIFO_CR2) |= ((_th) << 16) & CFG_H_WM_MASK;	\
} while(0)

#define __fifo_set_low_wm(_th)					\
do {								\
	REG32(ETH_FIFO_CR2) &= ~CFG_L_WM_MASK;			\
	REG32(ETH_FIFO_CR2) |= (_th) & CFG_L_WM_MASK;		\
} while(0)

#define __fifo_set_ft_high_wm(_th)				\
do {								\
	REG32(ETH_FIFO_CR3) &= ~CFG_H_WM_FT_MASK;		\
	REG32(ETH_FIFO_CR3) |= ((_th) << 16) & CFG_H_WM_FT_MASK;	\
} while(0)

#define __fifo_set_ft_threshold(_th)				\
do {								\
	REG32(ETH_FIFO_CR3) &= ~CFG_FT_TH_MASK;			\
	REG32(ETH_FIFO_CR3) |= (_th) & CFG_FT_TH_MASK;		\
} while(0)

#define __fifo_set_drop_cond(_cdt)				\
do {								\
	REG32(ETH_FIFO_CR4) |= ((_cdt) >> 16) & HST_FLT_RFRM_MASK;	\
} while(0)

#define __fifo_set_dropdc_cond(_cdt)				\
do {								\
	REG32(ETH_FIFO_CR5) &= ~(((_cdt)>>16) & HST_FLT_RFRMDC_MASK);	\
} while(0)

#define __fifo_set_pause_control()				\
do {								\
	REG32(ETH_FIFO_CR5) &= ~CFG_H_DPLX;			\
} while(0)

#define __fifo_set_clk_enable_mode()				\
do {								\
	REG32(ETH_FIFO_CR5) |= CLK_EN_MODE;			\
} while(0)

#define __fifo_set_half_duplex()				\
do {								\
	REG32(ETH_FIFO_CR5) |= CFG_H_DPLX;			\
} while(0)

#define __fifo_drop_lt64_frame()				\
do {								\
	REG32(ETH_FIFO_CR5) |= HST_DR_LT_64;			\
} while(0)

// Operations of ETH STAT registers
#define __stat_enable()						\
do {								\
	REG32(ETH_STAT_CR) |= STAT_EN;				\
} while(0)

#define __stat_disable()					\
do {								\
	REG32(ETH_STAT_CR) &= ~STAT_EN;				\
} while(0)

#define __stat_clear_counters()					\
do {								\
	REG32(ETH_STAT_CR) &= ~STAT_CL;				\
	REG32(ETH_STAT_CR) |= STAT_CL;				\
} while(0)

#define __stat_enable_auto_zero()				\
do {								\
	REG32(ETH_STAT_CR) |= STAT_AZ;				\
} while(0)

#define __stat_disable_auto_zero()				\
do {								\
	REG32(ETH_STAT_CR) &= ~STAT_AZ;				\
} while(0)

#define __stat_enable_carry_irq()				\
do {								\
	__stat_unmask1(CARM1_RPK | CARM1_RBY | CARM1_RFC | CARM1_RDR | CARM1_RMC);\
	__stat_unmask2(CARM2_TPK | CARM2_TBY | CARM2_TFC | CARM2_TDP | CARM2_TNC);\
} while(0)

#define __stat_disable_carry_irq()				\
do {								\
	__stat_unmask1(0);					\
	__stat_unmask2(0);					\
} while(0)

#define __stat_unmask1(_counter)				\
do {								\
	REG32(ETH_STAT_CARM1) = ~(_counter);			\
} while(0)

#define __stat_unmask2(_counter)				\
do {								\
	REG32(ETH_STAT_CARM2) = ~(_counter);			\
} while(0)

// Operations of ETH SAL registers

// Anyhow, enable broadcast ...
#define __sal_set_mode(_mode)					\
do {								\
	REG32(ETH_SAL_AFR) = (_mode) | AFR_ABC;			\
} while(0)

#define __sal_set_hash_table(_hi32, _lo32)			\
do {								\
	REG32(ETH_SAL_HT1) = _hi32;				\
	REG32(ETH_SAL_HT2) = _lo32;				\
} while(0)

#define __sal_get_hash_table(_hi32, _lo32)			\
do {								\
	_hi32 = REG32(ETH_SAL_HT1);				\
	_lo32 = REG32(ETH_SAL_HT2);				\
} while(0)

// Operations of DMA descripter
#define __desc_get_empty_flag(pktsize)		(pktsize & EMPTY_FLAG_MASK)
#define __desc_get_FTPP_flags(pktsize)		((pktsize & FTPP_FLAGS_MASK) >> 16)
#define __desc_get_FTCFRM_flag(pktsize)		(pktsize & FTCFRM_MASK)
#define __desc_get_FTPP_PADMODE_flag(pktsize)	((pktsize & FTPP_PADMODE_MASK) >> 18)
#define __desc_get_FTPP_GENFCS_flag(pktsize)	(pktsize & FTPP_GENFCS_MASK)
#define __desc_get_FTPP_enable_flag(pktsize)	(pktsize & FTPP_EN_MASK)
#define __desc_get_pkt_size(pktsize)		(pktsize & PKT_SIZE_MASK)



#endif
