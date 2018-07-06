/*
 * linux/arch/mips/include/asm/mach-jz4760/spi.h
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 *
 * Copyright (c) 2010 Ingenic Semiconductor Inc.
 * Author: Shumb <sbhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#ifndef __I_SPI_H__
#define __I_SPI_H__

#define R_MODE		0x1
#define W_MODE		0x2
#define RW_MODE	(R_MODE | W_MODE)

#define R_DMA		0x4
#define W_DMA		0x8
#define RW_DMA		(R_DMA |W_DMA)

#define SPI_DMA_ERROR  		3
#define SPI_CPU_ERROR		4

#define JZ_SSI_MAX_FIFO_ENTRIES 	128
#define JZ_SSI_DMA_BURST_LENGTH 	16

#define FIFO_8_BIT		1
#define FIFO_16_BIT		2
#define FIFO_32_BIT		4


/* the max number of spi devices */
#define MAX_SPI_DEVICES				10

#define PIN_SSI_CE0 	0
#define PIN_SSI_CE1 	1

struct jz47xx_spi_info {
	u8	chnl;								/* the chanel of SSI controller */
	u16	bus_num;							/* spi_master.bus_num */
	unsigned is_pllclk:1;					/* source clock: 1---pllclk;0---exclk */
	unsigned long		 board_size;		/* spi_master.num_chipselect */
	struct spi_board_info	*board_info; 	/* link to spi devices info */
	void (*set_cs)(struct jz47xx_spi_info *spi, u8 cs,unsigned int pol); /* be defined by spi devices driver user */
	void (*pins_config)(void);				/* configure spi function pins (CLK,DR,RT) by user if need. */
	u32		 pin_cs[MAX_SPI_DEVICES];		/* the member is pin_value according to spi_device.chip_select,
												if uses spi controller driver "set_cs",it must be filled.*/
};

/* Chipselect "set_cs" function could be defined by user. Example as the follow ... */
/*
static void spi_gpio_cs(struct jz47xx_spi_info *spi, int cs, int pol);
{
	int pinval;

	switch(cs){
	case 0:
		pinval = 32*1+31;
		break;
	case 1:
		pinval = 32*1+30;
		break;
	case 2:
		pinval = 32*1+29;
		break;
	default:
		pinval = 32*1+28;
		break;	
	}
	__gpio_as_output(pinval);				
	switch (pol) {
	case BITBANG_CS_ACTIVE:
		__gpio_set_pin(pinval);
		break;
	case BITBANG_CS_INACTIVE:
		__gpio_clear_pin(pinval);
		break;
	}

}*/

#endif
