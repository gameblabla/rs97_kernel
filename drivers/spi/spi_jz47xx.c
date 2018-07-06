/* linux/drivers/spi/spi_jz47xx.c
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 * base-to: linux/drivers/spi/spi_bitbang.c
 *
 * Copyright (c) 2010 Ingenic
 * Author:Shumb <sbhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h> 


//#define SSI_DEGUG
#ifdef SSI_DEGUG
#define SSI_KEY_DEGUG
#define SSI_MSG
#define  print_dbg(format,arg...)			\
	printk(format,## arg)
#else
#define  print_dbg(format,arg...)	
#endif

//#define SSI_KEY_DEGUG
#ifdef SSI_KEY_DEGUG
#define print_kdbg(format,arg...)			\
	printk(format,## arg)
#else
#define print_kdbg(format,arg...)	
#endif

//#define SSI_MSG
#ifdef SSI_MSG
#define print_msg(format,arg...)			\
	printk(format,## arg)
#else
#define print_msg(format,arg...)	
#endif


/* tx rx threshold from 0x0 to 0xF */
#define SSI_TX_FIFO_THRESHOLD		0x1
#define SSI_RX_FIFO_THRESHOLD		(0xF - SSI_TX_FIFO_THRESHOLD)

#define CPU_ONCE_BLOCK_ENTRIES 		((0xF-SSI_TX_FIFO_THRESHOLD)*8)

#define MAX_SSI_INTR		10000

#ifdef CONFIG_SOC_JZ4760  
#define MAX_SSICDR	63
#define MAX_CGV		255
#else
#define MAX_SSICDR	15
#define MAX_CGV		255
#endif

#define GPIO_AS_SSI(n)	 	\
do{								\
	if(n) __gpio_as_ssi1();		\
	else __gpio_as_ssi0();		\
}while(0)
#define GPIO_AS_SSI_EX(n)	 	\
do{								\
	if(n) __gpio_as_ssi1_x();		\
	else __gpio_as_ssi0_x();		\
}while(0)

#if defined(CONFIG_SOC_JZ4760)
extern unsigned int cpm_get_clock(cgu_clock); 	
#define CPM_SSI_START(n) 	((n) ?cpm_start_clock(CGM_SSI1):cpm_start_clock(CGM_SSI0))
#define CPM_SSI_STOP(n)		((n) ?cpm_stop_clock(CGM_SSI1):cpm_stop_clock(CGM_SSI0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*3+29)
#define SSI1_CE1_PIN	(32*3+30)

#elif defined(CONFIG_SOC_JZ4750D) /* jz4755 */
#define CPM_SSI_START(n) 	((n) ? :__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? :__cpm_stop_ssi(0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*1+29)		/* same as SSI0, for avoiding compilation error and ... */
#define SSI1_CE1_PIN	(32*1+31)
#else
#define CPM_SSI_START(n) 	((n) ? __cpm_start_ssi(1):__cpm_start_ssi(0))
#define CPM_SSI_STOP(n)		((n) ? __cpm_stop_ssi(1):__cpm_stop_ssi(0))

#define SSI0_CE0_PIN	(32*1+29)
#define SSI0_CE1_PIN	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define SSI1_CE0_PIN	(32*3+29)
#define SSI1_CE1_PIN	(32*3+30)
#endif

struct jz47xx_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;
	
	u8			 chnl;
	u8			 rw_mode;

	u8			 use_dma;
	u8			 is_first;
	
	void __iomem		*regs;
	int			 irq;
	u32			 len;
	u32			 rlen;	  /* receive len */
	u32			 count;   /* sent count */

	u8			 bits_per_word;		/*8 or 16 (or 32)*/
	u8			 transfer_unit_size;	/* 1 or 2 (or 4) */
	u8			 tx_trigger;					/* 0-128 */
	u8			 rx_trigger;					/* 0-128 */
	u8			 dma_tx_unit;			/* 1 or 2 or 4 or 16 or 32*/
	u8			 dma_rx_unit;			/* 1 or 2 or 4 or 16 or 32*/
	void			(*set_cs)(struct jz47xx_spi_info *spi, u8 cs, unsigned int pol);
	
	/* functions to deal with different size buffers */
	void (*get_rx) (u16 rx_data, struct jz47xx_spi *);
	u16 (*get_tx) (struct jz47xx_spi *);
	
	/* data buffers */
	const u8	*tx;
	u8			*rx;

	int dma_tx_chnl;                 /* dma tx channel                  */
	int dma_rx_chnl;                 /* dma rx channel                  */
	
	dma_addr_t	tx_dma;
	dma_addr_t	rx_dma;
	
	unsigned long src_clk;
	unsigned long spi_clk;
	
	struct jz_intr_cnt *g_jz_intr;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct device		*dev;
	struct jz47xx_spi_info *pdata;
};


struct jz_intr_cnt{
	int dma_tx_cnt;
	int dma_rx_cnt;
	int ssi_intr_cnt;
	int ssi_txi;
	int ssi_rxi;
	int ssi_eti;
	int ssi_eri;
	int ssi_rlen;
	int dma_tx_err;
	int dma_tx_end;
	int dma_rx_err;
	int dma_rx_end;
};

static inline struct jz47xx_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void jz47xx_spi_cs(struct jz47xx_spi_info *spi, u8 cs, unsigned int pol)
{
	u32 pin_value = spi->pin_cs[cs];

	__gpio_as_output(pin_value);
	if(!pol)
		__gpio_clear_pin(pin_value);
	else
		__gpio_set_pin(pin_value);
	
	print_msg("GPIO_PIN:0x%04X  LEVEL: %d:%d\n",
		pin_value,pol,__gpio_get_pin(pin_value));

}
static void jz47xx_spi_chipsel(struct spi_device *spi, int value)
{
	struct jz47xx_spi *hw = to_hw(spi);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

	switch (value) {
	case BITBANG_CS_INACTIVE:
		if(hw->set_cs)
			hw->set_cs(hw->pdata,spi->chip_select, cspol^1);
		break;

	case BITBANG_CS_ACTIVE:

		if (spi->mode & SPI_CPHA)
			__ssi_set_spi_clock_phase(hw->chnl,1);
		else
			__ssi_set_spi_clock_phase(hw->chnl,0);
		
		if (spi->mode & SPI_CPOL)
			__ssi_set_spi_clock_polarity(hw->chnl,1);
		else
			__ssi_set_spi_clock_polarity(hw->chnl,0);

		if (!(spi->mode & SPI_LSB_FIRST))
			__ssi_set_msb(hw->chnl);
		else
			__ssi_set_lsb(hw->chnl);
		
		if(spi->mode & SPI_LOOP)
			__ssi_enable_loopback(hw->chnl);
		else
			__ssi_disable_loopback(hw->chnl);

		if(hw->set_cs)
			hw->set_cs(hw->pdata,spi->chip_select, cspol);

		break;
	default:
		break;
	}
}

#define JZ47XX_SPI_RX_BUF(type) 			\
void jz47xx_spi_rx_buf_##type(u16 data, struct jz47xx_spi *hw) \
{									  		\
	type * rx = (type *)hw->rx;				\
	*rx++ = (type)(data);			  		\
	hw->rx = (u8 *)rx;						\
}

#define JZ47XX_SPI_TX_BUF(type)				\
u16 jz47xx_spi_tx_buf_##type(struct jz47xx_spi *hw)	\
{										\
	u16 data;							\
	const type * tx = (type *)hw->tx;	\
	data = *tx++;						\
	hw->tx = (u8 *)tx;					\
	return data;						\
}

JZ47XX_SPI_RX_BUF(u8)
JZ47XX_SPI_RX_BUF(u16)
JZ47XX_SPI_TX_BUF(u8)
JZ47XX_SPI_TX_BUF(u16)

static unsigned int jz_spi_get_clk(struct spi_device *spi)
{
	struct jz47xx_spi *hw = to_hw(spi);
	int ssicdr,cgv;
	unsigned long clk;
	
	ssicdr 	= __cpm_get_ssidiv();
	cgv 	= __ssi_get_grdiv(hw->chnl);
#ifdef CONFIG_SOC_JZ4760
	if(!hw->pdata->is_pllclk)
		ssicdr = 0;
#endif
	clk = hw->src_clk/(2*(cgv+1)*(ssicdr+1));
	hw->spi_clk = clk;
	return clk;
}

static unsigned int jz_spi_set_clk(struct spi_device *spi,u32 hz)
{
	struct jz47xx_spi *hw = to_hw(spi);
	int div,ssicdr=0,cgv=0;
	u32 ssiclk;

	ssiclk = hw->src_clk;

	div = ssiclk/ hz;

	/* clk = (exclk or pllclk)/( (SSICDR +1) * (2*(CGV+1)) )    */
	/* 4750: SSICDR (0-15)   CGV (0-255)   16*(2*256)     		*/
	/* 4760: SSICDR (0-63)   CGV (0-255)   64*(2*256)	  		*/

	if(hw->pdata->is_pllclk){  		/* source clock is PLLCLK */
		if(div <= 2*(MAX_SSICDR+1)){
			cgv    = 0;
			ssicdr = div/2 -1;
		}else if(div <= 2*(MAX_CGV+1)*(MAX_SSICDR+1)){	
			ssicdr = MAX_SSICDR;
			cgv    = (div/(MAX_SSICDR+1))/2 - 1;

		}else{
			ssicdr = MAX_SSICDR;
			cgv	   = MAX_CGV;
		}	
	}else{  						/* source clock is EXCLK */
		if(div <= 2*(MAX_CGV+1)){	
			cgv    = div/2- 1;
		}else
			cgv	   = MAX_CGV;
	}

	if (cgv < 0)
		cgv = 0;
	if (ssicdr < 0)
		ssicdr = 0;
	
	dev_dbg(&spi->dev,"SSICLK:%d setting pre-scaler to %d (hz %d) SSICDR:%d  CGV:%d\n",
		ssiclk,div, hz,ssicdr,cgv);

	__ssi_disable(0);
	__ssi_disable(1);
	__cpm_set_ssidiv(ssicdr);
	__ssi_set_grdiv(hw->chnl,cgv);

	return 0;
}

/* every spi_transfer could call this routine to setup itself */
static int jz47xx_spi_setupxfer(struct spi_device *spi,struct spi_transfer *t)
{
	struct jz47xx_spi *hw = to_hw(spi);
	u8 uintbits,bpw;
	u32 hz;
	
	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if(t){
		if(!bpw)
			bpw = spi->bits_per_word;
		if(!hz) 
			hz= spi->max_speed_hz;
	}
	
	if (bpw < 2 || bpw > 17) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	hw->bits_per_word = bpw;
	if(bpw <= 8 ){
		hw->transfer_unit_size = FIFO_8_BIT;
		hw->get_rx = jz47xx_spi_rx_buf_u8;
		hw->get_tx = jz47xx_spi_tx_buf_u8;
	}else if(bpw <= 16){
		hw->transfer_unit_size = FIFO_16_BIT;
		hw->get_rx = jz47xx_spi_rx_buf_u16;
		hw->get_tx = jz47xx_spi_tx_buf_u16;
	}
	uintbits = hw->transfer_unit_size*8;
	
	__ssi_set_frame_length(hw->chnl,uintbits);

//	__ssi_set_frame_length(hw->chnl,bpw);

	jz_spi_set_clk(spi,hz);

	
	dev_dbg(&spi->dev,"The real SPI CLK is %d Hz\n",jz_spi_get_clk(spi));
	
//	__ssi_set_clk(pdev->id,__cpm_get_extalclk(),hz);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}
/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_LOOP)

static int jz47xx_spi_setup(struct spi_device *spi)
{
	int ret;
	
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	
	if (spi->mode & ~MODEBITS) {
		dev_info(&spi->dev, "Warning: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}
	if (!spi->max_speed_hz)
		return -EINVAL;
	
	ret = jz47xx_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "Warning:setupxfer returned %d\n", ret);
		return ret;
	}

	return 0;
}

static inline int cpu_write_txfifo(struct jz47xx_spi *hw,unsigned int entries)
{
	unsigned int i;
	u16 dat;
		
	if((!entries )|| (!(hw->rw_mode & RW_MODE)))
	{
		return -1;
	}else if(hw->rw_mode & W_MODE)
	{                           
		for(i=0;i<entries;i++)
		{
			dat = hw->get_tx(hw);
			__ssi_transmit_data(hw->chnl,dat);
			print_dbg("0x%x,",dat);
		}	
	}else
	{		 /* read, fill txfifo with 0 */
		
		for(i=0;i<entries;i++)
			__ssi_transmit_data(hw->chnl,0);
		print_dbg("0x0...,");
	}
	print_kdbg("+:%d ",entries);

	return 0;	
}
static inline int spi_start_dma(struct jz47xx_spi *hw,unsigned int count, int mode)
{
	unsigned char dma_unit,src_bit,dest_bit;
	unsigned char bpw = hw->transfer_unit_size * 8;
	int chan;
	unsigned int phyaddr;
	unsigned long flags;

	mode &= DMA_MODE_MASK;
	if( mode == DMA_MODE_READ ){	/* dma read rxfifo */
		chan = hw->dma_rx_chnl;
		src_bit  = bpw;
		dest_bit = 32;
		phyaddr  = hw->rx_dma;	
		dma_unit = hw->dma_tx_unit;
	}else if( mode == DMA_MODE_WRITE ){  /* dma write txfifo */
		chan = hw->dma_tx_chnl;
		src_bit  = 32;
		dest_bit = bpw;
		phyaddr  = hw->tx_dma;
		dma_unit = hw->dma_rx_unit;
	}else{
		dev_err(hw->dev,"SPI Start DMA Fail(Mode Error) !!!\n");
		return -SPI_DMA_ERROR;
	}

	flags = claim_dma_lock();
	disable_dma(chan);
	clear_dma_ff(chan);	
	jz_set_dma_src_width(chan, src_bit);
	jz_set_dma_dest_width(chan, dest_bit);

	jz_set_dma_block_size(chan, dma_unit);	 /* n byte burst */
	set_dma_mode(chan, mode);
	set_dma_addr(chan, phyaddr);
	set_dma_count(chan, count + dma_unit -1); /* ensure dma count align */
	enable_dma(chan);
	release_dma_lock(flags);

	return 0;
}

static inline int spi_dma_setup(struct jz47xx_spi *hw,unsigned int len)
{
	int status = 0;

	if(hw->rx_trigger < hw->dma_rx_unit)
		print_dbg("DMA  probable get invalid datas from SSI RxFIFO\n");
	
	if(hw->rw_mode & W_DMA){
		
		/* Start Rx to read if hw->rx_dma is not NULL */
		if(hw->rx_dma)
			status = spi_start_dma(hw,len,DMA_MODE_READ);

		/* Start Tx for dummy write */
		status = spi_start_dma(hw,len,DMA_MODE_WRITE);
		if(status < 0)
			return status;
		print_kdbg("DMA Read and Write\n");
		
	}else{
		if(!hw->tx_dma)
			hw->tx_dma = hw->rx_dma;

		/* Start Rx to read */
		status = spi_start_dma(hw,len,DMA_MODE_READ);

		/* Start Tx for dummy write */
		status = spi_start_dma(hw,len,DMA_MODE_WRITE);

		if(status < 0)
			return status;
		
		print_kdbg("DMA Read sent\n");
	}
	
	return status;
}

/* Fill SSI TxFIFO according to the data count,and decide whether DMA is used */
static inline int fill_txfifo(struct jz47xx_spi *hw)
{
	unsigned char int_flag = 0,last_flag = 0;
	unsigned int leave_len_bytes,unit_size,trigger,send_entries,entries=0;
	int status = 0;
	hw->use_dma = 0;

	leave_len_bytes = hw->len - hw->count;
	/* calculate the left entries */
	unit_size = hw->transfer_unit_size;
	if( unit_size == FIFO_8_BIT )
		entries = leave_len_bytes;
	else if(unit_size == FIFO_16_BIT )
	{
		entries = (leave_len_bytes + 1)/2;
	}else{
		dev_err(hw->dev,"transfer_unit_size error!\n");
		return -1;
	}
	
	if(leave_len_bytes == 0)								/* --- End or ??? --- */
	{
		int_flag = 0;
		print_dbg("leave_len_bytes = 0\n");
		
	}else if(hw->rw_mode & R_DMA ||hw->rw_mode & W_DMA) 	/* --- DMA transfer --- */
	{													
		/* change SSI trigger for DMA transfer */
		/* Important!!! it probable die in waitting for DMA_RX_END intr
			if configure improper. */
		hw->rx_trigger = hw->dma_rx_unit / unit_size;
		
		if(hw->rx_trigger < hw->dma_rx_unit)
			print_dbg("DMA probable get invalid datas from SSI RxFIFO\n");
		
		__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);
		
		status = spi_dma_setup(hw,leave_len_bytes);
		if(status < 0)
			return status;

		hw->use_dma = 1;
	
		hw->count += leave_len_bytes;
		
	}else{			 									 /* --- CPU transfer --- */	

		trigger = JZ_SSI_MAX_FIFO_ENTRIES - hw->tx_trigger;
		
		/* calculate the entries which will be sent currently  */
		if(hw->is_first){  /* distinguish between the first and interrupt */

			/* CPU Mode should reset SSI triggers at first */
			hw->tx_trigger = SSI_TX_FIFO_THRESHOLD*8;
			hw->rx_trigger = SSI_RX_FIFO_THRESHOLD*8;
			__ssi_set_tx_trigger(hw->chnl,hw->tx_trigger);
			__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);
			
			if(entries <= JZ_SSI_MAX_FIFO_ENTRIES)	
				send_entries = entries;
			else{ 			
			/* need enable half_intr, left entries will be sent in SSI interrupt 
				and receive the datas*/
				send_entries = JZ_SSI_MAX_FIFO_ENTRIES;
				int_flag = 1;
			}
		}else{	/* happen in interrupts */
			if(entries <= trigger){
				send_entries = entries;
				last_flag = 1;	/* the last part of data shouldn't disable RXI_intr at once !!! */
			}
			else{			
			/* need enable half_intr, left entries will be sent in SSI interrupt 
			and receive the datas*/
				send_entries = CPU_ONCE_BLOCK_ENTRIES;
				int_flag = 1;
			}
		}
		/* fill the txfifo with CPU Mode */
		cpu_write_txfifo(hw,send_entries);
		hw->count += (send_entries*unit_size);
	}
	
	/* every time should control the SSI half_intrs */
	if(int_flag)
	{	
		__ssi_enable_txfifo_half_empty_intr(hw->chnl);
		__ssi_enable_rxfifo_half_full_intr(hw->chnl);
	}else
	{
		__ssi_disable_txfifo_half_empty_intr(hw->chnl);
		__ssi_disable_rxfifo_half_full_intr(hw->chnl);
	}
	
	/* to avoid RxFIFO overflow when CPU Mode at last time to fill */
	if(last_flag)
	{				
		last_flag = 0;
		__ssi_enable_rxfifo_half_full_intr(hw->chnl);
	}

	print_dbg("-----SSI%d half_int_flag= %d ,use_dma= %d\n",
		hw->chnl,int_flag,hw->use_dma);
	
	return status;
}
void print_ssi_regs(u8 n)
{
	print_msg("\nSSI%d\n",n);
//	print_msg("REG_SSI_DR ========0x%x  \n",REG_SSI_DR(n));
	print_msg("REG_SSI_CR0========0x%x  \n",REG_SSI_CR0(n));
	print_msg("REG_SSI_CR1========0x%x  \n",REG_SSI_CR1(n));
	print_msg("REG_SSI_SR ========0x%x  \n",REG_SSI_SR(n));
	print_msg("REG_SSI_ITR========0x%x  \n",REG_SSI_ITR(n));
	print_msg("REG_SSI_ICR========0x%x  \n",REG_SSI_ICR(n));
	print_msg("REG_SSI_GR ========0x%x  \n",REG_SSI_GR(n));

}

void save_ssi_regs(u32 *regs,u8 n)
{
	regs[0]=REG_SSI_CR0(n);
	regs[1]=REG_SSI_CR1(n);
	regs[2]=REG_SSI_SR(n);
	regs[3]=REG_SSI_ITR(n);
	regs[4]=REG_SSI_ICR(n);
	regs[5]=REG_SSI_GR(n);

//	return &regs[0];
}

void show_ssi_regs(u32 *regs)
{
	
//	print_msg("REG_SSI_DR ========0x%x  \n",REG_SSI_DR(n));
	print_msg("\nREG_SSI_CR0=====0x%x  \n",regs[0]);
	print_msg("REG_SSI_CR1=====0x%x  \n",regs[1]);
	print_msg("REG_SSI_SR =====0x%x  \n",regs[2]);
	print_msg("REG_SSI_ITR=====0x%x  \n",regs[3]);
	print_msg("REG_SSI_ICR=====0x%x  \n",regs[4]);
	print_msg("REG_SSI_GR =====0x%x  \n",regs[5]);
}

static int jz47xx_spi_txrx(struct spi_device * spi, struct spi_transfer *t)
{
	int status;
	struct jz47xx_spi * hw = to_hw(spi);
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;

	
	print_dbg("in %s\n",__FUNCTION__);
	
	print_kdbg("mname: %s  txrx: tx %p, rx %p, len %d ,tx_dma 0x%08X ,rx_dma 0x%08X\n",
		spi->modalias,t->tx_buf, t->rx_buf, t->len, t->tx_dma ,t->rx_dma);	

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->tx_dma = t->tx_dma;
	hw->rx_dma = t->rx_dma;
	hw->len = t->len;
	hw->count = 0;
	hw->rlen = 0;

//	dma_cache_wback_inv((unsigned long)t->tx_dma,t->len);
//	dma_cache_wback_inv((unsigned long)t->rx_dma,t->len);
	
	if( REG_SSI_CR0(hw->chnl) & (1<<10) )
		print_dbg("Loop Mode\n");
	else
		print_dbg("Normal Mode\n");
	

	hw->rw_mode = 0;
	if(hw->tx)
		hw->rw_mode |= W_MODE; 
	if(hw->rx)
		hw->rw_mode |= R_MODE; 

	if(hw->tx_dma)
		hw->rw_mode |= W_DMA;
	if(hw->rx_dma)
		hw->rw_mode |= R_DMA;

	print_kdbg("hw->rw_mode = 0x%X\n",hw->rw_mode);
	
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);

	__ssi_wait_transmit(hw->chnl);
	/* flush TX FIFO and fill FIFO */
	__ssi_flush_fifo(hw->chnl);
	
	__ssi_enable_receive(hw->chnl);
	__ssi_clear_errors(hw->chnl);
	
	memset(g_jz_intr, 0, sizeof *g_jz_intr);
	
	hw->is_first = 1;
	status = fill_txfifo(hw);

	if(status == -SPI_DMA_ERROR){
			hw->rw_mode &= ~RW_DMA;
			dev_info(hw->dev,"Try CPU mode instead!\n");
			status = fill_txfifo(hw);
	}
	if(status < 0)
	{
		__ssi_disable_tx_intr(hw->chnl);
		__ssi_disable_tx_intr(hw->chnl);
		dev_err(hw->dev,"CPU mode fail!\n");
		return -1;
	}
	
	if(hw->use_dma){
		__ssi_disable_tx_error_intr(hw->chnl);
		__ssi_disable_rx_error_intr(hw->chnl);

	}else{
		__ssi_enable_tx_error_intr(hw->chnl);
		__ssi_enable_rx_error_intr(hw->chnl);
	}
	hw->is_first = 0;
	
	/* start SSI transfer, and start DMA transfer when DMA Mode */
	__ssi_enable(hw->chnl);

	/* wait the interrupt finish the transfer( one spi_transfer be sent ) */
	wait_for_completion(&hw->done);
	
	__ssi_finish_transmit(hw->chnl);
	__ssi_clear_errors(hw->chnl);

	/* ------- for debug --------- */
	print_msg("\ninterrupt Enable:\nTIE:%d RIE:%d \nTEIE:%d REIE:%d\n",
		(regs[0]&1<<14)>>14,(regs[0]&1<<13)>>13,(regs[0]&1<<12)>>12,(regs[0]&1<<11)>>11);
	print_msg("\nSSI interrupt cnts = %d\n",g_jz_intr->ssi_intr_cnt);
	
	print_msg("TXI:%d  RXI:%d\nunderrun:%d  overrun:%d\n\n",
		g_jz_intr->ssi_txi,g_jz_intr->ssi_rxi,g_jz_intr->ssi_eti,g_jz_intr->ssi_eri);

	print_msg("DMA TX interrupt cnts = %d\nDMA RX interrupt cnts = %d\n",
		g_jz_intr->dma_tx_cnt,g_jz_intr->dma_rx_cnt);

	print_msg("dma_tx_err:%d  dma_tx_end:%d\ndma_rx_err:%d  dma_rx_end:%d\n\n",
		g_jz_intr->dma_tx_err,g_jz_intr->dma_tx_end,g_jz_intr->dma_rx_err,g_jz_intr->dma_rx_end);
	/* ---------------------------- */

	if(hw->rlen > hw->len)
		hw->rlen = hw->len;

	return hw->rlen;
}

static irqreturn_t jz47xx_spi_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	int left_count= hw->len - hw->count;
	u8 flag = 0,fs;
	u16 dat;
	u32 cnt,unit_size = hw->transfer_unit_size;
	int status;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;

	g_jz_intr->ssi_intr_cnt++;
	/* to avoid die in interrupt if some error occur */
	if(g_jz_intr->ssi_intr_cnt >MAX_SSI_INTR)
	{
		__ssi_disable_tx_intr(hw->chnl);
		__ssi_disable_rx_intr(hw->chnl);
		dev_err(hw->dev,"\nERROR:SSI interrupts too many count(%d)!\n",
			g_jz_intr->ssi_intr_cnt);
		complete(&hw->done);

		goto irq_done;
	}
	
	if( __ssi_underrun(hw->chnl) &&
		__ssi_tx_error_intr(hw->chnl)){
		print_kdbg("UNDR:");
		g_jz_intr->ssi_eti++;
		__ssi_disable_tx_error_intr(hw->chnl);

		cnt = g_jz_intr->ssi_rlen;
		if((hw->rw_mode & RW_MODE) == W_MODE){
			print_dbg("W_MODE\n");
			hw->rlen += unit_size*__ssi_get_rxfifo_count(hw->chnl);
			__ssi_flush_rxfifo(hw->chnl);
		}else{		
			while(!__ssi_rxfifo_empty(hw->chnl))
			{
				dat = __ssi_receive_data(hw->chnl);
				hw->get_rx(dat,hw);
				hw->rlen += unit_size;
				
				print_dbg("-%x,",dat);
				g_jz_intr->ssi_rlen++;
			}
		}
		print_kdbg("-:%d\n",g_jz_intr->ssi_rlen - cnt);
		if( left_count == 0){
			__ssi_disable_tx_intr(hw->chnl);
			__ssi_disable_rx_intr(hw->chnl);

			complete(&hw->done);
		}else
			__ssi_enable_tx_error_intr(hw->chnl);
		
		flag++;

	}
	
	if ( __ssi_txfifo_half_empty_intr(hw->chnl) &&
		 __ssi_txfifo_half_empty(hw->chnl)) {

		print_kdbg("TXI:");
		g_jz_intr->ssi_txi++;

		status = fill_txfifo(hw);
		if(status < 0)
		{
			__ssi_disable(hw->chnl);
			__ssi_disable_tx_intr(hw->chnl);
			__ssi_disable_rx_intr(hw->chnl);	
			dev_err(hw->dev,"data filling error!\n");
			complete(&hw->done);
			
			goto irq_done;
		}
		
		fs = REG_SSI_SR(hw->chnl);
		print_dbg("@VER-%d@UNDR-%d@RFHF-%d@TFHE-%d@\n",(fs>>0)&0x1,(fs>>1)&0x1,(fs>>2)&0x1,(fs>>3)&0x1);
		
		flag++;
	}

	if ( __ssi_rxfifo_half_full(hw->chnl) &&
		__ssi_rxfifo_half_full_intr(hw->chnl)) {
		__ssi_disable_rxfifo_half_full_intr(hw->chnl);
		print_kdbg("RXI:");
		g_jz_intr->ssi_rxi++;
		
		cnt = g_jz_intr->ssi_rlen;
		if((hw->rw_mode & RW_MODE) == W_MODE){
			print_dbg("W_MODE\n");
			hw->rlen += unit_size*__ssi_get_rxfifo_count(hw->chnl);
			__ssi_flush_rxfifo(hw->chnl);
		}else{
			
			while(!__ssi_rxfifo_empty(hw->chnl))
			{
				dat = __ssi_receive_data(hw->chnl);
				hw->get_rx(dat,hw);
				hw->rlen += unit_size;
				
				print_dbg("-%x,",dat);
				g_jz_intr->ssi_rlen++;
			}
		}
		print_kdbg("-:%d ",g_jz_intr->ssi_rlen - cnt);
		
		flag++;
	}

	if( __ssi_overrun(hw->chnl) &&
		__ssi_rx_error_intr(hw->chnl)){
		print_kdbg(" overrun:");
		g_jz_intr->ssi_eri++;

		__ssi_disable(hw->chnl);

		cnt = g_jz_intr->ssi_rlen;
		while(!__ssi_rxfifo_empty(hw->chnl))
		{
			dat = __ssi_receive_data(hw->chnl);
			hw->get_rx(dat,hw);
			hw->rlen += unit_size;
			
			print_dbg("-%x,",dat);
			g_jz_intr->ssi_rlen++;
		}
		print_kdbg("-:%d ",g_jz_intr->ssi_rlen - cnt);
		__ssi_enable(hw->chnl);
			
		flag++;
	}

	if(!flag)
	{
		dev_info(hw->dev,"\nERROR:interrupt Type error\n");
		complete(&hw->done);
	}
	
 irq_done:
 	__ssi_clear_errors(hw->chnl);
	return IRQ_HANDLED;
}
static irqreturn_t spi_dma_tx_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;
	int chan = hw->dma_tx_chnl;

	g_jz_intr->dma_tx_cnt++;
	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		__dmac_channel_clear_address_error(chan);
		g_jz_intr->dma_tx_err++;
		
		print_kdbg("DMA addr error\n");
		complete(&hw->done);
		
		return IRQ_HANDLED;
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		__dmac_channel_clear_transmit_end(chan);
		__ssi_disable_tx_error_intr(hw->chnl); /* disable underrun irq here ??? */
		g_jz_intr->dma_tx_end++;

		print_kdbg("DMA Write End\n");
	//	while(__ssi_get_txfifo_count(hw->chnl));

		return IRQ_HANDLED;
	}
	
	print_kdbg("DMA others tx int error\n");
	
	return IRQ_HANDLED;
}
static irqreturn_t spi_dma_rx_irq(int irq, void *dev)
{
	struct jz47xx_spi *hw = dev;
	struct jz_intr_cnt *g_jz_intr = hw->g_jz_intr;
	int chan = hw->dma_rx_chnl;
	u16 dat;
	
	g_jz_intr->dma_rx_cnt++;
	disable_dma(chan);
	if (__dmac_channel_address_error_detected(chan)) {
		printk(KERN_DEBUG "%s: DMAC address error.\n", __FUNCTION__);
		g_jz_intr->dma_rx_err++;
		__dmac_channel_clear_address_error(chan);
	//	complete(&hw->done);
		goto dma_rx_irq_done;
	}
	if (__dmac_channel_transmit_end_detected(chan)) {
		g_jz_intr->dma_rx_end++;
		__dmac_channel_clear_transmit_end(chan);

		while(!__ssi_rxfifo_empty(hw->chnl))
		{
			dat = __ssi_receive_data(hw->chnl);
			hw->get_rx(dat,hw);
			
			print_kdbg("-%x,",dat);
			g_jz_intr->ssi_rlen++;
		}
		hw->rlen += hw->count;
		print_kdbg("DMA Read End\n");
		
		goto dma_rx_irq_done;
	}
	print_kdbg("DMA others rx int error\n");
dma_rx_irq_done:
	complete(&hw->done);
	return IRQ_HANDLED;
}
static int jz_spi_dma_init(struct jz47xx_spi *hw)
{
	
	if ((hw->dma_tx_chnl= jz_request_dma(DMA_ID_SSI0_TX, "SSI0 Tx DMA", 
		spi_dma_tx_irq,IRQ_DISABLED, hw)) < 0 ) {

		printk(KERN_ERR "SSI0 Tx DMA request failed!\n");
		return -EINVAL;
	}
	if ((hw->dma_rx_chnl = jz_request_dma(DMA_ID_SSI0_RX, "SSI0 Rx DMA", 
		spi_dma_rx_irq,IRQ_DISABLED, hw)) < 0 ) {

		printk(KERN_ERR "SSI0 Rx DMA request failed!\n");
		return -EINVAL;
	}
	hw->dma_tx_unit = JZ_SSI_DMA_BURST_LENGTH;
	hw->dma_rx_unit = JZ_SSI_DMA_BURST_LENGTH;

	return 0;
}
int jz_spi_pinconfig(struct jz47xx_spi *hw)
{
	u8 f_gpiocs=0,f_spics0=0,f_spics1=0;
	int i;
	struct jz47xx_spi_info *pdata = hw->pdata;

	if(pdata->board_size > MAX_SPI_DEVICES)
	{
		pdata->board_size = MAX_SPI_DEVICES;
		dev_info(hw->dev,"SPI devices exceed defined max_num!!!\n");
	}
	
	for(i=0; i< pdata->board_size; i++)
	{
		if(pdata->pin_cs[i] == PIN_SSI_CE0){ 
			if(!hw->chnl)
				pdata->pin_cs[i] = SSI0_CE0_PIN;
			else
				pdata->pin_cs[i] = SSI1_CE0_PIN;
			f_spics0 = 1;
		}else if(pdata->pin_cs[i] == PIN_SSI_CE1){ 	
			if(!hw->chnl)
				pdata->pin_cs[i] = SSI0_CE1_PIN;
			else
				pdata->pin_cs[i] = SSI1_CE1_PIN;
			f_spics1 = 1;
		}else									
			f_gpiocs = 1;

		__gpio_as_output(pdata->pin_cs[i]);
		
		print_msg("PIN:0x%04X\n",pdata->pin_cs[i]);
	}
	print_msg("f_spics0=%d\nf_spics1=%d\nf_gpiocs=%d\n",f_spics0,f_spics1,f_gpiocs);

	
	if(pdata->pins_config){			/* if user want to configure by himself */
		pdata->pins_config();
		return 0;
	}
	
	if(f_spics0|| f_spics1){		/* spi chipselect for ssi controller internal */

		/* one of two controllers in SOC 
		 *
		 * ??? pin_output function  instead of controller internal_chipselect, because ...
		 *
		 */
		if(f_spics1)
			__ssi_select_ce2(hw->chnl);
		if(f_spics0)
			__ssi_select_ce(hw->chnl);
		
		GPIO_AS_SSI(hw->chnl);

	}
	
	if(f_gpiocs){					/* config SPI_PINs for spi function except for CE0 and CE1 
	 *
	 * SPI_PINs: SSI0_CLK, SSI0_DT, SSI0_DR
	 */
		
		GPIO_AS_SSI_EX(hw->chnl);
	}
	
	
	return 0;
}
EXPORT_SYMBOL_GPL(jz_spi_pinconfig);
	
static int jz47xx_spi_init_setup(struct jz47xx_spi *hw)
{
		
	/* for the moment,open the SSI clock gate */

	/* disable the SSI controller */
	__ssi_disable(hw->chnl);
	
	CPM_SSI_START(hw->chnl);
	
	/* set default half_intr trigger */
	hw->tx_trigger = SSI_TX_FIFO_THRESHOLD*8;
	hw->rx_trigger = SSI_RX_FIFO_THRESHOLD*8;
	__ssi_set_tx_trigger(hw->chnl,hw->tx_trigger);
	__ssi_set_rx_trigger(hw->chnl,hw->rx_trigger);

	/* First,mask the interrupt, while verify the status ? */
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);

	__ssi_disable_receive(hw->chnl);

	__ssi_set_spi_clock_phase(hw->chnl,0);
	__ssi_set_spi_clock_polarity(hw->chnl,0);  
	__ssi_set_msb(hw->chnl);
	__ssi_spi_format(hw->chnl);
	__ssi_set_frame_length(hw->chnl,8);
	__ssi_disable_loopback(hw->chnl);   
	__ssi_flush_fifo(hw->chnl);

	__ssi_underrun_auto_clear(hw->chnl);
 	__ssi_clear_errors(hw->chnl);
		
	return 0;
}


static int jz_ssi_clk_setup(struct jz47xx_spi *hw)
{
#ifdef CONFIG_SOC_JZ4760

	if(hw->pdata->is_pllclk){
		__ssi_select_pllclk();
	}else{
		__ssi_select_exclk();
	}
	__cpm_set_ssidiv(0);
	hw->src_clk = cpm_get_clock(CGU_SSICLK);
#else
	hw->src_clk = __cpm_get_pllout();
	hw->pdata->is_pllclk = 1;
#endif
	
	return 0;
}

static int __init jz47xx_spi_probe(struct platform_device *pdev)
{
	struct jz47xx_spi *hw;
	struct spi_master *master;
	
	struct resource *res;
	int err = 0;
	
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
	int i;
	struct spi_board_info *bi;
#endif

	print_msg("in %s\n",__FUNCTION__);
	master = spi_alloc_master(&pdev->dev, sizeof(struct jz47xx_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	
	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct jz47xx_spi));

	hw->g_jz_intr = kzalloc(sizeof(struct jz_intr_cnt),GFP_KERNEL);

	if(hw->g_jz_intr == NULL)
	{
		dev_err(&pdev->dev, "No memory for jz_intr_cnt\n");
		err = -ENOMEM;
		goto err_nomem;
	}
	
	hw->master = spi_master_get(master);
	hw->dev = &pdev->dev;
	
	hw->pdata = pdev->dev.platform_data;
	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}
	hw->chnl= hw->pdata->chnl;
	
	if(hw->chnl != 0 && hw->chnl != 1){
		dev_err(&pdev->dev, "No this channel\n");
		err = -ENOENT;
		goto err_no_pdata;
	}
	
	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);

	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = jz47xx_spi_setupxfer;
	hw->bitbang.chipselect     = jz47xx_spi_chipsel;
	hw->bitbang.txrx_bufs      = jz47xx_spi_txrx;
	hw->bitbang.master->setup  = jz47xx_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}
	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}
	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}
	
	/* request DMA irq */
	err =jz_spi_dma_init(hw);
	if(err)
		goto err_no_irq;
	
	/* request SSI irq */
	err = request_irq(hw->irq, jz47xx_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}
	
	/* get controller associated params */
	master->bus_num = hw->pdata->bus_num;
	master->num_chipselect = hw->pdata->board_size;


	/* setup chipselect */
	if (hw->pdata->set_cs)
		hw->set_cs = hw->pdata->set_cs; 
	else
		hw->set_cs = jz47xx_spi_cs;

	/* SPI_PINs and chipselect configure */
	jz_spi_pinconfig(hw);
	/* SSI controller clock configure */
	jz_ssi_clk_setup(hw);
	/* SSI controller initializations for SPI */
	jz47xx_spi_init_setup(hw);
	
	/* register our spi controller */
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master ERR_NO:%d\n",err);
		goto err_register;
	}

#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER	
	/* register all the devices associated */
	bi = &hw->pdata->board_info[0];
	if(bi){
		for (i = 0; i < hw->pdata->board_size; i++, bi++) {
			dev_info(hw->dev, "registering %s\n", bi->modalias);

			bi->controller_data = hw;
			spi_new_device(master, bi);
		}
	}
#endif

	printk(KERN_INFO
	       "JZ47xx SSI Controller for SPI channel %d driver\n",hw->chnl);
	
	return 0;

 err_register:
	CPM_SSI_STOP(hw->chnl);

	free_irq(hw->irq, hw);

 err_no_irq:
	iounmap(hw->regs);

 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}

static int __exit jz47xx_spi_remove(struct platform_device *dev)
{
	struct jz47xx_spi *hw = platform_get_drvdata(dev);

	CPM_SSI_STOP(hw->chnl);
	__ssi_disable_tx_intr(hw->chnl);
	__ssi_disable_rx_intr(hw->chnl);
	__ssi_disable(hw->chnl);
	
	platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	

	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);

	/* release DMA channel */
	if (hw->dma_rx_chnl >= 0) {
		jz_free_dma(hw->dma_rx_chnl);
		printk("dma_rx_chnl release\n");
	}
	if (hw->dma_tx_chnl >= 0) {
		jz_free_dma(hw->dma_tx_chnl);
		printk("dma_tx_chnl release\n");
	}

	kfree(hw->g_jz_intr);
	kfree(hw);
	
	return 0;
}
#ifdef CONFIG_PM

static int jz47xx_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct jz47xx_spi *hw = platform_get_drvdata(pdev);

	CPM_SSI_STOP(hw->chnl);

	return 0;
}

static int jz47xx_spi_resume(struct platform_device *pdev)
{
	struct jz47xx_spi *hw = platform_get_drvdata(pdev);

	CPM_SSI_START(hw->chnl);

	return 0;
}

#else
#define jz47xx_spi_suspend NULL
#define jz47xx_spi_resume  NULL
#endif

MODULE_ALIAS("jz47xx_spi");			/* for platform bus hotplug */
static struct platform_driver jz47xx_spidrv = {
	.remove		= __exit_p(jz47xx_spi_remove),
	.suspend	= jz47xx_spi_suspend,
	.resume		= jz47xx_spi_resume,
	.driver		= {
		.name	= "jz47xx-spi0",
		.owner	= THIS_MODULE,
	},
};
static int __init jz47xx_spi_init(void)
{
        return platform_driver_probe(&jz47xx_spidrv, jz47xx_spi_probe);
}

static void __exit jz47xx_spi_exit(void)
{
        platform_driver_unregister(&jz47xx_spidrv);

}
module_init(jz47xx_spi_init);
module_exit(jz47xx_spi_exit);

MODULE_DESCRIPTION("JZ47XX SPI Driver");
MODULE_LICENSE("GPL");

