/*
 *  linux/drivers/net/jz4760_eth.c
 *
 *  Jz4760 On-Chip ethernet driver.
 *
 *  Copyright (C) 2005 - 2007  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/kthread.h>
#include <linux/version.h>

#include <asm/io.h>
#include <asm/addrspace.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/cacheops.h>
#include <asm/jzsoc.h>

#include "jz4760_eth.h"

#define P2ADDR(a)	(((unsigned long)(a) & 0x1fffffff) | 0xa0000000)
#define P1ADDR(a)	(((unsigned long)(a) & 0x1fffffff) | 0x80000000)

#if 0
#define DUMP_RX_BUF_ADDR_MAX	256
#define DUMP_TX_BUF_ADDR_MAX	256

static unsigned int rx_buf_addr_array[DUMP_RX_BUF_ADDR_MAX] = {0};
static int rx_buf_addr_array_idx = 0;
static unsigned int tx_buf_addr_array[DUMP_TX_BUF_ADDR_MAX] = {0};
static int tx_buf_addr_array_idx = 0;

static void dump_eth_rx_addr()
{
	int i;
	for (i = 0; i < rx_buf_addr_array_idx; i++) {
		printk("rx_buf_addr_array[%d] = 0x%08x\n", i, rx_buf_addr_array[i]);
	}
}
static void dump_eth_tx_addr()
{
	int i;
	for (i = 0; i < tx_buf_addr_array_idx; i++) {
		printk("tx_buf_addr_array[%d] = 0x%08x\n", i, tx_buf_addr_array[i]);
	}
}
#endif

//#define USE_RMII	1
//#define ETH_DEBUG
#ifdef ETH_DEBUG
#define DBPRINTK(fmt,args...) printk(KERN_DEBUG fmt,##args)
#else
#define DBPRINTK(fmt,args...) do {} while(0)
#endif

#define errprintk(fmt,args...)  printk(KERN_ERR fmt,##args);
#define infoprintk(fmt,args...) printk(KERN_INFO fmt,##args);

#define DRV_NAME	"jz4760_eth"
#define DRV_VERSION	"0.1"
#define DRV_AUTHOR	"Jason Wang <xwang@ingenic.cn>"
#define DRV_DESC	"JzSOC 4760 On-chip Ethernet driver"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(debug, "i");
MODULE_PARM_DESC(hwaddr,"s");

/*
 * Local variables
 */
static struct net_device *netdev;
static char * hwaddr = NULL;
static int debug = -1;
static struct mii_if_info mii_info;

/*
 * Local routines
 */
static irqreturn_t jz_eth_interrupt(int irq, void *dev_id);
static int link_check_thread (void *data); 

#if 0
/*
 * Get MAC address
 */

#define I2C_DEVICE  0x57
#define MAC_OFFSET  64

extern void i2c_open(void);
extern void i2c_close(void);
extern int i2c_read(unsigned char device, unsigned char *buf,
		    unsigned char address, int count);
#endif

static inline unsigned char str2hexnum(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		unsigned char num;

		if((*str == '.') || (*str == ':'))
			str++;
		num = str2hexnum(*str++) << 4;
		num |= (str2hexnum(*str++));
		ea[i] = num;
	}
}

static int ethaddr_cmd = 0;
static unsigned char ethaddr_hex[6];

static int __init ethernet_addr_setup(char *str)
{
	if (!str) {
	        printk("ethaddr not set in command line\n");
		return -1;
	}
	ethaddr_cmd = 1;
	str2eaddr(ethaddr_hex, str);

	return 0;
}

__setup("ethaddr=", ethernet_addr_setup);

static int get_mac_address(struct net_device *dev)
{
	int i;
	unsigned char flag0=0;
	unsigned char flag1=0xff;
	
	dev->dev_addr[0] = 0xff;
	if (hwaddr != NULL) {
		/* insmod jz-ethc.o hwaddr=00:ef:a3:c1:00:10 */
		str2eaddr(dev->dev_addr, hwaddr);
	} else if (ethaddr_cmd) {
		/* linux command line: ethaddr=00:ef:a3:c1:00:10 */
		for (i=0; i<6; i++)
			dev->dev_addr[i] = ethaddr_hex[i];
	} else {
#if 0
		/* mac address in eeprom:  byte 0x40-0x45 */
		i2c_open();
		i2c_read(I2C_DEVICE, dev->dev_addr, MAC_OFFSET, 6);
		i2c_close();
#endif
	}

	/* check whether valid MAC address */
	for (i=0; i<6; i++) {
		flag0 |= dev->dev_addr[i];
		flag1 &= dev->dev_addr[i];
	}
	if ((dev->dev_addr[0] & 0xC0) || (flag0 == 0) || (flag1 == 0xff)) {
		printk("WARNING: There is not MAC address, use default ..\n");
		dev->dev_addr[0] = 0x00;
		dev->dev_addr[1] = 0xef;
		dev->dev_addr[2] = 0xa3;
		dev->dev_addr[3] = 0xc1;
		dev->dev_addr[4] = 0x00;
		dev->dev_addr[5] = 0x10;
		dev->dev_addr[5] = 0x03;
	}

	return 0;
}

/*---------------------------------------------------------------------*/

static u32 jz_eth_curr_mode(struct net_device *dev);

/*
 * Link check routines
 */
static void start_check(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif
	struct task_struct *t;

	np->thread_die = 0;
	init_waitqueue_head(&np->thr_wait);
	init_completion(&np->thr_exited);

	t = kthread_create(link_check_thread,(void *)dev, dev->name);
	if (IS_ERR(t))
		errprintk("%s: Unable to start kernel thread\n",dev->name);
	np->thread = t;
}

static int close_check(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif
	int ret = 0;

	if (np->thread != NULL) {
		np->thread_die = 1;
		wmb();
		send_sig(SIGTERM, np->thread, 1);
		if (ret) {
			errprintk("%s: Unable to signal thread\n", dev->name);
			return 1;
		}
		wait_for_completion (&np->thr_exited);
	}
	return 0;
}

static int link_check_thread(void *data)
{
	struct net_device *dev=(struct net_device *)data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)netdev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unsigned char current_link;
	unsigned long timeout;

	daemonize("%s", dev->name);
	spin_lock_irq(&current->sighand->siglock);
	sigemptyset(&current->blocked);
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);

	strncpy(current->comm, dev->name, sizeof(current->comm) - 1);
	current->comm[sizeof(current->comm) - 1] = '\0';

	while (1) {
		timeout = 3*HZ;
		do {
			timeout = interruptible_sleep_on_timeout(&np->thr_wait, timeout);
			/* make swsusp happy with our thread */
//			if (current->flags & PF_FREEZE)
//				refrigerator(PF_FREEZE);
		} while (!signal_pending(current) && (timeout > 0));

		if (signal_pending (current)) {
			spin_lock_irq(&current->sighand->siglock);
			flush_signals(current);
			spin_unlock_irq(&current->sighand->siglock);
		}

		if (np->thread_die)
			break;
		
		current_link = mii_link_ok(&mii_info);
		if (np->link_state != current_link) {
			if (current_link) {
				infoprintk("%s: Ethernet Link OK!\n", dev->name);
				jz_eth_curr_mode(dev);
				netif_carrier_on(dev);
			} else {
				errprintk("%s: Ethernet Link offline!\n", dev->name);
				netif_carrier_off(dev);
			}
		}
		np->link_state = current_link;

	}
	complete_and_exit(&np->thr_exited, 0);	
}

static void skb_dump(struct sk_buff *skb)
{
	printk("skb ----------- [ 0x%08x ]\n", (unsigned int)skb);
	printk("head = 0x%08x, data = 0x%08x, tail = 0x%08x, end = 0x%08x\n",
	       (unsigned int)(skb->head), (unsigned int)(skb->data),
	       (unsigned int)(skb->tail), (unsigned int)(skb->end));
	printk("truesize = %d (0x%08x), len = %d (0x%08x)\n",
	       skb->truesize, skb->truesize, skb->len, skb->len);
	printk("headroom = %d (0x%08x), tailroom = %d (0x%08x)\n",
	       skb_headroom(skb), skb_headroom(skb), skb_tailroom(skb), skb_tailroom(skb));
	printk("---------------------------\n");
}

#ifdef ETH_DEBUG

static void priv_data_dump(struct jz_eth_private *priv)
{
	printk("---- priv [0x%08x] -----------------------\n"
	       "tx_ring = 0x%08x, rx_ring = 0x%08x\n"
	       "dma_tx_ring = 0x%08x, dma_rx_ring = 0x%08x\n"
	       "dma_rx_buf = 0x%08x, vaddr_rx_buf = 0x%08x\n"
	       "rx_head = 0x%08x, tx_head = 0x%08x\n"
	       "tx_tail = 0x%08x, tx_full = 0x%08x, tx_skb = 0x%08x\n"
	       "---------------------------\n",
	       (unsigned int)priv,
	       (unsigned int)priv->tx_ring, (unsigned int)priv->rx_ring,
	       (unsigned int)priv->dma_tx_ring, (unsigned int)priv->dma_rx_ring,
	       (unsigned int)priv->dma_rx_buf, (unsigned int)priv->vaddr_rx_buf,
	       (unsigned int)priv->rx_head, (unsigned int)priv->tx_head,
	       (unsigned int)priv->tx_tail, (unsigned int)priv->tx_full, (unsigned int)priv->tx_skb);
}

static void desc_dump(struct jz_eth_private *np)
{
	int i;

	printk("==================================================\n");

	dma_cache_inv((unsigned int)np->tx_ring, sizeof(jz_desc_t) * NUM_TX_DESCS);
	dma_cache_inv((unsigned int)np->rx_ring, sizeof(jz_desc_t) * NUM_RX_DESCS);

	for (i = 0; i < NUM_TX_DESCS; i++) {
		printk("tx desc %2d :[0x%08x],  addr = 0x%08x,  pkt_size = 0x%08x,  next = 0x%08x\n",
		       i, (unsigned int)&(np->tx_ring[i]), (unsigned int)np->tx_ring[i].pkt_addr,
		       (unsigned int)np->tx_ring[i].pkt_size, (unsigned int)np->tx_ring[i].next_desc);
	}

	printk("\n");

	for (i = 0; i < 30/*NUM_RX_DESCS*/; i++) {
		printk("rx desc %2d :[0x%08x],  addr = 0x%08x,  pkt_size = 0x%08x,  next = 0x%08x\n",
		       i, (unsigned int)&(np->rx_ring[i]), (unsigned int)np->rx_ring[i].pkt_addr,
		       (unsigned int)np->rx_ring[i].pkt_size, (unsigned int)np->rx_ring[i].next_desc);
	}

	printk("REG32(ETH_DMA_TDR) = 0x%08x\nREG32(ETH_DMA_RDR) = 0x%08x\n",
	       REG32(ETH_DMA_TDR), REG32(ETH_DMA_RDR));

	printk("t pkt_addr = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_TDR)))->pkt_addr);
	printk("t pkt_size = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_TDR)))->pkt_size);
	printk("t next_desc = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_TDR)))->next_desc);
	
	printk("r pkt_addr = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_RDR)))->pkt_addr);
	printk("r pkt_size = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_RDR)))->pkt_size);
	printk("r next_desc = 0x%08x\n", ((jz_desc_t *)P2ADDR(REG32(ETH_DMA_RDR)))->next_desc);

	//priv_data_dump(np);
	printk("==================================================\n");
}

static void dma_regs_dump(char *str)
{
	printk("%s", str);
	printk("==================================================\n");
	printk("DMA_TCR = 0x%08x    DMA_TDR = 0x%08x\n", REG32(ETH_DMA_TCR), REG32(ETH_DMA_TDR));
	printk("DMA_TSR = 0x%08x    DMA_RCR = 0x%08x\n", REG32(ETH_DMA_TSR), REG32(ETH_DMA_RCR));
	printk("DMA_RDR = 0x%08x    DMA_RSR = 0x%08x\n", REG32(ETH_DMA_RDR), REG32(ETH_DMA_RSR));
	printk("DMA_IMR = 0x%08x    DMA_IR  = 0x%08x\n", REG32(ETH_DMA_IMR), REG32(ETH_DMA_IR));
	printk("==================================================\n");
}

static void mac_regs_dump(void)
{
	printk("==================================================\n");
	printk("MAC_MCR1 = 0x%08x    MAC_MCR2 = 0x%08x\n", REG32(ETH_MAC_MCR1), REG32(ETH_MAC_MCR2));
	printk("MAC_IPGR = 0x%08x    MAC_NIPGR= 0x%08x\n", REG32(ETH_MAC_IPGR), REG32(ETH_MAC_NIPGR));
	printk("MAC_CWR  = 0x%08x    MAC_MFR  = 0x%08x\n", REG32(ETH_MAC_CWR), REG32(ETH_MAC_MFR));
	printk("MAC_PSR  = 0x%08x    MAC_TR   = 0x%08x\n", REG32(ETH_MAC_PSR), REG32(ETH_MAC_TR));
	printk("MAC_MCFGR= 0x%08x    MAC_MCMDR= 0x%08x\n", REG32(ETH_MAC_MCFGR), REG32(ETH_MAC_MCMDR));
	printk("MAC_MADRR= 0x%08x    MAC_MINDR= 0x%08x\n", REG32(ETH_MAC_MADRR), REG32(ETH_MAC_MINDR));
	printk("MAC_SA0  = 0x%08x    MAC_SA1  = 0x%08x    MAC_SA2  = 0x%08x\n",
	       REG32(ETH_MAC_SA0), REG32(ETH_MAC_SA1), REG32(ETH_MAC_SA2));
	printk("==================================================\n");
}

static void fifo_regs_dump(void)
{
	printk("==================================================\n");
	printk("FIFO_CR0 = 0x%08x\n", REG32(ETH_FIFO_CR0));
	printk("FIFO_CR1 = 0x%08x\n", REG32(ETH_FIFO_CR1));
	printk("FIFO_CR2 = 0x%08x\n", REG32(ETH_FIFO_CR2));
	printk("FIFO_CR3 = 0x%08x\n", REG32(ETH_FIFO_CR3));
	printk("FIFO_CR4 = 0x%08x\n", REG32(ETH_FIFO_CR4));
	printk("FIFO_CR5 = 0x%08x\n", REG32(ETH_FIFO_CR5));
#if 0
	printk("RAR0 = 0x%08x    RAR1 = 0x%08x\n", REG32(ETH_FIFO_RAR0), REG32(ETH_FIFO_RAR1));
	printk("RAR2 = 0x%08x    RAR3 = 0x%08x\n", REG32(ETH_FIFO_RAR2), REG32(ETH_FIFO_RAR3));
	printk("RAR4 = 0x%08x    RAR5 = 0x%08x\n", REG32(ETH_FIFO_RAR4), REG32(ETH_FIFO_RAR5));
	printk("RAR6 = 0x%08x    RAR7 = 0x%08x\n", REG32(ETH_FIFO_RAR6), REG32(ETH_FIFO_RAR7));
#endif
	printk("==================================================\n");
}

static void stat_regs_dump(void)
{

	printk("==================================================\n");
	printk("ETH_STAT_TR64= 0x%08x, ETH_STAT_TR127= 0x%08x, ETH_STAT_TR255= 0x%08x, ETH_STAT_TR511= 0x%08x\n",
	       REG32(ETH_STAT_TR64), REG32(ETH_STAT_TR127), REG32(ETH_STAT_TR255), REG32(ETH_STAT_TR511));
	printk("ETH_STAT_TR1K= 0x%08x, ETH_STAT_TRMAX= 0x%08x, ETH_STAT_TRMGV= 0x%08x\n",
	       REG32(ETH_STAT_TR1K), REG32(ETH_STAT_TRMAX), REG32(ETH_STAT_TRMGV));
	
	printk("------\nETH_STAT_RBYT= 0x%08x, ETH_STAT_RPKT= 0x%08x, ETH_STAT_RFCS= 0x%08x, ETH_STAT_RMCA= 0x%08x\n",
	       REG32(ETH_STAT_RBYT), REG32(ETH_STAT_RPKT), REG32(ETH_STAT_RFCS), REG32(ETH_STAT_RMCA));
	printk("ETH_STAT_RDRP= 0x%08x\n", REG32(ETH_STAT_RDRP));

	printk("------\nETH_STAT_TBYT= 0x%08x, ETH_STAT_TPKT= 0x%08x, ETH_STAT_TFCS = 0x%08x, ETH_STAT_TNCL= 0x%08x\n",
	       REG32(ETH_STAT_TBYT), REG32(ETH_STAT_TPKT), REG32(ETH_STAT_TFCS), REG32(ETH_STAT_TNCL));
	printk("ETH_STAT_TDRP= 0x%08x\n", REG32(ETH_STAT_TDRP));
	printk("==================================================\n");

/*
	printk("==================================================\n");

	printk("ETH_STAT_TR64= 0x%08x, ETH_STAT_TR127= 0x%08x, ETH_STAT_TR255= 0x%08x, ETH_STAT_TR511= 0x%08x\n",
	       REG32(ETH_STAT_TR64), REG32(ETH_STAT_TR127), REG32(ETH_STAT_TR255), REG32(ETH_STAT_TR511));
	printk("ETH_STAT_TR1K= 0x%08x, ETH_STAT_TRMAX= 0x%08x, ETH_STAT_TRMGV= 0x%08x\n",
	       REG32(ETH_STAT_TR1K), REG32(ETH_STAT_TRMAX), REG32(ETH_STAT_TRMGV));
	printk("ETH_STAT_RBYT= 0x%08x, ETH_STAT_RPKT= 0x%08x, ETH_STAT_RFCS= 0x%08x, ETH_STAT_RMCA= 0x%08x\n",
	       REG32(ETH_STAT_RBYT), REG32(ETH_STAT_RPKT), REG32(ETH_STAT_RFCS), REG32(ETH_STAT_RMCA));
	printk("ETH_STAT_RBCA= 0x%08x, ETH_STAT_RXCF= 0x%08x, ETH_STAT_RXPF= 0x%08x, ETH_STAT_RXUO= 0x%08x\n",
	       REG32(ETH_STAT_RBCA), REG32(ETH_STAT_RXCF), REG32(ETH_STAT_RXPF), REG32(ETH_STAT_RXUO));
	printk("ETH_STAT_RALN= 0x%08x, ETH_STAT_RFLR= 0x%08x, ETH_STAT_RCDE= 0x%08x, ETH_STAT_RCSE= 0x%08x\n",
	       REG32(ETH_STAT_RALN), REG32(ETH_STAT_RFLR), REG32(ETH_STAT_RCDE), REG32(ETH_STAT_RCSE));
	printk("ETH_STAT_RUND= 0x%08x, ETH_STAT_ROVR= 0x%08x, ETH_STAT_RFRG= 0x%08x, ETH_STAT_RJBR= 0x%08x\n",
	       REG32(ETH_STAT_RUND), REG32(ETH_STAT_ROVR), REG32(ETH_STAT_RFRG), REG32(ETH_STAT_RJBR));
	printk("ETH_STAT_RDRP= 0x%08x\n", REG32(ETH_STAT_RDRP));
	printk("ETH_STAT_TBYT= 0x%08x, ETH_STAT_TPKT= 0x%08x, ETH_STAT_TMCA= 0x%08x, ETH_STAT_TBCA= 0x%08x\n",
	       REG32(ETH_STAT_TBYT), REG32(ETH_STAT_TPKT), REG32(ETH_STAT_TMCA), REG32(ETH_STAT_TBCA));
	printk("ETH_STAT_TXPF= 0x%08x, ETH_STAT_TDFR= 0x%08x, ETH_STAT_TEDF= 0x%08x, ETH_STAT_TSCL= 0x%08x\n",
	       REG32(ETH_STAT_TXPF), REG32(ETH_STAT_TDFR), REG32(ETH_STAT_TEDF), REG32(ETH_STAT_TSCL));
	printk("ETH_STAT_TMCL= 0x%08x, ETH_STAT_TLCL= 0x%08x, ETH_STAT_TXCL= 0x%08x, ETH_STAT_TNCL= 0x%08x\n",
	       REG32(ETH_STAT_TMCL), REG32(ETH_STAT_TLCL), REG32(ETH_STAT_TXCL), REG32(ETH_STAT_TNCL));
	printk("ETH_STAT_TPFH= 0x%08x, ETH_STAT_TDRP= 0x%08x, ETH_STAT_TJBR= 0x%08x, ETH_STAT_TFCS= 0x%08x\n",
	       REG32(ETH_STAT_TPFH), REG32(ETH_STAT_TDRP), REG32(ETH_STAT_TJBR), REG32(ETH_STAT_TFCS));
	printk("ETH_STAT_TXCF= 0x%08x, ETH_STAT_TOVR= 0x%08x, ETH_STAT_TUND= 0x%08x, ETH_STAT_TFRG= 0x%08x\n",
	       REG32(ETH_STAT_TXCF), REG32(ETH_STAT_TOVR), REG32(ETH_STAT_TUND), REG32(ETH_STAT_TFRG));
	printk("ETH_STAT_CAR1= 0x%08x, ETH_STAT_CAR2= 0x%08x, ETH_STAT_CARM1= 0x%08x, ETH_STAT_CARM2= 0x%08x\n",
	       REG32(ETH_STAT_CAR1), REG32(ETH_STAT_CAR2), REG32(ETH_STAT_CARM1), REG32(ETH_STAT_CARM2));
	printk("==================================================\n");
*/
}

static void counters_dump(struct jz_eth_private *np)
{
	int i = 0;

	printk("\n");

	do {
		printk("cnts[%d] = %d\n", i, np->carry_counters[i]);
	} while (++i < STAT_CNT_NUM);
}

static void sal_regs_dump(void)
{
	printk("==================================================\n");
	printk("ETH_SAL_AFR = 0x%08x, ETH_SAL_HT1 = 0x%08x, ETH_SAL_HT2 = 0x%08x\n",
	       REG32(ETH_SAL_AFR), REG32(ETH_SAL_HT1), REG32(ETH_SAL_HT2));
	printk("==================================================\n");	
}

/*
 * Display ethernet packet header
 * This routine is used for test function
 */
static void eth_dbg_rx(struct sk_buff *skb, int len) 
{
	int i, j;

	printk("R: %02x:%02x:%02x:%02x:%02x:%02x <- %02x:%02x:%02x:%02x:%02x:%02x len/SAP:%02x%02x [%d]\n",
	       (u8)skb->data[0], (u8)skb->data[1], (u8)skb->data[2], (u8)skb->data[3], (u8)skb->data[4], 
  	       (u8)skb->data[5], (u8)skb->data[6], (u8)skb->data[7], (u8)skb->data[8], (u8)skb->data[9],
  	       (u8)skb->data[10], (u8)skb->data[11], (u8)skb->data[12], (u8)skb->data[13], len);

	for (j = 0; len > 0; j += 16, len -= 16) {
		printk("    %03x: ",j); 
		for (i = 0; i < 16 && i < len; i++) {
			printk("%02x ", (u8)skb->data[i + j]);
		}
		printk("\n");
  	}
	return;
}
 
static void eth_dbg_tx(struct sk_buff *skb, int len) 
{

  	int i, j; 
    
  	printk("T: %02x:%02x:%02x:%02x:%02x:%02x <- %02x:%02x:%02x:%02x:%02x:%02x len/SAP:%02x%02x [%d]\n",
  	       (u8)skb->data[0], (u8)skb->data[1], (u8)skb->data[2], (u8)skb->data[3],
  	       (u8)skb->data[4], (u8)skb->data[5], (u8)skb->data[6], (u8)skb->data[7],
  	       (u8)skb->data[8], (u8)skb->data[9], (u8)skb->data[10], (u8)skb->data[11],
  	       (u8)skb->data[12], (u8)skb->data[13], len);
 
  	for (j = 0; len > 0; j += 16, len -= 16) { 
  		printk("    %03x: ",j); 
  		for (i = 0; i < 16 && i < len; i++) { 
  			printk("%02x ", (u8)skb->data[i+j]); 
  		} 
  		printk("\n"); 
  	} 
  	return; 
} 

#if 0
/*
 * Show all mii registers  -  this routine is used for test
 */
static void mii_db_out(struct net_device *dev) 
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unsigned int mii_test;

	mii_test = mdio_read(dev,np->valid_phy,MII_BMCR);
	DBPRINTK("BMCR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_BMSR);
	DBPRINTK("BMSR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_ANAR);
	DBPRINTK("ANAR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,MII_ANLPAR);
	DBPRINTK("ANLPAR ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,16);
	DBPRINTK("REG16 ====> 0x%4.4x \n",mii_test);
	
	mii_test = mdio_read(dev,np->valid_phy,17);
	DBPRINTK("REG17 ====> 0x%4.4x \n",mii_test);
}
#endif

#endif // ETH_DEBUG

/*
 * MII operation routines 
 */
static inline void mii_wait(void)
{
	int i;
//	for (i = 0; i < MAX_WAIT; i++, mdelay(1)) {
	for (i = 0; i < MAX_WAIT; i++, mdelay(20)) { /* Cynthia, Test, 2010-05-14 */
		if (!__mac_mii_is_busy())
			return ;
	}

//	printk("\nMAC_MCMDR= 0x%04x    MAC_MADRR= 0x%04x    MAC_MINDR = 0x%04x\n",
//	       REG16(ETH_MAC_MCMDR),  REG16(ETH_MAC_MADRR), REG16(ETH_MAC_MINDR));

	if (i == MAX_WAIT)
		printk("MII wait timeout\n");
}

static int mdio_read(struct net_device *dev,int phy_id, int location)
{
	int retval = 0;

	__mac_send_mii_read_cmd(phy_id, location, MII_NO_SCAN);
	mii_wait();
	retval = __mac_mii_read_data();
	
	return retval;
	
}

static void mdio_write(struct net_device *dev,int phy_id, int location, int data)
{
	__mac_send_mii_write_cmd(phy_id, location, data);
	mii_wait();
}


/*
 * Search MII phy
 */
static int jz_search_mii_phy(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	int phy, phy_idx = 0;

	np->valid_phy = 0xff;
	for (phy = 0; phy < 32; phy++) {
		int mii_status = mdio_read(dev, phy, 1);
		if (mii_status != 0xffff  &&  mii_status != 0x0000) {
			np->phys[phy_idx] = phy;
			np->ecmds[phy_idx].speed=SPEED_100;
			np->ecmds[phy_idx].duplex=DUPLEX_FULL;
			np->ecmds[phy_idx].port=PORT_MII;
			np->ecmds[phy_idx].transceiver=XCVR_INTERNAL;
			np->ecmds[phy_idx].phy_address=np->phys[phy_idx];
			np->ecmds[phy_idx].autoneg=AUTONEG_ENABLE;
			np->ecmds[phy_idx].advertising=(ADVERTISED_10baseT_Half |
							ADVERTISED_10baseT_Full |
							ADVERTISED_100baseT_Half |
							ADVERTISED_100baseT_Full);
			phy_idx++;
		}
	}
	if (phy_idx == 1) {
		np->valid_phy = np->phys[0];
		np->phy_type = 0;
	}
	if (phy_idx != 0) {
		phy = np->valid_phy;
		np->advertising = mdio_read(dev,phy, 4);
	}
	return phy_idx;
}

#if 1 // multicast

/*
 * CRC calc for Destination Address for gets hashtable index
 */
#define POLYNOMIAL 0x04c11db7UL
static u16 jz_hashtable_index(u8 *addr)
{
#if 1
	u32 crc = 0xffffffff, msb;
	int  i, j;
	u32  byte;
	for (i = 0; i < 6; i++) {
		byte = *addr++;
		for (j = 0; j < 8; j++) {
			msb = crc >> 31;
			crc <<= 1;
			if (msb ^ (byte & 1)) crc ^= POLYNOMIAL;
			byte >>= 1;
		}
	}
	return ((int)(crc >> 26));
#endif
#if 0
	int crc = -1;
	int length=6;
	int bit;
	unsigned char current_octet;
	while (--length >= 0) {
		current_octet = *addr++;
		for (bit = 0; bit < 8; bit++, current_octet >>= 1)
			crc = (crc << 1) ^ ((crc < 0) ^ (current_octet & 1) ?
			     POLYNOMIAL : 0);
	}
	return ((int)(crc >> 26));
#endif
}

/*
 * Multicast filter and config multicast hash table
 */
#define MULTICAST_FILTER_LIMIT 64

static void jz_set_multicast_list(struct net_device *dev)
{
	int i, hash_index;
	u32 hash_h, hash_l, hash_bit;
	
	if (dev->flags & IFF_PROMISC) {
		/* Accept any kinds of packets */
		__sal_set_mode(AFR_PRO);
		__sal_set_hash_table(0xffffffff, 0xffffffff);

		printk("%s: Enter promisc mode!\n",dev->name);
	} else  if ((dev->flags & IFF_ALLMULTI) || (dev->mc_count > MULTICAST_FILTER_LIMIT)) {
		/* Accept all multicast packets */
		__sal_set_mode(AFR_PRM);
		__sal_set_hash_table(0xffffffff, 0xffffffff);
		printk("%s: Enter allmulticast mode!   %d \n",dev->name,dev->mc_count);
	} else if (dev->flags & IFF_MULTICAST) {
		/* Update multicast hash table */
		struct dev_mc_list *mclist;
		__sal_get_hash_table(hash_h, hash_l);

		for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
		     i++, mclist = mclist->next)
		{
			hash_index = jz_hashtable_index(mclist->dmi_addr);
			hash_bit=0x00000001;
			hash_bit <<= (hash_index & 0x1f);
			if (hash_index > 0x1f) 
				hash_h |= hash_bit;
			else
				hash_l |= hash_bit;

#ifdef ETH_DEBUG
			DBPRINTK("----------------------------\n");
			{
				int j;
				for (j=0;j<mclist->dmi_addrlen;j++)
					printk("%2.2x:",mclist->dmi_addr[j]);
				printk("\n");
			}
#endif
			//printk("dmi.addrlen => %d\n",mclist->dmi_addrlen);
			//printk("dmi.users   => %d\n",mclist->dmi_users);
			//printk("dmi.gusers  => %d\n",mclist->dmi_users);
		}
		__sal_set_hash_table(hash_h, hash_l);

		__sal_set_mode(AFR_AMC);

		//printk("This is multicast hash table high bits [%4.4x]\n",readl(ETH_SAL_HT1));
		//printk("This is multicast hash table low  bits [%4.4x]\n",readl(ETH_SAL_HT2));
		//printk("%s: Enter multicast mode!\n",dev->name);
	}
}
#endif // multicast

static inline int jz_phy_reset(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unsigned int mii_reg0;
	unsigned int count;
	
	mii_reg0 = mdio_read(dev,np->valid_phy, MII_BMCR);
	mii_reg0 |= MII_CR_RST;

	mdio_write(dev, np->valid_phy, MII_BMCR, mii_reg0);	//reset phy
	for (count = 0; count < 2000; count++) {
		mdelay(1);
		mii_reg0 = mdio_read(dev,np->valid_phy, MII_BMCR);
		if (!(mii_reg0 & MII_CR_RST))
			break;  //reset completed
	}

	if (count == 2000) 
		return 1;     //phy error
	else
		return 0;
}

/*
 * Start Auto-Negotiation function for PHY 
 */
/*
static int jz_autonet_complete(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	int count;
	u32 mii_reg1, timeout = 3000;

	for (count = 0; count < timeout; count++) {
		mdelay(1);
		mii_reg1 = mdio_read(dev,np->valid_phy, MII_BMSR);
		if (mii_reg1 & 0x0020)
			break;
	}
	//mii_db_out(dev);  //for debug to display all register of MII
	if (count >= timeout) 
		return 1;     //auto negotiation  error
	else
		return 0;
}  
*/

/*
 * Get current mode of eth phy
 */
static u32 jz_eth_curr_mode(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unsigned int mii_reg17;
	u32 flag = 0;

	mii_reg17 = mdio_read(dev,np->valid_phy,MII_DSCSR); 
	np->media = mii_reg17>>12;
	if (np->media==8) {
		infoprintk("%s: Current Operation Mode is [100M Full Duplex]",dev->name);
		flag = 0;
		np->full_duplex=1;
	}
	if (np->media==4) {
		infoprintk("%s: Current Operation Mode is [100M Half Duplex]",dev->name);
		flag = 0;
		np->full_duplex=0;
	}
	if (np->media==2) {
		infoprintk("%s: Current Operation Mode is [10M Full Duplex]",dev->name);
		np->full_duplex=1;
	}
	if (np->media==1) {
		infoprintk("%s: Current Operation Mode is [10M Half Duplex]",dev->name);
		np->full_duplex=0;
	}
	printk("\n");
	return flag;
}

static void config_mac(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	u32	mac_cfg_1 = 0, mac_cfg_2 = 0;

	// Enable tx & rx flow control, enable receive
	mac_cfg_1 = MCR1_TFC | MCR1_RFC | MCR1_RE;

#ifdef USE_RMII
	// Enable RMII
	mac_cfg_1 |= 1 << 13;
#endif

	// Enable loopack mode
	//mac_cfg_1 |= MCR1_LB;

	/* bit 7	bit 6		bit 5
	   MCR2_ADPE	MCR2_VPE	MCR2_PCE
	   x		x		0		No pad, check CRC
	 > 0		0		1		Pad to 60B, append CRC
	   x		1		1		Pad to 64B, append CRC
	   1		0		1		if un-tagged, Pad to 60B, append CRC
	  						if VLAN tagged, Pad to 64B, append CRC
	 
	   if set MCR2_PCE(bit 5)
	  	MCR2_CE(bit 4) must be set.
	 
	   We need to pad frame to 60B and append 4-byte CRC.
	 */

	mac_cfg_2 = MCR2_PCE | MCR2_CE;

	// Pure preamble enforcement
	//mac_cfg_2 |= MCR2_PPE;

	// Frame length checking
	mac_cfg_2 |= MCR2_FLC;

	if (np->full_duplex) {
		mac_cfg_2 |= MCR2_FD;
		__mac_set_IPGR(0x15);
		
	} else {
		
		__mac_set_IPGR(0x12);
	}

	REG16(ETH_MAC_MCR1) = mac_cfg_1;
	REG16(ETH_MAC_MCR2) = mac_cfg_2;

	__mac_set_NIPGR1(0x0c);
	__mac_set_NIPGR2(0x12);

	//mac_regs_dump();
}

static void config_fifo(void)
{
	int	i;

	__fifo_reset_all();

	REG32(ETH_FIFO_CR0) |= 0x80000000;

	/* 4k rx FiFo */

	__fifo_set_fr_threshold(0x180);
	__fifo_set_high_wm(0x200);
	__fifo_set_low_wm(0x40);

	/* 4k tx FiFo */
/*
	__fifo_set_ft_threshold(0x0200);
	__fifo_set_ft_high_wm(0x0300);
*/
	/* for 2k FiFo both tx */
	__fifo_set_ft_threshold(0x0180);
	__fifo_set_ft_high_wm(0x01b0);

	__fifo_set_XOFF_RTX(4);
	__fifo_set_pause_control();

	REG32(ETH_FIFO_CR4) &= 0x0000;
	REG32(ETH_FIFO_CR5) |= 0xffff;

	__fifo_set_drop_cond(RSV_CRCE);
	__fifo_set_dropdc_cond(RSV_CRCE);

	__fifo_set_drop_cond(RSV_MP);
	__fifo_set_dropdc_cond(RSV_MP);

	__fifo_set_drop_cond(RSV_BP);
	__fifo_set_dropdc_cond(RSV_BP);

	__fifo_set_drop_cond(RSV_LCE);
	__fifo_set_dropdc_cond(RSV_LCE);

	__fifo_enable_all_modules();

	for (i = 0;
	     i < MAX_WAIT && !__fifo_all_enabled();
	     i++, udelay(100)) {
		;
	}

	if (i == MAX_WAIT) {
		printk("config_fifo: Wait time out !\n");
		return;
	}
	//fifo_regs_dump();
}

static void config_sal(void)
{
	/* SAL config */
	__sal_set_mode(AFR_AMC | AFR_ABC);
	__sal_set_hash_table(0, 0);
}

/*
static void config_stat(void)
{
	__stat_disable();
	__stat_clear_counters();
	// clear carry registers
	REG32(ETH_STAT_CAR1) = REG32(ETH_STAT_CAR1);
	REG32(ETH_STAT_CAR2) = REG32(ETH_STAT_CAR2);
	__stat_disable_carry_irq();
	__stat_enable_carry_irq();
	//printk("CARM1 = 0x%08x, CARM2 = 0x%08x\n", REG32(ETH_STAT_CARM1), REG32(ETH_STAT_CARM2));

	__stat_enable();
	printk("Enable eth stat module.\n");

}
*/

static int autonet_complete(struct net_device *dev)
{
	int	i;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	for (i = 0;
	     i < MAX_WAIT && !(mdio_read(dev, np->valid_phy, MII_BMSR) & 0x0020);
	     i++, udelay(10)) {
		;
	}

	if (i == MAX_WAIT) {
		printk("%s: Autonet time out!\n", dev->name);
		return -1;
	}

	return 0;
}

static void config_phy(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	u32	mii_anlpar, phy_mode;

	mii_anlpar = mdio_read(dev, np->valid_phy, MII_ANLPAR);

	if (mii_anlpar != 0xffff) {
		mii_anlpar = mdio_read(dev, np->valid_phy, MII_ANLPAR); // read twice to make data stable
		if ((mii_anlpar & 0x0100) || (mii_anlpar & 0x01C0) == 0x0040) {
			np->full_duplex = 1;
		}
		phy_mode = mii_anlpar >> 5;

#ifdef USE_RMII

		if (phy_mode & MII_ANLPAR_100M)
			REG32(ETH_MAC_PSR) |= PSR_OS;
#endif

		printk("%s: Setting %s %s-duplex based on MII tranceiver #%d\n",
		       dev->name, (phy_mode & MII_ANLPAR_100M) ? "100Mbps" : "10Mbps",
		       np->full_duplex ? "full" : "half", np->mii_phy_cnt);

	} else {
		printk("%s: config_phy: MII_ANLPAR is 0xFFFF, may be error ???\n", dev->name);
	}

}

/*
 * Ethernet device hardware init
 * This routine initializes the ethernet device hardware and PHY
 */
static int jz_init_hw(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	struct ethtool_cmd ecmd;
	int i;

#if defined(CONFIG_SOC_JZ4770) || defined(CONFIG_SOC_JZ4810)
	__gpio_as_eth();
#endif

	__eth_disable_irq();
	__mac_set_mii_clk(0x7);
	__mac_reset();

#if 0
	/* MII operation */
	if (jz_phy_reset(dev)) {
		errprintk("Ethernet PHY device does not reset!\n");
		//return operation not permitted 
		return -EPERM;
	}
#endif

	__eth_set_mac_address(dev->dev_addr[0], dev->dev_addr[1],
			      dev->dev_addr[2], dev->dev_addr[3],
			      dev->dev_addr[4], dev->dev_addr[5]);

	printk("%s: JZ On-Chip ethernet (MAC ", dev->name);
	for (i = 0; i < 5; i++) {
		printk("%2.2x:", dev->dev_addr[i]);
	}
	printk("%2.2x, IRQ %d)\n", dev->dev_addr[i], dev->irq);

	np->mii_phy_cnt = jz_search_mii_phy(dev);
	printk("%s: Found %d PHY on JZ MAC\n", dev->name, np->mii_phy_cnt);
//  New code, could make it work on 100M mode ...
	if (autonet_complete(dev))
		printk("ETH Auto-Negotiation failed\n");
	
	config_phy(dev);
//  old configuration
	mii_info.phy_id = np->valid_phy;
	mii_info.dev = dev;
	mii_info.mdio_read = &mdio_read;
	mii_info.mdio_write = &mdio_write;

	ecmd.speed = SPEED_100;
	ecmd.duplex = DUPLEX_FULL;
	ecmd.port = PORT_MII;
	ecmd.transceiver = XCVR_INTERNAL;
	ecmd.phy_address = np->valid_phy;
	ecmd.autoneg = AUTONEG_ENABLE;

/*
	mii_ethtool_sset(&mii_info, &ecmd);
	if (jz_autonet_complete(dev)) 
		errprintk("%s: Ethernet Module AutoNegotiation failed\n",dev->name);
	mii_ethtool_gset(&mii_info, &ecmd);

	infoprintk("%s: Provide Modes: ",dev->name);
	for (i = 0; i < 5;i++) 
		if (ecmd.advertising & (1<<i))
			printk("(%d)%s", i+1, media_types[i]);
	printk("\n");  

	flag = jz_eth_curr_mode(dev);
*/

	config_mac(dev);

	config_fifo();

	config_sal();

//	config_stat();

	/* Set base address of TX and RX descriptors */
	__eth_set_rx_desc_addr(np->dma_rx_ring);
	__eth_set_tx_desc_addr(np->dma_tx_ring);

	/* Burst length: 4, 8 or 16 */
	//__eth_dma_set_burst_len(BURST_LEN_4);

	//dma_regs_dump("set burst length\n");

	/* Clear status registers */
	__eth_clear_rx_flags();
	__eth_clear_rx_pkt_cnt();
	__eth_clear_tx_flags();
	__eth_clear_tx_pkt_cnt();

	__eth_enable_irq();
	__eth_dma_rx_enable();

	return 0;
}

static void jz_eth_read_stats(struct jz_eth_private *np, int carry1, int carry2)
{
	return ;

	if (carry1 != 0) {
		if (carry1 & CAR1_RPK) np->carry_counters[CNT_RPKT]++;
		if (carry1 & CAR1_RBY) np->carry_counters[CNT_RBYT]++;
		if (carry1 & CAR1_RFC) np->carry_counters[CNT_RFCS]++;
		if (carry1 & CAR1_RDR) np->carry_counters[CNT_RDRP]++;
		if (carry1 & CAR1_RMC) np->carry_counters[CNT_RMCA]++;
		printk("carry1 = 0x%08x\n", carry1);
	}

	if (carry2 != 0) {
		if (carry2 & CAR2_TPK) np->carry_counters[CNT_TPKT]++;
		if (carry2 & CAR2_TBY) np->carry_counters[CNT_TBYT]++;
		if (carry2 & CAR2_TFC) np->carry_counters[CNT_TFCS]++;
		if (carry2 & CAR2_TDP) np->carry_counters[CNT_TDRP]++;
		if (carry2 & CAR2_TNC) np->carry_counters[CNT_TNCL]++;
		printk("carry2 = 0x%08x\n", carry2);
	}

	np->stats.rx_packets	= REG32(ETH_STAT_RPKT) + (np->carry_counters[CNT_RPKT] << 18);
	np->stats.tx_packets	= REG32(ETH_STAT_TPKT) + (np->carry_counters[CNT_TPKT] << 18);
	np->stats.rx_bytes	= REG32(ETH_STAT_RBYT) + (np->carry_counters[CNT_RBYT] << 24);
	np->stats.tx_bytes	= REG32(ETH_STAT_TBYT) + (np->carry_counters[CNT_TBYT] << 24);
	np->stats.rx_errors	= REG32(ETH_STAT_RFCS) + (np->carry_counters[CNT_RFCS] << 12);
	np->stats.tx_errors	= REG32(ETH_STAT_TFCS) + (np->carry_counters[CNT_TFCS] << 12);
	np->stats.rx_dropped	= REG32(ETH_STAT_RDRP) + (np->carry_counters[CNT_RDRP] << 12);
	np->stats.tx_dropped	= REG32(ETH_STAT_TDRP) + (np->carry_counters[CNT_TDRP] << 12);
	np->stats.multicast	= REG32(ETH_STAT_RMCA) + (np->carry_counters[CNT_RMCA] << 18);
	np->stats.collisions	= REG32(ETH_STAT_TNCL) + (np->carry_counters[CNT_TNCL] << 13);

	//counters_dump(np);

}

static int jz_eth_open(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	int retval, i;

	retval = request_irq(dev->irq, jz_eth_interrupt, 0, dev->name, dev);
	if (retval) {
		errprintk("%s: Unable to get IRQ %d .\n", dev->name, dev->irq);
		return -EAGAIN;
	}

	for (i = 0; i < NUM_RX_DESCS; i++) {
		np->rx_ring[i].pkt_addr	= cpu_to_le32(np->dma_rx_buf + i * RX_BUF_SIZE);
		np->rx_ring[i].pkt_size	= cpu_to_le32(EMPTY_FLAG_MASK);
		np->rx_ring[i].next_desc= cpu_to_le32(np->dma_rx_ring + (i+1) * sizeof (jz_desc_t));
	}
	np->rx_ring[NUM_RX_DESCS - 1].next_desc = cpu_to_le32(np->dma_rx_ring);

	for (i = 0; i < NUM_TX_DESCS; i++) {
		np->tx_ring[i].pkt_addr	= cpu_to_le32(0);
		np->tx_ring[i].pkt_size	= cpu_to_le32(EMPTY_FLAG_MASK);
		np->tx_ring[i].next_desc= cpu_to_le32(np->dma_tx_ring + (i+1) * sizeof (jz_desc_t));
	}
	np->tx_ring[NUM_TX_DESCS - 1].next_desc = cpu_to_le32(np->dma_tx_ring);

	np->rx_head = 0;
	np->tx_head = np->tx_tail = 0;

	//desc_dump(dev);

	for (i = 0; i < STAT_CNT_NUM; i++) {
		np->carry_counters[i] = 0;
	}

	jz_init_hw(dev);

	dev->trans_start = jiffies;
	netif_start_queue(dev);
	start_check(dev);

	return 0;
}

static int jz_eth_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	close_check(dev);

	__eth_disable();
	
	free_irq(dev->irq, dev);
	return 0;
}

/*
 * Get the current statistics.
 * This may be called with the device open or closed.
 */
static struct net_device_stats * jz_eth_get_stats(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

//	unsigned int	flags;

//	spin_lock_irqsave(&np->lock, flags);

	jz_eth_read_stats(np, 0, 0);

//	spin_unlock_irqrestore(&np->lock, flags);

	return &np->stats;
}

/*
 * ethtool routines
 */
static int jz_ethtool_ioctl(struct net_device *dev, void *useraddr)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	u32 ethcmd;

	/* dev_ioctl() in ../../net/core/dev.c has already checked
	   capable(CAP_NET_ADMIN), so don't bother with that here.  */

	if (get_user(ethcmd, (u32 *)useraddr))
		return -EFAULT;

	switch (ethcmd) {

	case ETHTOOL_GDRVINFO: {
		struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
		strcpy (info.driver, DRV_NAME);
		strcpy (info.version, DRV_VERSION);
		strcpy (info.bus_info, "OCS");
		if (copy_to_user (useraddr, &info, sizeof (info)))
			return -EFAULT;
		return 0;
	}

	/* get settings */
	case ETHTOOL_GSET: {
		struct ethtool_cmd ecmd = { ETHTOOL_GSET };
		spin_lock_irq(&np->lock);
		mii_ethtool_gset(&mii_info, &ecmd);
		spin_unlock_irq(&np->lock);
		if (copy_to_user(useraddr, &ecmd, sizeof(ecmd)))
			return -EFAULT;
		return 0;
	}
	/* set settings */
	case ETHTOOL_SSET: {
		int r;
		struct ethtool_cmd ecmd;
		if (copy_from_user(&ecmd, useraddr, sizeof(ecmd)))
			return -EFAULT;
		spin_lock_irq(&np->lock);
		r = mii_ethtool_sset(&mii_info, &ecmd);
		spin_unlock_irq(&np->lock);
		return r;
	}
	/* restart autonegotiation */
	case ETHTOOL_NWAY_RST: {
		return mii_nway_restart(&mii_info);
	}
	/* get link status */
	case ETHTOOL_GLINK: {
		struct ethtool_value edata = {ETHTOOL_GLINK};
		edata.data = mii_link_ok(&mii_info);
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}

	/* get message-level */
	case ETHTOOL_GMSGLVL: {
		struct ethtool_value edata = {ETHTOOL_GMSGLVL};
		edata.data = debug;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}
	/* set message-level */
	case ETHTOOL_SMSGLVL: {
		struct ethtool_value edata;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		debug = edata.data;
		return 0;
	}

	default:
		break;
	}

	return -EOPNOTSUPP;
}

/*
 * Config device
 */
static int jz_eth_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	struct mii_ioctl_data *data, rdata;

	switch (cmd) {
	case SIOCETHTOOL:
		return jz_ethtool_ioctl(dev, (void *) rq->ifr_data);
	case SIOCGMIIPHY:
	case SIOCDEVPRIVATE:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		data->phy_id = np->valid_phy;
	case SIOCGMIIREG:
	case SIOCDEVPRIVATE+1:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		data->val_out = mdio_read(dev,np->valid_phy, data->reg_num & 0x1f);
		return 0;
	case SIOCSMIIREG:
	case SIOCDEVPRIVATE+2:
		data = (struct mii_ioctl_data *)&rq->ifr_data;
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		mdio_write(dev,np->valid_phy, data->reg_num & 0x1f, data->val_in);
		return 0;
	case READ_COMMAND:	
		data = (struct mii_ioctl_data *)rq->ifr_data;
		if (copy_from_user(&rdata,data,sizeof(rdata)))
			return -EFAULT;
		rdata.val_out = mdio_read(dev,rdata.phy_id, rdata.reg_num & 0x1f);
		if (copy_to_user(data,&rdata,sizeof(rdata)))
			return -EFAULT;
		return 0;
	case WRITE_COMMAND:
		if (np->phy_type==1) {
			data = (struct mii_ioctl_data *)rq->ifr_data;
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			if (copy_from_user(&rdata,data,sizeof(rdata)))
				return -EFAULT;
			mdio_write(dev,rdata.phy_id, rdata.reg_num & 0x1f, rdata.val_in);
		}
		return 0;
	case GETDRIVERINFO:
		if (np->phy_type==1) {
			data = (struct mii_ioctl_data *)rq->ifr_data;
			if (copy_from_user(&rdata,data,sizeof(rdata)))
				return -EFAULT;
			rdata.val_in = 0x1;
			rdata.val_out = 0x00d0;
			if (copy_to_user(data,&rdata,sizeof(rdata)))
				return -EFAULT;
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*
 * Received one packet
 */
static void eth_rxready(struct net_device *dev, int counter)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	jz_desc_t	*rx_current_desc;
	struct sk_buff	*skb;
	unsigned char	*pkt_ptr;
	u32		pkt_len;

	int counter_bak = counter;
	int i = 0;

	/* empty_flag == 0 */
	while (counter-- > 0) {
		i++;

		rx_current_desc = &(np->rx_ring[np->rx_head]);
		//dma_cache_inv((unsigned int)rx_current_desc, sizeof(jz_desc_t));

		if (__desc_get_empty_flag(le32_to_cpu(rx_current_desc->pkt_size))) {
			printk("%s: ?????????? fix me ??? %s, %d\n", dev->name, __FUNCTION__, __LINE__);
			printk("counter_bak = %d, counter = %d\n", counter_bak, counter);
			return ;
		}

		pkt_ptr = (unsigned char *)((rx_current_desc->pkt_addr) | 0xa0000000);
		pkt_len = (__desc_get_pkt_size(le32_to_cpu(rx_current_desc->pkt_size))) - 4;

		np->stats.rx_packets++;
		np->stats.rx_bytes += pkt_len;

		skb = dev_alloc_skb(pkt_len + 2);
		if (skb == NULL) {
			printk("%s: Memory squeeze, dropping. dev_alloc_skb(0x%08x)\n",
			       dev->name, pkt_len + 2);
			np->stats.rx_dropped++;
			__eth_reduce_pkt_recv_cnt();
			//break;
			return ;
		}
		skb->dev = dev;
		skb_reserve(skb, 2); /* 16 byte align */

#if 0
		rx_buf_addr_array[rx_buf_addr_array_idx] = (unsigned int)skb;
		rx_buf_addr_array_idx == DUMP_RX_BUF_ADDR_MAX - 1 ? rx_buf_addr_array_idx = 0 : rx_buf_addr_array_idx++;
#endif

		dma_cache_inv((unsigned int)pkt_ptr, pkt_len);
		memcpy(skb->data, pkt_ptr, pkt_len);
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb,dev);

		netif_rx(skb);	/* pass the packet to upper layers */
		dev->last_rx = jiffies;

		__eth_reduce_pkt_recv_cnt();

		rx_current_desc->pkt_size = EMPTY_FLAG_MASK;

		//dma_cache_wback((unsigned int)rx_current_desc, sizeof(jz_desc_t));

		np->rx_head == NUM_RX_DESCS - 1 ? np->rx_head = 0 : np->rx_head++;
		BUG_ON(np->rx_head >= NUM_RX_DESCS);
	}

	if (i == 0) {
		printk("%s: eth_rxready ... BUG_ON\n", dev->name);
		BUG_ON(1);
	} else {
		__eth_dma_rx_enable();
	}
}

/*
 * Tx timeout routine 
 */
static void jz_eth_tx_timeout(struct net_device *dev)
{
	jz_init_hw(dev);
	netif_wake_queue(dev);
}

/*
 * One packet was transmitted
 */
static void eth_txdone(struct net_device *dev, int counter)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	int tx_tail = np->tx_tail;

	while (counter-- > 0) {

#if NUM_RX_DESCS == 1
		int entry = 0;
#else
		int entry = tx_tail % NUM_TX_DESCS;
#endif

		/* Free the original skb */
		if (np->tx_skb[entry]) {
			np->stats.tx_packets++;
			np->stats.tx_bytes += le32_to_cpu(np->tx_ring[entry].pkt_size);

			dev_kfree_skb_irq(np->tx_skb[entry]);

			np->tx_skb[entry] = 0;
		} else {
			printk("%s: ?????????? fix me ??? %s, %d\n", dev->name, __FUNCTION__, __LINE__);
			//desc_dump(dev);
			//dma_regs_dump("");
		}

		tx_tail++;
		__eth_reduce_pkt_sent_cnt();
	}

	if (np->tx_full && (tx_tail + NUM_TX_DESCS > np->tx_head + 1)) {
		/* The ring is no longer full */
		np->tx_full = 0;
		netif_start_queue(dev);
	}
	np->tx_tail = tx_tail;
}

/*
 * Update the tx descriptor
 */
static void load_tx_packet(struct net_device *dev, char *buf, u32 length, struct sk_buff *skb)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	int entry = np->tx_head % NUM_TX_DESCS;
	int i;

	
#if 0
	tx_buf_addr_array[tx_buf_addr_array_idx] = (unsigned int)cpu_to_le32(virt_to_bus(buf));
	tx_buf_addr_array_idx == DUMP_TX_BUF_ADDR_MAX - 1 ? tx_buf_addr_array_idx = 0 : tx_buf_addr_array_idx++;
#endif

	np->tx_ring[entry].pkt_addr = cpu_to_le32(virt_to_bus(buf));
	np->tx_ring[entry].pkt_size = cpu_to_le32(length & ~EMPTY_FLAG_MASK);
	np->tx_skb[entry] = skb;

	/* Notice us when will send a packet which begin with address: xxx1(binary) */
	if (unlikely(np->tx_ring[entry].pkt_addr & 0x1)) {
		skb_dump(skb);
		printk("desc.pktaddr = 0x%08x\n", np->tx_ring[entry].pkt_addr);
		for (i = -2; i < 16; i++) {
			printk("%02x ", *((unsigned char *)le32_to_cpu(bus_to_virt(np->tx_ring[entry].pkt_addr)) + i));
		}
		printk("\n");
		for (i = -2; i < 16; i++) {
			printk("%02x ", *((unsigned char *)le32_to_cpu(buf) + i));
		}
		printk("\n");
		for (i = -2; i < 16; i++) {
			printk("%02x ", *((unsigned char *)(bus_to_virt(np->tx_ring[entry].pkt_addr)) + i));
		}
		printk("\n");
	}
}

/*
 * Transmit one packet
 */
static int jz_eth_send_packet(struct sk_buff *skb, struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif
	u32 length;

	if (np->tx_full) {
		return 0;
	}

	length = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	dma_cache_wback((unsigned long)skb->data, length);

	load_tx_packet(dev, (char *)skb->data, length, skb);

	spin_lock_irq(&np->lock);

	np->tx_head++;

	/* Start TX */
	__eth_dma_tx_enable();

	/* for timeout */
	dev->trans_start = jiffies;

	if (np->tx_tail + NUM_TX_DESCS > np->tx_head + 1) {
		np->tx_full = 0;
	} else {
		np->tx_full = 1;
		netif_stop_queue(dev);
	}

	spin_unlock_irq(&np->lock);
	return 0;
}

/*
 * Interrupt service routine
 */
static irqreturn_t jz_eth_interrupt(int irq, void *dev_id)
{
	struct net_device	*dev = (struct net_device *)dev_id;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif
	unsigned int	tx_sts;
	unsigned int	rx_sts;
	unsigned int	counter;
	unsigned int	carry1 = 0;
	unsigned int	carry2 = 0;

	spin_lock(&np->lock);

	__eth_disable_irq();

	/* Read tx & rx state reg, which indicate action */
	tx_sts = REG32(ETH_DMA_TSR);
	rx_sts = REG32(ETH_DMA_RSR);

	__eth_clear_tx_flags(); /* Clear UNDERRUN and BUSERROR */
	__eth_clear_rx_flags(); /* Clear OVERFLOW and BUSERROR */

	/* Tx PKTSENT: one or more frames have already been sent. Current tx round has completed. */
	if (tx_sts & TSR_PKTSENT) {
		/* To clear counter and PKTSENT bit,
		 * __eth_reduce_pkt_sent_cnt() was called inside */
		counter = (tx_sts & TSR_PKTCNT_MASK) >> 16;
		if (counter != 0) {
			eth_txdone(dev, counter);
		} else {
			printk("%s: Packet has been sent but counter is 0\n", dev->name);
			BUG_ON(1);
		}
	}

	/* Rx PKTRECV: one frame has already been received. Receive frame until there is no PKTRECV */
	if (rx_sts & RSR_PKTRECV) {
		/* To reduce counter and PKTRECV bit,
		 * __eth_reduce_pkt_recv_cnt() must called after each round */
		counter = (rx_sts & RSR_PKTCNT_MASK) >> 16;
		if (counter != 0) {
			eth_rxready(dev, counter);
		} else {
			printk("%s: Packet has been received but count is 0\n", dev->name);
			BUG_ON(1);
		}
	}

	if ( !(tx_sts & TSR_FLAGS) && !(rx_sts & RSR_FLAGS) ) {
		carry1 = REG32(ETH_STAT_CAR1);
		carry2 = REG32(ETH_STAT_CAR2);
		REG32(ETH_STAT_CAR1) = carry1;
		REG32(ETH_STAT_CAR2) = carry2;

/*
		if (carry1 || carry2) {
			jz_eth_read_stats(np, carry1, carry2);
		} else {
			printk("tx_sts = 0x%08x, rx_sts = 0x%08x\n", tx_sts, rx_sts);
			printk("BUG: interrupt, no irq source\n");
			BUG_ON(1);
		}
*/
		goto _exit_irq;
	}

	/* Fatal bus error */
	if (tx_sts & TSR_BUSERR || rx_sts & RSR_BUSERR) {
		__eth_disable();
		printk("%s: Fatal bus error occurred, sts=[%#8x %#8x], device stopped.\n", dev->name, tx_sts, rx_sts);
		goto _exit_irq;
	}

	/* Rx overflow */
	if (rx_sts & RSR_OVERFLOW) {
		printk("RX_OVERFLOW\n");
		__eth_dma_rx_disable();
	}

	/* Tx underrun, it occurred after every tx round */
	if (tx_sts & TSR_UNDERRUN) {
		//printk("TX_UNDERRUN\n");
	}

_exit_irq:

	__eth_enable_irq();

	spin_unlock(&np->lock);

	return IRQ_HANDLED;
}

#if 0 //def CONFIG_PM
/*
 * Suspend the ETH interface.
 */
static int jz_eth_suspend(struct net_device *dev, int state)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *jep = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *jep = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unsigned long flags, tmp;

	printk("ETH suspend\n");

	if (!netif_running(dev)) {
		return 0;
	}

	netif_device_detach(dev);

	spin_lock_irqsave(&jep->lock, flags);

	/* Disable interrupts, stop Tx and Rx. */
	__eth_disable_irq();
	__eth_disable();
	__stat_disable_carry_irq();

	spin_unlock_irqrestore(&jep->lock, flags);

	return 0;
}

/*
 * Resume the ETH interface.
 */
static int jz_eth_resume(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	printk("ETH resume.\n");

	if (!netif_running(dev))
		return 0;

	jz_init_hw(dev);

	netif_device_attach(dev);
	jz_eth_tx_timeout(dev);
	netif_wake_queue(dev);

	return 0;
}

static int jz_eth_pm_callback(struct pm_dev *dev, pm_request_t rqst, void *data)
{
	int ret;

	if (!dev->data)
		return -EINVAL;

	switch (rqst) {
	case PM_SUSPEND:
		ret = jz_eth_suspend((struct net_device *)dev->data, (int)data);
		break;

	case PM_RESUME:
		ret = jz_eth_resume((struct net_device *)dev->data);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#endif /* CONFIG_PM */


// Cynthia added
static const struct net_device_ops jz_netdev_ops = {

	.ndo_open = jz_eth_open,
	.ndo_stop = jz_eth_close,
	.ndo_start_xmit = jz_eth_send_packet,
	.ndo_get_stats = jz_eth_get_stats,
	.ndo_set_multicast_list = jz_set_multicast_list,
	.ndo_do_ioctl = jz_eth_ioctl,
	.ndo_tx_timeout = jz_eth_tx_timeout,
   
};

  //=== Cynthia end == //


static int __init jz_eth_init(void)
{
	struct net_device *dev;
	struct jz_eth_private *np;
	int err;

	dev = alloc_etherdev(sizeof(struct jz_eth_private));
	if (!dev) {
		printk(KERN_ERR "%s: Alloc_etherdev failed\n", DRV_NAME);
		return -ENOMEM;
	}
	netdev = dev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	np = (struct jz_eth_private *)P2ADDR(dev->priv);
	dev->priv = np;
#else
	np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif
	memset(np, 0, sizeof(struct jz_eth_private));

	np->vaddr_rx_buf = (u32)dma_alloc_noncoherent(NULL, NUM_RX_DESCS * RX_BUF_SIZE, 
						      &np->dma_rx_buf, 0);

	if (!np->vaddr_rx_buf) {
		printk(KERN_ERR "%s: Cannot alloc dma buffers\n", DRV_NAME);
		unregister_netdev(dev);
		free_netdev(dev);
		return -ENOMEM;
	}

	np->dma_rx_ring = (unsigned int)(np->rx_ring) & 0xfffffff;
	np->dma_tx_ring = (unsigned int)(np->tx_ring) & 0xfffffff;

	np->full_duplex = 1;
	np->link_state = 1;

	spin_lock_init(&np->lock);

	ether_setup(dev); // ?? the function has already been called in alloc_etherdev
	dev->irq = IRQ_ETH;
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	dev->open = jz_eth_open;
	dev->stop = jz_eth_close;
	dev->hard_start_xmit = jz_eth_send_packet;
	dev->get_stats = jz_eth_get_stats;
	dev->set_multicast_list = jz_set_multicast_list;
	dev->do_ioctl = jz_eth_ioctl;
	dev->tx_timeout = jz_eth_tx_timeout;

#else
	dev->netdev_ops = &jz_netdev_ops; // Cynthia zhzhao added 20100514
#endif

	dev->watchdog_timeo = ETH_TX_TIMEOUT;

	// configure MAC address
	get_mac_address(dev);

	if ((err = register_netdev(dev)) != 0) {
		printk("%s: Cannot register net device, error %d\n",
				DRV_NAME, err);
		free_netdev(dev);
		return -ENOMEM;
	}

#if 0 // CONFIG_PM
	np->pmdev = pm_register(PM_SYS_DEV, PM_SYS_UNKNOWN, jz_eth_pm_callback);
	if (np->pmdev) {
		np->pmdev->data = dev;
	}
#endif

	return 0;
}

static void __exit jz_eth_exit(void)
{
	struct net_device *dev = netdev;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	struct jz_eth_private *np = (struct jz_eth_private *)dev->priv;
#else
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(dev));
#endif

	unregister_netdev(dev);
	dma_free_noncoherent(NULL, NUM_RX_DESCS * RX_BUF_SIZE,
			     (void *)np->vaddr_rx_buf, np->dma_rx_buf);
	free_netdev(dev);
}	

module_init(jz_eth_init);
module_exit(jz_eth_exit);

#if 0
void eth_debug_show_priv_at_oops(void)
{
	printk("()()()()()()()()()()()()\n");
	printk("netdev_priv(dev) = 0x%08x\n", (unsigned int)netdev_priv(netdev));
	printk("()()()()()()()()()()()()\n");
}

void eth_debug_at_oops(void)
{
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(netdev));
	printk("=-=-=-=-=-=-=-=-=-=-=-=-=-=-= np = 0x%08x >>>\n", (unsigned int)np);
	dump_eth_tx_addr();
	desc_dump(np);
	priv_data_dump(np);
	printk("=-=-=-=-=-=-=-=-=-=-=-=-=-=-= <<<\n");
	//dma_regs_dump("--------- dump dma regs\n");
}

void eth_debug_show_status(void)
{
	struct jz_eth_private *np = (struct jz_eth_private *)P2ADDR(netdev_priv(netdev));
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printk("tx_packet = %u, tx_bytes = %u\n", np->stats.tx_packets, np->stats.tx_bytes);
	printk("rx_packet = %u, rx_bytes = %u\n", np->stats.rx_packets, np->stats.rx_bytes);
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

EXPORT_SYMBOL(eth_debug_show_priv_at_oops);
EXPORT_SYMBOL(eth_debug_at_oops);
EXPORT_SYMBOL(eth_debug_show_status);
#endif
