/*
 * jz_tssi.c
 *
 * MPEG2-TS interface driver for the Ingenic JZ47XX.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/jzsoc.h>
#include "jzchars.h"

#include "jz_tssi.h"


MODULE_AUTHOR("Lucifer Liu <yliu@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic MPEG2-TS interface Driver");
MODULE_LICENSE("GPL");

#define TSSI_NAME "tssi"
#define TSSI_MINOR 204         /* MAJOR: 10, MINOR: 16 */
#define TSSI_IRQ   IRQ_TSSI
#define PFX        TSSI_NAME
#define RING_BUF_NUM 4
#define OUT_BUF_LEN	10	/* 4M */
#define IN_BUF_LEN	9	/* 2^9*4K=2M */

//#define JZ_TSSI_DEBUG
#define DEBUG 0

#ifdef JZ_TSSISI_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG PFX ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR PFX ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format "\n" , ## arg)

static struct jz_tssi_t jz_tssi_g;
static struct jz_tssi_buf_ring jz_tssi_ring_g;
static struct jz_tssi_desc_t g_tssi_desc0;
static struct jz_tssi_desc_t g_tssi_desc1;
static unsigned char *in_buf0, *in_buf1;
static unsigned char *out_buf;

static void dump_tssi_regs( void )
{
        printk("REG_TSSI_ENA   %8x \n", REG_TSSI_ENA);
        printk("REG_TSSI_NUM   %8x \n", REG_TSSI_NUM);
        printk("REG_TSSI_DTR   %8x \n", REG_TSSI_DTR);
        printk("REG_TSSI_CFG   %8x \n", REG_TSSI_CFG);
        printk("REG_TSSI_CTRL  %8x \n", REG_TSSI_CTRL);
        printk("REG_TSSI_STAT  %8x \n", REG_TSSI_STAT);
        printk("REG_TSSI_FIFO  %8x \n", REG_TSSI_FIFO);
        printk("REG_TSSI_PEN   %8x \n", REG_TSSI_PEN);
        printk("REG_TSSI_PID0  %8x \n", REG_TSSI_PID0);
        printk("REG_TSSI_PID1  %8x \n", REG_TSSI_PID1);
        printk("REG_TSSI_PID2  %8x \n", REG_TSSI_PID2);
        printk("REG_TSSI_PID3  %8x \n", REG_TSSI_PID3);
        printk("REG_TSSI_PID4  %8x \n", REG_TSSI_PID4);
        printk("REG_TSSI_PID5  %8x \n", REG_TSSI_PID5);
        printk("REG_TSSI_PID6  %8x \n", REG_TSSI_PID6);
        printk("REG_TSSI_PID7  %8x \n", REG_TSSI_PID7);
}

static void tssi_free_buf(struct jz_tssi_buf_ring * ring)
{
        int i;
        struct jz_tssi_buf * ap;
        for ( i = 0; i < RING_BUF_NUM; i++) {
		ap = ring->front;
                ring->front = ring->front->next;
                kfree(ap);
        }
}

static void tssi_config_filting( void )
{
	__gpio_as_tssi();
	__tssi_disable_ctrl_irq();
	__tssi_dma_enable();
	__tssi_set_tigger_num(96);        //trig is 4 word!
	__tssi_filter_disable_pid0();
	__tssi_filter_enable();
//	__tssi_filter_disable();
	__tssi_set_wd_1();
	__tssi_set_data_pola_high();
	__tssi_select_paral_mode();
//	__tssi_select_clk_fast();
	__tssi_select_clk_slow(); 
	REG_TSSI_CTRL = 7;

#if 1
/* no add data 0 */
	REG_TSSI_CFG &= ~(1 << 10);
	REG_TSSI_CFG |= (2 << 10);
#endif




	__tssi_select_clk_posi_edge();
	__tssi_select_frm_act_high();
	__tssi_select_str_act_high();
	__tssi_select_fail_act_high();
//	__tssi_select_fail_act_low();
}

static void tssi_add_pid(int pid_num, int pid)
{
	unsigned int addr ;
	int n = pid_num / 2, hl = pid_num % 2;
	if ( hl )      //use high pid, pid1
	{ 
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= ( (pid & 0x1fff) << 16 );    //13bit
		REG_TSSI_PEN |= ( 1 << (16 + n) );
	}
	else           //use low  pid, pid0
	{
		addr = TSSI_PID0 + ( n * 4 );
		REG32( addr ) |= pid & 0x1fff;    //13bit
		REG_TSSI_PEN |= ( 1 << n  );
	}
}

static irqreturn_t tssi_interrupt(int irq, void * dev_id)
{
	unsigned char *tmp;
	int num = REG_TSSI_NUM;
	int did = (REG_TSSI_DST & TSSI_DST_DID_MASK) >> TSSI_DST_DID_BIT;
	

	struct jz_tssi_t *tssi = (struct jz_tssi_t *)dev_id;
	struct jz_tssi_buf_ring *cur_buf = tssi->cur_buf;
	struct jz_tssi_desc_t *tssi_desc0 = &g_tssi_desc0;
	struct jz_tssi_desc_t *tssi_desc1 = &g_tssi_desc1;

	__intc_mask_irq(TSSI_IRQ);

	__tssi_clear_desc_end_flag();

	if (REG_TSSI_STAT & TSSI_STAT_OVRN) {
		printk("tssi over run occur! %x, num = %d\n",REG8( TSSI_STAT ), num);
		__tssi_clear_state();
	}

#if 1
	/* exchange the in_buf0/1 <=> cur_buf->front->buf */
	if (did == 0) {
		tmp = cur_buf->front->buf;
		cur_buf->front->buf = in_buf0;
		in_buf0 = tmp;	
		tssi_desc0->dst_addr = (unsigned int)virt_to_phys((void *)in_buf0);
		dma_cache_wback((unsigned int)(&tssi_desc0), sizeof(struct jz_tssi_desc_t));
		
	} else if (did == 1) {
		tmp = cur_buf->front->buf;
		cur_buf->front->buf = in_buf1;
		in_buf1 = tmp;	
		tssi_desc1->dst_addr = (unsigned int)virt_to_phys((void *)in_buf1);
		dma_cache_wback((unsigned int)(&tssi_desc1), sizeof(struct jz_tssi_desc_t));
	} else {
		printk("DMA Transfer fault, no souch did value: %d\n", did);
		__intc_ack_irq(TSSI_IRQ);
		__intc_unmask_irq(TSSI_IRQ);

		return IRQ_HANDLED;
	}
#endif

	cur_buf->front = cur_buf->front->next;
	cur_buf->fu_num += 1;

	printk("num = %d, did = %d\n", cur_buf->fu_num, did);

	if (cur_buf->fu_num == 1)
		wake_up(&tssi->wait);

/* used for test */
#if DEBUG
/* it will be over run the buf */
	if (cur_buf->fu_num == 5) {
		__tssi_dma_enable();
		__tssi_disable();
		__intc_ack_irq(TSSI_IRQ);
		__intc_unmask_irq(TSSI_IRQ);
		return IRQ_HANDLED;
	}
#endif

	__intc_ack_irq(TSSI_IRQ);
	__intc_unmask_irq(TSSI_IRQ);

	return IRQ_HANDLED;
}
static ssize_t jz_read(struct file *filp, char *buffer, size_t count, loff_t *ppos)
{
	jz_char_dev_t *adev = (jz_char_dev_t *)filp->private_data;
        struct jz_tssi_t* tssi = (struct jz_tssi_t*)adev->private;
        struct jz_tssi_buf_ring* ring = tssi->cur_buf;
	int i;
	unsigned long flags;

	count /= MPEG2_PACKET_SIZE;

	wait_event_interruptible(tssi->wait, ring->fu_num != 0);

	spin_lock_irqsave(&tssi->lock, flags);
	if (count > ring->fu_num) {
		count = ring->fu_num;
	}
	spin_unlock_irqrestore(&tssi->lock, flags);

	for (i = 0; i < count; i++) {
#if DEBUG
	int j;
		copy_to_user(buffer+i*MPEG2_PACKET_SIZE, in_buf0, MPEG2_PACKET_SIZE);
		for (j = 0; j < MPEG2_PACKET_SIZE; j++) {
			if (j % 192 == 0) {
				printk("\n\n");
			}
		//	printk(" %02x ", ring->rear->buf[j]);
			printk("%02x ", in_buf0[j]);
		}
#endif
		copy_to_user(buffer+i*MPEG2_PACKET_SIZE, ring->rear->buf, MPEG2_PACKET_SIZE);
		ring->rear = ring->rear->next;
		ring->rear->pos = 0;
	}

	spin_lock_irqsave(&tssi->lock, flags);
	ring->fu_num -= count;
	spin_unlock_irqrestore(&tssi->lock, flags);
	

	return count*MPEG2_PACKET_SIZE;
}

static int jz_open(struct inode * inode, struct file * filp)
{
	try_module_get(THIS_MODULE);

	__intc_mask_irq(TSSI_IRQ);
	tssi_config_filting();
	__tssi_soft_reset();
	__tssi_clear_state();

	return 0;
}

static int jz_release(struct inode * inode, struct file * filp)
{
	__intc_mask_irq(TSSI_IRQ);
 	module_put(THIS_MODULE);
	return 0;
}

static int jz_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	jz_char_dev_t *adev = (jz_char_dev_t *)file->private_data;
	struct jz_tssi_t* tssi = (struct jz_tssi_t*)adev->private;

	switch (cmd)
	{
	case IOCTL_TSSI_ENABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();
		dump_tssi_regs();
		__intc_ack_irq(TSSI_IRQ);
		__intc_unmask_irq(TSSI_IRQ);
		__tssi_enable();

        	break;
	case IOCTL_TSSI_DISABLE :
		__tssi_disable();
		__tssi_soft_reset();
		__tssi_clear_state();

        	break;
	case IOCTL_TSSI_SOFTRESET :
		__tssi_soft_reset();

        	break;
	case IOCTL_TSSI_ENFILTER :
		__tssi_filter_enable();
        	break;
	case IOCTL_TSSI_DEFILTER :
		__tssi_filter_disable();
        	break;
	case IOCTL_TSSI_ADDPID :           //add one pid to filter
		if ( tssi->pid_num < 31 )
		{
			tssi_add_pid(tssi->pid_num, arg);
			tssi->pid_num ++ ;
		}
		break;

	case IOCTL_TSSI_FLUSHPID :               //set all filting pid to false
		REG_TSSI_PEN = 0x0;		
		REG_TSSI_PID0 = 0x0;
		REG_TSSI_PID1 = 0x0;
		REG_TSSI_PID2 = 0x0;
		REG_TSSI_PID3 = 0x0;
		REG_TSSI_PID4 = 0x0;
		REG_TSSI_PID5 = 0x0;
		REG_TSSI_PID6 = 0x0;
		REG_TSSI_PID7 = 0x0;
		break;

	case IOCTL_TSSI_INIT_DMA:
		break;
	case IOCTL_TSSI_DISABLE_DMA:
		break;
	}

	return 0;
}

static struct file_operations tssi_fops = {
	.owner	= THIS_MODULE,
	.read	= jz_read,
	.poll	= NULL,
	.fasync	= NULL,
	.ioctl	= jz_ioctl,
	.open	= jz_open,
//	.mmap	= jz_mmap,
	.release= jz_release,
};

static void tssi_dma_desc_init(void)
{
	struct jz_tssi_desc_t *tssi_desc0 = &g_tssi_desc0;
	struct jz_tssi_desc_t *tssi_desc1 = &g_tssi_desc1;
	
	tssi_desc0->next_desc = (unsigned int)virt_to_phys((void *)tssi_desc1);
	tssi_desc0->dst_addr = (unsigned int)virt_to_phys((void *)in_buf0);
	tssi_desc0->did = 0;

	 /* TLEN:1Mbytes TEFE:1 TSZ:32 TEIE:1 LINK:0 */
	tssi_desc0->cmd = ((MPEG2_PACKET_SIZE/4) << TSSI_DCMD_TLEN_BIT) | TSSI_DCMD_TEFE | TSSI_DCMD_TSZ_32 |
		  TSSI_DCMD_TEIE | TSSI_DCMD_LINK;


	tssi_desc1->next_desc = (unsigned int)virt_to_phys((void *)tssi_desc0);
	tssi_desc1->dst_addr = (unsigned int)virt_to_phys((void *)in_buf1);
	tssi_desc1->did = 1;

	 /* TLEN:1Mbytes TEFE:1 TSZ:32 TEIE:1 LINK:1 */
	tssi_desc1->cmd = ((MPEG2_PACKET_SIZE/4) << TSSI_DCMD_TLEN_BIT) | TSSI_DCMD_TEFE | TSSI_DCMD_TSZ_32 |
			  TSSI_DCMD_TEIE | TSSI_DCMD_LINK;

	REG_TSSI_DDA = (unsigned int)virt_to_phys((void*)tssi_desc0);
	dma_cache_wback((unsigned int)(&tssi_desc0), sizeof(struct jz_tssi_desc_t));
	dma_cache_wback((unsigned int)(&tssi_desc1), sizeof(struct jz_tssi_desc_t));
	
}
static int tssi_buf_init(struct jz_tssi_buf_ring * ring)
{
	int i;
	struct jz_tssi_buf *ap, *bp, *cp;

	/* used for dma desc0 and desc1 transfer buf total 2M=2^9*4K */
	in_buf0 = (unsigned char*)__get_free_pages(GFP_KERNEL, IN_BUF_LEN);
	if (!in_buf0) {
		printk("Alloc in_buf memory failed.\n");
		return -ENOMEM;
	}

	in_buf1 = in_buf0 + (PAGE_SIZE << (IN_BUF_LEN - 1));
	memset(in_buf0, 0, PAGE_SIZE << IN_BUF_LEN);

	/* used for save the data for the user space */
	out_buf = (unsigned char*)__get_free_pages(GFP_KERNEL, OUT_BUF_LEN);
	if (!out_buf) {
		printk("Alloc out_buf memory failed.\n");
		return -ENOMEM;
	}
	memset(out_buf, 0, PAGE_SIZE << OUT_BUF_LEN);
	
	ap = bp = cp = (struct jz_tssi_buf*)kmalloc(sizeof(struct jz_tssi_buf), GFP_KERNEL);
	if (!ap) {
		printk("Alloc tssi buf memory failed.\n");
		return -ENOMEM;
	}
	
	for (i = 0; i < RING_BUF_NUM; i++) {
		bp = ap;
		ap->buf = out_buf + i * (PAGE_SIZE << (OUT_BUF_LEN -2));

		ap = (struct jz_tssi_buf*)kmalloc(sizeof(struct jz_tssi_buf), GFP_KERNEL);
		if (!ap) {
			printk("Alloc the %dth buf ring failed.\n", i);
			return -ENOMEM;
		}

		bp->pos = 0;
		bp->next = ap;
	}

	bp->next = cp;
	ring->front = cp;
	ring->rear = cp;
	ring->fu_num = 0;
	kfree(ap);	

	return 0;
}

static int __init jztssi_init_module(void)
{
	int retval;
	struct jz_tssi_t *tssi = &jz_tssi_g;

//	cpm_start_clock(CGM_TSSI);

	retval = tssi_buf_init(&jz_tssi_ring_g);
	if (retval) {
		printk("tssi buf init failed.\n");
		return -1;
	}
	
	tssi->cur_buf = &jz_tssi_ring_g;
	tssi->pid_num = 0;

	tssi_dma_desc_init();

	spin_lock_init(&tssi->lock);
	init_waitqueue_head(&tssi->wait);

	retval = request_irq(TSSI_IRQ, tssi_interrupt, IRQF_DISABLED, TSSI_NAME, (void*)&jz_tssi_g);
	if (retval) {
		printk("unable to get IRQ %d",TSSI_IRQ);
		return retval;
	}

	jz_register_chrdev(TSSI_MINOR, TSSI_NAME, &tssi_fops, (void *)&jz_tssi_g);	

	printk(JZ_SOC_NAME": MPEG2-TS interface driver registered %x\n",(unsigned int)&jz_tssi_g);
	return 0;
}

static void jztssi_cleanup_module(void)
{
	free_irq(TSSI_IRQ,0);
	tssi_free_buf(&jz_tssi_ring_g);
	kfree(in_buf0);
	kfree(out_buf);
	jz_unregister_chrdev(TSSI_MINOR, TSSI_NAME);
}

module_init(jztssi_init_module);
module_exit(jztssi_cleanup_module);
