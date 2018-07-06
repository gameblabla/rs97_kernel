/*
 *  linux/drivers/mmc/host/jz_mmc/jz_mmc_main.c - JZ SD/MMC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (c) Ingenic Semiconductor Co., Ltd.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/card.h>
#include <linux/mm.h>
#include <linux/signal.h>
#include <linux/pm.h>
#include <linux/scatterlist.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/jzsoc.h>
#include "include/jz_mmc_host.h"
#include "include/jz_mmc_controller.h"

/* for Atheros wifi */
int is_virt_addr_valid(unsigned char *buffer) {
	return virt_addr_valid(buffer);
}
EXPORT_SYMBOL(is_virt_addr_valid);

struct jz_mmc_controller controller[JZ_MAX_MSC_NUM];
int is_permission = 0;

int jz_mmc_get_permission(struct mmc_host *mmc, struct mmc_request *mrq){
	int sector,up_limit,down_limit;

	if (!mmc_card_blockaddr(mmc->card)){
		up_limit = 16384 << 9 ;
		down_limit = 512;
	}
	else{
		up_limit = 16384 ;
		down_limit = 1;
	}
	sector = mrq->cmd->arg;

	if(sector>down_limit && sector<up_limit){

		return is_permission;
	}else{
		return 1;
	}
}

/* add partitions info for recovery */
#ifdef CONFIG_JZ_RECOVERY_SUPPORT
static ssize_t jz_mmc_partitions_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	int i;
	struct jz_mmc_platform_data *pdata = dev->platform_data;
	ssize_t count = 0;

	if(pdata->num_partitions == 0) {
		count = sprintf(buf, "null\n");
		return count;
	}

	for(i=0;i<pdata->num_partitions;i++)
		count += sprintf(buf+count, "%s %x %x %d\n",
				pdata->partitions[i].name,
				pdata->partitions[i].saddr,
				pdata->partitions[i].len,
				pdata->partitions[i].type);

	return count;
}


static ssize_t jz_mmc_permission_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct jz_mmc_platform_data *pdata = dev->platform_data;

	if (buf == NULL)
		return count;

	if (strcmp(buf, "RECOVERY_MODE") == 0) {
		printk("host->permission: MMC_BOOT_AREA_PROTECTED->MMC_BOOT_AREA_OPENED\n");
		pdata->permission = MMC_BOOT_AREA_OPENED;
	} else {
		printk("host->permission: MMC_BOOT_AREA_OPENED->MMC_BOOT_AREA_PROTECTED\n");
		pdata->permission = MMC_BOOT_AREA_PROTECTED;
	}

	return count;
}

static DEVICE_ATTR(partitions, S_IRUSR | S_IRGRP | S_IROTH, jz_mmc_partitions_show, NULL);
static DEVICE_ATTR(recovery_permission, S_IWUSR, NULL, jz_mmc_permission_set);

static struct attribute *jz_mmc_attributes[] = {
	&dev_attr_partitions.attr,
	&dev_attr_recovery_permission.attr,
	NULL
};

static const struct attribute_group jz_mmc_attr_group = {
	.attrs = jz_mmc_attributes,
};
#endif

void jz_mmc_finish_request(struct jz_mmc_host *host, struct mmc_request *mrq)
{
	host->curr_mrq = NULL;
	up(&host->mutex);
	mmc_request_done(host->mmc, mrq);
}

static void jz_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	struct jz_mmc_functions *functions = host->plat->driver_data;

	down(&host->mutex);

	if (SD_IO_SEND_OP_COND == mrq->cmd->opcode) {
		if(host->plat->support_sdio == 0) {
			mrq->cmd->error = -ETIMEDOUT;
			jz_mmc_finish_request(host, mrq);
			return;
		}
	}

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = -EIO;
			mrq->data->bytes_xfered = 0;
		} else
			mrq->cmd->error = -ENOMEDIUM;
		up(&host->mutex);
		mmc_request_done(mmc, mrq);
		return;
	}

#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
	if(host->pdev_id == 0 ){
		if (mrq->data && (mrq->data->flags & MMC_DATA_WRITE)){
			if(!jz_mmc_get_permission(mmc, mrq) && mrq->cmd->opcode != 6) {
				mrq->cmd->error = -EIO;
				mrq->data->bytes_xfered = 0;

					up(&host->mutex);
					mmc_request_done(mmc, mrq);
					return;
			}
		}
	}
#endif

	BUG_ON (host->curr_mrq);
	host->curr_mrq = mrq;
	functions->execute_cmd(host);
	jz_mmc_finish_request(host, mrq);
}

static int jz_mmc_get_ro(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->write_protect != NULL)
		return host->plat->write_protect(mmc_dev(host->mmc));
	else
		return 0;
}

static int jz_mmc_get_cd(struct mmc_host *mmc)
{
	struct jz_mmc_host *host = mmc_priv(mmc);

	if(host->plat->status != NULL) {
		return host->plat->status(mmc_dev(host->mmc));
	}
	else
		return 1;
}

/* set clock and power */
static void jz_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct jz_mmc_host *host = mmc_priv(mmc);
	struct jz_mmc_functions *functions = host->plat->driver_data;
	//void *dev;

	if(!functions) {
//		printk("%s: functions is NULL!\n", __FUNCTION__);
		while(1);
	}

	if (ios->clock) {
		functions->set_clock(host, ios->clock);
	}

	switch(ios->power_mode) {
	case MMC_POWER_ON:
		host->plat->power_on(NULL);
		host->cmdat |= MSC_CMDAT_INIT;
		break;
	case MMC_POWER_OFF:
		host->plat->power_off(NULL);
		break;
	default:
		break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 4)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_4BIT;
		else
			host->cmdat |= host->plat->bus_width;
	} else if (ios->bus_width == MMC_BUS_WIDTH_8) {

		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_MASK;

		if(host->plat->bus_width == 8)
			host->cmdat |= MSC_CMDAT_BUS_WIDTH_8BIT;
//		else
//			host->cmdat |= host->plat->bus_width;
	} else {
		/* 1 bit bus*/
		host->cmdat &= ~MSC_CMDAT_BUS_WIDTH_8BIT;
	}
}

static const struct mmc_host_ops jz_mmc_ops = {
	.request = jz_mmc_request,
	.get_ro = jz_mmc_get_ro,
	.set_ios = jz_mmc_set_ios,
	.get_cd = jz_mmc_get_cd,
};

#ifdef MSC_DEBUG_DMA
static struct jz_mmc_host *msc_hosts[JZ_MAX_MSC_NUM] = { NULL, NULL, NULL };

static void dump_host_info(struct jz_mmc_host *host) {
	int i = 0;
	JZ_MSC_DMA_DESC *desc = NULL;

	printk("*** msc%d host info ***\n", host->pdev_id);
	dump_jz_dma_channel(host->dma.channel);
	printk("*** last running descriptors = %d direction = %d ***\n", host->num_desc, host->last_direction);
	desc = host->dma_desc;
	for (i = 0; i < host->num_desc; i++) {
		printk("desc address = %p\n", desc + i);
		printk("dcmd = 0x%08x\n", desc[i].dcmd);
		printk("dsadr = 0x%08x\n", desc[i].dsadr);
		printk("dtadr = 0x%08x\n", desc[i].dtadr);
		printk("ddadr = 0x%08x\n", desc[i].ddadr);
		printk("dstrd = 0x%08x\n", desc[i].dstrd);
		printk("dreqt = 0x%08x\n", desc[i].dreqt);
		printk("resv0 = 0x%08x\n", desc[i].reserved0);
		printk("resv1 = 0x%08x\n", desc[i].reserved1);
		printk("==========\n");
	}

	printk("curr tx_ack = %d\n", host->tx_ack);
	printk("curr rx_ack = %d\n", host->rx_ack);
}

void msc_dump_host_info(void) {
	int i = 0;

	for (i = 0; i < JZ_MAX_MSC_NUM; i++) {
		if (msc_hosts[i] != NULL) {
			dump_host_info(msc_hosts[0]);
		}
	}
}
EXPORT_SYMBOL(msc_dump_host_info);
#endif	/* MSC_DEBUG_DMA */

#if defined(CONFIG_JZ_RECOVERY_SUPPORT) || defined(CONFIG_JZ_SYSTEM_AT_CARD)
static ssize_t mmc_permission_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_permission);
}

static ssize_t mmc_permission_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int permis;

	permis = simple_strtol(buf, NULL, 10);
	is_permission = permis;
	printk("%d\n",is_permission);
	return count;
}

static DEVICE_ATTR(permission, S_IWUSR | S_IRUGO, mmc_permission_show, mmc_permission_store);
#endif

static int jz_mmc_probe(struct platform_device *pdev)
{
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct jz_mmc_host *host = NULL;
	struct jz_mmc_functions *functions;

	struct resource *irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int i;

	if (pdev == NULL) {
		printk(KERN_ERR "%s: pdev is NULL\n", __func__);
		return -EINVAL;
	}
	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		return -EINVAL;
	}

	if (JZ_MSC_ID_INVALID(pdev->id))
		return -EINVAL;

	plat->cpm_start(&pdev->dev);

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}
	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM)
			memres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_IRQ)
			irqres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}
	if (!irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}
	/*
	 * Setup our host structure
	 */
	mmc = mmc_alloc_host(sizeof(struct jz_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}
	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
#if 0			      /* Lutts */
	// base address of MSC controller
	host->base = ioremap(memres->start, PAGE_SIZE);
	if (!host->base) {
		return -ENOMEM;
	}
#endif
	host->irq = irqres->start;
	if (dmares)
		host->dma_id = dmares->start;
	else
		host->dma_id = -1;
	//spin_lock_init(&host->lock);
	init_MUTEX(&host->mutex);

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &jz_mmc_ops;
	mmc->f_min = MMC_CLOCK_SLOW;
	mmc->f_max = SD_CLOCK_HIGH;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->caps |= host->plat->max_bus_width;
	mmc->max_phys_segs = NR_SG;
	mmc->max_blk_size = 4095;
	mmc->max_blk_count = 65535;

	mmc->max_req_size = PAGE_SIZE * 16;
	mmc->max_seg_size = mmc->max_req_size;
	plat->init(&pdev->dev);
	plat->power_on(&pdev->dev);
	/*
	 * Initialize controller and register some functions
	 * From here, we can do everything!
	 */
	controller_register(&controller[host->pdev_id], host);
	functions = host->plat->driver_data;
	if(controller[host->pdev_id].init(&controller[host->pdev_id], host, pdev))
		goto out;
	mmc_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);
#ifdef MSC_DEBUG_DMA
	msc_hosts[host->pdev_id] = host;
#endif

	if(host->pdev_id == 0){
#if defined(CONFIG_JZ_SYSTEM_AT_CARD)
		if(device_create_file(&pdev->dev, &dev_attr_permission))
			printk("MSC0: device_create_file for attr_permission failed!\n");;
#endif
	}
#ifdef CONFIG_JZ_RECOVERY_SUPPORT
	sysfs_create_group(&pdev->dev.kobj, &jz_mmc_attr_group);
#endif
	printk("JZ %s driver registered\n", mmc_hostname(host->mmc));

	return 0;

out:
	return -1;
}

static int jz_mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct jz_mmc_platform_data *plat = pdev->dev.platform_data;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		struct jz_mmc_host *host = mmc_priv(mmc);
		struct jz_mmc_functions *functions = host->plat->driver_data;

		plat->power_off(&pdev->dev);

		functions->deinit(host, pdev);

		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
static int jz_mmc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct jz_mmc_host *host = mmc_priv(mmc);
	int ret = 0;

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	if (host->pdev_id == 0)
		return 0;
#endif

	host->sleeping = 1;

	if (mmc) {
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO) {
			ret = mmc_suspend_host(mmc, state);
		}

	}
	return ret;
}

extern int jz_mmc_detect(struct jz_mmc_host *host, int from_resuming);
static int jz_mmc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	struct jz_mmc_host *host = mmc_priv(mmc);

#ifdef CONFIG_JZ_SYSTEM_AT_CARD
	if (host->pdev_id == 0)
		return 0;
#endif

	if (mmc) {
		if ( (mmc->card == NULL) || (mmc->card->type != MMC_TYPE_SDIO) )
			jz_mmc_detect(host, 1);
	}

	return 0;
}
#else
#define jz_mmc_suspend      NULL
#define jz_mmc_resume       NULL
#endif

static struct platform_driver jz_msc_driver = {
	.probe = jz_mmc_probe,
	.remove = jz_mmc_remove,
	.suspend = jz_mmc_suspend,
	.resume = jz_mmc_resume,
	.driver = {
		   .name = "jz-msc",
		   },
};

static int __init jz_mmc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&jz_msc_driver);
	return ret;
}

static void __exit jz_mmc_exit(void)
{
	platform_driver_unregister(&jz_msc_driver);
}

subsys_initcall(jz_mmc_init);
module_exit(jz_mmc_exit);

MODULE_DESCRIPTION("JZ47XX SD/Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
