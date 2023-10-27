/*
 * drivers/mmc/host/gk_sdio2_v1_00.c
 *
 * Author: louis liang<liangyou@gokemicro.com>
 * Copyright (C) 2014-2015, goke, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include <asm/dma.h>

#include <mach/hardware.h>
#include <mach/sd.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/event.h>
#include CONFIG_GK_CHIP_INCLUDE_FILE
#include <plat/cache.h>
#if defined(CONFIG_SYSTEM_USE_NETLINK_MSG_SD)
#include <plat/gk_net_link.h>
#endif
#define COLOR_GREEN  "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED    "\033[31m"
#define COLOR_NONE   "\033[0m"

/*----------------------------------------------------------------------------*/
/* registers base:       GK_VA_SDC                                        */
/*----------------------------------------------------------------------------*/


/* ==========================================================================*/

/* ==========================================================================*/

/* ==========================================================================*/
/* ==========================================================================*/
#define CONFIG_SD_GK_TIMEOUT_VAL        (0xe)
#define CONFIG_SD_GK_WAIT_TIMEOUT       (HZ / 100)
#define CONFIG_SD_GK_WAIT_COUNTER_LIMIT (1000)
#define CONFIG_SD_GK_DEFAULT_CLOCK      (46000000)
#define CONFIG_SD_GK_MAX_TIMEOUT        (10 * HZ)

//#define CONFIG_SD_GK_DEBUG
#undef CONFIG_SD_GK_DEBUG_VERBOSE

#define gk_sd_printk(level, phcinfo, format, arg...)    \
    printk("-- %s.%u: " format,            \
    dev_name(((struct gk_sd_controller_info *)phcinfo->pinfo)->dev),\
    phcinfo->slot_id, ## arg)

#define gk_sd_err(phcinfo, format, arg...)        \
    gk_sd_printk(KERN_ERR, phcinfo, format, ## arg)
#define gk_sd_warn(phcinfo, format, arg...)        \
    gk_sd_printk(KERN_WARNING, phcinfo, format, ## arg)
#define gk_sd_info(phcinfo, format, arg...)        \
    gk_sd_printk(KERN_INFO, phcinfo, format, ## arg)
#define gk_sd_rtdbg(phcinfo, format, arg...)        \
    gk_sd_printk(KERN_DEBUG, phcinfo, format, ## arg)

#ifdef CONFIG_SD_GK_DEBUG
#define gk_sd_dbg(phcinfo, format, arg...)        \
    gk_sd_printk(KERN_DEBUG, phcinfo, format, ## arg)
#else
#define gk_sd_dbg(phcinfo, format, arg...)        \
    ({ if (0) gk_sd_printk(KERN_DEBUG, phcinfo, format, ##arg); 0; })
#endif

#define INFO_NUM (3)
#define SD_MIN_BUF (2048)
/* ==========================================================================*/
enum gk_sd_state {
    GK_SD_STATE_IDLE,
    GK_SD_STATE_CMD,
    GK_SD_STATE_DATA,
    GK_SD_STATE_RESET,
    GK_SD_STATE_ERR
};

#define REG_WR(reg,value)   (*((volatile u32 *)(reg))=(value))
#define	GK_SD_PEND_DTO_b	(0)
#define	GK_SD_PEND_DTO_m	(1 << GK_SD_PEND_DTO_b)

struct gk_sd_mmc_info {
    struct mmc_host         *mmc;
    struct mmc_request      *mrq;

    wait_queue_head_t       wait;
    unsigned long       pending_events;
    spinlock_t      lock;

    enum gk_sd_state    state;

    struct scatterlist      *sg;
    u32                 sg_len;
    u32                 wait_tmo;
    u16                 blk_sz;
    u16                 blk_cnt;
    u32                 arg_reg;
    u16                 xfr_reg;
    u16                 cmd_reg;
    u16                 sta_reg;
    u8                  tmo;
    u8                  use_adma;
    u32                 sta_counter;

    char                *buf_vaddress;
    dma_addr_t          buf_paddress;
    u32                 dma_address;
    u32                 dma_size;

    void                (*pre_dma)(void *data);
    void                (*post_dma)(void *data);

    struct gk_sd_slot    *plat_info;
    u32                 slot_id;
    void                *pinfo;
    u32                 valid;

    u32                 pre_post_used;
    struct notifier_block   system_event;
    struct semaphore        system_event_sem;
};

struct gk_sd_controller_info {
    unsigned char __iomem   *regbase;
    struct device           *dev;
    unsigned int            irq;
    u32                 dma_fix;
    u32                 clk_limit;
    u32                 reset_error;
    u32                 max_blk_sz;
    int                 id;
    u32                 boot_media;
    struct kmem_cache   *buf_cache;

    struct gk_sd_controller *pcontroller;
    struct gk_sd_mmc_info   *pslotinfo[GK_SD_MAX_SLOT_NUM];
    struct gk_sd_mmc_info   *pslotinfo_cur;
    struct mmc_ios              controller_ios;
    struct semaphore        pslotinfo_sem;
};
static void gk_sd_post_req(struct mmc_host *host, struct mmc_request *req,int err);
static inline void gk_sd_pre_cmd(struct gk_sd_mmc_info *pslotinfo);

#define CONFIG_GK_MMC_CMD_USE_PIO
/* ==========================================================================*/
#ifdef CONFIG_SD_GK_DEBUG_VERBOSE
static void gk_sd_show_info(struct gk_sd_mmc_info *pslotinfo)
{
    gk_sd_dbg(pslotinfo, "Enter %s\n", __func__);
    gk_sd_dbg(pslotinfo, "sg = 0x%x.\n", (u32)pslotinfo->sg);
    gk_sd_dbg(pslotinfo, "sg_len = 0x%x.\n", pslotinfo->sg_len);
    gk_sd_dbg(pslotinfo, "tmo = 0x%x.\n", pslotinfo->tmo);
    gk_sd_dbg(pslotinfo, "blk_sz = 0x%x.\n", pslotinfo->blk_sz);
    gk_sd_dbg(pslotinfo, "blk_cnt = 0x%x.\n", pslotinfo->blk_cnt);
    gk_sd_dbg(pslotinfo, "arg_reg = 0x%x.\n", pslotinfo->arg_reg);
    gk_sd_dbg(pslotinfo, "xfr_reg = 0x%x.\n", pslotinfo->xfr_reg);
    gk_sd_dbg(pslotinfo, "cmd_reg = 0x%x.\n", pslotinfo->cmd_reg);
    gk_sd_dbg(pslotinfo, "buf_vaddress = 0x%x.\n", (u32)pslotinfo->buf_vaddress);
    gk_sd_dbg(pslotinfo, "buf_paddress = 0x%x.\n", pslotinfo->buf_paddress);
    gk_sd_dbg(pslotinfo, "dma_address = 0x%x.\n", pslotinfo->dma_address);
    gk_sd_dbg(pslotinfo, "dma_size = 0x%x.\n", pslotinfo->dma_size);
    gk_sd_dbg(pslotinfo, "pre_dma = 0x%x.\n", (u32)pslotinfo->pre_dma);
    gk_sd_dbg(pslotinfo, "post_dma = 0x%x.\n", (u32)pslotinfo->post_dma);
    gk_sd_dbg(pslotinfo, "SD: state = 0x%x.\n", pslotinfo->state);
    gk_sd_dbg(pslotinfo, "Exit %s\n", __func__);
}
#endif


static u32 gk_sd_check_dma_boundary(
    struct gk_sd_mmc_info *pslotinfo,
    u32 address, u32 size, u32 max_size)
{
    u32 start_512kb;
    u32 end_512kb;

    start_512kb = (address) & (~(max_size - 1));
    end_512kb = (address + size - 1) & (~(max_size - 1));

    if (start_512kb != end_512kb) {
        return 0;
    }

    return 1;
}

static u32 gk_sd_dma_mask_to_size(u32 mask)
{
    u32 size;

    switch (mask) {
    case SD_BLK_SZ_512KB:
        size = 0x80000;
        break;
    case SD_BLK_SZ_256KB:
        size = 0x40000;
        break;
    case SD_BLK_SZ_128KB:
        size = 0x20000;
        break;
    case SD_BLK_SZ_64KB:
        size = 0x10000;
        break;
    case SD_BLK_SZ_32KB:
        size = 0x8000;
        break;
    case SD_BLK_SZ_16KB:
        size = 0x4000;
        break;
    case SD_BLK_SZ_8KB:
        size = 0x2000;
        break;
    case SD_BLK_SZ_4KB:
        size = 0x1000;
        break;
    default:
        size = 0;
        BUG_ON(1);
        break;
    }

    return size;
}

static void gk_sd_pre_sg_to_dma(void *data)
{
    struct gk_sd_mmc_info *pslotinfo;
    struct gk_sd_controller_info *pinfo;
    int i;
    //int j;
    u32 offset;
    //u32 *res;

    pslotinfo = (struct gk_sd_mmc_info *)data;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;


    for (i = 0, offset = 0; i < pslotinfo->sg_len; i++) {
        //printk("copy: 0x%0x - 0x%0x len=%d\n", pslotinfo->buf_vaddress + offset,
        //            sg_virt(&pslotinfo->sg[i]), pslotinfo->sg[i].length);
#if 0
        res = (u32*)(pslotinfo->buf_vaddress + offset);
        for(j=0;j<pslotinfo->sg[i].length/4;j++)
            printk("0x%0x \n", res[j]);
        printk("\n");
#endif

        memcpy(pslotinfo->buf_vaddress + offset,
            sg_virt(&pslotinfo->sg[i]),
            pslotinfo->sg[i].length);
        offset += pslotinfo->sg[i].length;
    }

    BUG_ON(offset != pslotinfo->dma_size);
    dma_sync_single_for_device(pinfo->dev, pslotinfo->buf_paddress,
        pslotinfo->dma_size, DMA_TO_DEVICE);
    pslotinfo->dma_address = pslotinfo->buf_paddress;
    pslotinfo->blk_sz |= SD_BLK_SZ_512KB;
}

static void gk_sd_post_sg_to_dma(void *data)
{
    struct gk_sd_mmc_info *pslotinfo;
    struct gk_sd_controller_info *pinfo;


    pslotinfo = (struct gk_sd_mmc_info *)data;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    dma_sync_single_for_cpu(pinfo->dev, pslotinfo->buf_paddress,
        pslotinfo->dma_size, DMA_TO_DEVICE);
}

static void gk_sd_pre_dma_to_sg(void *data)
{
    struct gk_sd_mmc_info *pslotinfo;
    struct gk_sd_controller_info *pinfo;

    pslotinfo = (struct gk_sd_mmc_info *)data;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    dma_sync_single_for_device(pinfo->dev, pslotinfo->buf_paddress,    pslotinfo->dma_size, DMA_FROM_DEVICE);
    //printk("dma_sync_single_for_device: buf=0x%x size=%d\n", pslotinfo->buf_paddress, pslotinfo->dma_size);
    pslotinfo->dma_address = pslotinfo->buf_paddress;
    pslotinfo->blk_sz |= SD_BLK_SZ_512KB;
}

static void gk_sd_post_dma_to_sg(void *data)
{
    struct gk_sd_mmc_info *pslotinfo;
    struct gk_sd_controller_info *pinfo;
    int i;
    //int j;
    //u8 *res;
    u32 offset;

    pslotinfo = (struct gk_sd_mmc_info *)data;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    dma_sync_single_for_cpu(pinfo->dev, pslotinfo->buf_paddress,
        pslotinfo->dma_size, DMA_FROM_DEVICE);
    for (i = 0, offset = 0; i < pslotinfo->sg_len; i++) {

        //printk("copy: 0x%0x - 0x%0x len=%d\n", sg_virt(&pslotinfo->sg[i]),
        //            pslotinfo->buf_vaddress + offset, pslotinfo->sg[i].length);
#if 0
        res = (u8*)(pslotinfo->buf_vaddress + offset);
        for(j=0;j<pslotinfo->sg[i].length;j=j+8)
            printk("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n",
                        res[j], res[j+1], res[j+2], res[j+3],
                        res[j+4], res[j+5], res[j+6], res[j+7]);
        printk("\n");
#endif
        memcpy(sg_virt(&pslotinfo->sg[i]),
            pslotinfo->buf_vaddress + offset,
            pslotinfo->sg[i].length);
        offset += pslotinfo->sg[i].length;
    }
    BUG_ON(offset != pslotinfo->dma_size);
}

static void gk_sd_request_bus(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);

    down(&pslotinfo->system_event_sem);

    if (pslotinfo->plat_info->request)
        pslotinfo->plat_info->request();
}

static void gk_sd_release_bus(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);

    if (pslotinfo->plat_info->release)
        pslotinfo->plat_info->release();

    up(&pslotinfo->system_event_sem);
}

static void gk_sd_enable_int(struct mmc_host *mmc, u32 mask)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);

    pslotinfo->plat_info->set_int(mask, 1);
}

static void gk_sd_disable_int(struct mmc_host *mmc, u32 mask)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);

    pslotinfo->plat_info->set_int(mask, 0);
}

static void gk_sd_set_iclk(struct mmc_host *mmc, u16 clk_div)
{
    u16 clkreg;
    u32 counter = 0;
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;
//    printk("set clock div\n");


#if 1
    clk_div <<= 8;
    clk_div |= SD_CLK_ICLK_EN;
    gk_sd_writew((unsigned int)pinfo->regbase + SD_CLK_OFFSET, clk_div);
    //printk("## 1set clock div = %d \n", clk_div);
    while (1) {
        clkreg = gk_sd_readw((unsigned int)pinfo->regbase + SD_CLK_OFFSET);
        if (clkreg & SD_CLK_ICLK_STABLE)
            break;
        if ((clkreg & ~SD_CLK_ICLK_STABLE) != clk_div) {
            gk_sd_writew((unsigned int)pinfo->regbase + SD_CLK_OFFSET, clk_div);
            //printk("## 2set clock div = %d \n", clk_div);
            udelay(1);
        }
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            gk_sd_warn(pslotinfo,
                "Wait SD_CLK_ICLK_STABLE = %d @ 0x%x\n",
                counter, clkreg);
            break;
        }
    }
#endif
}

static void gk_sd_clear_clken(struct mmc_host *mmc)
{
    u16 clkreg;
    u32 counter = 0;
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    while (1) {
        clkreg = gk_sd_readw((unsigned int)pinfo->regbase + SD_CLK_OFFSET);
        if (clkreg & SD_CLK_EN) {
            gk_sd_writew((unsigned int)pinfo->regbase + SD_CLK_OFFSET,
                (clkreg & ~SD_CLK_EN));
            udelay(1);
        } else {
            break;
        }
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            gk_sd_warn(pslotinfo, "%s(%d @ 0x%x)\n",
                __func__, counter, clkreg);
            break;
        }
    }
}

static void gk_sd_set_clken(struct mmc_host *mmc)
{
    u16 clkreg;
    u32 counter = 0;
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    while (1) {
        clkreg = gk_sd_readw((unsigned int)pinfo->regbase + SD_CLK_OFFSET);
        if (clkreg & SD_CLK_EN) {
            break;
        } else {
            gk_sd_writew((unsigned int)pinfo->regbase + SD_CLK_OFFSET,    (clkreg | SD_CLK_EN));
            //printk("## 3set clock div = %d \n", (clkreg | SD_CLK_EN));
            udelay(1);
        }
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            gk_sd_warn(pslotinfo, "%s(%d @ 0x%x)\n",
                __func__, counter, clkreg);
            break;
        }
    }
}

static void gk_sd_reset_all(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_mmc_info *pslotinfo_next;
    struct gk_sd_controller_info *pinfo;
    u32 nis_flag = 0;
    u32 eis_flag = 0;
    u32 counter = 0;
    u8 reset_reg;

    pslotinfo_next = pslotinfo + 1;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    //printk(pslotinfo, "Enter %s with state %u\n", __func__, pslotinfo->state);

    gk_sd_disable_int(mmc, 0xFFFFFFFF);
    gk_sd_writew((unsigned int)pinfo->regbase + SD_NIS_OFFSET, 0xFFFF);
    gk_sd_writew((unsigned int)pinfo->regbase + SD_EIS_OFFSET, 0xFFFF);
    gk_sd_writeb((unsigned int)pinfo->regbase + SD_RESET_OFFSET, SD_RESET_ALL);
    while (1) {
        reset_reg = gk_sd_readb((unsigned int)pinfo->regbase + SD_RESET_OFFSET);
        if (!(reset_reg & SD_RESET_ALL))
            break;
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            gk_sd_warn(pslotinfo, "Wait SD_RESET_ALL....\n");
            break;
        }
    }

    gk_sd_set_iclk(mmc, 0x0000);
    gk_sd_writeb((unsigned int)pinfo->regbase + SD_TMO_OFFSET,
        CONFIG_SD_GK_TIMEOUT_VAL);

    nis_flag = SD_NISEN_REMOVAL    |
        SD_NISEN_INSERT        |
        SD_NISEN_DMA        |
        SD_NISEN_BLOCK_GAP    |
        SD_NISEN_XFR_DONE    |
        SD_NISEN_CMD_DONE;
    eis_flag = SD_EISEN_ACMD12_ERR    |
        SD_EISEN_CURRENT_ERR    |
        SD_EISEN_DATA_BIT_ERR    |
        SD_EISEN_DATA_CRC_ERR    |
        SD_EISEN_DATA_TMOUT_ERR    |
        SD_EISEN_CMD_IDX_ERR    |
        SD_EISEN_CMD_BIT_ERR    |
        SD_EISEN_CMD_CRC_ERR    |
        SD_EISEN_CMD_TMOUT_ERR;

    gk_sd_enable_int(mmc, (eis_flag << 16) | nis_flag);

#ifdef CONFIG_GK_MMC_CMD_USE_PIO
    //disable cmd comp irq mask why signal enable reg not int enable reg ????
    nis_flag = gk_sd_readw((unsigned int)pinfo->regbase + SD_NIXEN_OFFSET);
    nis_flag = nis_flag & 0xfffe;
    gk_sd_writew((unsigned int)pinfo->regbase + SD_NIXEN_OFFSET, nis_flag);
#endif
    //GD_SDIO_Rest();

    pslotinfo->state = GK_SD_STATE_RESET;
    pslotinfo_next->state = GK_SD_STATE_RESET;
    pinfo->reset_error = 0;

    gk_sd_dbg(pslotinfo, "Exit %s with counter %u\n", __func__, counter);
}

static void gk_sd_reset_cmd_line(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    u32 counter = 0;
    u8 reset_reg;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    gk_sd_writeb((unsigned int)pinfo->regbase + SD_RESET_OFFSET, SD_RESET_CMD);
    while (1) {
        reset_reg = gk_sd_readb((unsigned int)pinfo->regbase + SD_RESET_OFFSET);
        if (!(reset_reg & SD_RESET_CMD))
            break;
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            //printk("Wait SD_RESET_CMD...\n");
            pinfo->reset_error = 1;
            break;
        }
    }
}

static void gk_sd_reset_data_line(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    u32 counter = 0;
    u8 reset_reg;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    gk_sd_dbg(pslotinfo, "Enter %s with state %u\n",
        __func__, pslotinfo->state);

    gk_sd_writeb((unsigned int)pinfo->regbase + SD_RESET_OFFSET, SD_RESET_DAT);
    while (1) {
        reset_reg = gk_sd_readb((unsigned int)pinfo->regbase + SD_RESET_OFFSET);
        if (!(reset_reg & SD_RESET_DAT))
            break;
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            gk_sd_warn(pslotinfo, "Wait SD_RESET_DAT...\n");
            pinfo->reset_error = 1;
            break;
        }
    }
    gk_sd_dbg(pslotinfo, "Exit %s with counter %u\n", __func__, counter);
}

static inline void gk_sd_data_done(
    struct gk_sd_mmc_info *pslotinfo, u16 nis, u16 eis)
{
    struct mmc_command *cmd;
    struct mmc_data *data;
    struct mmc_host *host;
    struct mmc_request *local_req;

    host = pslotinfo->mmc;
    local_req = pslotinfo->mrq;
    if ((pslotinfo->state == GK_SD_STATE_CMD) &&
        ((pslotinfo->cmd_reg & 0x3) == SD_CMD_RSP_48BUSY)) {
        if (eis) {
            pslotinfo->state = GK_SD_STATE_ERR;
        } else {
            pslotinfo->state = GK_SD_STATE_IDLE;
        }
        mmc_request_done(host, pslotinfo->mrq);
        return;
    }
    if (pslotinfo->mrq == NULL) {
        gk_sd_dbg(pslotinfo, "%s: mrq is NULL, nis[0x%x] eis[0x%x]\n",
            __func__, nis, eis);
        return;
    }
    if (pslotinfo->mrq->data == NULL) {
        gk_sd_dbg(pslotinfo, "%s: data is NULL, nis[0x%x] eis[0x%x]\n",
            __func__, nis, eis);
        return;
    }

    data = pslotinfo->mrq->data;
    cmd  = pslotinfo->mrq->cmd;

    //printk("### CMD[%u] get eis[0x%x]\n", pslotinfo->mrq->cmd->opcode, eis);

    if (eis) {
        if (eis & SD_EIS_DATA_BIT_ERR) {
            data->error = -EILSEQ;
        } else if (eis & SD_EIS_DATA_CRC_ERR) {
            data->error = -EILSEQ;
        } else if (eis & SD_EIS_DATA_TMOUT_ERR) {
            data->error = -ETIMEDOUT;
        } else {
            data->error = -ETIMEDOUT;
        }
        cmd->error = data->error;
//#ifdef CONFIG_SD_GK_DEBUG_VERBOSE
        //printk("error -- %s: CMD[%u] get eis[0x%x]\n", __func__,
        //    pslotinfo->mrq->cmd->opcode, eis);
//#endif
        pslotinfo->state = GK_SD_STATE_ERR;
        gk_sd_release_bus(pslotinfo->mmc);
        if(pslotinfo->pre_post_used == 0)
        {
            gk_sd_post_req(host, pslotinfo->mrq,0);
        }
        mmc_request_done(host, local_req);
        return;
    } else {
        data->bytes_xfered = pslotinfo->dma_size;
    }

    gk_sd_release_bus(pslotinfo->mmc);
    pslotinfo->state = GK_SD_STATE_IDLE;
    if(pslotinfo->pre_post_used == 0)
    {
        gk_sd_post_req(host, pslotinfo->mrq,0);
    }
    mmc_request_done(host, local_req);
}

static inline void gk_sd_cmd_done(
    struct gk_sd_mmc_info *pslotinfo, u16 nis, u16 eis)
{
    struct mmc_command *cmd;
    u32 rsp0, rsp1, rsp2, rsp3;
    struct gk_sd_controller_info *pinfo;
    u16 ac12es;
    struct mmc_host *host;
    struct mmc_request *local_req;
    host = pslotinfo->mmc;
    local_req = pslotinfo->mrq;

    if (pslotinfo->mrq == NULL) {
        printk("%s: mrq is NULL, nis[0x%x] eis[0x%x]\n",
            __func__, nis, eis);
        return;
    }
    if (pslotinfo->mrq->cmd == NULL) {
        printk( "%s: cmd is NULL, nis[0x%x] eis[0x%x]\n",
            __func__, nis, eis);
        return;
    }

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;
    cmd = pslotinfo->mrq->cmd;

    //printk("#### CMD[%u] get eis[0x%x]\n", pslotinfo->mrq->cmd->opcode, eis);

    if (eis) {
        if (eis & SD_EIS_CMD_BIT_ERR) {
            cmd->error = -EILSEQ;
        } else if (eis & SD_EIS_CMD_CRC_ERR) {
            cmd->error = -EILSEQ;
        } else if (eis & SD_EIS_CMD_TMOUT_ERR) {
            cmd->error = -ETIMEDOUT;
        } else if (eis & SD_EIS_ACMD12_ERR) {
            ac12es = gk_sd_readl((unsigned int)pinfo->regbase + SD_AC12ES_OFFSET);
            if (ac12es & SD_AC12ES_TMOUT_ERROR) {
                cmd->error = -ETIMEDOUT;
            } else if (eis & SD_AC12ES_CRC_ERROR) {
                cmd->error = -EILSEQ;
            } else {
                cmd->error = -EIO;
            }

            if (pslotinfo->mrq->stop) {
                pslotinfo->mrq->stop->error = cmd->error;
            } else {
                gk_sd_err(pslotinfo, "%s NULL stop 0x%x %u\n",
                    __func__, ac12es, cmd->error);
            }
        } else {
            cmd->error = -EIO;
        }

        pslotinfo->state = GK_SD_STATE_ERR;

        //printk("error - %s: CMD[%u] get eis[0x%x] state= [%s]\n", __func__,
        //    pslotinfo->mrq->cmd->opcode, eis, sd_state(pslotinfo->state));

        if(!pslotinfo->mrq->data)
        {
            pslotinfo->state = GK_SD_STATE_IDLE;
            gk_sd_release_bus(pslotinfo->mmc);
            gk_sd_post_req(host, pslotinfo->mrq,0);
            mmc_request_done(host, local_req);
        }
        return;
    }
//printk("cmd handle ok...get reponse\n");
    if (cmd->flags & MMC_RSP_136) {
        rsp0 = gk_sd_readl((unsigned int)pinfo->regbase + SD_RSP0_OFFSET);
        rsp1 = gk_sd_readl((unsigned int)pinfo->regbase + SD_RSP1_OFFSET);
        rsp2 = gk_sd_readl((unsigned int)pinfo->regbase + SD_RSP2_OFFSET);
        rsp3 = gk_sd_readl((unsigned int)pinfo->regbase + SD_RSP3_OFFSET);
        cmd->resp[0] = ((rsp3 << 8) | (rsp2 >> 24));
        cmd->resp[1] = ((rsp2 << 8) | (rsp1 >> 24));
        cmd->resp[2] = ((rsp1 << 8) | (rsp0 >> 24));
        cmd->resp[3] = (rsp0 << 8);

        //printk("response1: [0x%x] [0x%x] [0x%x] [0x%x] \n", cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
    } else {
        cmd->resp[0] = gk_sd_readl((unsigned int)pinfo->regbase + SD_RSP0_OFFSET);
        //printk("response2: [0x%x] \n", cmd->resp[0]);
    }

    if(!pslotinfo->mrq->data)
    {
        pslotinfo->state = GK_SD_STATE_IDLE;
        gk_sd_release_bus(pslotinfo->mmc);
        gk_sd_post_req(host, pslotinfo->mrq,0);
        mmc_request_done(host, local_req);
    }
}

static irqreturn_t gk_sd_irq(int irq, void *devid)
{
    struct gk_sd_controller_info *pinfo;
    struct gk_sd_mmc_info *pslotinfo = NULL;
    u16 nis=0;
    u16 eis=0;
//    u32 enabled = 0;
    int handle = 0;
    pinfo = (struct gk_sd_controller_info *)devid;

    /* Read the interrupt registers */
    nis = gk_sd_readw((unsigned int)pinfo->regbase + SD_NIS_OFFSET);
    eis = gk_sd_readw((unsigned int)pinfo->regbase + SD_EIS_OFFSET);
    //printk("-- nis = 0x%x eis = 0x%x\n", nis, eis);

    switch(eis)
    {
        case SD_EIS_ACMD12_ERR:
            printk("-- SD_EIS_ACMD12_ERR\n");
            break;
        case SD_EIS_CURRENT_ERR:
            printk("-- SD_EIS_CURRENT_ERR\n");
            break;
        case SD_EIS_DATA_BIT_ERR:
            printk("-- SD_EIS_DATA_BIT_ERR\n");
            break;
        case SD_EIS_DATA_CRC_ERR:
            printk("-- SD_EIS_DATA_CRC_ERR\n");
            break;
        case SD_EIS_DATA_TMOUT_ERR:
            printk("-- SD_EIS_DATA_TMOUT_ERR\n");
            break;
        case SD_EIS_CMD_IDX_ERR:
            printk("-- SD_EIS_CMD_IDX_ERR\n");
            break;
        case SD_EIS_CMD_BIT_ERR:
            printk("-- SD_EIS_CMD_BIT_ERR\n");
            break;
        case SD_EIS_CMD_CRC_ERR:
            printk("-- SD_EIS_CMD_CRC_ERR\n");
            break;
        case SD_EIS_CMD_TMOUT_ERR:
            //printk("-- SD_EIS_CMD_TMOUT_ERR\n");
            break;
        default:
            break;
    }


    pslotinfo = pinfo->pslotinfo_cur;
    spin_lock(&pslotinfo->lock);

    /* Clear interrupt */
    gk_sd_writew((unsigned int)pinfo->regbase + SD_NIS_OFFSET, nis);
    gk_sd_writew((unsigned int)pinfo->regbase + SD_EIS_OFFSET, eis);

    if (nis & SD_NIS_CARD) {
        //printk("SD_NIS_CARD -- mmc_signal_sdio_irq\n");
        mmc_signal_sdio_irq(pslotinfo->mmc);
    }

    //if (!gk_is_valid_gpio_irq(&pslotinfo->plat_info->gpio_cd) &&
    //    (pslotinfo->plat_info->fixed_cd == -1))
    {
        if (nis & SD_NIS_REMOVAL) {
            //printk("SD_NIS_REMOVAL -- mmc_detect_change\n");
            mmc_detect_change(pslotinfo->mmc, pslotinfo->plat_info->cd_delay);
        } else if (nis & SD_NIS_INSERT) {
            //printk("SD_NIS_INSERT -- mmc_detect_change\n");
            mmc_detect_change(pslotinfo->mmc, pslotinfo->plat_info->cd_delay);
        }
    }

    if (eis) {
        if (eis & (SD_EIS_CMD_TMOUT_ERR | SD_EIS_CMD_CRC_ERR |
            SD_EIS_CMD_BIT_ERR | SD_EIS_CMD_IDX_ERR |
            SD_EIS_ACMD12_ERR)) {
            gk_sd_reset_cmd_line(pslotinfo->mmc);
        }
        if (eis & (SD_EIS_DATA_TMOUT_ERR | SD_EIS_DATA_CRC_ERR)) {
            gk_sd_reset_data_line(pslotinfo->mmc);
        }
        if (eis & (SD_EIS_DATA_BIT_ERR | SD_EIS_CURRENT_ERR)) {
            gk_sd_reset_all(pslotinfo->mmc);
        }
        if (pslotinfo->state == GK_SD_STATE_CMD) {
            gk_sd_cmd_done(pslotinfo, nis, eis);
        } else if (pslotinfo->state == GK_SD_STATE_DATA) {
            gk_sd_data_done(pslotinfo, nis, eis);
            pslotinfo->pending_events |= GK_SD_PEND_DTO_m;        
		    handle = 1;
        }
    } 
    else {
        if (nis & SD_NIS_XFR_DONE) {
            pslotinfo->pending_events |= GK_SD_PEND_DTO_m;      
		    handle = 1;
        }
    }
    spin_unlock(&pslotinfo->lock);
    if (handle)
    {
        wake_up(&pslotinfo->wait);
		return IRQ_HANDLED;
	}
    return IRQ_NONE;
}

static int gk_sd_gpio_cd_check_val(struct gk_sd_mmc_info *pslotinfo)
{
    u32 val = -1;
    unsigned int i, curr_status, status[3], detect_retry_count = 0;
    gk_gpio_config(pslotinfo->plat_info->gpio_cd.irq_gpio, GPIO_TYPE_INPUT_1);
    
    while (1) 
    {
		for (i = 0; i < 3; i++) {
			status[i] = gk_gpio_get(pslotinfo->plat_info->gpio_cd.irq_gpio);
			udelay(10);
		}
		if ((status[0] == status[1]) && (status[0] == status[2]))
		{
			break;
        }
		detect_retry_count++;
		if (detect_retry_count >= 100) {
			printk("this is a dithering,card detect error!");
			val = -1;
			goto err;
		}
	}
    val = status[0];    
    err:
    val = (val == pslotinfo->plat_info->gpio_cd.irq_gpio_val) ?    1 : 0;

    gk_sd_dbg(pslotinfo, "%s:%u\n", (val == 1) ?
        "card insert" : "card eject",
        pslotinfo->plat_info->cd_delay);

    pslotinfo->plat_info->fixed_cd = val;

#if defined(CONFIG_SYSTEM_USE_NETLINK_MSG_SD)
    GK_NET_LINK_MSG_S msg;
    msg.module = GK_NL_SD;
    msg.len = GK_NL_SD_MSG_LEN;
    msg.data.gk_nl_sd_data.insert = (u8)val;
    gk_nl_send_message(&msg);
#endif
    return val;
}

static irqreturn_t gk_sd_gpio_cd_irq(int irq, void *devid)
{
    struct gk_sd_mmc_info *pslotinfo;

    pslotinfo = (struct gk_sd_mmc_info *)devid;
    if (pslotinfo->valid &&
        (gk_sd_gpio_cd_check_val(pslotinfo) != -1)) {
        mmc_detect_change(pslotinfo->mmc,
            pslotinfo->plat_info->cd_delay);
    }

    return IRQ_HANDLED;
}

static void gk_sd_set_clk(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    u16 clk_div = 0x0000;
    u32 sd_clk;
    u32 desired_clk;
    u32 actual_clk;
    u32 bneed_div = 1;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    gk_sd_clear_clken(mmc);
    if (ios->clock == 0) {
        pinfo->pcontroller->active_clock = 0;
    } else {
        desired_clk = ios->clock;
        if (desired_clk > pinfo->clk_limit)
            desired_clk = pinfo->clk_limit;

        if (pinfo->pcontroller->support_pll_scaler) {
            if (desired_clk < 10000000) {
                /* Below 10Mhz, divide by sd controller */
                pinfo->pcontroller->set_pll(pinfo->clk_limit);
            } else {
                pinfo->pcontroller->set_pll(desired_clk);
                actual_clk = pinfo->pcontroller->get_pll();
                bneed_div = 0;
            }
        }

        if (bneed_div) {
            sd_clk = pinfo->pcontroller->get_pll();
            for (clk_div = 0x0; clk_div <= 0x80;) {
                if (clk_div == 0)
                    actual_clk = sd_clk;
                else
                    actual_clk = sd_clk / (clk_div << 1);

                if (actual_clk <= desired_clk)
                    break;

                if (clk_div >= 0x80)
                    break;

                if (clk_div == 0x0)
                    clk_div = 0x1;
                else
                    clk_div <<= 1;
            }
        }
        //printk("sd_pll = %u.\n",pinfo->pcontroller->get_pll());
        //printk("desired_clk = %u.\n", desired_clk);
        //printk("actual_clk = %u.\n", actual_clk);
        //printk("clk_div = %u.\n", clk_div);
        pinfo->pcontroller->active_clock = actual_clk;
        gk_sd_set_iclk(mmc, clk_div);
        gk_sd_set_clken(mmc);
    }
}

static void gk_sd_set_pwr(struct mmc_host *mmc, struct mmc_ios *ios)
{
    //printk("gk_sd_set_pwr ....\n");

    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    if (ios->power_mode == MMC_POWER_OFF) {
        //printk("power off\n");
        gk_sd_reset_all(pslotinfo->mmc);
        gk_sd_writeb((unsigned int)pinfo->regbase + SD_PWR_OFFSET, SD_PWR_OFF);
        //gk_set_gpio_output_can_sleep(
        //    &pslotinfo->plat_info->ext_reset, 1, 1);
        //gk_set_gpio_output_can_sleep(
        //    &pslotinfo->plat_info->ext_power, 0, 1);
        if (pslotinfo->plat_info->set_vdd) {
            pslotinfo->plat_info->set_vdd(0);
        }
    } else if (ios->power_mode == MMC_POWER_UP) {
        if (pslotinfo->plat_info->set_vdd) {
            pslotinfo->plat_info->set_vdd(3300);
        }
        //printk("power up...\n");
        //gk_set_gpio_output_can_sleep(
        //    &pslotinfo->plat_info->ext_power, 1, 1);
        //gk_set_gpio_output_can_sleep(
        //    &pslotinfo->plat_info->ext_reset, 0, 1);
        gk_sd_writeb((unsigned int)pinfo->regbase + SD_PWR_OFFSET,
            (SD_PWR_ON | SD_PWR_3_3V));
    } else if (ios->power_mode == MMC_POWER_ON) {
        //printk("power on...\n");
        switch (1 << ios->vdd) {
        case MMC_VDD_32_33:
        case MMC_VDD_33_34:
            break;
        default:
            gk_sd_err(pslotinfo, "%s Wrong voltage[%u]!\n",
                __func__, ios->vdd);
            break;
        }
    }
    msleep(pinfo->pcontroller->pwr_delay);
    gk_sd_dbg(pslotinfo, "pwr = 0x%x.\n",
        gk_sd_readb((unsigned int)pinfo->regbase + SD_PWR_OFFSET));

}

static void gk_sd_set_bus(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    u8 hostr = 0;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    hostr = gk_sd_readb((unsigned int)pinfo->regbase + SD_HOST_OFFSET);
    if (ios->bus_width == MMC_BUS_WIDTH_8) {
        hostr |= SD_HOST_8BIT;
        hostr &= ~(SD_HOST_4BIT);
    } else if (ios->bus_width == MMC_BUS_WIDTH_4) {
        hostr &= ~(SD_HOST_8BIT);
        hostr |= SD_HOST_4BIT;
    } else if (ios->bus_width == MMC_BUS_WIDTH_1) {
        hostr &= ~(SD_HOST_8BIT);
        hostr &= ~(SD_HOST_4BIT);
    } else {
        gk_sd_err(pslotinfo, "Unknown bus_width[%u], assume 1bit.\n",
            ios->bus_width);
        hostr &= ~(SD_HOST_8BIT);
        hostr &= ~(SD_HOST_4BIT);
    }
    // Steven Yu:
    // do not set this bit, would cause xc card rw failed.
    hostr &= ~SD_HOST_HIGH_SPEED;
    //hostr |= SD_HOST_HIGH_SPEED;
    gk_sd_writeb((unsigned int)pinfo->regbase + SD_HOST_OFFSET, hostr);

    gk_sd_dbg(pslotinfo, "hostr = 0x%x.\n", hostr);
}

static void gk_sd_check_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    if ((pinfo->controller_ios.power_mode != ios->power_mode) ||
        (pinfo->controller_ios.vdd != ios->vdd) ||
        (pslotinfo->state == GK_SD_STATE_RESET)) {
        gk_sd_set_pwr(mmc, ios);
        pinfo->controller_ios.power_mode = ios->power_mode;
        pinfo->controller_ios.vdd = ios->vdd;
    }

    if ((pinfo->controller_ios.clock != ios->clock) ||
        (pslotinfo->state == GK_SD_STATE_RESET)) {
        gk_sd_set_clk(mmc, ios);
        pinfo->controller_ios.clock = ios->clock;
    }

    if ((pinfo->controller_ios.bus_width != ios->bus_width) ||
        (pinfo->controller_ios.timing != ios->timing) ||
        //(pinfo->controller_ios.ddr != ios->ddr) ||
        (pslotinfo->state == GK_SD_STATE_RESET)) {
        gk_sd_set_bus(mmc, ios);
        pinfo->controller_ios.bus_width = ios->bus_width;
        pinfo->controller_ios.timing = ios->timing;
        //pinfo->controller_ios.ddr = ios->ddr;
    }

    if (pslotinfo->state == GK_SD_STATE_RESET) {
        pslotinfo->state = GK_SD_STATE_IDLE;
    }
}

static u32 gk_sd_check_cd(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    int cdpin;
    u32 valid_cd = 0;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    if (pslotinfo->plat_info->fixed_cd == 1) {
        valid_cd = 1;
    } else if (pslotinfo->plat_info->fixed_cd == 0) {
        valid_cd = 0;
    } else {

        cdpin = gk_sd_gpio_cd_check_val(pslotinfo);
        if (cdpin == 1) {
            valid_cd = 1;
        } else if (cdpin == -1) {
            if (gk_sd_tstbitsl((unsigned int)pinfo->regbase + SD_STA_OFFSET,
                SD_STA_CARD_INSERTED)) {
                valid_cd = 1;
            }
        }
    }

    return valid_cd;
}

static inline void gk_sd_prepare_tmo(
    struct gk_sd_mmc_info *pslotinfo,
    struct mmc_data *pmmcdata)
{
    struct gk_sd_controller_info *pinfo;
#if defined(CONFIG_SD_GK_CALIB_TMO)
    u32 actual_cycle_ns;
    u32 desired_tmo;
    u32 actual_tmo;
    u32 actual_hz;
#endif

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;
#if defined(CONFIG_SD_GK_CALIB_TMO)
    if (pinfo->pcontroller->active_clock >= HZ) {
        actual_hz = pinfo->pcontroller->active_clock / HZ;
        actual_cycle_ns = ((1000000000 / HZ) / actual_hz);
        desired_tmo = pmmcdata->timeout_ns / actual_cycle_ns;
        desired_tmo += pmmcdata->timeout_clks;
        for (pslotinfo->tmo = 0; pslotinfo->tmo < 0xe;
            pslotinfo->tmo++) {
            actual_tmo = (0x1 << (13 + pslotinfo->tmo));
            if (actual_tmo >= desired_tmo)
                break;
        }
        pslotinfo->wait_tmo = (actual_tmo + actual_hz - 1);
        pslotinfo->wait_tmo /= actual_hz;
    } else {
        pslotinfo->tmo = CONFIG_SD_GK_TIMEOUT_VAL;
        pslotinfo->wait_tmo = pinfo->pcontroller->wait_tmo;
    }
    if ((pslotinfo->wait_tmo > 0) && (pslotinfo->wait_tmo <
        CONFIG_SD_GK_MAX_TIMEOUT)) {
        pslotinfo->sta_counter = CONFIG_SD_GK_MAX_TIMEOUT;
        pslotinfo->sta_counter /= pslotinfo->wait_tmo;
    } else {
        pslotinfo->sta_counter = 1;
        pslotinfo->wait_tmo = (1 * HZ);
    }
#else
    pslotinfo->tmo = CONFIG_SD_GK_TIMEOUT_VAL;
    pslotinfo->wait_tmo = pinfo->pcontroller->wait_tmo;
    if ((pslotinfo->wait_tmo > 0) && (pslotinfo->wait_tmo <
        CONFIG_SD_GK_MAX_TIMEOUT)) {
        pslotinfo->sta_counter = CONFIG_SD_GK_MAX_TIMEOUT;
        pslotinfo->sta_counter /= pslotinfo->wait_tmo;
    } else {
        pslotinfo->sta_counter = 1;
        pslotinfo->wait_tmo = (1 * HZ);
    }
#endif
    gk_sd_dbg(pslotinfo, "timeout_ns = %u, timeout_clks = %u @ %uHz, "
        "wait_tmo = %u, tmo = %u, sta_counter = %u.\n",
        pmmcdata->timeout_ns, pmmcdata->timeout_clks,
        pinfo->pcontroller->active_clock, pslotinfo->wait_tmo,
        pslotinfo->tmo, pslotinfo->sta_counter);
}

static void gk_sd_pre_req(struct mmc_host *host, struct mmc_request *req,
           bool is_first_req)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(host);
    struct gk_sd_controller_info *pinfo;
    u32 i;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    down(&pinfo->pslotinfo_sem);

    for(i=0;i<INFO_NUM;i++)
    {
        if(pslotinfo->mrq==NULL)
        {
            break;
        }
        pslotinfo = pslotinfo+1;
    }
    if (i == INFO_NUM)
    {
        gk_sd_err(pslotinfo, "%s: pslotinfo wait queue full, INFO_NUM:%d\n",
                __func__, i);
        return;
    }
    pslotinfo->mrq = req;
    pslotinfo->pre_post_used = 1;
    gk_sd_pre_cmd(pslotinfo);
}

static inline void gk_sd_pre_cmd(
    struct gk_sd_mmc_info *pslotinfo)
{
    struct gk_sd_controller_info *pinfo;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    pslotinfo->state = GK_SD_STATE_CMD;
    pslotinfo->sg_len = 0;
    pslotinfo->sg = NULL;
    pslotinfo->blk_sz = 0;
    pslotinfo->blk_cnt = 0;
    pslotinfo->arg_reg = 0;
    pslotinfo->cmd_reg = 0;
    pslotinfo->sta_reg = 0;
    pslotinfo->tmo = CONFIG_SD_GK_TIMEOUT_VAL;
    pslotinfo->wait_tmo = (1 * HZ);
    pslotinfo->xfr_reg = 0;
    pslotinfo->dma_address = 0;
    pslotinfo->dma_size = 0;

    if (pslotinfo->mrq->stop) {
        //if (likely(pslotinfo->mrq->stop->opcode ==    MMC_STOP_TRANSMISSION)) {
        if ((pslotinfo->mrq->stop->opcode == MMC_STOP_TRANSMISSION)) {
            pslotinfo->xfr_reg = SD_XFR_AC12_EN;
        } else {
            gk_sd_err(pslotinfo, "%s strange stop cmd%u\n",
                __func__, pslotinfo->mrq->stop->opcode);
        }
    }

    if (!(pslotinfo->mrq->cmd->flags & MMC_RSP_PRESENT))
        pslotinfo->cmd_reg = SD_CMD_RSP_NONE;
    else if (pslotinfo->mrq->cmd->flags & MMC_RSP_136)
        pslotinfo->cmd_reg = SD_CMD_RSP_136;
    else if (pslotinfo->mrq->cmd->flags & MMC_RSP_BUSY)
        pslotinfo->cmd_reg = SD_CMD_RSP_48BUSY;
    else
        pslotinfo->cmd_reg = SD_CMD_RSP_48;
    if (pslotinfo->mrq->cmd->flags & MMC_RSP_CRC)
    {
        //printk("add crc check...\n");
        pslotinfo->cmd_reg |= SD_CMD_CHKCRC;
    }

    if (pslotinfo->mrq->cmd->flags & MMC_RSP_OPCODE)
        pslotinfo->cmd_reg |= SD_CMD_CHKIDX;

    //printk("## cmd = %d ##\n", pslotinfo->mrq->cmd->opcode);
    pslotinfo->cmd_reg |= SD_CMD_IDX(pslotinfo->mrq->cmd->opcode);
    pslotinfo->arg_reg = pslotinfo->mrq->cmd->arg;

//printk(" cmd_reg = 0x%x arg_reg = 0x%x \n", pslotinfo->cmd_reg, pslotinfo->arg_reg);

    if (pslotinfo->mrq->data) {
        if((pslotinfo->arg_reg == 0) && (pinfo->id == 0)){
            if(pinfo->boot_media == BOOT_FROM_SD){
                if((pslotinfo->mmc->card != NULL) && mmc_card_blockaddr(pslotinfo->mmc->card)){
                    pslotinfo->arg_reg = 0x600;
                }else{
                    pslotinfo->arg_reg = 0xC0000;
                }
            }
        }
        pslotinfo->state = GK_SD_STATE_DATA;
        gk_sd_prepare_tmo(pslotinfo, pslotinfo->mrq->data);
        pslotinfo->blk_sz = (pslotinfo->mrq->data->blksz & 0xFFF);
        pslotinfo->dma_size = pslotinfo->mrq->data->blksz *    pslotinfo->mrq->data->blocks;
        //printk("blk size = %d dma_size = %d \n", pslotinfo->blk_sz, pslotinfo->dma_size);

        pslotinfo->sg_len = pslotinfo->mrq->data->sg_len;
        pslotinfo->sg = pslotinfo->mrq->data->sg;
        pslotinfo->xfr_reg |= SD_XFR_DMA_EN;
        pslotinfo->cmd_reg |= SD_CMD_DATA;
        pslotinfo->blk_cnt = pslotinfo->mrq->data->blocks;
        if (pslotinfo->blk_cnt > 1) {
            pslotinfo->xfr_reg |= SD_XFR_MUL_SEL;
            pslotinfo->xfr_reg |= SD_XFR_BLKCNT_EN;
        }
        if (pslotinfo->mrq->data->flags & MMC_DATA_STREAM) {
            pslotinfo->xfr_reg |= SD_XFR_MUL_SEL;
            pslotinfo->xfr_reg &= ~SD_XFR_BLKCNT_EN;
        }
        if (pslotinfo->mrq->data->flags & MMC_DATA_WRITE) {
            pslotinfo->xfr_reg &= ~SD_XFR_CTH_SEL;
            pslotinfo->sta_reg = (SD_STA_WRITE_XFR_ACTIVE |
                SD_STA_DAT_ACTIVE);
            pslotinfo->pre_dma = &gk_sd_pre_sg_to_dma;
            pslotinfo->post_dma = &gk_sd_post_sg_to_dma;
        } else {
            pslotinfo->xfr_reg |= SD_XFR_CTH_SEL;
            pslotinfo->sta_reg = (SD_STA_READ_XFR_ACTIVE |
                SD_STA_DAT_ACTIVE);
            pslotinfo->pre_dma = &gk_sd_pre_dma_to_sg;
            pslotinfo->post_dma = &gk_sd_post_dma_to_sg;
        }
        //printk("-----pre dma handle----\n");
        pslotinfo->pre_dma(pslotinfo);
        if (pinfo->dma_fix) {
            pslotinfo->dma_address |= pinfo->dma_fix;
        }
    }
}
static  int gk_sd_dat_line_isvalid(struct gk_sd_controller_info *pinfo)
{
    u32 sta_reg;
    u32 counter = 0;
    int ret;
    while (1) 
    {
        sta_reg = gk_sd_readl((unsigned int)pinfo->regbase + SD_STA_OFFSET);
        if ((sta_reg & SD_STA_CMD_INHIBIT_DAT) == 0) {
            ret = 0;
            break;
        }
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            ret = -1;
        }
    }
    return ret;
}
static  int gk_sd_cmd_line_isvalid(struct gk_sd_controller_info *pinfo)
{
    u32 sta_reg;
    u32 counter = 0;
    int ret;
    while (1) 
    {
        sta_reg = gk_sd_readl((unsigned int)pinfo->regbase + SD_STA_OFFSET);
        if ((sta_reg & SD_STA_CMD_INHIBIT_CMD) == 0) {
            ret = 0;
            break;
        }
        counter++;
        if (counter > CONFIG_SD_GK_WAIT_COUNTER_LIMIT) {
            ret = -1;
        }
    }
    return ret;
}
static int gk_sd_wait_data_complete(struct gk_sd_mmc_info *pslotinfo)
{
    long time = 5*HZ;
    if (pslotinfo->state == GK_SD_STATE_DATA)
    {
        time = wait_event_timeout(pslotinfo->wait,
            test_bit(GK_SD_PEND_DTO_b, &pslotinfo->pending_events),
            time);
    
        if ((time <= 0)
                && (!test_bit(GK_SD_PEND_DTO_b, &pslotinfo->pending_events))) {
                spin_lock(&pslotinfo->lock);
                pslotinfo->mrq->cmd->error = -ENOMEDIUM;
                pslotinfo->state = GK_SD_STATE_ERR;
                printk(COLOR_RED"wait data request complete is timeout!\n"COLOR_NONE);
                gk_sd_data_done(pslotinfo, 0, SD_EIS_DATA_TMOUT_ERR);
                spin_unlock(&pslotinfo->lock);
                return -1;
         }
    }
    if (pslotinfo->state == GK_SD_STATE_DATA)
    {
        spin_lock(&pslotinfo->lock);
        pslotinfo->pending_events &= ~GK_SD_PEND_DTO_m;
        gk_sd_data_done(pslotinfo, SD_NIS_XFR_DONE, 0);
        spin_unlock(&pslotinfo->lock);
    }
    return 0;

}
static int gk_sd_wait_cmd_complete(struct gk_sd_mmc_info *pslotinfo)
{
    struct gk_sd_controller_info *pinfo;    
    u16 nis,eis;
    u32 cmd_jiffies_timeout = jiffies + HZ/4;
    int ret = 0;
    if(pslotinfo->state == GK_SD_STATE_CMD)
    {
        pinfo = pslotinfo->pinfo;
        do{
            nis = gk_sd_readw((unsigned int)pinfo->regbase + SD_NIS_OFFSET);
            if (!time_before(jiffies, cmd_jiffies_timeout)) 
            {
                spin_lock(&pslotinfo->lock);
                pslotinfo->mrq->cmd->error = -ENOMEDIUM;
                pslotinfo->state = GK_SD_STATE_ERR;
                printk(COLOR_RED"wait cmd request complete is timeout!\n"COLOR_NONE);
                gk_sd_cmd_done(pslotinfo, 0, SD_EIS_CMD_TMOUT_ERR);
                ret = -1;
                spin_unlock(&pslotinfo->lock);
                break;
            }                    
        }while ((pslotinfo->state == GK_SD_STATE_CMD) && !(nis & SD_NIS_CMD_DONE));
        if(pslotinfo->state == GK_SD_STATE_CMD)
        {
            spin_lock(&pslotinfo->lock);
            eis = gk_sd_readw((unsigned int)pinfo->regbase + SD_EIS_OFFSET);
            gk_sd_writew((unsigned int)pinfo->regbase + SD_NIS_OFFSET, (nis & SD_NIS_CMD_DONE));
            gk_sd_cmd_done(pslotinfo, SD_NIS_CMD_DONE, eis);
            spin_unlock(&pslotinfo->lock);
        }
    }
    return 0;
}

static inline void gk_sd_send_cmd(
    struct gk_sd_mmc_info *pslotinfo)
{
    struct gk_sd_controller_info *pinfo;
    u32 valid_request = 0;
    u32 counter = 0;
    u32 sta_reg;


    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    gk_sd_request_bus(pslotinfo->mmc);

    valid_request = gk_sd_check_cd(pslotinfo->mmc);
    //printk("send cmd = %u valid_request = %u.\n", pslotinfo->mrq->cmd->opcode, valid_request);

    if (!valid_request) {
        pslotinfo->mrq->cmd->error = -ENOMEDIUM;
        pslotinfo->state = GK_SD_STATE_ERR;
        //printk("send cmd error...\n");
        goto gk_sd_send_cmd_exit;
    }

    gk_sd_check_ios(pslotinfo->mmc, &pslotinfo->mmc->ios);
    if (pslotinfo->mrq->data) 
    {
        if(0 != gk_sd_dat_line_isvalid(pinfo))
        {
            printk("Wait SD_STA_CMD_INHIBIT_DAT...\n");
            pslotinfo->state = GK_SD_STATE_ERR;
            pinfo->reset_error = 1;
            pslotinfo->mrq->data->error = -EIO;
            pslotinfo->mrq->cmd->error = -EIO;
            goto gk_sd_send_cmd_exit;
        }
        //printk("@@ write cmd arg...\n");
        gk_sd_writeb((unsigned int)pinfo->regbase + SD_TMO_OFFSET, pslotinfo->tmo);
        gk_sd_writel((unsigned int)pinfo->regbase + SD_DMA_ADDR_OFFSET,
            pslotinfo->dma_address);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_BLK_SZ_OFFSET,
            pslotinfo->blk_sz);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_BLK_CNT_OFFSET,
            pslotinfo->blk_cnt);
        gk_sd_writel((unsigned int)pinfo->regbase + SD_ARG_OFFSET, pslotinfo->arg_reg);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_XFR_OFFSET,
            pslotinfo->xfr_reg);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_CMD_OFFSET,
            pslotinfo->cmd_reg);
        gk_sd_wait_data_complete(pslotinfo);
		
    } 
    else 
    {
        if(0 != gk_sd_cmd_line_isvalid(pinfo))
        {
            gk_sd_warn(pslotinfo,
                "Wait SD_STA_CMD_INHIBIT_CMD...\n");
            pslotinfo->state = GK_SD_STATE_ERR;
            pinfo->reset_error = 1;
            pslotinfo->mrq->data->error = -EIO;
            pslotinfo->mrq->cmd->error = -EIO;
            goto gk_sd_send_cmd_exit;        
        }

        gk_sd_writel((unsigned int)pinfo->regbase + SD_ARG_OFFSET, pslotinfo->arg_reg);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_XFR_OFFSET,
            0x00);
        gk_sd_writew((unsigned int)pinfo->regbase + SD_CMD_OFFSET,
            pslotinfo->cmd_reg);
        gk_sd_wait_cmd_complete(pslotinfo);
    }

    return;
gk_sd_send_cmd_exit:
    if(pslotinfo->mrq!=NULL)
    {
        gk_sd_release_bus(pslotinfo->mmc);
        mmc_request_done(pslotinfo->mmc,pslotinfo->mrq);
        gk_sd_post_req(pslotinfo->mmc,pslotinfo->mrq,0);
    }
    return;
}

static void gk_sd_post_req(struct mmc_host *host, struct mmc_request *req,
            int err)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(host);
    struct gk_sd_controller_info *pinfo;
    u32 i;

    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    for(i=0;i<INFO_NUM;i++)
    {
        if(pslotinfo->mrq==req)
        {
            break;
        }
        pslotinfo = pslotinfo+1;
    }
    if (pslotinfo->mrq!=NULL)
    {
        if (pslotinfo->state == GK_SD_STATE_IDLE)
        {
            if (pslotinfo->mrq->data) {
                pslotinfo->post_dma(pslotinfo);
            }
        }
        else
        {
            if (pinfo->reset_error) {
                gk_sd_reset_all(pslotinfo->mmc);
            }
        }
        pslotinfo->mrq=NULL;
        up(&pinfo->pslotinfo_sem);
    }
}

static void gk_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct gk_sd_controller_info *pinfo;
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc),*pslotinfo_def;
    u32 i;
    if(mrq==NULL)
    {
        printk("%s err mrq NULL\n",__FUNCTION__);
        return;
    }

    pslotinfo_def = pslotinfo;
    pinfo = pslotinfo->pinfo;

    for(i=0;i<INFO_NUM;i++)
    {
        if(pslotinfo->mrq==mrq)
        {
            break;
        }
        pslotinfo=pslotinfo+1;
    }

    if(i==INFO_NUM)
    {
        pslotinfo = pslotinfo_def;
        down(&pinfo->pslotinfo_sem);
        for(i=0;i<INFO_NUM;i++)
        {
            if(pslotinfo->mrq==NULL)
            {
                break;
            }
            pslotinfo = pslotinfo+1;
        }
        pslotinfo->mrq = mrq;
        pslotinfo->pre_post_used = 0;
        gk_sd_pre_cmd(pslotinfo);
    }

    //if(mrq && mrq->cmd)  printk("######### mmc cmd  request: CMD[%u] \n", mrq->cmd->opcode);
    //if(mrq && mrq->data) printk("######### mmc data request: blksz=%d blocks=%d\n", mrq->data->blksz, mrq->data->blocks);

    pinfo->pslotinfo_cur = pslotinfo;
    gk_sd_send_cmd(pslotinfo);
	
	

}

static void gk_sd_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    gk_sd_request_bus(mmc);
    gk_sd_check_ios(mmc, ios);
    gk_sd_release_bus(mmc);
}

static int gk_sd_get_ro(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    struct gk_sd_controller_info *pinfo;
    u32 wpspl;
    return  SD_STA_WRITABLE;
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    gk_sd_request_bus(mmc);

    if (pslotinfo->plat_info->fixed_wp < 0) {
        if (pslotinfo->plat_info->gpio_wp.gpio_id != -1) {
            wpspl = gk_get_gpio_input(&pslotinfo->plat_info->gpio_wp);
        } else {
            wpspl = gk_sd_readl((unsigned int)pinfo->regbase + SD_STA_OFFSET);
            wpspl &= SD_STA_WPS_PL;  //1 read only, 0 write
        }
    } else {
        wpspl =    pslotinfo->plat_info->fixed_wp;
    }

    gk_sd_release_bus(mmc);

    gk_sd_dbg(pslotinfo, "RO[0x%x].\n", wpspl);

    return wpspl ? SD_STA_READ_ONLY : SD_STA_WRITABLE;
}

static int gk_sd_get_cd(struct mmc_host *mmc)
{
    struct gk_sd_mmc_info *pslotinfo = mmc_priv(mmc);
    u32 cdpin;

    gk_sd_request_bus(mmc);
    cdpin = gk_sd_check_cd(mmc);
    gk_sd_release_bus(mmc);

    gk_sd_dbg(pslotinfo, "CD[%u].\n", cdpin);
    return cdpin ? 1 : 0;
}

static void gk_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
    if (enable)
        gk_sd_enable_int(mmc, SD_NISEN_CARD);
    else
        gk_sd_disable_int(mmc, SD_NISEN_CARD);
}

static const struct mmc_host_ops gk_sd_host_ops = {
    .post_req = gk_sd_post_req,
    .pre_req = gk_sd_pre_req,
    .request = gk_sd_request,
    .set_ios = gk_sd_ios,
    .get_ro  = gk_sd_get_ro,
    .get_cd  = gk_sd_get_cd,
    .enable_sdio_irq = gk_sd_enable_sdio_irq,
};

static int gk_sd_system_event(struct notifier_block *nb,
    unsigned long val, void *data)
{
    int retval = NOTIFY_OK;
    struct gk_sd_mmc_info *pslotinfo;
    struct gk_sd_controller_info *pinfo;

    pslotinfo = container_of(nb, struct gk_sd_mmc_info,
        system_event);
    pinfo = (struct gk_sd_controller_info *)pslotinfo->pinfo;

    switch (val) {
    case GK_EVENT_PRE_CPUFREQ:
        pr_debug("%s[%u]: Pre Change\n", __func__, pslotinfo->slot_id);
        down(&pslotinfo->system_event_sem);
        break;

    case GK_EVENT_POST_CPUFREQ:
        pr_debug("%s[%u]: Post Change\n", __func__, pslotinfo->slot_id);
        gk_sd_set_clk(pslotinfo->mmc, &pinfo->controller_ios);
        up(&pslotinfo->system_event_sem);
        break;

    default:
        break;
    }

    return retval;
}
struct gk_sd_mmc_info *gk_host[4] = {NULL, NULL, NULL, NULL};
void gk_sd_rescan_card(unsigned id, unsigned insert)
{
    struct gk_sd_mmc_info *host = NULL;
    struct gk_sd_controller_info *pinfo = NULL;
    unsigned int val = 0;


    BUG_ON(id > 3);
    BUG_ON(gk_host[id] == NULL);

	//printk("%s id = %d insert %d\n", __func__, id, insert);
    if(gk_host[id] == NULL) {
        printk("%s gk_host[%d] is NULL is err\n", __func__, id);
        return;
    }

    host = gk_host[id];
	//printk("[%s %d]%d %08x %08x %08x %08x\n", __func__, __LINE__, id, (u32)gk_host[0], (u32)gk_host[1], (u32)gk_host[2], (u32)gk_host[3]);

    pinfo = (struct gk_sd_controller_info *)host->pinfo;

    if(insert) {
        val = gk_sd_readl((unsigned int)pinfo->regbase + SD_STA_OFFSET);
        val |= SD_STA_CARD_INSERTED;
        gk_sd_writel((unsigned int)pinfo->regbase + SD_STA_OFFSET, val);
    }else {
        val = gk_sd_readl((unsigned int)pinfo->regbase + SD_STA_OFFSET);
        val &= ~SD_STA_CARD_INSERTED;
        gk_sd_writel((unsigned int)pinfo->regbase + SD_STA_OFFSET, val);
    }

    mmc_detect_change(host->mmc, 0);

    return;
}
EXPORT_SYMBOL_GPL(gk_sd_rescan_card);

/* ==========================================================================*/
static int __devinit gk_sd_probe(struct platform_device *pdev)
{
    int retval = 0;
    struct gk_sd_controller_info *pinfo = NULL;
    struct gk_sd_mmc_info *pslotinfo = NULL;
    struct gk_sd_mmc_info *pslotinfo_next = NULL;
    struct resource *irq = NULL;
    struct resource *mem = NULL;
    struct mmc_host *mmc = NULL;
    u32 hc_cap = 0;
    u32 i;
    u32 clock_min;
    u32 hc_timeout_clk = 0;

    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (mem == NULL) {
        dev_err(&pdev->dev, "Get SD/MMC mem resource failed!\n");
        retval = -ENXIO;
        goto gk_sd_probe_na;
    }

    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (irq == NULL) {
        dev_err(&pdev->dev, "Get SD/MMC irq resource failed!\n");
        retval = -ENXIO;
        goto gk_sd_probe_na;
    }
    dev_dbg(&pdev->dev, "sdio get irq: %d \n", irq->start);

    pinfo = kzalloc(sizeof(struct gk_sd_controller_info),
        GFP_KERNEL);
    if (pinfo == NULL) {
        dev_err(&pdev->dev, "Out of memory!\n");
        retval = -ENOMEM;
        goto gk_sd_probe_na;
    }
    pinfo->regbase = (unsigned char __iomem *)mem->start;
    //printk("regbase = 0x%x \n", pinfo->regbase);
    pinfo->dev = &pdev->dev;
    pinfo->irq = irq->start;
    pinfo->id  = pdev->id;
    pinfo->boot_media = gk_all_gpio_cfg.boot_media;
    pinfo->pcontroller = (struct gk_sd_controller *)pdev->dev.platform_data;
    if ((pinfo->pcontroller == NULL) ||
        (pinfo->pcontroller->get_pll == NULL) ||
        (pinfo->pcontroller->set_pll == NULL)) {
        dev_err(&pdev->dev, "Need SD/MMC controller info!\n");
        retval = -EPERM;
        goto gk_sd_probe_free_pinfo;
    }

    pinfo->dma_fix = pinfo->pcontroller->dma_fix;
    if (pinfo->pcontroller->wait_tmo < CONFIG_SD_GK_WAIT_TIMEOUT) {
        dev_notice(&pdev->dev, "Change wait timeout from %u to %u.\n",
            pinfo->pcontroller->wait_tmo,
            CONFIG_SD_GK_WAIT_TIMEOUT);
        pinfo->pcontroller->wait_tmo =
            CONFIG_SD_GK_WAIT_TIMEOUT;
    }

    pinfo->pcontroller->set_pll(pinfo->pcontroller->max_clock ?
                                pinfo->pcontroller->max_clock :
                                CONFIG_SD_GK_DEFAULT_CLOCK);

    pinfo->clk_limit = pinfo->pcontroller->get_pll();
    clock_min = pinfo->clk_limit >> 8;
    if (pinfo->clk_limit < clock_min) {
        pinfo->clk_limit = clock_min;
        dev_err(&pdev->dev, "Wrong clk_limit %uHz vs %uHz!\n",
            pinfo->clk_limit, clock_min);
        retval = -EPERM;
        goto gk_sd_probe_free_pinfo;
    }
    if (pinfo->pcontroller->max_clock != pinfo->clk_limit) {
        pinfo->pcontroller->max_clock = pinfo->clk_limit;
    }
    pinfo->max_blk_sz = gk_sd_dma_mask_to_size(
        pinfo->pcontroller->max_blk_mask);
    pinfo->buf_cache = kmem_cache_create(dev_name(&pdev->dev),
        pinfo->max_blk_sz*(INFO_NUM - 1) + SD_MIN_BUF, pinfo->max_blk_sz, 0, NULL);
    if (!pinfo->buf_cache) {
        dev_err(&pdev->dev, "Can't alloc DMA Cache");
        retval = -ENOMEM;
        goto gk_sd_probe_free_pinfo;
    }

    dev_dbg(&pdev->dev, "## max blk sz = %d \n", pinfo->max_blk_sz);

    sema_init(&pinfo->pslotinfo_sem, INFO_NUM);
    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        mmc = mmc_alloc_host(sizeof(struct gk_sd_mmc_info)*INFO_NUM, &pdev->dev);
        if (!mmc) {
            dev_err(&pdev->dev, "Failed to allocate mmc host"
                " for slot%u!\n", i);
            retval = -ENOMEM;
            goto gk_sd_probe_free_host;
        }
        dev_dbg(&pdev->dev, "mmc_alloc_host ok...\n");
        mmc->ops = &gk_sd_host_ops;

        pinfo->pslotinfo[i] = pslotinfo = mmc_priv(mmc);

        pslotinfo->mmc = mmc;
        pslotinfo->mrq = NULL;
        init_waitqueue_head(&pslotinfo->wait);
        spin_lock_init(&pslotinfo->lock);
        pslotinfo->pending_events = 0;
        pslotinfo->state = GK_SD_STATE_ERR;
        pslotinfo->plat_info = &(pinfo->pcontroller->slot[i]);
        if ((pslotinfo->plat_info == NULL) ||
            (pslotinfo->plat_info->check_owner == NULL) ||
            (pslotinfo->plat_info->set_int == NULL)) {
            dev_err(&pdev->dev, "Need SD/MMC slot info!\n");
            retval = -EPERM;
            goto gk_sd_probe_free_host;
        }
        pslotinfo->slot_id = i;
        pslotinfo->pinfo = pinfo;
        sema_init(&pslotinfo->system_event_sem, 1);

        gk_sd_request_bus(mmc);

        gk_sd_reset_all(mmc);
        //GD_SDIO_Rest();

        hc_cap = gk_sd_readl((unsigned int)pinfo->regbase + SD_CAP_OFFSET);
        hc_timeout_clk = SD_CAP_TOCLK_FREQ(hc_cap);
        if (hc_cap & SD_CAP_TOCLK_MHZ) {
            hc_timeout_clk *= 1000;
        }
        dev_dbg(&pdev->dev, "## Timeout Clock Frequency: %uKHz.\n",    hc_timeout_clk);

        mmc->f_min = clock_min;
        mmc->f_max = pinfo->clk_limit;
        dev_dbg(&pdev->dev, "## SD Clock: base[%uMHz], min[%uHz], max[%uHz].\n",
            SD_CAP_BASE_FREQ(hc_cap),
            mmc->f_min,
            mmc->f_max);

        if (hc_cap & SD_CAP_MAX_2KB_BLK)
            mmc->max_blk_size = 2048;
        else if (hc_cap & SD_CAP_MAX_1KB_BLK)
            mmc->max_blk_size = 1024;
        else if (hc_cap & SD_CAP_MAX_512B_BLK)
            mmc->max_blk_size = 512;

        //mmc->max_blk_size = 512;
        dev_dbg(&pdev->dev, "## SD max_blk_size: %u.\n", mmc->max_blk_size);

        mmc->caps = pslotinfo->plat_info->default_caps;
        if (pinfo->clk_limit > 25000000) {
            mmc->caps |= MMC_CAP_SD_HIGHSPEED;
            mmc->caps |= MMC_CAP_MMC_HIGHSPEED;
        }
        if (hc_cap & SD_CAP_DMA) {
            dev_dbg(&pdev->dev, "HW support DMA!\n");
        } else {
            dev_dbg(&pdev->dev, "HW do not support DMA!\n");
            retval = -ENODEV;
            goto gk_sd_probe_free_host;
        }
        if (hc_cap & SD_CAP_SUS_RES) {
            dev_dbg(&pdev->dev,  "HW support Suspend/Resume!\n");
        } else {
            dev_dbg(&pdev->dev, "HW do not support Suspend/Resume!\n");
        }

        mmc->ocr_avail = 0;
        if (hc_cap & SD_CAP_VOL_3_3V) {
            mmc->ocr_avail |= MMC_VDD_32_33 | MMC_VDD_33_34;
        }
        if (mmc->ocr_avail == 0) {
            dev_dbg(&pdev->dev, "Hardware report wrong voltages[0x%x].!\n",
                hc_cap);
            retval = -ENODEV;
            goto gk_sd_probe_free_host;
        }

        if (hc_cap & SD_CAP_INTMODE) {
            dev_dbg(&pdev->dev, "HW support Interrupt mode!\n");
        } else {
            dev_dbg(&pdev->dev, "HW do not support Interrupt mode!\n");
            retval = -ENODEV;
            goto gk_sd_probe_free_host;
        }

        mmc->max_blk_count = 0xFFFF;
        mmc->max_seg_size = pinfo->max_blk_sz;
        mmc->max_segs = mmc->max_seg_size / PAGE_SIZE;
        mmc->max_req_size = min(mmc->max_seg_size, mmc->max_blk_size * mmc->max_blk_count);
        dev_dbg(&pdev->dev, "## max req size = %d \n", mmc->max_req_size);
        pslotinfo->buf_vaddress = kmem_cache_alloc(pinfo->buf_cache, GFP_KERNEL);
        if (!pslotinfo->buf_vaddress) {
            gk_sd_err(pslotinfo, "Can't alloc DMA memory");
            retval = -ENOMEM;
            goto gk_sd_probe_free_host;
        }
        pslotinfo->buf_paddress = dma_map_single(
            pinfo->dev, pslotinfo->buf_vaddress,
            mmc->max_req_size, DMA_BIDIRECTIONAL);

        if (gk_sd_check_dma_boundary(pslotinfo,
            pslotinfo->buf_paddress, mmc->max_req_size,
            pinfo->max_blk_sz) == 0) {
            dev_dbg(&pdev->dev, "DMA boundary err!\n");
            retval = -ENOMEM;
            goto gk_sd_probe_free_host;
        }

        dev_dbg(&pdev->dev, "Slot%u use bounce buffer["
            "0x%p<->0x%08x]\n", pslotinfo->slot_id,
            pslotinfo->buf_vaddress,
            pslotinfo->buf_paddress);

        pslotinfo->plat_info->active_caps = mmc->caps;

        dev_dbg(&pdev->dev, "SD caps: 0x%lx.\n", mmc->caps);
        dev_dbg(&pdev->dev, "SD ocr: 0x%x.\n", mmc->ocr_avail);
        dev_notice(&pdev->dev, "Slot%u req_size=0x%08X, "
            "segs=%u, seg_size=0x%08X\n",
            pslotinfo->slot_id, mmc->max_req_size,
            mmc->max_segs, mmc->max_seg_size);
/*
        if (pslotinfo->plat_info->ext_power.gpio_id != -1) {
            retval = gpio_request(
                pslotinfo->plat_info->ext_power.gpio_id,
                pdev->name);
            if (retval < 0) {
                gk_sd_err(pslotinfo, "Can't get Power GPIO%d\n",
                pslotinfo->plat_info->ext_power.gpio_id);
                pslotinfo->plat_info->ext_power.gpio_id = -1;
            }
        }

        if (pslotinfo->plat_info->ext_reset.gpio_id != -1) {
            retval = gpio_request(
                pslotinfo->plat_info->ext_reset.gpio_id,
                pdev->name);
            if (retval < 0) {
                gk_sd_err(pslotinfo, "Can't get Reset GPIO%d\n",
                pslotinfo->plat_info->ext_reset.gpio_id);
                pslotinfo->plat_info->ext_reset.gpio_id = -1;
            }
        }

        if (pslotinfo->plat_info->gpio_wp.gpio_id != -1) {
            retval = gpio_request(
                pslotinfo->plat_info->gpio_wp.gpio_id,
                pdev->name);
            if (retval < 0) {
                gk_sd_err(pslotinfo, "Can't get WP GPIO%d\n",
                pslotinfo->plat_info->gpio_wp.gpio_id);
                pslotinfo->plat_info->gpio_wp.gpio_id = -1;
            }
        }
*/
        gk_sd_release_bus(mmc);
    }

    platform_set_drvdata(pdev, pinfo);

    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        pslotinfo->plat_info->pmmc_host = pslotinfo->mmc;
        pslotinfo->valid = 1;
        retval = mmc_add_host(pslotinfo->mmc);
        if (retval) {
            gk_sd_err(pslotinfo, "Can't add mmc host!\n");
        }
    }

    pslotinfo_next = pslotinfo+1;
    for(i=0;i<INFO_NUM-1;i++)
    {
        memcpy(pslotinfo_next,pslotinfo,sizeof(struct gk_sd_mmc_info));
        pslotinfo_next->buf_vaddress = pslotinfo->buf_vaddress + mmc->max_req_size*(i+1);
        if(i==INFO_NUM-2)
            pslotinfo_next->buf_paddress = dma_map_single(
                pinfo->dev, pslotinfo_next->buf_vaddress,
                SD_MIN_BUF, DMA_BIDIRECTIONAL);
        else
            pslotinfo_next->buf_paddress = dma_map_single(
                pinfo->dev, pslotinfo_next->buf_vaddress,
                mmc->max_req_size, DMA_BIDIRECTIONAL);
        init_waitqueue_head(&pslotinfo_next->wait);

        pslotinfo_next = pslotinfo_next+1;
    }
    pinfo->pslotinfo_cur = pslotinfo;

    retval = request_irq(pinfo->irq, gk_sd_irq,
                            IRQF_SHARED | IRQF_TRIGGER_HIGH,
                            "sd_control", pinfo);
    if (retval) {
        dev_err(&pdev->dev, "Can't Request IRQ%u!\n", pinfo->irq);
        goto gk_sd_probe_free_host;
    }

    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        pslotinfo->system_event.notifier_call =    gk_sd_system_event;
        gk_register_event_notifier(&pslotinfo->system_event);

        //if (gk_is_valid_gpio_irq( &pslotinfo->plat_info->gpio_cd))
        {
            /*  louis add
            retval = gpio_request(
                pslotinfo->plat_info->gpio_cd.irq_gpio,
                pdev->name);
            if (retval < 0) {
                gk_sd_err(pslotinfo, "Can't get CD GPIO%d\n",
                pslotinfo->plat_info->gpio_cd.irq_gpio);
                pslotinfo->plat_info->gpio_cd.irq_gpio = -1;
                continue;
            }
            */
            gk_sd_gpio_cd_check_val(pslotinfo);
            retval = request_irq(
                                pslotinfo->plat_info->gpio_cd.irq_line,
                                gk_sd_gpio_cd_irq,
                                pslotinfo->plat_info->gpio_cd.irq_type,
                                "sd_card_detect", pslotinfo);
            if (retval) {
                printk(
                    "Can't Request GPIO(%d) CD IRQ(%d)!\n",
                    pslotinfo->plat_info->gpio_cd.irq_gpio,
                    pslotinfo->plat_info->gpio_cd.irq_line);
            }
            //else printk("-- request detect irq %d ok\n", pslotinfo->plat_info->gpio_cd.irq_line);
        }
    }

    dev_notice(&pdev->dev,
        "GK SD/MMC[%d] has %u slots @ %uHz, [0x%08x:0x%08x]\n",
        pdev->id, pinfo->pcontroller->num_slots,
        pinfo->clk_limit, hc_cap, pinfo->dma_fix);
    retval = 0;

    gk_host[pdev->id] = pslotinfo;

    goto gk_sd_probe_na;

gk_sd_probe_free_host:
    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        if (pslotinfo->plat_info->gpio_wp.gpio_id != -1)
            gpio_free(pslotinfo->plat_info->gpio_wp.gpio_id);
        if (pslotinfo->plat_info->ext_power.gpio_id != -1)
            gpio_free(pslotinfo->plat_info->ext_power.gpio_id);
        if (pslotinfo->plat_info->ext_reset.gpio_id != -1)
            gpio_free(pslotinfo->plat_info->ext_reset.gpio_id);

        pslotinfo_next = pslotinfo;
        for(i=0;i<INFO_NUM;i++)
        {
            if (pslotinfo_next->buf_paddress) {
                dma_unmap_single(pinfo->dev, pslotinfo_next->buf_paddress,
                    pslotinfo_next->mmc->max_req_size,
                    DMA_BIDIRECTIONAL);
                pslotinfo_next->buf_paddress = (dma_addr_t)NULL;
            }
            pslotinfo_next = pslotinfo_next + 1;
        }
        if (pslotinfo->buf_vaddress) {
            kmem_cache_free(pinfo->buf_cache,
                pslotinfo->buf_vaddress);
            pslotinfo_next = pslotinfo;
            for(i=0;i<INFO_NUM;i++)
            {
                pslotinfo_next->buf_vaddress = NULL;
                pslotinfo_next = pslotinfo_next + 1;
            }
        }
        if (pslotinfo->mmc) {
            mmc_free_host(pslotinfo->mmc);
        }
    }
    if (pinfo->buf_cache) {
        kmem_cache_destroy(pinfo->buf_cache);
        pinfo->buf_cache = NULL;
    }

gk_sd_probe_free_pinfo:
    kfree(pinfo);

gk_sd_probe_na:
    return retval;
}

static int __devexit gk_sd_remove(struct platform_device *pdev)
{
    int retval = 0;
    struct gk_sd_controller_info *pinfo;
    struct gk_sd_mmc_info *pslotinfo;
    u32 i;

    pinfo = platform_get_drvdata(pdev);

    if (pinfo) {
        platform_set_drvdata(pdev, NULL);

        free_irq(pinfo->irq, pinfo);

        for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
            pslotinfo = pinfo->pslotinfo[i];
            pslotinfo->plat_info->pmmc_host = NULL;
            gk_unregister_event_notifier(
                &pslotinfo->system_event);

            free_irq(pslotinfo->plat_info->gpio_cd.irq_line, pslotinfo);
            gpio_free(pslotinfo->plat_info->gpio_cd.irq_gpio);

            if (pslotinfo->mmc) {
                mmc_remove_host(pslotinfo->mmc);
            }
        }

        for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
            pslotinfo = pinfo->pslotinfo[i];
            if (pslotinfo->plat_info->ext_power.gpio_id != -1) {
                gpio_free(
                pslotinfo->plat_info->ext_power.gpio_id);
            }
            if (pslotinfo->plat_info->ext_reset.gpio_id != -1) {
                gpio_free(
                pslotinfo->plat_info->ext_reset.gpio_id);
            }
            if (pslotinfo->plat_info->gpio_wp.gpio_id != -1) {
                gpio_free(
                pslotinfo->plat_info->gpio_wp.gpio_id);
            }
            if (pslotinfo->buf_paddress) {
                dma_unmap_single(pinfo->dev,
                    pslotinfo->buf_paddress,
                    pslotinfo->mmc->max_req_size,
                    DMA_BIDIRECTIONAL);
                pslotinfo->buf_paddress = (dma_addr_t)NULL;
            }
            if (pslotinfo->buf_vaddress) {
                kmem_cache_free(pinfo->buf_cache,
                    pslotinfo->buf_vaddress);
                pslotinfo->buf_vaddress = NULL;
            }
            if (pslotinfo->mmc) {
                mmc_free_host(pslotinfo->mmc);
            }
        }
        if (pinfo->buf_cache) {
            kmem_cache_destroy(pinfo->buf_cache);
            pinfo->buf_cache = NULL;
        }
        kfree(pinfo);
    }

    dev_notice(&pdev->dev,
        "Remove GK Media Processor SD/MMC Host Controller.\n");

    return retval;
}

#ifdef CONFIG_PM
static int gk_sd_suspend(struct platform_device *pdev,
    pm_message_t state)
{
    int retval = 0;
    struct gk_sd_controller_info *pinfo;
    struct gk_sd_mmc_info *pslotinfo;
    u32 i;

    pinfo = platform_get_drvdata(pdev);

    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        if (pslotinfo->mmc) {
            retval = mmc_suspend_host(pslotinfo->mmc);
            if (retval) {
                gk_sd_err(pslotinfo,
                "mmc_suspend_host[%d] failed[%d]!\n",
                i, retval);
            }
        }
    }

    disable_irq(pinfo->irq);
    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        disable_irq(pslotinfo->plat_info->gpio_cd.irq_line);
    }

    dev_dbg(&pdev->dev, "%s exit with %d @ %d\n",
        __func__, retval, state.event);
    return retval;
}

static int gk_sd_resume(struct platform_device *pdev)
{
    int retval = 0;
    struct gk_sd_controller_info *pinfo;
    struct gk_sd_mmc_info *pslotinfo;
    u32 i;

    pinfo = platform_get_drvdata(pdev);
    pinfo->pcontroller->set_pll(pinfo->clk_limit);
    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        gk_sd_reset_all(pslotinfo->mmc);
        enable_irq(pslotinfo->plat_info->gpio_cd.irq_line);
    }
    enable_irq(pinfo->irq);

    for (i = 0; i < pinfo->pcontroller->num_slots; i++) {
        pslotinfo = pinfo->pslotinfo[i];
        if (pslotinfo->mmc) {
            retval = mmc_resume_host(pslotinfo->mmc);
            if (retval) {
                gk_sd_err(pslotinfo,
                "mmc_resume_host[%d] failed[%d]!\n",
                i, retval);
            }
        }
    }

    dev_dbg(&pdev->dev, "%s exit with %d\n", __func__, retval);

    return retval;
}
#endif

static struct platform_driver gk_sd_driver = {
    .probe        = gk_sd_probe,
    .remove        = __devexit_p(gk_sd_remove),
#ifdef CONFIG_PM
    .suspend    = gk_sd_suspend,
    .resume        = gk_sd_resume,
#endif
    .driver        = {
        .name    = "gk-sd",
        .owner    = THIS_MODULE,
    },
};

static int __init gk_sd_init(void)
{
    int retval = 0;

    retval = platform_driver_register(&gk_sd_driver);
    if (retval) {
        printk(KERN_ERR "%s: Register failed %d!\n",
            __func__, retval);
    }

    return retval;
}

static void __exit gk_sd_exit(void)
{
    platform_driver_unregister(&gk_sd_driver);
}

fs_initcall(gk_sd_init);
module_exit(gk_sd_exit);

MODULE_DESCRIPTION("GOKE Media Processor SD/MMC Host Controller");
MODULE_AUTHOR("Goke Microelectronics Inc.");
MODULE_LICENSE("GPL");
