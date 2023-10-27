/*******************************************************************************
 *
 * Copyright (c)
 *
 *  \file parser_cfg.c
 *
 *  \brief parse config file.
 * 
 *   create version  v 1.0.0  wangguixing
 *
 ******************************************************************************/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <libgen.h>
#include <time.h>
#include <getopt.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/time.h>

#include "crc32.h"
#include "parser_cfg.h"
#include "flash_ctl.h"
#include "md5.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define WR_FLASH_BPMS 98     /* ��дflash ������, ��λ�ֽ�ÿ����. */



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* �ṹ�����ڶ�ȡ flash_map.ini ���ñ�� [FIRMWARE]  ��ѡ�� */
struct firmware_info
{
    unsigned int flash_size;    /* ����flash ������С */
    unsigned int upgrade_num;   /* ��Ҫ������MTD ��������, ע�ⲻ��flash�����е�MTD�������� */
    char  magic_num[32];        /* ���� */
    char  image_ver[32];        /* ����汾�� */
    char  Platform[32];         /* ƽ̨���� */
    char  machine_type[32];     /* ��Ʒ���� */
};


/* �ṹ�����ڶ�ȡ flash_map.ini ���ñ�� [FLASH] ��ѡ�� */
struct flash_info
{
    unsigned int mtd_addr;      /* (16����) �����������flash�ռ�0��ַ��ƫ�Ƶ�ַ */
    unsigned int mtd_size;      /* (16����)Ҫ������MTD ��ռ�����Ĵ�С */
    unsigned int mtd_offset;    /* Ҫ���������ڷ����ڵı��˵�ַ, ��������flash��ƫ�Ƶ�ַ */
    unsigned int block;         /* Ҫ�����Ŀ��� (��ʱû��ʹ��) */
    unsigned int upgrade_flag;  /* ������־, 0:��ʾ������; 1:��ʾ����  */
    int   format_bin;           /* �������������ʽ�Ƿ�Ϊ bin   (��ʱû��ʹ��) */
    int   format_rom;           /* �������������ʽ�Ƿ�Ϊ rom   (��ʱû��ʹ��) */
    char  mtd_part[32];         /* Ҫ������MTD ������: /dev/mtd3   */
    char  mtd_name[32];         /* Ҫ������MTD ��������:  rootfs  */
    char  upgrade_file[96];     /* ��Ҫ�������ļ���ַ·�� */
};


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static int       g_upgrade_start_flag = 0;
static long int  g_start_time         = 0;
static long int  g_upgrade_total_time = 0;
static unsigned int g_upgrade_rate    = 0;


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void set_upgrade_start(unsigned int file_size)
{
    struct timeval cur_time;
    memset(&cur_time, 0, sizeof(cur_time));

    g_upgrade_rate         = 0;
    g_upgrade_start_flag   = 1;
    g_upgrade_total_time = file_size / WR_FLASH_BPMS;

    gettimeofday(&cur_time, NULL);
    g_start_time = (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
    DEBUG_PRT("g_start_time:%ldms  tv_sec:%ld  tv_usec:%ld \n", 
                                              g_start_time,
                                              cur_time.tv_sec, 
                                              cur_time.tv_usec);
}

static void set_upgrade_end(void)
{
    g_upgrade_rate       = 100;
    g_upgrade_start_flag = 0;
    DEBUG_PRT("upgrade flash end! rate:100%%\n");
}

/******************************************************************************
 * ��������:  ��ȡ��ǰ���Ȱٷֱ�
 * �������:  ��
 *                           
 * �������:  0--100��ֵ. ����ʾ 0%--100%
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
unsigned int get_upgrade_rate(void)
{
    int            ret          = 0;
    long int       tmp_time     = 0;
    long int       user_time    = 0;
    struct timeval cur_time;
    memset(&cur_time, 0, sizeof(cur_time));

    if (g_upgrade_start_flag)
    {
        gettimeofday(&cur_time, NULL);
        tmp_time = (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
        if (tmp_time >= g_start_time)
        {
            user_time    = tmp_time - g_start_time;
            g_upgrade_rate = (user_time * 100) / g_upgrade_total_time;

            /* ������ɵĽ�����ʱ������, ʵ�������Ҫ����
                       * set_upgrade_end ��ȷ�����ʱ��.��˹�����������Ϊ100%
                       */
            if (g_upgrade_rate > 98)
            {
                g_upgrade_rate = 98;
            }
        }
        else
        {
            g_start_time = tmp_time;
        }
    }

    DEBUG_PRT("Current upgrade flag:%d total:%ld  start:%ld  curr:%ld  used:%ld  rate:%u%% \n", 
                                                g_upgrade_start_flag,
                                                g_upgrade_total_time,
                                                g_start_time,
                                                tmp_time,
                                                user_time,
                                                g_upgrade_rate);
    
    if (g_upgrade_rate > 100)
    {
        g_upgrade_rate = 100;
    }

    return g_upgrade_rate;
}

static int printf_md5(const char *md5, char *md5str)
{
    if (NULL == md5||NULL == md5str)
    {
        ERROR_PRT("md5 is NULL error!\n");
        return -1;
    }
    int i;
    for(i=0; i<16; i++)
    {
        sprintf(md5str+i*2,"%02hhx", md5[i]);
    }
    return 0;
}

/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
static int printf_upgrade_header_info(const struct gk_upgrade_header *pheader)
{
    if (NULL == pheader)
    {
        ERROR_PRT("pheader is NULL error!\n");
        return -1;
    }
	char md5str[64] = {0};
    printf_md5(pheader->pack_md5, md5str);

    DEBUG_PRT("--> magic_num   :%s \n",   pheader->magic_num);
    DEBUG_PRT("--> image_ver   :%s \n",   pheader->image_ver);
    DEBUG_PRT("--> Platform    :%s \n",   pheader->Platform);
    DEBUG_PRT("--> machine_type:%s \n",   pheader->machine_type);
    DEBUG_PRT("--> flash_size  :0x%x \n", pheader->flash_size);
    DEBUG_PRT("--> upgrade_flag:%d \n",   pheader->upgrade_flag);
    DEBUG_PRT("--> mtd_part    :%s \n",   pheader->mtd_part);
    DEBUG_PRT("--> mtd_name    :%s \n",   pheader->mtd_name);
    DEBUG_PRT("--> mtd_addr    :0x%x \n", pheader->mtd_addr);
    DEBUG_PRT("--> mtd_offset  :0x%x \n", pheader->mtd_offset);
    DEBUG_PRT("--> mtd_size    :0x%x \n", pheader->mtd_size);
    DEBUG_PRT("--> file_size   :0x%x \n", pheader->file_size);
    DEBUG_PRT("--> file_crc32  :%x \n",   pheader->file_crc32);
	DEBUG_PRT("--> pack_md5    :%s \n\n", md5str);

    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
static int parse_firmware_info(char *file_name, struct firmware_info *pfw_info)
{
    int    ret      = 0;
    int    line_num = 0;
    char  *pret     = NULL;
    FILE  *fp       = NULL;
    char   line_buf[256]  = {0};
    int    parse_flag  = 0;

    if (NULL == file_name || NULL == pfw_info)
    {
        ERROR_PRT("file_name:%p  pfw_info:%p error!\n", file_name, pfw_info);
        return -1;
    }

    fp = fopen(file_name, "r");
    if (NULL == fp)
    {
        ERROR_PRT("fopen %s ERROR! %s\n", file_name, strerror(errno));
        return -1;
    }

    parse_flag  = 0;
    while(1)
    {
        memset(line_buf, 0, sizeof(line_buf));

        /* read flash_map.ini  file */
        pret = fgets(line_buf, sizeof(line_buf) - 1, fp);
        if(NULL == pret)
        {
            /* û�н�����[FIRMWARE_END] �ͽ�����, ˵�������ļ������� */
            ERROR_PRT("The section if not [FIRMWARE_END] end! error!\n");
            fclose(fp);
            return -1;
        }

        /* 1. ����FIRMWARE �Ŀ�ʼ�� */
        if (NULL != (pret = strstr(line_buf, FIRMWARE_START)))
        {
            if (0 == parse_flag)
            {
                DEBUG_PRT("==========  parse [FIRMWARE_START] section start  =========\n");
                parse_flag = 1;
                continue;
            }
            else
            {
                /* ���ν�����[FIRMWARE] ���� */
                ERROR_PRT("twice parsed [FIRMWARE] section error!\n");
                fclose(fp);
                return -1;
            }
        }

        /* 2. ����FIRMWARE �Ľ����� */
        if (NULL != (pret = strstr(line_buf, FIRMWARE_END)))
        {
            DEBUG_PRT("==========  parse [FIRMWARE_END] section end!  =========\n\n");
            fclose(fp);
            return 0;
        }

        /* 3. ��������[FIRMWARE] ��ʼ����ÿ���ֶε�ֵ */
        if (parse_flag)
        {
            if (NULL != (pret = strstr(line_buf, "magic=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pfw_info->magic_num);
                DEBUG_PRT("parse [magic_num]:\t\t%s\n", pfw_info->magic_num);

                /* У����� check magic_num  */
                if (NULL == strstr(pfw_info->magic_num, MAGIC_NUM))
                {
                    ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
                    fclose(fp);
                    return 0;
                }
            }
            else if (NULL != (pret = strstr(line_buf, "Platform=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pfw_info->Platform);
                DEBUG_PRT("parse [Platform]:\t\t%s\n", pfw_info->Platform);
            }
            else if (NULL != (pret = strstr(line_buf, "machine_type=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pfw_info->machine_type);
                DEBUG_PRT("parse [machine_type]:\t%s\n", pfw_info->machine_type);
            }
            else if (NULL != (pret = strstr(line_buf, "image_ver=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pfw_info->image_ver);
                DEBUG_PRT("parse [image_ver]:\t\t%s\n", pfw_info->image_ver);
            }
            else if (NULL != (pret = strstr(line_buf, "flash_size=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pfw_info->flash_size));
                DEBUG_PRT("parse [flash_size]:\t\t0x%x (%dKB)\n", pfw_info->flash_size, pfw_info->flash_size/1024);
                /* nor_flash size �������2MB  */
                if(pfw_info->flash_size < MIMI_NORFLASH_SIZE
                   || pfw_info->flash_size > MAX_NORFLASH_SIZE)
                {
                    ERROR_PRT("flash_size 0x%x (%dKB) error!\n", 
                                                    pfw_info->flash_size, pfw_info->flash_size/1024);
                    fclose(fp);
                    return -1;
                }
            }
            else if (NULL != (pret = strstr(line_buf, "upgrade_num=")))
            {
                sscanf(line_buf, "%*[^=]=%d", &(pfw_info->upgrade_num));
                DEBUG_PRT("parse [upgrade_num]:\t%d\n", pfw_info->upgrade_num);
                /* upgrade_num ���ܴ���16������  */
                if(pfw_info->upgrade_num > MAX_MTD_NUM)
                {
                    ERROR_PRT("upgrade_num:%u is too more MAX_MTD_NUM[%d]! error! \n", pfw_info->upgrade_num, MAX_MTD_NUM);
                    fclose(fp);
                    return -1;
                }
            }
            else
            {
                ERROR_PRT("line:%s is don't parse!\n", line_buf);
            }
        }
    }

    fclose(fp);
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
static int parse_flash_info(char *file_name, char *flash_start, char *flash_end, struct flash_info *pflash)
{
    int    ret      = 0;
    int    line_num = 0;
    char  *pret     = NULL;
    FILE  *fp       = NULL;
    char   line_buf[256]  = {0};
    int    parse_flag  = 0;

    if (NULL == file_name || NULL == pflash || NULL == flash_start || NULL == flash_end)
    {
        ERROR_PRT("file_name:%p  pflash:%p  flash_start:%p  flash_end:%p error!\n", file_name, pflash, flash_start, flash_end);
        return -1;
    }

    fp = fopen(file_name, "r");
    if (NULL == fp)
    {
        ERROR_PRT("fopen %s ERROR!\n", file_name);
        return -1;
    }

    parse_flag  = 0;
    while(1)
    {
        memset(line_buf, 0, sizeof(line_buf));

        /* read flash_map.ini  file */
        pret = fgets(line_buf, sizeof(line_buf) - 1, fp);
        if(NULL == pret)
        {
            /* û�н�����[FIRMWARE_END] �ͽ�����, ˵�������ļ������� */
            ERROR_PRT("The section if not [FIRMWARE_END] end! error!\n");
            fclose(fp);
            return -1;
        }

        /* 1. ����[FLASH] �Ŀ�ʼ�� */
        if (NULL != (pret = strstr(line_buf, flash_start)))
        {
            if (0 == parse_flag)
            {
                DEBUG_PRT("==========  parse %s section start  =========\n", flash_start);
                parse_flag = 1;
                continue;
            }
            else
            {
                /* ���ν�����[FLASH] ���� */
                ERROR_PRT("twice parsed [FIRMWARE] section error!\n");
                fclose(fp);
                return -1;
            }
        }

        /* 2. ����[FLASH_END] �Ľ����� */
        if (NULL != (pret = strstr(line_buf, flash_end)))
        {
            DEBUG_PRT("==========  parse %s section end  =========\n\n", flash_end);
            fclose(fp);
            return 0;
        }

        /* 3. ��������[FLASH] ��ʼ����ÿ���ֶε�ֵ */
        if (parse_flag)
        {
            if (NULL != (pret = strstr(line_buf, "mtd_part=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pflash->mtd_part);
                DEBUG_PRT("parse [mtd_part]:\t%s\n", pflash->mtd_part);
            }
            else if (NULL != (pret = strstr(line_buf, "mtd_name=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pflash->mtd_name);
                DEBUG_PRT("parse [mtd_name]:\t%s\n", pflash->mtd_name);
            }
            else if (NULL != (pret = strstr(line_buf, "mtd_addr=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pflash->mtd_addr));
                DEBUG_PRT("parse [mtd_addr]:\t0x%x (%dKB)\n", pflash->mtd_addr, pflash->mtd_addr/1024);
                /* pflash->mtd_addr ���ܴ���32MB  */
                if(pflash->mtd_addr >= MAX_MTD_SIZE)
                {
                    ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                                    pflash->mtd_addr, pflash->mtd_addr/1024);
                    fclose(fp);
                    return -1;
                }
            }
            else if (NULL != (pret = strstr(line_buf, "mtd_size=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pflash->mtd_size));
                DEBUG_PRT("parse [mtd_size]:\t0x%x (%dKB)\n", pflash->mtd_size, pflash->mtd_size/1024);
                /* pflash->mtd_size ���ܴ���32MB  */
                if(pflash->mtd_size > MAX_MTD_SIZE)
                {
                    ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                                    pflash->mtd_size, pflash->mtd_size/1024);
                    fclose(fp);
                    return -1;
                }
            }
            else if (NULL != (pret = strstr(line_buf, "mtd_offset=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pflash->mtd_offset));
                DEBUG_PRT("parse [mtd_offset]:\t0x%x (%dKB)\n", pflash->mtd_offset, pflash->mtd_offset/1024);
                /* pflash->mtd_offset ���ܴ���32MB  */
                if(pflash->mtd_offset > MAX_MTD_SIZE)
                {
                    ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                                    pflash->mtd_offset, pflash->mtd_offset/1024);
                    fclose(fp);
                    return -1;
                }
            }
            else if (NULL != (pret = strstr(line_buf, "block=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pflash->block));
                DEBUG_PRT("parse [block]:\t\t%d\n", pflash->block);
            }
            else if (NULL != (pret = strstr(line_buf, "upgrade_file=")))
            {
                sscanf(line_buf, "%*[^=]=%s", pflash->upgrade_file);
                DEBUG_PRT("parse [upgrade_file]:\t%s\n", pflash->upgrade_file);
            }
            else if (NULL != (pret = strstr(line_buf, "format_rom=")))
            {
                sscanf(line_buf, "%*[^=]=%d", &(pflash->format_rom));
                DEBUG_PRT("parse [format_rom]:\t0x%x\n", pflash->format_rom);
            }
            else if (NULL != (pret = strstr(line_buf, "format_bin=")))
            {
                sscanf(line_buf, "%*[^=]=%d", &(pflash->format_bin));
                DEBUG_PRT("parse [format_bin]:\t0x%x\n", pflash->format_bin);
            }
            else if (NULL != (pret = strstr(line_buf, "upgrade_flag=")))
            {
                sscanf(line_buf, "%*[^=]=%x", &(pflash->upgrade_flag));
                DEBUG_PRT("parse [upgrade_flag]:\t%d\n", pflash->upgrade_flag);
            }
            else
            {
                ERROR_PRT("line:%s is don't parse!\n", line_buf);
            }
        }
    }

    fclose(fp);
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
static int get_upgradeinfo_by_cfgfile(char *file_name, struct firmware_info *pfw, struct flash_info *pfh)
{
    int    ret      = 0;
    int    cnt      = 0;
    FILE  *fp       = NULL;
    char  *pret     = NULL;
    char   line_buf[256]  = {0};
    char   start_line[32] = {0};
    char   end_line[32]   = {0};

    if (NULL == file_name || NULL == pfw || NULL == pfh)
    {
        ERROR_PRT("file_name:%p  pfw:%p  pfh:%p  \n", file_name, pfw, pfh);
        return -1;
    }

    ret = parse_firmware_info(file_name, pfw);
    if (ret < 0)
    {
        ERROR_PRT("Do parse_firmware_info error!  ret:%d\n", ret);
        return -1;
    }

    for (cnt = 0; cnt < pfw->upgrade_num; cnt++)
    {
        memset(start_line, 0, sizeof(start_line));
        memset(end_line,   0, sizeof(end_line));
        sprintf(start_line, "[FLASH%d]",     cnt);
        sprintf(end_line,   "[FLASH%d_END]", cnt);
        ret = parse_flash_info(file_name, start_line, end_line, (pfh+cnt));
        if (ret < 0)
        {
            ERROR_PRT("parse %s  section:%s error! ret:%d \n", file_name, start_line, ret);
            return -1;
        }
    }

    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int get_mtdinfo_by_proc_mtd(struct proc_mtd_info *pmtd_info, unsigned int *pmtd_num)
{
    int    ret  = 0;
    int    cnt  = 0;
    FILE  *fp   = NULL;
    char  *pret = NULL;
    char   line_buf[256];
    char   name[32]    = {0};
    char   mtd_part[32] = {0};
    signed int size = 0;
    signed int erase_size = 0;
    signed int addr_offset = 0;

    if (NULL == pmtd_info || NULL == pmtd_num)
    {
        ERROR_PRT("mtd_info:%p  mtd_num:%p is NULL !\n", pmtd_info, pmtd_num);
        return -1;
    }

    fp = fopen(MTD_PATH, "r");
    if (NULL == fp)
    {
        ERROR_PRT("fopen %s ERROR!\n", MTD_PATH);
        return -1;
    }

    while(1)
    {
        memset(line_buf, 0, sizeof(line_buf));
        memset(name,     0, sizeof(name));
        memset(mtd_part, 0, sizeof(mtd_part));

        /* read /proc/mtd file end or error! */
        pret = fgets(line_buf, sizeof(line_buf) - 1, fp);
        if(NULL == pret)
            break;

        /* filter title line: "dev:    size   erasesize  name"  */
        if (NULL == (pret = strstr(line_buf, "mtd")))
            continue;

        cnt++;
        if (cnt >= MAX_MTD_NUM)
        {
            ERROR_PRT("MTD partition more than %d, error!\n", MAX_MTD_NUM);
            fclose(fp);
            return -1;
        }

        /* parse the line: " mtd0: 00030000 00010000 "uboot" "  */
        
        sscanf(line_buf, "%[^:]: %x %x%*[^\"]\"%[^\"]", mtd_part, &size, &erase_size, name);
        pmtd_info[cnt].mtd_size = size;
        pmtd_info[cnt].mtd_addr = addr_offset;
        strcpy(pmtd_info[cnt].mtd_part, "/dev/");
        strcat(pmtd_info[cnt].mtd_part, mtd_part);
        strncpy(pmtd_info[cnt].mtd_name, name, sizeof(name)-1);
        addr_offset += size;
        
        DEBUG_PRT("cnt[%d]  mtd_part:%s  size:0x%x  mtd_addr:0x%x  name:%s \n", 
                                                    cnt, pmtd_info[cnt].mtd_part, 
                                                    pmtd_info[cnt].mtd_size, 
                                                    pmtd_info[cnt].mtd_addr, 
                                                    pmtd_info[cnt].mtd_name);
    }

    *pmtd_num = cnt;
    DEBUG_PRT("The total mtd_num:%d \n\n", cnt);

    fclose(fp);
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int get_all_mtd_info_by_proc(struct proc_mtd_info *pall_mtd)
{
    int    ret  = 0;
    int    cnt  = 0;
    FILE  *fp   = NULL;
    char  *pret = NULL;
    char   line_buf[256];
    char   name[32]    = {0};
    char   mtd_part[32] = {0};
    signed int size = 0;
    signed int erase_size = 0;

    if (NULL == pall_mtd)
    {
        ERROR_PRT("pall_mtd:%p is NULL !\n", pall_mtd);
        return -1;
    }

    fp = fopen(MTD_PATH, "r");
    if (NULL == fp)
    {
        ERROR_PRT("fopen %s ERROR!\n", MTD_PATH);
        return -1;
    }

    while(1)
    {
        memset(line_buf, 0, sizeof(line_buf));
        memset(name,     0, sizeof(name));
        memset(mtd_part, 0, sizeof(mtd_part));

        /* read /proc/mtd file end or error! */
        pret = fgets(line_buf, sizeof(line_buf) - 1, fp);
        if(NULL == pret)
        {
            ERROR_PRT("Don't find the all_mtd partition in /proc/mtd .\n");
            fclose(fp);
            return -1;
        }

        /* filter title line: "dev:    size   erasesize  name"  */
        if (NULL == (pret = strstr(line_buf, "mtd")))
            continue;

        cnt++;
        if (cnt >= MAX_MTD_NUM)
        {
            ERROR_PRT("MTD partition more than %d, error!\n", MAX_MTD_NUM);
            fclose(fp);
            return -1;
        }

        /* parse the line:  mtd0: 00030000 00010000 "all "  */
        if (NULL != strstr(line_buf, "all"))
        {
            sscanf(line_buf, "%[^:]: %x %x%*[^\"]\"%[^\"]", mtd_part, &size, &erase_size, name);
            pall_mtd->mtd_size = size;
            pall_mtd->mtd_addr = 0;
            strcpy(pall_mtd->mtd_part, "/dev/");
            strcat(pall_mtd->mtd_part, mtd_part);
            strncpy(pall_mtd->mtd_name, name, sizeof(name)-1);

            DEBUG_PRT("cnt[%d]  all_mtd_part:%s  size:0x%x  mtd_addr:0x%x  name:%s \n", 
                                                        cnt, pall_mtd->mtd_part, 
                                                        pall_mtd->mtd_size, 
                                                        pall_mtd->mtd_addr, 
                                                        pall_mtd->mtd_name);
            fclose(fp);
            return 0;
        }
    }

    fclose(fp);
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
static int fill_upgrade_header(struct firmware_info *pfw, struct flash_info *pfh, struct gk_upgrade_header *pheader)
{
    int    ret  = 0;
    int    cnt  = 0;
    int    fd   = -1;
    char  *pret = NULL;
    char  *pbuf = NULL;
    char   line_buf[256];
    char   name[40] = {0};
    unsigned int tmp_size = 0;
	MD5_CTX md5;
    
    memset(&md5, 0, sizeof(md5));
    
    if (NULL == pfw || NULL == pfh || NULL == pheader)
    {
        ERROR_PRT("pfw:%p  pfh:%p  pheader:%p is NULL!\n", pfw, pfh, pheader);
        return -1;
    }

    if (pfw->upgrade_num > MAX_MTD_NUM)
    {
        ERROR_PRT("MTD upgrade_num:%d more than %d, error!\n", pfw->upgrade_num, MAX_MTD_NUM);
        return -1;
    }

    if (NULL == pfh->upgrade_file)
    {
        ERROR_PRT("pfh->upgrade_file is NULL!\n");
        return -1;
    }

    strncpy(pheader->magic_num,    pfw->magic_num,    sizeof(pheader->magic_num)-1);
    strncpy(pheader->image_ver,    pfw->image_ver,    sizeof(pheader->image_ver)-1);
    strncpy(pheader->Platform,     pfw->Platform,     sizeof(pheader->Platform)-1);
    strncpy(pheader->machine_type, pfw->machine_type, sizeof(pheader->machine_type)-1);
    pheader->flash_size = pfw->flash_size;

    memcpy(pheader->pack_md5, pheader->machine_type, sizeof(pheader->pack_md5));
    strncpy(pheader->mtd_part,  pfh->mtd_part, sizeof(pheader->mtd_part)-1);
    strncpy(pheader->mtd_name,  pfh->mtd_name, sizeof(pheader->mtd_name)-1);
    pheader->mtd_addr     = pfh->mtd_addr;
    pheader->mtd_offset   = pfh->mtd_offset;
    pheader->mtd_size     = pfh->mtd_size;
    pheader->upgrade_flag = pfh->upgrade_flag;

    if (pfw->flash_size > MAX_MTD_SIZE)
    {
        ERROR_PRT("pfw->flash_size:0x%x is too large! MAX_MTD_SIZE:0x%x\n", pfw->flash_size, MAX_MTD_SIZE);
        return -1;
    }

    tmp_size = pfh->mtd_addr + pfh->mtd_offset + pfh->mtd_size;
    if (tmp_size > pfw->flash_size)
    {
        ERROR_PRT("mtd_name:%s mtd_addr:0x%x + mtd_offset:0x%x + mtd_size:0x%x = 0x%x > flash_size:0x%x  error!\n", 
                    pfh->mtd_name,
                    pfh->mtd_addr, pfh->mtd_offset, pfh->mtd_size,
                    tmp_size, pfw->flash_size);
        return -1;
    }

    fd = open(pfh->upgrade_file, O_RDONLY);
    if (fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", pfh->upgrade_file, strerror(errno));
        return -1;
    }
    pheader->file_size = lseek(fd, 0, SEEK_END);
    if (pheader->file_size < 0)
	{
        ERROR_PRT("lseek %s ERROR!\n", pfh->upgrade_file);
        close(fd);
        return -1;
    }
    lseek(fd, 0, SEEK_SET);

    /* У�������ļ���С */
    if(pheader->file_size > MAX_MTD_SIZE || pheader->file_size > pheader->mtd_size)
    {
        ERROR_PRT("upgrade file_size error! MAX_MTD_SIZE:0x%x  file_size:0x%x  mtd_size:0x%x\n", 
                                            MAX_MTD_SIZE, pheader->file_size, pheader->mtd_size);
        close(fd);
        return -1;
    }

    pbuf = malloc(pheader->file_size + 2);
    if (NULL == pbuf)
    {
        ERROR_PRT("malloc pheader->file_size:%u ERROR!\n", pheader->file_size);
        close(fd);
        return -1;
    }

    ret = read(fd, pbuf, pheader->file_size);
    if (ret <= 0 || ret != pheader->file_size)
    {
        ERROR_PRT("pheader->file_size:%u ret:%d %s\n", pheader->file_size, ret, strerror(errno));
        free(pbuf);
        close(fd);
        return -1;
    }
    pheader->file_crc32 = mtd_crc32(0, pbuf, pheader->file_size);
	
	MD5Init(&md5);
	MD5Update(&md5, (char *)pheader, sizeof(GK_UPGRADE_HEADER_S));
	MD5Update(&md5, pbuf, pheader->file_size);
    MD5Final(&md5, pheader->pack_md5);
	
    printf_upgrade_header_info(pheader);

    free(pbuf);
    close(fd);
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int pack_product_file(char *config_file, char *upgrade_bin)
{
    int    ret        = 0;
    int    cnt        = 0;
    int    image_fd   = -1;
    int    pack_fd    = -1;
    char   buff[1024] = {0};
    unsigned int             total_size = 0;
    unsigned int             file_size  = 0;
    struct firmware_info     fw_info;
    struct flash_info        fl_info[MAX_MTD_NUM];
    struct gk_upgrade_header upgrade_header[MAX_MTD_NUM];

    memset(&fw_info, 0, sizeof(fw_info));
    memset(&fl_info, 0, sizeof(fl_info));
    memset(&upgrade_header, 0, sizeof(upgrade_header));

    if (NULL == config_file || NULL == upgrade_bin)
    {
        ERROR_PRT("config_file:%p or upgrade_file:%p is NULL!\n", config_file, upgrade_bin);
        return -1;
    }

    /* ��ȡ����flash_map.ini �����ļ���Ϣ */
    ret = get_upgradeinfo_by_cfgfile(config_file, &fw_info, fl_info);
    if (ret < 0)
    {
        ERROR_PRT("Call get_upgradeinfo_by_cfgfile error! ret:%d\n", ret);
        return -1;
    }

    /* �����������ͷ�ṹ�� , ��������ʱ�������Ϣ*/
    for (cnt = 0; cnt < fw_info.upgrade_num; cnt++)
    {
        ret = fill_upgrade_header(&fw_info, &fl_info[cnt], &upgrade_header[cnt]);
        if (ret < 0)
        {
            ERROR_PRT("Call fill_upgrade_header error! cnt:%d ret:%d\n", cnt, ret);
            return -1;
        }
    }

    /* �����ָ������������, ���ϳ�һ��������upgrade.bin �ļ� */
    pack_fd = open(upgrade_bin, O_RDWR|O_CREAT);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_bin, strerror(errno));
        return -1;
    }

    total_size = 0;
    for (cnt = 0; cnt < fw_info.upgrade_num; cnt++)
    {
        /* �����ļ���С��Ӧ���� flash ������С */
        if (total_size + upgrade_header[cnt].file_size > fw_info.flash_size)
        {
            ERROR_PRT("cur_size:%u + file_size:%u > flash_size:%u\n", total_size,
                                                                      upgrade_header[cnt].file_size,
                                                                      fw_info.flash_size);
            ret = -1;
            goto pack_flash_error;
        }

        file_size = 0;
        image_fd = open(fl_info[cnt].upgrade_file, O_RDONLY);
        if (image_fd < 0)
        {
            ERROR_PRT("open %s ERROR! %s\n", fl_info[cnt].upgrade_file, strerror(errno));
            ret = -1;
            goto pack_flash_error;
        }

        /* д�������ļ����� */
        while(1)
        {
            /* �����ļ���С��Ӧ���� flash ������С */
            if (total_size >= fw_info.flash_size)
            {
                DEBUG_PRT("cur_size:%u >= flash_size:%u\n", total_size, fw_info.flash_size);
                ret = 0;
                goto pack_flash_error;
            }

            memset(buff, 0, sizeof(buff));
            ret = read(image_fd, buff, sizeof(buff));
            if (ret <= 0)
            {
                if (file_size != upgrade_header[cnt].file_size)
                {
                    ERROR_PRT("file:%s size error! upgrade_header.file_size:%u  file_size:%u \n", 
                                        fl_info[cnt].upgrade_file, upgrade_header[cnt].file_size, file_size);
                    goto pack_flash_error;
                }
                DEBUG_PRT("read %s wr_size:[%u <-> %u] end! ret:%d\n", fl_info[cnt].upgrade_file, 
                                                                        upgrade_header[cnt].file_size,
                                                                        file_size, 
                                                                        ret);
                close(image_fd);
                image_fd = -1;
                break;
            }

            ret = write(pack_fd, buff, ret);
            if (ret <= 0)
            {
                ERROR_PRT("write %s error! %s\n", fl_info[cnt].upgrade_file, strerror(errno));
                goto pack_flash_error;
            }

            file_size  += ret;
            total_size += ret;
        }

        /* check ������С���Ա���д����ļ���С��������������С,
                *  ����0xff ���ʣ��ռ�.
                */
        if (file_size >= fl_info[cnt].mtd_size)
        {
            close(image_fd);
            image_fd = -1;
        }
        else if (file_size < fl_info[cnt].mtd_size)
        {
            memset(buff, 0xff, sizeof(buff));
            while (file_size < fl_info[cnt].mtd_size)
            {
                if ((fl_info[cnt].mtd_size - file_size) > sizeof(buff))
                {
                    ret = write(pack_fd, buff, sizeof(buff));
                }
                else
                {
                    ret = write(pack_fd, buff, (fl_info[cnt].mtd_size - file_size));
                }
                if (ret <= 0)
                {
                    ERROR_PRT("write %s  file_size:%u error! ret:%d %s\n", upgrade_bin, file_size, ret, strerror(errno));
                    goto pack_flash_error;
                }
                file_size += ret;
            }
        }
    }

    ret = 0;
pack_flash_error:
    if(image_fd > 0)
    {
        close(image_fd);
    }
    if(pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int pack_upgrade_file(char *config_file, char *upgrade_bin)
{
    int    ret        = 0;
    int    cnt        = 0;
    int    image_fd   = -1;
    int    pack_fd    = -1;
    char   buff[1024] = {0};
    unsigned int             file_size = 0;
    struct firmware_info     fw_info;
    struct flash_info        fl_info[MAX_MTD_NUM];
    struct gk_upgrade_header upgrade_header[MAX_MTD_NUM];

    memset(&fw_info, 0, sizeof(fw_info));
    memset(&fl_info, 0, sizeof(fl_info));
    memset(&upgrade_header, 0, sizeof(upgrade_header));
    
    if (NULL == config_file || NULL == upgrade_bin)
    {
        ERROR_PRT("config_file:%p or upgrade_file:%p is NULL!\n", config_file, upgrade_bin);
        return -1;
    }

    /* ��ȡ����flash_map.ini �����ļ���Ϣ */
    ret = get_upgradeinfo_by_cfgfile(config_file, &fw_info, fl_info);
    if (ret < 0)
    {
        ERROR_PRT("Call get_upgradeinfo_by_cfgfile error! ret:%d\n", ret);
        return -1;
    }

    /* �����������ͷ�ṹ�� , ��������ʱ�������Ϣ*/
    for (cnt = 0; cnt < fw_info.upgrade_num; cnt++)
    {
        /* ��������־Ϊ0, �򲻰Ѿ������������ļ����� */
        if (0 == fl_info[cnt].upgrade_flag)
        {
            DEBUG_PRT("The file:%s don't upgrade. don't fill the upgrade_header.\n", fl_info[cnt].upgrade_file);
            continue;
        }

        ret = fill_upgrade_header(&fw_info, &fl_info[cnt], &upgrade_header[cnt]);
        if (ret < 0)
        {
            ERROR_PRT("Call fill_upgrade_header error! cnt:%d ret:%d\n", cnt, ret);
            return -1;
        }
    }

    /* �����ָ������������, ���ϳ�һ��������upgrade.bin �ļ� */
    pack_fd = open(upgrade_bin, O_RDWR|O_CREAT);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_bin, strerror(errno));
        return -1;
    }
    for (cnt = 0; cnt < fw_info.upgrade_num; cnt++)
    {
        /* ��������־Ϊ0, �򲻰Ѿ������������ļ����� */
        if (0 == fl_info[cnt].upgrade_flag)
        {
            DEBUG_PRT("The file:%s don't upgrade. no pack in %s \n", 
                                          fl_info[cnt].upgrade_file, upgrade_bin);
            continue;
        }
        
        file_size = 0;
        image_fd = open(fl_info[cnt].upgrade_file, O_RDONLY);
        if (image_fd < 0)
        {
            ERROR_PRT("open %s ERROR! %s\n", fl_info[cnt].upgrade_file, strerror(errno));
            ret = -1;
            goto pack_bin_error;
        }

        /* д�������ṹ��ͷ�ļ���Ϣ */
        ret = write(pack_fd, &upgrade_header[cnt], sizeof(struct gk_upgrade_header));
        if (ret <= 0)
        {
            ERROR_PRT("write %s ERROR! %s\n", upgrade_bin, strerror(errno));
            goto pack_bin_error;
        }
        /* д�������ļ����� */
        while(1)
        {
            memset(buff, 0, sizeof(buff));
            ret = read(image_fd, buff, sizeof(buff));
            if (ret <= 0)
            {
                if (file_size != upgrade_header[cnt].file_size)
                {
                    ERROR_PRT("file:%s size error! upgrade_header.file_size:%u  file_size:%u \n", 
                                        fl_info[cnt].upgrade_file, upgrade_header[cnt].file_size, file_size);
                    goto pack_bin_error;
                }
                DEBUG_PRT("read %s file_size:%u end! %s\n", fl_info[cnt].upgrade_file, file_size, strerror(errno));
                close(image_fd);
                image_fd = -1;
                break;
            }

            ret = write(pack_fd, buff, ret);
            if (ret <= 0)
            {
                ERROR_PRT("write %s error! %s\n", fl_info[cnt].upgrade_file, strerror(errno));
                goto pack_bin_error;
            }

            file_size += ret;
        }
    }

    ret = 0;
pack_bin_error:
    if(image_fd > 0)
    {
        close(image_fd);
    }
    if(pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int unpack_upgrade_file(char *upgrade_bin)
{
    int    ret        = 0;
    int    image_fd   = -1;
    int    pack_fd    = -1;
    char   *pbuf      = NULL;
    char   name[256]  = {0};
    unsigned int  total_size = 0;
    unsigned int  file_size  = 0;
    unsigned int  tmp_size   = 0;
    unsigned int  crc32_val  = 0;
    struct gk_upgrade_header upgrade_header;
	char   pack_md5[16]  = {0};
	MD5_CTX md5;
    
    memset(&upgrade_header, 0, sizeof(upgrade_header));
    memset(&md5, 0, sizeof(md5));
    
    if (NULL == upgrade_bin)
    {
        ERROR_PRT("upgrade_bin is NULL!\n");
        return -1;
    }

    pack_fd = open(upgrade_bin, O_RDONLY);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_bin, strerror(errno));
        return -1;
    }
    total_size = lseek(pack_fd, 0, SEEK_END);
    if (total_size < 0)
	{
        ERROR_PRT("lseek %s ERROR! %s\n", upgrade_bin, strerror(errno));
        ret = -1;
        goto unpack_bin_error;
    }
    ret = lseek(pack_fd, 0, SEEK_SET);
    if (ret < 0)
	{
        ERROR_PRT("lseek %s ERROR! %s\n", upgrade_bin, strerror(errno));
        ret = -1;
        goto unpack_bin_error;
    }
    file_size = 0;
    while (1)
    {
        /* 1. ��ȡ�ļ�ͷ��Ϣ */
        ret = read(pack_fd, &upgrade_header, sizeof(struct gk_upgrade_header));
        if (ret <= 0)
        {
            if (total_size != tmp_size)
            {
                ERROR_PRT("total_size:%u != tmp_size:%u ret:%d %s\n", total_size, tmp_size, ret, strerror(errno));
                goto unpack_bin_error;
            }
            DEBUG_PRT("read %s  total_size:%u end! ret:%d %s\n", upgrade_bin, tmp_size, ret, strerror(errno));
            break;
        }
        tmp_size += ret;

        /* У����� check magic_num  */
        if (NULL == strstr(upgrade_header.magic_num, MAGIC_NUM))
        {
            ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
            ret = -1;
            goto unpack_bin_error;
        }

        /* У�������ļ���С */
        if(upgrade_header.file_size >= MAX_MTD_SIZE)
        {
            ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                            upgrade_header.file_size, upgrade_header.file_size/1024);
            ret = -1;
            goto unpack_bin_error;
        }

        DEBUG_PRT("header_addr:0x%x  image_addr:0x%x \n", (tmp_size - ret), tmp_size);
        printf_upgrade_header_info(&upgrade_header);

        /*2.  ��ȡ��Ҫ�����ľ���buff ��, ����CRC У�������*/
        if (pbuf)
        {
            free(pbuf);
            pbuf = NULL;
        }
        pbuf = malloc(upgrade_header.file_size + 2);
        if (NULL == pbuf)
        {
            ERROR_PRT("malloc pheader->file_size:%u ERROR!\n", upgrade_header.file_size);
            ret = -1;
            goto unpack_bin_error;
        }

        ret = read(pack_fd, pbuf, upgrade_header.file_size);
        if (ret <= 0)
        {
            ERROR_PRT("total_size:%u != tmp_size:%u ret:%d %s\n", total_size, tmp_size, ret, strerror(errno));
            goto unpack_bin_error;
        }
        tmp_size += ret;
		
		memcpy(pack_md5, upgrade_header.pack_md5, sizeof(pack_md5));
		memcpy(upgrade_header.pack_md5, upgrade_header.machine_type, sizeof(upgrade_header.pack_md5));
		MD5Init(&md5);
		MD5Update(&md5, (unsigned char *)&upgrade_header, sizeof(GK_UPGRADE_HEADER_S));
		MD5Update(&md5, pbuf, upgrade_header.file_size);
		MD5Final(&md5, upgrade_header.pack_md5);
		if (0 != memcmp(pack_md5, upgrade_header.pack_md5, sizeof(upgrade_header.pack_md5)))
        {
			char md5str[2][64] = {0};
			printf_md5(upgrade_header.pack_md5, md5str[0]);
			printf_md5(pack_md5, md5str[1]);
            ERROR_PRT("The mtd:%s  name:%s md5 verify error! md5_val:%s, header_md5:%s\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name,  md5str[0],  md5str[1]);
            ret = -1;
            goto unpack_bin_error;
        }

        crc32_val = mtd_crc32(0, pbuf, upgrade_header.file_size);
        if (crc32_val != upgrade_header.file_crc32)
        {
            ERROR_PRT("The mtd:%s  name:%s crc32 verify error! header_crc:%u  crc32_val:%u\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name, 
                        upgrade_header.file_crc32, crc32_val);
            ret = -1;
            goto unpack_bin_error;
        }

        /* 3. ����������ȡ����Ӧ�������ļ�,  ����������ļ� */
        if(image_fd > 0)
        {
            close(image_fd);
            image_fd = -1;
        }
        memset(name, 0, sizeof(name));
        strcat(name, upgrade_header.mtd_name);
        strcat(name, ".bin");
        image_fd = open(name, O_RDWR|O_CREAT);
        if (image_fd < 0)
        {
            ERROR_PRT("open %s ERROR! %s\n", name, strerror(errno));
            return -1;
        }

        ret = write(image_fd, pbuf, upgrade_header.file_size);
        if (ret <= 0)
        {
            ERROR_PRT("write %s error! %s\n", name, strerror(errno));
            goto unpack_bin_error;
        }
    }

    ret = 0;
unpack_bin_error:
    if (pbuf)
    {
        free(pbuf);
    }
    if(image_fd > 0)
    {
        close(image_fd);
    }
    if(pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int verify_upgrade_file(char *upgrade_bin)
{
    int    ret        = 0;
    int    pack_fd    = -1;
    char  *pbuf       = NULL;
    unsigned int  crc32_val  = 0;
    unsigned int  file_size  = 0;
    struct gk_upgrade_header upgrade_header;
	char   pack_md5[16]  = {0};
	MD5_CTX md5;
    
    memset(&upgrade_header, 0, sizeof(upgrade_header));
    memset(&md5, 0, sizeof(md5));

    if (NULL == upgrade_bin)
    {
        ERROR_PRT("upgrade_bin is NULL!\n");
        return -1;
    }

    pack_fd = open(upgrade_bin, O_RDONLY);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_bin, strerror(errno));
        return -1;
    }

    while (1)
    {
        /* 1. ��ȡ�ļ�ͷ��Ϣ */
        ret = read(pack_fd, &upgrade_header, sizeof(struct gk_upgrade_header));
        if (ret <= 0)
        {
            DEBUG_PRT("read %s  total_size:%u end! ret:%d %s\n", upgrade_bin, file_size, ret, strerror(errno));
            break;
        }
        file_size += ret;

        /* У����� check magic_num  */
        if (NULL == strstr(upgrade_header.magic_num, MAGIC_NUM))
        {
            ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
            ret = -1;
            goto verify_upgrade_err;
        }

        /* У�������ļ���С */
        if(upgrade_header.file_size >= MAX_MTD_SIZE)
        {
            ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                            upgrade_header.file_size, upgrade_header.file_size/1024);
            ret = -1;
            goto verify_upgrade_err;
        }

        DEBUG_PRT("header_addr:0x%x  image_addr:0x%x \n", (file_size - ret), file_size);
        printf_upgrade_header_info(&upgrade_header);

        /*2.  ��ȡ��Ҫ�����ľ���buff ��, ����CRC У�������*/
        if (pbuf)
        {
            free(pbuf);
            pbuf = NULL;
        }
        pbuf = malloc(upgrade_header.file_size + 2);
        if (NULL == pbuf)
        {
            ERROR_PRT("malloc pheader->file_size:%u ERROR!\n", upgrade_header.file_size);
            ret = -1;
            goto verify_upgrade_err;
        }

        ret = read(pack_fd, pbuf, upgrade_header.file_size);
        if (ret <= 0 || ret != upgrade_header.file_size)
        {
            ERROR_PRT("file_size:%u ret:%d %s\n", file_size, ret, strerror(errno));
            goto verify_upgrade_err;
        }
        file_size += ret;
		
		memcpy(pack_md5, upgrade_header.pack_md5, sizeof(pack_md5));
		memcpy(upgrade_header.pack_md5, upgrade_header.machine_type, sizeof(upgrade_header.pack_md5));
		MD5Init(&md5);
		MD5Update(&md5, (unsigned char *)&upgrade_header, sizeof(GK_UPGRADE_HEADER_S));
		MD5Update(&md5, pbuf, upgrade_header.file_size);
		MD5Final(&md5, upgrade_header.pack_md5);
		if (0 != memcmp(pack_md5, upgrade_header.pack_md5, sizeof(upgrade_header.pack_md5)))
        {
            char md5str[2][64] = {0};
			printf_md5(upgrade_header.pack_md5, md5str[0]);
			printf_md5(pack_md5, md5str[1]);
            ERROR_PRT("The mtd:%s  name:%s md5 verify error! md5_val:%s, header_md5:%s\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name,  md5str[0],  md5str[1]);
            ret = -1;
            goto verify_upgrade_err;
        }

        /* 3. ����ʱ��ͷ�ṹ���е�CRC������У�� */
        crc32_val = mtd_crc32(0, pbuf, upgrade_header.file_size);
        if (crc32_val != upgrade_header.file_crc32)
        {
            ERROR_PRT("The mtd:%s  name:%s crc32 verify error! header_crc:%u  crc32_val:%u\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name, 
                        upgrade_header.file_crc32, crc32_val);
            ret = -1;
            goto verify_upgrade_err;
        }
    }

    ret = 0;
verify_upgrade_err:
    if (pbuf)
    {
        free(pbuf);
    }
    if(pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int verify_upgrade_buff(char *upgrade_buf, unsigned int buf_size)
{
    int    ret        = 0;
    unsigned int  pos        = 0;
    unsigned int  crc32_val  = 0;
    struct gk_upgrade_header upgrade_header;
	char   pack_md5[16]  = {0};
	MD5_CTX md5;
    
    memset(&upgrade_header, 0, sizeof(upgrade_header));
    memset(&md5, 0, sizeof(md5));

    if (NULL == upgrade_buf)
    {
        ERROR_PRT("upgrade_buf is NULL error!\n");
        return -1;
    }

    if (buf_size > MAX_NORFLASH_SIZE)
    {
        ERROR_PRT("file_size:0x%x is too large! MAX_NORFLASH_SIZE:0x%x\n", buf_size, MAX_NORFLASH_SIZE);
        return -1;
    }

    pos = 0;
    while (1)
    {
        if (pos >= buf_size)
        {
            DEBUG_PRT("total_size:%u pos:%d\n", buf_size, pos);
            return 0;
        }

        /* 1. ��ȡ�ļ�ͷ��Ϣ */
        memcpy(&upgrade_header, upgrade_buf + pos, sizeof(struct gk_upgrade_header));
        pos += sizeof(struct gk_upgrade_header);

        /* 2. У����� check magic_num  */
        if (NULL == strstr(upgrade_header.magic_num, MAGIC_NUM))
        {
            ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
            return -1;
        }

        /* 3. У�������ļ���С */
        if(upgrade_header.file_size >= MAX_MTD_SIZE)
        {
            ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                            upgrade_header.file_size, upgrade_header.file_size/1024);
            return -1;
        }

        DEBUG_PRT("mtd_name:%s  file_addr:0x%x \n", upgrade_header.mtd_name, pos);
        printf_upgrade_header_info(&upgrade_header);
		
		memcpy(pack_md5, upgrade_header.pack_md5, sizeof(pack_md5));
		memcpy(upgrade_header.pack_md5, upgrade_header.machine_type, sizeof(upgrade_header.pack_md5));
		MD5Init(&md5);
		MD5Update(&md5, (unsigned char *)&upgrade_header, sizeof(GK_UPGRADE_HEADER_S));
		MD5Update(&md5, upgrade_buf + pos, upgrade_header.file_size);
		MD5Final(&md5, upgrade_header.pack_md5);
		if (0 != memcmp(pack_md5, upgrade_header.pack_md5, sizeof(upgrade_header.pack_md5)))
        {
            char md5str[2][64] = {0};
			printf_md5(upgrade_header.pack_md5, md5str[0]);
			printf_md5(pack_md5, md5str[1]);
            ERROR_PRT("The mtd:%s  name:%s md5 verify error! md5_val:%s, header_md5:%s\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name,  md5str[0],  md5str[1]);
            return -1;
        }

        /* 4. ����ʱ��ͷ�ṹ���е�CRC������У�� */
        crc32_val = mtd_crc32(0, upgrade_buf + pos, upgrade_header.file_size);
        if (crc32_val != upgrade_header.file_crc32)
        {
            ERROR_PRT("The mtd:%s  name:%s crc32 verify error! header_crc:%u  crc32_val:%u\n", 
                        upgrade_header.mtd_part, upgrade_header.mtd_name, 
                        upgrade_header.file_crc32, crc32_val);
            return -1;
        }
        pos += upgrade_header.file_size;
    }

    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int get_upgradeinfo_by_file(char *upgrade_bin, struct upgrade_file_info *pinfo, int * mod_num)
{
    int    ret        = 0;
    int    mtd_cnt    = 0;
    int    pack_fd    = -1;
    unsigned int  file_size  = 0;

    if (NULL == upgrade_bin || NULL == pinfo || NULL == mod_num)
    {
        ERROR_PRT("upgrade_bin:%p pinfo:%p mod_num:%p is NULL!\n", upgrade_bin, pinfo, mod_num);
        return -1;
    }

    pack_fd = open(upgrade_bin, O_RDONLY);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_bin, strerror(errno));
        return -1;
    }

    file_size = 0;
    mtd_cnt   = 0;
    while (1)
    {
        if (mtd_cnt >= MAX_MTD_NUM)
        {
            ERROR_PRT("mtd_cnt >= MAX_MTD_NUM error!\n");
            ret = -1;
            goto get_upgradeinfo_err;
        }

        /* 1. ��ȡ�ļ�ͷ��Ϣ */
        ret = read(pack_fd, &(pinfo[mtd_cnt].upgrade_header), sizeof(struct gk_upgrade_header));
        if (ret <= 0)
        {
            DEBUG_PRT("read %s  total_size:%u end! ret:%d %s\n", upgrade_bin, file_size, ret, strerror(errno));
            break;
        }
        file_size += ret;

        /* 2. У����� check magic_num  */
        if (NULL == strstr(pinfo[mtd_cnt].upgrade_header.magic_num, MAGIC_NUM))
        {
            ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
            ret = -1;
            goto get_upgradeinfo_err;
        }

        /* 3. У�������ļ���С */
        if(pinfo[mtd_cnt].upgrade_header.file_size >= MAX_MTD_SIZE)
        {
            ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                            pinfo[mtd_cnt].upgrade_header.file_size, 
                                            pinfo[mtd_cnt].upgrade_header.file_size/1024);
            ret = -1;
            goto get_upgradeinfo_err;
        }

        /* 4. ��ȡ��Ӧ����*/
        pinfo[mtd_cnt].file_addr = lseek(pack_fd, 0, SEEK_CUR);
        pinfo[mtd_cnt].file_size = pinfo[mtd_cnt].upgrade_header.file_size;

        DEBUG_PRT("header_addr:0x%lx  image_addr:0x%x \n", 
                                        pinfo[mtd_cnt].file_addr - sizeof(struct gk_upgrade_header), 
                                        pinfo[mtd_cnt].file_addr);
        printf_upgrade_header_info(&(pinfo[mtd_cnt].upgrade_header));

        /* 5.  ������һ������ģ��λ��*/
        ret = lseek(pack_fd, pinfo[mtd_cnt].upgrade_header.file_size, SEEK_CUR);
        if (ret < 0)
    	{
            ERROR_PRT("lseek %s ERROR! %s\n", upgrade_bin, strerror(errno));
            ret = -1;
            goto get_upgradeinfo_err;
        }
        file_size += pinfo[mtd_cnt].upgrade_header.file_size;

        mtd_cnt++;
    }
    
    *mod_num = mtd_cnt;

    ret = 0;
get_upgradeinfo_err:
    if(pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.21  wangguixing
 *****************************************************************************/
int get_upgradeinfo_by_buff(char *upgrade_buf, 
                                    unsigned int buf_size, 
                                    struct upgrade_file_info *pinfo, 
                                    int * mod_num)
{
    int    ret         = 0;
    int    mtd_cnt     = 0;
    unsigned int  pos  = 0;

    if (NULL == upgrade_buf || NULL == pinfo || NULL == mod_num)
    {
        ERROR_PRT("upgrade_buff:%p pinfo:%p mod_num:%p is NULL!\n", upgrade_buf, pinfo, mod_num);
        return -1;
    }

    if (buf_size > MAX_NORFLASH_SIZE)
    {
        ERROR_PRT("file_size:0x%x is too large! MAX_NORFLASH_SIZE:0x%x\n", buf_size, MAX_NORFLASH_SIZE);
        return -1;
    }

    pos = 0;
    mtd_cnt   = 0;
    while (1)
    {
        if (mtd_cnt >= MAX_MTD_NUM)
        {
            ERROR_PRT("mtd_cnt >= MAX_MTD_NUM error!\n");
            return -1;
        }

        if (pos >= buf_size)
        {
            DEBUG_PRT("read_pos:%u >= buf_size:%u return!\n", pos, buf_size);
            break;
        }

        /* 1. ��ȡ�ļ�ͷ��Ϣ */
        memcpy(&(pinfo[mtd_cnt].upgrade_header), upgrade_buf + pos, sizeof(struct gk_upgrade_header));
        pos += sizeof(struct gk_upgrade_header);

        /* 2. У����� check magic_num  */
        if (NULL == strstr(pinfo[mtd_cnt].upgrade_header.magic_num, MAGIC_NUM))
        {
            ERROR_PRT("check magic_num error! ok_magic is [%s]\n", MAGIC_NUM);
            return -1;
        }

        /* 3. У�������ļ���С */
        if(pinfo[mtd_cnt].upgrade_header.file_size >= MAX_MTD_SIZE)
        {
            ERROR_PRT("upgrade address is too longht, error! 0x%x (%dKB)\n", 
                                            pinfo[mtd_cnt].upgrade_header.file_size, 
                                            pinfo[mtd_cnt].upgrade_header.file_size/1024);
            return -1;
        }

        /* 4. ��ȡ��Ӧ����*/
        pinfo[mtd_cnt].file_addr = pos;
        pinfo[mtd_cnt].file_size = pinfo[mtd_cnt].upgrade_header.file_size;

        DEBUG_PRT("header_addr:0x%lx  image_addr:0x%x \n", 
                                        pinfo[mtd_cnt].file_addr - sizeof(struct gk_upgrade_header), 
                                        pinfo[mtd_cnt].file_addr);
        printf_upgrade_header_info(&(pinfo[mtd_cnt].upgrade_header));

        /* 5.  ������һ������ģ��λ��*/
        pos += pinfo[mtd_cnt].upgrade_header.file_size;

        /* 6. �������еľ��������1 */
        mtd_cnt++;
    }
    
    *mod_num = mtd_cnt;

    return 0;
}


/******************************************************************************
 * ��������: �ú�������Ҫ����������, �������з�����Ϣ��
 *                           �����ļ���Ϣ, �ж��Ƕ���ͨ����������������
 *                           �Ƕ� ALL �������в���.
 * �������: pmtd_info       : �Ǵ�СΪMAX_MTD_NUM �Ľṹ������!
 *                           mtd_num        : pmtd_info����Ч����
 *                           pupgrade_info : ���������������Ϣ
 *                           
 * �������: ��
 * ����ֵ      : <0-�������
 *                           1-��ʾֻ��Ҫ������ALL_MTD ����ͨ����, ��: /dev/mtd1
 *                           2-��ʾ������ͷ�����С�ı���, ֻ�ܶ�ALL_MTD ��������
 *****************************************************************************/
int judge_upgrade_by_info(struct proc_mtd_info *pmtd_info, 
                                  int mtd_num, 
                                  struct upgrade_file_info *pupgrade_info)
{
    unsigned int tmp        = 0;
    unsigned int total_size = 0;

    if (NULL == pmtd_info || NULL == pupgrade_info)
    {
        ERROR_PRT("pmtd_info:%p  pupgrade_info:%p error!\n", pmtd_info, pupgrade_info);
        return -1;
    }

    if (mtd_num > MAX_MTD_NUM)
    {
        ERROR_PRT("Input mtd_num:%d too large MAX_MTD_NUM:%d\n", mtd_num, MAX_MTD_NUM);
        return -1;
    }
    
    if (pmtd_info[mtd_num].mtd_addr == pupgrade_info->upgrade_header.mtd_addr 
       && pmtd_info[mtd_num].mtd_size == pupgrade_info->upgrade_header.mtd_size
       && pmtd_info[mtd_num].mtd_size >= pupgrade_info->file_size)
    {
        /* ����ָ���ķ�ALL ����,  ��:  uboot, kernel ��Ӧ��/dev/mtd0 , /dev/mtd3 �� */
        DEBUG_PRT("Upgrade  part:%s  name:%s  flash_addr:0x%x  mtd_size:0x%x  file_size:0x%x\n", 
                  pmtd_info[mtd_num].mtd_part, pmtd_info[mtd_num].mtd_name,
                  pmtd_info[mtd_num].mtd_addr, pmtd_info[mtd_num].mtd_size,
                  pupgrade_info->file_size);
        return UPGRADE_NORMAL_MTD;
    }
    else
    {
        /* �˴�˵�������Ѿ��ı�, ��Ҫ�� ALL �������в��� */
        tmp        = pupgrade_info->upgrade_header.mtd_addr + pupgrade_info->upgrade_header.mtd_size;
        total_size = pupgrade_info->upgrade_header.flash_size;
        if (tmp <= total_size)
        {
            DEBUG_PRT("Upgrade to ALL_MTD [%s] name:%s  flash_addr:0x%x  mtd_size:0x%x  file_size:0x%x\n", 
                      pupgrade_info->upgrade_header.mtd_part, pupgrade_info->upgrade_header.mtd_name,
                      pupgrade_info->upgrade_header.mtd_addr, pupgrade_info->upgrade_header.mtd_size,
                      pupgrade_info->file_size);
            return UPGRADE_ALL_MTD;
        }
        else
        {
            ERROR_PRT("upgrade_file_info error addr:0x%x + file_size:0x%x > flash_size:0x%x \n\n",
                                                       pupgrade_info->upgrade_header.mtd_addr,
                                                       pupgrade_info->upgrade_header.mtd_size,
                                                       pupgrade_info->upgrade_header.flash_size);
            return -1;
        }
    }

    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.23  wangguixing
 *****************************************************************************/
 int upgrade_flash_by_buf(char *upgrade_buf, unsigned int buf_size)
 {
    int   ret         = 0;
    int   cnt         = 0;
    int   verify_cnt  = 0;
    int   upgrade_mtd = 0;
    int   mod_num     = 0;
    int   mtd_num     = 0;
    struct proc_mtd_info     mtd_info[MAX_MTD_NUM];
    struct proc_mtd_info     all_mtd;
    struct upgrade_file_info upgrade_info[MAX_MTD_NUM];

    memset(&mtd_info, 0, sizeof(mtd_info));
    memset(&all_mtd, 0, sizeof(all_mtd));
    memset(&upgrade_info, 0, sizeof(upgrade_info));

    if (NULL == upgrade_buf)
    {
        ERROR_PRT("upgrade_buf is NULL error!\n");
        return -1;
    }

    if (buf_size > MAX_NORFLASH_SIZE)
    {
        ERROR_PRT("file_size:0x%x is too large! MAX_NORFLASH_SIZE:0x%x\n", buf_size, MAX_NORFLASH_SIZE);
        return -1;
    }

    /* ��ʼ�� �������� */
    set_upgrade_start(buf_size);

    /* 1. У�������ļ� */
    ret = verify_upgrade_buff(upgrade_buf, buf_size);
    if (ret < 0)
    {
        ERROR_PRT("Call verify_upgrade_buff ret:%d \n", ret);
        return -1;
    }

    /* 2. �������ļ��л�ȡ, ��������ģ���λ�ú�ͷ��Ϣ, �������� */
    ret = get_upgradeinfo_by_buff(upgrade_buf, buf_size, upgrade_info, &mod_num);
    if (ret < 0)
    {
        ERROR_PRT("Call get_upgradeinfo_by_buff error! ret:%d \n", ret);
        return -1;
    }
    DEBUG_PRT(" get_upgradeinfo_by_buff -> mod_num:%d \n\n", mod_num);

    /* 3.  ��ȡ����linux ϵͳ��ALL mtd������Ϣ, ���ڷ�����С�ȸı������� */
    ret = get_all_mtd_info_by_proc(&all_mtd);
    if (ret < 0)
    {
        ERROR_PRT("Call get_all_mtd_info_by_proc error! ret:%d \n", ret);
        return -1;
    }

    /* 4. ��ȡ����linux ϵͳ�� /proc/mtd ��Ϣ */
    ret = get_mtdinfo_by_proc_mtd(mtd_info, &mtd_num);
    if (ret < 0)
    {
        ERROR_PRT("Call get_mtdinfo_by_proc_mtd error! ret:%d \n", ret);
        return -1;
    }

    /* 5. ��������ģ��ͷ��Ϣ��/proc/mtd �������� */
    for (cnt = 0; cnt < mod_num; cnt++)
    {
        printf("\n=============== [%d] mtd_name:%s ====================\n\n", cnt, upgrade_info[cnt].upgrade_header.mtd_name);
        
        /* 6. �ж�����������������ʽ */
        upgrade_mtd = judge_upgrade_by_info(mtd_info, cnt, &upgrade_info[cnt]);
        if (upgrade_mtd < 0)
        {
            ERROR_PRT("Call judge_upgrade_by_info error! upgrade_mtd:%d \n", upgrade_mtd);
            return -1;
        }
        DEBUG_PRT(" judge_upgrade_by_info cnt:%d  upgrade_mtd:%d \n", cnt, upgrade_mtd);

        /* 7. ������������*/
        if ((upgrade_info[cnt].file_addr + upgrade_info[cnt].file_size) > buf_size)
        {
            ERROR_PRT("file_addr:%u + file_size:%u > buf_size:%u\n", 
                                                        upgrade_info[cnt].file_addr, 
                                                        upgrade_info[cnt].file_size,
                                                        buf_size);
            return -1;
        }

        /* 8. ǿ��ж���������� MTD ���� */
        //umount2("/mnt/jffs2_mtd", MNT_FORCE);

        /* 9. �Է���������������, ��У��, �ٲ�д, ���дflash */
        if (UPGRADE_NORMAL_MTD == upgrade_mtd)
        {
            verify_cnt = 3;
            while (verify_cnt--)
            {
                /* ���������MTD�������ݲ�ͬ, �������� */
                ret = erase_flash_all(upgrade_info[cnt].upgrade_header.mtd_part,
                                      0,
                                      upgrade_info[cnt].upgrade_header.mtd_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����˳� */
                    ERROR_PRT("erase_flash_all %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                    //return -1;
                    continue;
                }

                ret = flash_copy(upgrade_info[cnt].upgrade_header.mtd_part,
                                 0,
                                 upgrade_buf + upgrade_info[cnt].file_addr,
                                 upgrade_info[cnt].file_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����Ƴ� */
                    ERROR_PRT("flash_copy %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                    //return -1;
                    continue;
                }

                break;  /* ע�� ��һ�β�д�ɹ���, ���˳�while ѭ��.*/
            }
        }
        else if(UPGRADE_ALL_MTD == upgrade_mtd)
        {
            DEBUG_PRT("upgrade mtd:%s  name:%s  addr:0x%x size:0x%x\n", all_mtd.mtd_part, all_mtd.mtd_name, 
                                                                        upgrade_info[cnt].upgrade_header.mtd_addr, 
                                                                        upgrade_info[cnt].upgrade_header.mtd_size);

            verify_cnt = 3;
            while (verify_cnt--)
            {
                /* ��ʽ�� ALL ��������Ķ�Ӧ��ַ�ͳ��� */
                ret = erase_flash_all(all_mtd.mtd_part,
                                      upgrade_info[cnt].upgrade_header.mtd_addr,
                                      upgrade_info[cnt].upgrade_header.mtd_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����Ƴ� */
                    ERROR_PRT("erase_flash_all %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                    //return -1;
                    continue;
                }

                ret = flash_copy(all_mtd.mtd_part, 
                                 upgrade_info[cnt].upgrade_header.mtd_addr,
                                 upgrade_buf + upgrade_info[cnt].file_addr,
                                 upgrade_info[cnt].file_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����Ƴ� */
                    ERROR_PRT("flash_copy %s error!!\n", all_mtd.mtd_part);
                    //return -1;
                    continue;
                }

                break;  /* ע�� ��һ�β�д�ɹ���, ���˳�while ѭ��.*/
            }
        }
        else
        {
            ERROR_PRT("Call judge_upgrade_by_info error! ret:%d \n", upgrade_mtd);
            return -1;
        }
    }

    /* ����������� */
    set_upgrade_end();
    
    return 0;
}


/******************************************************************************
 * ��������: 
 * �������: 
 *                           
 * �������: 
 * ����ֵ      : 
 *                               v1.0.0    2015.08.23  wangguixing
 *****************************************************************************/
int upgrade_flash_by_file(char *upgrade_file)
{
    int   ret         = 0;
    int   cnt         = 0;
    int   upgrade_mtd = 0;
    int   mod_num     = 0;
    int   mtd_num     = 0;
    int   pack_fd     = -1;
    char *pbuf        = NULL;
    struct proc_mtd_info     mtd_info[MAX_MTD_NUM];
    struct proc_mtd_info     all_mtd;
    struct upgrade_file_info upgrade_info[MAX_MTD_NUM];
    
    memset(&mtd_info, 0, sizeof(mtd_info));
    memset(&all_mtd, 0, sizeof(all_mtd));
    memset(&upgrade_info, 0, sizeof(upgrade_info));

    if (NULL == upgrade_file)
    {
        ERROR_PRT("upgrade_file is NULL error!\n");
        return -1;
    }

    /* 1. У�������ļ� */
    ret = verify_upgrade_file(upgrade_file);
    if (ret < 0)
    {
        ERROR_PRT("Call verify_upgrade_file:%s error! ret:%d \n", upgrade_file, ret);
        return -1;
    }

    /* 2. �������ļ��л�ȡ, ��������ģ���λ�ú�ͷ��Ϣ, �������� */
    ret = get_upgradeinfo_by_file(upgrade_file, upgrade_info, &mod_num);
    if (ret < 0)
    {
        ERROR_PRT("Call get_upgradeinfo_by_file error! ret:%d \n", ret);
        return -1;
    }
    DEBUG_PRT(" get_upgradeinfo_by_file -> mod_num:%d \n\n", mod_num);

    /* 3.  ��ȡ����linux ϵͳ��ALL mtd������Ϣ, ���ڷ�����С�ȸı������� */
    ret = get_all_mtd_info_by_proc(&all_mtd);
    if (ret < 0)
    {
        ERROR_PRT("Call get_all_mtd_info_by_proc error! ret:%d \n", ret);
        return -1;
    }

    /* 4. ��ȡ����linux ϵͳ�� /proc/mtd ��Ϣ */
    ret = get_mtdinfo_by_proc_mtd(mtd_info, &mtd_num);
    if (ret < 0)
    {
        ERROR_PRT("Call get_mtdinfo_by_proc_mtd error! ret:%d \n", ret);
        return -1;
    }

    /* 5. ��������ģ��ͷ���ĺ�/proc/mtd �������� */
    pack_fd = open(upgrade_file, O_RDONLY);
    if (pack_fd < 0)
    {
        ERROR_PRT("open %s ERROR! %s\n", upgrade_file, strerror(errno));
        return -1;
    }
    for (cnt = 0; cnt < mod_num; cnt++)
    {
        printf("\n=============== [%d] mtd_name:%s ====================\n\n", cnt, upgrade_info[cnt].upgrade_header.mtd_name);
        
        /* 6. �ж�����������������ʽ */
        upgrade_mtd = judge_upgrade_by_info(mtd_info, cnt, &upgrade_info[cnt]);
        if (upgrade_mtd < 0)
        {
            ERROR_PRT("Call judge_upgrade_by_info error! upgrade_mtd:%d \n", upgrade_mtd);
            ret = -1;
            goto upgrade_flash_err;
        }
        DEBUG_PRT(" judge_upgrade_by_info cnt:%d  upgrade_mtd:%d \n", cnt, upgrade_mtd);

        /* 7. ������������*/
        if (NULL != pbuf)
        {
            free(pbuf);
            pbuf = NULL;
        }
        pbuf = malloc(upgrade_info[cnt].file_size + 2);
        if (NULL == pbuf)
        {
            ERROR_PRT("malloc pheader->file_size:%u ERROR! %s\n", upgrade_info[cnt].file_size, strerror(errno));
            ret = -1;
            goto upgrade_flash_err;
        }
        ret = lseek(pack_fd, upgrade_info[cnt].file_addr, SEEK_SET);
        if (ret < 0)
    	{
            ERROR_PRT("lseek %s ERROR! %s\n", upgrade_file, strerror(errno));
            ret = -1;
            goto upgrade_flash_err;
        }
        ret = read(pack_fd, pbuf, upgrade_info[cnt].file_size);
        if (ret < 0 || ret != upgrade_info[cnt].file_size)
        {
            ERROR_PRT("file_size:%u ret:%d %s\n", upgrade_info[cnt].file_size, ret, strerror(errno));
            goto upgrade_flash_err;
        }

        /* 8. ǿ��ж���������� MTD ���� */
        //umount2("/mnt/jffs2_mtd", MNT_FORCE);

        /* 9. �Է���������������, ��У��, �ٲ�д, ���дflash */
        if (UPGRADE_NORMAL_MTD == upgrade_mtd)
        {
            ret = verify_flash(upgrade_info[cnt].upgrade_header.mtd_part, 
                               0,
                               pbuf,
                               upgrade_info[cnt].file_size);
            if (ret > 0)
            {
                /* ���������MTD�������ݲ�ͬ, �������� */
                DEBUG_PRT("verify_flash %s different! upgrade!!\n", 
                                                upgrade_info[cnt].upgrade_header.mtd_part);
                ret = erase_flash_all(upgrade_info[cnt].upgrade_header.mtd_part,
                                      0,
                                      upgrade_info[cnt].upgrade_header.mtd_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����Ƴ� */
                    ERROR_PRT("erase_flash_all %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                    goto upgrade_flash_err;
                }

                ret = flash_copy(upgrade_info[cnt].upgrade_header.mtd_part,
                                 0,
                                 pbuf,
                                 upgrade_info[cnt].file_size);
                if (ret < 0)
                {
                    /* У��ʧ��, �����Ƴ� */
                    ERROR_PRT("flash_copy %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                    goto upgrade_flash_err;
                }
            }
            else if (0 == ret)
            {
                /* ���������MTD��������һ��,  ���������� */
                DEBUG_PRT("verify_flash %s the same image_file! Don't upgrage! ret:%d\n", 
                                                upgrade_info[cnt].upgrade_header.mtd_part, ret);
                continue;
            }
            else
            {
                /* У��ʧ��, �����˳� */
                ERROR_PRT("verify_flash %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                goto upgrade_flash_err;
            }
        }
        else if(UPGRADE_ALL_MTD == upgrade_mtd)
        {
            DEBUG_PRT("upgrade mtd:%s  name:%s  addr:0x%x size:0x%x\n", all_mtd.mtd_part, all_mtd.mtd_name, 
                                                                        upgrade_info[cnt].upgrade_header.mtd_addr, 
                                                                        upgrade_info[cnt].upgrade_header.mtd_size);
            /* ��ʽ�� ALL ��������Ķ�Ӧ��ַ�ͳ��� */
            ret = erase_flash_all(all_mtd.mtd_part,
                                  upgrade_info[cnt].upgrade_header.mtd_addr,
                                  upgrade_info[cnt].upgrade_header.mtd_size);
            if (ret < 0)
            {
                /* У��ʧ��, �����Ƴ� */
                ERROR_PRT("erase_flash_all %s error!!\n", upgrade_info[cnt].upgrade_header.mtd_part);
                goto upgrade_flash_err;
            }

            ret = flash_copy(all_mtd.mtd_part, 
                             upgrade_info[cnt].upgrade_header.mtd_addr,
                             pbuf,
                             upgrade_info[cnt].file_size);
            if (ret < 0)
            {
                /* У��ʧ��, �����Ƴ� */
                ERROR_PRT("flash_copy %s error!!\n", all_mtd.mtd_part);
                goto upgrade_flash_err;
            }
            continue;  /////////////////////
        }
        else
        {
            ERROR_PRT("Call judge_upgrade_by_info error! ret:%d \n", upgrade_mtd);
            ret = -1;
            goto upgrade_flash_err;
        }
    }

    ret = 0;
upgrade_flash_err:
    if (NULL != pbuf)
    {
        free(pbuf);
    }
    if (pack_fd > 0)
    {
        close(pack_fd);
    }
    return ret;
}


#if 0  //Just for testing!
int main(void)
{
    int ret = 0;

    struct gk_upgrade_header upgrade_header;

    DEBUG_PRT("The sizeof(struct gk_upgrade_header):%d\n", sizeof(struct gk_upgrade_header));

    #if 0
    ret = verify_upgrade_file("ttttt");
    if (ret < 0)
    {
        ERROR_PRT("Call verify_upgrade_file error! ret:%d \n", ret);
        return -1;
    }
    #else
    ret = unpack_upgrade_file("ttttt");
    if (ret < 0)
    {
        ERROR_PRT("Call unpack_upgrade_file error! ret:%d \n", ret);
    }
    #endif

    return 0;
}

int main(void)
{
    int ret = 0;

    struct gk_upgrade_header upgrade_header;

    DEBUG_PRT("The sizeof(struct gk_upgrade_header):%d\n", sizeof(struct gk_upgrade_header));

    ret = pack_upgrade_file("./flashmap_gk7101_inception.ini", "upgrade__file.bin");
    if (ret < 0)
    {
        ERROR_PRT("Call pack_upgrade_file error! ret:%d \n", ret);
    }
}
#endif

