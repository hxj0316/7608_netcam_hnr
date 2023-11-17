/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include <signal.h>
#include <unistd.h>
#include "sample_comm.h"
#include "hi_sns_ctrl.h"
#include "hi_common_hnr.h"
#include "hi_mpi_hnr.h"
#include "hi_mpi_ae.h"

#define VO_WIDTH 1920
#define VO_HEIGTH 1080

#define VENC_WIDTH 3840
#define VENC_HEIGTH 2160

#define VB_HNR_LINE_CNT  12
#define VB_HNR_WDR_CNT   13
#define VB_MCF_YUV_CNT   30

volatile static sig_atomic_t g_sig_flag = 0;
static hi_vb_pool g_vb_pool = HI_VB_INVALID_POOL_ID;

typedef struct {
    hi_bool is_wdr_mode;
    hi_hnr_ref_mode ref_mode;
    hi_bool normal_blend;
    hi_vi_video_mode video_mode;
} sample_hnr_param;

static sample_vo_cfg g_vo_cfg = {
    .vo_dev            = SAMPLE_VO_DEV_UHD,
    .vo_intf_type      = HI_VO_INTF_HDMI,
    .intf_sync         = HI_VO_OUT_1080P30,
    .bg_color          = COLOR_RGB_BLACK,
    .pix_format        = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420,
    .disp_rect         = {0, 0, VO_WIDTH, VO_HEIGTH},
    .image_size        = {VO_WIDTH, VO_HEIGTH},
    .vo_part_mode      = HI_VO_PARTITION_MODE_SINGLE,
    .dis_buf_len       = 3, /* 3: def buf len for single */
    .dst_dynamic_range = HI_DYNAMIC_RANGE_SDR8,
    .vo_mode           = VO_MODE_1MUX,
    .compress_mode     = HI_COMPRESS_MODE_NONE,
};

static sample_comm_venc_chn_param g_venc_chn_param = {
    .frame_rate           = 30, /* 30 is a number */
    .stats_time           = 1,  /* 1 is a number */
    .gop                  = 30, /* 30 is a number */
    .venc_size            = {VENC_WIDTH, VENC_HEIGTH},
    .size                 = PIC_3840X2160,
    .profile              = 0,
    .is_rcn_ref_share_buf = HI_FALSE,
    .gop_attr             = {
        .gop_mode = HI_VENC_GOP_MODE_NORMAL_P,
        .normal_p = {2},
    },
    .type                 = HI_PT_H265,
    .rc_mode              = SAMPLE_RC_CBR,
};

static td_s32 g_hnr_cfg_id[HI_HNR_MAX_CFG_NUM] = {-1};
static td_void sample_get_char(hi_char *s)
{
    if (g_sig_flag == 1) {
        return;
    }

    printf("---------------press any key to %s!---------------\n", s);
    (td_void)getchar();
}

static td_void sample_hnr_usage(char *prg_name)
{
    printf("usage : %s <index> \n", prg_name);
    printf("index:\n");
    printf("    (0) hnr line mode : base mode, has reference frame\n");
    printf("    (1) hnr wdr mode  : wdr and long frame mode\n");
    printf("    (2) hnr line and none ref mode : no reference frame\n");
    printf("    (3) hnr line mode + normal blend : base mode and normal blend on\n");
    printf("    (4) hnr snap advanced mode: advanced mode of no reference frame \n");
    printf("    (5) hnr line switch to wdr: line mode and hnr normal blend switch to wdr mode\n");
    printf("    (6) hnr + mcf     : combine with hnr and mcf\n");
}

static td_void sample_hnr_get_default_vb_config(const ot_size *size, hi_vb_cfg *vb_cfg, td_u32 vb_cnt)
{
    hi_vb_calc_cfg calc_cfg;
    hi_pic_buf_attr buf_attr;

    (td_void)memset_s(vb_cfg, sizeof(hi_vb_cfg), 0, sizeof(hi_vb_cfg));
    vb_cfg->max_pool_cnt = 128; /* 128 blks */

    buf_attr.width         = size->width;
    buf_attr.height        = size->height;
    buf_attr.align         = HI_DEFAULT_ALIGN;
    buf_attr.bit_width     = HI_DATA_BIT_WIDTH_8;
    buf_attr.pixel_format  = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422;
    buf_attr.compress_mode = HI_COMPRESS_MODE_NONE;
    hi_common_get_pic_buf_cfg(&buf_attr, &calc_cfg);

    vb_cfg->common_pool[0].blk_size = calc_cfg.vb_size;
    vb_cfg->common_pool[0].blk_cnt  = vb_cnt;
}

static td_s32 sample_hnr_sys_init(ot_size *in_size, hi_vi_video_mode video_mode, td_u32 vb_cnt)
{
    td_s32 ret;
    hi_vb_cfg vb_cfg;
    td_u32 supplement_config;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;

    sample_hnr_get_default_vb_config(in_size, &vb_cfg, vb_cnt);

    supplement_config = HI_VB_SUPPLEMENT_BNR_MOT_MASK;
    ret = sample_comm_sys_init_with_vb_supplement(&vb_cfg, supplement_config);
    if (ret != HI_SUCCESS) {
        return HI_FAILURE;
    }

    ret = sample_comm_vi_set_vi_vpss_mode(mode_type, video_mode);
    if (ret != HI_SUCCESS) {
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

static hi_char *hnr_common_get_param_file(hi_hnr_ref_mode ref_mode)
{
    hi_char *param_file = HI_NULL;

    if (ref_mode == HI_HNR_REF_MODE_NORM) {
        param_file = "/sharefs/cfg/normal_hnr.bin";
    } else {
        param_file = "/sharefs/cfg/snap_hnr.bin";
    }

    printf("load config file %s\n", param_file);
    return param_file;
}

static td_s32 hnr_common_load_mem(hi_hnr_mem_info *mem, hi_hnr_ref_mode ref_mode)
{
    td_s32 ret;
    hi_char *param_file = HI_NULL;
    FILE *fp = HI_NULL;

    param_file = hnr_common_get_param_file(ref_mode);
    /* Get model file size */
    fp = fopen(param_file, "rb");
    if (fp == HI_NULL) {
        sample_print("open file %s err!\n", param_file);
        return HI_FAILURE;
    }

    ret = fseek(fp, 0L, SEEK_END);
    if (ret != HI_SUCCESS) {
        sample_print("fseek end failed!\n");
        goto fail_0;
    }

    mem->size = ftell(fp);
    if (mem->size <= 0) {
        sample_print("ftell failed!\n");
        goto fail_0;
    }

    ret = fseek(fp, 0L, SEEK_SET);
    if (ret != HI_SUCCESS) {
        sample_print("fseek set failed!\n");
        goto fail_0;
    }

    /* malloc model file mem */
    ret = hi_mpi_sys_mmz_alloc(&(mem->phys_addr), &(mem->virt_addr), "hnr_cfg", HI_NULL, mem->size);
    if (ret != HI_SUCCESS) {
        sample_print("malloc mmz failed!\n");
        goto fail_0;
    }

    ret = fread(mem->virt_addr, mem->size, 1, fp);
    if (ret != 1) {
        sample_print("read model file failed!\n");
        goto fail_1;
    }

    ret = fclose(fp);
    if (ret != 0) {
        sample_print("close file error\n");
    }
    return HI_SUCCESS;

fail_1:
    hi_mpi_sys_mmz_free(mem->phys_addr, mem->virt_addr);
    mem->phys_addr = 0;
    mem->virt_addr = HI_NULL;
fail_0:
    if (fp != HI_NULL) {
        fclose(fp);
    }
    return HI_FAILURE;
}

static td_void hnr_common_unload_mem(hi_hnr_mem_info *param_mem)
{
    if ((param_mem->phys_addr != 0) && (param_mem->virt_addr != HI_NULL)) {
        (td_void)hi_mpi_sys_mmz_free(param_mem->phys_addr, param_mem->virt_addr);
    }
}

static td_s32 sample_hnr_set_attr(hi_vi_pipe vi_pipe, sample_hnr_param *hnr_param)
{
    td_s32 ret;
    hi_hnr_attr hnr_attr;

    ret = hi_mpi_hnr_get_attr(vi_pipe, &hnr_attr);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_get_attr error\n");
        return ret;
    }

    hnr_attr.enable = HI_TRUE;
    hnr_attr.bnr_bypass = (hnr_param->normal_blend == HI_TRUE) ? HI_FALSE : HI_TRUE;
    hnr_attr.normal_blend = hnr_param->normal_blend;
    hnr_attr.op_type = HI_OP_MODE_MANUAL;
    hnr_attr.manual_attr.param.sfs = 31; /* sfs: 31 */
    hnr_attr.manual_attr.param.tfs = 31; /* tfs: 31 */
    ret = hi_mpi_hnr_set_attr(vi_pipe, &hnr_attr);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_set_attr error\n");
        return ret;
    }

    return HI_SUCCESS;
}

static td_s32 sample_hnr_load_cfg(ot_size in_size, hi_hnr_cfg *hnr_cfg, hi_hnr_ref_mode ref_mode)
{
    td_s32 ret;

    ret = hnr_common_load_mem((hi_hnr_mem_info *)&(hnr_cfg->mem_info), ref_mode);
    if (ret != HI_SUCCESS) {
        sample_print("hnr_common_load_mem error\n");
        return ret;
    }

    if (ref_mode != HI_HNR_REF_MODE_NORM) {
        hnr_cfg->image_size.width  = in_size.width;
        hnr_cfg->image_size.height = in_size.height;
        hnr_cfg->is_wdr_mode = HI_FALSE;
        ret = hi_mpi_hnr_load_cfg(hnr_cfg, &g_hnr_cfg_id[0]);
        if (ret != HI_SUCCESS) {
            sample_print("hi_mpi_hnr_load_cfg error(%#x)\n", ret);
            goto unload_mem;
        }
        return ret;
    }

    hnr_cfg->image_size.width  = in_size.width;
    hnr_cfg->image_size.height = in_size.height;
    hnr_cfg->is_wdr_mode = HI_FALSE;
    ret = hi_mpi_hnr_load_cfg(hnr_cfg, &g_hnr_cfg_id[0]);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_load_cfg error(%#x)\n", ret);
        goto unload_mem;
    }

    hnr_cfg->is_wdr_mode = HI_TRUE;
    ret = hi_mpi_hnr_load_cfg(hnr_cfg, &g_hnr_cfg_id[1]);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_load_cfg error(%#x)\n", ret);
        goto unload_mem;
    }
    return ret;

unload_mem:
    hnr_common_unload_mem((hi_hnr_mem_info *)&(hnr_cfg->mem_info));
    return ret;
}

static td_void sample_hnr_unload_cfg(hi_hnr_cfg *hnr_cfg, hi_hnr_ref_mode ref_mode)
{
    if (hi_mpi_hnr_unload_cfg(g_hnr_cfg_id[0]) != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_unload_cfg error\n");
    }
    if (ref_mode == HI_HNR_REF_MODE_NORM) {
        if (hi_mpi_hnr_unload_cfg(g_hnr_cfg_id[1]) != HI_SUCCESS) {
            sample_print("hi_mpi_hnr_unload_cfg error\n");
        }
    }
    hnr_common_unload_mem((hi_hnr_mem_info *)&(hnr_cfg->mem_info));
}

static td_s32 sample_hnr_start(hi_vi_pipe vi_pipe, ot_size in_size, hi_hnr_cfg *hnr_cfg,
    sample_hnr_param *hnr_param)
{
    td_s32 ret = hi_mpi_hnr_init();
    ot_hnr_alg_cfg attr;

    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_init error(%#x)\n", ret);
        return ret;
    }
    sample_print("hhhhhxxxxjjjj307\n");

    ret = sample_hnr_load_cfg(in_size, hnr_cfg, hnr_param->ref_mode);
    if (ret != HI_SUCCESS) {
        goto task_exit;
    }

    attr.ref_mode = hnr_param->ref_mode; /* ref mode NORM: need reference frame; NONE: no reference frame */
    sample_print("mode ==%d\n",attr.ref_mode);
    ret = hi_mpi_hnr_set_alg_cfg(vi_pipe, &attr);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_hnr_set_alg_cfg error(%#x)\n", ret);
        goto unload_cfg;
    }

    //sample_get_char("enable hnr");

   ret = hi_mpi_hnr_enable(vi_pipe);
   if (ret != HI_SUCCESS) {
       sample_print("hi_mpi_hnr_enable error(%#x)\n", ret);
       goto unload_cfg;
   } else {
       sample_print("hi_mpi_hnr_enable suceess, (%#x)\n", ret);
   }

     
    ret = sample_hnr_set_attr(vi_pipe, hnr_param);
    if (ret != HI_SUCCESS) {
        sample_print("sample_hnr_set_attr error(%#x)\n", ret);
        goto hnr_disable;
    }

         sample_print("hxjhxj340\n");
   // while(1) 
   // {
   //    sleep(1);
   // }
    

    return ret;

hnr_disable:
    hi_mpi_hnr_disable(vi_pipe);
unload_cfg:
    sample_hnr_unload_cfg(hnr_cfg, hnr_param->ref_mode);
task_exit:
    hi_mpi_hnr_exit();

    return ret;
}

static td_s32 sample_hnr_stop(hi_vi_pipe vi_pipe, hi_hnr_cfg *hnr_cfg, sample_hnr_param *hnr_param)
{
    td_s32 return_value = HI_SUCCESS;
    td_s32 ret;
    ret = hi_mpi_hnr_disable(vi_pipe);
    if (ret != HI_SUCCESS) {
        return_value = HI_FAILURE;
        sample_print("hi_mpi_hnr_disable false error(%#x)\n", ret);
    }

    ret = hi_mpi_hnr_unload_cfg(g_hnr_cfg_id[0]);
    if (ret != HI_SUCCESS) {
        return_value = HI_FAILURE;
        sample_print("hi_mpi_hnr_unload_cfg error(%#x)\n", ret);
    }

    if (hnr_param->ref_mode == HI_HNR_REF_MODE_NORM) {
        ret = hi_mpi_hnr_unload_cfg(g_hnr_cfg_id[1]);
        if (ret != HI_SUCCESS) {
            return_value = HI_FAILURE;
            sample_print("hi_mpi_hnr_unload_cfg error(%#x)\n", ret);
        }
    }
    hnr_common_unload_mem((hi_hnr_mem_info *)&(hnr_cfg->mem_info));

    hi_mpi_hnr_exit();
    return return_value;
}

static td_s32 sample_hnr_start_vpss(hi_vpss_grp grp, ot_size *in_size)
{
    td_s32 ret;
    hi_low_delay_info low_delay_info;
    hi_vpss_grp_attr grp_attr;
    hi_vpss_chn_attr chn_attr;
    hi_bool chn_enable[HI_VPSS_MAX_PHYS_CHN_NUM] = {HI_TRUE, HI_FALSE, HI_FALSE, HI_FALSE};

    sample_comm_vpss_get_default_grp_attr(&grp_attr);
    grp_attr.max_width  = in_size->width;
    grp_attr.max_height = in_size->height;
    sample_comm_vpss_get_default_chn_attr(&chn_attr);
    chn_attr.width  = in_size->width;
    chn_attr.height = in_size->height;

    ret = sample_common_vpss_start(grp, chn_enable, &grp_attr, &chn_attr, HI_VPSS_MAX_PHYS_CHN_NUM);
    if (ret != HI_SUCCESS) {
        return ret;
    }

     sample_print("hxjhxj409409\n");


    low_delay_info.enable     = HI_TRUE;
    low_delay_info.line_cnt   = 200; /* 200: lowdelay line */
    low_delay_info.one_buf_en = HI_FALSE;
    ret = hi_mpi_vpss_set_low_delay_attr(grp, 0, &low_delay_info);
    if (ret != HI_SUCCESS) {
        sample_common_vpss_stop(grp, chn_enable, HI_VPSS_MAX_PHYS_CHN_NUM);
        return ret;
    }

    return HI_SUCCESS;
}

static td_s32 sample_hnr_start_vo(sample_vo_mode vo_mode)
{
    g_vo_cfg.vo_mode = vo_mode;

    return sample_comm_vo_start_vo(&g_vo_cfg);
}

static td_void sample_hnr_stop_vo(td_void)
{
    sample_comm_vo_stop_vo(&g_vo_cfg);
}

static td_s32 sample_hnr_start_venc(hi_venc_chn venc_chn[], td_u32 venc_chn_len, td_u32 chn_num)
{
    td_s32 i;
    td_s32 ret;

    for (i = 0; i < (td_s32)chn_num && i < venc_chn_len; i++) {
        ret = sample_comm_venc_start(venc_chn[i], &g_venc_chn_param);
        if (ret != HI_SUCCESS) {
            goto exit;
        }
    }

    
  //  ret = sample_comm_venc_start_get_stream(venc_chn, chn_num);
  //  if (ret != HI_SUCCESS) {
  //      goto exit;
  //  }
   
     sample_print("hxjhxj457457\n");

    return HI_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        sample_comm_venc_stop(venc_chn[i]);
    }
    return HI_FAILURE;
}

static td_s32 sample_hnr_start_venc_and_vo(hi_vpss_grp vpss_grp[], td_u32 grp_num, ot_size *in_size)
{
    td_u32 i;
    td_s32 ret;
    sample_vo_mode vo_mode = VO_MODE_1MUX;
    const hi_vpss_chn vpss_chn = 0;
    const hi_vo_layer vo_layer = 0;
    hi_vo_chn vo_chn[4] = {0, 1, 2, 3};     /* 4: max chn num, 0/1/2/3 chn id */
    hi_venc_chn venc_chn[4] = {0, 1, 2, 3}; /* 4: max chn num, 0/1/2/3 chn id */

    if (grp_num > 1) {
        vo_mode = VO_MODE_4MUX;
    }

    ret = sample_hnr_start_vo(vo_mode);
    if (ret != HI_SUCCESS) {
        goto start_vo_failed;
    }

    g_venc_chn_param.venc_size.width  = in_size->width;
    g_venc_chn_param.venc_size.height = in_size->height;

//    if (SENSOR0_TYPE == SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT) {
//        g_venc_chn_param.size = PIC_2560X1440;
//    } else if ((SENSOR0_TYPE == OV_OS08A20_MIPI_8M_30FPS_12BIT) ||
//               (SENSOR0_TYPE == SONY_IMX485_MIPI_8M_30FPS_12BIT)) {
//        g_venc_chn_param.size = PIC_3840X2160;
//    } else {
//        g_venc_chn_param.size = PIC_1080P;
//    }
//
    ret = sample_hnr_start_venc(venc_chn, sizeof(venc_chn) / sizeof(hi_venc_chn), grp_num);
    if (ret != HI_SUCCESS) {
        goto start_venc_failed;
    }

    for (i = 0; i < grp_num; i++) {
        sample_comm_vpss_bind_vo(vpss_grp[i], vpss_chn, vo_layer, vo_chn[i]);
        sample_comm_vpss_bind_venc(vpss_grp[i], vpss_chn, venc_chn[i]);
    }
    return HI_SUCCESS;

start_venc_failed:
    sample_hnr_stop_vo();
start_vo_failed:
    return HI_FAILURE;
}

static td_void sample_hnr_stop_vpss(hi_vpss_grp grp)
{
    hi_bool chn_enable[HI_VPSS_MAX_PHYS_CHN_NUM] = {HI_TRUE, HI_FALSE, HI_FALSE, HI_FALSE};

    sample_common_vpss_stop(grp, chn_enable, HI_VPSS_MAX_PHYS_CHN_NUM);
}

static td_void sample_hnr_stop_venc(hi_venc_chn venc_chn[], td_u32 venc_chn_len, td_u32 chn_num)
{
    td_u32 i;

    //sample_comm_venc_stop_get_stream(chn_num);

    for (i = 0; i < chn_num && i < venc_chn_len; i++) {
        sample_comm_venc_stop(venc_chn[i]);
    }
}

static td_void sample_hnr_stop_venc_and_vo(hi_vpss_grp vpss_grp[], td_u32 grp_num)
{
    td_u32 i;
    const hi_vpss_chn vpss_chn = 0;
    const hi_vo_layer vo_layer = 0;
    hi_vo_chn vo_chn[4] = {0, 1, 2, 3};     /* 4: max chn num, 0/1/2/3 chn id */
    hi_venc_chn venc_chn[4] = {0, 1, 2, 3}; /* 4: max chn num, 0/1/2/3 chn id */

    for (i = 0; i < grp_num; i++) {
        sample_comm_vpss_un_bind_vo(vpss_grp[i], vpss_chn, vo_layer, vo_chn[i]);
        sample_comm_vpss_un_bind_venc(vpss_grp[i], vpss_chn, venc_chn[i]);
    }

    sample_hnr_stop_venc(venc_chn, sizeof(venc_chn) / sizeof(hi_venc_chn), grp_num);
    sample_hnr_stop_vo();
}

static td_void sample_hnr_sensor_cfg(hi_vi_pipe vi_pipe, sample_sns_type sns_type)
{
    hi_isp_sns_blc_clamp sns_blc_clamp;

    sns_blc_clamp.blc_clamp_en = HI_FALSE;

    switch (sns_type) {
        case SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT:
            g_sns_imx347_slave_obj.pfn_mirror_flip(vi_pipe, ISP_SNS_MIRROR);
            break;
        case SONY_IMX485_MIPI_8M_30FPS_12BIT:
            g_sns_imx485_obj.pfn_set_blc_clamp(vi_pipe, sns_blc_clamp);
            break;

        default:
            break;
    }

    return;
}

static td_s32 sample_hnr_start_vi(hi_vi_pipe vi_pipe, const sample_vi_cfg *vi_cfg, ot_size *in_size,
    hi_vi_video_mode video_mode, td_u32 vb_cnt)
{
    td_s32 ret;
    td_s32 blk_size;
    hi_vb_pool vb_pool;
    hi_vb_pool_cfg vb_pool_cfg = {0};

   // if (sample_hnr_sys_init(in_size, video_mode, vb_cnt) != HI_SUCCESS) {
   //     return HI_FAILURE;
   // }

    blk_size = hi_hnr_get_pic_buf_size(in_size->width, in_size->height);
    vb_pool_cfg.blk_size = blk_size;
    vb_pool_cfg.blk_cnt = vb_cnt;
    vb_pool_cfg.remap_mode = HI_VB_REMAP_MODE_NONE;
    vb_pool = hi_mpi_vb_create_pool(&vb_pool_cfg);
    if (vb_pool == HI_VB_INVALID_POOL_ID) {
        sample_print("create user pool failed!\n");
        return HI_FAILURE;
    }

  //  ret = sample_comm_vi_start_vi(vi_cfg);
  //  if (ret != HI_SUCCESS) {
  //      sample_print("start vi failed.\n");
  //     return ret;
  //  }

    ret = hi_mpi_vi_pipe_set_vb_src(vi_pipe, HI_VB_SRC_USER);
    if (ret != HI_SUCCESS) {
        sample_print("vi pipe set vb src failed.\n");
        return ret;
    }

    ret = hi_mpi_vi_pipe_attach_vb_pool(vi_pipe, vb_pool);
    if (ret != HI_SUCCESS) {
        sample_print("vi pipe attach vb pool failed.\n");
        return ret;
    }

    g_vb_pool = vb_pool;

    return HI_SUCCESS;
}

static td_s32 sample_hnr_switch_start_vi(hi_vi_pipe vi_pipe, const sample_vi_cfg *vi_cfg, ot_size *in_size)
{
    td_s32 ret;
    td_s32 blk_size;
    hi_vb_pool vb_pool;
    hi_vb_pool_cfg vb_pool_cfg = {0};

    blk_size = hi_hnr_get_pic_buf_size(in_size->width, in_size->height);
    vb_pool_cfg.blk_size = blk_size;
    vb_pool_cfg.blk_cnt = VB_HNR_WDR_CNT;
    vb_pool_cfg.remap_mode = HI_VB_REMAP_MODE_NONE;
    vb_pool = hi_mpi_vb_create_pool(&vb_pool_cfg);
    if (vb_pool == HI_VB_INVALID_POOL_ID) {
        sample_print("create user pool failed!\n");
        return HI_FAILURE;
    }

    ret = sample_comm_vi_mode_switch_start_vi(vi_cfg, HI_FALSE, (const ot_size *)&in_size);
    if (ret != HI_SUCCESS) {
        sample_print("start vi failed.\n");
        return ret;
    }

    ret = hi_mpi_vi_pipe_set_vb_src(vi_pipe, HI_VB_SRC_USER);
    if (ret != HI_SUCCESS) {
        sample_print("vi pipe set vb src failed.\n");
        return ret;
    }

    ret = hi_mpi_vi_pipe_attach_vb_pool(vi_pipe, vb_pool);
    if (ret != HI_SUCCESS) {
        sample_print("vi pipe attach vb pool failed.\n");
        return ret;
    }

    g_vb_pool = vb_pool;

    return HI_SUCCESS;
}

static td_void sample_hnr_stop_vi(hi_vi_pipe vi_pipe, const sample_vi_cfg *vi_cfg)
{
    sample_comm_vi_stop_vi(vi_cfg);

    if (g_vb_pool != HI_VB_INVALID_POOL_ID) {
        hi_mpi_vb_destroy_pool(g_vb_pool);
        g_vb_pool = HI_VB_INVALID_POOL_ID;
    }
}

static td_s32 sample_hnr_set_long_frame_mode(hi_vi_pipe vi_pipe, hi_bool is_wdr_mode)
{
    td_s32 ret;
    hi_isp_exposure_attr exp_attr;
    hi_isp_wdr_fs_attr fswdr_attr;

    if (is_wdr_mode == HI_FALSE) {
        return HI_SUCCESS;
    }

    sample_get_char("set long frame mode");

    ret = hi_mpi_isp_get_exposure_attr(vi_pipe, &exp_attr);
    if (ret != HI_SUCCESS) {
        printf("hi_mpi_isp_get_exposure_attr error[0x%x]\n", ret);
        return HI_FAILURE;
    }

    exp_attr.auto_attr.fswdr_mode = OT_ISP_FSWDR_LONG_FRAME_MODE;
    ret = hi_mpi_isp_set_exposure_attr(vi_pipe, &exp_attr);
    if (ret != HI_SUCCESS) {
        printf("hi_mpi_isp_set_exposure_attr error[0x%x]\n", ret);
        return HI_FAILURE;
    }

    ret = hi_mpi_isp_get_fswdr_attr(vi_pipe, &fswdr_attr);
    if (ret != HI_SUCCESS) {
        printf("hi_mpi_isp_get_fswdr_attr error[0x%x]\n", ret);
        return HI_FAILURE;
    }

    fswdr_attr.wdr_merge_mode = OT_ISP_MERGE_WDR_MODE;
    ret = hi_mpi_isp_set_fswdr_attr(vi_pipe, &fswdr_attr);
    if (ret != HI_SUCCESS) {
        printf("hi_mpi_isp_set_fswdr_attr error[0x%x]\n", ret);
        return HI_FAILURE;
    }

    return ret;
}

static td_void sample_hnr_get_default_cfg(sample_sns_type sns_type, hi_vi_pipe vi_pipe, hi_vi_video_mode video_mode,
    ot_size *size, sample_vi_cfg *vi_cfg)
{
    sample_comm_vi_get_size_by_sns_type(sns_type, size);
    sample_comm_vi_get_default_vi_cfg(sns_type, vi_cfg);

    if (video_mode == HI_VI_VIDEO_MODE_NORM) {
        vi_cfg->pipe_info[vi_pipe].pipe_attr.compress_mode = HI_COMPRESS_MODE_NONE;
    }
}

static td_s32 sample_hnr_set_blc(hi_vi_pipe vi_pipe, sample_sns_type sns_type)
{
    td_s32 i, j, ret;
    hi_isp_black_level_attr black_level_attr;

    ret = hi_mpi_isp_get_black_level_attr(vi_pipe, &black_level_attr);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_isp_get_black_level_attr failed.\n");
        return HI_FAILURE;
    }

    black_level_attr.user_black_level_en = HI_TRUE;

    for (i = 0; i < HI_ISP_WDR_MAX_FRAME_NUM; i++) {
        for (j = 0; j < HI_ISP_BAYER_CHN_NUM; j++) {
            black_level_attr.user_black_level[i][j] = 1200; /* user_black_level of hnr default as 1200 */
        }
    }

    if ((sns_type == SONY_IMX347_SLAVE_MIPI_4M_30FPS_12BIT) || (sns_type == SONY_IMX485_MIPI_8M_30FPS_12BIT)) {
        black_level_attr.black_level_mode = HI_ISP_BLACK_LEVEL_MODE_DYNAMIC;
    }

    ret = hi_mpi_isp_set_black_level_attr(vi_pipe, &black_level_attr);
    if (ret != HI_SUCCESS) {
        sample_print("hi_mpi_isp_set_black_level_attr failed.\n");
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

static td_s32 sample_hnr_check_support(hi_vi_pipe vi_pipe, sample_hnr_param *hnr_param)
{
    if (hnr_param->is_wdr_mode == TD_TRUE) {
        if (hnr_param->video_mode != HI_VI_VIDEO_MODE_NORM) {
            sample_print("not support HI_VI_VIDEO_MODE_ADVANCED in wdr mode, must be HI_VI_VIDEO_MODE_NORM \n");
            return HI_FAILURE;
        }

        if (hnr_param->normal_blend != HI_FALSE) {
            sample_print("normal_blend must be false in wdr mode\n");
            return HI_FAILURE;
        }

        if (vi_pipe >= HI_VI_MAX_PHYS_PIPE_NUM) {
            sample_print("vi_pipe must be phy pipe in wdr mode\n");
            return HI_FAILURE;
        }
    }

    /* bnr_bypass is invalid in HI_VI_VIDEO_MODE_ADVANCED */
    /* bnr_bypass must be false when attr->enable is false and in HI_VI_VIDEO_MODE_NORM */
    /* normal_blend is valid when bnr_bypass is false in HI_VI_VIDEO_MODE_NORM */
    /* vi must be offline when hnr in HI_VI_VIDEO_MODE_NORM */

    return HI_SUCCESS;
}

static td_s32 sample_hnr(sample_hnr_param *hnr_param)
{
    td_s32 ret = HI_FAILURE;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type = hnr_param->is_wdr_mode ? OV_OS08B10_MIPI_8M_30FPS_12BIT_WDR2TO1 : OV_OS08B10_MIPI_8M_30FPS_12BIT;
    const td_u32 vb_cnt = hnr_param->is_wdr_mode ? VB_HNR_WDR_CNT : VB_HNR_LINE_CNT;
    sample_print("vb_cnt = %d\n", vb_cnt);
    const hi_vi_pipe vi_pipe = hnr_param->is_wdr_mode ? 1 : 0; /* 1-long frame of wdr */
    sample_print("vi_pipe = %d\n", vi_pipe);
    const hi_vi_pipe master_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    ot_size in_size;
    hi_hnr_cfg hnr_cfg;

    sample_print("sns_type = %d\n", sns_type);

    sample_hnr_get_default_cfg(sns_type, vi_pipe, hnr_param->video_mode, &in_size, &vi_cfg);

  //  if (sample_hnr_check_support(vi_pipe, hnr_param) != HI_SUCCESS) {
  //      return HI_FAILURE;
  //  }

    if (sample_hnr_start_vi(vi_pipe, &vi_cfg, &in_size, hnr_param->video_mode, vb_cnt) != HI_SUCCESS) {
	sample_print("sample_hnr_start_vi failed\n");
        goto sys_exit;
    }

  //  sample_hnr_sensor_cfg(vi_pipe, sns_type);

  //  sample_comm_vi_bind_vpss(master_pipe, vi_chn, vpss_grp[0], 0);
  //  if (sample_hnr_start_vpss(vpss_grp[0], &in_size) != HI_SUCCESS) {
  //      goto stop_vi;
  //  }

  //  if (sample_hnr_start_venc_and_vo(vpss_grp, sizeof(vpss_grp) / sizeof(vpss_grp[0]), &in_size) != HI_SUCCESS) {
  //      goto stop_vpss;
  //  }

    if (sample_hnr_set_blc(master_pipe, sns_type) != HI_SUCCESS) {
	sample_print("sample_hnr_set_blc failed\n");
        goto stop_venc_and_vo;
    }

    if(HI_FALSE == hnr_param->is_wdr_mode)
    {
        if (sample_hnr_start(vi_pipe, in_size, &hnr_cfg, hnr_param) != HI_SUCCESS) {
	    sample_print("sample_hnr_start failed\n");
            goto stop_venc_and_vo;
        }

      hi_isp_exposure_attr exp_attr;
      ret = hi_mpi_isp_get_exposure_attr(vi_pipe, &exp_attr);
      if (ret != TD_SUCCESS) {
      isp_err_trace("get exp_attr failed!!!\n");
      return ret;
      }
      exp_attr.op_type = OT_OP_MODE_AUTO;
      exp_attr.auto_attr.ae_mode = OT_ISP_AE_MODE_SLOW_SHUTTER;
      ret = hi_mpi_isp_set_exposure_attr(vi_pipe, &exp_attr);
      if (ret != TD_SUCCESS) {
      isp_err_trace("set exp_attr failed!!!\n");
      return ret;
      }
    }

    /*
    if (sample_hnr_set_long_frame_mode(master_pipe, hnr_param->is_wdr_mode) != HI_SUCCESS) {
        goto stop_hnr;
    }*/

    //sample_get_char("exit");

      while(1) {
  //    while(0 == g_sig_flag) {
        sleep(1);
    }    

    ret = HI_SUCCESS;

stop_hnr:
    sample_hnr_stop(vi_pipe, &hnr_cfg, hnr_param);
stop_venc_and_vo:
    sample_hnr_stop_venc_and_vo(vpss_grp, sizeof(vpss_grp) / sizeof(vpss_grp[0]));
stop_vpss:
    sample_hnr_stop_vpss(vpss_grp[0]);
stop_vi:
    sample_comm_vi_un_bind_vpss(master_pipe, vi_chn, vpss_grp[0], 0);
    sample_hnr_stop_vi(vi_pipe, &vi_cfg);
sys_exit:
    sample_comm_sys_exit();
   
    return ret;
}

static td_void sample_hnr_handle_sig(td_s32 signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_sig_flag = 1;
    }
}

static td_void sample_register_sig_handler(td_void (*sig_handle)(td_s32))
{
    struct sigaction sa;

    (td_void)memset_s(&sa, sizeof(struct sigaction), 0, sizeof(struct sigaction));
    sa.sa_handler = sig_handle;
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, HI_NULL);
    sigaction(SIGTERM, &sa, HI_NULL);
}

td_s32 sample_hnr_execute_case(td_u32 case_index)
{
    td_s32 ret = HI_FAILURE;
    sample_hnr_param hnr_param;

    switch (case_index) {
        case 0: /* 0 hnr line mode */
            hnr_param.is_wdr_mode = HI_FALSE;
            hnr_param.ref_mode = HI_HNR_REF_MODE_NORM;
            hnr_param.normal_blend = HI_FALSE;
            hnr_param.video_mode = HI_VI_VIDEO_MODE_ADVANCED;
            ret = sample_hnr(&hnr_param);
            break;
        case 1: /* 1 hnr wdr mode */
            hnr_param.is_wdr_mode = HI_TRUE;
            hnr_param.ref_mode = HI_HNR_REF_MODE_NORM;
            hnr_param.normal_blend = HI_FALSE;
            hnr_param.video_mode = HI_VI_VIDEO_MODE_NORM;
            ret = sample_hnr(&hnr_param);
            break;
        default:
            break;
    }
    return ret;
}

#ifdef __LITEOS__
td_s32 app_main(td_s32 argc, hi_char *argv[])
#else
td_s32 app_1_main(td_s32 argc, hi_char *argv[])
#endif
{
    td_s32 ret;
    td_u32 index;

    if (argc != 2) { /* 2:arg num */
        sample_hnr_usage(argv[0]);
        return TD_FAILURE;
    }

    if (!strncmp(argv[1], "-h", 2)) { /* 2:arg num */
        sample_hnr_usage(argv[0]);
        return TD_FAILURE;
    }

    if ((strlen(argv[1]) != 1) ||
        (argv[1][0] < '0' || argv[1][0] > '6')) { /* only support case 0/1/2/3/4/6 */
        sample_hnr_usage(argv[0]);
        return TD_FAILURE;
    }

#ifndef __LITEOS__
    sample_register_sig_handler(sample_hnr_handle_sig);
#endif
    index = atoi(argv[1]);
    ret = sample_hnr_execute_case(index);

#ifdef __LITEOS__
    return ret;
#else
    exit(ret);
#endif
}

