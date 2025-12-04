/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "sample_comm.h"
#include "securec.h"

static volatile sig_atomic_t g_sig_flag = 0;

#define X_ALIGN 16
#define Y_ALIGN 2
#define out_ratio_1(x) ((x) / 3)
#define out_ratio_2(x) ((x) * 2 / 3)
#define out_ratio_3(x) ((x) / 2)
#define check_digit(x) ((x) >= '0' && (x) <= '9')

#define VB_RAW_CNT_NONE     0
#define VB_LINEAR_RAW_CNT   5
#define VB_WDR_RAW_CNT      8
#define VB_MULTI_RAW_CNT    15
#define VB_YUV_ROUTE_CNT    10
#define VB_DOUBLE_YUV_CNT   15
#define VB_MULTI_YUV_CNT    30


static hi_u16 g_lmf_coef[128] = {
    0, 15, 31, 47, 63, 79, 95, 111, 127, 143, 159, 175,
    191, 207, 223, 239, 255, 271, 286, 302, 318, 334, 350, 365, 381, 397, 412,
    428, 443, 459, 474, 490, 505, 520, 536, 551, 566, 581, 596, 611, 626, 641,
    656, 670, 685, 699, 713, 728, 742, 756, 769, 783, 797, 810, 823, 836, 848,
    861, 873, 885, 896, 908, 919, 929, 940, 950, 959, 969, 984, 998, 1013, 1027,
    1042, 1056, 1071, 1085, 1100, 1114, 1129, 1143, 1158, 1172, 1187, 1201, 1215,
    1230, 1244, 1259, 1273, 1288, 1302, 1317, 1331, 1346, 1360, 1375, 1389, 1404,
    1418, 1433, 1447, 1462, 1476, 1491, 1505, 1519, 1534, 1548, 1563, 1577, 1592,
    1606, 1621, 1635, 1650, 1664, 1679, 1693, 1708, 1722, 1737, 1751, 1766, 1780, 1795, 1809, 1823, 1838
};

static sample_vo_cfg g_vo_cfg = {
    .vo_dev            = SAMPLE_VO_DEV_UHD,
    .vo_intf_type      = HI_VO_INTF_HDMI,
    .intf_sync         = HI_VO_OUT_1080P30,
    .bg_color          = COLOR_RGB_BLACK,
    .pix_format        = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420,
    .disp_rect         = {0, 0, 1920, 1080},
    .image_size        = {1920, 1080},
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
    .venc_size            = {1920, 1080},
    .size                 = PIC_1080P,
    .profile              = 0,
    .is_rcn_ref_share_buf = HI_FALSE,
    .gop_attr             = {
        .gop_mode = HI_VENC_GOP_MODE_NORMAL_P,
        .normal_p = {2},
    },
    .type                 = HI_PT_H265,
    .rc_mode              = SAMPLE_RC_VBR,
};

static sample_vi_fpn_calibration_cfg g_calibration_cfg = {
    .threshold     = 4095, /* 4095 is a number */
    .frame_num     = 16,   /* 16 is a number */
    .fpn_type      = HI_ISP_FPN_TYPE_FRAME,
    .pixel_format  = HI_PIXEL_FORMAT_RGB_BAYER_16BPP,
    .compress_mode = HI_COMPRESS_MODE_NONE,
};

static sample_vi_fpn_correction_cfg g_correction_cfg = {
    .op_mode       = HI_OP_MODE_AUTO,
    .fpn_type      = HI_ISP_FPN_TYPE_FRAME,
    .strength      = 0,
    .pixel_format  = HI_PIXEL_FORMAT_RGB_BAYER_16BPP,
    .compress_mode = HI_COMPRESS_MODE_NONE,
};

static hi_void sample_get_char(hi_void)
{
    if (g_sig_flag == 1) {
        return;
    }

    sample_pause();
}

static hi_void sample_vi_get_default_vb_config(hi_size *size, hi_vb_cfg *vb_cfg, hi_vi_video_mode video_mode,
    hi_u32 yuv_cnt, hi_u32 raw_cnt)
{
    hi_vb_calc_cfg calc_cfg;
    hi_pic_buf_attr buf_attr;

    (hi_void)memset_s(vb_cfg, sizeof(hi_vb_cfg), 0, sizeof(hi_vb_cfg));
    vb_cfg->max_pool_cnt = 128; /* 128 blks */

    /* default YUV pool: SP420 + compress_seg */
    buf_attr.width         = size->width;
    buf_attr.height        = size->height;
    buf_attr.align         = HI_DEFAULT_ALIGN;
    buf_attr.bit_width     = HI_DATA_BIT_WIDTH_8;
    buf_attr.pixel_format  = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    buf_attr.compress_mode = OT_COMPRESS_MODE_SEG;
    hi_common_get_pic_buf_cfg(&buf_attr, &calc_cfg);

    vb_cfg->common_pool[0].blk_size = calc_cfg.vb_size;
    vb_cfg->common_pool[0].blk_cnt  = yuv_cnt;

    /* default raw pool: raw12bpp + compress_line */
    buf_attr.pixel_format  = HI_PIXEL_FORMAT_RGB_BAYER_12BPP;
    buf_attr.compress_mode = (video_mode == HI_VI_VIDEO_MODE_NORM ? HI_COMPRESS_MODE_LINE : HI_COMPRESS_MODE_NONE);
    hi_common_get_pic_buf_cfg(&buf_attr, &calc_cfg);
    vb_cfg->common_pool[1].blk_size = calc_cfg.vb_size;
    vb_cfg->common_pool[1].blk_cnt  = raw_cnt;
}

static hi_s32 sample_vio_sys_init(hi_vi_vpss_mode_type mode_type, hi_vi_video_mode video_mode,
    hi_u32 yuv_cnt, hi_u32 raw_cnt)
{
    hi_s32 ret;
    hi_size size;
    hi_vb_cfg vb_cfg;
    hi_u32 supplement_config;
    sample_sns_type sns_type = SENSOR0_TYPE;

    sample_comm_vi_get_size_by_sns_type(sns_type, &size);
    sample_vi_get_default_vb_config(&size, &vb_cfg, video_mode, yuv_cnt, raw_cnt);

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

static hi_s32 sample_vio_start_vpss(hi_vpss_grp grp, hi_size *in_size)
{
    hi_s32 ret;
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

static hi_void sample_vio_stop_vpss(hi_vpss_grp grp)
{
    hi_bool chn_enable[HI_VPSS_MAX_PHYS_CHN_NUM] = {HI_TRUE, HI_FALSE, HI_FALSE, HI_FALSE};

    sample_common_vpss_stop(grp, chn_enable, HI_VPSS_MAX_PHYS_CHN_NUM);
}

static hi_s32 sample_vio_start_venc(hi_venc_chn venc_chn[], hi_u32 chn_num, const hi_size *in_size)
{
    hi_s32 i, ret;

    g_venc_chn_param.venc_size.width  = in_size->width;
    g_venc_chn_param.venc_size.height = in_size->height;
    g_venc_chn_param.size = sample_comm_sys_get_pic_enum(in_size);

    for (i = 0; i < (hi_s32)chn_num; i++) {
        ret = sample_comm_venc_start(venc_chn[i], &g_venc_chn_param);
        if (ret != HI_SUCCESS) {
            goto exit;
        }
    }

    ret = sample_comm_venc_start_get_stream(venc_chn, chn_num);
    if (ret != HI_SUCCESS) {
        goto exit;
    }

    return HI_SUCCESS;

exit:
    for (i = i - 1; i >= 0; i--) {
        sample_comm_venc_stop(venc_chn[i]);
    }
    return HI_FAILURE;
}

static hi_void sample_vio_stop_venc(hi_venc_chn venc_chn[], hi_u32 chn_num)
{
    hi_u32 i;

    sample_comm_venc_stop_get_stream(chn_num);

    for (i = 0; i < chn_num; i++) {
        sample_comm_venc_stop(venc_chn[i]);
    }
}

static hi_s32 sample_vio_start_vo(sample_vo_mode vo_mode)
{
    g_vo_cfg.vo_mode = vo_mode;

    return sample_comm_vo_start_vo(&g_vo_cfg);
}

static hi_void sample_vio_stop_vo(hi_void)
{
    sample_comm_vo_stop_vo(&g_vo_cfg);
}

static hi_s32 sample_vio_start_venc_and_vo(hi_vpss_grp vpss_grp[], hi_u32 grp_num, const hi_size *in_size)
{
    hi_u32 i;
    hi_s32 ret;
    sample_vo_mode vo_mode = VO_MODE_1MUX;
    const hi_vpss_chn vpss_chn = 0;
    const hi_vo_layer vo_layer = 0;
    hi_vo_chn vo_chn[4] = {0, 1, 2, 3};     /* 4: max chn num, 0/1/2/3 chn id */
    hi_venc_chn venc_chn[4] = {0, 1, 2, 3}; /* 4: max chn num, 0/1/2/3 chn id */

    if (grp_num > 1) {
        vo_mode = VO_MODE_4MUX;
    }

    ret = sample_vio_start_vo(vo_mode);
    if (ret != HI_SUCCESS) {
        goto start_vo_failed;
    }

    ret = sample_vio_start_venc(venc_chn, grp_num, in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_failed;
    }

    for (i = 0; i < grp_num; i++) {
        sample_comm_vpss_bind_vo(vpss_grp[i], vpss_chn, vo_layer, vo_chn[i]);
        sample_comm_vpss_bind_venc(vpss_grp[i], vpss_chn, venc_chn[i]);
    }
    return HI_SUCCESS;

start_venc_failed:
    sample_vio_stop_vo();
start_vo_failed:
    return HI_FAILURE;
}

static hi_void sample_vio_stop_venc_and_vo(hi_vpss_grp vpss_grp[], hi_u32 grp_num)
{
    hi_u32 i;
    const hi_vpss_chn vpss_chn = 0;
    const hi_vo_layer vo_layer = 0;
    hi_vo_chn vo_chn[4] = {0, 1, 2, 3};     /* 4: max chn num, 0/1/2/3 chn id */
    hi_venc_chn venc_chn[4] = {0, 1, 2, 3}; /* 4: max chn num, 0/1/2/3 chn id */

    for (i = 0; i < grp_num; i++) {
        sample_comm_vpss_un_bind_vo(vpss_grp[i], vpss_chn, vo_layer, vo_chn[i]);
        sample_comm_vpss_un_bind_venc(vpss_grp[i], vpss_chn, venc_chn[i]);
    }

    sample_vio_stop_venc(venc_chn, grp_num);
    sample_vio_stop_vo();
}

static hi_void sample_vio_print_vi_mode_list(hi_bool is_wdr_mode)
{
    printf("vi vpss mode list: \n");
    printf("    (0) VI_ONLINE_VPSS_ONLINE,   Normal mode\n");
    printf("    (1) VI_ONLINE_VPSS_OFFLINE,  Normal mode\n");
    printf("    (2) VI_OFFLINE_VPSS_ONLINE,  Normal mode\n");
    printf("    (3) VI_OFFLINE_VPSS_OFFLINE, Normal mode\n");
    if (is_wdr_mode == HI_FALSE) {
        printf("    (4) VI_ONLINE_VPSS_ONLINE,   Advanced mode\n");
        printf("    (5) VI_ONLINE_VPSS_OFFLINE,  Advanced mode\n");
        printf("    (6) VI_OFFLINE_VPSS_ONLINE,  Advanced mode\n");
        printf("    (7) VI_OFFLINE_VPSS_OFFLINE, Advanced mode\n");
    }

    printf("please select mode: ");
}

static hi_void sample_vio_get_vb_blk_num_by_char(hi_char ch, hi_u32 *yuv_cnt, hi_u32 *raw_cnt, hi_bool is_wdr)
{
    *yuv_cnt  = VB_YUV_ROUTE_CNT;
    switch (ch) {
        case '0':
        case '1':
            *raw_cnt = VB_RAW_CNT_NONE;
            break;
        case '2':
        case '3':
            *raw_cnt = (is_wdr ? VB_WDR_RAW_CNT : VB_LINEAR_RAW_CNT);
            break;
        case '4':
        case '5':
            *raw_cnt = VB_LINEAR_RAW_CNT;
            break;
        case '6':
        case '7':
            *raw_cnt = VB_WDR_RAW_CNT;
            break;
        default:
            *raw_cnt = VB_WDR_RAW_CNT;
            break;
    }
}

static hi_void sample_vio_get_vi_vpss_mode_by_char(hi_char ch,
    hi_vi_vpss_mode_type *mode_type, hi_vi_video_mode *video_mode)
{
    switch (ch) {
        case '0':
            *mode_type  = HI_VI_ONLINE_VPSS_ONLINE;
            *video_mode = HI_VI_VIDEO_MODE_NORM;
            break;
        case '1':
            *mode_type  = HI_VI_ONLINE_VPSS_OFFLINE;
            *video_mode = HI_VI_VIDEO_MODE_NORM;
            break;
        case '2':
            *mode_type  = HI_VI_OFFLINE_VPSS_ONLINE;
            *video_mode = HI_VI_VIDEO_MODE_NORM;
            break;
        case '3':
            *mode_type  = HI_VI_OFFLINE_VPSS_OFFLINE;
            *video_mode = HI_VI_VIDEO_MODE_NORM;
            break;
        case '4':
            *mode_type  = HI_VI_ONLINE_VPSS_ONLINE;
            *video_mode = HI_VI_VIDEO_MODE_ADVANCED;
            break;
        case '5':
            *mode_type  = HI_VI_ONLINE_VPSS_OFFLINE;
            *video_mode = HI_VI_VIDEO_MODE_ADVANCED;
            break;
        case '6':
            *mode_type  = HI_VI_OFFLINE_VPSS_ONLINE;
            *video_mode = HI_VI_VIDEO_MODE_ADVANCED;
            break;
        case '7':
            *mode_type  = HI_VI_OFFLINE_VPSS_OFFLINE;
            *video_mode = HI_VI_VIDEO_MODE_ADVANCED;
            break;
        default:
            *mode_type  = HI_VI_ONLINE_VPSS_ONLINE;
            *video_mode = HI_VI_VIDEO_MODE_NORM;
            break;
    }
}
static hi_void sample_vio_get_vi_vpss_mode(hi_vi_vpss_mode_type *mode_type, hi_vi_video_mode *video_mode,
    hi_u32 *yuv_cnt, hi_u32 *raw_cnt, hi_bool is_wdr_mode)
{
    hi_char ch = '0';
    hi_char end_ch;

    if (is_wdr_mode == HI_TRUE) {
        end_ch = '3';
    } else {
        end_ch = '7';
    }

    sample_vio_print_vi_mode_list(is_wdr_mode);

    while (((ch = getchar()) != EOF) && (g_sig_flag == 0)) {
        if ((ch >= '0') && (ch <= end_ch)) {
            getchar();
            break;
        } else {
            printf("\nInvalid param, please enter again!\n\n");
            sample_vio_print_vi_mode_list(is_wdr_mode);
            getchar();
        }
    }

    sample_vio_get_vi_vpss_mode_by_char(ch, mode_type, video_mode);
    sample_vio_get_vb_blk_num_by_char(ch, yuv_cnt, raw_cnt, is_wdr_mode);
}

static hi_void sample_vi_get_two_sensor_vi_cfg(sample_sns_type sns_type, sample_vi_cfg *vi_cfg0, sample_vi_cfg *vi_cfg1)
{
    const hi_vi_dev vi_dev = 2; /* dev2 for sensor1 */
    const hi_vi_pipe vi_pipe = 1; /* dev2 bind pipe1 */

    sample_comm_vi_get_default_vi_cfg(sns_type, vi_cfg0);
    sample_comm_vi_get_default_vi_cfg(sns_type, vi_cfg1);

    vi_cfg0->mipi_info.divide_mode = LANE_DIVIDE_MODE_1;

    vi_cfg1->sns_info.bus_id = 5; /* i2c5 */
    vi_cfg1->sns_info.sns_clk_src = 1;
    vi_cfg1->sns_info.sns_rst_src = 1;

    sample_comm_vi_get_mipi_info_by_dev_id(sns_type, vi_dev, &vi_cfg1->mipi_info);
    vi_cfg1->dev_info.vi_dev = vi_dev;
    vi_cfg1->bind_pipe.pipe_id[0] = vi_pipe;
    vi_cfg1->grp_info.grp_num = 1;
    vi_cfg1->grp_info.fusion_grp[0] = 1;
    vi_cfg1->grp_info.fusion_grp_attr[0].pipe_id[0] = vi_pipe;
}

static hi_s32 sample_vio_all_mode(hi_void)
{
    hi_s32 ret;
    hi_u32 yuv_cnt, raw_cnt;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    sample_vio_get_vi_vpss_mode(&mode_type, &video_mode, &yuv_cnt, &raw_cnt, HI_FALSE);
    ret = sample_vio_sys_init(mode_type, video_mode, yuv_cnt, raw_cnt);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_s32 sample_vio_switch_first_route(sample_sns_type sns_type)
{
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    hi_size in_size;
    hi_s32 ret;

    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }
    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }
    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }
    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);
start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    if (ret == HI_SUCCESS) {
        sample_comm_vi_mode_switch_stop_vi(&vi_cfg);
    } else {
        sample_comm_vi_stop_vi(&vi_cfg);
    }
start_vi_failed:
    return ret;
}

static hi_s32 sample_vio_switch_second_route(sample_sns_type sns_type)
{
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    hi_size in_size;
    sample_vi_cfg vi_cfg;
    hi_s32 ret;

    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);

    ret = sample_comm_vi_mode_switch_start_vi(&vi_cfg, HI_FALSE, &in_size);
    if (ret != HI_SUCCESS) {
        return HI_FAILURE;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);
start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);

    return ret;
}

static hi_s32 sample_vio_switch_resolution_route(sample_sns_type sns_type)
{
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    hi_size in_size;
    sample_vi_cfg vi_cfg;
    hi_s32 ret;

    in_size.width = 1920;  // 1920: target width
    in_size.height = 1080; // 1080: target height
    sample_comm_vi_init_vi_cfg(sns_type, &in_size, &vi_cfg);

    ret = sample_comm_vi_mode_switch_start_vi(&vi_cfg, HI_TRUE, &in_size);
    if (ret != HI_SUCCESS) {
        return HI_FAILURE;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);
start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);

    return ret;
}

static hi_s32 sample_vio_switch_mode(hi_void)
{
    hi_s32 ret;
    hi_u32 yuv_cnt, raw_cnt;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    sample_sns_type sns_type = SENSOR0_TYPE;

    sample_vio_get_vi_vpss_mode(&mode_type, &video_mode, &yuv_cnt, &raw_cnt, HI_TRUE);
    ret = sample_vio_sys_init(mode_type, video_mode, yuv_cnt, raw_cnt);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    ret = sample_vio_switch_first_route(sns_type);
    if (ret != HI_SUCCESS) {
        sample_comm_sys_exit();
        return ret;
    }

    if (sns_type == OV_OS08A20_MIPI_8M_30FPS_12BIT) {
        sns_type = OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1;
    } else {
        sns_type = OV_OS08A20_MIPI_8M_30FPS_12BIT;
    }

    ret = sample_vio_switch_second_route(sns_type);
    sample_comm_sys_exit();
    return ret;
}

static hi_s32 sample_vio_switch_resolution(hi_void)
{
    hi_s32 ret;
    hi_u32 yuv_cnt, raw_cnt;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    sample_sns_type sns_type = SENSOR0_TYPE;

    sample_vio_get_vi_vpss_mode(&mode_type, &video_mode, &yuv_cnt, &raw_cnt, HI_TRUE);
    ret = sample_vio_sys_init(mode_type, video_mode, yuv_cnt, raw_cnt);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    ret = sample_vio_switch_first_route(sns_type);
    if (ret != HI_SUCCESS) {
        sample_comm_sys_exit();
        return ret;
    }

    ret = sample_vio_switch_resolution_route(sns_type);
    sample_comm_sys_exit();
    return ret;
}

static hi_s32 sample_vio_wdr(hi_void)
{
    hi_s32 ret;
    hi_u32 yuv_cnt, raw_cnt;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type = SENSOR0_TYPE;
    hi_size in_size;

    sample_vio_get_vi_vpss_mode(&mode_type, &video_mode, &yuv_cnt, &raw_cnt, HI_TRUE);
    ret = sample_vio_sys_init(mode_type, video_mode, yuv_cnt, raw_cnt);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    if (sns_type == OV_OS08A20_MIPI_8M_30FPS_12BIT) {
        sns_type = OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1;
    } else if (sns_type == SONY_IMX485_MIPI_8M_30FPS_12BIT) {
        sns_type = SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1;
    }
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_s32 sample_vio_wdr_send_frame(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_YUV_ROUTE_CNT, VB_WDR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    if (sns_type == OV_OS08A20_MIPI_8M_30FPS_12BIT) {
        sns_type = OV_OS08A20_MIPI_8M_30FPS_12BIT_WDR2TO1;
    } else if (sns_type == SONY_IMX485_MIPI_8M_30FPS_12BIT) {
        sns_type = SONY_IMX485_MIPI_8M_30FPS_10BIT_WDR3TO1;
    }
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    vi_cfg.pipe_info[0].isp_need_run = HI_FALSE;
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    ret = sample_comm_vi_send_wdr_frame(&vi_cfg.bind_pipe);

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vio_restart_get_venc_stream(hi_venc_chn venc_chn[], hi_u32 chn_num)
{
    hi_u32 i;
    hi_s32 ret;
    hi_venc_start_param start_param;

    for (i = 0; i < chn_num; i++) {
        start_param.recv_pic_num = -1;
        if ((ret = hi_mpi_venc_start_chn(venc_chn[i], &start_param)) != HI_SUCCESS) {
            sample_print("hi_mpi_venc_start_recv_pic failed with%#x! \n", ret);
            return;
        }
    }

    sample_comm_venc_start_get_stream(venc_chn, chn_num);
}

static hi_void sample_vio_do_fpn_calibrate_and_correction(hi_vi_pipe vi_pipe)
{
    hi_venc_chn venc_chn[1] = {0};
    const hi_u32 chn_num = 1;

    sample_comm_venc_stop_get_stream(chn_num);
    sample_comm_vi_fpn_calibrate(vi_pipe, &g_calibration_cfg);

    printf("please enter any key to enable fpn correction!\n");
    sample_get_char();

    sample_vio_restart_get_venc_stream(venc_chn, chn_num);
    sample_comm_vi_enable_fpn_correction(vi_pipe, &g_correction_cfg);
}

static hi_s32 sample_vio_fpn(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_YUV_ROUTE_CNT, VB_WDR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_vio_do_fpn_calibrate_and_correction(vi_pipe);

    sample_get_char();

    sample_comm_vi_disable_fpn_correction(vi_pipe, &g_correction_cfg);

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vio_set_dis_en(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_bool enable)
{
    hi_s32 ret;
    hi_dis_cfg dis_cfg = {0};
    hi_dis_attr dis_attr = {0};

    dis_cfg.motion_level  = HI_DIS_MOTION_LEVEL_NORM;
    dis_cfg.crop_ratio    = 80; /* 80 sample crop  ratio */
    dis_cfg.buf_num       = 10; /* 10 sample buf   num   */
    dis_cfg.frame_rate    = 30; /* 30 sample frame rate  */
    dis_cfg.camera_steady = HI_FALSE;
    dis_cfg.scale         = HI_TRUE;
    dis_cfg.pdt_type      = HI_DIS_PDT_TYPE_IPC;
    dis_cfg.mode          = HI_DIS_MODE_6_DOF_GME;
    ret = hi_mpi_vi_set_chn_dis_cfg(vi_pipe, vi_chn, &dis_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("set dis config failed.ret:0x%x !\n", ret);
    }

    dis_attr.enable               = enable;
    dis_attr.moving_subject_level = 0;
    dis_attr.rolling_shutter_coef = 0;
    dis_attr.timelag              = 1000;     /* 1000: timelag */
    dis_attr.still_crop           = HI_FALSE;
    dis_attr.hor_limit            = 512;      /* 512  sample hor_limit */
    dis_attr.ver_limit            = 512;      /* 512  sample ver_limit */
    dis_attr.gdc_bypass           = HI_FALSE;
    dis_attr.strength             = 1024;     /* 1024 sample strength  */
    ret = hi_mpi_vi_set_chn_dis_attr(vi_pipe, vi_chn, &dis_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set dis attr failed.ret:0x%x !\n", ret);
    }
}

static hi_void sample_vio_set_ldc_en(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_bool enable)
{
    hi_s32 ret;
    hi_ldc_attr ldc_attr;

    ldc_attr.enable                       = enable;
    ldc_attr.ldc_version                  = HI_LDC_V1;
    ldc_attr.ldc_v1_attr.aspect           = 0;
    ldc_attr.ldc_v1_attr.x_ratio          = 100; /* 100: x ratio */
    ldc_attr.ldc_v1_attr.y_ratio          = 100; /* 100: y ratio */
    ldc_attr.ldc_v1_attr.xy_ratio         = 100; /* 100: x y ratio */
    ldc_attr.ldc_v1_attr.center_x_offset  = 0;
    ldc_attr.ldc_v1_attr.center_y_offset  = 0;
    ldc_attr.ldc_v1_attr.distortion_ratio = 500; /* 500: distortion ratio */

    ret = hi_mpi_vi_set_chn_ldc_attr(vi_pipe, vi_chn, &ldc_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set ldc attr failed.ret:0x%x !\n", ret);
    }
}

static hi_void sample_vio_switch_ldc_dis_en(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn)
{
    printf("please enter any key to enable ldc && dis!\n");
    sample_get_char();

    sample_vio_set_dis_en(vi_pipe, vi_chn, HI_TRUE);
    sample_vio_set_ldc_en(vi_pipe, vi_chn, HI_TRUE);

    printf("please enter any key to disable ldc && dis!\n");
    sample_get_char();

    sample_vio_set_ldc_en(vi_pipe, vi_chn, HI_FALSE);
    sample_vio_set_dis_en(vi_pipe, vi_chn, HI_FALSE);
}

static hi_s32 sample_vio_ldc_dis(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_YUV_ROUTE_CNT, VB_LINEAR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_vio_switch_ldc_dis_en(vi_pipe, vi_chn);
    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vio_get_fisheye_correction_attr(hi_fisheye_correction_attr *correction_attr, hi_size *out_size)
{
    correction_attr->enable                      = HI_TRUE;
    correction_attr->dst_size.width              = out_size->width;
    correction_attr->dst_size.height             = out_size->height;
    correction_attr->fisheye_attr.lmf_en         = HI_TRUE;
    correction_attr->fisheye_attr.bg_color_en    = HI_TRUE;
    correction_attr->fisheye_attr.bg_color       = COLOR_RGB_BLUE;
    correction_attr->fisheye_attr.hor_offset     = 0;
    correction_attr->fisheye_attr.ver_offset     = 0;
    correction_attr->fisheye_attr.trapezoid_coef = 0;
    correction_attr->fisheye_attr.fan_strength   = 0;
    correction_attr->fisheye_attr.mount_mode     = HI_FISHEYE_MOUNT_MODE_CEILING;
    correction_attr->fisheye_attr.rgn_num        = 2; /* 2: rgn_num */

    correction_attr->fisheye_attr.fisheye_rgn_attr[0].view_mode       = HI_FISHEYE_VIEW_MODE_360_PANORAMA;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].in_radius       = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_radius      = 1200; /* 1200: out_radius */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].pan             = 180;  /* 180: pan */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].tilt            = 180;  /* 180: tilt */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].hor_zoom        = 4095; /* 4095: hor_zoom */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].ver_zoom        = 4095; /* 4095: ver_zoom */
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.x      = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.y      = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.width  = out_size->width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[0].out_rect.height
        = HI_ALIGN_DOWN(out_ratio_1(out_size->height), Y_ALIGN);

    correction_attr->fisheye_attr.fisheye_rgn_attr[1].view_mode       = HI_FISHEYE_VIEW_MODE_360_PANORAMA;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].in_radius       = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_radius      = 1200; /* 1200: out_radius */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].pan             = 180;  /* 180: pan */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].tilt            = 180;  /* 180: tilt */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].hor_zoom        = 4095; /* 4095: hor_zoom */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].ver_zoom        = 4095; /* 4095: ver_zoom */
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.x      = 0;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.y      =
        HI_ALIGN_DOWN(out_ratio_1(out_size->height), Y_ALIGN);
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.width  = out_size->width;
    correction_attr->fisheye_attr.fisheye_rgn_attr[1].out_rect.height =
        HI_ALIGN_DOWN(out_ratio_1(out_size->height), Y_ALIGN);
}

static hi_void sample_vio_set_fisheye_param(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_size *out_size)
{
    hi_s32 ret;
    hi_fisheye_correction_attr correction_attr;
    hi_fisheye_cfg fisheye_cfg;

    (hi_void)memcpy_s(fisheye_cfg.lmf_coef, sizeof(g_lmf_coef), g_lmf_coef, sizeof(g_lmf_coef));
    ret = hi_mpi_vi_set_pipe_fisheye_cfg(vi_pipe, &fisheye_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("set fisheye cfg failed.ret:0x%x !\n", ret);
    }

    sample_vio_get_fisheye_correction_attr(&correction_attr, out_size);
    ret = hi_mpi_vi_set_chn_fisheye(vi_pipe, vi_chn, &correction_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set chn fisheye failed.ret:0x%x !\n", ret);
    }
}

static hi_s32 sample_vio_start_vi_ext_chn(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, hi_vi_chn vi_ext_chn)
{
    hi_s32 ret;
    hi_vi_ext_chn_attr ext_chn_attr;
    hi_size out_size;

    sample_comm_vi_get_size_by_sns_type(SENSOR0_TYPE, &out_size);

    ext_chn_attr.bind_chn                       = vi_chn;
    ext_chn_attr.src_type                       = HI_EXT_CHN_SRC_TYPE_BEFORE_FISHEYE;
    ext_chn_attr.size.width                     = out_size.width;
    ext_chn_attr.size.height                    = out_size.height;
    ext_chn_attr.pixel_format                   = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    ext_chn_attr.video_format                   = HI_VIDEO_FORMAT_LINEAR;
    ext_chn_attr.dynamic_range                  = HI_DYNAMIC_RANGE_SDR8;
    ext_chn_attr.compress_mode                  = HI_COMPRESS_MODE_NONE;
    ext_chn_attr.depth                          = 0;
    ext_chn_attr.frame_rate_ctrl.src_frame_rate = -1;
    ext_chn_attr.frame_rate_ctrl.dst_frame_rate = -1;
    ret = hi_mpi_vi_set_ext_chn_attr(vi_pipe, vi_ext_chn, &ext_chn_attr);
    if (ret != HI_SUCCESS) {
        sample_print("set ext chn failed, ret: 0x%x\n", ret);
        return HI_FAILURE;
    }

    ret = hi_mpi_vi_enable_chn(vi_pipe, vi_ext_chn);
    if (ret != HI_SUCCESS) {
        sample_print("enable ext chn failed, ret: 0x%x\n", ret);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

static hi_void sample_vio_stop_vi_ext_chn(hi_vi_pipe vi_pipe, hi_vi_chn vi_ext_chn)
{
    hi_s32 ret;

    ret = hi_mpi_vi_disable_chn(vi_pipe, vi_ext_chn);
    if (ret != HI_SUCCESS) {
        sample_print("disable ext chn failed, ret: 0x%x\n", ret);
    }
}

static hi_s32 sample_vio_start_vi_vpss_double_chn_route(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, sample_vi_cfg *vi_cfg,
                                                        hi_vpss_grp vpss_grp[], hi_u32 grp_num)
{
    hi_s32 ret;
    sample_sns_type sns_type;
    hi_size in_size;
    const hi_vi_chn vi_ext_chn = 4;
    const hi_vpss_chn vpss_chn = 0;

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, vi_cfg);
    ret = sample_comm_vi_start_vi(vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    ret = sample_vio_start_vi_ext_chn(vi_pipe, vi_chn, vi_ext_chn);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    hi_unused(grp_num);

    sample_comm_vi_bind_vpss(vi_pipe, vi_ext_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss0_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[1], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[1], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss1_failed;
    }

    return HI_SUCCESS;

start_vpss1_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[1], vpss_chn);
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss0_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_ext_chn, vpss_grp[0], vpss_chn);
    sample_vio_stop_vi_ext_chn(vi_pipe, vi_ext_chn);
    sample_comm_vi_stop_vi(vi_cfg);
start_vi_failed:
    return ret;
}

static hi_void sample_vio_stop_vi_vpss_double_chn_route(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn, sample_vi_cfg *vi_cfg,
                                                        hi_vpss_grp vpss_grp[], hi_u32 grp_num)
{
    const hi_vi_chn vi_ext_chn = 4;
    const hi_vpss_chn vpss_chn = 0;

    hi_unused(grp_num);

    sample_vio_stop_vpss(vpss_grp[1]);
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[1], vpss_chn);
    sample_vio_stop_vpss(vpss_grp[0]);
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_ext_chn, vpss_grp[0], vpss_chn);
    sample_vio_stop_vi_ext_chn(vi_pipe, vi_ext_chn);
    sample_comm_vi_stop_vi(vi_cfg);
}

static hi_s32 sample_vio_fisheye(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_ONLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[2] = {0, 1};
    const hi_u32 grp_num = 2;
    sample_vi_cfg vi_cfg;
    hi_size out_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_MULTI_YUV_CNT, VB_RAW_CNT_NONE);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    ret = sample_vio_start_vi_vpss_double_chn_route(vi_pipe, vi_chn, &vi_cfg, vpss_grp, grp_num);
    if (ret != HI_SUCCESS) {
        goto start_vi_vpss_failed;
    }

    sample_comm_vi_get_size_by_sns_type(SENSOR0_TYPE, &out_size);
    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &out_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_vio_set_fisheye_param(vi_pipe, vi_chn, &out_size);
    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vi_vpss_double_chn_route(vi_pipe, vi_chn, &vi_cfg, vpss_grp, grp_num);
start_vi_vpss_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vio_switch_low_delay(hi_vi_pipe vi_pipe, hi_vi_chn vi_chn)
{
    hi_s32 ret;
    hi_low_delay_info low_delay_info;

    low_delay_info.enable = HI_TRUE;
    low_delay_info.line_cnt = 300; /* 300: low delay line cnt */
    low_delay_info.one_buf_en = HI_FALSE;

    printf("please enter any key to enable pipe low delay!\n");
    sample_get_char();

    ret = hi_mpi_vi_set_pipe_low_delay_attr(vi_pipe, &low_delay_info);
    if (ret != HI_SUCCESS) {
        sample_print("enable pipe low delay failed!\n");
    }

    printf("please enter any key to disable pipe low delay!\n");
    sample_get_char();

    low_delay_info.enable = HI_FALSE;
    ret = hi_mpi_vi_set_pipe_low_delay_attr(vi_pipe, &low_delay_info);
    if (ret != HI_SUCCESS) {
        sample_print("disable pipe low delay failed!\n");
    }

    printf("please enter any key to enable chn low delay!\n");
    sample_get_char();

    low_delay_info.enable = HI_TRUE;
    ret = hi_mpi_vi_set_chn_low_delay_attr(vi_pipe, vi_chn, &low_delay_info);
    if (ret != HI_SUCCESS) {
        sample_print("enable chn low delay failed!\n");
    }

    printf("please enter any key to disable chn low delay!\n");
    sample_get_char();

    low_delay_info.enable = HI_FALSE;
    ret = hi_mpi_vi_set_chn_low_delay_attr(vi_pipe, vi_chn, &low_delay_info);
    if (ret != HI_SUCCESS) {
        sample_print("disable chn low delay failed!\n");
    }
}

static hi_s32 sample_vio_lowdelay(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_YUV_ROUTE_CNT, VB_LINEAR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_vio_switch_low_delay(vi_pipe, vi_chn);
    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_s32 sample_vio_yuv_4mux(hi_void)
{
    printf("Not support!\n");
    return HI_SUCCESS;
}

static hi_void sample_switch_user_pic(hi_vi_pipe vi_pipe)
{
    hi_s32 ret;
    sample_vi_user_pic_type user_pic_type;
    sample_vi_user_frame_info user_frame_info;

    for (user_pic_type = VI_USER_PIC_FRAME; user_pic_type <= VI_USER_PIC_BGCOLOR; user_pic_type++) {
        ret = sample_common_vi_load_user_pic(vi_pipe, user_pic_type, &user_frame_info);
        if (ret != HI_SUCCESS) {
            sample_print("load user pic failed!\n");
            return;
        }

        ret = hi_mpi_vi_set_user_pic(vi_pipe, &user_frame_info.frame_info);
        if (ret != HI_SUCCESS) {
            sample_print("hi_mpi_vi_set_user_pic failed!\n");
        }

        printf("Enter any key to enable user pic!\n");
        sample_get_char();
        ret = hi_mpi_vi_enable_user_pic(vi_pipe);
        if (ret != HI_SUCCESS) {
            sample_print("hi_mpi_vi_enable_user_pic failed!\n");
        }

        printf("Enter any key to disable user pic!\n");
        sample_get_char();
        ret = hi_mpi_vi_disable_user_pic(vi_pipe);
        if (ret != HI_SUCCESS) {
            sample_print("hi_mpi_vi_disable_user_pic failed!\n");
        }

        sleep(1);
        sample_common_vi_unload_user_pic(&user_frame_info);
    }
}

static hi_s32 sample_vio_user_pic(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[1] = {0};
    const hi_u32 grp_num = 1;
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg;
    sample_sns_type sns_type;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_YUV_ROUTE_CNT, VB_LINEAR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sns_type = SENSOR0_TYPE;
    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    ret = sample_comm_vi_start_vi(&vi_cfg);
    if (ret != HI_SUCCESS) {
        goto start_vi_failed;
    }

    sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    ret = sample_vio_start_vpss(vpss_grp[0], &in_size);
    if (ret != HI_SUCCESS) {
        goto start_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_switch_user_pic(vi_pipe);
    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
start_vpss_failed:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg);
start_vi_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_s32 sample_vio_start_multi_vi_vpss(sample_vi_cfg *vi_cfg, hi_vpss_grp *vpss_grp,
                                             hi_s32 dev_num, hi_s32 grp_num)
{
    hi_s32 ret;
    hi_s32 i, j;
    hi_size in_size;
    sample_sns_type sns_type = SENSOR0_TYPE; /* default: use same sensors */

    if (dev_num != grp_num) {
        return HI_FAILURE;
    }

    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);

    for (i = 0; i < dev_num; i++) {
        ret = sample_comm_vi_start_vi(&vi_cfg[i]);
        if (ret != HI_SUCCESS) {
        goto start_vi_failed;
        }
    }

    for (i = 0; i < grp_num; i++) {
        sample_comm_vi_bind_vpss(i, 0, vpss_grp[i], 0);
    }

    for (i = 0; i < grp_num; i++) {
        ret = sample_vio_start_vpss(vpss_grp[i], &in_size);
        if (ret != HI_SUCCESS) {
            goto start_vpss_failed;
        }
    }

    return HI_SUCCESS;

start_vpss_failed:
    for (j = i - 1; j >= 0; j--) {
        sample_vio_stop_vpss(vpss_grp[j]);
    }

    for (i = 0; i < grp_num; i++) {
        sample_comm_vi_un_bind_vpss(i, 0, vpss_grp[i], 0);
    }

start_vi_failed:
    for (j = i - 1; j >= 0; j--) {
        sample_comm_vi_stop_vi(&vi_cfg[j]);
    }

    return HI_FAILURE;
}

/* Set pin_mux i2c2 & i2c5 & sensor0 & sensor1 & MIPI0 & MIPI1 before using this sample! */
static hi_s32 sample_vio_two_sensor(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_OFFLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    hi_vi_pipe vi_pipe[2] = {0, 1}; /* 2 pipe */
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[2] = {0, 1}; /* 2 vpss grp */
    const hi_u32 grp_num = 2; /* 2 vpss grp */
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg[2];
    sample_sns_type sns_type = SENSOR0_TYPE;
    hi_size in_size;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_DOUBLE_YUV_CNT, VB_WDR_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sample_vi_get_two_sensor_vi_cfg(sns_type, &vi_cfg[0], &vi_cfg[1]);
    ret = sample_vio_start_multi_vi_vpss(vi_cfg, vpss_grp, 2, 2);  /* start 2 route */
    if (ret != HI_SUCCESS) {
        goto start_vi_vpss_failed;
    }

    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);
    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    sample_vio_stop_vpss(vpss_grp[0]);
    sample_vio_stop_vpss(vpss_grp[1]);
    sample_comm_vi_un_bind_vpss(vi_pipe[0], vi_chn, vpss_grp[0], vpss_chn);
    sample_comm_vi_un_bind_vpss(vi_pipe[1], vi_chn, vpss_grp[1], vpss_chn);
    sample_comm_vi_stop_vi(&vi_cfg[0]);
    sample_comm_vi_stop_vi(&vi_cfg[1]);
start_vi_vpss_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vi_get_four_vi_cfg(sample_sns_type sns_type, sample_vi_cfg *vi_cfg, hi_s32 dev_num)
{
    hi_s32 i;

    for (i = 0; i < dev_num; i++) {
        sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg[i]);
        sample_comm_vi_get_mipi_info_by_dev_id(sns_type, i, &vi_cfg[i].mipi_info);
        vi_cfg[i].mipi_info.divide_mode = LANE_DIVIDE_MODE_3;

        vi_cfg[i].dev_info.vi_dev = i;
        vi_cfg[i].bind_pipe.pipe_id[0] = i;
        vi_cfg[i].grp_info.grp_num = 1;
        vi_cfg[i].grp_info.fusion_grp[0] = i;
        vi_cfg[i].grp_info.fusion_grp_attr[0].pipe_id[0] = i;

        vi_cfg[i].sns_info.sns_clk_src = i;
        vi_cfg[i].sns_info.sns_rst_src = i;
    }

    vi_cfg[0].sns_info.bus_id = 2; /* arg0: i2c5 */
    vi_cfg[1].sns_info.bus_id = 3; /* arg1: i2c3 */
    vi_cfg[2].sns_info.bus_id = 5; /* arg2: i2c5 */
    vi_cfg[3].sns_info.bus_id = 4; /* arg3: i2c4 */
}

/* Set pin_mux i2c2-5 & sensor0-3 & sensor_slave0/1_en & MIPI0 & MIPI1 before using this sample! */
static hi_s32 sample_vio_four_sensor(hi_void)
{
    hi_s32 ret;
    hi_vi_vpss_mode_type mode_type = HI_VI_OFFLINE_VPSS_ONLINE;
    hi_vi_video_mode video_mode = HI_VI_VIDEO_MODE_NORM;
    const hi_vi_chn vi_chn = 0;
    hi_vpss_grp vpss_grp[4] = {0, 1, 2, 3}; /* 4 vpss grp:0/1/2/3 */
    const hi_u32 grp_num = 4; /* 4 vpss grp */
    const hi_vpss_chn vpss_chn = 0;
    sample_vi_cfg vi_cfg[HI_VI_MAX_DEV_NUM];
    sample_sns_type sns_type = SENSOR0_TYPE;
    hi_size in_size;
    hi_s32 i;

    ret = sample_vio_sys_init(mode_type, video_mode, VB_MULTI_YUV_CNT, VB_MULTI_RAW_CNT);
    if (ret != HI_SUCCESS) {
        goto sys_init_failed;
    }

    sample_comm_vi_get_size_by_sns_type(sns_type, &in_size);

    sample_vi_get_four_vi_cfg(sns_type, vi_cfg, HI_VI_MAX_DEV_NUM);
    ret = sample_vio_start_multi_vi_vpss(vi_cfg, vpss_grp, HI_VI_MAX_DEV_NUM, HI_VI_MAX_DEV_NUM); // 4 route
    if (ret != HI_SUCCESS) {
        goto start_vi_vpss_failed;
    }

    ret = sample_vio_start_venc_and_vo(vpss_grp, grp_num, &in_size);
    if (ret != HI_SUCCESS) {
        goto start_venc_and_vo_failed;
    }

    sample_get_char();

    sample_vio_stop_venc_and_vo(vpss_grp, grp_num);

start_venc_and_vo_failed:
    for (i = 0; i < HI_VI_MAX_DEV_NUM; i++) {
        sample_vio_stop_vpss(vpss_grp[i]);
        sample_comm_vi_un_bind_vpss(i, vi_chn, vpss_grp[i], vpss_chn);
    }
    sample_comm_vi_stop_four_vi(vi_cfg, HI_VI_MAX_DEV_NUM);
start_vi_vpss_failed:
    sample_comm_sys_exit();
sys_init_failed:
    return ret;
}

static hi_void sample_vio_usage(char *prg_name)
{
    printf("usage : %s <index> \n", prg_name);
    printf("index:\n");
    printf("    (0) all mode route          :vi linear(Online/Offline) -> vpss(Online/Offline) -> venc && vo.\n");
    printf("    (1) wdr route               :vi wdr(Online) -> vpss(Offline) -> venc && vo.\n");
    printf("    (2) fpn calibrate & correct :vi fpn calibrate & correct -> vpss -> venc && vo.\n");
    printf("    (3) ldc & dis               :vi ldc & dis -> vpss -> venc && vo.\n");
    printf("    (4) fisheye                 :vi fisheye(phys_chn + ext_chn) -> vpss -> venc && vo.\n");
    printf("    (5) low delay               :vi(pipe & chn lowdelay) -> vpss(lowdelay) -> venc && vo.\n");
    printf("    (6) input yuv mux           :vi 4mux yuv input -> vpss -> venc && vo.\n");
    printf("    (7) user pic                :vi user pic (offline) -> vpss -> venc && vo.\n");
    printf("    (8) two sensor              :vi two sensor (offline) -> vpss -> venc && vo.\n");
    printf("    (9) four sensor             :vi four sensor (offline) -> vpss -> venc && vo.\n");
    printf("    (10) switch mode            :vi linear switch to wdr -> vpss -> venc && vo.\n");
    printf("    (11) wdr send route         :vi wdr(offline) -> isp_runonce -> vpss(Offline) -> venc && vo.\n");
    printf("    (12) switch resolution      :vi 4K switch to FHD -> vpss -> venc && vo.\n");
}

static hi_void sample_vio_handle_sig(hi_s32 signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_sig_flag = 1;
    }
}

static hi_void sample_register_sig_handler(hi_void (*sig_handle)(hi_s32))
{
    struct sigaction sa;

    (hi_void)memset_s(&sa, sizeof(struct sigaction), 0, sizeof(struct sigaction));
    sa.sa_handler = sig_handle;
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, HI_NULL);
    sigaction(SIGTERM, &sa, HI_NULL);
}

static hi_s32 sample_vio_execute_case(hi_u32 case_index)
{
    hi_s32 ret;

    switch (case_index) {
        case 0: /* 0 all mode route */
            ret = sample_vio_all_mode();
            break;
        case 1: /* 1 wdr route */
            ret = sample_vio_wdr();
            break;
        case 2: /* 2 fpn calibrate and correct */
            ret = sample_vio_fpn();
            break;
        case 3: /* 3 ldc and dis */
            ret = sample_vio_ldc_dis();
            break;
        case 4: /* 4 fisheye */
            ret = sample_vio_fisheye();
            break;
        case 5: /* 5 low delay */
            ret = sample_vio_lowdelay();
            break;
        case 6: /* 6 yuv input mux */
            ret = sample_vio_yuv_4mux();
            break;
        case 7: /* 7 user pic */
            ret = sample_vio_user_pic();
            break;
        case 8: /* 8 two sensor */
            ret = sample_vio_two_sensor();
            break;
        case 9: /* 9 four sensor */
            ret = sample_vio_four_sensor();
            break;
        case 10: /* 10 switch mode */
            ret = sample_vio_switch_mode();
            break;
        case 11: /* 11 wdr runonce route */
            ret = sample_vio_wdr_send_frame();
            break;
        case 12: /* 12 switch resolution */
            ret = sample_vio_switch_resolution();
            break;
        default:
            ret = HI_FAILURE;
            break;
    }

    return ret;
}

#ifdef __LITEOS__
hi_s32 app_main(hi_s32 argc, hi_char *argv[])
#else
hi_s32 main(hi_s32 argc, hi_char *argv[])
#endif
{
    hi_s32 ret;
    hi_u32 index;

    if (argc != 2) { /* 2:arg num */
        sample_vio_usage(argv[0]);
        return HI_FAILURE;
    }

    if (!strncmp(argv[1], "-h", 2)) { /* 2:arg num */
        sample_vio_usage(argv[0]);
        return HI_FAILURE;
    }

    if (strlen(argv[1]) > 2 || strlen(argv[1]) <= 0 || !check_digit(argv[1][0]) || /* 2:arg len */
        (strlen(argv[1]) == 2 && (!check_digit(argv[1][1]) || argv[1][0] == '0'))) { /* 2:arg len */
        sample_vio_usage(argv[0]);
        return HI_FAILURE;
    }

#ifndef __LITEOS__
    sample_register_sig_handler(sample_vio_handle_sig);
#endif

    index = atoi(argv[1]);
    ret = sample_vio_execute_case(index);
    if ((ret == HI_SUCCESS) && (g_sig_flag == 0)) {
        printf("\033[0;32mprogram exit normally!\033[0;39m\n");
    } else {
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

#ifdef __LITEOS__
    return ret;
#else
    exit(ret);
#endif
}
