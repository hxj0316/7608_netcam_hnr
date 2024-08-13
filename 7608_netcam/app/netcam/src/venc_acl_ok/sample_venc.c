/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Lhi.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include "sample_comm.h"
#include "securec.h"
#include "acl.h"

#define BIG_STREAM_SIZE     PIC_3840X2160
#define SMALL_STREAM_SIZE   PIC_1080P

int SMALL_PIC_WIDTH = 640;
int SMALL_PIC_HEIGHT = 640;

float g_Conf_Thresh = 0.25;
float g_Iou_Thresh = 0.7;

#define PATH_MAX 255
#define MAX_INPUT_NUM 5
#define MAX_OUTPUT_NUM 5
#define NPU_SHOW_TOP_NUM    5
#define PATH_MAX 255
#define MAX_INPUT_NUM 5
#define MAX_OUTPUT_NUM 5

#define NPU_SHOW_TOP_NUM    5
#define MAX_OUTPUT_BOX 8400

const static char *yolov3Label[] = { "person", "bicycle", "car", "motorbike",
"aeroplane","bus", "train", "truck", "boat",
"traffic light", "fire hydrant", "stop sign", "parking meter",
"bench", "bird", "cat", "dog", "horse",
"sheep", "cow", "elephant", "bear", "zebra",
"giraffe", "backpack", "umbrella", "handbag","tie",
"suitcase", "frisbee", "skis", "snowboard", "sports ball",
"kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
"tennis racket", "bottle", "wine glass", "cup",
"fork", "knife", "spoon", "bowl", "banana",
"apple", "sandwich", "orange", "broccoli", "carrot",
"hot dog", "pizza", "donut", "cake", "chair",
"sofa", "potted plant", "bed", "dining table", "toilet",
"TV monitor", "laptop", "mouse", "remote", "keyboard",
"cell phone", "microwave", "oven", "toaster", "sink",
"refrigerator", "book", "clock", "vase","scissors",
"teddy bear", "hair drier", "toothbrush" };

enum BBoxIndex { 
    TOPLEFTX = 0, 
    TOPLEFTY, 
    BOTTOMRIGHTX, 
    BOTTOMRIGHTY, 
    SCORE, 
    LABEL 
};

typedef struct _stBox{
float x;
float y;
float w;
float h;
float conf;
int id;
int vaild;
}stbox;


typedef struct npu_acl_model {
    hi_u32 model_id;
    hi_ulong model_mem_size;
    hi_ulong model_weight_size;
    hi_void *model_mem_ptr;
    hi_void *model_weight_ptr;
    hi_phys_addr_t model_mem_phy_addr;
    hi_phys_addr_t model_weight_phy_addr;
    hi_bool is_load_flag;
    aclmdlDesc *model_desc;
    aclmdlDataset *input_dataset;
    aclmdlDataset *output_dataset;
    hi_phys_addr_t output_phy_addr[MAX_OUTPUT_NUM];
    hi_phys_addr_t input_phy_addr[MAX_INPUT_NUM];
} npu_acl_model_t;

#define VI_VB_YUV_CNT    6
#define VPSS_VB_YUV_CNT  8

#define ENTER_ASCII 10

#define VB_MAX_NUM     10

#define CHN_NUM_MAX    2

typedef struct {
    hi_size            max_size;
    ot_pixel_format    pixel_format;
    hi_size            output_size[OT_VPSS_MAX_PHYS_CHN_NUM];
    ot_compress_mode   compress_mode[OT_VPSS_MAX_PHYS_CHN_NUM];
    hi_bool            enable[OT_VPSS_MAX_PHYS_CHN_NUM];
} sample_venc_vpss_chn_attr;

typedef struct {
    hi_u32 valid_num;
    hi_u64 blk_size[OT_VB_MAX_COMMON_POOLS];
    hi_u32 blk_cnt[OT_VB_MAX_COMMON_POOLS];
    hi_u32 supplement_config;
} sample_venc_vb_attr;

typedef struct {
    ot_vpss_chn vpss_chn[CHN_NUM_MAX];
    ot_venc_chn venc_chn[CHN_NUM_MAX];
} sample_venc_vpss_chn;

/******************************************************************************
* function : show usage
******************************************************************************/
static hi_void sample_venc_usage(hi_char *s_prg_nm)
{
    printf("usage : %s [index] \n", s_prg_nm);
    printf("index:\n");
    printf("\t  0) normal           :H.265e@1080P@30fps + h264e@D1@30fps.\n");
    printf("\t  1) qpmap            :H.265e@1080P@30fps + h264e@1080P@30fps.\n");
    printf("\t  2) intra_refresh    :H.265e@1080P@30fps(row) + h264e@1080P@30fps(column).\n");
    printf("\t  3) roi_bg_frame_rate:H.265e@1080P@30fps + H.264@1080P@30fps.\n");
    printf("\t  4) debreath_effect  :H.265e@1080P@30fps(enable) + H.265e@1080P@30fps(disable).\n");
    printf("\t  5) roi_set          :Mjpege@1080P@30fps(user set roi info by API).\n");
    printf("\t  6) roimap           :Mjpege@1080P@30fps(user customize every region).\n");
}

static hi_bool g_sample_venc_exit = HI_FALSE;

/******************************************************************************
* function : to process abnormal case
******************************************************************************/
static hi_void sample_venc_handle_sig(hi_s32 signo)
{
    if (g_sample_venc_exit == HI_TRUE) {
        return;
    }

    if (signo == SIGINT || signo == SIGTERM) {
        g_sample_venc_exit = HI_TRUE;
    }
}

static hi_s32 sample_venc_getchar()
{
    hi_s32 c;
    if (g_sample_venc_exit == HI_TRUE) {
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return 'e';
    }

    c = getchar();

    if (g_sample_venc_exit == HI_TRUE) {
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return 'e';
    }

    return c;
}

static hi_s32 get_gop_mode(ot_venc_gop_mode *gop_mode)
{
    hi_s32 c[2] = {'\0'}; /* 2: len */

begin_get:
    printf("please input choose gop mode!\n");
    printf("\t 0) normal p.\n");
    printf("\t 1) dual p.\n");
    printf("\t 2) smart p.\n");

    c[0] = sample_venc_getchar();
    if (c[0] == 'e') {
        return HI_FAILURE;
    } else if ((c[0] == ENTER_ASCII) || ((c[0] != '0') && (c[0] != '1') && (c[0] != '2'))) {
        sample_print("invalid input! please try again.\n");
        while (sample_venc_getchar() != ENTER_ASCII) {
        }
        goto begin_get;
    }

    c[1] = sample_venc_getchar();
    if (c[1] != ENTER_ASCII) {
        sample_print("invalid input! please try again.\n");
        while (sample_venc_getchar() != ENTER_ASCII) {
        }
        goto begin_get;
    }
    switch (c[0]) {
        case '0':
            *gop_mode = OT_VENC_GOP_MODE_NORMAL_P;
            break;

        case '1':
            *gop_mode = OT_VENC_GOP_MODE_DUAL_P;
            break;

        case '2':
            *gop_mode = OT_VENC_GOP_MODE_SMART_P;
            break;

        default:
            sample_print("input invalid!\n");
            goto begin_get;
    }

    return HI_SUCCESS;
}

static hi_void print_rc_mode(ot_payload_type type)
{
    printf("please input choose rc mode!\n");
    printf("\t c) cbr.\n");
    printf("\t v) vbr.\n");
    if (type != OT_PT_MJPEG) {
        printf("\t a) avbr.\n");
        printf("\t x) cvbr.\n");
        printf("\t q) qvbr.\n");
    }
    printf("\t f) fix_qp\n");
}

static hi_s32 get_rc_mode(ot_payload_type type, sample_rc *rc_mode)
{
    hi_s32 c[2] = {'\0'}; /* 2: len */

begin_get:
    if (type != OT_PT_JPEG) {
        print_rc_mode(type);

        c[0] = sample_venc_getchar();
        if (c[0] == 'e') {
            return HI_FAILURE;
        } else if ((c[0] == ENTER_ASCII) || ((c[0] != 'c') && (c[0] != 'v') && (c[0] != 'a') && (c[0] != 'x') &&
            (c[0] != 'q') && (c[0] != 'f'))) {
            sample_print("invalid input! please try again.\n");
            while (sample_venc_getchar() != ENTER_ASCII) {
            }
            goto begin_get;
        }
        c[1] = sample_venc_getchar();
        if (c[1] != ENTER_ASCII) {
            sample_print("invalid input! please try again.\n");
            while (sample_venc_getchar() != ENTER_ASCII) {
            }
            goto begin_get;
        }
        switch (c[0]) {
            case 'c':
                *rc_mode = SAMPLE_RC_CBR;
                break;

            case 'v':
                *rc_mode = SAMPLE_RC_VBR;
                break;

            case 'a':
                *rc_mode = SAMPLE_RC_AVBR;
                break;

            case 'x':
                *rc_mode = SAMPLE_RC_CVBR;
                break;

            case 'q':
                *rc_mode = SAMPLE_RC_QVBR;
                break;

            case 'f':
                *rc_mode = SAMPLE_RC_FIXQP;
                break;

            default:
                sample_print("input invalid!\n");
                goto begin_get;
        }
    }

    return HI_SUCCESS;
}

static hi_s32 get_intra_refresh_mode(ot_venc_intra_refresh_mode *intra_refresh_mode)
{
    hi_s32 c[2] = {'\0'}; /* 2: len */

begin_get:
    printf("please input choose intra refresh mode!\n");
    printf("\t r) row.\n");
    printf("\t c) column.\n");

    c[0] = sample_venc_getchar();
    if (c[0] == 'e') {
        return HI_FAILURE;
    } else if ((c[0] == ENTER_ASCII) || ((c[0] != 'c') && (c[0] != 'r'))) {
        sample_print("invalid input! please try again.\n");
        while (sample_venc_getchar() != ENTER_ASCII) {
        }
        goto begin_get;
    }
    c[1] = sample_venc_getchar();
    if (c[1] != ENTER_ASCII) {
        sample_print("invalid input! please try again.\n");
        while (sample_venc_getchar() != ENTER_ASCII) {
        }
        goto begin_get;
    }
    switch (c[0]) {
        case 'r':
            *intra_refresh_mode = OT_VENC_INTRA_REFRESH_ROW;
            break;

        case 'c':
            *intra_refresh_mode = OT_VENC_INTRA_REFRESH_COLUMN;
            break;

        default:
            sample_print("input invalid!\n");
            goto begin_get;
    }

    return HI_SUCCESS;
}

static hi_void get_vb_attr(const hi_size *vi_size, const sample_venc_vpss_chn_attr *vpss_chn_attr,
    sample_venc_vb_attr *vb_attr)
{
    hi_s32 i;
    ot_pic_buf_attr pic_buf_attr = {0};

    vb_attr->valid_num = 0;

    // vb for vi-vpss
    pic_buf_attr.width  = vi_size->width;
    pic_buf_attr.height = vi_size->height;
    pic_buf_attr.align = OT_DEFAULT_ALIGN;
    pic_buf_attr.bit_width = OT_DATA_BIT_WIDTH_8;
    pic_buf_attr.pixel_format = OT_PIXEL_FORMAT_YUV_SEMIPLANAR_422;
    pic_buf_attr.compress_mode = OT_COMPRESS_MODE_NONE;
    vb_attr->blk_size[vb_attr->valid_num] = ot_common_get_pic_buf_size(&pic_buf_attr);
    vb_attr->blk_cnt[vb_attr->valid_num] = VI_VB_YUV_CNT;
    vb_attr->valid_num++;

    // vb for vpss-venc(big stream)
    if (vb_attr->valid_num >= OT_VB_MAX_COMMON_POOLS) {
        return;
    }

    for (i = 0; i < OT_VPSS_MAX_PHYS_CHN_NUM && vb_attr->valid_num < OT_VB_MAX_COMMON_POOLS; i++) {
        if (vpss_chn_attr->enable[i] == HI_TRUE) {
            pic_buf_attr.width = vpss_chn_attr->output_size[i].width;
            pic_buf_attr.height = vpss_chn_attr->output_size[i].height;
            pic_buf_attr.align = OT_DEFAULT_ALIGN;
            pic_buf_attr.bit_width = OT_DATA_BIT_WIDTH_8;
            pic_buf_attr.pixel_format = vpss_chn_attr->pixel_format;
            pic_buf_attr.compress_mode = vpss_chn_attr->compress_mode[i];
            if (pic_buf_attr.compress_mode == OT_COMPRESS_MODE_SEG_COMPACT) {
                ot_vb_calc_cfg calc_cfg = {0};
                ot_common_get_vpss_compact_seg_buf_size(&pic_buf_attr, &calc_cfg);
                vb_attr->blk_size[vb_attr->valid_num] = calc_cfg.vb_size;
            } else {
                vb_attr->blk_size[vb_attr->valid_num] = ot_common_get_pic_buf_size(&pic_buf_attr);
            }
            vb_attr->blk_cnt[vb_attr->valid_num] = VPSS_VB_YUV_CNT;

            vb_attr->valid_num++;
        }
    }

    vb_attr->supplement_config = OT_VB_SUPPLEMENT_JPEG_MASK | OT_VB_SUPPLEMENT_BNR_MOT_MASK;
}

static hi_void get_default_vpss_chn_attr(hi_size *vi_size, hi_size enc_size[], hi_s32 len,
    sample_venc_vpss_chn_attr *vpss_chan_attr)
{
    hi_s32 i;
    hi_u32 max_width;
    hi_u32 max_height;

    if (memset_s(vpss_chan_attr, sizeof(sample_venc_vpss_chn_attr), 0, sizeof(sample_venc_vpss_chn_attr)) != EOK) {
        printf("vpss chn attr call memset_s error\n");
        return;
    }

    max_width = vi_size->width;
    max_height = vi_size->height;

    for (i = 0; (i < len) && (i < OT_VPSS_MAX_PHYS_CHN_NUM); i++) {
        vpss_chan_attr->output_size[i].width = enc_size[i].width;
        vpss_chan_attr->output_size[i].height = enc_size[i].height;
        vpss_chan_attr->compress_mode[i] = (i == 0) ? OT_COMPRESS_MODE_SEG_COMPACT : OT_COMPRESS_MODE_NONE;
        vpss_chan_attr->enable[i] = HI_TRUE;

        max_width = MAX2(max_width, enc_size[i].width);
        max_height = MAX2(max_height, enc_size[i].height);
    }

    vpss_chan_attr->max_size.width = max_width;
    vpss_chan_attr->max_size.height = max_height;
    vpss_chan_attr->pixel_format = OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420;

    return;
}

static hi_s32 sample_venc_sys_init(sample_venc_vb_attr *vb_attr)
{
    hi_u32 i;
    hi_s32 ret;
    ot_vb_cfg vb_cfg = {0};

    if (vb_attr->valid_num > OT_VB_MAX_COMMON_POOLS) {
        sample_print("sample_venc_sys_init vb valid num(%d) too large than OT_VB_MAX_COMMON_POOLS(%d)!\n",
            vb_attr->valid_num, OT_VB_MAX_COMMON_POOLS);
        return HI_FAILURE;
    }

    for (i = 0; i < vb_attr->valid_num; i++) {
        vb_cfg.common_pool[i].blk_size = vb_attr->blk_size[i];
        vb_cfg.common_pool[i].blk_cnt = vb_attr->blk_cnt[i];
    }

    vb_cfg.max_pool_cnt = vb_attr->valid_num;

    if (vb_attr->supplement_config == 0) {
        ret = sample_comm_sys_init(&vb_cfg);
    } else {
        ret = sample_comm_sys_init_with_vb_supplement(&vb_cfg, vb_attr->supplement_config);
    }

    if (ret != HI_SUCCESS) {
        sample_print("sample_venc_sys_init failed!\n");
    }

    return ret;
}

static hi_s32 sample_venc_vi_init(sample_vi_cfg *vi_cfg)
{
    hi_s32 ret;

    ret = sample_comm_vi_start_vi(vi_cfg);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_vi_start_vi failed: 0x%x\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_void sample_venc_vi_deinit(sample_vi_cfg *vi_cfg)
{
    sample_comm_vi_stop_vi(vi_cfg);
}

static hi_s32 sample_venc_vpss_init(ot_vpss_grp vpss_grp, sample_venc_vpss_chn_attr *vpss_chan_cfg)
{
    hi_s32 ret;
    ot_vpss_chn vpss_chn;
    ot_vpss_grp_attr grp_attr = { 0 };
    ot_vpss_chn_attr chn_attr[OT_VPSS_MAX_PHYS_CHN_NUM] = {0};

    grp_attr.max_width = vpss_chan_cfg->max_size.width;
    grp_attr.max_height = vpss_chan_cfg->max_size.height;
    grp_attr.nr_en = HI_FALSE;
    grp_attr.dei_mode = OT_VPSS_DEI_MODE_OFF;
    grp_attr.pixel_format = vpss_chan_cfg->pixel_format;
    grp_attr.frame_rate.src_frame_rate = -1;
    grp_attr.frame_rate.dst_frame_rate = -1;

    for (vpss_chn = 0; vpss_chn < OT_VPSS_MAX_PHYS_CHN_NUM; vpss_chn++) {
        if (vpss_chan_cfg->enable[vpss_chn] == 1) {
            chn_attr[vpss_chn].width = vpss_chan_cfg->output_size[vpss_chn].width;
            chn_attr[vpss_chn].height = vpss_chan_cfg->output_size[vpss_chn].height;
            chn_attr[vpss_chn].chn_mode = OT_VPSS_CHN_MODE_USER;
            chn_attr[vpss_chn].compress_mode = vpss_chan_cfg->compress_mode[vpss_chn];
            chn_attr[vpss_chn].pixel_format = vpss_chan_cfg->pixel_format;
            chn_attr[vpss_chn].frame_rate.src_frame_rate = -1;
            chn_attr[vpss_chn].frame_rate.dst_frame_rate = -1;
            chn_attr[vpss_chn].depth = 2;
            chn_attr[vpss_chn].mirror_en = 0;
            chn_attr[vpss_chn].flip_en = 0;
            chn_attr[vpss_chn].aspect_ratio.mode = OT_ASPECT_RATIO_NONE;
        }
    }

    ret = sample_common_vpss_start(vpss_grp, vpss_chan_cfg->enable, &grp_attr, chn_attr, OT_VPSS_MAX_PHYS_CHN_NUM);
    if (ret != HI_SUCCESS) {
        sample_print("failed with %#x!\n", ret);
    }

    return ret;
}

static hi_void sample_venc_vpss_deinit(ot_vpss_grp vpss_grp, sample_venc_vpss_chn_attr *vpss_chan_cfg)
{
    hi_s32 ret;

    ret = sample_common_vpss_stop(vpss_grp, vpss_chan_cfg->enable, OT_VPSS_MAX_PHYS_CHN_NUM);
    if (ret != HI_SUCCESS) {
        sample_print("failed with %#x!\n", ret);
    }
}

static hi_s32 sample_venc_init_param(hi_size *enc_size, hi_s32 chn_num_max, hi_size *vi_size,
    sample_venc_vpss_chn_attr *vpss_param)
{
    hi_s32 i;
    hi_s32 ret;
    hi_pic_size pic_size[CHN_NUM_MAX] = {BIG_STREAM_SIZE, SMALL_STREAM_SIZE};

    for (i = 0; i < chn_num_max && i < CHN_NUM_MAX; i++) {
        ret = sample_comm_sys_get_pic_size(pic_size[i], &enc_size[i]);
        if (ret != HI_SUCCESS) {
            sample_print("sample_comm_sys_get_pic_size failed!\n");
            return ret;
        }
    }

    // get vpss param
    get_default_vpss_chn_attr(vi_size, enc_size, CHN_NUM_MAX, vpss_param);

    return 0;
}

static hi_void sample_venc_set_video_param(sample_comm_venc_chn_param *chn_param,
    ot_venc_gop_attr gop_attr, hi_s32 chn_num_max, hi_bool qp_map)
{
    hi_u32 profile[CHN_NUM_MAX] = {0, 0};
    hi_bool share_buf_en = HI_TRUE;
    hi_pic_size pic_size[CHN_NUM_MAX] = {BIG_STREAM_SIZE, SMALL_STREAM_SIZE};
    ot_payload_type payload[CHN_NUM_MAX] = {OT_PT_H265, OT_PT_H264};
    sample_rc rc_mode = 0;

    if (qp_map) {
        rc_mode = SAMPLE_RC_QPMAP;
    } else {
        if (get_rc_mode(payload[0], &rc_mode) != HI_SUCCESS) {
            return;
        }
    }

    if (chn_num_max < 2) { /* 2: chn_param array len */
        sample_print("chn_num_max  %d not enough! should > 2\n", chn_num_max);
        return;
    }

    /* encode h.265 */
    chn_param[0].gop_attr = gop_attr;
    chn_param[0].type = payload[0];
    chn_param[0].size = pic_size[0];
    chn_param[0].rc_mode = rc_mode;
    chn_param[0].profile = profile[0];
    chn_param[0].is_rcn_ref_share_buf = share_buf_en;

    /* encode h.264 */
    chn_param[1].gop_attr = gop_attr;
    chn_param[1].type = payload[1];
    chn_param[1].size = pic_size[1];
    chn_param[1].rc_mode = rc_mode;
    chn_param[1].profile = profile[1];
    chn_param[1].is_rcn_ref_share_buf = share_buf_en;
}

static hi_void sample_set_venc_vpss_chn(sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 i;

    for (i = 0; i < CHN_NUM_MAX; i++) {
        venc_vpss_chn->vpss_chn[i] = i;
        venc_vpss_chn->venc_chn[i] = i;
    }
}

static hi_void sample_venc_unbind_vpss_stop(ot_vpss_grp vpss_grp, const sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 i;

    for (i = 0; i < CHN_NUM_MAX; i++) {
        sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[i], venc_vpss_chn->venc_chn[i]);
        sample_comm_venc_stop(venc_vpss_chn->venc_chn[i]);
    }
}

static hi_void sample_venc_stop(const sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 i;

    for (i = 0; i < CHN_NUM_MAX; i++) {
        sample_comm_venc_stop(venc_vpss_chn->venc_chn[i]);
    }
}

static hi_s32 sample_venc_normal_start_encode(ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_mode gop_mode;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param chn_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param *h265_chn_param = HI_NULL;
    sample_comm_venc_chn_param *h264_chn_param = HI_NULL;

    if (get_gop_mode(&gop_mode) != HI_SUCCESS) {
        return HI_FAILURE;
    }
    if ((ret = sample_comm_venc_get_gop_attr(gop_mode, &gop_attr)) != HI_SUCCESS) {
        sample_print("Venc Get GopAttr for %#x!\n", ret);
        return ret;
    }

    sample_venc_set_video_param(chn_param, gop_attr, CHN_NUM_MAX, HI_FALSE);

    /* encode h.265 */
    h265_chn_param = &(chn_param[0]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], h265_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    ret = sample_comm_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_vpss_bind_venc failed for %#x!\n", ret);
        goto EXIT_VENC_H265_STOP;
    }

    /* encode h.264 */
    h264_chn_param = &(chn_param[1]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], h264_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_H265_UnBind;
    }

    ret = sample_comm_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_vpss_bind_venc failed for %#x!\n", ret);
        goto EXIT_VENC_H264_STOP;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
        goto EXIT_VENC_H264_UnBind;
    }

    return HI_SUCCESS;

EXIT_VENC_H264_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H264_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H265_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
EXIT_VENC_H265_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

static hi_void sample_venc_exit_process()
{
    printf("please press twice ENTER to exit this sample\n");
    (hi_void)getchar();

    if (g_sample_venc_exit != HI_TRUE) {
        (hi_void)getchar();
    }
    sample_comm_venc_stop_get_stream(CHN_NUM_MAX);
}

static int sortBboxConf(stbox *box, int lenth){
    int ret = 0;
    if(lenth > 1){
        stbox boxTmp;
        float confMax = 0.0f;
        int idx = 0;
        for(int i =0; i<lenth-1; i++){
            confMax = box[i].conf;
            idx = i;
            for(int j=i+1; j<lenth; j++){
                if(box[j].conf > confMax){
                    confMax = box[j].conf;
                    idx = j;
                }
            }
            //如果找到最大conf，交换位置
            if(idx != i){
                boxTmp = box[i];
                box[i] = box[idx];
                box[idx] = boxTmp;
            }
        }

        ret = 1;
    }

    return ret;
}

static float calcIOU(stbox box0, stbox box1){
    float iou = 0.0f;
    float interAera = 0.0f;
    float unionAera = 1.0f;
    float x0max = box0.x + box0.w;
    float y0max = box0.y + box0.h;
    float x1max = box1.x + box1.w;
    float y1max = box1.y + box1.h;   
    //判断box是否有交集
    if((x0max > box1.x) && (y0max > box1.y) && (x1max > box0.x) && (y1max > box0.y)){
        //计算box交集
        if(x1max > x0max){
            if(y1max > y0max){
                interAera = (x0max - box1.x) * (y0max - box1.y); 
            }else{
                interAera = (x0max - box1.x) * (y1max - box0.y); 
            }
        }else{
             if(y1max > y0max){
                interAera = (x1max - box0.x) * (y0max - box1.y); 
            }else{
                interAera = (x1max - box0.x) * (y1max - box0.y); 
            }           
        }
        //计算box并集
        unionAera = box0.w * box0.h + box1.w * box1.h - interAera;
        //计算交并比 = 交集/并集，需要保证分母不为0
        if(unionAera > 0.0f){
            iou = interAera / unionAera;
        }
    }

    return iou;
}

static int nmsBbox(stbox *box, int lenth){
    int ret = 0;
    if(lenth > 1){
        float iou = 0.0f;
        for(int i =0; i<lenth-1; i++){
            if(1 == box[i].vaild){//第i个box有效
                for(int j=i+1; j<lenth; j++){
                    if(1 == box[j].vaild){//第j个box有效
                        iou = calcIOU(box[i], box[j]);
                        if(iou > g_Iou_Thresh){
                            box[j].vaild = 0;
                        }                         
                    }
                }
            }
        }

        ret = 1;
    }

    return ret;
}

hi_s32 sample_acl(hi_void)
{
    td_s32 ret;
    ot_video_frame_info frame_info;
    //char *v = NULL;
    //char *u = NULL;
    int i,j;
    FILE   *fp;

    //ret = ss_mpi_snap_enable_pipe(0);
    //if (ret != TD_SUCCESS) {
    //    printf("ss_mpi_snap_enable_pipe failed, ret: 0x%x\n", ret);
    //    return TD_FAILURE;
    //}
    //
    //sample_get_char();
    //
    //ret = ss_mpi_vpss_get_chn_frame(1, 0, &frame_info, 1000); /* milli_sec: 1000us */
    //if (ret != TD_SUCCESS) {
    //    sample_print("ss_mpi_vpss_get_chn_frame failed! 0x%x\n",ret);
    //    goto exit;
    //}

    ////=============
    /* ACL init */
    static npu_acl_model_t npu_acl_model = {0};
    const char *acl_config_path = "";
    td_u32 npu_dev_id = 0;
    aclrtRunMode run_mode;
    const char *om_model_path = "/sharefs/best.om";
    //const char *om_model_path = "./yolov3.om";
    td_char path[PATH_MAX] = { 0 };
    td_u32 output_size;
    td_u32 buffer_size;
    td_u32 input_size;

    td_void *input_buffer = TD_NULL;
    td_void *input_buffer2 = TD_NULL;
    td_void *output_buffer = TD_NULL;

    aclDataBuffer *input_data = TD_NULL;
    aclDataBuffer *input_data2 = TD_NULL;
    aclDataBuffer *output_data = TD_NULL;
    aclDataBuffer *output_buffer_0 = TD_NULL;
    aclDataBuffer *output_buffer_1 = TD_NULL;
    td_void *data = TD_NULL;
    td_u32 len;
    td_float top[NPU_SHOW_TOP_NUM] = { 0.0 };


    ret = aclInit(acl_config_path);
    if (ret != ACL_ERROR_NONE) {
        sample_print("acl init fail.\n");
        return TD_FAILURE;
    }
    sample_print("acl init success.\n");

    /* open device */
    ret = aclrtSetDevice(npu_dev_id);
    if (ret != ACL_ERROR_NONE) {
        sample_print("acl open device %d fail.\n", npu_dev_id);
        return TD_FAILURE;
    }
    sample_print("open device %d success.\n", npu_dev_id);

    /* get run mode */
    ret = aclrtGetRunMode(&run_mode);
    if ((ret != ACL_ERROR_NONE) || (run_mode != ACL_DEVICE)) {
        sample_print("acl get run mode fail.\n");
        return TD_FAILURE;
    }
    sample_print("get run mode success\n");

    if (realpath(om_model_path, path) == TD_NULL) {
        sample_print("invalid file!.\n");
        return TD_NULL;
    }

    ret = aclmdlQuerySize(path, &npu_acl_model.model_mem_size, &npu_acl_model.model_weight_size);
    if (ret != ACL_ERROR_NONE) {
        sample_print("query model failed, model file is %s\n", path);
        return TD_FAILURE;
    }
    printf("mem_size=%lu weight_size=%lu\n",npu_acl_model.model_mem_size,npu_acl_model.model_weight_size);

    ret = aclrtMalloc(&npu_acl_model.model_mem_ptr, npu_acl_model.model_mem_size, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        sample_print("malloc buffer for mem failed, require size is %lu\n", npu_acl_model.model_mem_size);
        return TD_FAILURE;
    }

    ret = aclrtMalloc(&npu_acl_model.model_weight_ptr, npu_acl_model.model_weight_size, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        sample_print("malloc buffer for weight fail, require size is %lu\n", npu_acl_model.model_weight_size);
        return TD_FAILURE;
    }

    ret = aclmdlLoadFromFileWithMem(path, &npu_acl_model.model_id, npu_acl_model.model_mem_ptr, npu_acl_model.model_mem_size,
        npu_acl_model.model_weight_ptr, npu_acl_model.model_weight_size);
    if (ret != ACL_ERROR_NONE) {
        sample_print("load model from file failed, model file is %s\n", path);
        return TD_FAILURE;
    }

    npu_acl_model.is_load_flag = TD_TRUE;
    printf("Load model success!\n");

    npu_acl_model.model_desc = aclmdlCreateDesc();
    if (npu_acl_model.model_desc == TD_NULL) {
        sample_print("create model description failed\n");
        return TD_FAILURE;
    }

    ret = aclmdlGetDesc(npu_acl_model.model_desc, npu_acl_model.model_id);
    if (ret != ACL_ERROR_NONE) {
        sample_print("get model description failed\n");
        return TD_FAILURE;
    }
    sample_print("create model description success\n");

    npu_acl_model.input_dataset = aclmdlCreateDataset();
    if (npu_acl_model.input_dataset == TD_NULL) {
        sample_print("can't create dataset, create input failed\n");
        return TD_FAILURE;
    }

    npu_acl_model.output_dataset = aclmdlCreateDataset();
    if (npu_acl_model.output_dataset == TD_NULL) {
        sample_print("can't create dataset, create output failed\n");
        return TD_FAILURE;
    }

    input_size = aclmdlGetNumInputs(npu_acl_model.model_desc);//获取模型的输入个数
    //printf("%s:%d, input_size=%lu\n",__func__,__LINE__,input_size);

    output_size = aclmdlGetNumOutputs(npu_acl_model.model_desc);//获取模型的输出个数
    //printf("%s:%d, output_size = %lu\n",__func__,__LINE__,output_size);

    for (i = 0; i < output_size; ++i) {
        buffer_size = aclmdlGetOutputSizeByIndex(npu_acl_model.model_desc, i);
        //printf("output %d size = %d\n",i,buffer_size);
        //printf("%s:%d,index=%d,output_size=%lu\n",__func__,__LINE__,i,buffer_size);

        ret = aclrtMalloc(&output_buffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            sample_print("can't malloc buffer, size is %u, create output failed\n", buffer_size);
            return TD_FAILURE;
        }
        //printf("%s:%d\n",__func__,__LINE__);

        output_data = aclCreateDataBuffer(output_buffer, buffer_size);
        if (output_data == TD_NULL) {
            sample_print("can't create data buffer, create output failed\n");
            aclrtFree(output_buffer);
            return TD_FAILURE;
        }
        //printf("%s:%d\n",__func__,__LINE__);

        ret = aclmdlAddDatasetBuffer(npu_acl_model.output_dataset, output_data);
        if (ret != ACL_ERROR_NONE) {
            sample_print("can't add data buffer, create output failed\n");
            aclrtFree(output_buffer);
            aclDestroyDataBuffer(output_data);
            return TD_FAILURE;
        }
        //printf("%s:%d\n",__func__,__LINE__);
    }

    input_size = aclmdlGetInputSizeByIndex(npu_acl_model.model_desc, 0);
    //printf("%s:%d,index=%d,input_size=%lu\n",__func__,__LINE__,0,input_size);

    ret = aclrtMalloc(&input_buffer, input_size, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        sample_print("malloc device buffer fail. size is %zu, errorCode is %d.\n", input_size, ret);
        return TD_FAILURE;
    }

    //printf("%s:%d\n",__func__,__LINE__);

    #if 0
    //fill_input_data
    char *addr0 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[0], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
    char *addr1 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[1], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);

    memcpy_s((char *)input_buffer,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT,addr0,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
    memcpy_s((char *)(input_buffer+SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT),SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2,addr1,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);

    fp = fopen ("snap.yuv", "w+");
    fwrite(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT, 1, fp);
    fwrite(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2, 1, fp );

    fclose(fp);

    ss_mpi_sys_munmap(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
    ss_mpi_sys_munmap(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
    #endif

    #if 1
    unsigned char *temp_in_buf = TD_NULL;//bgr
    //unsigned short *temp_out_buf = TD_NULL;
    //temp_in_buf = malloc(SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3*2);
    //temp_out_buf = malloc(input_size);
    temp_in_buf = malloc(SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3);

    memset((char *)temp_in_buf,0x0,input_size);

    fp = fopen ("/sharefs/RGB_test.bgr", "rb");
    if (fp == TD_NULL) {
        printf("Open file failed:RGB_test.bgr!\n");
    }

    //fread(temp_in_buf, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3*2, 1,fp);
    fread(temp_in_buf, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3, 1,fp);

    memcpy_s((char *)input_buffer,input_size,temp_in_buf,input_size);

    fclose(fp);

    free(temp_in_buf);
    //free(temp_out_buf);
    //fill_input_data
    #endif

    //printf("0x%02x 0x%02x 0x%02x 0x%02x\n",*(char *)(input_buffer),*(char *)(input_buffer+1),*(char *)(input_buffer+2),*(char *)(input_buffer+3));

    //add input1
    input_data = aclCreateDataBuffer(input_buffer, input_size);
    if (input_data == TD_NULL) {
        sample_print("can't create data buffer, create input failed\n");
        return TD_FAILURE;
    }

    ret = aclmdlAddDatasetBuffer(npu_acl_model.input_dataset, input_data);
    if (ret != ACL_SUCCESS) {
        sample_print("add input dataset buffer failed, ret is %d\n", ret);
        (void)aclDestroyDataBuffer(input_data);
        input_data = TD_NULL;
        return TD_FAILURE;
    }
    /*
    //add input2
    input_data2 = aclCreateDataBuffer(imageInfo, sizeof(imageInfo));
    if (input_data2 == TD_NULL) {
        sample_print("can't create data buffer, create input failed\n");
        return TD_FAILURE;
    }

    ret = aclmdlAddDatasetBuffer(npu_acl_model.input_dataset, input_data2);
    if (ret != ACL_SUCCESS) {
        sample_print("add input dataset buffer failed, ret is %d\n", ret);
        (void)aclDestroyDataBuffer(input_data);
        input_data = TD_NULL;
        return TD_FAILURE;
    }
    */

    //printf("%s:%d\n",__func__,__LINE__);
    ret = aclmdlExecute(npu_acl_model.model_id, npu_acl_model.input_dataset,npu_acl_model.output_dataset);
    sample_print("end aclmdlExecute, modelId is %u\n", npu_acl_model.model_id);
    //printf("%s:%d\n",__func__,__LINE__);

    //output_model_result---(1, box(4)+score(cls)+32, 8400)
    output_buffer_0 = aclmdlGetDatasetBuffer(npu_acl_model.output_dataset, 0);
    float* detectData = (float *)aclGetDataBufferAddr(output_buffer_0);

    // output data shape is nchw, 2 is stand h
    //[1, 1, 37, 8400]
    aclmdlIODims aclDims;
    ret = aclmdlGetOutputDims(npu_acl_model.model_desc, 0, &aclDims);
    int imgChn = aclDims.dims[aclDims.dimCount - 3];
    int imgHeight = aclDims.dims[aclDims.dimCount - 2];
    int imgWidth = aclDims.dims[aclDims.dimCount - 1];
    //printf("output image channel[%d]; width[%d]; height[%d]\n", imgChn, imgWidth, imgHeight);

    aclDataType aclType;
    aclType = aclmdlGetOutputDataType(npu_acl_model.model_desc, 0);
    //printf("aclmdlGetOutputDataType = %d\n", aclType);

    int idx0,idx1,idx2,idx3,idx4;
    float conf = 0;
    int boxId = 0;
    stbox bbox[MAX_OUTPUT_BOX];
    memset((char *)bbox, 0x0, sizeof(bbox));

    for (int k = 0; k < imgWidth; k++) {
        idx0 = k;
        idx1 = idx0 + imgWidth;
        idx2 = idx1 + imgWidth;
        idx3 = idx2 + imgWidth;
        idx4 = idx3 + imgWidth;
        conf = detectData[idx4];
        if(conf > g_Conf_Thresh){
            //printf("index = %d, box = %f,%f,%f,%f,%f\n", boxId, conf);
            bbox[boxId].x = detectData[idx0];
            bbox[boxId].y = detectData[idx1];
            bbox[boxId].w = detectData[idx2];
            bbox[boxId].h = detectData[idx3];
            bbox[boxId].conf = conf;//detectData[idx4];
            bbox[boxId].id = 0;
            bbox[boxId].vaild = 1;
            boxId++;
        }
    }

    //printf("%s:%d, boxnum = %d\n",__func__,__LINE__, boxId);

    //对检测结果按conf进行排序
    sortBboxConf(&bbox, boxId);

    //非极大值抑制
    nmsBbox(&bbox, boxId);

    int boxNum = 0;

    for (int k = 0; k < boxId; k++) {
        if(1 == bbox[k].vaild){
            printf("box%d = %f, %f, %f, %f, %f\n", k, bbox[k].x, bbox[k].y, bbox[k].w, bbox[k].h, bbox[k].conf);
            boxNum++;
        }
    }

    printf("box num = %d\n", boxNum);

    //printf("Release buffer\n");
    ret = ss_mpi_vpss_release_chn_frame(1, 0, &frame_info);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_vpss_release_chn_frame failed with %#x!\n",ret);
    }
    aclDestroyDataBuffer(input_data);
    aclDestroyDataBuffer(output_data);
    aclrtFree(input_buffer);
    aclrtFree(output_buffer);
    aclmdlDestroyDataset(npu_acl_model.output_dataset);
    aclmdlDestroyDataset(npu_acl_model.input_dataset);
    aclmdlDestroyDesc(npu_acl_model.model_desc);
    aclrtFree(npu_acl_model.model_mem_ptr);
    aclrtFree(npu_acl_model.model_weight_ptr);
    ////=============
#if 0
printf("width=%d height=%d pixel_format=%d\n",frame_info.video_frame.width,frame_info.video_frame.height,frame_info.video_frame.pixel_format);
    char *addr0 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[0], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
    char *addr1 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[1], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);



    fp = fopen ("snap.yuv", "w+");
    fwrite(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT, 1, fp);
    fwrite(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2, 1, fp );

    fclose(fp);

    ss_mpi_sys_munmap(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
    ss_mpi_sys_munmap(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);


    ret = ss_mpi_vpss_release_chn_frame(1, 0, &frame_info);
    if (ret != TD_SUCCESS) {
        printf("ss_mpi_vpss_release_chn_frame failed with %#x!\n",ret);
    }

    printf("snap success!\n");
#endif

exit:
    //ss_mpi_snap_disable_pipe(0);
    return ret;
}
//    hi_s32 ret;
//    ot_video_frame_info frame_info;
//    //char *v = NULL;
//    //char *u = NULL;
//    int i,j;
//    FILE   *fp;
//
//    //ret = ss_mpi_snap_enable_pipe(0);
//    //if (ret != HI_SUCCESS) {
//    //    printf("ss_mpi_snap_enable_pipe failed, ret: 0x%x\n", ret);
//    //    return HI_FAILURE;
//    //}
//
//    // sample_get_char();
//    sleep(5);
//
//    ret = ss_mpi_vpss_get_chn_frame(0, 0, &frame_info, 20000); /* milli_sec: 1000us */
//    if (ret != HI_SUCCESS) {
//        sample_print("ss_mpi_vpss_get_chn_frame failed! 0x%x\n",ret);
//        goto exit;
//    }
//    ////=============
//    /* ACL init */
//    static npu_acl_model_t npu_acl_model = {0};
//    const char *acl_config_path = "";
//    hi_u32 npu_dev_id = 0;
//    aclrtRunMode run_mode;
//    const char *om_model_path = "/sharefs/yolov3_framework_caffe_aipp_1_batch_1_input_int8_output_FP32.om.om";
//    hi_char path[PATH_MAX] = { 0 };
//    hi_u32 output_size;
//    hi_u32 buffer_size;
//    hi_u32 input_size;
//
//    hi_void *input_buffer = HI_NULL;
//    hi_void *input_buffer2 = HI_NULL;
//    hi_void *output_buffer = HI_NULL;
//
//    aclDataBuffer *input_data = HI_NULL;
//    aclDataBuffer *input_data2 = HI_NULL;
//    aclDataBuffer *output_data = HI_NULL;
//    aclDataBuffer *output_buffer_0 = HI_NULL;
//    aclDataBuffer *output_buffer_1 = HI_NULL;
//    hi_void *data = HI_NULL;
//    hi_u32 len;
//    hi_float top[NPU_SHOW_TOP_NUM] = { 0.0 };
//
// 
//    ret = aclInit(acl_config_path);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("acl init fail.\n");
//        return HI_FAILURE;
//    }
//    sample_print("acl init success.\n");
//
//    /* open device */
//    ret = aclrtSetDevice(npu_dev_id);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("acl open device %d fail.\n", npu_dev_id);
//        return HI_FAILURE;
//    }
//    sample_print("open device %d success.\n", npu_dev_id);
//
//    /* get run mode */
//    ret = aclrtGetRunMode(&run_mode);
//    if ((ret != ACL_ERROR_NONE) || (run_mode != ACL_DEVICE)) {
//        sample_print("acl get run mode fail.\n");
//        return HI_FAILURE;
//    }
//    sample_print("get run mode success\n");
//
//    if (realpath(om_model_path, path) == HI_NULL) {
//        sample_print("invalid file!.\n");
//        return HI_NULL;
//    }
//
//    ret = aclmdlQuerySize(path, &npu_acl_model.model_mem_size, &npu_acl_model.model_weight_size);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("query model failed, model file is %s\n", path);
//        return HI_FAILURE;
//    }
//    printf("mem_size=%lu weight_size=%lu\n",npu_acl_model.model_mem_size,npu_acl_model.model_weight_size);
//    
//    ret = aclrtMalloc(&npu_acl_model.model_mem_ptr, npu_acl_model.model_mem_size, ACL_MEM_MALLOC_HUGE_FIRST);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("malloc buffer for mem failed, require size is %lu\n", npu_acl_model.model_mem_size);
//        return HI_FAILURE;
//    }
//
//    ret = aclrtMalloc(&npu_acl_model.model_weight_ptr, npu_acl_model.model_weight_size, ACL_MEM_MALLOC_HUGE_FIRST);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("malloc buffer for weight fail, require size is %lu\n", npu_acl_model.model_weight_size);
//        return HI_FAILURE;
//    }
//
//    ret = aclmdlLoadFromFileWithMem(path, &npu_acl_model.model_id, npu_acl_model.model_mem_ptr, npu_acl_model.model_mem_size,
//        npu_acl_model.model_weight_ptr, npu_acl_model.model_weight_size);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("load model from file failed, model file is %s\n", path);
//        return HI_FAILURE;
//    }
//
//    npu_acl_model.is_load_flag = HI_TRUE;
//    printf("Load model success!\n");
//
//    npu_acl_model.model_desc = aclmdlCreateDesc();
//    if (npu_acl_model.model_desc == HI_NULL) {
//        sample_print("create model description failed\n");
//        return HI_FAILURE;
//    }
//
//    ret = aclmdlGetDesc(npu_acl_model.model_desc, npu_acl_model.model_id);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("get model description failed\n");
//        return HI_FAILURE;
//    }
//    sample_print("create model description success\n");
//
//    npu_acl_model.input_dataset = aclmdlCreateDataset();
//    if (npu_acl_model.input_dataset == HI_NULL) {
//        sample_print("can't create dataset, create input failed\n");
//        return HI_FAILURE;
//    }
//
//    npu_acl_model.output_dataset = aclmdlCreateDataset();
//    if (npu_acl_model.output_dataset == HI_NULL) {
//        sample_print("can't create dataset, create output failed\n");
//        return HI_FAILURE;
//    }
//
//    output_size = aclmdlGetNumOutputs(npu_acl_model.model_desc);
//    for (i = 0; i < output_size; ++i) {
//        buffer_size = aclmdlGetOutputSizeByIndex(npu_acl_model.model_desc, i);
//        printf("output %d size = %d\n",i,buffer_size);
//
//        ret = aclrtMalloc(&output_buffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
//        if (ret != ACL_ERROR_NONE) {
//            sample_print("can't malloc buffer, size is %u, create output failed\n", buffer_size);
//            return HI_FAILURE;
//        }
//printf("%s:%d\n",__func__,__LINE__);
//        output_data = aclCreateDataBuffer(output_buffer, buffer_size);
//        if (output_data == HI_NULL) {
//            sample_print("can't create data buffer, create output failed\n");
//            aclrtFree(output_buffer);
//            return HI_FAILURE;
//        }
//printf("%s:%d\n",__func__,__LINE__);
//        ret = aclmdlAddDatasetBuffer(npu_acl_model.output_dataset, output_data);
//        if (ret != ACL_ERROR_NONE) {
//            sample_print("can't add data buffer, create output failed\n");
//            aclrtFree(output_buffer);
//            aclDestroyDataBuffer(output_data);
//            return HI_FAILURE;
//        }
//    }
//printf("%s:%d\n",__func__,__LINE__);
//    input_size = aclmdlGetInputSizeByIndex(npu_acl_model.model_desc, 0);
//printf("input_size=%lu\n",input_size);
//printf("%s:%d\n",__func__,__LINE__);
//    ret = aclrtMalloc(&input_buffer, input_size, ACL_MEM_MALLOC_NORMAL_ONLY);
//    if (ret != ACL_ERROR_NONE) {
//        sample_print("malloc device buffer fail. size is %zu, errorCode is %d.\n", input_size, ret);
//        return HI_FAILURE;
//    }
//    //add input2
//    float imageInfo[4] = {(float)416, (float)416, (float)416, (float)416};
//
//printf("%s:%d\n",__func__,__LINE__);
//    //fill_input_data
//    #if 1
////    char *addr0 = hi_mpi_sys_mmap(frame_info.video_frame.phys_addr[0], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
////    printf("123123\n");
////    char *addr1 = hi_mpi_sys_mmap(frame_info.video_frame.phys_addr[1], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
////    printf("112233\n");
////
////
////
////    memcpy_s((char *)input_buffer,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT,addr0,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
////    printf("11223344\n");
////    memcpy_s((char *)(input_buffer+SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT),SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2,addr1,SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
////    printf("12341234\n");
////    // fp = fopen ("snap.yuv", "w+");
////    // fwrite(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT, 1, fp);
////    // fwrite(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2, 1, fp );
////
////    // fclose(fp);
////
////    ss_mpi_sys_munmap(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
////    printf("addr0addr0\n");
////    ss_mpi_sys_munmap(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
////    printf("addr1addr1\n");
//    #endif
////    #if 0
//    unsigned char *temp_in_buf = HI_NULL;//bgr
//    //unsigned short *temp_out_buf = HI_NULL;
//    temp_in_buf = malloc(SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3*2);
//    //temp_out_buf = malloc(input_size);
//
//    memset((char *)temp_in_buf,0x0,input_size);
//
//    fp = fopen ("/sharefs/car_test.yuv", "rb");
//    if (fp == HI_NULL) {
//        printf("Open file failed:%s!\n", fp);
//    }
//
//    fread(temp_in_buf, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT*3*2, 1,fp);
//    printf("fread!!\n");
//    memcpy_s((char *)input_buffer,input_size,temp_in_buf,input_size);
//
//    fclose(fp);
//
//    free(temp_in_buf);
//    //free(temp_out_buf);
//    //fill_input_data
////    #endif
//
//    printf("0x%02x 0x%02x 0x%02x 0x%02x\n",*(char *)(input_buffer),*(char *)(input_buffer+1),*(char *)(input_buffer+2),*(char *)(input_buffer+3));
//
//    input_data = aclCreateDataBuffer(input_buffer, input_size);
//    if (input_data == HI_NULL) {
//        sample_print("can't create data buffer, create input failed\n");
//        return HI_FAILURE;
//    }
//
//    ret = aclmdlAddDatasetBuffer(npu_acl_model.input_dataset, input_data);
//    if (ret != ACL_SUCCESS) {
//        sample_print("add input dataset buffer failed, ret is %d\n", ret);
//        (void)aclDestroyDataBuffer(input_data);
//        input_data = HI_NULL;
//        return HI_FAILURE;
//    }
//    //add input2
//    input_data2 = aclCreateDataBuffer(imageInfo, sizeof(imageInfo));
//    if (input_data2 == HI_NULL) {
//        sample_print("can't create data buffer, create input failed\n");
//        return HI_FAILURE;
//    }
//
//    ret = aclmdlAddDatasetBuffer(npu_acl_model.input_dataset, input_data2);
//    if (ret != ACL_SUCCESS) {
//        sample_print("add input dataset buffer failed, ret is %d\n", ret);
//        (void)aclDestroyDataBuffer(input_data);
//        input_data = HI_NULL;
//        return HI_FAILURE;
//    }
//    //add input2
//printf("%s:%d\n",__func__,__LINE__);
//    ret = aclmdlExecute(npu_acl_model.model_id, npu_acl_model.input_dataset,npu_acl_model.output_dataset);
//    sample_print("end aclmdlExecute, modelId is %u\n", npu_acl_model.model_id);
//printf("%s:%d\n",__func__,__LINE__);
//    //output_model_result
//
//    output_buffer_0 = aclmdlGetDatasetBuffer(npu_acl_model.output_dataset, 0);
//    float* detechiata = (float *)aclGetDataBufferAddr(output_buffer_0);
//
//    output_buffer_1 = aclmdlGetDatasetBuffer(npu_acl_model.output_dataset, 1);
//    uint32_t* boxNum = (uint32_t*)aclGetDataBufferAddr(output_buffer_1);
//
//printf("%s:%d\n",__func__,__LINE__);
//    if (boxNum == HI_NULL) {
//        printf("%s:%d\n",__func__,__LINE__);
//        return HI_FAILURE;
//    }
//    hi_u32 totalBox = boxNum[0];
//printf("%s:%d totalBox=%d \n",__func__,__LINE__,totalBox);
//    float widthScale = 1;
//    float heightScale = 1;
//    for (i = 0; i < totalBox; i++) {
//        printf("%s:%d\n",__func__,__LINE__);
//        //get the confidence of the detected object. Anything less than 0.8 is considered invalid
//        hi_u32 score = (hi_u32)(detechiata[totalBox * SCORE + i] * 100);
//    printf("%s:%d score=%d \n",__func__,__LINE__,score);
//        if (score < 80) continue;
//        //get the frame coordinates and converts them to the coordinates on the original frame
//        printf("[%f %f %f %f]\n",\
//            detechiata[totalBox * TOPLEFTX + i],\
//            detechiata[totalBox * TOPLEFTY + i],\
//            detechiata[totalBox * BOTTOMRIGHTX + i],\
//            detechiata[totalBox * BOTTOMRIGHTY + i]);
//printf("%s:%d\n",__func__,__LINE__);
//        //Construct a string that marks the object: object name + confidence
//        hi_u32 objIndex = (hi_u32)detechiata[totalBox * LABEL + i];
//        printf("%s \n",yolov3Label[objIndex]);
//printf("%s:%d\n",__func__,__LINE__);
//    }    
//
//#if 0
//    for (i = 0; i < aclmdlGehiatasetNumBuffers(npu_acl_model.output_dataset); ++i) {
//        data_buffer = aclmdlGehiatasetBuffer(npu_acl_model.output_dataset, i);
//        if (data_buffer == HI_NULL) {
//            sample_print("get data buffer null.\n");
//            continue;
//        }
//
//        data = aclGehiataBufferAddr(data_buffer);
//        len = aclGehiataBufferSizeV2(data_buffer);
//        if (data == HI_NULL || len == 0) {
//            sample_print("get data null.\n");
//            continue;
//        }
//        printf("len=%d\n",len);
//        for (j = 0; j < len/4; j++) {
//            printf("%f ",*((float *)data+j*4));
//        }
//        printf("\n");
//
//        npu_sort_output_result(data, (len / sizeof(hi_float)), top, NPU_SHOW_TOP_NUM);
//
//        for (j = 0; j < NPU_SHOW_TOP_NUM; j++) {
//            sample_print("top %d: value[%lf]\n", j, top[j]);
//        }
//
//    }
//#endif
//    printf("Release buffer\n");
//    ret = ss_mpi_vpss_release_chn_frame(0, 0, &frame_info);
//    if (ret != HI_SUCCESS) {
//        printf("ss_mpi_vpss_release_chn_frame failed with %#x!\n",ret);
//    }
//    aclDestroyDataBuffer(input_data);
//    aclDestroyDataBuffer(output_data);
//    aclrtFree(input_buffer);
//    aclrtFree(output_buffer);     
//    aclmdlDestroyDataset(npu_acl_model.output_dataset);
//    aclmdlDestroyDataset(npu_acl_model.input_dataset);
//    aclmdlDestroyDesc(npu_acl_model.model_desc);
//    aclrtFree(npu_acl_model.model_mem_ptr);
//    aclrtFree(npu_acl_model.model_weight_ptr);
//    ////=============
//#if 0   
//printf("width=%d height=%d pixel_format=%d\n",frame_info.video_frame.width,frame_info.video_frame.height,frame_info.video_frame.pixel_format);
//    char *addr0 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[0], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
//    char *addr1 = ss_mpi_sys_mmap(frame_info.video_frame.phys_addr[1], SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
//
//
//
//    fp = fopen ("snap.yuv", "w+");
//    fwrite(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT, 1, fp);
//    fwrite(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2, 1, fp );
//
//    fclose(fp);
//
//    ss_mpi_sys_munmap(addr0, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT);
//    ss_mpi_sys_munmap(addr1, SMALL_PIC_WIDTH*SMALL_PIC_HEIGHT/2);
//
//
//    ret = ss_mpi_vpss_release_chn_frame(1, 0, &frame_info);
//    if (ret != HI_SUCCESS) {
//        printf("ss_mpi_vpss_release_chn_frame failed with %#x!\n",ret);
//    }
//
//    printf("snap success!\n");
//#endif
//
//exit:
//    //ss_mpi_snap_disable_pipe(0);
//    return ret;
//}

/******************************************************************************
* function :  H.265e@1080P@30fps + h264e@D1@30fps
******************************************************************************/
static hi_s32 sample_venc_normal(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    hi_size enc_size[CHN_NUM_MAX];
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        sample_print("Init SYS err for %#x!\n", ret);
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        sample_print("Init VI err for %#x!\n", ret);
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        sample_print("Init VPSS err for %#x!\n", ret);
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        sample_print("VI Bind VPSS err for %#x!\n", ret);
        goto EXIT_VPSS_STOP;
    }

    if ((ret = sample_venc_normal_start_encode(vpss_grp, &venc_vpss_chn)) != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

	printf("1112223333\n");
    if ((ret = sample_acl()) != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }
	printf("1112223333-end\n");


    /******************************************
     exit process
    ******************************************/
    sample_venc_exit_process();
    sample_venc_unbind_vpss_stop(vpss_grp, &venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_s32 sample_venc_qpmap_start_encode(hi_size *enc_size, hi_s32 chn_num_max,
    ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_mode gop_mode;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param chn_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param *h265_chn_param = HI_NULL;
    sample_comm_venc_chn_param *h264_chn_param = HI_NULL;
    ot_unused(chn_num_max);

    if (get_gop_mode(&gop_mode) != HI_SUCCESS) {
        return HI_FAILURE;
    }
    if ((ret = sample_comm_venc_get_gop_attr(gop_mode, &gop_attr)) != HI_SUCCESS) {
        sample_print("Venc Get GopAttr for %#x!\n", ret);
        return ret;
    }

    sample_venc_set_video_param(chn_param, gop_attr, CHN_NUM_MAX, HI_TRUE);

    /* encode h.265 */
    h265_chn_param = &(chn_param[0]);
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], h265_chn_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    /* encode h.264 */
    h264_chn_param = &(chn_param[1]);
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], h264_chn_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_H265_STOP;
    }

    ret = sample_comm_venc_qpmap_send_frame(vpss_grp, venc_vpss_chn->vpss_chn,
        venc_vpss_chn->venc_chn, CHN_NUM_MAX, enc_size);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_qpmap_send_frame failed for %#x!\n", ret);
        goto EXIT_VENC_H264_STOP;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
        goto EXIT_VENC_H264_STOP;
    }

    return HI_SUCCESS;

EXIT_VENC_H264_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H265_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

static hi_void sample_venc_qpmap_exit_process()
{
    printf("please press twice ENTER to exit this sample\n");
    (hi_void)getchar();

    if (g_sample_venc_exit != HI_TRUE) {
        (hi_void)getchar();
    }

    sample_comm_venc_stop_send_qpmap_frame();
    sample_comm_venc_stop_get_stream(CHN_NUM_MAX);
}

static hi_s32 sample_venc_qpmap(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    hi_size enc_size[CHN_NUM_MAX];
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        sample_print("Init SYS err for %#x!\n", ret);
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        sample_print("Init VI err for %#x!\n", ret);
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        sample_print("Init VPSS err for %#x!\n", ret);
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        sample_print("VI Bind VPSS err for %#x!\n", ret);
        goto EXIT_VPSS_STOP;
    }

    /******************************************
    start stream venc
    ******************************************/
    if ((ret = sample_venc_qpmap_start_encode(enc_size, CHN_NUM_MAX, vpss_grp, &venc_vpss_chn)) != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_qpmap_exit_process();
    sample_venc_stop(&venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_s32 sample_venc_vpss_bind_venc(ot_vpss_grp vpss_grp, ot_vpss_chn vpss_chn, ot_venc_chn venc_chn)
{
    hi_s32 ret;

    ret = sample_comm_vpss_bind_venc(vpss_grp, vpss_chn, venc_chn);
    if (ret != HI_SUCCESS) {
        sample_print("call sample_comm_vpss_bind_venc vpss grp %d, vpss chn %d, venc chn %d, ret =  %#x!\n",
            vpss_grp, vpss_chn, venc_chn, ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_void sample_venc_intra_refresh_param_init(ot_venc_intra_refresh_mode intra_refresh_mode,
    ot_venc_intra_refresh *intra_refresh)
{
    intra_refresh->enable = HI_TRUE;
    intra_refresh->mode = intra_refresh_mode;
    if (intra_refresh_mode == OT_VENC_INTRA_REFRESH_ROW) {
        intra_refresh->refresh_num = 5; /* 5: refresh num */
    } else {
        intra_refresh->refresh_num = 6; /* 6: refresh num */
    }
    intra_refresh->request_i_qp = 30; /* 30: request num */
}

static hi_s32 sample_venc_set_intra_refresh(ot_venc_chn venc_chn, ot_venc_intra_refresh_mode intra_refresh_mode)
{
    hi_s32 ret;
    ot_venc_intra_refresh intra_refresh = {0};

    if ((ret = ss_mpi_venc_get_intra_refresh(venc_chn, &intra_refresh)) != HI_SUCCESS) {
        sample_print("Get Intra Refresh failed for %#x!\n", ret);
        return ret;
    }

    sample_venc_intra_refresh_param_init(intra_refresh_mode, &intra_refresh);

    if ((ret = ss_mpi_venc_set_intra_refresh(venc_chn, &intra_refresh)) != HI_SUCCESS) {
        sample_print("Set Intra Refresh failed for %#x!\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_s32 sample_comm_venc_get_gop_default_attr(ot_venc_gop_attr *gop_attr)
{
    hi_s32 ret;
    ot_venc_gop_mode gop_mode;

    if (get_gop_mode(&gop_mode) != HI_SUCCESS) {
        return HI_FAILURE;
    }
    ret = sample_comm_venc_get_gop_attr(gop_mode, gop_attr);
    if (ret != HI_SUCCESS) {
        sample_print("Venc get gop default attr for mode %d failed return %#x!\n", gop_mode, ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_s32 sample_venc_intra_refresh_set_video_param(ot_venc_intra_refresh_mode *intra_refresh_mode,
    sample_comm_venc_chn_param *chn_param, hi_s32 len)
{
    hi_s32 ret;
    ot_venc_gop_attr gop_attr;

    if ((ret = sample_comm_venc_get_gop_default_attr(&gop_attr)) != HI_SUCCESS) {
        return ret;
    }

    if (get_intra_refresh_mode(intra_refresh_mode) != HI_SUCCESS) {
        return HI_FAILURE;
    }

    if (len > CHN_NUM_MAX) {
        sample_print("the num of venc_create_param is beyond CHN_NUM_MAX !\n");
        return HI_FAILURE;
    }
    sample_venc_set_video_param(chn_param, gop_attr, CHN_NUM_MAX, HI_FALSE);

    return HI_SUCCESS;
}

static hi_s32 sample_venc_intra_refresh_start_encode(ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_intra_refresh_mode intra_refresh_mode;
    sample_comm_venc_chn_param chn_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param *h265_chn_param = HI_NULL;
    sample_comm_venc_chn_param *h264_chn_param = HI_NULL;

    ret = sample_venc_intra_refresh_set_video_param(&intra_refresh_mode, chn_param, CHN_NUM_MAX);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /* encode h.265 */
    h265_chn_param = &(chn_param[0]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], h265_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H265_STOP;
    }

    /* set intra refresh mode for chn 0 */
    if ((ret = sample_venc_set_intra_refresh(venc_vpss_chn->venc_chn[0], intra_refresh_mode)) != HI_SUCCESS) {
        goto EXIT_VENC_H265_UnBind;
    }

    /* encode h.264 */
    h264_chn_param = &(chn_param[1]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], h264_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_H265_UnBind;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H264_STOP;
    }

    /* set intra refresh mode for chn 1 */
    if ((ret = sample_venc_set_intra_refresh(venc_vpss_chn->venc_chn[1], intra_refresh_mode)) != HI_SUCCESS) {
        goto EXIT_VENC_H264_UnBind;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
    }

    return ret;

EXIT_VENC_H264_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H264_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H265_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
EXIT_VENC_H265_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

/******************************************************************************
* function : intra_refresh:H.265e@1080P@30fps(row) + h264e@1080P@30fps(column).
******************************************************************************/
static hi_s32 sample_venc_intra_refresh(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    hi_size enc_size[CHN_NUM_MAX];
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        sample_print("Init SYS err for %#x!\n", ret);
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        sample_print("Init VI err for %#x!\n", ret);
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        sample_print("Init VPSS err for %#x!\n", ret);
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        sample_print("VI Bind VPSS err for %#x!\n", ret);
        goto EXIT_VPSS_STOP;
    }

    /******************************************
    start stream venc
    ******************************************/
    ret = sample_venc_intra_refresh_start_encode(vpss_grp, &venc_vpss_chn);
    if (ret != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_exit_process();
    sample_venc_unbind_vpss_stop(vpss_grp, &venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_void sample_venc_roi_attr_init(ot_venc_roi_attr *roi_attr)
{
    roi_attr->is_abs_qp = HI_TRUE;
    roi_attr->enable = HI_TRUE;
    roi_attr->qp = 30; /* 30: qp value */
    roi_attr->idx = 0;
    roi_attr->rect.x = 64; /* 64: rect.x value */
    roi_attr->rect.y = 64; /* 64: rect.y value */
    roi_attr->rect.height = 256; /* 256: rect.height value */
    roi_attr->rect.width = 256; /* 256: rect.width value */
}

static hi_void sample_venc_roi_bg_frame_rate_init(ot_venc_roi_bg_frame_rate *roi_bg_frame_rate)
{
    roi_bg_frame_rate->src_frame_rate = 30; /* 30: src_frame_rate value */
    roi_bg_frame_rate->dst_frame_rate = 15; /* 15: dst_frame_rate value */
}

static hi_s32 sample_venc_set_roi_attr(ot_venc_chn venc_chn)
{
    hi_s32 ret;
    ot_venc_roi_attr roi_attr;
    ot_venc_roi_bg_frame_rate roi_bg_frame_rate;

    if ((ret = ss_mpi_venc_get_roi_attr(venc_chn, 0, &roi_attr)) != HI_SUCCESS) {   /* 0: roi index */
        sample_print("chn %d Get Roi Attr failed for %#x!\n", venc_chn, ret);
        return ret;
    }

    sample_venc_roi_attr_init(&roi_attr);

    if ((ret = ss_mpi_venc_set_roi_attr(venc_chn, &roi_attr)) != HI_SUCCESS) {
        sample_print("chn %d Set Roi Attr failed for %#x!\n", venc_chn, ret);
        return ret;
    }

    if ((ret = ss_mpi_venc_get_roi_bg_frame_rate(venc_chn, &roi_bg_frame_rate)) != HI_SUCCESS) {
        sample_print("chn %d Get Roi BgFrameRate failed for %#x!\n", venc_chn, ret);
        return ret;
    }

    sample_venc_roi_bg_frame_rate_init(&roi_bg_frame_rate);

    if ((ret = ss_mpi_venc_set_roi_bg_frame_rate(venc_chn, &roi_bg_frame_rate)) != HI_SUCCESS) {
        sample_print("chn %d Set Roi BgFrameRate failed for %#x!\n", venc_chn, ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_s32 sample_venc_roi_bg_start_encode(ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param chn_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param *h265_chn_param = HI_NULL;
    sample_comm_venc_chn_param *h264_chn_param = HI_NULL;

    if ((ret = sample_comm_venc_get_gop_default_attr(&gop_attr)) != HI_SUCCESS) {
        return ret;
    }

    sample_venc_set_video_param(chn_param, gop_attr, CHN_NUM_MAX, HI_FALSE);

    /* encode h.265 */
    h265_chn_param = &(chn_param[0]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], h265_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H265_STOP;
    }

    /* set roi bg frame rate for chn 0 */
    if ((ret = sample_venc_set_roi_attr(venc_vpss_chn->venc_chn[0])) != HI_SUCCESS) {
        goto EXIT_VENC_H265_UnBind;
    }

    /* encode h.264 */
    h264_chn_param = &(chn_param[1]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], h264_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_H265_UnBind;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H264_STOP;
    }

    /* set roi bg frame rate for chn 1 */
    if ((ret = sample_venc_set_roi_attr(venc_vpss_chn->venc_chn[1])) != HI_SUCCESS) {
        goto EXIT_VENC_H264_UnBind;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
        goto EXIT_VENC_H264_UnBind;
    }

    return ret;

EXIT_VENC_H264_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H264_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H265_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
EXIT_VENC_H265_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

/******************************************************************************
* function : roi_bg_frame_rate:H.265e@1080P@30fps + H.264@1080P@30fps.
******************************************************************************/
static hi_s32 sample_venc_roi_bg(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    hi_size enc_size[CHN_NUM_MAX];
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        sample_print("Init SYS err for %#x!\n", ret);
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        sample_print("Init VI err for %#x!\n", ret);
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        sample_print("Init VPSS err for %#x!\n", ret);
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        sample_print("VI Bind VPSS err for %#x!\n", ret);
        goto EXIT_VPSS_STOP;
    }

    /******************************************
    start stream venc
    ******************************************/
    if ((ret = sample_venc_roi_bg_start_encode(vpss_grp, &venc_vpss_chn)) != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_exit_process();
    sample_venc_unbind_vpss_stop(vpss_grp, &venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_s32 sample_venc_set_debreath_effect(ot_venc_chn venc_chn, hi_bool enable)
{
    hi_s32 ret;
    ot_venc_debreath_effect debreath_effect;

    if ((ret = ss_mpi_venc_get_debreath_effect(venc_chn, &debreath_effect)) != HI_SUCCESS) {
        sample_print("Get debreath_effect failed for %#x!\n", ret);
        return ret;
    }

    if (enable) {
        debreath_effect.enable = HI_TRUE;
        debreath_effect.strength0 = 3; /* 3 : param */
        debreath_effect.strength1 = 20; /* 20 : param */
    } else {
        debreath_effect.enable = HI_FALSE;
    }

    if ((ret = ss_mpi_venc_set_debreath_effect(venc_chn, &debreath_effect)) != HI_SUCCESS) {
        sample_print("Set debreath_effect failed for %#x!\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

static hi_s32 sample_venc_debreath_effect_start_encode(ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param chn_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param *h265_chn_param = HI_NULL;
    sample_comm_venc_chn_param *h264_chn_param = HI_NULL;

    if ((ret = sample_comm_venc_get_gop_default_attr(&gop_attr)) != HI_SUCCESS) {
        return ret;
    }

    sample_venc_set_video_param(chn_param, gop_attr, CHN_NUM_MAX, HI_FALSE);

    /* encode h.265 */
    h265_chn_param = &(chn_param[0]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], h265_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H265_STOP;
    }

    /* set intra refresh mode for chn 0 */
    if ((ret = sample_venc_set_debreath_effect(venc_vpss_chn->venc_chn[0], HI_TRUE)) != HI_SUCCESS) {
        goto EXIT_VENC_H265_UnBind;
    }

    /* encode h.264 */
    h264_chn_param = &(chn_param[1]);
    if ((ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], h264_chn_param)) != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_H265_UnBind;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_H264_STOP;
    }

    /* set intra refresh mode for chn 1 */
    if ((ret = sample_venc_set_debreath_effect(venc_vpss_chn->venc_chn[1], HI_FALSE)) != HI_SUCCESS) {
        goto EXIT_VENC_H264_UnBind;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
        goto EXIT_VENC_H264_UnBind;
    }

    return HI_SUCCESS;

EXIT_VENC_H264_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H264_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_H265_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
EXIT_VENC_H265_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

static hi_s32 sample_venc_debreath_effect(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    hi_size enc_size[CHN_NUM_MAX];
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);
    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        sample_print("Init SYS err for %#x!\n", ret);
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        sample_print("Init VI err for %#x!\n", ret);
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        sample_print("Init VPSS err for %#x!\n", ret);
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        sample_print("VI Bind VPSS err for %#x!\n", ret);
        goto EXIT_VPSS_STOP;
    }

    /******************************************
    start stream venc
    ******************************************/
    ret = sample_venc_debreath_effect_start_encode(vpss_grp, &venc_vpss_chn);
    if (ret != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_exit_process();
    sample_venc_unbind_vpss_stop(vpss_grp, &venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_void sample_venc_set_jpeg_param(sample_comm_venc_chn_param *venc_create_param,
    ot_venc_gop_attr gop_attr, hi_s32 chn_num_max)
{
    hi_u32 profile[CHN_NUM_MAX] = {0, 0};
    hi_pic_size pic_size[CHN_NUM_MAX] = {BIG_STREAM_SIZE, SMALL_STREAM_SIZE};
    hi_bool share_buf_en = HI_FALSE;
    ot_payload_type payload[CHN_NUM_MAX] = {OT_PT_MJPEG, OT_PT_MJPEG};
    sample_rc rc_mode = 0;

    if (get_rc_mode(payload[0], &rc_mode) != HI_SUCCESS) {
        return;
    }
    if (chn_num_max < 2) { /* 2: venc_create_param array len */
        sample_print("chn_num_max  %d not enough! should > 2\n", chn_num_max);
        return;
    }

    venc_create_param[0].gop_attr                  = gop_attr;
    venc_create_param[0].type                      = payload[0];
    venc_create_param[0].size                      = pic_size[0];
    venc_create_param[0].rc_mode                   = rc_mode;
    venc_create_param[0].profile                   = profile[0];
    venc_create_param[0].is_rcn_ref_share_buf      = share_buf_en;

    venc_create_param[1].gop_attr                  = gop_attr;
    venc_create_param[1].type                      = payload[1];
    venc_create_param[1].size                      = pic_size[1];
    venc_create_param[1].rc_mode                   = rc_mode;
    venc_create_param[1].profile                   = profile[1];
    venc_create_param[1].is_rcn_ref_share_buf      = share_buf_en;
}

static hi_void sample_venc_mjpeg_roi_param_init(ot_venc_jpeg_roi_attr *roi_param)
{
    roi_param->idx = 0;
    roi_param->enable = HI_TRUE;
    roi_param->level = 0;
    roi_param->rect.x = 0;
    roi_param->rect.y = 0;
    roi_param->rect.width = 1280; /* 1280: rect.width value */
    roi_param->rect.height = 720; /* 720: rect.height value */
}

static hi_s32 sample_venc_set_mjpeg_roi(ot_venc_chn venc_chn)
{
    hi_s32 ret;
    const hi_u32 idx = 0;
    ot_venc_jpeg_roi_attr roi_param;

    if ((ret = ss_mpi_venc_get_jpeg_roi_attr(venc_chn, idx, &roi_param)) != HI_SUCCESS) {
        sample_print("Get roi_param failed for %#x!\n", ret);
        return ret;
    }

    sample_venc_mjpeg_roi_param_init(&roi_param);

    if ((ret = ss_mpi_venc_set_jpeg_roi_attr(venc_chn, &roi_param)) != HI_SUCCESS) {
        sample_print("Set roi_param failed for %#x!\n", ret);
    }

    return ret;
}

static hi_s32 sample_venc_mjpeg_roi_set_start_encode(ot_vpss_grp vpss_grp, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param venc_create_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param venc_chn0_param, venc_chn1_param;

    if ((ret = sample_comm_venc_get_gop_attr(OT_VENC_GOP_MODE_NORMAL_P, &gop_attr)) != HI_SUCCESS) {
        return ret;
    }

    sample_venc_set_jpeg_param(venc_create_param, gop_attr, CHN_NUM_MAX);

    venc_chn0_param = venc_create_param[0];
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], &venc_chn0_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_CHN0_STOP;
    }

    /* set intra refresh mode for chn 0 */
    if ((ret = sample_venc_set_mjpeg_roi(venc_vpss_chn->venc_chn[0])) != HI_SUCCESS) {
        goto EXIT_VENC_CHN_0_UnBind;
    }

    venc_chn1_param = venc_create_param[1];
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], &venc_chn1_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_CHN_0_UnBind;
    }

    ret = sample_venc_vpss_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
    if (ret != HI_SUCCESS) {
        goto EXIT_VENC_CHN_1_STOP;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        goto EXIT_VENC_CHN_1_UnBind;
    }

    return HI_SUCCESS;

EXIT_VENC_CHN_1_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[1], venc_vpss_chn->venc_chn[1]);
EXIT_VENC_CHN_1_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_CHN_0_UnBind:
    sample_comm_vpss_un_bind_venc(vpss_grp, venc_vpss_chn->vpss_chn[0], venc_vpss_chn->venc_chn[0]);
EXIT_VENC_CHN0_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

static hi_s32 sample_venc_mjpeg_roi_set(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    hi_size enc_size[CHN_NUM_MAX];
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);

    /******************************************
      step 0: related parameter ready
    ******************************************/
    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        goto EXIT_VPSS_STOP;
    }

    /******************************************
    start stream venc
    ******************************************/
    ret = sample_venc_mjpeg_roi_set_start_encode(vpss_grp, &venc_vpss_chn);
    if (ret != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_exit_process();
    sample_venc_unbind_vpss_stop(vpss_grp, &venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_void sample_venc_mjpeg_roi_attr_init(ot_venc_jpeg_roi_attr *roi_attr, hi_s32 chn_num_max)
{
    if (chn_num_max < 2) {  /* 2: roi_attr array len */
        sample_print("roi_attr array len not enough, need 2, current %d!\n", chn_num_max);
        return;
    }
    roi_attr[0].enable = 1;
    roi_attr[0].idx = 0;
    roi_attr[0].rect.x = 0;
    roi_attr[0].rect.y = 0;
    roi_attr[0].rect.width = 160; /* 160: rect.width value */
    roi_attr[0].rect.height = 160; /* 160: rect.height value */
    roi_attr[0].level = 0;

    roi_attr[1].enable = 1;
    roi_attr[1].idx = 0;
    roi_attr[1].rect.x = 0;
    roi_attr[1].rect.y = 0;
    roi_attr[1].rect.width = 160; /* 160: rect.width value */
    roi_attr[1].rect.height = 160; /* 160: rect.height value */
    roi_attr[1].level = 0;
}

static hi_s32 sample_venc_mjpeg_roimap_start_encode(ot_vpss_grp vpss_grp, ot_venc_jpeg_roi_attr *roi_attr,
    hi_size *enc_size, hi_s32 chn_num_max, sample_venc_vpss_chn *venc_vpss_chn)
{
    hi_s32 ret;
    ot_venc_gop_attr gop_attr;
    sample_comm_venc_chn_param venc_create_param[CHN_NUM_MAX] = {0};
    sample_comm_venc_chn_param venc_chn0_param, venc_chn1_param;
    sample_venc_roimap_chn_info roimap_chn_info = {0};
    ot_unused(chn_num_max);

    if ((ret = sample_comm_venc_get_gop_attr(OT_VENC_GOP_MODE_NORMAL_P, &gop_attr)) != HI_SUCCESS) {
        sample_print("Venc Get GopAttr for %#x!\n", ret);
        return ret;
    }

    sample_venc_set_jpeg_param(venc_create_param, gop_attr, CHN_NUM_MAX);

    venc_chn0_param = venc_create_param[0];
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[0], &venc_chn0_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        return ret;
    }

    venc_chn1_param = venc_create_param[1];
    ret = sample_comm_venc_start(venc_vpss_chn->venc_chn[1], &venc_chn1_param);
    if (ret != HI_SUCCESS) {
        sample_print("Venc Start failed for %#x!\n", ret);
        goto EXIT_VENC_CHN0_STOP;
    }

    roimap_chn_info.vpss_chn = venc_vpss_chn->vpss_chn;
    roimap_chn_info.venc_chn = venc_vpss_chn->venc_chn;
    roimap_chn_info.cnt = CHN_NUM_MAX;
    ret = sample_comm_venc_send_roimap_frame(vpss_grp, roimap_chn_info, enc_size, roi_attr);
    if (ret != HI_SUCCESS) {
        sample_print("sample_comm_venc_qpmap_send_frame failed for %#x!\n", ret);
        goto EXIT_VENC_CHN1_STOP;
    }

    /******************************************
     stream save process
    ******************************************/
    if ((ret = sample_comm_venc_start_get_stream(venc_vpss_chn->venc_chn, CHN_NUM_MAX)) != HI_SUCCESS) {
        sample_print("Start Venc failed!\n");
        goto EXIT_VENC_CHN1_STOP;
    }

    return HI_SUCCESS;

EXIT_VENC_CHN1_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[1]);
EXIT_VENC_CHN0_STOP:
    sample_comm_venc_stop(venc_vpss_chn->venc_chn[0]);

    return ret;
}

static hi_void sample_venc_mjpeg_roimap_exit_process()
{
    printf("please press twice ENTER to exit this sample\n");
    (hi_void)getchar();

    if (g_sample_venc_exit != HI_TRUE) {
        (hi_void)getchar();
    }

    sample_comm_venc_stop_send_roimap_frame();
    sample_comm_venc_stop_get_stream(CHN_NUM_MAX);
}

static hi_s32 sample_venc_mjpeg_roimap(hi_void)
{
    hi_s32 ret;
    sample_sns_type sns_type = OV_OS08B10_MIPI_8M_30FPS_12BIT;
    sample_vi_cfg vi_cfg;
    const hi_vi_pipe vi_pipe = 0;
    const hi_vi_chn vi_chn = 0;
    const hi_vpss_grp vpss_grp = 0;
    sample_venc_vpss_chn_attr vpss_param;
    sample_venc_vb_attr vb_attr = {0};
    ot_venc_jpeg_roi_attr roi_attr[CHN_NUM_MAX];
    hi_size enc_size[CHN_NUM_MAX];
    sample_venc_vpss_chn venc_vpss_chn = {0};

    sample_set_venc_vpss_chn(&venc_vpss_chn);
    sample_comm_vi_get_default_vi_cfg(sns_type, &vi_cfg);

    sample_venc_mjpeg_roi_attr_init(roi_attr, CHN_NUM_MAX);

    ret = sample_venc_init_param(enc_size, CHN_NUM_MAX, &(vi_cfg.dev_info.dev_attr.in_size), &vpss_param);
    if (ret != HI_SUCCESS) {
        return ret;
    }

    /******************************************
      step 1: init sys alloc common vb
    ******************************************/
    get_vb_attr(&(vi_cfg.dev_info.dev_attr.in_size), &vpss_param, &vb_attr);

    if ((ret = sample_venc_sys_init(&vb_attr)) != HI_SUCCESS) {
        return ret;
    }

    if ((ret = sample_venc_vi_init(&vi_cfg)) != HI_SUCCESS) {
        goto EXIT_SYS_STOP;
    }

    if ((ret = sample_venc_vpss_init(vpss_grp, &vpss_param)) != HI_SUCCESS) {
        goto EXIT_VI_STOP;
    }

    if ((ret = sample_comm_vi_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0)) != HI_SUCCESS) {
        goto EXIT_VPSS_STOP;
    }

    ret = sample_venc_mjpeg_roimap_start_encode(vpss_grp, roi_attr, enc_size, CHN_NUM_MAX, &venc_vpss_chn);
    if (ret != HI_SUCCESS) {
        goto EXIT_VI_VPSS_UNBIND;
    }

    /******************************************
     exit process
    ******************************************/
    sample_venc_mjpeg_roimap_exit_process();
    sample_venc_stop(&venc_vpss_chn);

EXIT_VI_VPSS_UNBIND:
    sample_comm_vi_un_bind_vpss(vi_pipe, vi_chn, vpss_grp, 0);
EXIT_VPSS_STOP:
    sample_venc_vpss_deinit(vpss_grp, &vpss_param);
EXIT_VI_STOP:
    sample_venc_vi_deinit(&vi_cfg);
EXIT_SYS_STOP:
    sample_comm_sys_exit();

    return ret;
}

static hi_s32 sample_venc_choose_mode(hi_u32 index)
{
    hi_s32 ret;

    switch (index) {
        case 0: /* 0: mode 0 */
            ret = sample_venc_normal();
            break;

        case 1: /* 1: mode 1 */
            ret = sample_venc_qpmap();
            break;

        case 2: /* 2: mode 2 */
            ret = sample_venc_intra_refresh();
            break;

        case 3: /* 3: mode 3 */
            ret = sample_venc_roi_bg();
            break;

        case 4: /* 4: mode 4 */
            ret = sample_venc_debreath_effect();
            break;

        case 5: /* 5: mode 5 */
            ret = sample_venc_mjpeg_roi_set();
            break;

        case 6: /* 6: mode 6 */
            ret = sample_venc_mjpeg_roimap();
            break;

        default:
            printf("the index is invalid!\n");
            return HI_FAILURE;
    }

    return ret;
}

/******************************************************************************
* function    : main()
* description : video venc sample
******************************************************************************/
#ifdef __LITEOS__
hi_s32 app_main(hi_s32 argc, hi_char *argv[])
#else
hi_s32 acl_main(hi_s32 argc, hi_char *argv[])
#endif
{
    hi_s32 ret;
    hi_u32 index;

    if (argc != 2) { /* 2:arg num */
        sample_venc_usage(argv[0]);
        return HI_FAILURE;
    }

    if (!strncmp(argv[1], "-h", 2)) { /* 2:arg num */
        sample_venc_usage(argv[0]);
        return HI_FAILURE;
    }

    if (strlen(argv[1]) != 1) {
        sample_venc_usage(argv[0]);
        return HI_FAILURE;
    }

    if (argv[1][0] < '0' || argv[1][0] > '9') {
        sample_venc_usage(argv[0]);
        return HI_FAILURE;
    }

    index = atoi(argv[1]);

#ifndef __LITEOS__
    sample_sys_signal(sample_venc_handle_sig);
#endif

    ret = sample_venc_choose_mode(index);
    if (ret == HI_SUCCESS) {
        printf("program exit normally!\n");
    } else {
        printf("program exit abnormally!\n");
    }

#ifdef __LITEOS__
    return ret;
#else
    exit(ret);
#endif
}

