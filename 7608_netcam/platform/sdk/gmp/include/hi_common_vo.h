/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __HI_COMMON_VO_H__
#define __HI_COMMON_VO_H__

#include "hi_type.h"
#include "hi_common.h"
#include "hi_errno.h"
#include "hi_common_video.h"
#include "ot_common_vo.h"


#ifdef __cplusplus
extern "C" {
#endif

#define HI_ERR_VO_INVALID_DEV_ID OT_ERR_VO_INVALID_DEV_ID
#define HI_ERR_VO_INVALID_CHN_ID OT_ERR_VO_INVALID_CHN_ID
#define HI_ERR_VO_INVALID_LAYER_ID OT_ERR_VO_INVALID_LAYER_ID
#define HI_ERR_VO_ILLEGAL_PARAM OT_ERR_VO_ILLEGAL_PARAM
#define HI_ERR_VO_NULL_PTR OT_ERR_VO_NULL_PTR
#define HI_ERR_VO_NOT_CFG OT_ERR_VO_NOT_CFG
#define HI_ERR_VO_NOT_SUPPORT OT_ERR_VO_NOT_SUPPORT
#define HI_ERR_VO_NOT_PERM OT_ERR_VO_NOT_PERM
#define HI_ERR_VO_NOT_ENABLE OT_ERR_VO_NOT_ENABLE
#define HI_ERR_VO_NOT_DISABLE OT_ERR_VO_NOT_DISABLE
#define HI_ERR_VO_NO_MEM OT_ERR_VO_NO_MEM
#define HI_ERR_VO_NOT_READY OT_ERR_VO_NOT_READY
#define HI_ERR_VO_TIMEOUT OT_ERR_VO_TIMEOUT
#define HI_ERR_VO_BUSY OT_ERR_VO_BUSY
#define HI_ERR_VO_NOT_BINDED OT_ERR_VO_NOT_BINDED
#define HI_ERR_VO_BINDED OT_ERR_VO_BINDED
#define HI_VO_INTF_CVBS OT_VO_INTF_CVBS
#define HI_VO_INTF_VGA OT_VO_INTF_VGA
#define HI_VO_INTF_BT656 OT_VO_INTF_BT656
#define HI_VO_INTF_BT1120 OT_VO_INTF_BT1120
#define HI_VO_INTF_HDMI OT_VO_INTF_HDMI
#define HI_VO_INTF_RGB_6BIT OT_VO_INTF_RGB_6BIT
#define HI_VO_INTF_RGB_8BIT OT_VO_INTF_RGB_8BIT
#define HI_VO_INTF_RGB_16BIT OT_VO_INTF_RGB_16BIT
#define HI_VO_INTF_RGB_18BIT OT_VO_INTF_RGB_18BIT
#define HI_VO_INTF_RGB_24BIT OT_VO_INTF_RGB_24BIT
#define HI_VO_INTF_MIPI OT_VO_INTF_MIPI
#define HI_VO_INTF_MIPI_SLAVE OT_VO_INTF_MIPI_SLAVE
#define HI_VO_INTF_HDMI1 OT_VO_INTF_HDMI1
typedef ot_vo_intf_type hi_vo_intf_type;
#define HI_VO_OUT_PAL OT_VO_OUT_PAL
#define HI_VO_OUT_NTSC OT_VO_OUT_NTSC
#define HI_VO_OUT_960H_PAL OT_VO_OUT_960H_PAL
#define HI_VO_OUT_960H_NTSC OT_VO_OUT_960H_NTSC
#define HI_VO_OUT_640x480_60 OT_VO_OUT_640x480_60
#define HI_VO_OUT_480P60 OT_VO_OUT_480P60
#define HI_VO_OUT_576P50 OT_VO_OUT_576P50
#define HI_VO_OUT_800x600_60 OT_VO_OUT_800x600_60
#define HI_VO_OUT_1024x768_60 OT_VO_OUT_1024x768_60
#define HI_VO_OUT_720P50 OT_VO_OUT_720P50
#define HI_VO_OUT_720P60 OT_VO_OUT_720P60
#define HI_VO_OUT_1280x800_60 OT_VO_OUT_1280x800_60
#define HI_VO_OUT_1280x1024_60 OT_VO_OUT_1280x1024_60
#define HI_VO_OUT_1366x768_60 OT_VO_OUT_1366x768_60
#define HI_VO_OUT_1400x1050_60 OT_VO_OUT_1400x1050_60
#define HI_VO_OUT_1440x900_60 OT_VO_OUT_1440x900_60
#define HI_VO_OUT_1680x1050_60 OT_VO_OUT_1680x1050_60
#define HI_VO_OUT_1080P24 OT_VO_OUT_1080P24
#define HI_VO_OUT_1080P25 OT_VO_OUT_1080P25
#define HI_VO_OUT_1080P30 OT_VO_OUT_1080P30
#define HI_VO_OUT_1080I50 OT_VO_OUT_1080I50
#define HI_VO_OUT_1080I60 OT_VO_OUT_1080I60
#define HI_VO_OUT_1080P50 OT_VO_OUT_1080P50
#define HI_VO_OUT_1080P60 OT_VO_OUT_1080P60
#define HI_VO_OUT_1600x1200_60 OT_VO_OUT_1600x1200_60
#define HI_VO_OUT_1920x1200_60 OT_VO_OUT_1920x1200_60
#define HI_VO_OUT_1920x2160_30 OT_VO_OUT_1920x2160_30
#define HI_VO_OUT_2560x1440_30 OT_VO_OUT_2560x1440_30
#define HI_VO_OUT_2560x1440_60 OT_VO_OUT_2560x1440_60
#define HI_VO_OUT_2560x1600_60 OT_VO_OUT_2560x1600_60
#define HI_VO_OUT_3840x2160_24 OT_VO_OUT_3840x2160_24
#define HI_VO_OUT_3840x2160_25 OT_VO_OUT_3840x2160_25
#define HI_VO_OUT_3840x2160_30 OT_VO_OUT_3840x2160_30
#define HI_VO_OUT_3840x2160_50 OT_VO_OUT_3840x2160_50
#define HI_VO_OUT_3840x2160_60 OT_VO_OUT_3840x2160_60
#define HI_VO_OUT_4096x2160_24 OT_VO_OUT_4096x2160_24
#define HI_VO_OUT_4096x2160_25 OT_VO_OUT_4096x2160_25
#define HI_VO_OUT_4096x2160_30 OT_VO_OUT_4096x2160_30
#define HI_VO_OUT_4096x2160_50 OT_VO_OUT_4096x2160_50
#define HI_VO_OUT_4096x2160_60 OT_VO_OUT_4096x2160_60
#define HI_VO_OUT_7680x4320_30 OT_VO_OUT_7680x4320_30
#define HI_VO_OUT_240x320_50 OT_VO_OUT_240x320_50
#define HI_VO_OUT_320x240_50 OT_VO_OUT_320x240_50
#define HI_VO_OUT_240x320_60 OT_VO_OUT_240x320_60
#define HI_VO_OUT_320x240_60 OT_VO_OUT_320x240_60
#define HI_VO_OUT_800x600_50 OT_VO_OUT_800x600_50
#define HI_VO_OUT_720x1280_60 OT_VO_OUT_720x1280_60
#define HI_VO_OUT_1080x1920_60 OT_VO_OUT_1080x1920_60
#define HI_VO_OUT_USER OT_VO_OUT_USER
#define HI_VO_OUT_BUTT OT_VO_OUT_BUTT
typedef ot_vo_intf_sync hi_vo_intf_sync;
typedef ot_vo_sync_info hi_vo_sync_info;
typedef ot_vo_pub_attr hi_vo_pub_attr;
typedef ot_vo_dev_param hi_vo_dev_param;
typedef ot_vo_mod_param hi_vo_mod_param;
#define HI_VO_CLK_SRC_PLL OT_VO_CLK_SRC_PLL
#define HI_VO_CLK_SRC_LCDMCLK OT_VO_CLK_SRC_LCDMCLK
#define HI_VO_CLK_SRC_PLL_FOUT4 OT_VO_CLK_SRC_PLL_FOUT4
#define HI_VO_CLK_SRC_FIXED OT_VO_CLK_SRC_FIXED
#define HI_VO_CLK_SRC_BUTT OT_VO_CLK_SRC_BUTT
typedef ot_vo_clk_src hi_vo_clk_src;
#define HI_VO_FIXED_CLK_148_5M OT_VO_FIXED_CLK_148_5M
#define HI_VO_FIXED_CLK_134_4M OT_VO_FIXED_CLK_134_4M
#define HI_VO_FIXED_CLK_108M OT_VO_FIXED_CLK_108M
#define HI_VO_FIXED_CLK_74_25M OT_VO_FIXED_CLK_74_25M
#define HI_VO_FIXED_CLK_64M OT_VO_FIXED_CLK_64M
#define HI_VO_FIXED_CLK_54M OT_VO_FIXED_CLK_54M
#define HI_VO_FIXED_CLK_37_125M OT_VO_FIXED_CLK_37_125M
#define HI_VO_FIXED_CLK_13_5M OT_VO_FIXED_CLK_13_5M
#define HI_VO_FIXED_CLK_BUTT OT_VO_FIXED_CLK_BUTT
typedef ot_vo_fixed_clk hi_vo_fixed_clk;
typedef ot_vo_pll hi_vo_pll;
typedef ot_vo_user_sync_attr hi_vo_user_sync_attr;
typedef ot_vo_user_sync_info hi_vo_user_sync_info;
typedef ot_vo_less_buf_attr hi_vo_less_buf_attr;
typedef ot_vo_user_notify_attr hi_vo_user_notify_attr;
#define HI_VO_INTF_STATUS_NO_PLUG OT_VO_INTF_STATUS_NO_PLUG
#define HI_VO_INTF_STATUS_PLUG OT_VO_INTF_STATUS_PLUG
#define HI_VO_INTF_STATUS_BUTT OT_VO_INTF_STATUS_BUTT
typedef ot_vo_intf_plug_status hi_vo_intf_plug_status;
typedef ot_vo_intf_status hi_vo_intf_status;
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_BT601LIMIT OT_VO_CSC_MATRIX_BT601LIMIT_TO_BT601LIMIT
#define HI_VO_CSC_MATRIX_BT601FULL_TO_BT601LIMIT OT_VO_CSC_MATRIX_BT601FULL_TO_BT601LIMIT
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_BT601LIMIT OT_VO_CSC_MATRIX_BT709LIMIT_TO_BT601LIMIT
#define HI_VO_CSC_MATRIX_BT709FULL_TO_BT601LIMIT OT_VO_CSC_MATRIX_BT709FULL_TO_BT601LIMIT
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_BT709LIMIT OT_VO_CSC_MATRIX_BT601LIMIT_TO_BT709LIMIT
#define HI_VO_CSC_MATRIX_BT601FULL_TO_BT709LIMIT OT_VO_CSC_MATRIX_BT601FULL_TO_BT709LIMIT
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_BT709LIMIT OT_VO_CSC_MATRIX_BT709LIMIT_TO_BT709LIMIT
#define HI_VO_CSC_MATRIX_BT709FULL_TO_BT709LIMIT OT_VO_CSC_MATRIX_BT709FULL_TO_BT709LIMIT
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_BT601FULL OT_VO_CSC_MATRIX_BT601LIMIT_TO_BT601FULL
#define HI_VO_CSC_MATRIX_BT601FULL_TO_BT601FULL OT_VO_CSC_MATRIX_BT601FULL_TO_BT601FULL
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_BT601FULL OT_VO_CSC_MATRIX_BT709LIMIT_TO_BT601FULL
#define HI_VO_CSC_MATRIX_BT709FULL_TO_BT601FULL OT_VO_CSC_MATRIX_BT709FULL_TO_BT601FULL
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_BT709FULL OT_VO_CSC_MATRIX_BT601LIMIT_TO_BT709FULL
#define HI_VO_CSC_MATRIX_BT601FULL_TO_BT709FULL OT_VO_CSC_MATRIX_BT601FULL_TO_BT709FULL
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_BT709FULL OT_VO_CSC_MATRIX_BT709LIMIT_TO_BT709FULL
#define HI_VO_CSC_MATRIX_BT709FULL_TO_BT709FULL OT_VO_CSC_MATRIX_BT709FULL_TO_BT709FULL
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_RGBFULL OT_VO_CSC_MATRIX_BT601LIMIT_TO_RGBFULL
#define HI_VO_CSC_MATRIX_BT601FULL_TO_RGBFULL OT_VO_CSC_MATRIX_BT601FULL_TO_RGBFULL
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_RGBFULL OT_VO_CSC_MATRIX_BT709LIMIT_TO_RGBFULL
#define HI_VO_CSC_MATRIX_BT709FULL_TO_RGBFULL OT_VO_CSC_MATRIX_BT709FULL_TO_RGBFULL
#define HI_VO_CSC_MATRIX_BT601LIMIT_TO_RGBLIMIT OT_VO_CSC_MATRIX_BT601LIMIT_TO_RGBLIMIT
#define HI_VO_CSC_MATRIX_BT601FULL_TO_RGBLIMIT OT_VO_CSC_MATRIX_BT601FULL_TO_RGBLIMIT
#define HI_VO_CSC_MATRIX_BT709LIMIT_TO_RGBLIMIT OT_VO_CSC_MATRIX_BT709LIMIT_TO_RGBLIMIT
#define HI_VO_CSC_MATRIX_BT709FULL_TO_RGBLIMIT OT_VO_CSC_MATRIX_BT709FULL_TO_RGBLIMIT
#define HI_VO_CSC_MATRIX_RGBFULL_TO_BT601LIMIT OT_VO_CSC_MATRIX_RGBFULL_TO_BT601LIMIT
#define HI_VO_CSC_MATRIX_RGBFULL_TO_BT601FULL OT_VO_CSC_MATRIX_RGBFULL_TO_BT601FULL
#define HI_VO_CSC_MATRIX_RGBFULL_TO_BT709LIMIT OT_VO_CSC_MATRIX_RGBFULL_TO_BT709LIMIT
#define HI_VO_CSC_MATRIX_RGBFULL_TO_BT709FULL OT_VO_CSC_MATRIX_RGBFULL_TO_BT709FULL
#define HI_VO_CSC_MATRIX_BUTT OT_VO_CSC_MATRIX_BUTT
#define HI_VO_CLK_EDGE_SINGLE OT_VO_CLK_EDGE_SINGLE
#define HI_VO_CLK_EDGE_DUAL OT_VO_CLK_EDGE_DUAL
#define HI_VO_CLK_EDGE_BUTT OT_VO_CLK_EDGE_BUTT
typedef ot_vo_clk_edge hi_vo_clk_edge;
typedef ot_vo_csc_matrix hi_vo_csc_matrix;
typedef ot_vo_csc hi_vo_csc;
typedef ot_vo_vga_param hi_vo_vga_param;
typedef ot_vo_hdmi_param hi_vo_hdmi_param;
typedef ot_vo_rgb_param hi_vo_rgb_param;
typedef ot_vo_bt_param hi_vo_bt_param;
typedef ot_vo_mipi_param hi_vo_mipi_param;
#define HI_VO_PARTITION_MODE_SINGLE OT_VO_PARTITION_MODE_SINGLE
#define HI_VO_PARTITION_MODE_MULTI OT_VO_PARTITION_MODE_MULTI
#define HI_VO_PARTITION_MODE_BUTT OT_VO_PARTITION_MODE_BUTT
typedef ot_vo_partition_mode hi_vo_partition_mode;
typedef ot_vo_video_layer_attr hi_vo_video_layer_attr;
typedef ot_vo_layer_param hi_vo_layer_param;
typedef ot_vo_chn_attr hi_vo_chn_attr;
typedef ot_vo_chn_param hi_vo_chn_param;
#define HI_VO_ZOOM_IN_RECT OT_VO_ZOOM_IN_RECT
#define HI_VO_ZOOM_IN_RATIO OT_VO_ZOOM_IN_RATIO
#define HI_VO_ZOOM_IN_BUTT OT_VO_ZOOM_IN_BUTT
typedef ot_vo_zoom_in_type hi_vo_zoom_in_type;
typedef ot_vo_zoom_ratio hi_vo_zoom_ratio;
typedef ot_vo_zoom_attr hi_vo_zoom_attr;
typedef ot_vo_border hi_vo_border;
#define HI_VO_MIRROR_NONE OT_VO_MIRROR_NONE
#define HI_VO_MIRROR_HOR OT_VO_MIRROR_HOR
#define HI_VO_MIRROR_VER OT_VO_MIRROR_VER
#define HI_VO_MIRROR_BOTH OT_VO_MIRROR_BOTH
#define HI_VO_MIRROR_BUTT OT_VO_MIRROR_BUTT
typedef ot_vo_mirror_mode hi_vo_mirror_mode;
typedef ot_vo_chn_status hi_vo_chn_status;
typedef ot_vo_wbc_attr hi_vo_wbc_attr;
#define HI_VO_WBC_MODE_NORM OT_VO_WBC_MODE_NORM
#define HI_VO_WBC_MODE_DROP_REPEAT OT_VO_WBC_MODE_DROP_REPEAT
#define HI_VO_WBC_MODE_PROGRESSIVE_TO_INTERLACED OT_VO_WBC_MODE_PROGRESSIVE_TO_INTERLACED
#define HI_VO_WBC_MODE_BUTT OT_VO_WBC_MODE_BUTT
typedef ot_vo_wbc_mode hi_vo_wbc_mode;
#define HI_VO_WBC_SRC_DEV OT_VO_WBC_SRC_DEV
#define HI_VO_WBC_SRC_VIDEO OT_VO_WBC_SRC_VIDEO
#define HI_VO_WBC_SRC_BUTT OT_VO_WBC_SRC_BUTT
typedef ot_vo_wbc_src_type hi_vo_wbc_src_type;
typedef ot_vo_wbc_src hi_vo_wbc_src;
#define HI_VO_CAS_MODE_SINGLE OT_VO_CAS_MODE_SINGLE
#define HI_VO_CAS_MODE_DUAL OT_VO_CAS_MODE_DUAL
#define HI_VO_CAS_MODE_BUTT OT_VO_CAS_MODE_BUTT
typedef ot_vo_cas_mode hi_vo_cas_mode;
#define HI_VO_CAS_MODE_SINGLE_EDGE OT_VO_CAS_MODE_SINGLE_EDGE
#define HI_VO_CAS_MODE_DUAL_EDGE OT_VO_CAS_MODE_DUAL_EDGE
#define HI_VO_CAS_MODE_EDGE_BUTT OT_VO_CAS_MODE_EDGE_BUTT
typedef ot_vo_cas_data_transmission_mode hi_vo_cas_data_transmission_mode;
#define HI_VO_CAS_RGN_64 OT_VO_CAS_RGN_64
#define HI_VO_CAS_RGN_32 OT_VO_CAS_RGN_32
#define HI_VO_CAS_RGN_BUTT OT_VO_CAS_RGN_BUTT
typedef ot_vo_cas_rgn hi_vo_cas_rgn;
typedef ot_vo_cas_attr hi_vo_cas_attr;

#ifdef __cplusplus
}
#endif
#endif /* __HI_COMMON_VO_H__ */
