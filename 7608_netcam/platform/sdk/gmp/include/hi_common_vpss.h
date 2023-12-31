/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __HI_COMMON_VPSS_H__
#define __HI_COMMON_VPSS_H__

#include "hi_type.h"
#include "hi_common.h"
#include "hi_errno.h"
#include "hi_common_video.h"
#include "hi_common_sys.h"
#include "ot_common_vpss.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HI_ERR_VPSS_NULL_PTR OT_ERR_VPSS_NULL_PTR
#define HI_ERR_VPSS_NOT_READY OT_ERR_VPSS_NOT_READY
#define HI_ERR_VPSS_INVALID_DEV_ID OT_ERR_VPSS_INVALID_DEV_ID
#define HI_ERR_VPSS_INVALID_CHN_ID OT_ERR_VPSS_INVALID_CHN_ID
#define HI_ERR_VPSS_EXIST OT_ERR_VPSS_EXIST
#define HI_ERR_VPSS_UNEXIST OT_ERR_VPSS_UNEXIST
#define HI_ERR_VPSS_NOT_SUPPORT OT_ERR_VPSS_NOT_SUPPORT
#define HI_ERR_VPSS_NOT_PERM OT_ERR_VPSS_NOT_PERM
#define HI_ERR_VPSS_NO_MEM OT_ERR_VPSS_NO_MEM
#define HI_ERR_VPSS_NO_BUF OT_ERR_VPSS_NO_BUF
#define HI_ERR_VPSS_SIZE_NOT_ENOUGH OT_ERR_VPSS_SIZE_NOT_ENOUGH
#define HI_ERR_VPSS_ILLEGAL_PARAM OT_ERR_VPSS_ILLEGAL_PARAM
#define HI_ERR_VPSS_BUSY OT_ERR_VPSS_BUSY
#define HI_ERR_VPSS_BUF_EMPTY OT_ERR_VPSS_BUF_EMPTY
#define HI_VPSS_CHN0 OT_VPSS_CHN0
#define HI_VPSS_CHN1 OT_VPSS_CHN1
#define HI_VPSS_CHN2 OT_VPSS_CHN2
#define HI_VPSS_CHN3 OT_VPSS_CHN3
#define HI_VPSS_INVALID_CHN OT_VPSS_INVALID_CHN
#define HI_VPSS_S_IDX_LEN OT_VPSS_S_IDX_LEN
#define HI_VPSS_3DNR_MAX_AUTO_PARAM_NUM OT_VPSS_3DNR_MAX_AUTO_PARAM_NUM
#define HI_VPSS_DEI_MODE_OFF OT_VPSS_DEI_MODE_OFF
#define HI_VPSS_DEI_MODE_ON OT_VPSS_DEI_MODE_ON
#define HI_VPSS_DEI_MODE_AUTO OT_VPSS_DEI_MODE_AUTO
#define HI_VPSS_DEI_MODE_BUTT OT_VPSS_DEI_MODE_BUTT
typedef ot_vpss_dei_mode hi_vpss_dei_mode;
#define HI_VPSS_NR_TYPE_VIDEO_NORM OT_VPSS_NR_TYPE_VIDEO_NORM
#define HI_VPSS_NR_TYPE_SNAP_NORM OT_VPSS_NR_TYPE_SNAP_NORM
#define HI_VPSS_NR_TYPE_VIDEO_SPATIAL OT_VPSS_NR_TYPE_VIDEO_SPATIAL
#define HI_VPSS_NR_TYPE_VIDEO_ENHANCED OT_VPSS_NR_TYPE_VIDEO_ENHANCED
#define HI_VPSS_NR_TYPE_BUTT OT_VPSS_NR_TYPE_BUTT
typedef ot_vpss_nr_type hi_vpss_nr_type;
#define HI_VPSS_NR_MOTION_MODE_NORM OT_VPSS_NR_MOTION_MODE_NORM
#define HI_VPSS_NR_MOTION_MODE_COMPENSATION OT_VPSS_NR_MOTION_MODE_COMPENSATION
#define HI_VPSS_NR_MOTION_MODE_BUTT OT_VPSS_NR_MOTION_MODE_BUTT
typedef ot_vpss_nr_motion_mode hi_vpss_nr_motion_mode;
typedef ot_vpss_nr_attr hi_vpss_nr_attr;
typedef ot_vpss_grp_attr hi_vpss_grp_attr;
#define HI_VPSS_CHN_MODE_AUTO OT_VPSS_CHN_MODE_AUTO
#define HI_VPSS_CHN_MODE_USER OT_VPSS_CHN_MODE_USER
#define HI_VPSS_CHN_MODE_BUTT OT_VPSS_CHN_MODE_BUTT
typedef ot_vpss_chn_mode hi_vpss_chn_mode;
typedef ot_vpss_chn_attr hi_vpss_chn_attr;
typedef ot_vpss_ext_chn_attr hi_vpss_ext_chn_attr;
typedef ot_vpss_sizer_info hi_vpss_sizer_info;
typedef ot_vpss_crop_info hi_vpss_crop_info;
typedef ot_vpss_nrx_norm_param_v1 hi_vpss_nrx_norm_param_v1;
typedef ot_vpss_nrx_v2_iey hi_vpss_nrx_v2_iey;
typedef ot_vpss_nrx_v2_sfy hi_vpss_nrx_v2_sfy;
typedef ot_vpss_nrx_v2_tfy hi_vpss_nrx_v2_tfy;
typedef ot_vpss_nrx_v2_mdy hi_vpss_nrx_v2_mdy;
typedef ot_vpss_nrx_v2_nrc0 hi_vpss_nrx_v2_nrc0;
typedef ot_vpss_nrx_v2_nrc1 hi_vpss_nrx_v2_nrc1;
typedef ot_vpss_nrx_v2 hi_vpss_nrx_v2;
typedef ot_vpss_nrx_param_manual_v2 hi_vpss_nrx_param_manual_v2;
typedef ot_vpss_nrx_param_auto_v2 hi_vpss_nrx_param_auto_v2;
typedef ot_vpss_nrx_norm_param_v2 hi_vpss_nrx_norm_param_v2;
typedef ot_vpss_nrx_adv_param hi_vpss_nrx_adv_param;
#define HI_VPSS_NR_V1 OT_VPSS_NR_V1
#define HI_VPSS_NR_V2 OT_VPSS_NR_V2
#define HI_VPSS_NR_BUTT OT_VPSS_NR_BUTT
typedef ot_vpss_nr_version hi_vpss_nr_version;
typedef ot_vpss_grp_nrx_param hi_vpss_grp_nrx_param;
typedef ot_vpss_grp_param hi_vpss_grp_param;
typedef ot_vpss_chn_sharpen_param hi_vpss_chn_sharpen_param;
typedef ot_vpss_mod_param hi_vpss_mod_param;
typedef ot_vpss_luma_result hi_vpss_luma_result;
#define HI_VPSS_COLOR_RANGE_LIMITED OT_VPSS_COLOR_RANGE_LIMITED
#define HI_VPSS_COLOR_RANGE_FULL OT_VPSS_COLOR_RANGE_FULL
#define HI_VPSS_COLOR_RANGE_BUTT OT_VPSS_COLOR_RANGE_BUTT
typedef ot_vpss_color_range hi_vpss_color_range;
typedef ot_vpss_ldci_gauss_coef_attr hi_vpss_ldci_gauss_coef_attr;
typedef ot_vpss_ldci_he_weight_attr hi_vpss_ldci_he_weight_attr;
typedef ot_vpss_ldci_attr hi_vpss_ldci_attr;

#ifdef __cplusplus
}
#endif
#endif /* __HI_COMMON_VPSS_H__ */
