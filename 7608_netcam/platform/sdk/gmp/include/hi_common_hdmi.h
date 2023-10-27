/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#ifndef __HI_COMMON_HDMI_H__
#define __HI_COMMON_HDMI_H__

#include "hi_type.h"
#include "hi_errno.h"
#include "ot_common_hdmi.h"


#ifdef __cplusplus
extern "C" {
#endif

#define HI_HDMI_MAX_AUDIO_CAPBILITY_CNT OT_HDMI_MAX_AUDIO_CAPBILITY_CNT
#define HI_HDMI_MAX_SAMPLE_RATE_NUM OT_HDMI_MAX_SAMPLE_RATE_NUM
#define HI_HDMI_MAX_BIT_DEPTH_NUM OT_HDMI_MAX_BIT_DEPTH_NUM
#define HI_HDMI_DETAIL_TIMING_MAX OT_HDMI_DETAIL_TIMING_MAX
#define HI_HDMI_EDID_RAW_DATA_LEN OT_HDMI_EDID_RAW_DATA_LEN
#define HI_HDMI_VENDOR_NAME_LEN OT_HDMI_VENDOR_NAME_LEN
#define HI_HDMI_HW_PARAM_NUM OT_HDMI_HW_PARAM_NUM
#define HI_HDMI_BKSV_LEN OT_HDMI_BKSV_LEN
#define HI_HDMI_MANUFACTURE_NAME_LEN OT_HDMI_MANUFACTURE_NAME_LEN
#define HI_HDMI_VENDOR_USER_DATA_MAX_LEN OT_HDMI_VENDOR_USER_DATA_MAX_LEN
#define HI_ERRNO_HDMI_NOT_INIT OT_ERRNO_HDMI_NOT_INIT
#define HI_ERRNO_HDMI_INVALID_PARAM OT_ERRNO_HDMI_INVALID_PARAM
#define HI_ERRNO_HDMI_NULL_PTR OT_ERRNO_HDMI_NULL_PTR
#define HI_ERRNO_HDMI_DEV_NOT_OPEN OT_ERRNO_HDMI_DEV_NOT_OPEN
#define HI_ERRNO_HDMI_DEV_NOT_CONNECT OT_ERRNO_HDMI_DEV_NOT_CONNECT
#define HI_ERRNO_HDMI_READ_SINK_FAILED OT_ERRNO_HDMI_READ_SINK_FAILED
#define HI_ERRNO_HDMI_INIT_ALREADY OT_ERRNO_HDMI_INIT_ALREADY
#define HI_ERRNO_HDMI_CALLBACK_ALREADY OT_ERRNO_HDMI_CALLBACK_ALREADY
#define HI_ERRNO_HDMI_INVALID_CALLBACK OT_ERRNO_HDMI_INVALID_CALLBACK
#define HI_ERRNO_HDMI_FEATURE_NO_SUPPORT OT_ERRNO_HDMI_FEATURE_NO_SUPPORT
#define HI_ERRNO_HDMI_BUS_BUSY OT_ERRNO_HDMI_BUS_BUSY
#define HI_ERRNO_HDMI_READ_EVENT_FAILED OT_ERRNO_HDMI_READ_EVENT_FAILED
#define HI_ERRNO_HDMI_NOT_START OT_ERRNO_HDMI_NOT_START
#define HI_ERRNO_HDMI_READ_EDID_FAILED OT_ERRNO_HDMI_READ_EDID_FAILED
#define HI_ERRNO_HDMI_INIT_FAILED OT_ERRNO_HDMI_INIT_FAILED
#define HI_ERRNO_HDMI_CREATE_TESK_FAILED OT_ERRNO_HDMI_CREATE_TESK_FAILED
#define HI_ERRNO_HDMI_MALLOC_FAILED OT_ERRNO_HDMI_MALLOC_FAILED
#define HI_ERRNO_HDMI_FREE_FAILED OT_ERRNO_HDMI_FREE_FAILED
#define HI_ERRNO_HDMI_PTHREAD_CREATE_FAILED OT_ERRNO_HDMI_PTHREAD_CREATE_FAILED
#define HI_ERRNO_HDMI_PTHREAD_JOIN_FAILED OT_ERRNO_HDMI_PTHREAD_JOIN_FAILED
#define HI_ERRNO_HDMI_STRATEGY_FAILED OT_ERRNO_HDMI_STRATEGY_FAILED
#define HI_ERRNO_HDMI_SET_ATTR_FAILED OT_ERRNO_HDMI_SET_ATTR_FAILED
#define HI_ERRNO_HDMI_CALLBACK_NOT_REGISTER OT_ERRNO_HDMI_CALLBACK_NOT_REGISTER
#define HI_ERRNO_HDMI_UNKNOWN_CMD OT_ERRNO_HDMI_UNKNOWN_CMD
#define HI_ERRNO_HDMI_MUTEX_LOCK_FAILED OT_ERRNO_HDMI_MUTEX_LOCK_FAILED
typedef ot_hdmi_errno hi_hdmi_errno;
#define HI_HDMI_AUDIO_CHN_CNT_STREAM OT_HDMI_AUDIO_CHN_CNT_STREAM
#define HI_HDMI_AUDIO_CHN_CNT_2 OT_HDMI_AUDIO_CHN_CNT_2
#define HI_HDMI_AUDIO_CHN_CNT_3 OT_HDMI_AUDIO_CHN_CNT_3
#define HI_HDMI_AUDIO_CHN_CNT_4 OT_HDMI_AUDIO_CHN_CNT_4
#define HI_HDMI_AUDIO_CHN_CNT_5 OT_HDMI_AUDIO_CHN_CNT_5
#define HI_HDMI_AUDIO_CHN_CNT_6 OT_HDMI_AUDIO_CHN_CNT_6
#define HI_HDMI_AUDIO_CHN_CNT_7 OT_HDMI_AUDIO_CHN_CNT_7
#define HI_HDMI_AUDIO_CHN_CNT_8 OT_HDMI_AUDIO_CHN_CNT_8
#define HI_HDMI_AUDIO_CHN_CNT_BUTT OT_HDMI_AUDIO_CHN_CNT_BUTT
typedef ot_hdmi_audio_chn_cnt hi_hdmi_audio_chn_cnt;
#define HI_HDMI_AUDIO_SAMPLE_SIZE_STREAM OT_HDMI_AUDIO_SAMPLE_SIZE_STREAM
#define HI_HDMI_AUDIO_SAMPLE_SIZE_16 OT_HDMI_AUDIO_SAMPLE_SIZE_16
#define HI_HDMI_AUDIO_SAMPLE_SIZE_20 OT_HDMI_AUDIO_SAMPLE_SIZE_20
#define HI_HDMI_AUDIO_SAMPLE_SIZE_24 OT_HDMI_AUDIO_SAMPLE_SIZE_24
#define HI_HDMI_AUDIO_SAMPLE_SIZE_BUTT OT_HDMI_AUDIO_SAMPLE_SIZE_BUTT
typedef ot_hdmi_audio_sample_size hi_hdmi_audio_sample_size;
#define HI_HDMI_AUDIO_SAMPLE_FREQ_STREAM OT_HDMI_AUDIO_SAMPLE_FREQ_STREAM
#define HI_HDMI_AUDIO_SAMPLE_FREQ_32000 OT_HDMI_AUDIO_SAMPLE_FREQ_32000
#define HI_HDMI_AUDIO_SAMPLE_FREQ_44100 OT_HDMI_AUDIO_SAMPLE_FREQ_44100
#define HI_HDMI_AUDIO_SAMPLE_FREQ_48000 OT_HDMI_AUDIO_SAMPLE_FREQ_48000
#define HI_HDMI_AUDIO_SAMPLE_FREQ_88200 OT_HDMI_AUDIO_SAMPLE_FREQ_88200
#define HI_HDMI_AUDIO_SAMPLE_FREQ_96000 OT_HDMI_AUDIO_SAMPLE_FREQ_96000
#define HI_HDMI_AUDIO_SAMPLE_FREQ_176400 OT_HDMI_AUDIO_SAMPLE_FREQ_176400
#define HI_HDMI_AUDIO_SAMPLE_FREQ_192000 OT_HDMI_AUDIO_SAMPLE_FREQ_192000
#define HI_HDMI_AUDIO_SAMPLE_FREQ_BUTT OT_HDMI_AUDIO_SAMPLE_FREQ_BUTT
typedef ot_hdmi_audio_sample_freq hi_hdmi_audio_sample_freq;
#define HI_HDMI_LEVEL_SHIFT_VAL_0_DB OT_HDMI_LEVEL_SHIFT_VAL_0_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_1_DB OT_HDMI_LEVEL_SHIFT_VAL_1_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_2_DB OT_HDMI_LEVEL_SHIFT_VAL_2_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_3_DB OT_HDMI_LEVEL_SHIFT_VAL_3_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_4_DB OT_HDMI_LEVEL_SHIFT_VAL_4_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_5_DB OT_HDMI_LEVEL_SHIFT_VAL_5_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_6_DB OT_HDMI_LEVEL_SHIFT_VAL_6_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_7_DB OT_HDMI_LEVEL_SHIFT_VAL_7_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_8_DB OT_HDMI_LEVEL_SHIFT_VAL_8_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_9_DB OT_HDMI_LEVEL_SHIFT_VAL_9_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_10_DB OT_HDMI_LEVEL_SHIFT_VAL_10_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_11_DB OT_HDMI_LEVEL_SHIFT_VAL_11_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_12_DB OT_HDMI_LEVEL_SHIFT_VAL_12_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_13_DB OT_HDMI_LEVEL_SHIFT_VAL_13_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_14_DB OT_HDMI_LEVEL_SHIFT_VAL_14_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_15_DB OT_HDMI_LEVEL_SHIFT_VAL_15_DB
#define HI_HDMI_LEVEL_SHIFT_VAL_BUTT OT_HDMI_LEVEL_SHIFT_VAL_BUTT
typedef ot_hdmi_level_shift_val hi_hdmi_level_shift_val;
#define HI_HDMI_LFE_PLAYBACK_NO OT_HDMI_LFE_PLAYBACK_NO
#define HI_HDMI_LFE_PLAYBACK_0_DB OT_HDMI_LFE_PLAYBACK_0_DB
#define HI_HDMI_LFE_PLAYBACK_10_DB OT_HDMI_LFE_PLAYBACK_10_DB
#define HI_HDMI_LFE_PLAYBACK_BUTT OT_HDMI_LFE_PLAYBACK_BUTT
typedef ot_hdmi_lfe_playback_level hi_hdmi_lfe_playback_level;
#define HI_INFOFRAME_TYPE_AVI OT_INFOFRAME_TYPE_AVI
#define HI_INFOFRAME_TYPE_AUDIO OT_INFOFRAME_TYPE_AUDIO
#define HI_INFOFRAME_TYPE_VENDORSPEC OT_INFOFRAME_TYPE_VENDORSPEC
#define HI_INFOFRAME_TYPE_BUTT OT_INFOFRAME_TYPE_BUTT
typedef ot_hdmi_infoframe_type hi_hdmi_infoframe_type;
#define HI_HDMI_VIDEO_FORMAT_1080P_60 OT_HDMI_VIDEO_FORMAT_1080P_60
#define HI_HDMI_VIDEO_FORMAT_1080P_50 OT_HDMI_VIDEO_FORMAT_1080P_50
#define HI_HDMI_VIDEO_FORMAT_1080P_30 OT_HDMI_VIDEO_FORMAT_1080P_30
#define HI_HDMI_VIDEO_FORMAT_1080P_25 OT_HDMI_VIDEO_FORMAT_1080P_25
#define HI_HDMI_VIDEO_FORMAT_1080P_24 OT_HDMI_VIDEO_FORMAT_1080P_24
#define HI_HDMI_VIDEO_FORMAT_1080i_60 OT_HDMI_VIDEO_FORMAT_1080i_60
#define HI_HDMI_VIDEO_FORMAT_1080i_50 OT_HDMI_VIDEO_FORMAT_1080i_50
#define HI_HDMI_VIDEO_FORMAT_720P_60 OT_HDMI_VIDEO_FORMAT_720P_60
#define HI_HDMI_VIDEO_FORMAT_720P_50 OT_HDMI_VIDEO_FORMAT_720P_50
#define HI_HDMI_VIDEO_FORMAT_576P_50 OT_HDMI_VIDEO_FORMAT_576P_50
#define HI_HDMI_VIDEO_FORMAT_480P_60 OT_HDMI_VIDEO_FORMAT_480P_60
#define HI_HDMI_VIDEO_FORMAT_PAL OT_HDMI_VIDEO_FORMAT_PAL
#define HI_HDMI_VIDEO_FORMAT_NTSC OT_HDMI_VIDEO_FORMAT_NTSC
#define HI_HDMI_VIDEO_FORMAT_861D_640X480_60 OT_HDMI_VIDEO_FORMAT_861D_640X480_60
#define HI_HDMI_VIDEO_FORMAT_VESA_800X600_60 OT_HDMI_VIDEO_FORMAT_VESA_800X600_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1024X768_60 OT_HDMI_VIDEO_FORMAT_VESA_1024X768_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1280X800_60 OT_HDMI_VIDEO_FORMAT_VESA_1280X800_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1280X1024_60 OT_HDMI_VIDEO_FORMAT_VESA_1280X1024_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1366X768_60 OT_HDMI_VIDEO_FORMAT_VESA_1366X768_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1440X900_60 OT_HDMI_VIDEO_FORMAT_VESA_1440X900_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1400X1050_60 OT_HDMI_VIDEO_FORMAT_VESA_1400X1050_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1600X1200_60 OT_HDMI_VIDEO_FORMAT_VESA_1600X1200_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1680X1050_60 OT_HDMI_VIDEO_FORMAT_VESA_1680X1050_60
#define HI_HDMI_VIDEO_FORMAT_VESA_1920X1200_60 OT_HDMI_VIDEO_FORMAT_VESA_1920X1200_60
#define HI_HDMI_VIDEO_FORMAT_2560x1440_30 OT_HDMI_VIDEO_FORMAT_2560x1440_30
#define HI_HDMI_VIDEO_FORMAT_2560x1440_60 OT_HDMI_VIDEO_FORMAT_2560x1440_60
#define HI_HDMI_VIDEO_FORMAT_2560x1600_60 OT_HDMI_VIDEO_FORMAT_2560x1600_60
#define HI_HDMI_VIDEO_FORMAT_1920x2160_30 OT_HDMI_VIDEO_FORMAT_1920x2160_30
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_24 OT_HDMI_VIDEO_FORMAT_3840X2160P_24
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_25 OT_HDMI_VIDEO_FORMAT_3840X2160P_25
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_30 OT_HDMI_VIDEO_FORMAT_3840X2160P_30
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_50 OT_HDMI_VIDEO_FORMAT_3840X2160P_50
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_60 OT_HDMI_VIDEO_FORMAT_3840X2160P_60
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_24 OT_HDMI_VIDEO_FORMAT_4096X2160P_24
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_25 OT_HDMI_VIDEO_FORMAT_4096X2160P_25
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_30 OT_HDMI_VIDEO_FORMAT_4096X2160P_30
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_50 OT_HDMI_VIDEO_FORMAT_4096X2160P_50
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_60 OT_HDMI_VIDEO_FORMAT_4096X2160P_60
#define HI_HDMI_VIDEO_FORMAT_3840X2160P_120 OT_HDMI_VIDEO_FORMAT_3840X2160P_120
#define HI_HDMI_VIDEO_FORMAT_4096X2160P_120 OT_HDMI_VIDEO_FORMAT_4096X2160P_120
#define HI_HDMI_VIDEO_FORMAT_7680X4320P_30 OT_HDMI_VIDEO_FORMAT_7680X4320P_30
#define HI_HDMI_VIDEO_FORMAT_VESA_CUSTOMER_DEFINE OT_HDMI_VIDEO_FORMAT_VESA_CUSTOMER_DEFINE
#define HI_HDMI_VIDEO_FORMAT_BUTT OT_HDMI_VIDEO_FORMAT_BUTT
typedef ot_hdmi_video_format hi_hdmi_video_format;
#define HI_HDMI_PIC_ASPECT_RATIO_NO_DATA OT_HDMI_PIC_ASPECT_RATIO_NO_DATA
#define HI_HDMI_PIC_ASPECT_RATIO_4TO3 OT_HDMI_PIC_ASPECT_RATIO_4TO3
#define HI_HDMI_PIC_ASPECT_RATIO_16TO9 OT_HDMI_PIC_ASPECT_RATIO_16TO9
#define HI_HDMI_PIC_ASPECT_RATIO_64TO27 OT_HDMI_PIC_ASPECT_RATIO_64TO27
#define HI_HDMI_PIC_ASPECT_RATIO_256TO135 OT_HDMI_PIC_ASPECT_RATIO_256TO135
#define HI_HDMI_PIC_ASPECT_RATIO_BUTT OT_HDMI_PIC_ASPECT_RATIO_BUTT
typedef ot_pic_aspect_ratio hi_pic_aspect_ratio;
#define HI_HDMI_ACTIVE_ASPECT_RATIO_16TO9_TOP OT_HDMI_ACTIVE_ASPECT_RATIO_16TO9_TOP
#define HI_HDMI_ACTIVE_ASPECT_RATIO_14TO9_TOP OT_HDMI_ACTIVE_ASPECT_RATIO_14TO9_TOP
#define HI_HDMI_ACTIVE_ASPECT_RATIO_16TO9_BOX_CENTER OT_HDMI_ACTIVE_ASPECT_RATIO_16TO9_BOX_CENTER
#define HI_HDMI_ACTIVE_ASPECT_RATIO_SAME_PIC OT_HDMI_ACTIVE_ASPECT_RATIO_SAME_PIC
#define HI_HDMI_ACTIVE_ASPECT_RATIO_4TO3_CENTER OT_HDMI_ACTIVE_ASPECT_RATIO_4TO3_CENTER
#define HI_HDMI_ACTIVE_ASPECT_RATIO_16TO9_CENTER OT_HDMI_ACTIVE_ASPECT_RATIO_16TO9_CENTER
#define HI_HDMI_ACTIVE_ASPECT_RATIO_14TO9_CENTER OT_HDMI_ACTIVE_ASPECT_RATIO_14TO9_CENTER
#define HI_HDMI_ACTIVE_ASPECT_RATIO_4TO3_14_9 OT_HDMI_ACTIVE_ASPECT_RATIO_4TO3_14_9
#define HI_HDMI_ACTIVE_ASPECT_RATIO_16TO9_14_9 OT_HDMI_ACTIVE_ASPECT_RATIO_16TO9_14_9
#define HI_HDMI_ACTIVE_ASPECT_RATIO_16TO9_4_3 OT_HDMI_ACTIVE_ASPECT_RATIO_16TO9_4_3
#define HI_HDMI_ACTIVE_ASPECT_RATIO_BUTT OT_HDMI_ACTIVE_ASPECT_RATIO_BUTT
typedef ot_hdmi_active_aspect_ratio hi_hdmi_active_aspect_ratio;
#define HI_HDMI_SAMPLE_RATE_UNKNOWN OT_HDMI_SAMPLE_RATE_UNKNOWN
#define HI_HDMI_SAMPLE_RATE_8K OT_HDMI_SAMPLE_RATE_8K
#define HI_HDMI_SAMPLE_RATE_11K OT_HDMI_SAMPLE_RATE_11K
#define HI_HDMI_SAMPLE_RATE_12K OT_HDMI_SAMPLE_RATE_12K
#define HI_HDMI_SAMPLE_RATE_16K OT_HDMI_SAMPLE_RATE_16K
#define HI_HDMI_SAMPLE_RATE_22K OT_HDMI_SAMPLE_RATE_22K
#define HI_HDMI_SAMPLE_RATE_24K OT_HDMI_SAMPLE_RATE_24K
#define HI_HDMI_SAMPLE_RATE_32K OT_HDMI_SAMPLE_RATE_32K
#define HI_HDMI_SAMPLE_RATE_44K OT_HDMI_SAMPLE_RATE_44K
#define HI_HDMI_SAMPLE_RATE_48K OT_HDMI_SAMPLE_RATE_48K
#define HI_HDMI_SAMPLE_RATE_88K OT_HDMI_SAMPLE_RATE_88K
#define HI_HDMI_SAMPLE_RATE_96K OT_HDMI_SAMPLE_RATE_96K
#define HI_HDMI_SAMPLE_RATE_176K OT_HDMI_SAMPLE_RATE_176K
#define HI_HDMI_SAMPLE_RATE_192K OT_HDMI_SAMPLE_RATE_192K
#define HI_HDMI_SAMPLE_RATE_768K OT_HDMI_SAMPLE_RATE_768K
#define HI_HDMI_SAMPLE_RATE_BUTT OT_HDMI_SAMPLE_RATE_BUTT
typedef ot_hdmi_sample_rate hi_hdmi_sample_rate;
#define HI_HDMI_AUDIO_FORMAT_CODE_RESERVED OT_HDMI_AUDIO_FORMAT_CODE_RESERVED
#define HI_HDMI_AUDIO_FORMAT_CODE_PCM OT_HDMI_AUDIO_FORMAT_CODE_PCM
#define HI_HDMI_AUDIO_FORMAT_CODE_AC3 OT_HDMI_AUDIO_FORMAT_CODE_AC3
#define HI_HDMI_AUDIO_FORMAT_CODE_MPEG1 OT_HDMI_AUDIO_FORMAT_CODE_MPEG1
#define HI_HDMI_AUDIO_FORMAT_CODE_MP3 OT_HDMI_AUDIO_FORMAT_CODE_MP3
#define HI_HDMI_AUDIO_FORMAT_CODE_MPEG2 OT_HDMI_AUDIO_FORMAT_CODE_MPEG2
#define HI_HDMI_AUDIO_FORMAT_CODE_AAC OT_HDMI_AUDIO_FORMAT_CODE_AAC
#define HI_HDMI_AUDIO_FORMAT_CODE_DTS OT_HDMI_AUDIO_FORMAT_CODE_DTS
#define HI_HDMI_AUDIO_FORMAT_CODE_ATRAC OT_HDMI_AUDIO_FORMAT_CODE_ATRAC
#define HI_HDMI_AUDIO_FORMAT_CODE_ONE_BIT OT_HDMI_AUDIO_FORMAT_CODE_ONE_BIT
#define HI_HDMI_AUDIO_FORMAT_CODE_DDP OT_HDMI_AUDIO_FORMAT_CODE_DDP
#define HI_HDMI_AUDIO_FORMAT_CODE_DTS_HD OT_HDMI_AUDIO_FORMAT_CODE_DTS_HD
#define HI_HDMI_AUDIO_FORMAT_CODE_MAT OT_HDMI_AUDIO_FORMAT_CODE_MAT
#define HI_HDMI_AUDIO_FORMAT_CODE_DST OT_HDMI_AUDIO_FORMAT_CODE_DST
#define HI_HDMI_AUDIO_FORMAT_CODE_WMA_PRO OT_HDMI_AUDIO_FORMAT_CODE_WMA_PRO
#define HI_HDMI_AUDIO_FORMAT_CODE_BUTT OT_HDMI_AUDIO_FORMAT_CODE_BUTT
typedef ot_hdmi_audio_format_code hi_hdmi_audio_format_code;
#define HI_HDMI_BIT_DEPTH_UNKNOWN OT_HDMI_BIT_DEPTH_UNKNOWN
#define HI_HDMI_BIT_DEPTH_8 OT_HDMI_BIT_DEPTH_8
#define HI_HDMI_BIT_DEPTH_16 OT_HDMI_BIT_DEPTH_16
#define HI_HDMI_BIT_DEPTH_18 OT_HDMI_BIT_DEPTH_18
#define HI_HDMI_BIT_DEPTH_20 OT_HDMI_BIT_DEPTH_20
#define HI_HDMI_BIT_DEPTH_24 OT_HDMI_BIT_DEPTH_24
#define HI_HDMI_BIT_DEPTH_32 OT_HDMI_BIT_DEPTH_32
#define HI_HDMI_BIT_DEPTH_BUTT OT_HDMI_BIT_DEPTH_BUTT
typedef ot_hdmi_bit_depth hi_hdmi_bit_depth;
#define HI_HDMI_ID_0 OT_HDMI_ID_0
#define HI_HDMI_ID_1 OT_HDMI_ID_1
#define HI_HDMI_ID_BUTT OT_HDMI_ID_BUTT
typedef ot_hdmi_id hi_hdmi_id;
#define HI_HDMI_EVENT_HOTPLUG OT_HDMI_EVENT_HOTPLUG
#define HI_HDMI_EVENT_NO_PLUG OT_HDMI_EVENT_NO_PLUG
#define HI_HDMI_EVENT_EDID_FAIL OT_HDMI_EVENT_EDID_FAIL
#define HI_HDMI_EVENT_BUTT OT_HDMI_EVENT_BUTT
typedef ot_hdmi_event_type hi_hdmi_event_type;
#define HI_HDMI_VIDEO_MODE_RGB444 OT_HDMI_VIDEO_MODE_RGB444
#define HI_HDMI_VIDEO_MODE_YCBCR422 OT_HDMI_VIDEO_MODE_YCBCR422
#define HI_HDMI_VIDEO_MODE_YCBCR444 OT_HDMI_VIDEO_MODE_YCBCR444
#define HI_HDMI_VIDEO_MODE_YCBCR420 OT_HDMI_VIDEO_MODE_YCBCR420
#define HI_HDMI_VIDEO_MODE_BUTT OT_HDMI_VIDEO_MODE_BUTT
typedef ot_hdmi_video_mode hi_hdmi_video_mode;
#define HI_HDMI_COLOR_SPACE_RGB444 OT_HDMI_COLOR_SPACE_RGB444
#define HI_HDMI_COLOR_SPACE_YCBCR422 OT_HDMI_COLOR_SPACE_YCBCR422
#define HI_HDMI_COLOR_SPACE_YCBCR444 OT_HDMI_COLOR_SPACE_YCBCR444
#define HI_HDMI_COLOR_SPACE_YCBCR420 OT_HDMI_COLOR_SPACE_YCBCR420
#define HI_HDMI_COLOR_SPACE_BUTT OT_HDMI_COLOR_SPACE_BUTT
typedef ot_hdmi_color_space hi_hdmi_color_space;
#define HI_HDMI_DEEP_COLOR_24BIT OT_HDMI_DEEP_COLOR_24BIT
#define HI_HDMI_DEEP_COLOR_30BIT OT_HDMI_DEEP_COLOR_30BIT
#define HI_HDMI_DEEP_COLOR_36BIT OT_HDMI_DEEP_COLOR_36BIT
#define HI_HDMI_DEEP_COLOR_BUTT OT_HDMI_DEEP_COLOR_BUTT
typedef ot_hdmi_deep_color hi_hdmi_deep_color;
#define HI_HDMI_BAR_INFO_NOT_VALID OT_HDMI_BAR_INFO_NOT_VALID
#define HI_HDMI_BAR_INFO_V OT_HDMI_BAR_INFO_V
#define HI_HDMI_BAR_INFO_H OT_HDMI_BAR_INFO_H
#define HI_HDMI_BAR_INFO_VH OT_HDMI_BAR_INFO_VH
#define HI_HDMI_BAR_INFO_BUTT OT_HDMI_BAR_INFO_BUTT
typedef ot_hdmi_bar_info hi_hdmi_bar_info;
#define HI_HDMI_SCAN_INFO_NO_DATA OT_HDMI_SCAN_INFO_NO_DATA
#define HI_HDMI_SCAN_INFO_OVERSCANNED OT_HDMI_SCAN_INFO_OVERSCANNED
#define HI_HDMI_SCAN_INFO_UNDERSCANNED OT_HDMI_SCAN_INFO_UNDERSCANNED
#define HI_HDMI_SCAN_INFO_BUTT OT_HDMI_SCAN_INFO_BUTT
typedef ot_hdmi_scan_info hi_hdmi_scan_info;
#define HI_HDMI_PIC_NON_UNIFORM_SCALING OT_HDMI_PIC_NON_UNIFORM_SCALING
#define HI_HDMI_PIC_SCALING_H OT_HDMI_PIC_SCALING_H
#define HI_HDMI_PIC_SCALING_V OT_HDMI_PIC_SCALING_V
#define HI_HDMI_PIC_SCALING_HV OT_HDMI_PIC_SCALING_HV
#define HI_HDMI_PIC_SCALING_BUTT OT_HDMI_PIC_SCALING_BUTT
typedef ot_hdmi_pic_scaline hi_hdmi_pic_scaline;
#define HI_HDMI_COMMON_COLORIMETRY_NO_DATA OT_HDMI_COMMON_COLORIMETRY_NO_DATA
#define HI_HDMI_COMMON_COLORIMETRY_ITU601 OT_HDMI_COMMON_COLORIMETRY_ITU601
#define HI_HDMI_COMMON_COLORIMETRY_ITU709 OT_HDMI_COMMON_COLORIMETRY_ITU709
#define HI_HDMI_COMMON_COLORIMETRY_BUTT OT_HDMI_COMMON_COLORIMETRY_BUTT
typedef ot_hdmi_colorimetry hi_hdmi_colorimetry;
#define HI_HDMI_COMMON_COLORIMETRY_XVYCC_601 OT_HDMI_COMMON_COLORIMETRY_XVYCC_601
#define HI_HDMI_COMMON_COLORIMETRY_XVYCC_709 OT_HDMI_COMMON_COLORIMETRY_XVYCC_709
#define HI_HDMI_COMMON_COLORIMETRY_S_YCC_601 OT_HDMI_COMMON_COLORIMETRY_S_YCC_601
#define HI_HDMI_COMMON_COLORIMETRY_ADOBE_YCC_601 OT_HDMI_COMMON_COLORIMETRY_ADOBE_YCC_601
#define HI_HDMI_COMMON_COLORIMETRY_ADOBE_RGB OT_HDMI_COMMON_COLORIMETRY_ADOBE_RGB
#define HI_HDMI_COMMON_COLORIMETRY_2020_CONST_LUMINOUS OT_HDMI_COMMON_COLORIMETRY_2020_CONST_LUMINOUS
#define HI_HDMI_COMMON_COLORIMETRY_2020_NON_CONST_LUMINOUS OT_HDMI_COMMON_COLORIMETRY_2020_NON_CONST_LUMINOUS
#define HI_HDMI_COMMON_COLORIMETRY_EXT_BUTT OT_HDMI_COMMON_COLORIMETRY_EXT_BUTT
typedef ot_hdmi_ex_colorimetry hi_hdmi_ex_colorimetry;
#define HI_HDMI_RGB_QUANT_DEFAULT_RANGE OT_HDMI_RGB_QUANT_DEFAULT_RANGE
#define HI_HDMI_RGB_QUANT_LIMITED_RANGE OT_HDMI_RGB_QUANT_LIMITED_RANGE
#define HI_HDMI_RGB_QUANT_FULL_RANGE OT_HDMI_RGB_QUANT_FULL_RANGE
#define HI_HDMI_RGB_QUANT_FULL_BUTT OT_HDMI_RGB_QUANT_FULL_BUTT
typedef ot_hdmi_rgb_quant_range hi_hdmi_rgb_quant_range;
#define HI_HDMI_YCC_QUANT_LIMITED_RANGE OT_HDMI_YCC_QUANT_LIMITED_RANGE
#define HI_HDMI_YCC_QUANT_FULL_RANGE OT_HDMI_YCC_QUANT_FULL_RANGE
#define HI_HDMI_YCC_QUANT_BUTT OT_HDMI_YCC_QUANT_BUTT
typedef ot_hdmi_ycc_quant_range hi_hdmi_ycc_quant_range;
#define HI_HDMI_PIXEL_REPET_NO OT_HDMI_PIXEL_REPET_NO
#define HI_HDMI_PIXEL_REPET_2_TIMES OT_HDMI_PIXEL_REPET_2_TIMES
#define HI_HDMI_PIXEL_REPET_3_TIMES OT_HDMI_PIXEL_REPET_3_TIMES
#define HI_HDMI_PIXEL_REPET_4_TIMES OT_HDMI_PIXEL_REPET_4_TIMES
#define HI_HDMI_PIXEL_REPET_5_TIMES OT_HDMI_PIXEL_REPET_5_TIMES
#define HI_HDMI_PIXEL_REPET_6_TIMES OT_HDMI_PIXEL_REPET_6_TIMES
#define HI_HDMI_PIXEL_REPET_7_TIMES OT_HDMI_PIXEL_REPET_7_TIMES
#define HI_HDMI_PIXEL_REPET_8_TIMES OT_HDMI_PIXEL_REPET_8_TIMES
#define HI_HDMI_PIXEL_REPET_9_TIMES OT_HDMI_PIXEL_REPET_9_TIMES
#define HI_HDMI_PIXEL_REPET_10_TIMES OT_HDMI_PIXEL_REPET_10_TIMES
#define HI_HDMI_PIXEL_REPET_BUTT OT_HDMI_PIXEL_REPET_BUTT
typedef ot_hdmi_pixel_repetition hi_hdmi_pixel_repetition;
#define HI_HDMI_CONTNET_GRAPHIC OT_HDMI_CONTNET_GRAPHIC
#define HI_HDMI_CONTNET_PHOTO OT_HDMI_CONTNET_PHOTO
#define HI_HDMI_CONTNET_CINEMA OT_HDMI_CONTNET_CINEMA
#define HI_HDMI_CONTNET_GAME OT_HDMI_CONTNET_GAME
#define HI_HDMI_CONTNET_BUTT OT_HDMI_CONTNET_BUTT
typedef ot_hdmi_content_type hi_hdmi_content_type;
#define HI_HDMI_AUDIO_CODING_REFER_STREAM_HEAD OT_HDMI_AUDIO_CODING_REFER_STREAM_HEAD
#define HI_HDMI_AUDIO_CODING_PCM OT_HDMI_AUDIO_CODING_PCM
#define HI_HDMI_AUDIO_CODING_AC3 OT_HDMI_AUDIO_CODING_AC3
#define HI_HDMI_AUDIO_CODING_MPEG1 OT_HDMI_AUDIO_CODING_MPEG1
#define HI_HDMI_AUDIO_CODING_MP3 OT_HDMI_AUDIO_CODING_MP3
#define HI_HDMI_AUDIO_CODING_MPEG2 OT_HDMI_AUDIO_CODING_MPEG2
#define HI_HDMI_AUDIO_CODING_AACLC OT_HDMI_AUDIO_CODING_AACLC
#define HI_HDMI_AUDIO_CODING_DTS OT_HDMI_AUDIO_CODING_DTS
#define HI_HDMI_AUDIO_CODING_ATRAC OT_HDMI_AUDIO_CODING_ATRAC
#define HI_HDMI_AUDIO_CODIND_ONE_BIT_AUDIO OT_HDMI_AUDIO_CODIND_ONE_BIT_AUDIO
#define HI_HDMI_AUDIO_CODING_ENAHNCED_AC3 OT_HDMI_AUDIO_CODING_ENAHNCED_AC3
#define HI_HDMI_AUDIO_CODING_DTS_HD OT_HDMI_AUDIO_CODING_DTS_HD
#define HI_HDMI_AUDIO_CODING_MAT OT_HDMI_AUDIO_CODING_MAT
#define HI_HDMI_AUDIO_CODING_DST OT_HDMI_AUDIO_CODING_DST
#define HI_HDMI_AUDIO_CODING_WMA_PRO OT_HDMI_AUDIO_CODING_WMA_PRO
#define HI_HDMI_AUDIO_CODING_BUTT OT_HDMI_AUDIO_CODING_BUTT
typedef ot_hdmi_coding_type hi_hdmi_coding_type;
#define HI_HDMI_AUDIO_SPEAKER_FL_FR OT_HDMI_AUDIO_SPEAKER_FL_FR
#define HI_HDMI_AUDIO_SPEAKER_LFE OT_HDMI_AUDIO_SPEAKER_LFE
#define HI_HDMI_AUDIO_SPEAKER_FC OT_HDMI_AUDIO_SPEAKER_FC
#define HI_HDMI_AUDIO_SPEAKER_RL_RR OT_HDMI_AUDIO_SPEAKER_RL_RR
#define HI_HDMI_AUDIO_SPEAKER_RC OT_HDMI_AUDIO_SPEAKER_RC
#define HI_HDMI_AUDIO_SPEAKER_FLC_FRC OT_HDMI_AUDIO_SPEAKER_FLC_FRC
#define HI_HDMI_AUDIO_SPEAKER_RLC_RRC OT_HDMI_AUDIO_SPEAKER_RLC_RRC
#define HI_HDMI_AUDIO_SPEAKER_FLW_FRW OT_HDMI_AUDIO_SPEAKER_FLW_FRW
#define HI_HDMI_AUDIO_SPEAKER_FLH_FRH OT_HDMI_AUDIO_SPEAKER_FLH_FRH
#define HI_HDMI_AUDIO_SPEAKER_TC OT_HDMI_AUDIO_SPEAKER_TC
#define HI_HDMI_AUDIO_SPEAKER_FCH OT_HDMI_AUDIO_SPEAKER_FCH
#define HI_HDMI_AUDIO_SPEAKER_BUTT OT_HDMI_AUDIO_SPEAKER_BUTT
typedef ot_hdmi_audio_speaker hi_hdmi_audio_speaker;
typedef ot_hdmi_audio_info hi_hdmi_audio_info;
typedef ot_hdmi_eotf hi_hdmi_eotf;
typedef ot_hdmi_hdr_metadata_type hi_hdmi_hdr_metadata_type;
typedef ot_hdmi_hdr_capability hi_hdmi_hdr_capability;
typedef ot_hdmi_timing_info hi_hdmi_timing_info;
typedef ot_hdmi_detail_timing hi_hdmi_detail_timing;
typedef ot_hdmi_sink_capability hi_hdmi_sink_capability;
typedef ot_hdmi_attr hi_hdmi_attr;
typedef ot_hdmi_edid hi_hdmi_edid;
typedef ot_hdmi_audio_infoframe hi_hdmi_audio_infoframe;
typedef ot_hdmi_avi_infoframe hi_hdmi_avi_infoframe;
typedef ot_hdmi_vendorspec_infoframe hi_hdmi_vendorspec_infoframe;
typedef ot_hdmi_infoframe_unit hi_hdmi_infoframe_unit;
typedef ot_hdmi_infoframe hi_hdmi_infoframe;
typedef ot_hdmi_callback hi_hdmi_callback;
typedef ot_hdmi_callback_func hi_hdmi_callback_func;
typedef ot_hdmi_hw_param hi_hdmi_hw_param;
typedef ot_hdmi_hw_spec hi_hdmi_hw_spec;
typedef ot_hdmi_status hi_hdmi_status;
#define HI_HDMI_TRACE_LEN_0 OT_HDMI_TRACE_LEN_0
#define HI_HDMI_TRACE_LEN_1 OT_HDMI_TRACE_LEN_1
#define HI_HDMI_TRACE_LEN_2 OT_HDMI_TRACE_LEN_2
#define HI_HDMI_TRACE_LEN_3 OT_HDMI_TRACE_LEN_3
#define HI_HDMI_TRACE_LEN_4 OT_HDMI_TRACE_LEN_4
#define HI_HDMI_TRACE_LEN_5 OT_HDMI_TRACE_LEN_5
#define HI_HDMI_TRACE_LEN_6 OT_HDMI_TRACE_LEN_6
#define HI_HDMI_TRACE_LEN_7 OT_HDMI_TRACE_LEN_7
#define HI_HDMI_TRACE_LEN_8 OT_HDMI_TRACE_LEN_8
#define HI_HDMI_TRACE_DEFAULT OT_HDMI_TRACE_DEFAULT
#define HI_HDMI_TRACE_BUTT OT_HDMI_TRACE_BUTT
typedef ot_hdmi_trace_len hi_hdmi_trace_len;
typedef ot_hdmi_mod_param hi_hdmi_mod_param;
#define HI_ERR_HDMI_NOT_INIT OT_ERR_HDMI_NOT_INIT
#define HI_ERR_HDMI_INVALID_PARA OT_ERR_HDMI_INVALID_PARA
#define HI_ERR_HDMI_NULL_PTR OT_ERR_HDMI_NULL_PTR
#define HI_ERR_HDMI_DEV_NOT_OPEN OT_ERR_HDMI_DEV_NOT_OPEN
#define HI_ERR_HDMI_DEV_NOT_CONNECT OT_ERR_HDMI_DEV_NOT_CONNECT
#define HI_ERR_HDMI_READ_SINK_FAILED OT_ERR_HDMI_READ_SINK_FAILED
#define HI_ERR_HDMI_INIT_ALREADY OT_ERR_HDMI_INIT_ALREADY
#define HI_ERR_HDMI_CALLBACK_ALREADY OT_ERR_HDMI_CALLBACK_ALREADY
#define HI_ERR_HDMI_INVALID_CALLBACK OT_ERR_HDMI_INVALID_CALLBACK
#define HI_ERR_HDMI_FEATURE_NO_SUPPORT OT_ERR_HDMI_FEATURE_NO_SUPPORT
#define HI_ERR_HDMI_BUS_BUSY OT_ERR_HDMI_BUS_BUSY
#define HI_ERR_HDMI_READ_EVENT_FAILED OT_ERR_HDMI_READ_EVENT_FAILED
#define HI_ERR_HDMI_NOT_START OT_ERR_HDMI_NOT_START
#define HI_ERR_HDMI_READ_EDID_FAILED OT_ERR_HDMI_READ_EDID_FAILED
#define HI_ERR_HDMI_INIT_FAILED OT_ERR_HDMI_INIT_FAILED
#define HI_ERR_HDMI_CREATE_TESK_FAILED OT_ERR_HDMI_CREATE_TESK_FAILED
#define HI_ERR_HDMI_MALLOC_FAILED OT_ERR_HDMI_MALLOC_FAILED
#define HI_ERR_HDMI_FREE_FAILED OT_ERR_HDMI_FREE_FAILED
#define HI_ERR_HDMI_PTHREAD_CREATE_FAILED OT_ERR_HDMI_PTHREAD_CREATE_FAILED
#define HI_ERR_HDMI_PTHREAD_JOIN_FAILED OT_ERR_HDMI_PTHREAD_JOIN_FAILED
#define HI_ERR_HDMI_STRATEGY_FAILED OT_ERR_HDMI_STRATEGY_FAILED
#define HI_ERR_HDMI_SET_ATTR_FAILED OT_ERR_HDMI_SET_ATTR_FAILED
#define HI_ERR_HDMI_CALLBACK_NOT_REGISTER OT_ERR_HDMI_CALLBACK_NOT_REGISTER
#define HI_ERR_HDMI_UNKNOWN_COMMAND OT_ERR_HDMI_UNKNOWN_COMMAND
#define HI_ERR_HDMI_MUTEX_LOCK_FAILED OT_ERR_HDMI_MUTEX_LOCK_FAILED

#ifdef __cplusplus
}
#endif
#endif /* __HI_COMMON_HDMI_H__ */
