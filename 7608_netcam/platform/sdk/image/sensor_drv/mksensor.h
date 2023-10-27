/*!
*****************************************************************************
** \file        mksensor.h
**
** \version     $Id: mksensor.h 12902 2017-10-24 02:35:40Z yulindeng $
**
** \brief
**
** \attention   THIS SAMPLE CODE IS PROVIDED AS IS. GOKE MICROELECTRONICS
**              ACCEPTS NO RESPONSIBILITY OR LIABILITY FOR ANY ERRORS OR
**              OMMISSIONS
**
** (C) Copyright 2012-2016 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/
#ifndef _MKSENSOR_H_
#define _MKSENSOR_H_
#include "adi_isp.h"

typedef enum
{
    GADI_ISP_TUNNING_DATA_ADJ = 0XBA50,
    GADI_ISP_TUNNING_DATA_ADJ_IR,
    GADI_ISP_TUNNING_DATA_CHROMA_SCALE,
    GADI_ISP_TUNNING_DATA_AWB,
    GADI_ISP_TUNNING_DATA_50HZ_LINES,
    GADI_ISP_TUNNING_DATA_60HZ_LINES,
    GADI_ISP_TUNNING_DATA_TILE_CONFIG,
    GADI_ISP_TUNNING_DATA_AE_AGC_DGAIN,
    GADI_ISP_TUNNING_DATA_DLIGHT,
    GADI_ISP_TUNNING_DATA_AE_MANUAL_LE,
    GADI_ISP_TUNNING_DATA_RGB_TO_YUV,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_1080P_30,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_1080P_25,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_960P_30,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_960P_25,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_720P_60,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_720P_50,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_720P_30,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_720P_25,
//added by Liu Yuxuan 20170221
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_VGA_30,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_VGA_25,
//end
//added by Hu Yin 06-01-2016
    GADI_ISP_TUNNING_DATA_LENS_SHADING_PROPERTY,
    GADI_ISP_TUNNING_DATA_WARP_CONTROL,
//end

    GADI_ISP_TUNNING_DATA_3D_TABLE_1 = 0XBA80,
    GADI_ISP_TUNNING_DATA_3D_TABLE_2,
    GADI_ISP_TUNNING_DATA_3D_TABLE_3,
    GADI_ISP_TUNNING_DATA_3D_TABLE_4,

    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE0 = 0XBB50,  // would be delete
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE1,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE2,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE3,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE4,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE5,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE6,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE7,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE8,
    GADI_ISP_TUNNING_DATA_AE_SHT_DGAIN_MODE9,

}GADI_ISP_TUNNING_DATA_TypeT; /*param_load.h*/



//*****************************************************************************
//*****************************************************************************
//** Defines and Macros
//*****************************************************************************
//*****************************************************************************
#define GADI_VIDEO_FORMAT(format)             (format)
#define GADI_VIDEO_FORMAT_AUTO                GADI_VIDEO_FORMAT(0)
#define GADI_VIDEO_FORMAT_INTERLACE           GADI_VIDEO_FORMAT(1)
#define GADI_VIDEO_FORMAT_PROGRESSIVE         GADI_VIDEO_FORMAT(2)

#define GADI_VIDEO_TYPE(type)                 (type)
#define GADI_VIDEO_TYPE_AUTO                  GADI_VIDEO_TYPE(0)
#define GADI_VIDEO_TYPE_YUV_601               GADI_VIDEO_TYPE(1)
#define GADI_VIDEO_TYPE_YUV_656               GADI_VIDEO_TYPE(2)
#define GADI_VIDEO_TYPE_RGB_601               GADI_VIDEO_TYPE(3)
#define GADI_VIDEO_TYPE_RGB_656               GADI_VIDEO_TYPE(4)
#define GADI_VIDEO_TYPE_RGB_RAW               GADI_VIDEO_TYPE(5)    //Sensor
#define GADI_VIDEO_TYPE_YUV_BT1120            GADI_VIDEO_TYPE(6)
#define GADI_VIDEO_TYPE_RGB_BT1120            GADI_VIDEO_TYPE(7)
#define GADI_VIDEO_TYPE_I80_601               GADI_VIDEO_TYPE(8)    //I80
#define GADI_VIDEO_TYPE_RGB_555               GADI_VIDEO_TYPE(9)
#define GADI_VIDEO_TYPE_RGB_565               GADI_VIDEO_TYPE(10)
#define GADI_VIDEO_TYPE_RGB_666               GADI_VIDEO_TYPE(11)
#define GADI_VIDEO_TYPE_RGB_888               GADI_VIDEO_TYPE(12)
#define GADI_VIDEO_TYPE_YUV_420               GADI_VIDEO_TYPE(13)
#define GADI_VIDEO_TYPE_YUV_422               GADI_VIDEO_TYPE(14)

#define GADI_VIDEO_IN_INTERFACE(interface)    (interface)
#define GADI_VIDEO_IN_AUTO                    GADI_VIDEO_IN_INTERFACE(0)
#define GADI_VIDEO_IN_DVP                     GADI_VIDEO_IN_INTERFACE(1)
#define GADI_VIDEO_IN_BT601                   GADI_VIDEO_IN_INTERFACE(2)
#define GADI_VIDEO_IN_BT656                   GADI_VIDEO_IN_INTERFACE(3)
#define GADI_VIDEO_IN_BT1120                  GADI_VIDEO_IN_INTERFACE(4)
#define GADI_VIDEO_IN_MIPI_1LANE              GADI_VIDEO_IN_INTERFACE(5)
#define GADI_VIDEO_IN_MIPI_2LANES             GADI_VIDEO_IN_INTERFACE(6)

#define GADI_VIDEO_TYPE_IS_YUV(type)          (((type) == GADI_VIDEO_TYPE_YUV_601) || ((type) == GADI_VIDEO_TYPE_YUV_656) ||     \
                                                ((type) == GADI_VIDEO_TYPE_YUV_BT1120) || ((type) == GADI_VIDEO_TYPE_YUV_420) \
                                                ((type) == GADI_VIDEO_TYPE_YUV_422))
                                                
#define GADI_VIDEO_IN_INTERFACE_TYPE_IS_MIPI(interface) (((interface) == GADI_VIDEO_IN_MIPI_1LANE) || ((interface) == GADI_VIDEO_IN_MIPI_2LANES))

#define GADI_VIDEO_BITS(bits)                 (bits)
#define GADI_VIDEO_BITS_AUTO                  GADI_VIDEO_BITS(0)
#define GADI_VIDEO_BITS_8                     GADI_VIDEO_BITS(8)
#define GADI_VIDEO_BITS_10                    GADI_VIDEO_BITS(10)
#define GADI_VIDEO_BITS_12                    GADI_VIDEO_BITS(12)
#define GADI_VIDEO_BITS_14                    GADI_VIDEO_BITS(14)
#define GADI_VIDEO_BITS_16                    GADI_VIDEO_BITS(16)

#define GADI_VIDEO_MODE(W,H,F,PI)         (((PI<<31)&0x80000000)|((F<<24)&0x3F000000)|(((W/4)<<12)&0x00FFF000)|((H/2)&0x00000FFF))
#define GADI_VIDEO_MODE_GET_WIDTH(mode)   ((mode&0x00FFF000)>>10)
#define GADI_VIDEO_MODE_GET_HEIGTH(mode)  ((mode&0x00000FFF)<<1)
#define GADI_VIDEO_MODE_GET_FPS(mode)     ((mode&0x3F000000)>>24)
#define GADI_VIDEO_MODE_GET_PI(mode)      ((mode&0x80000000)>>31)
#define GADI_VIDEO_MODE_GET_MODE(mode)    (mode&0x00FFFFFF)

#define GADI_VI_SENSOR_INITIAL_REG_NUM               (280)
#define GADI_VI_SENSOR_PLL_TABLE_NUM                 (16)
#define GADI_VI_SENSOR_PLL_REG_NUM                   (16)
#define GADI_VI_SENSOR_VIDEO_FORMAT_TABLE_NUM        (16)
#define GADI_VI_SENSOR_VIDEO_FORMAT_REG_NUM          (32)
#define GADI_VI_SENSOR_VIDEO_MIRROR_REG_NUM          (16)

#define GADI_VI_SENSOR_TABLE_FLAG_END                (0xFFFFFFFF)
#define GADI_VI_SENSOR_TABLE_FLAG_NULL               (0xFFFFFFFF)
#define GADI_VI_SENSOR_TABLE_FLAG_SLEEP              (0xFFFFFFFE)
#define GADI_VI_SENSOR_TABLE_FLAG_MSLEEP             (0xFFFFFFFE)
#define GADI_VI_SENSOR_TABLE_FLAG_MDELAY             (0xFFFFFFFD)
#define GADI_VI_SENSOR_TABLE_FLAG_UDELAY             (0xFFFFFFFC)
#define GADI_VI_SENSOR_TABLE_FLAG_READ_THEN_WRITE    (0xFFFFFFFB)
#define GADI_VI_SENSOR_TABLE_FLAG_FIND_THEN_WRITE    (0xFFFFFFFA)
#define GADI_VI_SENSOR_TABLE_FLAG_CHCEK_SKIP         (0xFFFFFFF9)
#define GADI_VI_SENSOR_TABLE_FLAG_WRITE_REG          (0xFE000000)

#define GADI_VIDEO_FPS(fps)     (DIV_ROUND(512000000,fps))



//*****************************************************************************
//*****************************************************************************
//** Enumerated types
//*****************************************************************************
//*****************************************************************************
typedef enum
{
    GADI_VI_CMOS_CHANNEL_TYPE_AUTO                = 0x00,
}GADI_VI_CMOS_SRC_TYPE_E;

typedef enum
{
    GADI_VI_CCD_CHANNEL_TYPE_AUTO                 = 0x00,
}GADI_VI_CCD_SRC_TYPE_E;

typedef enum
{
    GADI_VI_DECODER_CHANNEL_TYPE_CVBS             = 0x00,
    GADI_VI_DECODER_CHANNEL_TYPE_SVIDEO,
    GADI_VI_DECODER_CHANNEL_TYPE_YPBPR,
    GADI_VI_DECODER_CHANNEL_TYPE_HDMI,
    GADI_VI_DECODER_CHANNEL_TYPE_VGA,
    GADI_VI_DECODER_CHANNEL_TYPE_NUM,
}GADI_VI_DEC_SRC_TYPE_E;

typedef enum {
    GADI_VI_BAYER_PATTERN_RG       = 0,
    GADI_VI_BAYER_PATTERN_BG,
    GADI_VI_BAYER_PATTERN_GR,
    GADI_VI_BAYER_PATTERN_GB,
    GADI_VI_YUV_CR_Y0_CB_Y1,
    GADI_VI_YUV_CB_Y0_CR_Y1,
    GADI_VI_YUV_Y0_CR_Y1_CB,
    GADI_VI_YUV_Y0_CB_Y1_CR,
    GADI_VI_BAYER_PATTERN_AUTO     = 255,
}GADI_VI_BayerPatternEnumT;


//*****************************************************************************
//*****************************************************************************
//** Data Structures
//*****************************************************************************
//*****************************************************************************
typedef struct
{
    GADI_U16 width;
    GADI_U16 height;
    GADI_U8  format;
    GADI_U8  type;
    GADI_U8  interface;
    GADI_U8  bits;
#define GADI_VIDEO_RATIO(ratio)               (ratio)
#define GADI_VIDEO_RATIO_AUTO                 GADI_VIDEO_RATIO(0)
#define GADI_VIDEO_RATIO_4_3                  GADI_VIDEO_RATIO(1)
#define GADI_VIDEO_RATIO_16_9                 GADI_VIDEO_RATIO(2)
#define GADI_VIDEO_RATIO_1_1                  GADI_VIDEO_RATIO(4)
    GADI_U8  ratio;
    GADI_U32 mipi_bps;    //per lane
    GADI_U32 srm;
    GADI_U32 pll_index;
    GADI_U32 max_fps;
    GADI_U32 min_fps;
    GADI_U32 auto_fps;
    GADI_ISP_TUNNING_DATA_TypeT dgain_type;
}GADI_VI_SensorInfoT;

typedef struct
{
    GADI_U16 def_start_x;
    GADI_U16 def_start_y;
    GADI_U16 def_width;
    GADI_U16 def_height;
    GADI_U16 sync_start;
    GADI_U8  bayer_pattern;     /* GADI_VI_BayerPatternEnumT */
    GADI_U8  type_ext;
    GADI_U16 slvs_eav_col;
    GADI_U16 slvs_sav2sav_dist;
    GADI_VI_BayerPatternEnumT   mirror_bayer_pattern[4];
}GADI_VIDEO_InfoT;

typedef struct
{
    GADI_U32 mode;                      /*!< GADI_VIDEO_MODE*/
#define GADI_VI_SRC_DISABLED            (0x00)
#define GADI_VI_SRC_ENABLED_FOR_VIDEO   (0x01 << 0)
#define GADI_VI_SRC_ENABLED_FOR_STILL   (0x01 << 1)
    GADI_U32 preview_mode_type;
    GADI_U32 still_mode_type;
}GADI_VIDEO_ModeT;

typedef struct
{
#define GADI_GAIN_OPT_SET_NOW             0
#define GADI_GAIN_OPT_READ_AND_CHECK      1
#define GADI_GAIN_OPT_SKIP                2
    u32 opt         : 8;
    u32 skip_frame  : 8;
    u32 frame_count : 8;
    u32 update      : 8;
}GADI_VI_SensorGainControlT;

typedef struct
{
    GADI_U32 reg;
    GADI_U32 data;
    GADI_U32 mask;
    GADI_VI_SensorGainControlT control;
}GADI_VI_SensorRegT;

typedef struct
{
    GADI_U32 reg;
    GADI_U32 data;
    GADI_U32 mask;
    GADI_U16 right;
    GADI_U16 shift_bits;
    GADI_S32 fix;
}GADI_VI_Sensor_HMLRegT;

typedef struct
{
    GADI_U32 pixclk;
    GADI_U32 extclk;
    GADI_VI_SensorRegT  regs[GADI_VI_SENSOR_PLL_REG_NUM];
}GADI_VI_SensorPllRegTabT ;

typedef struct
{
    // ��Ƶ��Ϣ��
    GADI_VIDEO_InfoT    video_info_table;
    GADI_VI_SensorRegT  fmt_reg[GADI_VI_SENSOR_VIDEO_FORMAT_REG_NUM];
    GADI_VI_SensorInfoT vi_info;
    // ��Ƶ��ʽ��
    GADI_VIDEO_ModeT    video_mode_table;
    // gain/shutter time�Ĵ�����vsync�жϺ����ms��Χ��д��
    GADI_U8             update_after_vsync_start;
    GADI_U8             update_after_vsync_end;
    GADI_U16            reserved;
}GADI_VI_SensorVideoFmtTabT;

typedef struct
{
    GADI_U32 version;
    GADI_U32 mask;
#define GADI_MAX_SENSOR_VER_CTL_REG_NUM     GADI_VI_SENSOR_INITIAL_REG_NUM
    GADI_VI_SensorRegT  regs[GADI_MAX_SENSOR_VER_CTL_REG_NUM];
}GADI_VI_SensorVerCtlRegTabT ;

typedef struct
{
    GADI_S32     reserve;
    GADI_S32     db_max;
    GADI_S32     db_min;
    GADI_S32     db_step;
} GADI_VI_AGCInfoT;

typedef struct
{
    GADI_U32 db_value;
#define GADI_MAX_SENSOR_GAIN_REG_NUM     10
    GADI_U32 reg_val[GADI_MAX_SENSOR_GAIN_REG_NUM];
    GADI_VI_SensorGainControlT control[GADI_MAX_SENSOR_GAIN_REG_NUM];
}GADI_VI_SensorGainTableT;

typedef struct
{
    // sensor����
#define GADI_VI_MAX_NAME_LENGTH                       0x20
    char name[GADI_VI_MAX_NAME_LENGTH];
    // ͨ��i2c���ƻ���spi����
#define GADI_SENSOR_CTL_I2C          0
#define GADI_SENSOR_CTL_SPI          1
#define GADI_SENSOR_CTL_I2C_GROUP_WR (1<<3)
    GADI_U8  ctl_type;
    // �Ĵ�����ַ����
    GADI_U8  addr_len;               /*!< reg addr width*/
    // �Ĵ������ݳ���
    GADI_U8  data_len;               /*!< reg data width*/
    // sensor��λ��Ч��ƽ
#define GADI_GPIO_HIGH  0x01
#define GADI_GPIO_LOW   0x00
    GADI_U8  reset_active_level;     /*!<GADI_GPIO_HIGH  or GADI_GPIO_LOW*/;
    // ͨ��i2c/spi���Ƶ�i2c/spi��ַ,ʹ��7bitģʽ
    GADI_U32 hw_addr[4];
    GADI_U32 hw_addr_index;    
    // sensor id�Ĵ�����
#define GADI_MAX_SENSOR_ID_REG_NUM       2
    GADI_VI_SensorRegT  id_reg[GADI_MAX_SENSOR_ID_REG_NUM];
#define GADI_MAX_SENSOR_RESET_REG_NUM    8
    // sensor��λ���ƼĴ�����
    GADI_VI_SensorRegT  reset_reg[GADI_MAX_SENSOR_RESET_REG_NUM];
    // software standby registers
#define GADI_MAX_SENSOR_STANDBY_REG_NUM  2    
    GADI_VI_SensorRegT  standby_reg[GADI_MAX_SENSOR_STANDBY_REG_NUM];

    GADI_VI_SensorRegT  test_reg;
}GADI_VI_SensorHWInfoT;

typedef struct
{
    // �Ƿ����ø��¿���ģʽ
    GADI_U8  agc_control_by_temperatue;
    // Optimal sensor�¶ȼ���ģʽ
    GADI_U8  Aptina_temperatue_mode;
    // ���¿����߼�����ʱgain_reg�б����ƵļĴ���������
    GADI_U8  temperatue_control_reg_index;
    // ���¿����߼�������
    GADI_U8  temperatue_gain_num;
    // �¶ȼĴ���
    GADI_VI_SensorRegT  temperatue_reg;
    // gain�Ĵ�����
    GADI_VI_SensorRegT          gain_reg[GADI_MAX_SENSOR_GAIN_REG_NUM];
    // gain��
#define GADI_MAX_SENSOR_TEMP_GAIN_ROW_NUM    16
    GADI_VI_SensorGainTableT    temperatue_gain[GADI_MAX_SENSOR_TEMP_GAIN_ROW_NUM];
    // �¿�again��
#define GADI_MAX_SENSOR_AGAIN_ROW_NUM       0x100
    GADI_U32                    temperatue_control[GADI_MAX_SENSOR_AGAIN_ROW_NUM];
}GADI_VI_SensorTempConInfoT;

typedef struct
{
#define GADI_SENSOR_HW_INFO_MAGIC_START 0x47495043    // GIPC
    // �Ժ��checkʹ��
    GADI_U32 magic_start;
#define GADI_SENSOR_HW_INFO_VERSION     0x20171128
    GADI_U32 version;
    // �ṹ����
    GADI_U32 length;
    // ͨ���ţ��Զ����
    GADI_INT src_id;

    // �ļ���������
    GADI_U32 file_date;
    GADI_VI_SensorHWInfoT   HwInfo;

    // ��ӡ������Ϣ
    GADI_U8  debug_print;
#define GADI_VI_INPUT_FORMAT_RGB_RAW        0x00
#define GADI_VI_INPUT_FORMAT_YUV_422_INTLC  0x01
#define GADI_VI_INPUT_FORMAT_YUV_422_PROG   0x02
    // ��������������ģʽ
    GADI_U8  input_format;
    // ��ʱȫ��ʹ��1
    GADI_U8  field_format;
    GADI_U8  reserved1;

    // ��ʱȫ��ʹ��0
    GADI_INT adapter_id;
    // sensor id����Ҫ��image��adiȫ��ͳһһ��
    GADI_ISP_SensorModelEnumT sensor_id; /* GADI_ISP_SensorModelEnumT */

    // �⼸��ֵ��û����ȫ���ף���ʱʹ����������ԭ��������ֵ
    GADI_U16 sensor_double_step;
    GADI_U8 typeofsensor;
    GADI_U8 sync_time_delay;
    // �����豸����
    union
    {
        GADI_VI_CMOS_SRC_TYPE_E     cmos;
        GADI_VI_CCD_SRC_TYPE_E      ccd;
        GADI_VI_DEC_SRC_TYPE_E      decoder;
        GADI_U32                    dummy;
    } source_type;
#define GADI_VI_DEV_TYPE_CMOS                         (0x01 << 0)
#define GADI_VI_DEV_TYPE_CCD                          (0x01 << 1)
#define GADI_VI_DEV_TYPE_DECODER                      (0x01 << 2)
    // ����Ժ����ԣ���sensor��������source_type����
    GADI_U32 dev_type;

#define GADI_VIDEO_SYSTEM_AUTO                (0x0)
#define GADI_VIDEO_SYSTEM_NTSC                (0x1)
#define GADI_VIDEO_SYSTEM_PAL                 (0x2)
#define GADI_VIDEO_SYSTEM_SECAM               (0x4)
#define GADI_VIDEO_SYSTEM_ALL                 (0xF)
    // ��Ƶ���ݸ�ʽ
    GADI_U8  video_system;
#define GADI_VI_RISING_EDGE                     (0)
#define GADI_VI_FALLING_EDGE                    (1)
    GADI_U8  vs_polarity;
    GADI_U8  hs_polarity;
    GADI_U8  data_edge;
#define GADI_VI_EMB_SYNC_OFF                    (0)
#define GADI_VI_EMB_SYNC_ON                     (1)
    GADI_U8  emb_sync_switch;
#define GADI_VI_EMB_SYNC_LOWER_PEL              (0)
#define GADI_VI_EMB_SYNC_UPPER_PEL              (1)
#define GADI_VI_EMB_SYNC_BOTH_PELS              (2)
    GADI_U8  emb_sync_loc;
#define GADI_VI_EMB_SYNC_ITU_656                (0)
#define GADI_VI_EMB_SYNC_FULL_RANGE             (1)
    GADI_U8  emb_sync_mode;

    GADI_U8  reserved2;

    // sensor��������pixel
    GADI_U16 max_width;
    // sensor��������pixel
    GADI_U16 max_height;
    // Ĭ���ع�ʱ��
    GADI_U32 def_sh_time;
    // ֧�ֵ���Ƶ��ʽ����
    GADI_U32 fmt_num;
    // Ĭ����Ƶ��ʽ
    GADI_U32 auto_fmt;               /*!< GADI_VIDEO_MODE*/
    //////////////////////////////////////////////
    // pll���ƼĴ�����
    GADI_VI_SensorPllRegTabT    pll_table[GADI_VI_SENSOR_PLL_TABLE_NUM];
    // ��Ƶ��ʽ���ƼĴ�����
    GADI_VI_SensorVideoFmtTabT  video_fmt_table[GADI_VI_SENSOR_VIDEO_FORMAT_TABLE_NUM];
#define GADI_VI_SRC_MIRROR_HORRIZONTALLY_VERTICALLY 0x00
#define GADI_VI_SRC_MIRROR_HORRIZONTALLY            0x01
#define GADI_VI_SRC_MIRROR_VERTICALLY               0x02
#define GADI_VI_SRC_MIRROR_NONE                     0x03
    // ��Ƶ��ת���ƼĴ�����
    GADI_VI_SensorRegT          mirror_table[4][GADI_VI_SENSOR_VIDEO_MIRROR_REG_NUM];
#define GADI_MAX_SENSOR_VER_REG_NUM      2
    // sensor �汾�Ĵ�����
    GADI_VI_SensorRegT          version_reg[GADI_MAX_SENSOR_VER_REG_NUM];
    // sensor�Ĵ����Ƿ��а汾����,��д�汾����ļĴ���������û����Ϊ0
    GADI_U8                     reg_ctl_by_ver_num;
    GADI_U8                     reserved3;
    GADI_U16                    reserved4;
#define GADI_SENSOR_VER_CTL_REG_INIT        0
#define GADI_SENSOR_VER_CTL_REG_PLL         1
#define GADI_SENSOR_VER_CTL_REG_FMT         2
#define GADI_SENSOR_VER_CTL_REG_INIT_FIRST  3

#define GADI_MAX_SENSOR_VER_NUM     8
    // sensor��ͬ�汾�ļĴ���������б�reg_ctl_by_ver_num!=0����Ч
    // �Ĵ�����maskֵ����Ϊ�����Ҫ����init/pll/fmt�б�ļĴ�����������initǰ������ʼ��
    GADI_VI_SensorVerCtlRegTabT control_by_version_reg[GADI_MAX_SENSOR_VER_NUM];
    //////////////////////////////////////////////
#define GADI_SENSOR_CAL_CLK_PIXCLK       0
#define GADI_SENSOR_CAL_CLK_PIXCLK_DIV2  1
#define GADI_SENSOR_CAL_CLK_EXTCLK       2
#define GADI_SENSOR_CAL_CLK_EXTCLK_DIV2  3
#define GADI_SENSOR_CAL_CLK_PIXCLK_X2    4
#define GADI_SENSOR_CAL_CLK_EXTCLK_X2    5
#define GADI_SENSOR_CAL_CLK_PIXCLK_X3_DIV16    6
#define GADI_SENSOR_CAL_CLK_PIXCLK_DIV4  7


    // vmax��shutter time�����ʱ�ӱ�׼
    GADI_U8  vmax_clk_ref;
    GADI_U8  shs_clk_ref;
#define GADI_SENSOR_CAL_SHS_VMAX_MODE    0
#define GADI_SENSOR_CAL_SHS_BLK_MODE     1
    // ����shutter time������ģʽ��һ��������blank��ģʽ
    GADI_U8  cal_shs_mode;
    // �������shutter time��vertical�Ĳ�ֵ��shutter time<=(vertical-shs_fix)
    GADI_U8  shs_fix;

#define GADI_SENSOR_UPDATE_REG_BY_SET       0 //immediately
#define GADI_SENSOR_UPDATE_REG_BY_VSYNC     1 //vblank after active area in current frame
#define GADI_SENSOR_UPDATE_REG_AT_BEGINNING 2 //at beginning in current frame
    // shutter time�Ĵ���д��ģʽ��������д�뻹����vsync�ж���д��
    GADI_U8  shs_reg_update_mode:2;
    // gain�Ĵ���д��ģʽ
    GADI_U8  agc_reg_update_mode:2;
    // vmax�Ĵ���д��ģʽ
    GADI_U8  vmax_reg_update_mode:2;
    // mirror�Ĵ���д��ģʽ
    GADI_U8  mirror_reg_update_mode:2;
    // shutter time���ӼĴ����Ƿ�ʹ�ã����sc1035
    GADI_U8  shs_additional_use;
    GADI_U8  reserved5;

    GADI_U8  shs_additional_percent;

#define GADI_MAX_SENSOR_LINE_REG_NUM     4
    GADI_VI_Sensor_HMLRegT  hmax_reg[GADI_MAX_SENSOR_LINE_REG_NUM];
    GADI_VI_Sensor_HMLRegT  vmax_reg[GADI_MAX_SENSOR_LINE_REG_NUM];
    GADI_VI_Sensor_HMLRegT  vmax_wadd_reg[GADI_MAX_SENSOR_LINE_REG_NUM];
    GADI_VI_Sensor_HMLRegT  shs_reg[GADI_MAX_SENSOR_LINE_REG_NUM];
    GADI_VI_SensorRegT      shs_additional_reg;

#define GADI_SENSOR_CAL_GAIN_MODE0       0   // set/step
#define GADI_SENSOR_CAL_GAIN_MODE1       1   // table index search by gain
#define GADI_SENSOR_CAL_GAIN_MODE2       2   // table index = (max-set)/step
#define GADI_SENSOR_CAL_GAIN_MODE3       3   // table index = (set-min)/step
    // ����gain�ķ�ʽ��Ŀǰ֧�����֣�һ���Ǽ��㷨��һ���ǲ��
    GADI_U8  cal_gain_mode;
    // again����
    GADI_U8   max_again_index;
    // gain����
    GADI_U16  max_agc_index;

    // ����ʹ��,��0
    GADI_U8  cur_again_index;
    GADI_U8  reserved6;
    GADI_U16 reserved7;

    // gain��Ϣ��0x1000000Ϊ1db
    GADI_VI_AGCInfoT            agc_info;
    // gain�Ĵ�����
    GADI_VI_SensorRegT          gain_reg[GADI_MAX_SENSOR_GAIN_REG_NUM];
#define GADI_MAX_SENSOR_GAIN_ROW_NUM    1024
    // gain��
    GADI_VI_SensorGainTableT    gain_table[GADI_MAX_SENSOR_GAIN_ROW_NUM];
    // �¿�������Ʊ�
    GADI_VI_SensorTempConInfoT  temperatue_gain_table;
#define GADI_SENSOR_HW_INFO_MAGIC_END    0x4B495043    // KIPC
    // �Ժ��checkʹ��
    GADI_U32 magic_end;

    // ��ʼ���Ĵ�����
    GADI_U32 init_length;
    GADI_VI_SensorRegT          init_reg[];
}GADI_VI_SensorDrvInfoT;

typedef struct
{
#define GADI_SENSOR_DETECT_MAGIC_START  0x47495044    // GIPD
    GADI_U32 magic_start;
#define GADI_SENSOR_DETECT_VERSION      0x20180925
    GADI_U32 version;
    // �ṹ����
    GADI_U32 length;
    GADI_INT src_id;
    // detect����
    GADI_U8  num;
    // ��ӡ������Ϣ
    GADI_U8  debug_print;
    GADI_U16 debug_hts;
    GADI_U32  reserved1;
    GADI_U32  reserved2;
//#define GADI_SENSOR_DETECT_MAX_NUM      64
    GADI_VI_SensorHWInfoT   HwInfo[];
#define GADI_SENSOR_DETECT_MAGIC_END    0x4B495044    // KIPD
//    GADI_U32 magic_end;
}GADI_VI_SensorDetectInfoT;



//*****************************************************************************
//*****************************************************************************
//** Global Data
//*****************************************************************************
//*****************************************************************************



//*****************************************************************************
//*****************************************************************************
//** API Functions
//*****************************************************************************
//*****************************************************************************



#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif



#endif /* _MKSENSOR_H_ */

