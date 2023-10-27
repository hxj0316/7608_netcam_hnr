/*!
*****************************************************************************
** FileName     : avi_common.h
**
** Description  : common file
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Create       : 2015-8-11, create
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/

#ifndef __AVI_COMMON_H__
#define __AVI_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sys/types.h>
#include <time.h>
#include <assert.h>
#include <sys/vfs.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <pthread.h>
#include <fcntl.h>
#include <semaphore.h>

#include "common.h"

typedef unsigned long long 	u64t;
typedef long long 			s64t;
typedef	unsigned int		u32t;
typedef	int					s32t;


#define __AVI_LOG(syntax,fmt,arg...) \
	do{\
		char bname[64];\
		strncpy(bname, __FILE__, sizeof(bname));\
		printf("\033[%sm[%16s:%4d]\033[0m "fmt"\r\n", syntax, basename(bname), __LINE__, ##arg);\
	}while(0)

#define PRINT_ERR_MSG(fmt, arg...)  __AVI_LOG("1;31",fmt,##arg)

#define PRINT_WARN_MSG(fmt, arg...)  __AVI_LOG("1;33",fmt,##arg)

#define PRINT_INFO_MSG(fmt, arg...)  __AVI_LOG("1;36",fmt,##arg)



#define REC_BUF_SIZE (600*1024)

#define AUDIO_ARRAY_SIZE 20000

//ONE_FRAME_NODE frame_node;

/* AVI flag, defined by aviriff.h */
#define AVIF_HASINDEX        0x00000010 //����AVI�� ������һ��index��
#define AVIF_MUSTUSEINDEX    0x00000020 //����Ӧ�ó�����Ҫʹ��index�������������ϵ�˳�����������ݵ�չ��˳�����磬�ñ�־�������ڴ���һ���༭�õ�֡�б�
#define AVIF_ISINTERLEAVED   0x00000100 //����AVI�� ���ǽ���ġ�
#define AVIF_TRUSTCKTYPE     0x00000800 // Use CKType to find key frames
#define AVIF_WASCAPTUREFILE  0x00010000 //�������ļ���һ�����ڲ���ʵʱ��Ƶ�ģ����ر�����AVI �� �������һ���ļ������˸ñ�־�����û�д���ļ�֮ǰ��Ӧ�ó���Ӧ�÷������棬��Ϊ�û����ܻ�Ը��ļ�������Ƭ����
#define AVIF_COPYRIGHTED     0x00020000 //����AVI�� �������˰�Ȩ���ݺ��������������˸ı�־��������������Ը����ݽ��п�����

/* record mode */
#define RECORD_FIXED_SIZE         0X1
#define RECORD_FIXED_DURATION     0X2
#define RECORD_DELETE_OLD_FILES   0X4

#define RECORD_IS_FIXED_SIZE(x)          (x&0x1)
#define RECORD_IS_FIXED_DURATION(x)      (x&0x2)
#define RECORD_IS_DELETE(x)              (x&0x4)

/* FOURCC */
#define MAKE_FOURCC(a,b,c,d)         (int)((a)|(b)<<8|(c)<<16|(d)<<24)

//video
#define AVI_TYPE_H264     MAKE_FOURCC('H','2','6','4')
#define AVI_TYPE_H265     MAKE_FOURCC('H','2','6','5')
#define AVI_TYPE_MPEG4    MAKE_FOURCC('D','I','V','X')
#define AVI_TYPE_MJPEG    MAKE_FOURCC('M','J','P','G')
#define AVI_TYPE_GMTAG    MAKE_FOURCC('G','M','T','G')
//audio
#define AVI_TYPE_PCM      MAKE_FOURCC('P','C','M',' ')
#define AVI_TYPE_MP3      MAKE_FOURCC('M','P','E','G')

/*¼�����*/
#define	DMS_NET_RECORD_TYPE_SCHED   0x000001  //��ʱ¼��
#define	DMS_NET_RECORD_TYPE_MOTION  0x000002  //�Ƶ����¼��
#define	DMS_NET_RECORD_TYPE_ALARM   0x000004  //����¼��
#define	DMS_NET_RECORD_TYPE_CMD     0x000008  //����¼��
#define	DMS_NET_RECORD_TYPE_MANU    0x000010  //�ֹ�¼��
#define	DMS_NET_RECORD_TYPE_SCHED_2  0x000011 //�ֹ�¼��

typedef enum
{
	ENC_TYPE_H264 , //H264
	ENC_TYPE_MPEG,      //< MPEG4
	ENC_TYPE_MJPEG,    //< Motion JPEG
	ENC_TYPE_YUV422,
	ENC_TYPE_H265,     //<H265
	ENC_TYPE_COUNT     //< do not use
}eEncType;

typedef struct tagAVI_DMS_TIME
{
    unsigned long    dwYear;
    unsigned long    dwMonth;
    unsigned long    dwDay;
    unsigned long    dwHour;
    unsigned long    dwMinute;
    unsigned long    dwSecond;
} AVI_DMS_TIME, *LPAVI_DMS_TIME;

#if 0
/* Status Code */
typedef enum AviStatusTag {
    STS_OK                =  0,
    STS_FISTFRAME_NOT_KEY =  1,
    STS_FRAME_TYPE_ERR    =  2,
    //error
    STS_OPEN_FAILED       = -1,
    STS_INVALID_INPUT     = -2,
    STS_MKDIR_ERROR       = -3,
    STS_INVALID_FORAMT    = -4,
    STS_MALLOC_FAILED     = -5,
    STS_FTRUNCATE_FAILED  = -6,
    STS_RENAME_FAILED     = -7,
    STS_POLL_FAILED       = -8,
    STS_RECORD_MODE_ERR   = -9,
    STS_WRITE_FAILED      = -10,
    STS_READ_FAILED       = -11,
    STS_SDCARD_NOT_MOUNT  = -12,
    STS_SDCARD_NO_SPACE   = -13,
    STS_INDEX_COUNT_ERR   = -14,
    STS_DOUBLE_CLOSE   = -15,

}AviStatus;
#endif

//======================================================//
/* AVI Datastruct */
typedef struct AviFileTag {
    FILE   *file;
    char   filename[128];
    long   time_s;
    long   time_e;
    long   time_b;

    int    data_offset;

    int    video_count;
    int    audio_enable;
    int    audio_bytes;
    int    samplesize;
    int    audio_type; // 0 a-law; 1 u-law; 2 pcm

    //index
    int    index_count;   //real index count
    int    *idx_array;
    int    idx_array_count;
	int	   video_fps; // add video fps
}AviFile;


typedef struct tagAviInitParam {
    int bps;
    int fps;
    int width;
    int height;
    int codec_type;

    int audio_enable;
	int a_enc_type; // 0 a-law; 1 u-law; 2- pcm
    int a_chans;    /* Audio channels, 0 for no audio */
    int a_rate;     /* Rate in Hz */
    int a_bits;     /* bits per audio sample */

    int ch_num;
    int mode;
    int size_m;
    int duration; // min
} AviInitParam;

#ifndef HAVE_AVIMAINHEADER
#define HAVE_AVIMAINHEADER
typedef struct tagAviMainHeader {
    int  fcc;
    int  cb;
    int  dwMicroSecPerFrame;
    int  dwMaxBytesPerSec;
    int  dwPaddingGranularity;
    int  dwFlags;
    int  dwTotalFrames;
    int  dwInitialFrames;
    int  dwStreams;
    int  dwSuggestedBufferSize;
    int  dwWidth;
    int  dwHeight;
    int  dwReserved[4];
} AviMainHeader;
#endif

#ifndef HAVE_AVISTREAMHEADER
#define HAVE_AVISTREAMHEADER
typedef struct tagAviStreamHeader {
     int  fcc;
     int  cb;
     int  fccType;
     int  fccHandler;
     int  dwFlags;
     short int   wPriority;
     short int   wLanguage;
     int  dwInitialFrames;
     int  dwScale;
     int  dwRate;
     int  dwStart;
     int  dwLength;
     int  dwSuggestedBufferSize;
     int  dwQuality;
     int  dwSampleSize;
     struct {
         short int left;
         short int top;
         short int right;
         short int bottom;
     }  rcFrame;
} AviStreamHeader;
#endif

#ifndef HAVE_BITMAPINFOHEADER
#define HAVE_BITMAPINFOHEADER
typedef struct tagBitMapInfoHeader{
  int   biSize;
  int   biWidth;
  int   biHeight;
  short int   biPlanes;
  short int   biBitCount;
  int   biCompression;
  int   biSizeImage;
  int   biXPelsPerMeter;
  int   biYPelsPerMeter;
  int   biClrUsed;
  int   biClrImportant;
} BitMapInfoHeader;
#endif

#ifndef HAVE_RGBQUAD
#define HAVE_RGBQUAD
typedef struct tagRGBQuad {
  unsigned char    rgbBlue;
  unsigned char    rgbGreen;
  unsigned char    rgbRed;
  unsigned char    rgbReserved;
} RGBQuad ;
#endif

#ifndef HAVE_BITMAPINFO
#define HAVE_BITMAPINFO
typedef struct tagBitmapInfo {
  BitMapInfoHeader bmiHeader;
  //RGBQuad          bmiColors[1];
} BitmapInfo;
#endif

#ifndef HAVE_WAVEFORMATEX
#define HAVE_WAVEFORMATEX
typedef struct tagWaveFormateX{
  short int  wFormatTag;
  short int  nChannels;
  int nSamplesPerSec;
  int nAvgBytesPerSec;
  short int  nBlockAlign;
  short int  wBitsPerSample;
  short int  cbSize;
} WaveFormateX;
#endif

#ifndef HAVE_AUDIOFORMATEX
#define HAVE_AUDIOFORMATEX
typedef struct tagAudioFormateX{
  short int  wFormatTag;
  short int  nChannels;
  int nSamplesPerSec;
  int nAvgBytesPerSec;
  short int  nBlockAlign;
  short int  wBitsPerSample;
  //short int  cbSize;
} AudioFormateX;
#endif


#ifndef HAVE_AVIINDEX
#define HAVE_AVIINDEX
typedef struct tagAviIndex {
  int   dwChunkId;
  int   dwFlags;
  int   dwOffset;
  int   dwSize;
} AviIndex;
#endif

typedef union AviStreamFormatTag {
    BitmapInfo    video_format;
    WaveFormateX  audio_format;
}AviStreamFormat;

typedef struct tagINDEX_NODE
{
    u32t ch_num;
    u64t start;
    u64t stop;
}INDEX_NODE;

typedef struct tagFILE_NODE
{
    char path[128];
    u64t start;
    u64t stop;
    struct tagFILE_NODE *next;
} FILE_NODE;

typedef struct tagFILE_LIST
{
    u64t start;
    u64t stop;
    u32t type;
    u32t ch_num;

    FILE_NODE *head;
    u32t len;
    u32t index;
}FILE_LIST, *LPFILE_LIST;

#define ONE_FRAME_BUF_SIZE 700000

typedef struct tagAviFrameNode {
    int frame_type; // 0: audio; 0x11: I frame, 0x10: P frame
    int offset;
    int size;       //֡��С
    unsigned int timetick;  //֡ʱ����Ϣ
    char *one_frame_buf;  //֡����
} AviFrameNode;


typedef struct tagAviPBHandle {
    AviFrameNode node;
    FILE_LIST *list;

    int no;
    int video_no;
    int audio_no;

    int gop;
	int fps;
    int video_width;
    int video_height;

    FILE    *file;
    char   file_path[128];

    int    video_count;

    //index
    int    index_count;   //real index count
    int    *idx_array;
    int    idx_array_count;

    //buffer for playback
    char *pb_buf;
    int pb_buf_size;
    int pb_buf_index;
    int pb_buf_pos;
    u32t pb_timetick_tmp;

    int pb_open_flag;
    int last_tick;
	int pb_buf_num;
} AviPBHandle;

typedef struct tagAviHeader {
    int fcc_riff;  //RIFF
    int avi_len;
    int fcc_avi;  //AVI

    int fcc_list_hdrl;  //LIST
    int hdrl_len;
    int fcc_hdrl;  //hdrl

    AviMainHeader main_header;

    int fcc_list_strl_video;  //LIST
    int size_strl_video;
    int fcc_strl_video;  //strl
    AviStreamHeader video_header;
    int fcc_strf_video;  //strf
    int size_video_format;
    //AviStreamFormat video_format;
    BitMapInfoHeader bmiHeader;

    int fcc_list_strl_audio;  //LIST
    int size_strl_audio;
    int fcc_strl_audio;  //strl
    AviStreamHeader audio_header;
    int fcc_strf_audio;  //strf
    int size_audio_format;
    AudioFormateX audio_format;

    int fcc_list_movi;
    int movi_len;
    int fcc_movi;
} AviHeader;





/**********************************************************************
�������������SD���洢�ռ䣬�����ļ���дAVIͷ��Ϣ
��ڲ�����AviFile *avi_file, AVI�ļ��Ŀ��ƾ��
          AviInitParam *param, ����Ĳ�����Ϣ
����ֵ�� 0: ��������
        <0: ���󷵻�
**********************************************************************/
extern int avi_record_open(AviFile *avi_file, AviInitParam *param);


/**********************************************************************
������������AVI�ļ���д��һ֡����
��ڲ�����AviFile *avi_file:  AVI�ļ��Ŀ��ƾ��
          AviInitParam *param, ����Ĳ�����Ϣ
          unsigned char *data:  д������ݵĵ�ַ
          int length: д������ݵĳ���
          int intra: 1,��ʾΪ�ؼ�������;0��ʾ��Ϊ�ؼ�������

����ֵ�� 0: ��������
        <0: ���󷵻�
**********************************************************************/
extern int avi_record_write(AviFile *avi_file,
                                AviInitParam *param,
                                char *data,
                                int length,
                                int intra,
                                unsigned int timetick);


/**********************************************************************
����������������AVI�ļ�Ϊ���ո�ʽ�����ر��ļ����ͷ��ڴ�
��ڲ�����AviFile *avi_file:  AVI�ļ��Ŀ��ƾ��
����ֵ�� 0: ��������
        <0: ���󷵻�
**********************************************************************/
extern  int avi_record_close(AviFile *avi_file);


/**********************************************************************
�طŲ��ֽӿں���ʹ��������
1������ avi_pb_open ��Ҫ���ŵ�avi�ļ�,�ļ���Ϊfile_name��
2������ avi_pb_seek_by_percent ���� avi_pb_seek_by_time ��λ������ʼλ�ã�
   �����ʹ����2����������Ĭ�ϴ�ͷ��ʼ���š�
3������ set_frame_info ȥ���� avi֡���ݿ��ƾ�� AviFrameInfo
4������ avi_pb_set_speed ȥ���ò����ٶȣ���ʹ���򲥷��ٶ�Ĭ��Ϊ1���١�
5��ѭ������ avi_pb_get_frame ȥ�õ�ÿһ֡���ݡ�����ֵΪ1003�򲥷ŵ���ĩβ��
6������ avi_pb_close ������
ע��: ������һ���߳̽��лط�ʱ���Իط��ٶȵĲ���(avi_pb_set_speed)��
����һ���߳���(һ�������߳�)��2���̶߳� avi_pb_speed �Ĳ����ǻ���ġ�
**********************************************************************/
/**********************************************************************
����������ͨ����AVI�ļ���Ϣ���õ����Ŀ��ƾ���Ͳ�����Ϣ
��ڲ�����char *file_name: Ҫ�򿪵��ļ�·����
          AviFile *avi_file: avi�ļ��Ŀ��ƾ��
          AviInitParam *param: avi�ļ��еĲ�����Ϣ
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int avi_pb_open(char *file_path, AviPBHandle *pPBHandle);

/**********************************************************************
����������ͨ��AVI�ļ��Ŀ��ƾ���Ͳ�����Ϣ, ������ðٷֱ�λ���ǵڼ�֡
          ���ݣ������µ� AviFrameInfo �� no ������
��ڲ�����AviFile *avi_file: avi�ļ��Ŀ��ƾ��
          AviInitParam *param: avi�ļ��еĲ�����Ϣ
          int percent: �ٷֱȣ���25%��percent = 25
          AviFrameInfo *p_frame: avi�ļ���֡���ݵĿ��ƾ��
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int avi_pb_seek_by_percent(AviFile *avi_file, AviInitParam *param, int percent, AviPBHandle *pPBHandle);

/**********************************************************************
����������ͨ��AVI�ļ��Ŀ��ƾ���Ͳ�����Ϣ, ������ö�λʱ���ǵڼ�֡
          ���ݣ������µ� AviFrameInfo �� no ������
��ڲ�����AviFile *avi_file: avi�ļ��Ŀ��ƾ��
          AviInitParam *param: avi�ļ��еĲ�����Ϣ
          char *seek_time: ͨ��ʱ���ڽ������ж�λ
          AviFrameInfo *p_frame: avi�ļ���֡���ݵĿ��ƾ��
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int avi_pb_seek_by_time(AviFile *avi_file, AviInitParam *param, char *seek_time, AviPBHandle *pPBHandle);

/**********************************************************************
����������ͨ��AVI�ļ��Ŀ��ƾ���Ͳ�����Ϣ��������֡���ݵĿ��ƾ��
��ڲ�����AviFile *avi_file: avi�ļ��Ŀ��ƾ��
          AviInitParam *param: avi�ļ��еĲ�����Ϣ
          AviFrameInfo *p_frame: avi�ļ���֡���ݵĿ��ƾ��
����ֵ��  ��
**********************************************************************/
//extern void set_frame_info(AviFile *avi_file, AviInitParam *param, AviFrameInfo *p_frame);

/**********************************************************************
������������ӡ��avi�ļ���֡�������ڵ�avi�ļ��е���Ϣ
��ڲ�����AviPBHandle *pPBHandle: avi�ļ���֡���ݵĿ��ƾ��
����ֵ��  ��
**********************************************************************/
extern void print_pb_handle(AviPBHandle *pPBHandle);

/**********************************************************************
������������ӡ��avi�ļ���֡���ݵ���Ϣ
��ڲ�����AviPBHandle *pPBHandle: avi�ļ���֡���ݵĿ��ƾ��
����ֵ��  ��
**********************************************************************/
extern void print_frame_node(AviPBHandle *pPBHandle);

extern AviPBHandle *create_pb_handle(void);

/**********************************************************************
�������������� n1 - n2 ��ֵ
��ڲ�����AviPBHandle *pPBHandle
����ֵ��  �����ļ�����ʱ������λΪ��
**********************************************************************/
extern int cal_pb_all_time(AviPBHandle *pPBHandle);


/**********************************************************************
�������������� n1 - n2 ��ֵ
��ڲ�����AviPBHandle *pPBHandle,AVI_DMS_TIME *ptime
����ֵ��  ����ָ��ʱ�䵽�ļ���ʼʱ��ĳ���
**********************************************************************/
int cal_pb_seek_time(AviPBHandle *pPBHandle,AVI_DMS_TIME *ptime);


/**********************************************************************
����������ͨ��AVI�ļ��Ŀ��ƾ���Ͳ�����Ϣ, ������ö�λʱ���ǵڼ�֡
          ���ݣ������µ� AviFrameInfo �� no ������
��ڲ�����AviFrameInfo *p_frame: avi�ļ���֡���ݵĿ��ƾ��
          char *pBuff: ��û�����λ��ָ���������
          int *pSize: �õ���֡���ݵĴ�С
����ֵ��  <0: ����
           0: ����
          DMS_NET_NOMOREFILE: ���Ž���(DMS_NET_NOMOREFILEֵΪ1003)
**********************************************************************/
extern int avi_pb_slide_to_next_file(AviPBHandle *pPBHandle);

extern int avi_pb_get_frame(AviPBHandle *pPBHandle, char *pBuff, int *pSize);

/**********************************************************************
��������������avi�ļ����ŵ��ٶȣ�Ŀǰֻ֧�ֿ��
��ڲ�����int speed: �����ٶȣ��� 1,2,4,8,16,32
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int avi_pb_set_speed(int speed);

/**********************************************************************
�����������ر�֡���ݵĿ��ƾ��
��ڲ�����AviFrameInfo *p_frame: avi�ļ���һ֡���ݵĿ��ƾ��
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int avi_pb_close(AviPBHandle *pPBHandle);

/**********************************************************************
������������ÿ��¼������¼�������ļ���
��ڲ�����u32t type: ���ն�������
          GK_NET_RECORD_TYPE_SCHED   ��ʱ¼��
          GK_NET_RECORD_TYPE_MOTION  �Ƶ����¼��
          GK_NET_RECORD_TYPE_ALARM   �ֶ�����¼��
          GK_NET_RECORD_TYPE_MANU    �ֹ�¼��
          u32t ch_num: ͨ����
          u64t start: ��ʼʱ��
          u64t stop:  ����ʱ��
����ֵ��  <0: ����
           0: ����
**********************************************************************/
extern int write_manage_info(u32t type, u32t ch_num, u64t start, u64t stop);

/**********************************************************************
��������������6����ֻ��԰��նԽ�
**********************************************************************/
extern int avi_pb_start(void);
extern int avi_pb_restart(AviPBHandle *pPBHandle);
extern int avi_pb_pause(void);
extern int avi_pb_get_pos(AviPBHandle *pPBHandle);
extern int avi_pb_set_pos(AviPBHandle *pPBHandle, int *pTimePos);
extern int avi_pb_seek(AviPBHandle *pPBHandle, int seek_no);
extern int avi_pb_get_time(AviPBHandle *pPBHandle,  struct tm *p_nowtime);
extern int avi_get_frame_loop(AviPBHandle *pPBHandle, char *p_buff, int *p_size);
extern int avi_get_frame_by_no(AviPBHandle *pPBHandle, char *p_buff, int *p_size);

/**********************************************************************
�����������޸�sd�������е�δ¼��ɵ�avi�ļ�
��ڲ�������
����ֵ��  ��
**********************************************************************/
extern void fix_all_avi_file(void);


/**********************************************************************
�������������һ���ļ��Ƿ���Ҫ�޸�
��ڲ�������
����ֵ��  ��
**********************************************************************/
int avi_check_file_index(char *file_path);

/**********************************************************************
�������������ݲ�����ӡ��������ʾ.
��ڲ�����int ret: avi_record_open,avi_record_write,avi_record_close
                   �ķ���ֵ��Ϊ����
����ֵ����
**********************************************************************/
//extern  void avi_print_err(int ret);


// ��¼��cache ����ˢ��SD����
int avi_record_write_cache(AviFile *avi_file);

//���¼���Ƿ�ֹͣ
int avi_reocrd_check_file_stoped();

#ifdef __cplusplus
}
#endif

#endif /* __GRD_AVI_TYPEDEF_H__ */

