#ifndef __DAHUA_SESSION_H__
#define __DAHUA_SESSION_H__

#include <fcntl.h>
#include <netdb.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <semaphore.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <linux/unistd.h>
#include <errno.h>
#include <sys/syscall.h>

#include "dahua_common.h"


/*************************************************
********************Э�鱨������� ****************
*************************************************/
//����������
#define CMD_ZHINUO_LOG_ON 0xd2    //��¼
#define CMD_DAHUA_LOG_ON 0xa0    //��¼
#define CMD_STATUS_SEARCH 0xa1    //��ѯ����������״̬
#define CMD_DEV_EVENT_SEARCH 0x68    //��ѯ�豸�¼�
#define CMD_MEDIA_REQUEST 0x11    //ý����������
#define CMD_MEDIA_CAPACITY_SEARCH 0x83    //��ѯ�豸ý��������Ϣ
#define CMD_CONFIG_SEARCH 0xa3    //��ѯ���ò���
#define CMD_CHANNEL_NAME_SEARCH 0xa8    //��ѯͨ������
#define CMD_SYSTEM_INFO_SEARCH 0xa4    //��ѯϵͳ��Ϣ
#define CMD_CONNECT_REQUEST 0xf1    //����������
#define CMD_I_FRAME_REQUEST 0x80    //ǿ��I֡
#define CMD_DEV_CONTROL 0x60    //�豸����
#define CMD_SET_CONFIG 0xc1    //�޸Ĳ�������
#define CMD_TIME_MANAGE 0x24    //ʱ�����
#define CMD_SET_CHANNEL_NAME 0xC6    //�޸�ͨ������
#define CMD_PTZ_CONTROL 0x12    //��̨����
#define CMD_TALK_REQUEST 0x1d    //�Խ�����
#define CMD_TALK 0xc0    //�Խ�
#define CMD_TALK_CONTROL 0x1e    //�Խ�����
#define CMD_RECORD_SEARCH 0xa5    //¼������
#define CMD_RECORD_PLAYBACK 0xc2    //¼��ط�
#define CMD_RECORD_DOWNLOAD 0xbb    //¼������
#define CMD_DEV_SEARCH 0xa3    //�豸����
#define CMD_INFO_NEW 0xf6    //δ֪����
#define CMD_CONTROL_CONNECT 0xf4    //��������


//�ظ�����������
#define ACK_LOG_ON 0xb0    //��¼
#define ACK_STATUS_SEARCH 0xb1    //��ѯ����������״̬
#define ACK_DEV_EVENT_SEARCH 0x69    //��ѯ�豸�¼�
#define ACK_MEDIA_REQUEST 0xbc    //ý����������
#define ACK_MEDIA_CAPACITY_SEARCH 0x83    //��ѯ�豸ý��������Ϣ
#define ACK_CONFIG_SEARCH 0xb3    //��ѯ���ò���
#define ACK_CHANNEL_NAME_SEARCH 0xb8   //��ѯͨ������
#define ACK_SYSTEM_INFO_SEARCH 0xb4    //��ѯϵͳ��Ϣ
#define ACK_CONNECT_REQUEST 0xf1    //����������
#define ACK_I_FRAME_REQUEST 0x80    //ǿ��I֡
#define ACK_DEV_CONTROL 0x60    //�豸����
#define ACK_SET_CONFIG 0xC1    //�޸Ĳ�������
#define ACK_TIME_MANAGE 0x24    //ʱ�����
#define ACK_SET_CHANNEL_NAME 0xb8   //�޸�ͨ������
#define ACK_PTZ_CONTROL 0x12    //��̨����
#define ACK_TALK_REQUEST 0xc0    //�Խ�����
#define ACK_TALK 0x1d    //�Խ�
#define ACK_TALK_CONTROL 0x1e    //�Խ�����
#define ACK_RECORD_SEARCH 0xb6   //¼������
#define ACK_RECORD_PLAYBACK 0xc2    //¼��ط�
#define ACK_RECORD_DOWNLOAD 0xbb    //¼������
#define ACK_DEV_SEARCH 0xb3    //�豸����
#define ACK_UNKOWN 0xf6    //δ֪����
#define ACK_CONTROL_CONNECT 0xf4    //��������



#define DMS_PTZ_CMD_UP              1
#define DMS_PTZ_CMD_DOWN            2
#define DMS_PTZ_CMD_LEFT            3
#define DMS_PTZ_CMD_RIGHT           4
#define DMS_PTZ_CMD_UP_LEFT			31
#define DMS_PTZ_CMD_UP_RIGHT		32
#define DMS_PTZ_CMD_DOWN_LEFT		33
#define DMS_PTZ_CMD_DOWN_RIGHT		34

#define DMS_PTZ_CMD_FOCUS_SUB       5       //Focus Far
#define DMS_PTZ_CMD_FOCUS_ADD       6       //Focus Near
#define DMS_PTZ_CMD_ZOOM_SUB        7       //Zoom Wide
#define DMS_PTZ_CMD_ZOOM_ADD        8       //Zoom Tele
#define DMS_PTZ_CMD_IRIS_SUB        9       //Iris Close
#define DMS_PTZ_CMD_IRIS_ADD        10      //Iris Open
#define DMS_PTZ_CMD_STOP            11
#define DMS_PTZ_CMD_PRESET          12      //Ԥ��
#define DMS_PTZ_CMD_CALL            13      //����

#define DMS_PTZ_CMD_AUTO_STRAT      14      //�Զ�      //STRAT shoule be START
#define DMS_PTZ_CMD_AUTO_STOP       15
#define DMS_PTZ_CMD_LIGHT_OPEN      16      //�ƹ�
#define DMS_PTZ_CMD_LIGHT_CLOSE     17
#define DMS_PTZ_CMD_BRUSH_START     18      //��ˢ
#define DMS_PTZ_CMD_BRUSH_STOP      19
#define DMS_PTZ_CMD_TRACK_START     20      //�켣
#define DMS_PTZ_CMD_TRACK_STOP      21
#define DMS_PTZ_CMD_TRACK_RUN       22
#define DMS_PTZ_CMD_PRESET_CLS      23      //���Ԥ�õ�

#define DMS_PTZ_CMD_ADD_POS_CRU		24		/* ��Ԥ�õ����Ѳ������ */
#define DMS_PTZ_CMD_DEL_POS_CRU		25		/* ��Ѳ�����Ѳ��������ɾ�� */
#define DMS_PTZ_CMD_DEL_PRE_CRU		26		/* ��Ԥ�õ��Ѳ��������ɾ�� */
#define DMS_PTZ_CMD_MOD_POINT_CRU	27		/* �޸�Ѳ����*/
#define DMS_PTZ_CMD_START_CRU		28		/* ��ʼѲ�� */
#define DMS_PTZ_CMD_STOP_CRU		29		/* ֹͣѲ�� */
#define DMS_PTZ_CMD_CRU_STATUS		30		/* Ѳ��״̬ */


////////////////////////////////////

//�����궨��
#define DAHUA_MAX_LINK_NUM  10
//#define	DVRIP_VERSION	5
#define	DVRIP_VERSION	6

#define RE_CONNECT_TIME                          (10)    //�����¼ʧ�ܵ�������ʱ��
#define BUFLEN (10 * 1024)  //�µ�nvr�������ܴ�
#define SEND_TIMEOUT                              (15) //socket�������ݵĳ�ʱʱ��
#define USER_NAME "admin"
#define PASSWD "admin"
#define MAX_VIDEO_FRAME_SIZE                     (524 * 1024)   //ʵʱ��Ƶ�����ȡ��֡������ֽ���
#define MAX_SEND_BUFF_SIZE                       (524 * 1024)   //send�������buf�Ĵ�С
 
#define ZHINUO_ONE_PACKAGE_SIZE    (40*1024)  
//#define DAHUA_ONE_PACKAGE_SIZE    (20*1024)  //����������ó���40k��Ȼ�󻪵���Ƶ������
#define DAHUA_ONE_PACKAGE_SIZE    (32*1024)  //�µ�NVR��32K

#define AUDIO_SIZE_PRE_FRAME    (320) //ÿ����Ƶ֡G711A�������ݵ�ʵ�ʴ�С
//#define DEV_NAME "IPC-IPVM3150F" //"IPC_3507_720P"
//#define DEV_NAME "IPC"
//#define DEV_NAME "TP5"
#define DEV_NAME "IPC-HDW1025C"
//#define DEV_NAME "IPC"


#define DEV_DEVICE_NAME "IPC"


//#define DEV_DEVICE "IPC"
//#define DEV_DEVICE "IPC-HDW1025C"

//#define DEV_SERIAL "3L04587PAG02470"
//#define DEV_SERIAL "1K026D1PAA00928"
#define DEV_SERIAL "000000000000000"

#define DEV_VER "2.621.0000000.8.R"
#define DEV_WEB_VER "3.2.1.514808"


#define ZHINUO_RCV_PORT 6060
#define ZHINUO_SEND_PORT 6061
#define IFNAME "eth0"
#define DAHUA_RCV_PORT 5050
#define DAHUA_SEND_PORT 5050
#define DAHUA_TCP_LISTEN_PORT 37777
#define ZHINUO_NVR    45
#define DAHUA_NVR    46

//#define DEV_FUNCTION_SEARCH "FTP:1:Record,Snap&&SMTP:1:AlarmText,AlarmSnap&&NTP:2:AdjustSysTime&&VideoCover:1:MutiCover&&AutoRegister:1:Login&&AutoMaintain:1:Reboot,DeleteFiles,ShutDown&&UPNP:1:SearchDevice&&DHCP:1:RequestIP&&STORE POSITION:1:FTP&&DefaultQuery:1:DQuery&&ACFControl:1:ACF&&ENCODE OPTION:1:AssiCompression&&DavinciModule:1:WorkSheetCFGApart,StandardGOP&&Dahua.a4.9:1:Login&&Dahua.Device.Record.General:1:General&&IPV6:1:IPV6Config&&Log:1:PageForPageLog&&QueryURL:1:CONFIG&&DriverTypeInfo:1:DriverType&&ProtocolFramework:1:V3_1"
#define DEV_FUNCTION_SEARCH "FTP:1:Record,Snap&&SMTP:1:AlarmText,AlarmSnap,HealthMail&&NTP:2:AdjustSysTime&&VideoCover:1:MutiCover&&AutoRegister:1:Login&&DriverTypeInfo:1:DriverType&&AutoMaintain:1:Reboot,DeleteFiles,ShutDown&&UPNP:1:SearchDevice&&DHCP:1:RequestIP&&STORE POSITION:1:FTP&&DefaultQuery:1:DQuery&&ImportantRecID:1:RECID&&SNMP:1:SNMPConfig&&ISCSI:1:ISCSIConfig&&PlayBackSpeedControl:1:SpeedUp&&ACFControl:1:ACF&&IPV6:1:IPV6Config&&DavinciModule:1:WorkSheetCFGApart,StandardGOP&&Dahua.a4.9:1:Login&&Dahua.Device.Record.General:1:General&&Log:1:PageForPageLog&&QueryURL:1:CONFIG&&SearchRecord:1:V3&&BackupVideoExtFormat:1:DAV,ASF&&Dahua_Config:1:Json,V3,MotionDetect_F6&&ProtocolFramework:1:V3_1&&DynamicMultiConnecting:1:TalkSnd&&Dahua.MultiNet:1:Config&&CommSniffer:1:CommATM&&NetSniffer:1:NetSniffer&&Net:1:RSA,A3"


//�ṹ����غ�
#define MAX_ENC_CHIP_NR 32
#define MD_REGION_ROW 32
#define N_SYS_CH 16
#define EXTRATYPES 3
#define N_COLOR_SECTION 2


//��չ֡ͷ������
#define EXPAND_PICTURE_SIZE 0x80
#define EXPAND_PLAY_BACK_TYPE 0x81
#define EXPAND_AUDIO_FORMAT 0x83
#define EXPAND_DATA_CHECK 0x88

//֡����
#define 	FRAME_TYPE_I  0xFD  
#define 	FRAME_TYPE_P  0XFC
#define 	FRAME_TYPE_B  0XFE
#define 	AUDIO_TYPE  0XF0

typedef struct __prevideo_capture{
    unsigned char iCifCaps;     // ��С
    char iFrameCaps;    // ֡��
    unsigned char reserved[30]; // �����ֽ�
}Prevideo_Capture;

typedef enum capture_size_t {
    CAPTURE_SIZE_D1,        ///< 704*576(PAL)   704*480(NTSC)
    CAPTURE_SIZE_HD1,       ///< 352*576(PAL)   352*480(NTSC)
    CAPTURE_SIZE_BCIF,      ///< 704*288(PAL)   704*240(NTSC)
    CAPTURE_SIZE_CIF,       ///< 352*288(PAL)   352*240(NTSC)
    CAPTURE_SIZE_QCIF,      ///< 176*144(PAL)   176*120(NTSC)
    CAPTURE_SIZE_VGA,       ///< 640*480(PAL)   640*480(NTSC)
    CAPTURE_SIZE_QVGA,      ///< 320*240(PAL)   320*240(NTSC)
    CAPTURE_SIZE_SVCD,      ///< 480*480(PAL)   480*480(NTSC)
    CAPTURE_SIZE_QQVGA,     ///< 160*128(PAL)   160*128(NTSC)
    CAPTURE_SIZE_SVGA,      ///< 800*592
    CAPTURE_SIZE_XVGA,      ///< 1024*768
    CAPTURE_SIZE_WXGA,      ///< 1280*800
    CAPTURE_SIZE_SXGA,      ///< 1280*1024 
    CAPTURE_SIZE_WSXGA,     ///< 1600*1024 
    CAPTURE_SIZE_UXGA,      ///< 1600*1200
    CAPTURE_SIZE_WUXGA,     ///< 1920*1200
    CAPTURE_SIZE_LTF,       ///< 240*192(PAL);����ƽ̨ʹ��
    CAPTURE_SIZE_720p ,     ///< 1280*720
    CAPTURE_SIZE_1080p,     ///< 1920*1080
    CAPTURE_SIZE_1_3M,      ////<1280*960
    CAPTURE_SIZE_NR         ///< ö�ٵ�ͼ�δ�С�������Ŀ��
}CAPTURE_SIZE_T;

typedef struct tagZhiNuo_Dev_Search
{
    unsigned char Version[8]; // 8�ֽڵİ汾��Ϣ
    char HostName[16]; // ������
    unsigned long HostIP; // IP ��ַ
    unsigned long Submask; // ��������
    unsigned long GateWayIP; // ����IP
    unsigned long DNSIP; // DNS IP 40���ֽ�λ��

    // �ⲿ�ӿ�
    unsigned long	AlarmServerIP; // ��������IP
    unsigned short  AlarmServerPort; // �������Ķ˿�
    unsigned long	SMTPServerIP; // SMTP server IP
    unsigned short  SMTPServerPort; // SMTP server port
    unsigned short  resPort1; 
    unsigned long	LogServerIP; // Log server IP
    unsigned short  LogServerPort; // Log server port

    // ��������˿�
    unsigned short  HttpPort; // HTTP����˿ں�
    unsigned short  HttpsPort; // HTTPS����˿ں�
    unsigned short  TCPPort; // TCP �����˿�   68���ֽ�λ��
    unsigned short  TCPMaxConn; // TCP ���������
    unsigned short  SSLPort; // SSL �����˿� 72
    unsigned short  UDPPort; // UDP �����˿� 76
    unsigned short  resPort2; // 
    unsigned long	McastIP; // �鲥IP  80
    unsigned short  McastPort; // �鲥�˿�

    // ����
    unsigned char  MonMode; // ����Э��0-TCP, 1-UDP, 2-MCAST //��ȷ��-TCP
    unsigned char  PlayMode; // �ط�Э��0-TCP, 1-UDP, 2-MCAST//��ȷ��-TCP 84
    unsigned char  AlmSvrStat; // ��������״̬0-�ر�, 1-�� 88
}ZhiNuo_Dev_Search;


//ʱ�����ڽṹ
typedef struct tagZhiNuo_DateTime					                        
{
    DWORD second	:6;		//	��	0-59
    DWORD minute	:6;		//	��	0-59
    DWORD hour		:5;		//	ʱ	0-23
    DWORD day		:5;		//	��	1-31
    DWORD month		:4;		//	��	1-12
    DWORD year		:6;		//	��	2000-2063	
}ZhiNuo_DateTime;

//��չ֡ͷͼ��ߴ�
typedef struct tagZhiNuo_Expand_Picture_Size
{
    unsigned char ch_cmd;
    unsigned char ch_coding_type; //ZhiNuo_coding_type_e
    unsigned char ch_width;
    unsigned char ch_high;    
}ZhiNuo_Expand_Picture_Size;

//��չ֡ͷ�ط����͡�
typedef struct tagZhiNuo_Expand_PlayBack_Type
{
    unsigned char ch_cmd;
    unsigned char ch_reserve;
    unsigned char ch_video_type; //ZhiNuo_video_type_e
    unsigned char ch_frame_rate;
}ZhiNuo_Expand_PlayBack_Type;

typedef struct
{
    unsigned char ch_cmd;
    unsigned char ch_reserve;
    unsigned char ch_video_type; //ZhiNuo_video_type_e
    unsigned char ch_frame_rate;
    unsigned char cmd2[8];
}ZhiNuo_Expand_PlayBack_Type2;


//��չ֡ͷ��Ƶ��ʽ��
typedef struct tagZhiNuo_Expand_Audio_Format
{
    unsigned char ch_cmd;
    unsigned char ch_audio_channel; //ZhiNuo_audio_channel_e
    unsigned char ch_audio_type;   //ZhiNuo_audio_type_e
    unsigned char ch_sample_frequency; //ZhiNuo_sample_sequence_e
}ZhiNuo_Expand_Audio_Format;

#pragma pack(1)
//��չ֡ͷ����У�顣
typedef struct tagZhiNuo_Expand_Data_Check
{
    unsigned char ch_cmd;
    unsigned int  n_check_result;
    unsigned short s_reserve;
    unsigned char ch_check_type; //ZhiNuo_data_check_type_e
}ZhiNuo_Expand_Data_Check;
#pragma pack()


//����Ƶ����ͷ�ṹ�� 24���ֽ�
typedef struct tagZhiNuo_Media_Frame_Head
{
    unsigned char sz_tag[4];
    unsigned char ch_media_type;
    unsigned char ch_child_type;
    unsigned char ch_channel_num;
    unsigned char ch_child_sequence;
    unsigned int  n_frame_sequence;
    unsigned long   n_frame_len;
    ZhiNuo_DateTime t_date;
    unsigned short s_timestamp;
    unsigned char ch_expand_len;
    unsigned char ch_checksum;    
}ZhiNuo_Media_Frame_Head;

typedef struct tagZhiNuo_Media_Frame_Tail
{
    unsigned char sz_tag[4];
    unsigned long  unl_data_len;   
}ZhiNuo_Media_Frame_Tail;

//¼����������ظ��ṹ
typedef struct tagZhiNuo_sVideoRecordDownloadInfo
{
    DWORD  begin_seek;
    DWORD  end_seek;
    DWORD  record_id;  
}ZhiNuo_sVideoRecordDownloadInfo;

typedef struct 
{
    char				szIP[16];		// IP
    int					nPort;							// �˿�
    char				szSubmask[16];	// ��������
    char				szGateway[16];	// ����
    char				szMac[40];			// MAC��ַ
    char				szDeviceType[32];	// �豸����
    BYTE				bReserved[32];					// �����ֽ�
} ZhiNuo_DEVICE_NET_INFO;

//�����ͷ�ṹ��
typedef struct dvrip
{
    unsigned char	cmd;			/* command  */
    unsigned char	dvrip_r0;		/* reserved */
    unsigned char	dvrip_r1;		/* reserved */
    unsigned char	dvrip_hl:4,		/* header length */
                    dvrip_v :4;		/* version */
    unsigned int	dvrip_extlen;	/* ext data length */
    unsigned char	dvrip_p[24];	/* inter params */
}DVRIP;

typedef union
{
    struct dvrip	dvrip; /* struct def */
    unsigned char	c[32]; /* 1 byte def */
    unsigned short	s[16]; /* 2 byte def */
    unsigned int	l[8];  /* 4 byte def */
}DVRIP_HEAD_T;

#define DVRIP_HEAD_T_SIZE sizeof(DVRIP)
#define ZERO_DVRIP_HEAD_T(X) memset((X), 0, DVRIP_HEAD_T_SIZE);	\
    (X)->dvrip.dvrip_hl = DVRIP_HEAD_T_SIZE/4;\
(X)->dvrip.dvrip_v = DVRIP_VERSION;

//��������
typedef enum
{
    ONE_FRAME,  //0:����ʱֻ��һ��(֡)
    MIX_FRAME,  //1:����ʱ������֯
    TWO_FRAME,	 //2:����ʱ������
}ZhiNuo_coding_type_e;

//��Ƶ֡����
typedef enum
{
    MPEG4 = 1,  
    H264,
    MPEG4_LB,	
    H264_GBE,
    JPEG,
    JPEG2000,
    AVS,
}ZhiNuo_video_type_e;

//��Ƶ֡����
typedef enum
{
    PCM8 = 7,  
    G729 ,
    IMA_ADPCM,	
    G711U,
    G721,
    PCM8_VWIS ,
    MS_ADPCM,
    G711A,
    PCM16,
}ZhiNuo_audio_type_e;

//��Ƶ��������
typedef enum
{
    SINGLE_CHANNEL = 1, //������
    DOUBLE_CHANNEL,//˫����     
}ZhiNuo_audio_channel_e;

//��Ƶ������
typedef enum
{
    SAMPLE_FREQ_4000 = 1,  
    SAMPLE_FREQ_8000,
    SAMPLE_FREQ_11025,	
    SAMPLE_FREQ_16000,
    SAMPLE_FREQ_20000,
    SAMPLE_FREQ_22050 ,
    SAMPLE_FREQ_32000,
    SAMPLE_FREQ_44100,
    SAMPLE_FREQ_48000,
}ZhiNuo_sample_sequence_e;

typedef enum
{
    check_type_add,  
    check_type_xor, 
    check_type_CRC32, 	
}ZhiNuo_data_check_type_e;

typedef enum 
{
    ZhiNuo_PTZ_CMD_UP = 0x00, // ��
    ZhiNuo_PTZ_CMD_DOWN, // ��
    ZhiNuo_PTZ_CMD_LEFT, // ��
    ZhiNuo_PTZ_CMD_RIGHT, // ��
    ZhiNuo_PTZ_CMD_ZOOM_ADD, // �䱶��     
    ZhiNuo_PTZ_CMD_ZOOM_SUB, // �䱶С 
    ZhiNuo_PTZ_CMD_FOCUS_ADD = 0x07,    // �佹��    
    ZhiNuo_PTZ_CMD_FOCUS_SUB, // �佹С
    ZhiNuo_PTZ_CMD_IRIS_ADD, // ��Ȧ+
    ZhiNuo_PTZ_CMD_IRIS_SUB, // ��Ȧ-
    ZhiNuo_PTZ_CMD_LIGHT_BRUSH = 0x0e,//   �ƹ� 
    ZhiNuo_PTZ_CMD_AUTO = 0x0f, // Ѳ�� 
    ZhiNuo_PTZ_CMD_PRESET_GO = 0X10,//����Ԥ�õ�   
    ZhiNuo_PTZ_CMD_PRESET_SET = 0X11,//����Ԥ�õ�  
    ZhiNuo_PTZ_CMD_PRESET_CLR = 0X12,//���Ԥ�õ�  
    ZhiNuo_PTZ_CMD_LEFT_UP = 0x20, // ����
    ZhiNuo_PTZ_CMD_RIGHT_UP, // ����
    ZhiNuo_PTZ_CMD_LEFT_DOWN, // ����
    ZhiNuo_PTZ_CMD_RIGHT_DOWN, // ���� 
    ZhiNuo_PTZ_CMD_STOP = 0xff, //ֹͣ
    ZhiNuo_PTZ_CMD_ADD_POS_CRU = 0x24,   
    ZhiNuo_PTZ_CMD_DEL_POS_CRU, 
    ZhiNuo_PTZ_CMD_CLR_POS_CRU,     
} ZhiNuo_PT_PTZCMD_E;

/// ����ѹ����ʽ����
enum capture_comp_t {
    CAPTURE_COMP_DIVX_MPEG4,	///< DIVX MPEG4��
    CAPTURE_COMP_MS_MPEG4,		///< MS MPEG4��
    CAPTURE_COMP_MPEG2,			///< MPEG2��
    CAPTURE_COMP_MPEG1,			///< MPEG1��
    CAPTURE_COMP_H263,			///< H.263
    CAPTURE_COMP_MJPG,			///< MJPG
    CAPTURE_COMP_FCC_MPEG4,		///< FCC MPEG4
    CAPTURE_COMP_H264,			///< H.264
    CAPTURE_COMP_NR				///< ö�ٵ�ѹ����׼��Ŀ��
};

enum rec_type
{
    REC_TYP_TIM = 0,		/*��ʱ¼��*/
    REC_TYP_MTD,					
    REC_TYP_ALM,
    REC_TYP_NUM,
};

enum capture_brc_t {
    CAPTURE_BRC_CBR,
    CAPTURE_BRC_VBR,
    CAPTURE_BRC_MBR,
    CAPTURE_BRC_NR,
};

typedef struct tagAv_Connect_Info
{
    int n_socket; 					
    int n_flag;						
}Av_Connect_Info;

typedef struct __frame_caps
{
    DWORD Compression; //!ѹ��ģʽ,  ֧�ֵ�ѹ����׼�����룬λ��Ŷ�Ӧö������						
    //!capture_comp_t��ÿһ��ֵ��
    /// ��1��ʾ֧�ָ�ö��ֵ��Ӧ�����ԣ���0��ʾ��֧�֡�
    DWORD ImageSize;	//!�ֱ���, /// ֧�ֵ�ͼ���С���͵����룬λ��Ŷ�Ӧö������						
    //!capture_size_t��ÿһ��ֵ��
    ///! ��1��ʾ֧�ָ�ö��ֵ��Ӧ�����ԣ���0��ʾ��֧��
}FRAME_CAPS;

typedef struct tagCAPTURE_EXT_STREAM
{
    DWORD ExtraStream;							//��channel_t��λ����ʾ֧�ֵĹ���
    DWORD CaptureSizeMask[64];	//ÿһ��ֵ��ʾ��Ӧ�ֱ���֧�ֵĸ���������
}CAPTURE_EXT_STREAM, *pCAPTURE_EXT_STREAM;	

typedef struct {	// ��ʱʱ��
    BYTE	StartHour;
    BYTE	StartMin;
    BYTE	StartSec;
    BYTE	EndHour;
    BYTE    EndMin;
    BYTE	EndSec;
    BYTE	State;			//�ڶ�λ�Ƕ�ʱ������λ�Ƕ�̬��⣬����λ�Ǳ���
    BYTE	Reserve;		/*!< Reserve�Ѿ���ʹ�ã����ĵĻ���֪ͨ¼��ģ�� */
} TSECT;

//! ��ɫ��������
typedef struct 
{
    TSECT 	Sector;				/*!< ��Ӧ��ʱ���*/
    BYTE	Brightness;			/*!< ����	0-100		*/
    BYTE	Contrast;			/*!< �Աȶ�	0-100		*/
    BYTE	Saturation;			/*!< ���Ͷ�	0-100		*/
    BYTE	Hue;				/*!< ɫ��	0-100		*/
    BYTE	Gain;				/*!< ����	0-100		*/
    BYTE	Reserve[3];
}COLOR_PARAM;

//! ��ɫ�ṹ
typedef struct  
{
    BYTE ColorVersion[8];	
    COLOR_PARAM Color[N_COLOR_SECTION];
}CONFIG_COLOR_OLD;

//! ����ѡ��
typedef struct  
{
    BYTE ImageSize;   /*!< �ֱ��� ����ö��capture_size_t(DVRAPI.H) */
    BYTE BitRateControl;  /*!< �������� ����ö��capture_brc_t(DVRAPI.H) */
    BYTE ImgQlty;   /*!< �����Ļ��� ����1-6  */
    BYTE Frames;    /*!< ֡�ʡ�����N��1-6,P��1-5 */
    BYTE AVEnable;   /*!< ����Ƶʹ�� 1λΪ��Ƶ��2λΪ��Ƶ��ONΪ�򿪣�OFFΪ�ر� */
    BYTE IFrameInterval;  /*!< I֡���֡��������������I֮֡���P֡������0-149, 255��ʾ�˹��ܲ�֧������*/
    WORD usBitRate;
}ENCODE_OPTION;

//! ����ṹ
typedef struct  
{
    DWORD	TitlefgRGBA;			/*!< �����ǰ��RGB����͸���� */
    DWORD	TitlebgRGBA;		/*!< ����ĺ�RGB����͸����*/
    WORD	TitleLeft;			/*!< �������ߵľ����������ı���*8192 */
    WORD	TitleTop;			/*!< ������ϱߵľ����������ı���*8192 */
    WORD	TitleRight;			/*!< ������ұߵľ����������ı���*8192 */
    WORD	TitleBottom;			/*!< ������±ߵľ����������ı���*8192 */
    BYTE	TitleEnable;			/*!< ����ʹ�� */
    BYTE	Reserved[3];
}ENCODE_TITLE;

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
}RECT,*PRECT;

//! ������Ϣ�ṹ(˫��������)
typedef struct {
    BYTE				CapVersion[8];				/*!< �汾��			*/		
    ENCODE_OPTION		MainOption[REC_TYP_NUM];	/*!< ��������REC_TYP_NUM��ͬ¼������*/
    ENCODE_OPTION		AssiOption[EXTRATYPES];	/*!< ֧��3 ·������ */
    BYTE				Compression;				/*!< ѹ��ģʽ */;	
    BYTE    			CoverEnable;				/*!< �����ڸǿ��ء�0x00��ʹ���ڸǣ�0x01���ڸ�Ԥ����0x10���ڸ�¼��0x11���ڸ�	*/
    BYTE 				alignres[2];			/*!< ���������� */
    RECT				Cover;						/*!< �����ڸǷ�Χ	*/	

    ENCODE_TITLE 		TimeTitle;					/*!< ʱ�����*/
    ENCODE_TITLE 		ChannelTitle;				/*!< ͨ������*/

    ENCODE_OPTION		PicOption[2];				//��ʱץͼ������ץͼ

    BYTE	Volume;				/*!< ���������ķ�ֵ */
    BYTE    VolumeEnable;       /*!< ������ֵʹ��*/
    BYTE	 Reserved[46];
}CONFIG_CAPTURE_OLD;

typedef struct tagPTZ_LINK
{
    int iType;				/*!< ���������� */
    int iValue;				/*!< ���������Ͷ�Ӧ��ֵ */
}PTZ_LINK;


//! �¼�����ṹ
typedef struct tagEVENT_HANDLER
{
    unsigned long	dwRecord;
    int		iRecordLatch;
    unsigned long	dwTour;
    unsigned long	dwSnapShot;
    unsigned long	dwAlarmOut;
    int		iAOLatch;
    PTZ_LINK	PtzLink[16];

    int 	bRecordEn;
    int 	bTourEn;
    int 	bSnapEn;
    int 	bAlarmOutEn;
    int 	bPtzEn;
    int 	bTip;
    int 	bMail;
    int 	bMessage;
    int 	bBeep;
    int 	bVoice;
    int 	bFtp;

    unsigned long	dwWorkSheet;

    unsigned long	dwMatrix;				/*!< �������� */
    int 	bMatrixEn;				/*!< ����ʹ�� */
    int 	bLog;					/*!< ��־ʹ�ܣ�Ŀǰֻ����WTN��̬�����ʹ�� */
    int		iEventLatch;			/*!< ������ʼ��ʱʱ�䣬sΪ��λ ��Χ��0����15 Ĭ��ֵ��0*/
    int 	bMessagetoNet;			/*!< ��Ϣ�ϴ�������ʹ�� */
    unsigned long	wiAlarmOut; 			/*!< ���߱������ */
    unsigned char	bMMSEn;					/*!< ���ű���ʹ��  */
    unsigned char	SnapshotTimes;          /*!< ץͼ���� */
    char	dReserved[22];			/*!< �����ֽ� */
} EVENT_HANDLER, *LPEVENT_HANDLER;

//!��̬�������
typedef struct tagCONFIG_MOTIONDETECT
{
    BOOL bEnable;					/*!< ��̬��⿪�� */
    int iLevel;						/*!< ������ */
    DWORD mRegion[MD_REGION_ROW];	/*!< ����ÿһ��ʹ��һ�������ƴ� */	
    EVENT_HANDLER hEvent;			/*!< ��̬������� */
} CONFIG_MOTIONDETECT;

typedef struct CAPTURE_DSPINFO 
{
    unsigned int	nMaxEncodePower;	///< DSP ֧�ֵ���߱���������
    unsigned short	nMaxSupportChannel;	///< DSP ֧�����������Ƶͨ������
    unsigned short	bChannelMaxSetSync;	///< DSP ÿͨ���������������Ƿ�ͬ�� 0-��ͬ��, 1 -ͬ����
    unsigned short	nExpandChannel;		///< DSP ֧�ֵ���չͨ��������Ҫ�Ƕ�·�ط�ʹ�ã�Ŀǰֻ��һ��
    unsigned short  rev;
}CAPTURE_DSPINFO, *PCAPTURE_DSPINFO;

typedef struct CAPTURE_ENCCHIPCAPABILITY
{
    int EncChipNR;
    CAPTURE_DSPINFO EncChipInfo[MAX_ENC_CHIP_NR];
}CAPTURE_ENCCHIPCAPABILITY,*PCAPTURE_ENCCHIPCAPABILITY;

#pragma pack(1)
typedef struct _CONFIG_TIME_
{
    char c[22];
}CONFIG_TIME;
#pragma pack()


#define ZHINUO_SET_BIT(addr, index)\
    (addr) |= (1 << ((index) - 1));

/// ����ͷ���ƽṹ
typedef struct cam_control{
    unsigned char  Exposure;        ///< �ع��ٶȵȼ���ȡֵ��Χȡ�����豸��������0-�Զ��ع⣬1-�ع�ȼ�1��2-�ع�ȼ�2��n-����ع�ȼ���
    unsigned char  Backlight;      /*< ���ⲹ���ȼ� ȡֵ��Χȡ�����豸��������0-�رգ�1-���ⲹ��ǿ��1��
                                     2-���ⲹ��ǿ��2��n-��󱳹ⲹ���ȼ���*/
    unsigned char AutoColor2BW;    ///< �Զ��ʺ�ת�� 0-���ǲ�ɫ 1- �Զ� 2-���Ǻڰ�
    unsigned char  Mirror;       //< ����  1 ���� 0��
    unsigned char  Flip;             //< ��ת  1 ���� 0��
    unsigned char   LensFunction;   //< ��4λ��ʾ������������4λ��ʾ���ܿ���
    //< bit4 = 1:֧���Զ���Ȧ���ܣ�
    //< bit0 = 0:�ر��Զ���Ȧ��bit0 = 1:�����Զ���Ȧ��
    unsigned char  WhiteBalance;     //��ƽ�� 0:Disabled,1:Auto 2:sunny 3:cloudy 4:home 5:office 6:night 
    unsigned char  SignalFormatMask;  //�źŸ�ʽ0-Inside(�ڲ�����) 1- BT656 2-720p 3-1080i  4-1080p  5-1080sF
    unsigned char  iRev[120];  // ����
}CAM_CONTROL;              //�ܳ���128�ֽ�

typedef struct __cam_caps{
    unsigned char iBrightnessEn;  // ���� �ɵ� 1 �ɣ� 0����
    unsigned char iContrastEn;        // �Աȶ� �ɵ�
    unsigned char iColorEn;       // ɫ�� �ɵ�
    unsigned char iGainEn;            // ���� �ɵ�
    unsigned char iSaturationEn;   // ���Ͷ� �ɵ�
    unsigned char iBacklightEn;   // ���ⲹ�� ֧�ֵı��ⲹ���ȼ��� 0��ʾ��֧�ֱ��ⲹ��,1��ʾ֧��һ����������,�أ���2��ʾ֧��������������,��,�ͣ���3��ʾ֧��������������,��,��,�ͣ�
    unsigned char iExposureEn;  // �ع�ѡ�� ֧�ֵ��ع��ٶȵȼ��� 0��ʾ��֧���ع���ƣ�1��ʾֻ֧���Զ��ع⣬������ʾ֧���ֶ����Ƶĵȼ���+1��1 ��Ӧ�Զ��ع⣩
    unsigned char iColorConvEn;   // �Զ��ʺ�ת�� �ɵ�
    unsigned char iAttrEn;            // ����ѡ��  1 �ɣ� 0����
    unsigned char iMirrorEn;       // ����  1 ֧�֣� 0��֧��
    unsigned char iFlipEn;             // ��ת   1 ֧�֣� 0��֧��
    unsigned char iWhiteBalance;      //��ƽ�� 0-��֧�ְ�ƽ�⣬1-֧���Զ���ƽ�⣬2-֧��Ԥ�ð�ƽ�⣨���龰ģʽ�� 
    unsigned char iSignalFormatMask;  //�źŸ�ʽ���룬��λ�ӵ͵���λ�ֱ�Ϊ��0-Inside(�ڲ�����) 1- BT656 2-720p 3-1080i  4-1080p  5-1080sF
    unsigned char iRev[123];      // ����
}CAM_CAPS;


#define N_WEEKS 7
#define N_TSECT 6

typedef struct tagTIMESECTION
{
    //!ʹ��
    int enable;
    //!��ʼʱ��:Сʱ
    int startHour;
    //!��ʼʱ��:����
    int startMinute;
    //!��ʼʱ��:����
    int startSecond;
    //!����ʱ��:Сʱ
    int endHour;
    //!����ʱ��:����
    int endMinute;
    //!����ʱ��:����
    int endSecond;
}TIMESECTION;   


typedef struct tagWORKSHEET
{
    int         iName;                          /*!< ʱ������� */  
    TIMESECTION tsSchedule[N_WEEKS][N_TSECT];   /*!< ʱ��� */
}CONFIG_WORKSHEET;
//ʹ��λ�Ķ��壺
#define REC_SYS                 0x0001      /*!< ϵͳԤ¼�¼�   */
#define REC_TIM                 0x0002      /*!< ��ʱ¼���¼�   */
#define REC_MTD                 0x0004      /*!< ����¼���¼�   */
#define REC_ALM                 0x0008      /*!< ����¼���¼�   */
#define REC_CRD                 0x0010      /*!< ����¼���¼�   */
#define REC_MAN                 0x0020      /*!< �ֶ�¼���¼�   */
#define SNAP_TIM                0x20000     /*!< ��ʱץͼ�¼�   */
#define SNAP_MTD                0x40000     /*!< ����ץͼ�¼�   */
#define SNAP_ALM                0x80000     /*!< ����ץͼ�¼�   */

//�ƶ������Ϣ 
typedef struct 
{
    DWORD dwVersion;
    BYTE dbReserve[4];
}VER_CONFIG;

typedef struct {
    VER_CONFIG sVerConfig;              /*! �����ļ��汾��Ϣ */
    DWORD   MotionRecordMask;           /*! ¼������ */
    DWORD   MotionAlarmMask;            /*! �������: ��λ��ʾ����ͨ�� */
    DWORD   MotionTimeDelay;            /*! ��ʱ��10��300 sec */
    DWORD   MotionRegion[18];           /*! ���� */
    DWORD   BlindAlarmMask;             /*! �������: ��λ��ʾ����ͨ�� */
    DWORD   LossAlarmMask;              /*! ��ʧ���: ��λ��ʾ����ͨ�� */
    BYTE    MotionLever;                /*! �����ȣ�1��6 */
    BYTE    dbReserved[2];              /*! �����ֶ� */
    BYTE    BlindTipEn;                 /*! ���������ʾʹ�ܣ�0:����ʾ1:��ʾ */
    BYTE    LossTipEn;                  /*! ��ʧ�����ʾʹ�ܣ�0:����ʾ1:��ʾ*/
    BYTE    Mailer;                     /*! �����ʼ����루1-8λ��ʾ���8ͨ����ͼ�� */
    BYTE    SendAlarmMsg;               /*! ���ͱ�����Ϣ���������������ȣ� */
    BYTE    adbReserved[2];             /*! �����ֶ� */
} CONFIG_DETECT;

typedef struct tagMOTION_DETECT_CAPS
{
    DWORD Enabled;       // ��1��ʾ֧�ֶ�̬��⣬��0��ʾ��֧�ֶ�̬��⡣
    DWORD GridLines;     // ��̬����������Ҫ���ֳɵ�������
    DWORD GridRows;      // ��̬����������Ҫ���ֳɵ�������
    BYTE  Result;        // �Ƿ��ܵõ�ÿ������ļ������
    BYTE  Hint;          // �Ƿ����ڼ��ɹ�ʱ������ʾ��
}MOTION_DETECT_CAPS;


//�¼�
#define DMS_MD_STRIDE_WIDTH     44
#define DMS_MD_STRIDE_HEIGHT    36
#define DMS_MD_STRIDE_SIZE        (DMS_MD_STRIDE_WIDTH*DMS_MD_STRIDE_HEIGHT/8)


typedef struct tagDMS_SCHEDTIME
{
    //��ʼʱ��
    BYTE byStartHour;
    BYTE byStartMin;
    //����ʱ��
    BYTE byStopHour;
    BYTE byStopMin;
}DMS_SCHEDTIME;

// ��̨����
typedef struct
{
    BYTE  byType;//���DMS_PTZ_LINK_TYPE��0:��Ч��1:Ԥ�õ㣬2:���Ѳ����3:���ù켣
    BYTE  byValue;
    BYTE  Reserve[2];
} DMS_PTZ_LINK, *LPDMS_PTZ_LINK;


#define DMS_MAX_ALARMOUT 			8
#define DMS_MAX_CHANNUM 			32
// ���������ṹ��
typedef struct
{
    /* ��ǰ������֧�ֵĴ���ʽ����λ�����ʾ */
    WORD                	wActionMask;

    /* ������������λ�����ʾ�����嶯������Ҫ�Ĳ����ڸ��Ե����������� */
    WORD                	wActionFlag;

    /* �������������ͨ�������������������Ϊ1��ʾ��������� */
    BYTE                		byRelAlarmOut[DMS_MAX_ALARMOUT/8];

    /* ����¼�� */
    BYTE                		byRecordChannel[DMS_MAX_CHANNUM/8]; /* ����������¼��ͨ����Ϊ1��ʾ������ͨ�� */

    /* ץͼͨ�� */
    BYTE                		bySnap[DMS_MAX_CHANNUM/8]; /*0��ʾ��ץ�ĸ�ͨ����1��ʾץ�ĵ�ǰͨ��*/
    /* ��Ѳͨ�� */
    BYTE                		byTour[DMS_MAX_CHANNUM/8]; /* �ð汾�ݲ���Ч*/

    /* ��̨���� */
    DMS_PTZ_LINK        stPtzLink[DMS_MAX_CHANNUM];
    WORD                	wDuration;              /* �����������ʱ�� */
    WORD                	wRecTime;               /* ����¼�����ʱ�� */
    WORD                	wSnapNum;              /* ����ץ��ͼƬ����*/
    BYTE                		wDelayTime;             /* �������ʱ�䣬sΪ��λ*/
    BYTE                		wBuzzerTime;            /* ���������ʱ��*/

} DMS_HANDLE_EXCEPTION;


typedef struct tagDMS_NET_MOTION_DETECT
{
    DWORD           dwSize;
    DWORD           dwChannel;
    BOOL            bEnable;                //�Ƿ���в���
    DWORD           dwSensitive;            //������ ȡֵ0 - 100, ԽСԽ����*/
    BOOL                bManualDefence;     //(yxhu 20130203 ����) �ֶ�������־��==YES(1)������==NO(0)����ʱ�ж�
    BYTE            byMotionArea[DMS_MD_STRIDE_SIZE];   //�����������.����������44*36����,��������BIT Ϊ��λ���ڴ��ȡ�
    DMS_HANDLE_EXCEPTION   stHandle;
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
}DMS_NET_CHANNEL_MOTION_DETECT, *LPDMS_NET_CHANNEL_MOTION_DETECT;

//osd ����
typedef struct __video_cover_t
{
    unsigned char iChannel;     // ͨ���ţ�[1~n]
    unsigned char iCoverNum;    // ���ǵ��������
    unsigned char unknown1;
    unsigned char unknown2;
    unsigned char unknown3;
    unsigned char unknown4;
    unsigned char iRev[26];     // ����
}VIDEO_COVER_T;

typedef struct __video_cover_attr_t
{
    RECT tBlock;    // ���ǵ���������
    unsigned char Color;      // ���ǵ���ɫ
    unsigned char iBlockType;  // ���Ƿ�ʽ��0-�ڿ飬1-������
    unsigned char Encode;     //! ���뼶�ڵ�, 1����Ч, 0��������
    unsigned char Priview;    //!Ԥ���ڵ�, 1����Ч, 0������Ч
    unsigned char iRev[32];    // ����
}VIDEO_COVER_ATTR_T;

typedef struct __video_cover_caps_t
{
    unsigned char iEnabled;         // ��1��ʾ֧�� �����ڵ���0 ��֧��
    unsigned char iBlockNum;        // ֧�ֵ��ڵ�����
    unsigned char iRev[62];         // ��̬����������Ҫ���ֳɵ�������
}VIDEO_COVER_CAPS_T;

//extern pid_t gettid();	//system_msg.c�����ж���
//extern int errno;

#define DAHUA_MSG_SEND_BUF_SIZE (10*1024)


typedef enum{
    UNINIT_SESSION = 0,
    CMD_SESSION,
    CONNECT_SESSION,
} DAHUA_SESSION_TYPE;

typedef struct {
	int fd; //accept_sock;
	int fSessionInt;
    char c_ip[16];
    DAHUA_SESSION_TYPE fSessionType;
	char fSendbuf[DAHUA_MSG_SEND_BUF_SIZE];
    unsigned int nSendLen;

    pthread_mutex_t send_mutex; //cmd_resq_send_mutex
} DAHUA_SESSION_CTRL;	

typedef struct {
    int msg_seq;
    int v_seq;
    int v_timestmp;
    int a_seq;
    int a_timestmp;
    unsigned char ch_frame_type;
    unsigned char ch_width;
    unsigned char ch_height;
    unsigned char ch_fps;
    int frame_len;
    char *pData;
} DahuaFrameInfo;

typedef struct {
    unsigned int transaction_id;
    char method[100];
    char parameter_name[1000];
    unsigned int sid;
    unsigned int connect_id;
    unsigned int connect_protocol;
    int channel;
    int stream;
    int state;
} DahuaMethodField;

typedef struct {
    int enable;
    int brightness;
    int chromaSuppress;
    int contrast;
    int gamma;
    int hue;
    int saturation;
    int sharpness;
    int day_night_color;
    int flip;
    int mirror;
    int nSwitchMode;
} DahuaColorProfile;


typedef struct {
    int sock;
    char ip[16];
} DahuaClientInfo;


typedef	void *(*ThreadEntryPtrType)(void *);


#endif
