/**
 * goolink_apiv2.h
 *
 * External APIs for device side implementation.
 *
 * Created by Wang  2017-02-07
 *
 * utf-8
 */

#ifndef __GOO_LINK_APIV2_H__
#define __GOO_LINK_APIV2_H__

#ifdef __cplusplus
extern "C" {
#endif


#include<stdint.h>
#include<inttypes.h>

/*****************************************************************************
 							���ݳ��Ⱥ�
 *****************************************************************************/
#define GLNKPACKET_STR_LEN_64						64
#define GLNKPACKET_STR_LEN_32						32
#define GLNKPACKET_STR_LEN_16						16

/*****************************************************************************
 							��Ƶ��ʽ
 *****************************************************************************/
//G711a wave format
#ifndef GLNK_WAVE_FORMAT_G711
#define GLNK_WAVE_FORMAT_G711						0x7A19
#endif
//G711U wave format
#ifndef GLNK_WAVE_FORMAT_G711U
#define GLNK_WAVE_FORMAT_G711U                   	0x7A25
#endif
//AAC  wave format
#ifndef WAVE_FORMAT_AAC
#define WAVE_FORMAT_AAC                   			0x7A26
#endif

/*****************************************************************************
 							��Ƶ��ʽ
 *****************************************************************************/
//H264 wave format
#define GLNK_CODEC_H264								0x34363248
//mjpeg
#define CODEC_MJPEG									0x45464758
//H265
#define GLNK_CODEC_H265								0x56565268

//G726 wave format
#ifndef GLNK_WAVE_FORMAT_G726
#define GLNK_WAVE_FORMAT_G726						0x7A20
#endif

//AMR_NB CBR wave format
#ifndef GLNK_WAVE_FORMAT_AMR_CBR
#define GLNK_WAVE_FORMAT_AMR_CBR 					0x7A21
#endif

//AMR VBR Not support yet
#ifndef GLNK_WAVE_FORMAT_AMR_VBR
#define GLNK_WAVE_FORMAT_AMR_VBR 					0x7A22
#endif

//SPEEX nb mode wave format
#ifndef GLNK_WAVE_FORMAT_SPEEX_NB_MODE
#define GLNK_WAVE_FORMAT_SPEEX_NB_MODE           	0x7A23
#endif

//SPEEX wb mode wave format
#ifndef GLNK_WAVE_FORMAT_SPEEX_WB_MODE
#define GLNK_WAVE_FORMAT_SPEEX_WB_MODE           	0x7A24
#endif

//AMR_WB Wave format
#ifndef GLNK_WAVE_FORMAT_VOICEAGE_AMR_WB
#define GLNK_WAVE_FORMAT_VOICEAGE_AMR_WB			0xA104
#endif

/*****************************************************************************
 							�����뼰һЩö��
 *****************************************************************************/
#define GLNK_ERR_NOERR             0       // No error
#define GLNK_ERR_MUTEX             -0x101  // Mutex creation error
#define GLNK_ERR_THREAD            -0x102  // Thread creation error
#define GLNK_ERR_SOCKET            -0x103  // Socket creation error
#define GLNK_ERR_SOCK_OPT          -0x104  // Socket option setting error
#define GLNK_ERR_SOCK_BIND         -0x105  // Socket bind error
#define GLNK_ERR_TIMEOUT           -0x106  // timeout error
#define GLNK_ERR_NOMEM             -0x107  // memory error
#define GLNK_ERR_PARAM             -0x108  // invalid param
#define GLNK_ERR_IVALSID           -0x109  // invalid session id
#define GLNK_ERR_NAT_TRAVERSAL     -0x10A  // NAT traversal failure
#define GLNK_ERR_PROTOCOL     		-0x10B  //
#define GLNK_ERR_MAX               -0x200  //
#define GLNK_ERR_BUFNOTEXIST       -0x202  // buffer not exist
#define GLNK_ERR_NOTSTART          -0x203  // goolink module not start
#define GLNK_ERR_CLOSING           -0x204  // goolink module is closing
#define GLNK_ERR_NOENOUTGMEM       -0X205  // not enough memery
#define GLNK_ERR_APINOTSUPPORT     -0x206  // in the case the api is not support
#define GLNK_ERR_BUSY              -0x205  // busy

/*		�ļ���������			*/
typedef enum _OpenDownLoadFIleType
{
	NORMALDOWNLOAD = 1,							// ��������
	BREAKPOINTDOWNLOAD,							// �ϵ�����
	ENDDOWNLOAD,								// ��������
}OpenDownLoadFIleType;

/*		�ƴ����ϴ�ͼƬ���ؽ��  */
typedef enum _UPLOADDATARESULT
{
	UPLOADDATASUC 					= 1,		// �ϴ��ɹ�
	UPLOADDATAFAIL_NETWORK			= 2,		// �������⵼���ϴ�ʧ��
	UPLOADDATAFAIL_COMPETENCE		= 3,		// û��Ȩ�޵����ϴ�ʧ��
	UPLOADDATAFAIL_UPLOADERROR		= 4,		// �ϴ���;�������ϴ�ʧ��
	UPLOADDATAFAIL_MODULESTOP		= 5,		// �ƴ洢ģ��û����
	UPLOADDATAFAIL_DATAUPLOADING	= 6,		// �Ѿ������������ϴ�����ʧ��
	UPLOADDATAFAIL_NOSTREAMUPLOAD	= 7,		// û���ϴ���Ƶ���������ϴ�ͼƬ
}UPLOADDATARESULT;


/*		���۽ӿ����ͺ�			*/
enum SwitchType
{
	MUSIC_SWITCH = 1, 							// ���ֿ���	��ӦSwtichOCValue
	FEEDING_SWITCH,  							// ιʳ����	��ӦSwtichOCValue
	FISHEYE_SWITCH,   							// ���ۿ���	��ӦFishEyeModeValue
};

enum SwtichOCValue
{
	CLOSE_VALUE = 0,
	OPEN_VALUE = 1,
};

enum FishEyeModeValue
{
	//����ģʽ
	CEILINGMODE_DEFAULT = 0x10,					// Ĭ��ģʽ
	CEILINGMODE_360PANORAMIC = 0x11,			// 360��ȫ��
	CEILINGMODE_QUAD = 0x12, 					// �ķָ�
	CEILINGMODE_PTZ = 0x13, 					// PTZ
	//�������
	DESKTOPMODE_DEFAULT = 0x20,					// Ĭ��ģʽ
	DESKTOPMODE_360PANORAMIC = 0x21,			// 360��ȫ��
	DESKTOPMODE_QUAD = 0X22, 					// �ķָ�
	DESKTOPMODE_PTZ = 0X23,						// PTZ
	//��װģʽ
	WALLMODE_DEFAULT = 0X30, 					// Ĭ��ģʽ
	WALLMODE_180HORIZONTALLY = 0x31,			// ˮƽ180��
	WALLMODE_QUAD = 0x32,						// �ķָ�
	WALLMODE_PTZ  = 0x33,						// PTZ
};

/*		��Ƶ�ֱ��ʺ�	*/
typedef enum _OWSP_VIDEO_RESOLUTION
{
	RESOLUTION_1080P = 0,
	RESOLUTION_720P,
	RESOLUTION_D1,
	RESOLUTION_VGA,
	RESOLUTION_CIF,
	RESOLUTION_QVGA,
	RESOLUTION_QCIF,
	RESOLUTION_QQVGA
}OWSP_VIDEO_RESOLUTION;

/*****************************************************************************
 							����
 *****************************************************************************/
/*		����ʱ�䶨��		*/
typedef struct _GLNK_DateTime
{
	uint32_t		m_microsecond;				// ����	0-1000
	uint32_t 		m_year;						// ��,2009
	uint32_t		m_month;					// ��,1-12
	uint32_t		m_day;						// ��,1-31
	uint32_t		m_hour;						// 0-24
	uint32_t		m_minute;					// 0-59
	uint32_t		m_second;					// 0-59
} GLNK_DateTime;

typedef struct _GLNK_V_AudioInfo
{
	unsigned char  channelId;
	unsigned char  reserve;
	unsigned short checksum;
	unsigned int time;
}GLNK_V_AudioInfo;

typedef struct _GLNK_V_RecordChgRequest
{
    uint32_t  		command;
    GLNK_DateTime   startTime;
    GLNK_DateTime   endTime;
}GLNK_V_RecordChgRequest;
/*
 *	TLV: TLV_T_VA_GET_CONFIG_RSP
 *	Direction: Deivce -> Client
 *	Description:
 *		channel: the channel of
 */
typedef struct _TLV_V_VA_GET_CONFIG_RSP
{
	unsigned char 	channel;
	unsigned char  isOpenAudio;
	unsigned char	reverse[2];
	OWSP_VIDEO_RESOLUTION mainStreamResolution;
	unsigned int	mainStreamFrameRate;
	unsigned int mainStreamBitRate;
	OWSP_VIDEO_RESOLUTION subStreamResolution;
	unsigned int subStreamFrameRate;
	unsigned int subStreamBitRate;
}TLV_V_VA_GET_CONFIG_RSP;
/*
 *	TLV: TLV_T_VA_SET_CONFIG_REQ
 *	Direction: Client -> Device
 *	Description:
 *
 */
typedef struct _TLV_V_VA_SET_CONFIG_REQ
{
	unsigned char 	channel;
	unsigned char  isOpenAudio;
	unsigned char	reverse[2];
	OWSP_VIDEO_RESOLUTION mainStreamResolution;
	unsigned int	mainStreamFrameRate;
	unsigned int mainStreamBitRate;

	OWSP_VIDEO_RESOLUTION subStreamResolution;
	unsigned int subStreamFrameRate;
	unsigned int subStreamBitRate;
}TLV_V_VA_SET_CONFIG_REQ;

typedef struct _TLV_V_Network
{
	unsigned int deviceId;
	unsigned char  hostIP[4];
	unsigned char    hostName[32];
	unsigned char  gateway[4];
	unsigned char  dnsServer[4];
	unsigned char  dnsServer2[4];
	unsigned char  subnetMask[4];

	unsigned char  wifiHostIP[4];
	unsigned char  wifiHostName[32];
	unsigned char  wifiGateway[4];
	unsigned char  wifiDnsServer[4];
	unsigned char  wifiDnsServer2[4];
	unsigned char  wifiSubnetMask[4];
	unsigned char  wifiMac[8];
	unsigned char  mac[8];
	unsigned char  wifiIPMode;
	unsigned char  IPMode;
	unsigned char  reverse[2];
} TLV_V_Network;

typedef enum _OWSP_StreamType
{
	OWSP_STREAM_MAIN	= 0,        //������
	OWSP_STREAM_SUB		= 1,        //������
	OWSP_STREAM_VOD		= 2,
	OWSP_MODE_SETTING	= 3,
	OWSP_VIEWMODE_LLD   = 4,				//very Low Display resolution  eg:QQVGA
	OWSP_VIEWMODE_LD    = 5,				//Low Display resolution	  eg:QVGA QCIF
	OWSP_VIEWMODE_SD    = 6,				//Stand Display resolution	eg:VGA D1
	OWSP_VIEWMODE_HD    = 7				//High Display resolution	  eg:720p 1080p
} OWSP_StreamType;

typedef struct _GLNK_V_VideoModeRequest
{
	uint32_t deviceId;
	uint16_t channel;
	uint16_t reserve;
	OWSP_StreamType mode;
}GLNK_V_ViewModeRequest;


/*		������������			*/
typedef struct _GLNK_V_SearchAudioFileRequest
{
	uint32_t  AudioType;      					// ������������ 0x01 = ��������, 0x02 = ��ʿ��Ƶ, 0x04 = ҡ������, 0x08 = �ŵ����֣� 0xFF = ȫ������*/
} GLNK_V_SearchAudioFileRequest;

/*		�������ֽṹ			*/
typedef struct _GLNK_V_AudioFileInfo
{
	char	       fileName[128];   			// �ļ�����
	uint32_t       length;        				// ����ʱ�䣺��λ��
	uint8_t        AudioType;     				// �������� /* AudioType : ������������ 0x01 = ��������, 0x02 = ��ʿ��Ƶ, 0x04 = ҡ������, 0x08 = �ŵ����֣� 0xFF = ȫ������*/
	uint8_t        reserve[3];
}GLNK_V_AudioFileInfo;


typedef struct _GLNK_V_AlarmInfo{
	uint64_t AlarmTime;							//����ʱ�䵥λ���루s��  1970-1-1 0:0:0 ���𵽱���ʱ�侭��������
    int32_t  AlarmType;							//��������
    int32_t  Channel;							//����ͨ��
}GLNK_V_AlarmInfo;

typedef enum _SendVodStreamType
{
	SendFileType = 1,
	SendStreamType = 2,
} SendVodStreamType;

typedef enum _UPDATECTRL
{
	UPDATECTRL_ENABLE = 1,
	UPDATECTRL_DISABLE,
}UPDATECTRL;

/*****************************************************************************
 							��ʼ���ṹ
 *****************************************************************************/
typedef struct InitNetParam
{
	char 				dev[8];						//�������ƣ���Ҫ��
	uint16_t 			localTCPport;				//ֱ��tcp�˿�
	uint16_t 			localUDPport;				//ֱ��udp�˿�
	char 				udid[32];					//goolink id
	unsigned char		channelnum;					//ͨ���� 1-64
	unsigned char 		issupportsubvideostream;	//�Ƿ�֧����Ƶ������0-��ʾ��֧�֣�1��ʾ֧��
	unsigned char		maxsession;					//�豸���ͬʱ������
	char 				reserve;
}InitNetParam;

/*****************************************************************************
 							���ڶ���
 *****************************************************************************/
typedef struct _GooTime
{
    uint16_t  year;    						// YYYY, eg. 2009
    uint16_t  month;   						// MM, 1-12
    uint8_t   day;     						// DD, 1-31
    uint8_t   hour;    						// HH, 0-24
    uint8_t   minute;  						// MM, 0-59
    uint8_t   second;  						// SS, 0-59
} GooTime;

typedef struct _GLNK_Date
{
	uint16_t year;		       				// YYYY, 2012
	uint8_t month;		       				// MM, 1-12
	uint8_t day;			   				// DD, 01-31
} GLNK_Date;

/*****************************************************************************
 							�����ӿڽṹ
 *****************************************************************************/
typedef struct _PushAlarm
{
    short   	alarm_type;					// �������ͣ�����
    short   	channel;					// ͨ����
    GooTime   	timestamp;					// ����ʱ��
	char 		CustomBuf[64];				// �����ı��������
} PushAlarm;

/*		��������		*/
typedef enum _PushAlarmType
{
	PAT_VIDEO_FRAME    			= 0,       	// IP��ַ��ͻ����	(���ã�ûʲô��)
	PAT_DEVICE_RESTART 			= 1,       	// �豸����
	PAT_MOTION_DETECT  			= 2,       	// �ƶ���ⱨ��
	PAT_VIDEO_LOSS    			= 3,       	// ��Ƶ��ʧ
	PAT_VIDEO_SHEILD      		= 4,       	// ��Ƶ�ڵ�����
	PAT_BORDER_DETECT   		= 5,       	// Խ����ⱨ��
	PAT_ZONE_INTRUSION       	= 6,       	// �������ֱ���
	PAT_SWITCH_SENSOR  			= 7,       	// ������̽ͷ����
	PAT_PIR_ALARM      			= 8,       	// ���ⱨ��
	PAT_SMOKE_ALARM   			= 9,       	// ������
	PAT_NOISE_ALARM    			= 10,      	// ��������
	PAT_TEMPERATURE_ALARM    	= 11,      	// �¶��쳣����
	PAT_HUMIDITY_ALARM          = 12,      	// ʪ���쳣����
	PAT_GAS_ALARM       		= 13,      	// ���屨��
	PAT_CALLING_ALARM  			= 14,	  	// �������
	PAT_SDCARDERROR_ALARM		= 15,		// SD��/Ӳ���쳣����
	PAT_SDCARDOUT_ALARM 		= 16, 		// SD��/Ӳ�̰γ�����
	PAT_SDCARDFULL_ALARM 		= 17, 		// SD��/Ӳ������������
	PAT_DEVICEMOVE_ALARM 		= 18, 		// �豸�ƶ�����
	PAT_ENERGYREMOVE_ALARM 		= 19, 		// ��Դ�γ�����
	PAT_EXTERNALPOWER_ALARM 	= 20, 		// �ⲿ��Դ���뱨��
	PAT_LOWPOWER_ALARM 			= 21, 		// �͵�������
	PAT_IO_ALARM 				= 22,  		// I/O����
	PAT_RINGING_ALARM			= 25,		// ���˺�����
	PAT_CALLANSWERED_ALARM		= 26,		// �����ѽ���
	PAT_DOORBELL_ALAEM			= 100,  	// �ſڻ�����
} PushAlarmType;

/*     433�����ӿڽṹ   */
typedef struct _Push433Alarm
{
	char 		AddrCode[20];				// 433�豸��ַ��
	char 		Mcode;						// ��ֵ
	int32_t 	Tcode;						// �豸����
	GooTime   	timestamp;					// ����ʱ��
}Push433Alarm;

/*****************************************************************************
 							����ӿڽṹ
 *****************************************************************************/
/*		��Ƶ����ӿ�	*/
typedef struct _GLNK_VideoDataFormat
{
	unsigned int codec;					// ���뷽ʽ
	unsigned int bitrate;        			// ������, bps
	unsigned short width;					// ͼ����
	unsigned short height;					// ͼ��߶�
	unsigned char framerate;				// ֡��, fps
	unsigned char colorDepth;				// should be 24 bits
	unsigned char frameInterval;   		// I֡���(s)
	unsigned char reserve;
}GLNK_VideoDataFormat;

/*		��Ƶ����ӿ�	*/
typedef struct _GLNK_AudioDataFormat
{
	unsigned int samplesRate;				// ÿ�����
	unsigned int bitrate;					// ������, bps
	unsigned short waveFormat;				// �����ʽ
	unsigned short channelNumber;			// ��Ƶͨ�� 1��ͨ�� 2˫ͨ��һ��Ϊ��ͨ����
	unsigned short blockAlign;				// �����, channelSize * (bitsSample/8)
	unsigned short bitsPerSample;			// ÿ����������
	unsigned short frameInterval;			// ֡���, ��λms
	unsigned short reserve;
} GLNK_AudioDataFormat;

/*****************************************************************************
 							�豸��Ϣ�ӿ�
 *****************************************************************************/
typedef struct _GLNK_V_DeviceInfo
{
	char companyId[GLNKPACKET_STR_LEN_16];			// ��˾id
	char productId[GLNKPACKET_STR_LEN_16];			// ��Ʒ����
	char name[GLNKPACKET_STR_LEN_16];				// ��Ʒ����
	char softwareVersion[GLNKPACKET_STR_LEN_16];	// ����汾
	GLNK_Date	manufactureDate;					// �̼�����
	unsigned char channelNum;						// ͨ����
	unsigned char alarmType;						// ��������
	unsigned char deviceType;						// �豸����
	unsigned char reserve2;
} GLNK_V_DeviceInfo;

/*****************************************************************************
 							��̨�ӿ�
 *****************************************************************************/
typedef struct _GLNK_V_PTZControlRequest
{
	unsigned int deviceId;					// �豸ID
	unsigned char  channel;				// ͨ����
	unsigned char  cmdCode;				// ����������, �μ�GLNK_PTZControlCmd
	unsigned short size;					// ���Ʋ������ݳ���,���size==0 ��ʾ�޿��Ʋ���
} GLNK_V_PTZControlRequest;

// ControlArgData
//   GLNK_PTZ_MV_STOP   	����
//   GLNK_PTZ_ZOOM_DEC  	��arg1, ����
//   GLNK_PTZ_ZOOM_INC  	��arg1, ����
//   GLNK_PTZ_FOCUS_INC 	��arg1, ����
//   GLNK_PTZ_FOCUS_DEC 	��arg1, ����
//   GLNK_PTZ_MV_UP     	��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_DOWN   	��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_LEFT   	��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_RIGHT  	��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_IRIS_INC  	��arg1, ����
//   GLNK_PTZ_IRIS_DEC 		��arg1, ����
//   GLNK_PTZ_AUTO_CRUISE  : arg1, 1 = ��ʼѲ��, 0 = ֹͣѲ��; arg2, ˮƽ�ٶ�; arg3, ��ֱ�ٶ�
//   GLNK_PTZ_GOTO_PRESET  : arg1, Ԥ�õ���
//   GLNK_PTZ_SET_PRESET   : arg1, Ԥ�õ���
//   GLNK_PTZ_CLEAR_PRESET : arg1, Ԥ�õ���, ���Ϊ0xFFFFFFFF��ʶ���ȫ��
//   GLNK_PTZ_ACTION_RESET
//   GLNK_PTZ_MV_LEFTUP    ��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_LEFTDOWN  ��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_RIGHTUP   ��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_MV_RIGHTDOWN ��arg1, ˮƽ�ٶ�; arg2, ��ֱ�ٶ�; arg3, ����
//   GLNK_PTZ_CLEAR_TOUR   ��arg1, ��·���
//   GLNK_PTZ_ADD_PRESET_TO_TOUR : arg1, Ԥ�õ���; arg2, ��·���
//   GLNK_PTZ_DEL_PRESET_TO_TOUR : arg1, Ԥ�õ���; arg2, ��·���

typedef enum _GLNK_PTZControlCmd
{
	GLNK_PTZ_MV_STOP      = 0,    	//ֹͣ�˶�
	GLNK_PTZ_ZOOM_DEC     = 5,		//�Ŵ�
	GLNK_PTZ_ZOOM_INC     = 6,		//��С
	GLNK_PTZ_FOCUS_INC    = 7,    	//����Ŵ�
	GLNK_PTZ_FOCUS_DEC    = 8,		//������С
	GLNK_PTZ_MV_UP        = 9,    	//����
	GLNK_PTZ_MV_DOWN      = 10,   	//����
	GLNK_PTZ_MV_LEFT      = 11,   	//����
	GLNK_PTZ_MV_RIGHT     = 12,   	//����
	GLNK_PTZ_IRIS_INC     = 13,   	//��Ȧ�Ŵ�
	GLNK_PTZ_IRIS_DEC     = 14,   	//��Ȧ��С
	GLNK_PTZ_AUTO_CRUISE  = 15,	  	//�Զ�Ѳ��
	GLNK_PTZ_GOTO_PRESET  = 16,   	//��תԤ��λ
	GLNK_PTZ_SET_PRESET   = 17,   	//����Ԥ��λ��; 255: ����, 254:��λ
	GLNK_PTZ_CLEAR_PRESET = 18,   	//���Ԥ��λ��
	GLNK_PTZ_ACTION_RESET = 20,   	//PTZ��λ
	GLNK_PTZ_MV_LEFTUP    = 21,
	GLNK_PTZ_MV_LEFTDOWN  = 22,
	GLNK_PTZ_MV_RIGHTUP   = 23,
	GLNK_PTZ_MV_RIGHTDOWN = 24,
	GLNK_PTZ_CLEAR_TOUR   = 25,
	GLNK_PTZ_ADD_PRESET_TO_TOUR  = 26,
	GLNK_PTZ_DEL_PRESET_TO_TOUR  = 27
} GLNK_PTZControlCmd;

/*		��̨���ƽӿ�		*/
typedef struct _GLNK_ControlArgData
{
	unsigned int arg1;
	unsigned int arg2;
	unsigned int arg3;
	unsigned int arg4;
} GLNK_ControlArgData;

/*****************************************************************************
 							¼��ӿ�
 *****************************************************************************/
/*		¼�������ṹ		*/
typedef struct _GLNK_V_SearchFileRequest
{
	uint32_t 		deviceId;					// �豸��ţ�Ĭ��Ϊ0��
	uint32_t		channelMask;				// ��ͨ������1-32ͨ��, ��Ҫ������Щͨ����¼��, ����λ��1. 0xFFFF FFFF��ʾ����ͨ��
	GLNK_DateTime 	startTime;					// ��ѯ��ʼʱ��
	GLNK_DateTime 	endTime;					// ��ѯ��ֹʱ��
	uint32_t  		recordTypeMask;				// ¼���������� 0x01 = �������澯¼��, 0x02 = �ƶ����¼��, 0x04 = ����¼��, 0x08 = �ֶ�¼�� 0xFF = ȫ��¼��
	uint32_t  		index;						// �ļ�����, ��Ϊ0
	uint32_t		count;						// �������ݸ���, Ĭ��Ϊ10
	uint32_t  		channelMask2;				// ��ͨ������33-64ͨ��
} GLNK_V_SearchFileRequest;

/*		¼�������ṹ		*/
typedef struct _GLNK_V_FileInfo
{
	char	        fileName[260];				// �ļ���, ����·��
	uint32_t       	deviceId;					// �豸ID��Ĭ��Ϊ0��
	uint32_t       	length;						// �ļ��ܳ���
	uint32_t       	frames;						// ��֡��
	GLNK_DateTime	startTime;					// ��ʼʱ��
	GLNK_DateTime	endTime;					// ����ʱ��
	uint8_t	      	channel;					// ¼��ͨ����
	uint8_t        	recordType;					// ¼���������� 0x01 = �������澯¼��, 0x02 = �ƶ����¼��,0x04 = ����¼��, 0x08 = �ֶ�¼��
	uint8_t        	reserve[2];
} GLNK_V_FileInfo;

/*****************************************************************************
			 	 	 	 	 ��¼��ӿ�
*****************************************************************************/
typedef struct _GLNK_SearchPlaybackRequest
{
	uint32_t  		deviceId;			//�豸id��Ĭ����0
	uint32_t  		channelMask;		//��ͨ�����룬0x88��ʾ1000 1000��ʾͨ��4��ͨ��8����������1-32��ͨ��
	uint32_t  		channelMask2;		//��ͨ�����룬������33-64��ͨ��
	GLNK_DateTime  	startTime;			//������ʼʱ��
	GLNK_DateTime 	endTime;			//��������ʱ��
	uint32_t  		recordTypeMask;		//¼���������� 0x01 = �������澯¼��, 0x02 = �ƶ����¼��, 0x04 = ����¼��, 0x08 = �ֶ�¼�� 0xFF = ȫ��¼��
	uint32_t  		reserve;			//����λ
} GLNK_SearchPlaybackRequest;

typedef struct _GLNK_SearchPlaybackReply
{
	uint8_t       	recordType;			//¼���������롣0x01 = �������澯¼��, 0x02 = �ƶ����¼��, 0x04 = ����¼��, 0x08 = �ֶ�¼��
	uint8_t	      	channel; 			//ͨ����
	uint32_t       	deviceId;			//�豸id��Ĭ����0
	GLNK_DateTime	startTime;			//¼����ʼʱ��
	GLNK_DateTime	endTime;			//¼�����ʱ��
	uint32_t        reserve;			//����λ
} GLNK_SearchPlaybackReply;

/*		¼������˵��		*/
typedef enum _ContrlSendVodStreamType
{
	ResumeSendStream 		= 0,				// ��������
	PauseSendStream 		= 1,				// ��ͣ����
	StopSendStream 			= 2,				// ֹͣ����
	PlusSendStream 			= 3,				// ���
	MinusSendStream 		= 4,				// ����
	JumpFrameSendStream 	= 5					// �϶�����
} ContrlSendVodStreamType;

/*		¼������˵��		*/
typedef enum _VodStreamType
{
	VideoStream 			= 0,				// ��Ƶ
	AudioStream 			= 1,				// ��Ƶ
	VideoAndAudioStream 	= 2					// ��Ƶ����Ƶ
} VodStreamType;

/*****************************************************************************
			 	 	 	 	 ap�ӿ�
*****************************************************************************/
/*		wifi�ź�ǿ��		*/
typedef enum _GLNK_WifiSignalLevel
{
	GLNK_Signal_Strong = 0,						// ǿ
	GLNK_Signal_Mid,							// ��
	GLNK_Signal_Weak							// ��
}GLNK_WifiSignalLevel;

/*		�豸����wifi����		*/
typedef struct _GLNK_V_WifiInfo
{
	char name[GLNKPACKET_STR_LEN_32];			// goolink id
	char ssid[GLNKPACKET_STR_LEN_32];			// ssid
	GLNK_WifiSignalLevel level;					// wifi�ź�ǿ��
}GLNK_V_WifiInfo;

/*		app����wifi������Ϣ		*/
typedef struct _GLNK_V_WifiConfigRequest
{
	char name[GLNKPACKET_STR_LEN_32];			// goolink id
	char ssid[GLNKPACKET_STR_LEN_32];			// ssid
	char password[GLNKPACKET_STR_LEN_32];		// ����
	uint32_t networkType;						//
	uint32_t encryptType;						// ��������
}GLNK_V_WifiConfigRequest;

/*		 AP������Э�� 			*/
typedef struct _GLNK_SSIDList
{
	char SSID[32];
	GLNK_WifiSignalLevel SignalLevel;				//�ź�ǿ�� 0=ǿ��1=�У�2=��
	int32_t reverse;								//����λ
}GLNK_SSIDList;

typedef struct _GLNK_GetSSIDListResponse
{
	int32_t SSIDListLen;							//�б�ĳ���=sizeof(GLNK_GetSSIDListResponset) + sizeof(GLNK_SSIDList)*ListNum,numΪ�б�ĸ���
	GLNK_SSIDList SSIDList[0];						//wifi�б�䳤�ṹ��
}GLNK_GetSSIDListResponset;

/*****************************************************************************
			 	 	 	 	 �û��б�����
*****************************************************************************/
/* 		�û����붨�� 			*/
typedef struct _GLNK_V_UserPWD
{
	char username[32];							// 1-32���ַ�
	char pwd[16]; 								// 1-16���ַ�
  uint32_t reserve;								// �����ֽ�
}GLNK_V_UserPWD;

/*		�û��б����ݽṹ		*/
typedef struct _GLNK_V_SetUserListRequest
{
	GLNK_V_UserPWD  user[8];					//�û��б�
	uint8_t  		reserve[8];					// �����ֽ�
}GLNK_V_SetUserListRequest;

/*		�û��б����ݽṹ		*/
typedef struct _GLNK_V_GetUserListResponse
{
	GLNK_V_UserPWD  user[8];					//�û��б�
	uint8_t  reserve[8];						//�����ֽ�
}GLNK_V_GetUserListResponse;

/*****************************************************************************
			 	 	 	 	 ��ȡ�豸Ӳ��(sd��)�����Ľӿ�
*****************************************************************************/
typedef struct _GLNK_DeviceStorageList
{
	int32_t StorageID;								//Ӳ��(sd��)ID	��һ����1���ڶ�����2��������
	int32_t StorageCap;								//�ܴ洢����(MB)
	int32_t StorageCapRemain;						//ʣ������(MB)
	int32_t reverse;								//����λ
}GLNK_DeviceStorageList;

typedef struct _GLNK_DeviceStorageResponse
{
	int32_t DeviceStorageListLen;							//�б�ĳ���=sizeof(GLNK_DeviceStorageResponset) + sizeof(GLNK_DeviceStorageList)*ListNum,numΪӲ�̻�sd���ĸ���
	GLNK_DeviceStorageList DeviceStorageList[0];			//Ӳ��(sd��)�б�䳤�ṹ��
}GLNK_DeviceStorageResponse;


/*****************************************************************************
			 	 	 	 	��������Э��
 *****************************************************************************
	����ʾ��Ļ����˳��Ϊ��д˳����Ϊ4������1�� 4����ⱨ������
	GLNK_SetScreenAlarmRequest->ScreenListLen = sizeof(GLNK_SetScreenAlarmRequest) + sizeof(GLNK_ScreenList) * 4;
	GLNK_SetScreenAlarmReques->ScreenList[0].ifSetAlarm = 1;
	GLNK_SetScreenAlarmReques->ScreenList[3].ifSetAlarm = 1;
  						---------
  						| 1 | 2 |
  						---------
  						| 3 | 4 |
  						---------												*/
typedef struct _GLNK_ScreenList
{
	int32_t ifSetAlarm;				//1Ϊ����Ϊ��ⱨ����0Ϊ�����
	int32_t reserve;				//�����ֶ�
}GLNK_ScreenList;

typedef struct _GLNK_SetScreenAlarmRequest
{
	int32_t ScreenChannel; 				//ѡ����ⱨ����ͨ����
	int32_t ScreenListLen;				//�б�ĳ��� = sizeof(GLNK_SetScreenAlarmRequest) + sizeof(GLNK_ScreenList)*ListNum,numΪ��������
	GLNK_ScreenList ScreenList[0];			//�����ֶ�
}GLNK_SetScreenAlarmRequest;

// ��������ΪҪ��ȡ��ͨ����
typedef struct
{
	int32_t ScreenChannel; 				//ѡ����ⱨ����ͨ����
}GLNK_GetScreenAlarmRequest;

/*****************************************************************************
			 	 	 	 	app�������ap���뼰����
*****************************************************************************/
typedef struct _TLV_V_ChangeAPPasswordRequest
{
	char oldpassword[GLNKPACKET_STR_LEN_32];	// ԭ��������
	char newpassword[GLNKPACKET_STR_LEN_32];	// ����������
	int32_t isopenorclose;						// 0 -- ��wifi 1 -- ��wifi 2 -- �޸�wifi����
	int32_t reserve;				//�����ֶ�
}TLV_V_ChangeAPPasswordRequest;


/*****************************************************************************
			 	 	 	 	API	�ӿ�
 *****************************************************************************/

/*	��ȡgoolink�汾��
 * 	retutn : 16���Ƶ�SDK�汾�ţ��緵��0x15160412���ʾ��1.5�汾2016��4��12�ո��µİ汾 		*/
unsigned long glnk_get_version();


/*	goolink�汾��
 *  netparam ������
 * 	retutn : 0�ɹ�	*/
int glnk_init( InitNetParam* netparam);


/*	����goolink
 * 	retutn : 0--�ɹ�   ����--ʧ��															*/
int glnk_destroy();


/*	���ͱ����ӿ�
 * 	retutn : 1--�ɹ�   0--ʧ��																*/
int glnk_push_alarm(PushAlarm alarm);


/*	����433�����ӿ�
 * 	retutn : 1--�ɹ�
 * 			 2--�������ͼ��С��5s
 * 			 3--�豸û�д˵�ַ��
 * 			 4--����ģʽ���ַ����������													*/
int32_t glnk_push_alarm2(Push433Alarm _433alarm);


/*	��ȡgoolink���ӷ�����״ֵ̬
 * 	retutn : 0--����ʧ�ܣ�ģ��δ����
 * 			 1--������
 * 			 2--������
 * 			 3--�ȴ���Ӧ
 * 			 4--�ȴ���������
 * 			 5--���ӳɹ�
 * 			 7--����ʧ��																	*/
int glnk_get_state_to_server();


/*	֪ͨ�豸״̬����ӿ�
 *  Data��һ��json�ֶ�
	SD��				��SDCard��:			"int8_t"	(0--��sd����1--��sd��)
	PTZ				��PTZ��:				"int8_t"	(0--�ޣ�1--ҡͷ��2--����ѧ�佹)
	����				��FishEye�� :			"int8_t"	(0--�ޣ�1--��)
	720ȫ��			��Panorama��:			"int8_t"	(0--�ޣ�1--��)
	��Ʒ��̬			��ProductType��:		"int8_t"	(1--IPC��2--NVR��3--���壬4--��������)
	���				��Battery��	:		"int8_t"	(0--�ޣ�1--��)
	�������			��StreamingClass��:	"int8_t"	(2--���֣�3--����)
	��Ƶ��������		��VideoEncoderType��:	"int8_t"	(4--h264��5--h265��0--���ֶ�֧��)
	�Ƿ�֧�ֻ�������	��EchoCancel��:		"int8_t"	(0--��֧�֣�1--֧��)
	�Ƿ�֧����Ƶȫ˫��	��Full_Duplex��		"int8_t"	(0--��֧�֣�1--֧��)
	ȫ������			��PanoData����			��string�� 	(ȫ���������⴮������GLNK_GetPanoData�յ���ֵ������ֵ )
	ͨ����			��Channels��			"int8_t"	(ͨ����)
//	�汾����			��SDKVersion��:		"string"	(sdk�汾�ţ���glnk_get_version���أ���һ��16�����������ں�)
 	�̼��汾			��Firmware��:			"string"	(ͬGLNK_GetVersionFirmware_Callback,��|����)
 	ȫ��ģ��汾		��PanoModelVersion��	"string"	����GLNK_DownLoadPanoModel_Callback������ģ��ʱҪ���´˲���
 													��һ��UTCʱ�䣬��ģ������ʱ��)
	.....��������չ��
 *  len:	json�ֶεĳ���
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int glnk_SendDeviceInfo(void *Data,int32_t len);


/*	������Ƶ�����ӿڣ��Ƽ�ʹ�ã�
 *  channel	��		ͨ����
 *  ismainorsub��	������--0 ������--1
 *  isIFrame��		�Ƿ�ΪI֡ 0Ϊp֡��1ΪI֡
 *  videoData��		ԭʼH264��H265����
 *  videoDataLen:	ԭʼH264��H265���ݳ���
 *  StreamTime:		����ʱ��ľ���ʱ�䣨ms���磺23:30:30:500 = (23*60*60 + 30*60 + 30)*1000 + 500
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int32_t glnk_sendvideostream(unsigned char channel,unsigned char ismainorsub,char isIframe,void *videoData,uint32_t videoLen,uint32_t StreamTime);


/*	������Ƶ�����ӿڣ��Ƽ�ʹ�ã�
 *  channel	��		ͨ����
 *  audioData��		ԭʼ����
 *  audioLen:		ԭʼ���ݳ���
 *  StreamTime:		����ʱ��ľ���ʱ�䣨ms���磺23:30:30:500 = (23*60*60 + 30*60 + 30)*1000 + 500
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int32_t glnk_sendaudiostream(unsigned char channel,void *audioData,uint32_t audioLen,uint32_t StreamTime);


/*	������Ƶ�����ӿڣ��ɣ�
 *  channel	��		ͨ����
 *  ismainorsub��	������--0 ������--1
 *  isIFrame��		�Ƿ�ΪI֡ 0Ϊp֡��1ΪI֡
 *  videoData��		ԭʼH264��H265����
 *  videoDataLen:	ԭʼH264��H265���ݳ���
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int glnk_sendvideodata( unsigned char  channel,unsigned char ismainorsub,char isIFrame,void* videoData,uint32_t videoDataLen);


/*	������Ƶ�����ӿڣ��ɣ�
 *  channel	��		ͨ����
 *  audioData��		ԭʼ����
 *  audioLen:		ԭʼ���ݳ���
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int glnk_sendaudiodata(	unsigned char channel,void* audioData,uint32_t audioDataLen);


/*	���ͻط������ӿ�
 *  channel	��		��������ͨ����GLNK_PlaybackOpen_CallBack��targetֵ��
 *  isIFrame��		�Ƿ�ΪI֡ 0Ϊp֡��1ΪI֡
 *  AudioOrVideo:	����Ƶ��־λ��1Ϊ��Ƶ��0Ϊ��Ƶ��
 *  Data��			��������
 *  Steamtime��		����ʱ�������ǰ ʱ: ��: ��: ���� ��0��ĺ��������統ǰ����2016-6-14 16:20:30:20��ʱ������������գ�Ϊ ��16*60*60 + 20*60 + 30��*1000 + 20
 *  DataLen��		�������ݳ���
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int32_t glnk_sendplaybackdata(	unsigned char channel, char isIFrame, char AudioOrVideo,void* Data, uint32_t StreamTime,uint32_t DataLen);


/*	�����ط�����
 *  channel:		��������ͨ����GLNK_PlaybackOpen_CallBack��targetֵ��
 * 	retutn : 0--ʧ��   1--�ɹ�												*/
int32_t glnk_stopSendplaybackdata(unsigned char channel);


/*	���������ӿ�
 * 	ifname : �������ƣ��ַ�����
 * 	retutn : 0--�ɹ�   ����--ʧ��															*/
int glnk_set_ifname(char* ifname);


/*	��ȡ�ӷ������õ���utcʱ��
 * 	retutn : 0--ʧ��   ����--utcʱ��														*/
int32_t glnk_get_utc_from_server();


/*	�ļ����ؽӿ�
 *  sessionID �� 	�������ݵ�ID��(GLNK_DownLoadFileConfig_Callback����)
 *  data��    			����ָ��
 *  datalen��		���ݳ���
 *  start_pos��		�������ļ��е�ƫ�ƵĿ�ʼλ�ã��ֽڣ�
 *  end_pos��		�������ļ��е�ƫ�ƵĽ���λ�ã��ֽڣ�
 * 	retutn : 0--ʧ��   -1--���ͳ�ʱ���´οɼ���������һ����   >0--Ϊ�������ݳ��ȣ����ͳɹ�	*/
int32_t glnk_SendDownLoadFileData(int32_t sessionID,char * data,int32_t datalen,int32_t start_pos,int32_t end_pos);


/*	�����ļ����ؽӿ�
 * 	sessionID ���������ݵ�ID��
 * 	filesize �� �ļ��Ĵ�С
 * 	retutn : 0--ʧ��   filesize--�ɹ�														*/
int32_t glnk_FinishDownLoadFileData(int32_t sessionID, int filesize);

/*	�ļ����ؽӿ�
 *  sessionID �� 	�������ݵ�ID��(GLNK_DownLoadPanoModel_Callback����)
 *  filename:		�ļ���
 *  data��    			����ָ��
 *  datalen��		���ݳ��� (ÿƬ�̶�1024�ֽڣ�ֱ�����һƬ)
 *  size:			�ļ��ܴ�С
 * 	retutn : 0--ʧ��   -1--���ͳ�ʱ���´οɼ���������һ����   >0--Ϊ�������ݳ��ȣ����ͳɹ�	*/
int32_t glnk_SendPanoModelData(int32_t sessionID,char *filename,char *data,int32_t datalen,int32_t size);

/*	�����ļ����ؽӿ�
 * 	sessionID ��		�������ݵ�ID��
 * 	filename:		�ļ���
 * 	size:			�ļ����ܴ�С - ���سɹ���0 -- ʧ�ܽ�������
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_FinishPanoModelData(int32_t sessionID,char *filename,int size);

/*	�ϴ���ʷ�����ӿ�
 * 	FileName �� ͼƬ��¼���ļ���������·����
 * 	FileType �� �ļ����� 1 ¼�� 2 ͼƬ
 * 	Channel��   ��Ƶͨ��
 * 	AlarmType��	�������ͣ�433������������һ��6λ��16������ɣ���xx(433���̱��)xx(433������)xx(433������ֵ)����ͨ���ͼ�glnk_push_alarm��
 * 	flag��		һ�������п��ܻ��ж��ͼƬ��¼���ļ�����ÿ�ε��ô˺����ϴ�һ���ļ�������һ�α��������һ�ε��ú���ʱflag��Ϊ1������Ϊ0
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_AlarmFileStoreInfo(const char *FileName, int32_t FileType, int32_t Channel, uint32_t AlarmType, int32_t flag);


/*	��������433�豸��ַ��ӿ�
 * 	Sid ��			��GLNK_433DeviceLearnCallback������Sid��Ӧ
 * 	DeviceType �� 	ѧϰ433�豸����
 * 	CodeAddr��		433�豸��ַ�루ʮ�������ַ�����ʽ��
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_Send433DeviceCodeAddr(int32_t Sid, int32_t DeviceType, char *CodeAddr);


/*	���ûطŻ�����֡����С�ӿڣ��ɲ����ã�
 * 	value ��		֡����С														*/
void SetPlayBackFrameBuffer(int value);

/*	�豸�����Ͽ��ӿ�
 * 	sid ��		GLNK_PwdAuthWithChannel_Callback���ص�sid
 * 	reserve �� 	����λ���ɺ���
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_CloseSession(int32_t sid,int32_t reserve);


/*	�û���ȡ��ǰ�������ӿ�
 * 	channel ��			��Ҫ��ѯ��ͨ����
 * 	connectionnum �� 	��ѯ��ͨ���ŵ�ǰ��������
 * 	connectionsum ��	�豸��ǰ���е�������
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_GetSessionNum(int32_t channel,int32_t *connectionnum,int32_t *connectionsum);


/*	������Ƶ��ʽ�ӿ�
 * 	channel ��		��Ƶͨ����
 * 	videoinfo �� 	��Ƶ��ʽ
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int glnk_SetVideoDataFormat(unsigned char  channel, GLNK_VideoDataFormat* videoinfo);


/*	���÷�����433����״̬�ӿ�
 * 	AddrCode ��		�豸�ĵ�ַ��
 * 	isopen �� 		����״̬��0--�أ�1--��
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int32_t glnk_Set433Switch(char *AddrCode,char isopen);


/*	����ֱ������Ƶ�����ӿ�
 *  channel	��		ͨ����
 *  ismainorsub��	������--0 ������--1
 *  videoData��		ԭʼH264��H265����
 *  videoDataLen:	ԭʼH264��H265���ݳ���
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int glnk_sendRtmpvideodata( unsigned char  channel,unsigned char ismainorsub,char* videoData,uint32_t videoDataLen);


/*	����ֱ������Ƶ�����ӿ�
 *  channel	��		ͨ����
 *  audioData��		ԭʼ����
 *  audioLen:		ԭʼ���ݳ���
 * 	retutn : ���ݳ���--�ɹ�   0--ʧ��														*/
int glnk_sendRtmpaudiodata(	unsigned char channel,char* audioData,uint32_t audioDataLen);

/*	�̼����������ӿڣ��ѷ�������Ϊappѡ��������
 * 	type��	 1--��ʼ�ɹ�
 * 	retutn : 0--ʧ��   1--�ɹ�														*/
int glnk_upload_Control(int type);


/*	��ӡ���ӵ�ǰ���������ѷ�����													*/
void glnk_print_sessions();

/*	�ϴ�ͼƬ���ݽӿڣ��������ͱ����ӿں���ܵ��ô˽ӿڣ��ѷ����������ϴ�ͼƬ��
 * 	DataPtr ��		�ϴ�����ָ��
 * 	DataLen �� 		�ϴ����ݳ���
 * 	retutn : 0--ʧ��   1--�ɹ���ʼ�ϴ�												*/
int32_t glnk_UploadData(char *DataPtr, int32_t DataLen);


/*
�豸�ƴ�ʱ�������豸����ȷ����ʱ�������ܵ���,��������
tz �������ʱ����ƫ��ֵ,�������Ǹ���������������
return -1 --ʧ��
*/
int glnk_SetTimeZoneFunc(int tz);


/*
ǿ��I֡ע��ӿ�,�����û���¼���л�����ʱ��ֱ��ǿ��I֡��
void(*fun)(int channel, int videotype)��
channel Ϊͨ���� videotype Ϊ�������ͣ�������������
*/
void SetInsertIFrameFun(void(*fun)(int channel, int videotype));


#ifdef __cplusplus
}
#endif

#endif // __GOO_LINK_API_H__
