/*!
*****************************************************************************
** FileName     : gk_cms_protocol.h
**
** Description  : api about cms protocol.
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Date         : 2015-9-9, create.
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/

#ifndef _GK_CMS_PROTOCOL_H_
#define _GK_CMS_PROTOCOL_H_

#ifdef __cplusplus
extern "C"
{
#endif

//#include "DMSType.h"

#ifndef _HAVE_TYPE_BYTE
#define _HAVE_TYPE_BYTE
	typedef unsigned char       BYTE;
#endif

#ifndef _HAVE_TYPE_Bool
#define _HAVE_TYPE_Bool
typedef unsigned char       Bool;
#endif

#ifndef BOOL
#ifndef _HAVE_TYPE_BOOL
#define _HAVE_TYPE_BOOL
	typedef int					BOOL;
#endif
#endif

#ifndef _HAVE_TYPE_WORD
#define _HAVE_TYPE_WORD
	typedef unsigned short      WORD;
#endif

#ifndef _HAVE_TYPE_DWORD
#define _HAVE_TYPE_DWORD
	//typedef unsigned long		DWORD;
	typedef unsigned int         DWORD;	//xqq
	typedef unsigned long		ULONG;
#endif

#ifndef _HAVE_TYPE_ULONGLONG
#define _HAVE_TYPE_ULONGLONG
	typedef unsigned long long		ULONGLONG;
#endif

#ifndef _HAVE_TYPE_LONG
#define _HAVE_TYPE_LONG
	typedef long				LONG;
#endif

#ifndef _HAVE_TYPE_UINT
#define _HAVE_TYPE_UINT
	typedef unsigned int		UINT;
#endif

#ifndef _HAVE_TYPE_VOID
#define _HAVE_TYPE_VOID
	typedef void				VOID;
#endif

#ifndef _HAVE_TYPE_DWORD64
#define _HAVE_TYPE_DWORD64
	typedef unsigned long long  DWORD64;
	typedef unsigned long long	QWORD;
#endif

#ifndef _HAVE_TYPE_UINT8
#define _HAVE_TYPE_UINT8
	typedef unsigned char   	UINT8;
#endif


#ifndef _HAVE_TYPE_UINT16
#define _HAVE_TYPE_UINT16
	typedef unsigned short 		UINT16;
#endif

#ifndef _HAVE_TYPE_UINT32
#define _HAVE_TYPE_UINT32
	typedef unsigned int    	UINT32;
#endif

#ifndef _HAVE_TYPE_HWND
#define _HAVE_TYPE_HWND
	typedef unsigned int 		HWND;
#endif

#ifndef _HAVE_TYPE_WPARAM
#define _HAVE_TYPE_WPARAM
	typedef unsigned int    	WPARAM;
#endif

#ifndef _HAVE_TYPE_LPARAM
#define _HAVE_TYPE_LPARAM
	typedef unsigned long   	LPARAM;
#endif

#ifndef _HAVE_TYPE_SINT64
#define _HAVE_TYPE_SINT64
	typedef signed long long	SINT64;
#endif

#ifndef _HAVE_TYPE_UINT64
#define _HAVE_TYPE_UINT64
	typedef unsigned long long 	UINT64;
#endif

#ifndef _HAVE_TYPE_Int
#define _HAVE_TYPE_Int
	typedef int		Int;
#endif

#ifndef _HAVE_TYPE_Char
#define _HAVE_TYPE_Char
	typedef char	Char;
#endif

//�����������
#define		JBNV_FLAG		9000
#define		JB_STANDARD_ALARM_MSG_HEADER	0x200000
/* 6, video decode use ffmpeg; 4 hisi */
#define VIDEO_DECODE_H264_V2		4
#define VIDEO_DECODE_H265			1006
#define DMS_MAX_STREAMNUM 3


typedef enum {
    NETCMD_NOT_JBDATA = 5005,
    NETCMD_NOT_LOGON = 5006,
    NETCMD_RECV_ERR = 5018,
    NETCMD_CHANNEL_INVALID = 5019,
} GK_CMS_NETCMD;

typedef enum {
    WAVE_FORMAT_ALAW = 0x0006,
    WAVE_FORMAT_MPEGLAYER3 = 0x0055,
    WAVE_FORMAT_G722_ADPCM = 0x0065,
    WAVE_FORMAT_G711A = 0x003E,
    WAVE_FORMAT_ADPCM = 0x0002,
    WAVE_FORMAT_G726_ADPCM = 0x0064,	//��˼�����ADPCM
} GK_CMS_WAVE_TYPE;

typedef enum {
	JBCMD_SERVER_INFO =		5002,		//��������Ϣ
	JBCMD_PASSWORD_UNMATCH =	5003,		//�������
	JBCMD_MAX_LINK =			5004,		//�������������
	JBCMD_KEEP_ALIVE =		5001,		//��������
	JBCMD_SERVER_MSG =		1006,		//��������Ϣ
	JBCMD_CH_CONNECT_BACK =	5016,		//ͨ����������
	JBCMD_PLAY_CLOSE =		5009,		//�ر�ͨ��
	JBCMD_MULTI_OPEN =		5008,		//�򿪶ಥ
	JBCMD_PLAYBACK_CMD =	5007,
} emJBCMD;


typedef enum {
	JB_MSG_VIDEOLOST =					0x40000001,  //��Ƶ��ʧ
	JB_MSG_MOVEDETECT =                  0x40000002,  //�ƶ�����
	JB_MSG_SENSOR_ALARM =              0x40000003,  //̽ͷ����
	JB_MSG_RESETSERVER =                 0x40000004,  //��������ΪĳЩ�������ı䣬��Ҫ��������
	JB_MSG_JPEGSNAP =                   0x40000005,  //JPEGץͼ
	JB_MSG_UPGRADE =                    0x40000006,  //��������֪ͨ
	JB_MSG_CRCERROR =                   0x40000007,  //����CRC��
	JB_MSG_SERVER_BUSY =                 0x40000008,  //������æ������ʧ��
	JB_MSG_SERVER_BREAK =                0x40000009,  //�������ж�����
	JB_MSG_CHANNEL_BREAK =               0x4000000A,  //ͨ���ж�����
	JB_MSG_TALK_REQUEST =				0x4000000B,	//Զ�̷���������Խ�
	JB_MSG_UPGRADEOK =          		0x4000000C,  //�������
	JB_MSG_VIDEORESUME =					0x4000000D,	//��Ƶ�ָ�
	JB_MSG_COMDATA =						0x4000000E,	//��������
	JB_MSG_USERDATA =					0x4000000F,	//�û�����
	JB_MSG_DISK_REMOVE	=				0x40000010,	//���̱�̽�����Ƴ�
	JB_MSG_ALARM_SNAP =					0x40000011,
	JB_MSG_FILE_DATA =					0x40000012,	//ǰ�˷��ص��ļ������ӽṹΪ��JBNV_DVR_FILE_DATA
	JB_MSG_DISK_ERROR =					0x40000013,	//���̴��ļ����ɶ�д��
	JB_MSG_TEL_ALARM =					0x40000014,	//�绰�澯����
	JB_MSG_CHANNEL_RECONN_SUCCESS =		0x40000015,	//ͨ�����ӳɹ�
	//Add 2009-02-24
	JB_MSG_TEMPHUM_LOST_ALARM =			0x40000016,	//TempHum Device is Off
	JB_MSG_TEMPHUM_TEMP_LOWER =			0x40000017,	//TempHum Temp is Lower
	JB_MSG_TEMPHUM_TEMP_UPPER =			0x40000018,	//TempHum Temp is Upper
	JB_MSG_TEMPHUM_HUM_LOWER =			0x40000019,	//TempHum Hum is Lower
	JB_MSG_TEMPHUM_HUM_UPPER =			0x4000001A,	//TempHum Hum is Upper
	JB_MSG_POWER_220V_OFF =				0x4000001B,	//220V Power is Off
	JB_MSG_POWER_48V_OFF =				0x4000001C,	//220V Power is Off
	JB_MSG_POWER_DEVICE_LOST =			0x4000001D,  //Power Device is Off
	JB_MSG_FILE_NAME_DATA =				0x4000001E, //ǰ�˷��ص��ļ���  ���ӽṹ��JBNV_FILE_DATA_INFO
	JB_MSG_ERROR_DEVICE =				0x4000001F, //��֧�ֵ�ǰ���豸
	JB_MSG_IVS_DATA =					0x40000020, //�̻𱨾�
	JB_MSG_SENSOR_RESUME =				0x40000021, //̽ͷ�����ָ�
	JB_MSG_SERVER_LINK_OK =              0x40000022, //���������ӳɹ�
	#ifdef MODULE_SUPPORT_RF433
	JB_MSG_RF433_DEV_ALARM_BASENUM =	0x50000000,//433�豸������ʼID��
	JB_MSG_RF433_DEV_ALARM_MAXNUM =		0x50000032,//433�豸�������ID��
	#endif
} emMsg;


typedef enum emJBERR
{
	JB_SUCCESS =                        0x00000000,
	JBERR_BASE	 =						0x10000000,
	JBERR_PASSWORD_UNMATCH = 			(JBERR_BASE|0x01),			//���벻ƥ��
	JBERR_TIME_OVER = 					(JBERR_BASE|0x02),			//������ʱ
	JBERR_INVALID_PARAM = 				(JBERR_BASE|0x03),			//��Ч����
	JBERR_MAX_LINK = 					(JBERR_BASE|0x04),			//�������������
	JBERR_INVALID_HANDLE = 				(JBERR_BASE|0x05),			//����Ƿ������
	JBERR_INVALID_COMMAND = 			(JBERR_BASE|0x06),			//�������ܵ�����
	JBERR_SENDCMD_FAILD	= 				(JBERR_BASE|0x07),			//������������ʧ��
	JBERR_GETCONFIG_FAILD = 			(JBERR_BASE|0x08),			//ȡ����������ʧ��
	JBERR_NO_LOGON = 					(JBERR_BASE|0x09),			//û�е�¼
	JBERR_ALLOC_FAILD = 				(JBERR_BASE|0x0a),			//�����ڴ�ʧ��
	JBERR_INVALID_NETADDRESS = 			(JBERR_BASE|0x0b),			//��Ч�Ļ��޷������������ַ
	JBERR_FILE_CRC32 =					(JBERR_BASE|0x0C),			//�ļ�У���
	JBERR_SOFTVER_ERR =					(JBERR_BASE|0x0D),			//����汾���ͣ��޷�У�������ļ�
	JBERR_CPUTYPE_ERR =					(JBERR_BASE|0x0E),			//�����ļ��������ڴ�CPU���͵�����
	JBERR_ERROR_10054 =					(JBERR_BASE|0x0F),			//���ӱ�������ǿ�ȹر�!
	JBERR_ERROR_10061 =					(JBERR_BASE|0x10),			//������û����ָ���˿ڴ򿪷���!
	JBERR_ERROR_10060 =					(JBERR_BASE|0x11),			//û�з���ָ��IP�ķ�����!
	JBERR_ERROR_10065 =					(JBERR_BASE|0x12),			//����δ׼����!
	JBERR_INITSURFACE =					(JBERR_BASE|0x13),			//��ʼ����ʾ�������
	JBERR_UNSUPPORT	=					(JBERR_BASE|0x14),			//��������֧�ִ˹���
	JBERR_TALK_REJECTED =				(JBERR_BASE|0x15),			//�Խ����󱻷������ܾ�
	JBERR_TALK_INITAUDIO =			    (JBERR_BASE|0x16),			//�����Խ�ʱ��Ƶ��ʼ��ʧ��
	JBERR_OPEN_FILE =					(JBERR_BASE|0x17),			//���ļ���
	JBERR_BIND_PORT =					(JBERR_BASE|0x18),			//�󶨱��ض˿�ʧ��
	JBERR_NO_FILE	=					(JBERR_BASE|0x19),			//û���ҵ��ļ�
	JBERR_NOMORE_FILE	=				(JBERR_BASE|0x1A),			//û�и����ļ�
	JBERR_FILE_FINDING	=				(JBERR_BASE|0x1B),			//���ڲ���
	JBERR_DISK_NOTEXIST =				(JBERR_BASE|0x1C),			//��ʽ��/������Ӳ�̲�����
	JBERR_FILE_ERROR =					(JBERR_BASE|0x1D),			//�ļ�����ȷ���߲�ƥ��˷�����
	JBERR_UNINITOBJ =					(JBERR_BASE|0x1E),			//����û�г�ʼ�����Ժ�����
	JBERR_UNKNOW_SERVER =				(JBERR_BASE|0x1F),			//�������޷�ʶ��
	JBERR_CHANNEL_NOT_OPEN =			(JBERR_BASE|0x20),			//ͨ��û�д򿪣�����ʧ��
	JBERR_INVALID_FILE =				(JBERR_BASE|0x21),			//
	JBERR_ENCRYPT_IC_NO_FIND =			(JBERR_BASE|0x22),			//������û���ҵ�����IC
	JBERR_ENCRYPT_IC_NO_MATCH =			(JBERR_BASE|0x23),			//����IC��ƥ��
	JBERR_RTSP_GET_DESCRIBE =			(JBERR_BASE|0x24),			//��ȡRTSP����ʧ��
	JBERR_RTSP_SETUPAUDIO =				(JBERR_BASE|0x25),			//RTSP������Ƶʧ��
	JBERR_RTSP_SETUPVIDIO =				(JBERR_BASE|0x26),			//RTSP������Ƶʧ��
	JBERR_RTSP_NOSTREAM	=				(JBERR_BASE|0x27),			//RTSPû��������
	JBERR_RTSP_PLAY	=					(JBERR_BASE|0x28),			//RTSP��������ʧ��
	JBERR_IP_ERROR	=					(JBERR_BASE|0x29),			//����IP���󣬲�������ָ��������
	JBERR_SERVER_UPDATAING	=			(JBERR_BASE|0x2A),			//���������������������ܿͻ��˵����Ӳ���
} GK_JBERR_TYPE_E;




/* �������©���� */
//����
#define GK_NET_CMD_REBOOT           0x00000001
//�ָ���������
#define GK_NET_CMD_RESTORE			0x00000002
//�����û����õ�flash��
#define GK_NET_CMD_SAVE             0x00000003
//ץͼ
#define GK_NET_CMD_SNAP             0x00000004
#define GK_NET_CMD_NOTIFY           0x00200000
//��������������������
#define GK_NET_CMD_CONNECT_CMS      0x00030308

#define CMD_GETSYSTIME      0x10000000

typedef enum emSetServerCfg
{
	CMD_REBOOT	=				0x00000001,		//Param:
	CMD_RESTORE	=				0x00000002,		//Param:
	CMD_UPDATEFLASH	=			0x00000003,		//Param:
	CMD_SNAPSHOT =				0x00000004,		//Param:int
	CMD_SETSYSTIME =				0x00000005,		//Param:SYSTEMTIME
	CMD_SET_OSDINFO =				0x00000006,		//Param:JB_CHANNEL_OSDINFO
	CMD_SET_SHELTER =				0x00000007,		//Param:JB_CHANNEL_SHELTER
	CMD_SET_LOGO =				0x00000008,		//Param:JB_CHANNEL_LOGO
	CMD_SET_CHANNEL_CONFIG =		0x00000009,		//Param:JB_CHANNEL_CONFIG
	CMD_SET_COLOR =				0x0000000A,		//Param:JB_CHANNEL_COLOR
	CMD_SET_MOTION_DETECT =		0x0000000B,		//Param:JB_CHANNEL_MOTION_DETECT
	CMD_SET_SENSOR_ALARM =		0x0000000C,		//Param:JB_SENSOR_ALARM
	CMD_SET_VIDEO_LOST =			0x0000000D,		//Param:JB_CHANNEL_VIDEO_LOST
	CMD_SET_COMINFO =				0x0000000E,		//Param:JB_SERVER_COMINFO
	CMD_SET_USERINFO =			0x0000000F,		//Param:JB_SERVER_USER
	CMD_SET_NETWORK =				0x00000010,		//Param:JB_SERVER_NETWORK
	CMD_UPLOAD_PTZ_PROTOCOL =		0x00000011,		//Param:JB_UPLOAD_PTZ_PROTOCOL
	CMD_SEND_COMDATA =			0x00000012,		//Param:JB_COM_DATA
	CMD_SET_FTPUPDATA_PARAM =		0x00000013,		//Param:JB_FTPUPDATA_PARAM
	CMD_CLEAR_ALARM_OUT	=		0x00000014,		//Param:
	CMD_SET_SERVER_COM2	=		0x00000015,		//Param:JB_SERVER_COM2INFO
	CMD_SET_ALARM_OUT =			0x00000016,		//Param:JBNV_ALARM_OUT_INFO
	CMD_SET_NOTIFY_SERVER =		0x00000017,		//Param:JB_NOTIFY_SERVER
	CMD_SET_PPPOE_DDNS =			0x00000018,		//Param:JB_PPPOE_DDNS_CONFIG
	CMD_SET_SENSOR_STATE =		0x00000019,		//Param:JBNV_SENSOR_STATE
	CMD_SET_SERVER_RECORD =		0x00000020,		//Param:JB_SERVER_RECORD_SET
	CMD_RECORD_BEGIN =			0x00000021,		//Param:
	CMD_RECORD_STOP =				0x00000022,		//Param:
	CMD_SET_CENTER_INFO =			0x00000023,		//Param:JB_CENTER_INFO
	CMD_UPDATE_CENTER_LICENCE =	0x00000024,
	CMD_SET_CHANNEL_ALARM_CONFIG = 0x00000025,		//Param:JB_CHANNEL_ALARM_CONFIG
	CMD_SET_EMAIL_PARAM =			0x00000026,		//Param:JBNV_EMAIL_PARAM
	CMD_SET_COMMODE	=			0x00000027,		//Param:JBNV_SERVER_COMMODE
	CMD_SET_3322DDNS =			0x00000028,		//Param:JBNV_3322DDNS_CONFIG
	CMD_STOP_FILE_DOWNLOAD =		0x00000029,
	CMD_SET_NVD_SENSOR_ALARM =	0x0000002A,		//Param:JB_NVD_SENSOR_ALARM_SET
	CMD_SET_WIFI =				0x00000046,		//Param:JB_WIFI_CONFIG
	CMD_SET_TDSCDMA	 =			0x00000047,		//Param:JB_TDSCDMA_CONFIG
	CMD_SET_PERIPH_CONFIG =		0x00000048,		//Param:JB_PERIPH_CONFIG
	CMD_SET_TEL_ALARM = 	 		0x00000049,		//Param:JB_TEL_ALARM_SET
	CMD_SET_TEMPHUM_SENSOR =		0x0000004A,		//Param:JB_TEMP_HUM_SENSOR_CONFIG
	CMD_SET_POWER_DEVICE =		0x0000004B,		//Param:JB_POWER_DEVICE_CONFIG
	CMD_SET_RECORD_CONFIG =		0x0000004C,		//Param:JB_SERVER_RECORD_CONFIG
	CMD_SET_UPNP =				0x0000004D, 	//Param:JB_UPNP_CONFIG
	CMD_SET_PLAYBACK_NAME =		0x0000004E,		//Param: char *csPlayBackFileName
	CMD_SET_PLAYBACK_TIME =		0x0000004F,		//Param:JBNV_TIME
	CMD_PLAYBACK_VODACTION =		0x00000050,		//Param:JBNV
	CMD_SET_MOBILE_CENTER =		0x00000051,		//Param:JB_MOBILE_CENTER_INFO
	CMD_SET_CHANNEL_SUBSTREAM_CONFIG = 0x00000052,	//Param:JB_CHANNEL_CONFIG
	CMD_FDISK_DISK =				0x00000061,		//Param:JBNV_FDISK_DISK
	CMD_FORMAT_DISK =	 			0x00000062,		//Param:JBNV_FORMAT_DISK
	CMD_REQUEST_IFRAME =				0x00000063, //Param:JB_CHANNEL_FRAMEREQ
	CMD_SET_CENTER_INFO_EX =			0x00000064, //Param:JBNV_NXSIGHT_SERVER_ADDR_EX
	CMD_GET_CENTER_INFO_EX =			0x00000065, //Param:JBNV_NXSIGHT_SERVER_ADDR_EX
	CMD_SET_AUDIO_PARA =				0x00000066, //Param:HI_CFG_AUDIO_ATTR_S
	CMD_GET_AUDIO_PARA	=			0x00000067, //Param:HI_CFG_AUDIO_ATTR_S
	CMD_FORMAT_DISK_NEW =				0x00000068, //Param:HI_HDI_FORMAT
	CMD_TEST_SPEAKER    =           0x00000069, //Param:
	CMD_TEST_IRCUT      =           0x00000070, //Param:
	CMD_TEST_GET_TEST_RESULT        =   0x00000071, //Param: NET_TEST_RESULT_INFO
	CMD_TEST_SET_TEST_RESULT        =   0x00000072, //Param: NET_TEST_RESULT_INFO
    CMD_GET_FAC_CFG                 =   0x00000073, //Param: DMS_NET_FAC_CFG
    CMD_SET_FAC_CFG                 =   0x00000074, //Param: DMS_NET_FAC_CFG
    CMD_GET_AF_OFFSET               =   0x00000075, //Param: NET_AF_OFFSET_INFO
	CMD_SET_AF_OFFSET               =   0x00000076, //Param: NET_AF_OFFSET_INFO
    CMD_TEST_WHITE                  =   0x00000077, //Param: NET_AF_OFFSET_INFO
    CMD_TEST_CMD                    =   0x00000078, //Param: NET_AF_OFFSET_INFO
} GK_CMD_TYPE_E;


#if 0
typedef enum emSetServerCfg {
    DMS_NET_SET_EXCEPTIONCFG = 0x010901, //�쳣��������
    DMS_NET_SET_RESTORECFG = 0x020301,  //�ָ�����ֵ����
    DMS_NET_SET_SAVECFG = 0x020303, //��������
    DMS_NET_CMD_ALARMOUT_CONTROL = 0xA0200008, //�����������
    DMS_NET_SET_ALARMOUTCFG = 0x010421, //���ñ����������
    DMS_NET_SET_ALARMINCFG = 0x010411,  //���ñ����������
    DMS_NET_SET_CRUISE_INFO = 0x010429, //����һ��Ѳ��
    DMS_NET_SET_SHELTERCFG = 0x010231, //ͼ���ڵ�
    //DMS_NET_CMD_PTZ_CONTROL = 0xA0200007, //��̨����
    DMS_NET_CMD_PTZ_CONTROL = 0x00000050, //��̨����
    DMS_NET_SET_DEVICECFG = 0x010001, //�豸��Ϣ
    DMS_NET_SET_WANWIRELESSCFG_V2 = 0x010173, //����������
    DMS_NET_SET_PICCFG = 0x010201, //����ͼ��ѹ������
    DMS_NET_SET_COLORCFG_SINGLE = 0x010223, //��������ĳ��ͼ��ɫ�ʲ���,
    DMS_NET_SET_COLOR_BLACK_DETECTION = 0x010227, //���ò�ת�ڼ�����
    DMS_UPNP_SET_CONFIG = 0x010181, //����UPNP����
    DMS_NET_SET_FTPCFG = 0x010151, //����FTP����
    DMS_NET_SET_OSDCFG = 0x010211, //����ͼ���ַ����Ӳ���
    DMS_SET_MOBILE_CENTER_INFO = 0x010191, //�����ֻ�ƽ̨����
    DMS_NET_SET_RTSPCFG = 0x020305, //rtsp����
    DMS_NET_SET_NETCFG = 0x010101, //�����������
    DMS_NET_SET_PPPOECFG = 0x010121, //����PPPOE����
    DMS_NET_SET_P2PCFG = 0x020411, //����P2P����
    DMS_DEV_SET_DEVICEMAINTAINCFG = 0x020601, //�豸ά��
    DMS_NET_SET_EMAILCFG = 0x010141,  //����EMAIL����
    DMS_NET_SET_RECORDCFG = 0x010271, //����ͼ��¼�����
    DMS_NET_SET_RECORDSTREAMMODE = 0x010274l, //����¼��������
    DMS_NET_SET_RECORDMODECFG = 0x010281, //����ͼ���ֶ�¼�����
    DMS_NET_SET_SNMPCFG = 0x040701, //����SNMP����
    DMS_NET_CMD_SET_SYSTIME = 0xA0200014, //����ϵͳʱ��
    DMS_NET_SET_NTPCFG = 0x010521, //����NTP ����
    DMS_NET_DEF_NTPCFG = 0x010522, //Ĭ��NTP ����
    DMS_NET_SET_ZONEANDDSTCFG = 0x010511, //ʱ������ʱ��
    DMS_NET_SET_SNAPTIMERCFG = 0x010301, //����ͼ��ʱץ�Ĳ���
    DMS_NET_SET_SNAPEVENTCFG = 0x010303, //����ͼ���¼�ץ�Ĳ���
    DMS_NET_SET_RS232CFG = 0x010311, //����232���ڲ���
    DMS_NET_GET_DEF_SERIAL = 0x010312, //Ĭ�ϴ��ڲ���
    DMS_NET_SET_USERCFG = 0x010801, //�����û�����
    DMS_NET_SET_USERGROUPCFG = 0x010811, //�����û������
    DMS_NET_SET_MOTIONCFG = 0x010241, //����ͼ���ƶ�������
    DMS_NET_DEF_MOTIONCFG = 0x010242, //Ĭ��ͼ���ƶ�������
    DMS_NET_SET_VLOSTCFG = 0x010251, //����ͼ����Ƶ��ʧ����
    DMS_NET_DEF_VLOSTCFG = 0x010252, //Ĭ��ͼ����Ƶ��ʧ����
    DMS_NET_SET_HIDEALARMCFG = 0x010261, //����ͼ���ڵ���������
    DMS_NET_DEF_HIDEALARMCFG = 0x010262, //Ĭ��ͼ���ڵ���������
    DMS_NET_SET_WIFICFG = 0x010161, //����WIFI����
    DMS_NET_SET_DDNSCFG = 0x010131, //����DDNS����
    DMS_NET_SET_WIFI_WPS_START = 0x010163, //����WPS��ʼ
    DMS_NET_CMD_REBOOT = 0xA020000C, // �����豸
    DMS_NET_CMD_SHUTDOWN = 0xA020000D, // �豸�ػ�
    DMS_NET_CMD_SNAPSHOT = 0xA020000E, // ץ��ͼƬ
} GK_CMS_CMD_SET;

typedef enum emGetServerCfg {
    DMS_NET_GET_EXCEPTIONCFG = 0x010900,  //��ȡ�쳣����
    DMS_NET_GET_RESTORECFG = 0x020300, //�ָ�����ֵ����
    DMS_NET_GET_DEF_RESTORECFG = 0x020302, //��ȡĬ�ϻָ�����ֵ����
    DMS_NET_GET_ALARMOUTCFG = 0x010420, //��ȡ�����������
    DMS_NET_GET_ALARMOUT_STATE = 0xA0200009, // ��ȡ�������״̬
    DMS_NET_GET_ALARMINCFG = 0x010410, //��ȡ�����������
    DMS_NET_GET_ALL_PRESET = 0x010427, //��ȡ���������õ�Ԥ�õ����Ϣ
    DMS_NET_GET_CRUISE_CFG = 0x010428, //��ȡͨ���������Ѿ����õ�Ѳ����
    DMS_NET_GET_SHELTERCFG = 0x010230, //��ȡͼ���ڵ�����
    DMS_NET_GET_DEVICECFG = 0x010000, //��ȡ�豸����
    DMS_NET_GET_DEF_DEVICECFG = 0x010003, //����Ĭ���豸����
    DMS_NET_GET_WANWIRELESSCFG_V2 = 0x010172, //��ȡ���������߲���
    DMS_NET_GET_SUPPORT_STREAM_FMT = 0x010202, // //��ȡϵͳ֧�ֵ�ͼ������
    DMS_NET_GET_PICCFG = 0x010200, //��ȡͼ��ѹ������
    DMS_NET_GET_DEF_PICCFG = 0x010203, //��ȡĬ��ͼ��ѹ������
    DMS_NET_GET_COLOR_SUPPORT = 0x010222, //��ȡͼ��ɫ�ʲ���
    DMS_NET_GET_ENHANCED_COLOR_SUPPORT = 0x010224, //��ȡͼ��ɫ��֧�ָ߼�����
    DMS_NET_GET_ENHANCED_COLOR = 0x010226, //��ȡͼ��ɫ�ʸ߼�����
    DMS_NET_GET_COLOR_BLACK_DETECTION = 0x010228, //��ȡ��ת�ڼ�����
    DMS_NET_GET_COLORCFG = 0x010220, //��ȡͼ��ɫ�ʲ���
    DMS_NET_GET_DEF_COLORCFG = 0x010225, //��ȡͼ��ɫ�ʲ���
    DMS_UPNP_GET_CONFIG = 0x010180, //��ȡUPNP����
    DMS_NET_GET_FTPCFG = 0x010150, //��ȡFTP����
    DMS_NET_GET_OSDCFG = 0x010210, //��ȡͼ���ַ����Ӳ���
    DMS_GET_MOBILE_CENTER_INFO = 0x010190, //��ȡ�ֻ�ƽ̨����
    DMS_NET_GET_NETCFG = 0x010100, //��ȡ�������
    DMS_NET_GET_DEF_NETCFG = 0x010103, //��ȡĬ���������
    DMS_NET_GET_RTSPCFG = 0x020304, //��ȡRTSP����
    DMS_NET_GET_PPPOECFG = 0x010120, //��ȡPPPOE����
    DMS_NET_GET_P2PCFG = 0x020410, //��ȡP2P����
    DMS_DEV_GET_DEVICEMAINTAINCFG = 0x020600, //��ȡ�豸ά��
    DMS_NET_GET_LOGINFO = 0x040500, //��ȡLOG��Ϣ
    DMS_NET_GET_REC_DAY_IN_MONTH = 0x050101,
    DMS_NET_GET_EMAILCFG = 0x010140, //��ȡEMAIL����
    DMS_NET_GET_DEF_RECORDCFG = 0x010272, //��ȡĬ�ϵ�¼�����
    DMS_NET_GET_RECORDCFG = 0x010270, //��ȡͼ��¼�����
    DMS_NET_GET_RECORDSTREAMMODE = 0x010273,//��ȡ¼��������
    DMS_NET_GET_RECORDMODECFG = 0x010280, //��ȡͼ���ֶ�¼�����
    DMS_NET_GET_SNMPCFG = 0x040700, //��ȡsnmp����
    DMS_NET_GET_ZONEANDDSTCFG = 0x010510, //��ȡʱ������ʱ�Ʋ���
    DMS_NET_GET_NTPCFG = 0x010520, ////��ȡNTP ����
    DMS_NET_GET_SNAPTIMERCFG = 0x010300,//��ȡͼ��ʱץ�Ĳ���
    DMS_NET_GET_SNAPEVENTCFG = 0x010302, //��ȡͼ���¼�ץ�Ĳ���
    DMS_NET_GET_RS232CFG = 0x010310, //��ȡ232���ڲ���
    DMS_NET_GET_USERCFG = 0x010800, //��ȡ�û�����
    DMS_NET_GET_MOTIONCFG = 0x010240, //��ȡͼ���ƶ�������
    DMS_NET_GET_VLOSTCFG = 0x010250, //��ȡͼ����Ƶ��ʧ����
    DMS_NET_GET_HIDEALARMCFG = 0x010260, //��ȡͼ���ڵ���������
    DMS_NET_GET_WIFI_SITE_LIST = 0x010162, //��ȡWIFI վ���б�
    DMS_NET_GET_WIFICFG = 0x010160, //��ȡWIFI����
    DMS_NET_GET_DDNSCFG = 0x010130, //��ȡDDNS����
    DMS_NET_CMD_UPGRADE_REQ = 0xA0200021, //�ļ�ϵͳ��������
    DMS_NET_CMD_UPGRADE_RESP = 0xA0200022, //�ļ�ϵͳ��������
    DMS_NET_CMD_UPGRADE_DATA = 0xA0200023, //�ļ�ϵͳ��������
    DMS_NET_CMD_UPDATE_FLASH = 0xA0200027, //����Flash
} GK_CMS_CMD_GET;
#endif

#if 0
/************************************************************************/
/*       �����������������                                              */
/************************************************************************/
//1) �豸��DMS_NET_DEVICE_INFO�ṹ��
#define DMS_NET_GET_DEVICECFG				0x020000		//��ȡ�豸����
#define DMS_NET_SET_DEVICECFG				0x020001		//�����豸����
#define DMS_NET_GET_DEF_DEVICECFG			0x020003		//�����豸����

//2������
//�������磨DMS_NET_NETWORK_CFG�ṹ��
#define DMS_NET_GET_NETCFG					0x020100		//��ȡ�������
#define DMS_NET_SET_NETCFG					0x020101		//�����������
#define DMS_NET_GET_DEF_NETCFG				0x020103

//���Ĺ���ƽ̨��DMS_NET_PLATFORM_INFO_V2�ṹ��
#define DMS_NET_GET_PLATFORMCFG				0x020110		//��ȡ���Ĺ���ƽ̨����
#define DMS_NET_SET_PLATFORMCFG				0x020111		//�������Ĺ���ƽ̨����

//PPPOE��DMS_NET_PPPOECFG�ṹ��
#define DMS_NET_GET_PPPOECFG				0x020120		//��ȡPPPOE����
#define DMS_NET_SET_PPPOECFG				0x020121		//����PPPOE����
//DDNS��DMS_NET_DDNSCFG�ṹ��
#define DMS_NET_GET_DDNSCFG					0x020130		//��ȡDDNS����
#define DMS_NET_SET_DDNSCFG					0x020131		//����DDNS����
//EMAIL��DMS_NET_EMAIL_PARAM�ṹ��
#define DMS_NET_GET_EMAILCFG				0x020140		//��ȡEMAIL����
#define DMS_NET_SET_EMAILCFG				0x020141		//����EMAIL����
//FTP��DMS_NET_FTP_PARAM�ṹ��
#define DMS_NET_GET_FTPCFG					0x020150		//��ȡFTP����
#define DMS_NET_SET_FTPCFG					0x020151		//����FTP����
//WIFI��DMS_NET_WIFI_CONFIG�ṹ��
#define DMS_NET_GET_WIFICFG					0x020160		//��ȡWIFI����
#define DMS_NET_SET_WIFICFG					0x020161		//����WIFI����
#define DMS_NET_GET_WIFI_SITE_LIST			0x020162		//��ȡWIFI վ���б�
//���������ߣ�DMS_NET_WANWIRELESS_CONFIG�ṹ��
#define DMS_NET_GET_WANWIRELESSCFG			0x020170		//��ȡWANWIRELESS����
#define DMS_NET_SET_WANWIRELESSCFG			0x020171		//����WANWIRELESS����
//UPNP (DMS_UPNP_CONFIG�ṹ)
#define DMS_UPNP_GET_CONFIG					0x020180			//��ȡUPNP����
#define DMS_UPNP_SET_CONFIG					0x020181			//����UPNP����
//�ֻ�ע��ƽ̨����(DMS_MOBILE_CENTER_INFO�ṹ)
#define DMS_GET_MOBILE_CENTER_INFO			0x020190			//��ȡ�ֻ�ƽ̨����
#define DMS_SET_MOBILE_CENTER_INFO			0x020191			//�����ֻ�ƽ̨����


//3������Ƶͨ��
//ͼ��ѹ����DMS_NET_CHANNEL_PIC_INFO�ṹ��
#define DMS_NET_GET_PICCFG					0x020200		//��ȡͼ��ѹ������
#define DMS_NET_GET_SUPPORT_STREAM_FMT  	0x020202        //��ȡϵͳ֧�ֵ�ͼ������
#define DMS_NET_SET_PICCFG					0x020201		//����ͼ��ѹ������
#define DMS_NET_GET_DEF_PICCFG				0x020203

//ͼ���ַ����ӣ�DMS_NET_CHANNEL_OSDINFO�ṹ��
#define DMS_NET_GET_OSDCFG					0x020210		//��ȡͼ���ַ����Ӳ���
#define DMS_NET_SET_OSDCFG					0x020211		//����ͼ���ַ����Ӳ���
//ͼ��ɫ�ʣ�DMS_NET_CHANNEL_COLOR�ṹ��
#define DMS_NET_GET_COLORCFG				0x020220		//��ȡͼ��ɫ�ʲ���
#define DMS_NET_SET_COLORCFG				0x020221		//����ͼ��ɫ�ʲ���

//(DMS_NET_COLOR_SUPPORT�ṹ)
#define DMS_NET_GET_COLOR_SUPPORT			0x020222		//��ȡͼ��ɫ�ʲ���
//ͼ��ɫ��(DMS_NET_CHANNEL_COLOR_SINGLE�ṹ)
#define DMS_NET_SET_COLORCFG_SINGLE			0x020223		//��������ĳ��ͼ��ɫ�ʲ���,
#define DMS_NET_GET_DEF_COLORCFG			0x020225

//(DMS_NET_ENHANCED_COLOR_SUPPORT�ṹ)
#define DMS_NET_GET_ENHANCED_COLOR_SUPPORT	0x020224		//��ȡͼ��ɫ��֧�ָ߼�����
//(DMS_NET_CHANNEL_ENHANCED_COLOR�ṹ)
#define DMS_NET_GET_ENHANCED_COLOR			0x020226		//��ȡͼ��ɫ�ʸ߼�����


//ͼ���ڵ���DMS_NET_CHANNEL_SHELTER�ṹ��
#define DMS_NET_GET_SHELTERCFG				0x020230		//��ȡͼ���ڵ�����
#define DMS_NET_SET_SHELTERCFG				0x020231		//����ͼ���ڵ�����
//ͼ���ƶ���⣨DMS_NET_CHANNEL_MOTION_DETECT�ṹ��
#define DMS_NET_GET_MOTIONCFG				0x020240		//��ȡͼ���ƶ�������
#define DMS_NET_SET_MOTIONCFG				0x020241		//����ͼ���ƶ�������
#define DMS_NET_DEF_MOTIONCFG				0x020242
//ͼ����Ƶ��ʧ��DMS_NET_CHANNEL_VILOST�ṹ��
#define DMS_NET_GET_VLOSTCFG				0x020250		//��ȡͼ����Ƶ��ʧ����
#define DMS_NET_SET_VLOSTCFG				0x020251		//����ͼ����Ƶ��ʧ����
#define DMS_NET_DEF_VLOSTCFG				0x020252
//ͼ���ڵ�������DMS_NET_CHANNEL_HIDEALARM�ṹ��
#define DMS_NET_GET_HIDEALARMCFG			0x020260		//��ȡͼ���ڵ���������
#define DMS_NET_SET_HIDEALARMCFG			0x020261		//����ͼ���ڵ���������
#define DMS_NET_DEF_HIDEALARMCFG			0x020262

//ͼ��¼��DMS_NET_CHANNEL_RECORD�ṹ��
#define DMS_NET_GET_RECORDCFG				0x020270		//��ȡͼ��¼�����
#define DMS_NET_SET_RECORDCFG				0x020271		//����ͼ��¼�����
#define DMS_NET_GET_DEF_RECORDCFG			0x020272
//ͼ���ֶ�¼��DMS_NET_CHANNEL_RECORD�ṹ)
#define DMS_NET_GET_RECORDMODECFG			0x020280		//��ȡͼ���ֶ�¼�����
#define DMS_NET_SET_RECORDMODECFG			0x020281		//����ͼ���ֶ�¼�����

//��������DMS_NET_DECODERCFG�ṹ��
#define DMS_NET_GET_DECODERCFG				0x020290		//��ȡ����������
#define DMS_NET_SET_DECODERCFG				0x020291		//���ý���������
#define DMS_NET_GET_DEF_DECODERCFG		    0x020292		//���ý���������

//ͼ��ʱץ�ģ�DMS_NET_SNAP_TIMER�ṹ��
#define DMS_NET_GET_SNAPTIMERCFG			0x020300		//��ȡͼ��ʱץ�Ĳ���
#define DMS_NET_SET_SNAPTIMERCFG			0x020301		//����ͼ��ʱץ�Ĳ���

//4�����ڣ�DMS_NET_RS232CFG�ṹ��
#define DMS_NET_GET_RS232CFG 				0x020310		//��ȡ232���ڲ���
#define DMS_NET_SET_RS232CFG				0x020311		//����232���ڲ���
#define DMS_NET_GET_DEF_SERIAL				0x020312

//(DMS_NET_PTZ_PROTOCOLCFG�ṹ)
#define DMS_NET_GET_PTZ_PROTOCOLCFG 		0x020316		//��ȡ֧�ֵ���̨Э����Ϣ
//(DMS_NET_PTZ_PROTOCOL_DATA �ṹ)
#define DMS_NET_ADD_PTZ_PROTOCOL            0x020317		//�����̨Э��

//5������
//���루DMS_NET_ALARMINCFG�ṹ��
#define DMS_NET_GET_ALARMINCFG 				0x020410		//��ȡ�����������
#define DMS_NET_SET_ALARMINCFG				0x020411		//���ñ����������
#define DMS_NET_DEF_ALARMINCFG				0x020412

//�����DMS_NET_ALARMOUTCFG�ṹ��
#define DMS_NET_GET_ALARMOUTCFG 			0x020420		//��ȡ�����������
#define DMS_NET_SET_ALARMOUTCFG				0x020421		//���ñ����������
#define DMS_NET_DEF_ALARMOUTCFG				0x020422


//6��ʱ��
//ʱ������ʱ�ƣ�DMS_NET_ZONEANDDST�ṹ��
#define DMS_NET_GET_ZONEANDDSTCFG 			0x020510		//��ȡʱ������ʱ�Ʋ���
#define DMS_NET_SET_ZONEANDDSTCFG			0x020511		//����ʱ������ʱ�Ʋ���

//7������Ԥ����DMS_NET_PREVIEWCFG�ṹ��
#define DMS_NET_GET_PREVIEWCFG 				0x020600		//��ȡԤ������
#define DMS_NET_SET_PREVIEWCFG				0x020601		//����Ԥ������
#define DMS_NET_GET_DEF_PREVIEWCFG			0x020602
//8����Ƶ�����DMS_NET_VIDEOOUT�ṹ��
#define DMS_NET_GET_VIDEOOUTCFG 			0x020700		//��ȡ��Ƶ�������
#define DMS_NET_SET_VIDEOOUTCFG				0x020701		//������Ƶ�������
#define DMS_NET_GET_DEF_VIDEOOUTCFG			0x020702		//������Ƶ�������
//9) �û����û���
//�û���DMS_NET_USER_INFO�ṹ��
#define DMS_NET_GET_USERCFG 				0x020800		//��ȡ�û�����
#define DMS_NET_SET_USERCFG					0x020801		//�����û�����
//�û��飨DMS_NET_USER_GROUP_INFO�ṹ��
#define DMS_NET_GET_USERGROUPCFG 			0x020810		//��ȡ�û������
#define DMS_NET_SET_USERGROUPCFG			0x020811		//�����û������
//10)�쳣��DMS_NET_EXCEPTION�ṹ��
#define DMS_NET_GET_EXCEPTIONCFG 			0x020900		//��ȡ�쳣����
#define DMS_NET_SET_EXCEPTIONCFG			0x020901		//�����쳣����
//11)Ӳ��
//����Ӳ����Ϣ��DMS_NET_HDCFG�ṹ��
#define DMS_NET_GET_HDCFG 					0x030100		//��ȡӲ�̲���
#define DMS_NET_SET_HDCFG					0x030101		//����(����)Ӳ�̲���
#define DMS_NET_HD_FORMAT           		0x030102        //��ʽ��Ӳ��
#define DMS_NET_GET_HD_FORMAT_STATUS        0x030103        //��ʽ��Ӳ��״̬�Լ�����

//����������Ϣ���ã�DMS_NET_HDGROUP_CFG�ṹ��
#define DMS_NET_GET_HDGROUPCFG 				0x030200		//��ȡӲ�������
#define DMS_NET_SET_HDGROUPCFG				0x030201		//����Ӳ�������
//12)�ָ�����ֵ���ã�DMS_NET_RESTORECFG��
#define DMS_NET_GET_RESTORECFG          	0x030300    	//�ָ�����ֵ����
#define DMS_NET_SET_RESTORECFG          	0x030301
#define DMS_NET_GET_DEF_RESTORECFG          0x030302
#define DMS_NET_SET_SAVECFG          0x030303 //��������
//13) RTSP
#define DMS_NET_GET_RTSPCFG          0x030304
#define DMS_NET_SET_RTSPCFG          0x030305 //rtsp����

//���ûص�����
// ������Ƶ�Խ����ݻص�����(OnNetAudioStreamCallBack)
#define DMS_NET_REG_AUDIOCALLBACK          	0xA0100001
// ������Ƶ���ݻص�����(DMS_NET_VIDEOCALLBACK)
// ÿ��ͨ���Ĳ�ͨ�������Էֱ����ò�ͬ�Ļص������ӿ�
#define DMS_NET_REG_VIDEOCALLBACK          	0xA0100002
// ���ñ�����Ϣ�ص�����(OnNetAlarmCallback)
#define DMS_NET_REG_ALARMCALLBACK          	0xA0100003
// ���ô������ݻص�����(OnNetSerialDataCallback)
#define DMS_NET_REG_SERIALDATACALLBACK      0xA0100004
// ��͸�����ڴ���
#define  DMS_NET_CMD_OPEN_SERIALTRANSP  	0xA0100005
// �ر�͸�����ڴ���
#define  DMS_NET_CMD_CLOSE_SERIALTRANSP 	0xA0100006
// ��̨����( DMS_NET_PTZ_CONTROL �ṹ��)
#define DMS_NET_CMD_PTZ_CONTROL             0xA0100007
// �����������(����ģʽDMS_NET_ALARMOUT_CONTROL)
#define DMS_NET_CMD_ALARMOUT_CONTROL        0xA0100008
// ��ȡ�������״̬(DMS_NET_SENSOR_STATE)
#define	DMS_NET_GET_ALARMOUT_STATE			0xA0100009
// ��ȡ��������״̬(DMS_NET_SENSOR_STATE)
#define	DMS_NET_GET_ALARMIN_STATE			0xA010000A
// ������Ƶ���ؼ�֡
#define	DMS_NET_CMD_IFRAME_REQUEST        	0xA010000B
// �����豸
#define 	DMS_NET_CMD_REBOOT		       	0xA010000C
// �豸�ػ�
#define 	DMS_NET_CMD_SHUTDOWN		   	0xA010000D
// ץ��ͼƬ(DMS_NET_SNAP_DATA)
#define 	DMS_NET_CMD_SNAPSHOT		   	0xA010000E
//¼�����(DMS_NET_REC_CONTROL)
#define 	DMS_NET_CMD_REC_CONTROL		    	0xA010000F
//ע��طŻص�����
#define   DMS_NET_REG_PLAYBACKCALLBACK      0xA0100010
//��ʼ��Ƶ�Խ�
#define 	DMS_NET_CMD_START_TALKAUDIO	0xA0100011
//ֹͣ��Ƶ�Խ�
#define 	DMS_NET_CMD_STOP_TALKAUDIO		0xA0100012
//��ȡϵͳʱ��
#define 	DMS_NET_CMD_GET_SYSTIME			0xA0100013
//����ϵͳʱ��
#define 	DMS_NET_CMD_SET_SYSTIME 			0xA0100014
//��ĳͨ����Ƶ
#define 	DMS_NET_CMD_START_VIDEO  		0xA0100015
//�ر�ĳͨ����Ƶ
#define 	DMS_NET_CMD_STOP_VIDEO 			0xA0100016
// ͸�����ڴ���
#define  DMS_NET_CMD_DATA_SERIALTRANSP  	0xA0100017


//hujh add 2012-05-11
#define   DMS_NET_CMD_GET_SPSPPSBASE64ENCODE_DATA 0xA0100018
#define 	DMS_NET_CMD_CLOSE_RS485 			0xA0100019
#define 	DMS_NET_CMD_GET_VIDEO_STATE   	0xA0100020

//�ļ�ϵͳ��������(DMS_NET_UPGRADE_REQ)
#define   DMS_NET_CMD_UPGRADE_REQ		0xA0100021
//�ļ�ϵͳ��������(DMS_NET_UPGRADE_RESP)
#define   DMS_NET_CMD_UPGRADE_RESP		0xA0100022
//�ļ�ϵͳ��������(DMS_NET_UPGRADE_DATA)
#define   DMS_NET_CMD_UPGRADE_DATA		0xA0100023
//����Flash
#define  DMS_NET_CMD_UPDATE_FLASH 0xA0100027


//��ʾָ��OSD�ı�--DMS_NET_SPRCIAL_ESD
#define DMS_NET_CMD_SHOW_SPECIAL_OSD        0xA0100024

//��ȡ���������õ�Ԥ�õ����Ϣ
#define DMS_NET_GET_ALL_PRESET  0x010427
//��ȡͨ���������Ѿ����õ�Ѳ����
#define DMS_NET_GET_CRUISE_CFG 0x010428
//����һ��Ѳ��
#define DMS_NET_SET_CRUISE_INFO 0x010429

//���������߲���
#define DMS_NET_GET_WANWIRELESSCFG_V2 0x010172
#define DMS_NET_SET_WANWIRELESSCFG_V2 0x010173

//��ת�ڼ�����
#define DMS_NET_GET_COLOR_BLACK_DETECTION 0x010228
#define DMS_NET_SET_COLOR_BLACK_DETECTION 0x010227

//P2P����
#define DMS_NET_GET_P2PCFG 0x020410
#define DMS_NET_SET_P2PCFG 0x020411

//�豸ά��
#define DMS_DEV_GET_DEVICEMAINTAINCFG 0x020600
#define DMS_DEV_SET_DEVICEMAINTAINCFG 0x020601

//��ȡLOG��Ϣ
#define DMS_NET_GET_LOGINFO 0x040500

//¼��������
#define DMS_NET_GET_REC_DAY_IN_MONTH 0x050101
#define DMS_NET_GET_RECORDSTREAMMODE 0x010273
#define DMS_NET_SET_RECORDSTREAMMODE 0x010274

//snmp����
#define DMS_NET_GET_SNMPCFG  0x040700
#define DMS_NET_SET_SNMPCFG  0x040701

//NTP ����
#define DMS_NET_GET_NTPCFG 0x010520
#define DMS_NET_SET_NTPCFG 0x010521
#define DMS_NET_DEF_NTPCFG 0x010522

//��ȡͼ���¼�ץ�Ĳ���
#define DMS_NET_GET_SNAPEVENTCFG 0x010302
#define DMS_NET_SET_SNAPEVENTCFG 0x010303

//����WPS��ʼ
#define DMS_NET_SET_WIFI_WPS_START 0x010163
#endif

/************************************************************************/
/*       �����������������                                              */
/************************************************************************/
//1) �豸��DMS_NET_DEVICE_INFO�ṹ��
#define DMS_NET_GET_DEVICECFG               0x020000        //��ȡ�豸����
#define DMS_NET_SET_DEVICECFG               0x020001        //�����豸����
#define DMS_NET_GET_DEF_DEVICECFG           0x020003        //�����豸����
#define DMS_NET_GET_DEVICE_EXTCFG			0x020004		//��ȡ�豸��չ��Ϣ
#define DMS_NET_SET_DEVICE_EXTCFG			0x020005		//�����豸��չ��Ϣ


//2������
//�������磨DMS_NET_NETWORK_CFG�ṹ��
#define DMS_NET_GET_NETCFG                  0x020100        //��ȡ�������
#define DMS_NET_SET_NETCFG                  0x020101        //�����������
#define DMS_NET_GET_DEF_NETCFG              0x020103

//���Ĺ���ƽ̨��DMS_NET_PLATFORM_INFO_V2�ṹ��
#define DMS_NET_GET_PLATFORMCFG             0x020110        //��ȡ���Ĺ���ƽ̨����
#define DMS_NET_SET_PLATFORMCFG             0x020111        //�������Ĺ���ƽ̨����

//PPPOE��DMS_NET_PPPOECFG�ṹ��
#define DMS_NET_GET_PPPOECFG                0x020120        //��ȡPPPOE����
#define DMS_NET_SET_PPPOECFG                0x020121        //����PPPOE����

//DDNS��DMS_NET_DDNSCFG�ṹ��
#define DMS_NET_GET_DDNSCFG                 0x020130        //��ȡDDNS����
#define DMS_NET_SET_DDNSCFG                 0x020131        //����DDNS����

//EMAIL��DMS_NET_EMAIL_PARAM�ṹ��
#define DMS_NET_GET_EMAILCFG                0x020140        //��ȡEMAIL����
#define DMS_NET_SET_EMAILCFG                0x020141        //����EMAIL����
#define DMS_NET_TEST_SEND_EMAIL             0x020142        //���Է���EMAIL

//FTP��DMS_NET_FTP_PARAM�ṹ��
#define DMS_NET_GET_FTPCFG                  0x020150        //��ȡFTP����
#define DMS_NET_SET_FTPCFG                  0x020151        //����FTP����
#define DMS_NET_TEST_FTP_CONNECT            0x020152        //����FTP����

//WIFI��DMS_NET_WIFI_CONFIG�ṹ��
#define DMS_NET_GET_WIFICFG                 0x020160        //��ȡWIFI����
#define DMS_NET_SET_WIFICFG                 0x020161        //����WIFI����
#define DMS_NET_GET_WIFI_SITE_LIST          0x020162        //��ȡWIFI վ���б�
#define DMS_NET_SET_WIFI_WPS_START          0x020163        //����WPS��ʼ

#define DMS_NET_TEST_WIFI_CONNECT			0x020164        //����WIFI ����

//���������ߣ�DMS_NET_WANWIRELESS_CONFIG�ṹ��
#define DMS_NET_GET_WANWIRELESSCFG          0x020170        //��ȡWANWIRELESS����
#define DMS_NET_SET_WANWIRELESSCFG          0x020171        //����WANWIRELESS����

//���������ߣ�DMS_NET_WANWIRELESS_CONFIG_V2�ṹ��
#define DMS_NET_GET_WANWIRELESSCFG_V2       0x020172        //��ȡWANWIRELESS����
#define DMS_NET_SET_WANWIRELESSCFG_V2       0x020173        //����WANWIRELESS����

//UPNP (DMS_UPNP_CONFIG�ṹ)
#define DMS_UPNP_GET_CONFIG                 0x020180            //��ȡUPNP����
#define DMS_UPNP_SET_CONFIG                 0x020181            //����UPNP����
//�ֻ�ע��ƽ̨����(DMS_MOBILE_CENTER_INFO�ṹ)
#define DMS_GET_MOBILE_CENTER_INFO          0x020190            //��ȡ�ֻ�ƽ̨����
#define DMS_SET_MOBILE_CENTER_INFO          0x020191            //�����ֻ�ƽ̨����


//3������Ƶͨ��
//ͼ��ѹ����DMS_NET_CHANNEL_PIC_INFO�ṹ��
#define DMS_NET_GET_PICCFG                  0x020200        //��ȡͼ��ѹ������
#define DMS_NET_SET_PICCFG                  0x020201        //����ͼ��ѹ������
#define DMS_NET_GET_DEF_PICCFG              0x020203
//DMS_NET_SUPPORT_STREAM_FMT  ͼ������
#define DMS_NET_GET_SUPPORT_STREAM_FMT      0x020202        //��ȡϵͳ֧�ֵ�ͼ������

//ͼ���ַ����ӣ�DMS_NET_CHANNEL_OSDINFO�ṹ��
#define DMS_NET_GET_OSDCFG                  0x020210        //��ȡͼ���ַ����Ӳ���
#define DMS_NET_SET_OSDCFG                  0x020211        //����ͼ���ַ����Ӳ���
//ͼ��ɫ�ʣ�DMS_NET_CHANNEL_COLOR�ṹ��
#define DMS_NET_GET_COLORCFG                0x020220        //��ȡͼ��ɫ�ʲ���
#define DMS_NET_SET_COLORCFG                0x020221        //����ͼ��ɫ�ʲ���

//(DMS_NET_COLOR_SUPPORT�ṹ)
#define DMS_NET_GET_COLOR_SUPPORT           0x020222        //��ȡͼ��ɫ�ʲ���
//ͼ��ɫ��(DMS_NET_CHANNEL_COLOR_SINGLE�ṹ)
#define DMS_NET_SET_COLORCFG_SINGLE         0x020223        //��������ĳ��ͼ��ɫ�ʲ���,
#define DMS_NET_GET_DEF_COLORCFG            0x020225

//(DMS_NET_ENHANCED_COLOR_SUPPORT�ṹ)
#define DMS_NET_GET_ENHANCED_COLOR_SUPPORT  0x020224        //��ȡͼ��ɫ��֧�ָ߼�����
//(DMS_NET_CHANNEL_ENHANCED_COLOR�ṹ)
#define DMS_NET_GET_ENHANCED_COLOR          0x020226        //��ȡͼ��ɫ�ʸ߼�����

//DMS_NET_DAY_NIGHT_DETECTION_EX
#define DMS_NET_SET_COLOR_BLACK_DETECTION   0x020227        //���ò�ת�ڼ�����
#define DMS_NET_GET_COLOR_BLACK_DETECTION   0x020228        //��ȡ��ת�ڼ�����

//ͼ���ڵ���DMS_NET_CHANNEL_SHELTER�ṹ��
#define DMS_NET_GET_SHELTERCFG              0x020230        //��ȡͼ���ڵ�����
#define DMS_NET_SET_SHELTERCFG              0x020231        //����ͼ���ڵ�����
//ͼ���ƶ���⣨DMS_NET_CHANNEL_MOTION_DETECT�ṹ��
#define DMS_NET_GET_MOTIONCFG               0x020240        //��ȡͼ���ƶ�������
#define DMS_NET_SET_MOTIONCFG               0x020241        //����ͼ���ƶ�������
#define DMS_NET_DEF_MOTIONCFG               0x020242
//ͼ����Ƶ��ʧ��DMS_NET_CHANNEL_VILOST�ṹ��
#define DMS_NET_GET_VLOSTCFG                0x020250        //��ȡͼ����Ƶ��ʧ����
#define DMS_NET_SET_VLOSTCFG                0x020251        //����ͼ����Ƶ��ʧ����
#define DMS_NET_DEF_VLOSTCFG                0x020252
//ͼ���ڵ�������DMS_NET_CHANNEL_HIDEALARM�ṹ��
#define DMS_NET_GET_HIDEALARMCFG            0x020260        //��ȡͼ���ڵ���������
#define DMS_NET_SET_HIDEALARMCFG            0x020261        //����ͼ���ڵ���������
#define DMS_NET_DEF_HIDEALARMCFG            0x020262

//ͼ��¼��DMS_NET_CHANNEL_RECORD�ṹ��
#define DMS_NET_GET_RECORDCFG               0x020270        //��ȡͼ��¼�����
#define DMS_NET_SET_RECORDCFG               0x020271        //����ͼ��¼�����
#define DMS_NET_GET_DEF_RECORDCFG           0x020272
//get/set record stream type (DMS_NET_RECORD_STREAMMODE)
#define DMS_NET_GET_RECORDSTREAMMODE        0x020273
#define DMS_NET_SET_RECORDSTREAMMODE        0x020274

//ͼ���ֶ�¼��DMS_NET_CHANNEL_RECORD�ṹ)
#define DMS_NET_GET_RECORDMODECFG           0x020280        //��ȡͼ���ֶ�¼�����
#define DMS_NET_SET_RECORDMODECFG           0x020281        //����ͼ���ֶ�¼�����

//��������DMS_NET_DECODERCFG�ṹ��
#define DMS_NET_GET_DECODERCFG              0x020290        //��ȡ����������
#define DMS_NET_SET_DECODERCFG              0x020291        //���ý���������
#define DMS_NET_GET_DEF_DECODERCFG          0x020292        //���ý���������

//ͼ��ʱץ�ģ�DMS_NET_SNAP_TIMER�ṹ��
#define DMS_NET_GET_SNAPTIMERCFG            0x020300        //��ȡͼ��ʱץ�Ĳ���
#define DMS_NET_SET_SNAPTIMERCFG            0x020301        //����ͼ��ʱץ�Ĳ���

//ͼ���¼�ץ�ģ�DMS_NET_SNAP_EVENT�ṹ��
#define DMS_NET_GET_SNAPEVENTCFG            0x020302        //��ȡͼ���¼�ץ�Ĳ���
#define DMS_NET_SET_SNAPEVENTCFG            0x020303        //����ͼ���¼�ץ�Ĳ���

//4�����ڣ�DMS_NET_RS232CFG�ṹ��
#define DMS_NET_GET_RS232CFG                0x020310        //��ȡ232���ڲ���
#define DMS_NET_SET_RS232CFG                0x020311        //����232���ڲ���
#define DMS_NET_GET_DEF_SERIAL              0x020312
//(DMS_NET_PTZ_PROTOCOLCFG�ṹ)
#define DMS_NET_GET_PTZ_PROTOCOLCFG         0x020316        //��ȡ֧�ֵ���̨Э����Ϣ
//(DMS_NET_PTZ_PROTOCOL_DATA �ṹ)
#define DMS_NET_ADD_PTZ_PROTOCOL            0x020317        //�����̨Э��
//(DMS_NET_PTZ_PROTOCOL_DATA �ṹ)
#define DMS_NET_GET_PTZ_PROTOCOL_DATA       0x020318        //��ȡ��̨Э������
//5������
//���루DMS_NET_ALARMINCFG�ṹ��
#define DMS_NET_GET_ALARMINCFG              0x020410        //��ȡ�����������
#define DMS_NET_SET_ALARMINCFG              0x020411        //���ñ����������
#define DMS_NET_DEF_ALARMINCFG              0x020412

//�����DMS_NET_ALARMOUTCFG�ṹ��
#define DMS_NET_GET_ALARMOUTCFG             0x020420        //��ȡ�����������
#define DMS_NET_SET_ALARMOUTCFG             0x020421        //���ñ����������
#define DMS_NET_DEF_ALARMOUTCFG             0x020422
//Ĭ������
#define DMS_NET_GET_DEFAULTCFG_FILE         0x020423
#define DMS_NET_SET_DEFAULTCFG_FILE         0x020424
//logo�ļ�
#define DMS_NET_GET_LOGO_FILE               0x020425
#define DMS_NET_SET_LOGO_FILE               0x020426
//��ȡ���������õ�Ԥ�õ����Ϣ-DMS_NET_PRESET_INFO
#define DMS_NET_GET_ALL_PRESET              0x020427
//��ȡͨ���������Ѿ����õ�Ѳ���� DMS_NET_CRUISE_CFG
#define DMS_NET_GET_CRUISE_CFG              0x020428
//����һ��Ѳ��   DMS_NET_CRUISE_INFO
#define DMS_NET_SET_CRUISE_INFO				0x020429

//6��ʱ��
//ʱ������ʱ�ƣ�DMS_NET_ZONEANDDST�ṹ��
#define DMS_NET_GET_ZONEANDDSTCFG           0x020510        //��ȡʱ������ʱ�Ʋ���
#define DMS_NET_SET_ZONEANDDSTCFG           0x020511        //����ʱ������ʱ�Ʋ���
//NTP ��DMS_NET_NTP_CFG�ṹ��
#define DMS_NET_GET_NTPCFG					0x020520        //��ȡNTP ����
#define DMS_NET_SET_NTPCFG					0x020521        //����NTP ����
#define DMS_NET_DEF_NTPCFG					0x020522

//7������Ԥ����DMS_NET_PREVIEWCFG�ṹ��
#define DMS_NET_GET_PREVIEWCFG              0x020600        //��ȡԤ������
#define DMS_NET_SET_PREVIEWCFG              0x020601        //����Ԥ������
#define DMS_NET_GET_DEF_PREVIEWCFG          0x020602
//8����Ƶ�����DMS_NET_VIDEOOUT�ṹ��
#define DMS_NET_GET_VIDEOOUTCFG             0x020700        //��ȡ��Ƶ�������
#define DMS_NET_SET_VIDEOOUTCFG             0x020701        //������Ƶ�������
#define DMS_NET_GET_DEF_VIDEOOUTCFG         0x020702        //������Ƶ�������
//9) �û����û���
//�û���DMS_NET_USER_INFO�ṹ��
#define DMS_NET_GET_USERCFG                 0x020800        //��ȡ�û�����
#define DMS_NET_SET_USERCFG                 0x020801        //�����û�����
//�û��飨DMS_NET_USER_GROUP_INFO�ṹ��
#define DMS_NET_GET_USERGROUPCFG            0x020810        //��ȡ�û������
#define DMS_NET_SET_USERGROUPCFG            0x020811        //�����û������
//10)�쳣��DMS_NET_EXCEPTION�ṹ��
#define DMS_NET_GET_EXCEPTIONCFG            0x020900        //��ȡ�쳣����
#define DMS_NET_SET_EXCEPTIONCFG            0x020901        //�����쳣����
//11)Ӳ��
//����Ӳ����Ϣ��DMS_NET_HDCFG�ṹ��
#define DMS_NET_GET_HDCFG                   0x030100        //��ȡӲ�̲���
#define DMS_NET_SET_HDCFG                   0x030101        //����(����)Ӳ�̲���
#define DMS_NET_HD_FORMAT                   0x030102        //��ʽ��Ӳ��
#define DMS_NET_GET_HD_FORMAT_STATUS        0x030103        //��ʽ��Ӳ��״̬�Լ�����
#define DMS_NET_UNLOAD_DISK                 0x030104        //ж�ش���
//#define DMS_NET_UNLOAD_DISK                 0x10000013        //ж�ش���


#define DMS_NET_SET_NASCFG                     0x030105        //����NAS����
#define DMS_NET_GET_NASCFG                     0x030106        //��ȡNAS����

//����������Ϣ���ã�DMS_NET_HDGROUP_CFG�ṹ��
#define DMS_NET_GET_HDGROUPCFG              0x030200        //��ȡӲ�������
#define DMS_NET_SET_HDGROUPCFG              0x030201        //����Ӳ�������
//12)�ָ�����ֵ���ã�DMS_NET_RESTORECFG��
#define DMS_NET_GET_RESTORECFG              0x030300
#define DMS_NET_SET_RESTORECFG              0x030301
#define DMS_NET_GET_DEF_RESTORECFG          0x030302
#define DMS_NET_SET_SAVECFG					0x030303 		//��������

//13) RTSP
#define DMS_NET_GET_RTSPCFG					0x030304
#define DMS_NET_SET_RTSPCFG					0x030305 		//rtsp����


//14)  P2P
#define DMS_NET_GET_P2PCFG                  0x030410
#define DMS_NET_SET_P2PCFG                  0x030411

//15)�û���Կ����
#define DMS_NET_GET_USERKEY                 0x030500
#define DMS_NET_SET_USERKEY                 0x030501

//16)HVR �豸ά��
#define DMS_DEV_GET_DEVICEMAINTAINCFG		0x030600
#define DMS_DEV_SET_DEVICEMAINTAINCFG		0X030601

//
#define DMS_NET_CMD_AUTO_TEST               0x040411
#define DMS_NET_GET_LOGINFO					0x050500

//Զ��¼��ط�
#define  DMS_NET_GET_REC_DAY_IN_MONTH				0x060101
#define	  DMS_NET_GET_TS_INFO									0x060001
#define  DMS_NET_SET_TS_INFO									0x060002
#define DMS_NET_GET_UPGRADE_STATUS					0x060003
#define DMS_NET_CMD_REMOTE_PLAYBACK			0x050600

#define		CMD_GET_FILELIST		0x1000004E	//Param:JBNV_FIND_FILE_RESP


//SNMP
#define DMS_NET_GET_SNMPCFG                  0x050700
#define DMS_NET_SET_SNMPCFG                  0x050701
#ifdef MODULE_SUPPORT_RF433
#define DMS_RF433_DEV_LEARN_CODE			0x070001
#define DMS_RF433_DEV_GET_DEV_LIST			0x070002
#define DMS_RF433_DEV_MODIFY_NAME			0x070003
#define DMS_RF433_DEV_DELETE_DEV			0x070004
#define DMS_RF433_DEV_EXIT_LEARN_CODE		0x070005
#endif
//���ûص�����
// ������Ƶ�Խ����ݻص�����(OnNetAudioStreamCallBack)
#define DMS_NET_REG_AUDIOCALLBACK           0xA0100001
// ������Ƶ���ݻص�����(DMS_NET_VIDEOCALLBACK)
// ÿ��ͨ���Ĳ�ͨ�������Էֱ����ò�ͬ�Ļص������ӿ�
#define DMS_NET_REG_VIDEOCALLBACK           0xA0100002
// ���ñ�����Ϣ�ص�����(OnNetAlarmCallback)
#define DMS_NET_REG_ALARMCALLBACK           0xA0100003
// ���ô������ݻص�����(OnNetSerialDataCallback)
#define DMS_NET_REG_SERIALDATACALLBACK      0xA0100004
// ��͸�����ڴ���
#define  DMS_NET_CMD_OPEN_SERIALTRANSP      0xA0100005
// �ر�͸�����ڴ���
#define  DMS_NET_CMD_CLOSE_SERIALTRANSP     0xA0100006
// ��̨����( DMS_NET_PTZ_CONTROL �ṹ��)
#define DMS_NET_CMD_PTZ_CONTROL             0xA0100007
// �����������(����ģʽDMS_NET_ALARMOUT_CONTROL)
#define DMS_NET_CMD_ALARMOUT_CONTROL        0xA0100008
// ��ȡ�������״̬(DMS_NET_SENSOR_STATE)
#define DMS_NET_GET_ALARMOUT_STATE          0xA0100009
// ��ȡ��������״̬(DMS_NET_SENSOR_STATE)
#define DMS_NET_GET_ALARMIN_STATE           0xA010000A
// ������Ƶ���ؼ�֡
#define DMS_NET_CMD_IFRAME_REQUEST          0xA010000B
// �����豸
#define     DMS_NET_CMD_REBOOT              0xA010000C

// �豸�ػ�
#define     DMS_NET_CMD_SHUTDOWN            0xA010000D
// ץ��ͼƬ(DMS_NET_SNAP_DATA)
#define     DMS_NET_CMD_SNAPSHOT            0xA010000E
#define     DMS_NET_CMD_SNAPSHOT_V2         0xA020000E
//¼�����(DMS_NET_REC_CONTROL)
#define     DMS_NET_CMD_REC_CONTROL					0xA010000F
//ע��طŻص�����
#define   DMS_NET_REG_PLAYBACKCALLBACK				0xA0100010
//��ʼ��Ƶ�Խ�
#define     DMS_NET_CMD_START_TALKAUDIO				0xA0100011
//ֹͣ��Ƶ�Խ�
#define     DMS_NET_CMD_STOP_TALKAUDIO				0xA0100012
//��ȡϵͳʱ��
#define     DMS_NET_CMD_GET_SYSTIME                 0xA0100013
//����ϵͳʱ��
#define     DMS_NET_CMD_SET_SYSTIME                 0xA0100014
//��ĳͨ����Ƶ
#define     DMS_NET_CMD_START_VIDEO                 0xA0100015
//�ر�ĳͨ����Ƶ
#define     DMS_NET_CMD_STOP_VIDEO                  0xA0100016

//hujh add 2012-05-11
#define   DMS_NET_CMD_GET_SPSPPSBASE64ENCODE_DATA   0xA0100018
#define     DMS_NET_CMD_CLOSE_RS485                 0xA0100019
#define     DMS_NET_CMD_GET_VIDEO_STATE             0xA0100020

//�ļ�ϵͳ��������(DMS_NET_UPGRADE_REQ)
#define   DMS_NET_CMD_UPGRADE_REQ                   0xA0100021
//�ļ�ϵͳ��������(DMS_NET_UPGRADE_RESP)
#define   DMS_NET_CMD_UPGRADE_RESP                  0xA0100022
//�ļ�ϵͳ��������(DMS_NET_UPGRADE_DATA)
#define   DMS_NET_CMD_UPGRADE_DATA                  0xA0100023

//��ʾָ��OSD�ı�--DMS_NET_SPRCIAL_ESD
#define DMS_NET_CMD_SHOW_SPECIAL_OSD                0xA0100024
//2013-1-25 15:05:47--FTP�ϴ�ͼƬ
#define DMS_NET_CMD_UPLOAD_FTP                      0xA0100025
//2013-1-25 15:05:47--����EMAIL
#define DMS_NET_CMD_SEND_MAIL                       0xA0100026

#define DMS_NET_CMD_UPDATE_FLASH                    0xA0100027

#define DMS_NET_CMD_COM_DATA						0xA0100028
//��ȡ�ܽ���� -- DMS_PERIMETER_RULE_PARA_S
#define DMS_NET_CMD_GET_PERIMETER_PARA				0xA0100029
//�����ܽ���� -- DMS_PERIMETER_RULE_PARA_S
#define DMS_NET_CMD_SET_PERIMETER_PARA				0xA010002A
//�������ܷ���Ŀ������--DMS_TGT_SET_S
#define DMS_NET_CMD_ITEV_PUSH_TARGET				0xA010002B
//��ȡ���ܷ�����������Ϣ
#define DMS_NET_CMD_GET_ITEV_CTRLCFG				0xA010002C
//�������ܷ�����������Ϣ
#define DMS_NET_CMD_SET_ITEV_CTRLCFG				0xA010002D
//���õ�����
#define DMS_NET_CMD_SET_TRIPWIRE_PARA				0xA010002E
//��ȡ������
#define DMS_NET_CMD_GET_TRIPWIRE_PARA				0xA010002F
//������Ƶ�쳣
#define DMS_NET_CMD_SET_VIDEO_ANOMALY_PARA                    0xA0100030
//��ȡ��Ƶ�쳣
#define DMS_NET_CMD_GET_VIDEO_ANOMALY_PARA                    0xA0100031
#define DMS_NET_CMD_GET_RECORDFILEENDTIME				0xA0100032
#define DMS_NET_CMD_REG_TEMPERATUREHUMIDITY_CALL		0xA0100033
#define DMS_NET_CMD_REG_HEADCOUNTREG_CALL				0xA0100034

////////////
#ifdef MODULE_SUPPORT_RF433
/*
	433 device:
	                id,type,name
*/
typedef enum{
	GKCMS_DEV_433_TYPE_REMOTE_CONTROL = 10,	// ң��
	GKCMS_DEV_433_TYPE_ALARM,					// ����
	GKCMS_DEV_433_TYPE_OTHER,					// ����
}ENUM_GKCMS_DEV_433_TYPE;

typedef struct{
	int 	id;				// 433 ��ʶ
	int		type;			// 433 �豸���� 
	char	name[64];		// �豸����		
}GkCmsCfg433Req,GkCmsSet433Req,GkCmsDel433Req,GkCmsS433Dev;;
typedef struct{
	int		num;			// �豸����
	GkCmsS433Dev	dev[32];		// �豸
}GkCmsGet433Resp;

typedef struct{
	int		result;			// ENUM_AVIOTC_CFG_433_RET,0 :success,1:failure
	int		none;			// Ԥ��
}GkCmsGet433Req,GkCmsCfg433Resp,GkCmsSet433Resp,GkCmsDel433Resp,GkCmsCfg433ExitReq,GkCmsCfg433ExitResp;
#endif


typedef struct _SYSTEMTIME
{
    WORD wYear;
    WORD wMonth;
    WORD wDayOfWeek;
    WORD wDay;
    WORD wHour;
    WORD wMinute;
    WORD wSecond;
    WORD wMilliseconds;
} SYSTEMTIME;

typedef struct tagDMS_SIZE
{
	int           nWidth;
	int           nHeight;
}DMS_SIZE;

typedef struct tagDMS_RANGE
{
	int			nMin;
	int			nMax;
}DMS_RANGE, *LDMS_RANGE;

/*��������ע�����ݽṹ��*/
typedef struct _ACCEPT_HEAD
{
	unsigned long	nFlag;  //Э��汾��ʶ
	unsigned long	nSockType;
}ACCEPT_HEAD,*PACCEPT_HEAD;

typedef struct _USER_INFO
{
	char	szUserName[32];
	char	szUserPsw[32];
	char	MacCheck[36];
} USER_INFO,*PUSER_INFO;

typedef enum
{
    NETCMD_NOT_USER = 5003,
    NETCMD_MAX_LINK = 5004,
    JBERR_IPMAC_UNMATCH = 0x10000014,
} LOGIN_ERR;

typedef enum _PROTOCOL_TYPE
{
    PROTOCOL_TCP = 0,
    PROTOCOL_UDP = 1,
    PROTOCOL_MULTI = 2
}PROTOCOL_TYPE;

//CHECK_USER_INFO
typedef struct tagCHECK_USER_INFO
{
	ULONG	nFlag;			//JBNV_FLAG
	ULONG	nSockType;		//SOCK_TYPEö���� SOCK_LOGON
	char	szUserName[32];		//�û���
	char	szPassword[32];		//����
	BYTE	bMacAddress[36];	//pc mac��ַ
}CHECK_USER_INFO,*PCHECK_USER_INFO;


//SOCK_TYPE
typedef enum _SOCK_TYPE
{
	SOCK_LOGON = 0,
	SOCK_DATA = 1,
	SOCK_FILE = 2,
	SOCK_TALKBACK = 3,
	SOCK_CENTER,
}SOCK_TYPE;


//JB_NET_PACK_HEAD
typedef struct tagJB_NET_PACK_HEAD
{
    ULONG	nFlag;			//9000
    ULONG	nCommand; 		//���������ؽ��
    ULONG	nChannel;		//�ɺ���
    ULONG	nErrorCode;		//�ɺ���
    ULONG	nBufSize;		//�ɹ��󷵻ص����ݴ�С
}JB_NET_PACK_HEAD,*PJB_NET_PACK_HEAD;


//JBNV_SERVER_INFO
typedef struct tagJBNV_SERVER_INFO
{
	DWORD	dwSize;			//�ṹ��С
	DWORD	dwServerFlag;		//����������
	DWORD	dwServerIp;		//������IP(������ʾ��ʽ)
	char		szServerIp[16];		//������IP(�ַ�����ʾ��ʽ)
	WORD		wServerPort;		//�������˿�
	WORD		wChannelNum;		//ͨ������
	DWORD	dwVersionNum;		//�汾
	char		szServerName[32];	//����������
	DWORD	dwServerCPUType;	//��ǰCPU����
	BYTE		bServerSerial[48];  //���������кţ�����Ψһ��ʶ����
	BYTE		byMACAddr[6];		//�������������ַ
	DWORD	dwAlarmInCount;
	DWORD	dwAlarmOutCount;
	DWORD	dwSysFlags;		//ϵͳ֧�ֵĹ���
	DWORD	dwUserRight;		//��ǰ�û�Ȩ��
	DWORD	dwNetPriviewRight;	//����ۿ�Ȩ��
	char		csServerDes[64];
}JBNV_SERVER_INFO,*PJBNV_SERVER_INFO;



//CMD_SERVER_INFO
typedef struct tagCMD_SERVER_INFO{
	DWORD	dwMultiAddr;
	DWORD	dwServerIndex;
	JBNV_SERVER_INFO ServerInfo;
}CMD_SERVER_INFO;


//JBNV_CHANNEL_INFO
typedef struct tagJBNV_CHANNEL_INFO
{
	DWORD	dwSize;
	DWORD	dwStream1Height;	//��Ƶ��(1)
	DWORD	dwStream1Width;	//��Ƶ��
	DWORD	dwStream1CodecID;	//��Ƶ�������ͺţ�MPEG4Ϊ								0��JPEG2000Ϊ1,H264Ϊ2��
	DWORD	dwStream2Height;	//��Ƶ��(2)
	DWORD	dwStream2Width;	//��Ƶ��
	DWORD	dwStream2CodecID;	//��Ƶ�������ͺţ�MPEG4Ϊ								0��JPEG2000Ϊ1,H264Ϊ2��
	DWORD	dwAudioChannels;	//��Ƶͨ����
	DWORD	dwAudioBits;		//��Ƶ������
	DWORD	dwAudioSamples;	//��Ƶ������
	DWORD	dwWaveFormatTag;	//��Ƶ�������ͺ�
	char		csChannelName[32];	//ͨ������
}JBNV_CHANNEL_INFO,*PJBNV_CHANNEL_INFO;

//JBNV_SENSOR_INFO
typedef struct tagJBNV_SENSOR_INFO
{
    DWORD	dwSize;
    DWORD	dwIndex;			//̽ͷ����
    DWORD	dwSensorType;			//̽ͷ����
    char		csSensorName[32];		//̽ͷ����
}JBNV_SENSOR_INFO,*PJBNV_SENSOR_INFO;


//OPEN_HEAD
typedef struct _OPEN_HEAD
{
	ULONG		nFlag2;			// JBNV_FLAG
	ULONG		nID;			//�豸��¼ʱ								CMD_SERVER_INFO�ṹ���е�dwServerIndex
	DWORD   	nProtocolType;		//0
	ULONG		nStreamType;		//�������� 0 ������ 1 ������
	ULONG		nSerChannel;		//ͨ����
	ULONG		nClientChannel;		//Ҫ��ʾ�Ĵ��ں�
}OPEN_HEAD,*POPEN_HEAD;

//JB_FRAME_HEADER
typedef struct tagJB_FRAME_HEADER
{
    WORD		wMotionDetect;		//�˶���
    WORD		wFrameIndex;		//֡����
    DWORD	nVideoSize;		//��Ƶ��С
    DWORD	nTimeTick;		//ʱ���
    WORD		nAudioSize;		//��Ƶ��С
    BYTE		bKeyFrame;		//�Ƿ�ؼ�֡
    BYTE		nPackIndex;		//������
}JB_FRAME_HEADER,*PJB_FRAME_HEADER;

//JB_DATAPACK
#define		JB_PACKDATA_SIZE	(64 *1024)
typedef struct tagJB_DATAPACK
{
    WORD	IsSampleHead;		//�Ƿ�һ֡�ĵ�һ�����ݰ�
    WORD	BufSize;			// Buf ���ж��ٿ���
    JB_FRAME_HEADER	jbFrameHeader;
    BYTE	PackData[JB_PACKDATA_SIZE];		//����Ƶ����
}JB_DATAPACK, *PJB_DATAPACK;

//JB_DATA_PACKET
typedef struct tagJB_DATA_PACKET
{
    ULONG			nFlag;		//JBNV_FLAG
    ULONG			nSize;		//packData ���ݴ�С
    JB_DATAPACK		packData;
}JB_DATA_PACKET,*PJB_DATA_PACKET;


//CHANNEL_OPTION
typedef struct tagCHANNEL_OPTION
{
	ULONG		nFlag;		//JBNV_FLAG
	ULONG		nID;		//�豸��¼ʱCMD_SERVER_INFO�ṹ							���е�dwServerIndex
	DWORD	nProtocolType;	//0
	ULONG		nStreamType;	//�������� 0 ������ 1 ������
	ULONG		nSerChannel;	//ͨ����
	ULONG		nClientChannel;	//Ҫ��ʾ�Ĵ��ں�
}CHANNEL_OPTION,*PCHANNEL_OPTION;

//CMD_CLOSE_CHANNEL
typedef struct tagCMD_CLOSE_CHANNEL
{
	JB_NET_PACK_HEAD	packhead;
	CHANNEL_OPTION	option;
}CMD_CLOSE_CHANNEL;



//JBNV_PTZ_CONTROL






//JB_SERVER_NETWORK
typedef struct tagJB_SERVER_NETWORK
{
	DWORD	dwSize;
	DWORD	dwNetIpAddr;			//IP��ַ
	DWORD       dwNetMask;			//����
	DWORD       dwGateway;			//����
	BYTE		bEnableDHCP;			//DHCP
	BYTE		bSetDNS;
	BYTE		bReserve;			//����
	BYTE		bVideoStandard;			//0 - NTSC, 1 - PAL
	DWORD       dwHttpPort;			//Http�˿�
	DWORD       dwDataPort;			//���ݶ˿�
	DWORD	dwDNSServer;			//DNS������
	DWORD	dwTalkBackIp;		//�豸�澯ʱ�Զ����ӵĶԽ�IP
	Char		szMacAddr[6];			//����MAC��ַ
	char		szServerName[32];		//����������
}JB_SERVER_NETWORK,*PJB_SERVER_NETWORK;


//DMS_NET_SUPPORT_STREAM_FMT
typedef struct tagDMS_NET_SUPPORT_STREAM_FMT
{
    DWORD       dwSize;                     //struct size
    DWORD           dwChannel;
    DWORD       dwVideoSupportFmt[DMS_MAX_STREAMNUM][4]; // Video Format.
    DMS_RANGE   stVideoBitRate[DMS_MAX_STREAMNUM];
    DMS_SIZE    stVideoSize[DMS_MAX_STREAMNUM][10];// Video Size(height,width)
    DWORD       dwAudioFmt[4];              //Audio Format
    DWORD       dwAudioSampleRate[4];       //Audio Sample Rate
    BOOL        bSupportAudioAEC;           //b Support Audio Echo Cancellation
    BYTE            byStreamCount;           //max is DMS_MAX_STREAMNUM
    BYTE            byReserve[31];
}DMS_NET_SUPPORT_STREAM_FMT;



#define PACK_SIZE (8 * 1024)
typedef struct _NET_DATA_HEAD
{
	unsigned long	nFlag;
	unsigned long	nSize;
} NET_DATA_HEAD,*PNET_DATA_HEAD;

typedef struct tagFRAME_HEADER
{
	WORD		wMotionDetect;
	WORD		wFrameIndex;
	DWORD	dwVideoSize;
	DWORD	dwTimeTick;
	WORD		wAudioSize;
	BYTE		byKeyFrame;
	BYTE		byReserved;
}FRAME_HEADER,*PFRAME_HEADER;

typedef struct tagDATA_PACKET
{
	WORD			wIsSampleHead;
	WORD			wBufSize;
	FRAME_HEADER	stFrameHeader;
	unsigned char		byPackData[PACK_SIZE];
}DATA_PACKET,*PDATA_PACKET;

typedef struct _NET_DATA_PACKET
{
    NET_DATA_HEAD       stPackHead;
    DATA_PACKET         stPackData;
} NET_DATA_PACKET,*PNET_DATA_PACKET;

/*����*/
typedef struct tagJB_SERVER_MSG{
	DWORD		dwMsg;		//��Ϣ���ʹ���
	DWORD		dwChannel;	//ͨ��
	SYSTEMTIME	st;			//������ʱ��
	DWORD		cbSize;		//�������ݳ���
}JB_SERVER_MSG,*PJB_SERVER_MSG;

typedef struct tagJBNV_SERVER_PACK
{
	char	szIp[16];			//������Ip
	WORD	wMediaPort;			//���˿�
	WORD	wWebPort;			//Http�˿ں�
	WORD	wChannelCount;		//ͨ������
	char	szServerName[32];	//��������
	DWORD	dwDeviceType;		//����������
	DWORD	dwServerVersion;	//�������汾
	WORD	wChannelStatic;		//ͨ��״̬(�Ƿ���Ƶ��ʧ)
	WORD	wSensorStatic;		//̽ͷ״̬
	WORD	wAlarmOutStatic;	//�������״̬
}JBNV_SERVER_PACK;

typedef struct tagJBNV_SERVER_PACK_EX
{
	JBNV_SERVER_PACK jspack;
	BYTE	        bMac[6];
	BOOL	        bEnableDHCP;
	BOOL			bEnableDNS;
	DWORD			dwNetMask;
	DWORD			dwGateway;
	DWORD			dwDNS;
	DWORD			dwComputerIP;
	BOOL	        bEnableCenter;
	DWORD			dwCenterIpAddress;
	DWORD			dwCenterPort;
	char			csServerNo[64];
	int				bEncodeAudio;
}JBNV_SERVER_PACK_EX;

typedef struct tagJBNV_SERVER_MSG_DATA
{
	DWORD						dwSize;
	DWORD						dwPackFlag;
	JBNV_SERVER_PACK			jbServerPack;
	JB_SERVER_MSG				jbMsg;
}JBNV_SERVER_MSG_DATA;

typedef struct tagJBNV_SERVER_MSG_DATA_EX
{
	DWORD					dwSize;
	DWORD					dwPackFlag; // == SERVER_PACK_FLAG
	JBNV_SERVER_PACK_EX		jbServerPack;
}JBNV_SERVER_MSG_DATA_EX;

/*ʱ��*/
typedef struct  _tagDMS_SYSTEMTIME
{
	WORD wYear;
	WORD wMonth;
	WORD wDayOfWeek;
	WORD wDay;
	WORD wHour;
	WORD wMinute;
	WORD wSecond;
	WORD wMilliseconds;
}DMS_SYSTEMTIME;

//����ؼ�֡
typedef struct tagJB_CHANNEL_FRAMEREQ
{
	DWORD	dwSize;
	DWORD	dwChannel;
	DWORD	dwStreamType;
	DWORD	dwFrameType;		// 0 - I�����ౣ��
}JB_CHANNEL_FRAMEREQ, *PJB_CHANNEL_FRAMEREQ;

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////
//���Ӳ���
///////////////////////////////////////////////


#define DMS_MAX_OSD_LEN 	128
#define DMS_MAX_VIDEO_HIDE_RECT 		5
#define DMS_MAX_OSD_NUM 				5  //1·��Ƶ�������֧����Ƶ���Ӹ���
#define DMS_NAME_LEN 				32
#define DMS_SERIALNO_LEN 				48
#define DMS_MACADDR_LEN 				6
#define DMS_MAX_ETHERNET 				2
#define DMS_PATHNAME_LEN 			128
#define DMS_PASSWD_LEN 		 		32
#define DMS_MAX_CHANNUM 			32
#define DMS_MAX_STREAMNUM 		3
#define DMS_MAX_ALARMIN 				16
#define DMS_MAX_ALARMOUT 			8
#define DMS_MAX_DAYS 				8
#define DMS_MAX_TIMESEGMENT 			4
#define DMS_MAX_PRESET					255
#define DMS_MAX_CRUISE_GROUP_NUM 	    5
#define DMS_MAX_CRUISE_POINT_NUM 		32
#define DMS_PHONENUMBER_LEN 			32
#define DMS_MAX_DISKNUM 			16
#define DMS_MAX_WINDOW				16
#define DMS_MAX_PLAYER_WINDOW 	16
#define DMS_MAX_VGA 			 		1
#define DMS_MAX_USERNUM 				32
#define DMS_MAX_USERGROUPNUM 			8
#define DMS_MAX_EXCEPTIONNUM 			16
#define DMS_MAX_LINK 					6
#define DMS_MAXCARD_NUM 			40
#define DMS_MAX_SERIALNUM 		3
#define DMS_CARDNUM_LEN 				20
#define DMS_MAX_VIDEOOUT 				2
#define DMS_MAX_DISPLAY_REGION			16
#define DMS_MAX_RIGHT					32
#define DMS_MAX_DOMAIN_NAME 			64
#define DMS_MAX_PREVIEW_MODE 			8
#define DMS_PTZ_PROTOCOL_NUM  	200   /* ���֧�ֵ���̨Э���� */
#define DMS_MAX_HD_GROUP 			16
#define DMS_MAX_DDNS_NUMS 		3
#define DMS_MAX_IP_LENGTH 		16
#define DMS_MAX_IPV6_LENGTH          128
#define DMS_MAX_PATH_LEN 			256
#define DMS_EMAIL_ADDR_MAXNUM (3)

#define DMS_MD_STRIDE_WIDTH     44
#define DMS_MD_STRIDE_HEIGHT    36
#define DMS_MD_STRIDE_SIZE        (DMS_MD_STRIDE_WIDTH*DMS_MD_STRIDE_HEIGHT/8)

typedef struct
{
    char  csIpV4[DMS_MAX_IP_LENGTH];
    BYTE  byRes[DMS_MAX_IPV6_LENGTH];
}DMS_NET_IPADDR, *LPDMS_NET_IPADDR;


typedef enum __DMS_DDNS_TYPE
{
    DDNS_3322 = 1,
    DDNS_DYNDNS,
    DDNS_NIGHTOWLDVR,
    DDNS_NOIP,
    DDNS_MYEYE,
    DDNS_PEANUTHULL,
    DDNS_ANSHIBAO,//see-world.cc
	DDNS_OURVIDEO,//0328
    DDNS_BUTT,
}DMS_DDNS_TYPE;

typedef struct
{
    DMS_NET_IPADDR  strIPAddr;
    DMS_NET_IPADDR  strIPMask;  //�����ַ
    DWORD dwNetInterface;       //����ӿ� 1-10MBase-T 2-10MBase-Tȫ˫�� 3-100MBase-TX 4-100Mȫ˫�� 5-10M/100M����Ӧ
    WORD  wDataPort;             //���ݶ˿�
    WORD  wMTU;                 //MTU��С
    BYTE  byMACAddr[DMS_MACADDR_LEN];   //Mac��ַ
    char   csNetName[DMS_NAME_LEN]; //�������ƣ����ڶ���������
    BYTE  byRes[2];
}DMS_NET_ETHERNET, *LPDMS_NET_ETHERNET;

typedef struct tagDMS_TIME
{
    DWORD    dwYear;
    DWORD    dwMonth;
    DWORD    dwDay;
    DWORD    dwHour;
    DWORD    dwMinute;
    DWORD    dwSecond;
}DMS_TIME, *LPDMS_TIME;

typedef struct DMS_NET_EMAIL_ADDR
{
    char  csName[DMS_NAME_LEN];/**< �ʼ���ַ��Ӧ���û��� */
    char  csAddress[DMS_MAX_DOMAIN_NAME];   /**< �ʼ���ַ ��: hw@huawei.com */
    BYTE byReserve[32];               /**< ������,�����۽��Ԥ��32�ֽ� */
} DMS_NET_EMAIL_ADDR;

/** ¼��ƻ�*/
typedef struct
{
    BYTE    byRecordType[7][48]; /* ¼������ ��λ��Ч��
    								��DMS_NET_RECORD_TYPE_SCHED/MOTION/ALARM (������DMS_NET_RECORD_TYPE_MANU)*/
    BYTE    byHour[7][48];     /**< ¼��ƻ�ʱ����Ϊ��λ*/
} DMS_RECORDSCHED;




typedef struct hiOSD_Net_S
{
    BOOL        bOsdEnable;         /**< OSDʹ��*/
    DWORD   dwOsdContentType;    /**< OSD��������HI_CFG_OSD_TYPE_E */
    DWORD   dwLayer;             /**< �����Σ��ڶ���������ʱ����,0 ~100*/
    DWORD   dwAreaAlpha;         /**< OSD����Alphaֵ*/
    DWORD   dwFgColor;           /**< OSD��ɫ, ���ظ�ʽRGB8888 */
    DWORD   dwBgColor;           /**< OSD������ɫ, ���ظ�ʽRGB8888 */
    int     nLeftX;       /**< �������Ͻ�����ڴ������Ͻ�X�������:0~255*/
    int     nLeftY;       /**< �������Ͻ�����ڴ������Ͻ�Y�������:0~255*/
    int     nWidth;       /**< �����Ϊ����ֵ,ͼƬʱ��Ч*/
    int     nHeight;       /**< ����ߣ�Ϊ����ֵ,ͼƬʱ��Ч*/
    char        csOsdCotent[DMS_MAX_OSD_LEN];/**< OSDΪ�ַ�ʱ��֧������string��
                                                 OSDΪͼƬʱ��֧��path: "/bin/osd.bmp";
                                                 �����ͼƬ�ڴ棬ǰ4λΪ�ڴ��ַ*/
}DMS_NET_OSD_S;


//�ڵ���������
typedef struct tagDMS_NET_SHELTER_RECT
{
    int     nType;          /* �ڵ�ʹ�� 0-����, 1-�����ڵ�, 2-Ԥ���ڵ�, 3-ȫ���ڵ� */
    WORD    wLeft;          /* �ڵ��������Ͻǵ�x���� */
    WORD    wTop;           /* �ڵ��������Ͻǵ�y���� */
    WORD    wWidth;         /* �ڵ������� */
    WORD    wHeight;        /* �ڵ�����߶� */
    DWORD   dwColor;        /* �ڵ���ɫ, Ĭ�� 0:��ɫ ��RGB��ʽ*/
}DMS_NET_SHELTER_RECT;

//��Ƶͨ��ѹ������
typedef struct tagDMS_NET_COMPRESSION_INFO
{
    DWORD   dwCompressionType;      //
    DWORD   dwFrameRate;            //֡�� (1-25/30) PALΪ25��NTSCΪ30
    DWORD   dwStreamFormat;         //��Ƶ�ֱ��� (0ΪCIF,1ΪD1,2ΪHALF-D1,3ΪQCIF) DMS_VIDEO_FORMAT_CIF
    WORD    wHeight;            //����ʵ����;��ֻ������ʾ
    WORD    wWidth;             //����ʵ����;��ֻ������ʾ
    DWORD   dwRateType;         //��ģʽ(0Ϊ��������1Ϊ������)
    DWORD   dwBitRate;          //���� (16000-4096000)
    DWORD   dwImageQuality;     //��������(0-4),0Ϊ���
    DWORD   dwMaxKeyInterval;   //�ؼ�֡���(1-100)

    WORD    wEncodeAudio;       //�Ƿ������Ƶ
    WORD    wEncodeVideo;       //�Ƿ������Ƶ

    WORD    wFormatTag;        /* format type */
    WORD    wBitsPerSample;    /* Number of bits per sample of mono data */

    BYTE    byReseved[16];
}DMS_NET_COMPRESSION_INFO, *LPDMS_NET_COMPRESSION_INFO;


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






//�����ļ�ͷ��Ϣ
typedef struct tagDMS_UPGRADE_FILE_HEADER
{
    DWORD   dwFlag;         //�����ļ�����(DMS_UPGRADE_FILE_FLAG)
    DWORD   dwSize;         //���ṹ���С
    DWORD   dwVersion;      //�ļ��汾
    DWORD   dwItemCount;    //���ڰ������ļ�����
    DWORD   dwPackVer;      //����İ汾
    DWORD   dwCPUType;      //CPU����
    DWORD   dwChannel;      //������ͨ������0x01 | 0x04 | 0x08
    DWORD   dwServerType;   //����������
    char        csDescrip[64];      //������Ϣ
    BYTE    byReserved[32];
}DMS_UPGRADE_FILE_HEADER, *LPDMS_UPGRADE_FILE_HEADER;

//�����ļ�����ͷ��Ϣ
typedef struct tagUPGRADE_ITEM_HEADER
{
    DWORD   dwSize;
    DWORD   dwDataType;    //����������
    DWORD   dwDataLen;     //���ݳ���
    DWORD   dwCRC;         //����CRCУ��
    DWORD   dwDataPos;     //����λ��
    DWORD   dwVersion;
}UPGRADE_ITEM_HEADER;

//������������
typedef struct tagDMS_NET_UPGRADE_REQ
{
    DMS_UPGRADE_FILE_HEADER  FileHdr;
    UPGRADE_ITEM_HEADER         stItemHdr[10];
}DMS_NET_UPGRADE_REQ;

//��������Ӧ��
typedef struct tagDMS_NET_UPGRADE_RESP
{
    DWORD   dwSize;
    int     nResult;        //����Ӧ����
    char        reserved[32];
}DMS_NET_UPGRADE_RESP;

//��������
typedef struct tagDMS_NET_UPGRADE_DATA
{
    DWORD   dwSize;
    DWORD   dwFileLength;       //���������ļ�����
    DWORD   dwPackNo;           //�ְ���ţ���0��ʼ
    DWORD   dwPackSize;         //�ְ���С
    BOOL    bEndFile;           //�Ƿ������һ���ְ�
    char        reserved[32];
    BYTE    *pData;
}DMS_NET_UPGRADE_DATA;



/*ʱ������ʱ�Ʋ���*/
typedef struct
{
    DWORD dwMonth;  //�� 0-11��ʾ1-12����
    DWORD dwWeekNo;  //�ڼ��ܣ�0����1�ܣ�1����2�ܣ�2����3�ܣ� 3����4�ܣ�4�����һ��
    DWORD dwWeekDate;  //���ڼ���0�������գ�1������һ��2�����ڶ���3����������4�������ģ�5�������壻6��������
    DWORD dwHour; //Сʱ����ʼʱ��ȡֵ��Χ0��23�� ����ʱ��ȡֵ��Χ1��23
    DWORD dwMin; //��0��59
}DMS_NET_TIMEPOINT;





/*�쳣��������*/
typedef struct
{
    DWORD dwSize;
    DMS_HANDLE_EXCEPTION stExceptionHandleType[DMS_MAX_EXCEPTIONNUM];
    /*����0-����,1- Ӳ�̳���,2-���߶�,3-��������IP ��ַ��ͻ,4-�Ƿ�����, 5-����/�����Ƶ��ʽ��ƥ��*/
}DMS_NET_EXCEPTION,*LPDMS_NET_EXCEPTION;

typedef struct
{
    DWORD   dwSize;           //���ṹ��С
    DWORD   dwMask;           //���룬��λ��Ч
    BYTE        byRes[16];
}DMS_NET_RESTORECFG, *LPDMS_NET_RESTORECFG;

//�������������
typedef struct tagDMS_NET_ALARMOUT_CONTROL
{
    DWORD   dwSize;
    int     nChannel;
    WORD    wAction;     //0:ֹͣ��� 1:��ʼ���
    WORD    wDuration;   //����ʱ������Ϊ��λ��0xFFFFFFFF��ʾ��ֹͣ��0��ʾ����ֹͣ
}DMS_NET_ALARMOUT_CONTROL;


/*�������*/
typedef struct
{
    DWORD   dwSize; /* �˽ṹ�Ĵ�С */
    DWORD   dwChannel;
    char    csAlarmOutName[DMS_NAME_LEN]; /* ���� */
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
}DMS_NET_ALARMOUTCFG, *LPDMS_NET_ALARMOUTCFG;

//������״̬��ȡ
typedef struct tagDMS_NET_SENSOR_STATE
{
    DWORD   dwSize;
    //0xfffffff��ʶ���б����������ͨ������λ��Ч��
    //�ӵ͵���ǰ16λ��ʾ��������ͨ����λ��������16λ��ʾ�������ͨ��������
    DWORD   dwSensorID;
    //0xfffffff��ʶ���б����������ͨ��״̬����λ��Ч��
    //�ӵ͵���ǰ16λ��ʾ��������ͨ��״̬λ����16λ��ʾ�������ͨ��״̬����
    DWORD   dwSensorOut;
}DMS_NET_SENSOR_STATE;


/*��������*/
typedef struct
{
    DWORD   dwSize;                 /* �˽ṹ�Ĵ�С */
    DWORD   dwChannel;
    char    csAlarmInName[DMS_NAME_LEN];    /* ���� */
    BYTE    byAlarmType;                /* ���������ͣ�Ŀǰ�汾������,0������,1������ */
    BYTE    byAlarmInHandle;            /* �Ƿ��� 0��������,1������*/
    char    reserve[2];
    DMS_HANDLE_EXCEPTION stHandle; /* ����ʽ */
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
}DMS_NET_ALARMINCFG, *LPDMS_NET_ALARMINCFG;

typedef struct tagDMS_NET_PRESET_INFO
{
    DWORD 		dwSize;
    unsigned short   nChannel;
    unsigned short   nPresetNum;                   //Ԥ�õ����
    unsigned int   no[DMS_MAX_PRESET];        //Ԥ�õ����
    char           csName[DMS_MAX_PRESET][64];//Ԥ�õ�����
}DMS_NET_PRESET_INFO;

typedef struct
{
	BYTE 	byPointIndex;	//Ѳ�����е��±�,���ֵ����DMS_MAX_CRUISE_POINT_NUM��ʾ��ӵ�ĩβ
	BYTE 	byPresetNo;	//Ԥ�õ���
	BYTE 	byRemainTime;	//Ԥ�õ�����ʱ��
	BYTE 	bySpeed;		//��Ԥ�õ��ٶ�
}DMS_NET_CRUISE_POINT;

typedef struct
{
	BYTE byPointNum; 		//Ԥ�õ�����
	BYTE byCruiseIndex;	//��Ѳ�������
	BYTE byRes[2];
	DMS_NET_CRUISE_POINT struCruisePoint[DMS_MAX_CRUISE_POINT_NUM];
}DMS_NET_CRUISE_INFO;

typedef struct
{
	DWORD dwSize;
	int     nChannel;
	BYTE     byIsCruising;		//�Ƿ���Ѳ��
	BYTE     byCruisingIndex;	//����Ѳ����Ѳ�����
	BYTE     byPointIndex;		//����Ѳ����Ԥ�õ����(�����±�)
	BYTE     byEnableCruise;;	//�Ƿ���Ѳ��
	DMS_NET_CRUISE_INFO struCruise[DMS_MAX_CRUISE_GROUP_NUM];
}DMS_NET_CRUISE_CFG;


//��̨����
typedef struct tagDMS_NET_PTZ_CONTROL
{
    DWORD   dwSize;
    int     nChannel;		//ͨ����
    DWORD  	dwCommand;		//��̨����������DMS_PTZ_CMD_UP
    DWORD 	dwParam;		//��̨���Ʋ���(Ѳ�����)	//�ٶ�ȡֵ��Χ[1,64)
    BYTE    byRes[64];		//Ԥ�õ�����  (����Ѳ�����ʱ��ΪDMS_NET_CRUISE_POINT����)
}DMS_NET_PTZ_CONTROL;



typedef struct tagDMS_NET_CHANNEL_SHELTER
{
    DWORD   dwSize;
    DWORD   dwChannel;
    BOOL    bEnable;        //�Ƿ���������ڵ�
    DMS_NET_SHELTER_RECT    strcShelter[DMS_MAX_VIDEO_HIDE_RECT];   //�ڵ��������֧��5��������ڵ���RECT��D1Ϊ׼
}DMS_NET_CHANNEL_SHELTER,*PDMS_NET_CHANNEL_SHELTER;


// ��������
typedef enum __LANGUAGE_TYPE_E
{
    DMS_LANGUAGE_CHINESE_SIMPLIFIED = 0,	 //����
    DMS_LANGUAGE_ENGLISH = 1,		         //Ӣ��     
    DMS_LANGUAGE_RUSSIAN = 2,		         //���     
    DMS_LANGUAGE_PORTUGUESE = 3,	         //������   
    DMS_LANGUAGE_SPANISH = 4,		         //������   
    DMS_LANGUAGE_TURKEY = 5,		         //������   
    DMS_LANGUAGE_ITALIAN = 6,		         //����� 
    DMS_LANGUAGE_FARSI = 7,		             //��˹     
    DMS_LANGUAGE_FRENCH = 8,		         //����     
    DMS_LANGUAGE_POLISH = 9,		         //����     
    DMS_LANGUAGE_THAI = 10,			         //̩��     
    DMS_LANGUAGE_SERBIAN = 11,		         //����ά�� 
    DMS_LANGUAGE_CHINESE_TRADITIONAL = 12,	 //����     
    DMS_LANGUAGE_JAPANESE = 13,		         //�ձ�     
    DMS_LANGUAGE_SABRA = 14,		         //ϣ����
    DMS_LANGUAGE_KOREAN = 15,		         //����     
    DMS_LANGUAGE_VIETNAMESE = 16,	         //Խ��     
    DMS_LANGUAGE_KAZAKHSTAN = 17,            //������˹̹      
	DMS_LANGUAGE_GERMAN = 18 ,               //����
	DMS_LANGUAGE_BUTT,                       //����
}DMS_LANGUAGE_TYPE_E;


/* DEVICE SYSTEM INFO �豸��Ϣ*/
typedef struct tagDMS_NET_DEVICE_INFO {
    DWORD		dwSize;
    char		csDeviceName[DMS_NAME_LEN];	//DVR����
    DWORD		dwDeviceID;					//DVR ID,����ң����

    BYTE		byRecordLen;				//¼���ļ����ʱ��,�Է���Ϊ��λ
    BYTE		byLanguage;					//��������,��ϸ�� DMS_LANGUAGE_TYPE
    BYTE		byRecycleRecord;			//�Ƿ�ѭ��¼��,0:����; 1:��
    BYTE		byOverWritePeriod;			//¼�񸲸�����/ ��СʱΪ��λ

    BYTE		byVideoStandard;			//��Ƶ��ʽ
    BYTE		byDateFormat;				/*���ڸ�ʽ(�����ո�ʽ)��
    											0��YYYY-MM-DD 		������
    											1��MM-DD-YYYY 		������
    											2��YYYY/MM/DD 		������
											3��MM/DD/YYYY 		������
											4��DD-MM-YYYY 		������
											5��DD/MM/YYYY 		������

    										*/

    BYTE		byDateSprtr;				//���ڷָ��(0��":"��1��"-"��2��"/"). 2011-01-21 yxhu: ��Ч�ֶ�
    BYTE		byTimeFmt;					//ʱ���ʽ (0-24Сʱ��1��12Сʱ).

    BYTE		byConfigWizard;				//�Ƿ������˿�����
    BYTE		byReserve[3];

    //���²����޸�
    char		bySoftwareVersion[32];          //����汾�ţ���16λ�����汾����16λ�Ǵΰ汾
    char		bySoftwareBuildDate[32];        //����������ڣ�0xYYYYMMDD
    char		byDspSoftwareVersion[32];
    char		byDspSoftwareBuildDate[32];
    char		byPanelVersion[32];
    char		byPanelSoftwareBuildDate[32];
    char		byHardwareVersion[32];
    char		byHardwareBuildDate[32];
    char		byWebVersion[32];
    char		byWebBuildDate[32];

    char		csSerialNumber[DMS_SERIALNO_LEN];//���к�
    DWORD		dwServerCPUType;			//��ǰCPU����(DMS_CPU_TYPE_E)
    DWORD		dwSysFlags;					//ϵͳ֧�ֵĹ���
    DWORD		dwServerType;				//�豸����(DMS_SERVER_TYPE_E)

    BYTE		byVideoInNum;				/* ���֧�ֵ�Video Channel���� */
    BYTE		byAudioInNum;				/* ���֧�ֵ�Audio Channel���� */
    BYTE		byAlarmInNum;				//DVR�����������
    BYTE		byAlarmOutNum;				//DVR�����������

    BYTE		byDiskNum;					//DVR Ӳ�̸���
    BYTE		byRS232Num;					//DVR 232���ڸ���
    BYTE		byRS485Num;					//DVR 485���ڸ���
    BYTE		byNetworkPortNum;			//����ڸ���

    BYTE		byDecordChans;				//DVR ����·��
    BYTE		byVGANum;					//VGA�ڵĸ���
    BYTE		byUSBNum;					//USB�ڵĸ���
    BYTE		byDiskCtrlNum;				//Ӳ�̿���������

    BYTE		byAuxOutNum;				//���ڵĸ���
    BYTE		byStreamNum;				//ÿ·��Ƶ������֧����Ƶ����
	BYTE		byIPChanNum;				//����ͨ����
    BYTE		byResvered;
} DMS_NET_DEVICE_INFO;


typedef struct tagDMS_NET_WANWIRELESS_CONFIG_V2
{
	DWORD       dwSize;
	BOOL        Enable3G;
	BYTE        byCardNum3G; 			//3G����
	BYTE        bySeviceProvider3G;		//3G��Ӫ��:�Զ�����ƶ�,��ͨ,����
	BYTE        byStatus;				//0:�Ͽ�����, 1:���ųɹ�, 2:���ڲ���
	BYTE        bySigs;					//�ź�ǿ��
	char	    csApn3G[32];			//3G APN:  �Զ����cmnet\3gnet\ctnet
	char       	csUsrName3G[32];		//3G�û���
	char		csUsrPSWD3G[32];		//3G�û�����
	char		csPhoneNumber3G[18];	//3G�绰����
	BYTE        byReserve2[14];
	DMS_NET_IPADDR stNetIpaddr;			//3G��IP
}DMS_NET_WANWIRELESS_CONFIG_V2, *LPDMS_NET_WANWIRELESS_CONFIG_V2;



typedef struct tagDMS_NET_CHANNEL_PIC_INFO
{
    DWORD   dwSize;
    DWORD   dwChannel;
    char    csChannelName[DMS_NAME_LEN];    //ͨ������

    DMS_NET_COMPRESSION_INFO   stRecordPara; /* ¼�� */
    DMS_NET_COMPRESSION_INFO   stNetPara;    /* ���� */
    DMS_NET_COMPRESSION_INFO   stPhonePara;  /* �ֻ��࿴ */
    DMS_NET_COMPRESSION_INFO   stEventRecordPara; /*�¼�����¼��ѹ������*/
}DMS_NET_CHANNEL_PIC_INFO, *LPDMS_NET_CHANNEL_PIC_INFO;


//Color DMS_RANGE
typedef struct tagDMS_NET_COLOR_SUPPORT
{
    DWORD           dwSize;                     //struct size
    DWORD           dwMask;              //��λ��Ч(DMS_COLOR_SET_BRIGHTNESS)

    DMS_RANGE       strBrightness;
    DMS_RANGE       strHue;
    DMS_RANGE       strSaturation;
    DMS_RANGE       strContrast;
    DMS_RANGE       strDefinition;
    BYTE                byReseved[32];
}DMS_NET_COLOR_SUPPORT;

typedef struct tagDMS_NET_ENHANCED_COLOR_SUPPORT
{
    DWORD           dwSize;                     //struct size
    DWORD           dwMask;              //��λ��Ч(DMS_COLOR_SET_BRIGHTNESS)

    DMS_RANGE       strDenoise; 		//����
    DMS_RANGE       strIrisBasic;		//��Ȧ��׼ֵ
    DMS_RANGE       strRed;
    DMS_RANGE       strBlue;
    DMS_RANGE       strGreen;
    DMS_RANGE       strGamma;

    DMS_RANGE       strEC;		//�عֵⷧ
    DMS_RANGE       strGC;		//���淧ֵ
    DMS_RANGE       strWD;		//��̬����
    BYTE                byReseved[32];
}DMS_NET_ENHANCED_COLOR_SUPPORT;

//ͨ����ɫ����
typedef struct tagDMS_NET_CHANNEL_ENHANCED_COLOR
{
    DWORD   dwSize;
    DWORD   dwChannel;
    DWORD   dwSetFlag;              //0,���õ�������;1,�������;2,�ָ���һ�α����

    BOOL    bEnableAutoDenoise;     //�Զ�����ʹ��
    int     nDenoise;               //���뷧ֵ

    BOOL    bEnableAWB;             //�Զ���ƽ��
    int     nRed;
    int     nBlue;
    int     nGreen;

    BOOL    bEnableAEC;             //�Զ��ع�(���ӿ���)ʹ��
    int     nEC;                    //�Զ��ع�(���ӿ���)��ֵ

    BOOL    bEnableAGC;             //�Զ�����/�ֶ�����
    int     GC;                     //�Զ�����/�ֶ����淧ֵ

    int     nMirror;                //0:������, 1: ���¾���, 2: ���Ҿ���  3: �������Ҿ���
    BOOL    bEnableBAW;             //�ֶ���ת��ʹ��0,�Զ�1,ҹ��,2 ����

    int     nIrisBasic;             //���
    int     nGamma;
    BOOL    bWideDynamic;           //��̬ʹ��
    int     nWDLevel;               //��̬�ȼ�

    int     nSceneMode;             //����ģʽ
                                    //0x00    /*scene��outdoor*/
                                    //0x01    /*scene��indoor*/
                                    //0x02    /*scene��manual*/
                                    //0x03    /*scene��auto*/
    BOOL    bEnableAIris;           //�Զ���Ȧ
    BOOL    bEnableBLC;             //�������ʹ��
    BYTE    byReseved[28];
}DMS_NET_CHANNEL_ENHANCED_COLOR, *PDMS_NET_CHANNEL_ENHANCED_COLOR;


//Save Flag
typedef enum emSaveFlag
{
	DMS_COLOR_NO_SAVE = 0,
	DMS_COLOR_SAVE = 1,
	DMS_COLOR_SET_DEF = 2,
} eSaveFlag;

typedef struct tagDMS_NET_CHANNEL_COLOR_SINGLE
{
    DWORD       dwSize;                     //struct size
    DWORD           dwChannel;
    DWORD       dwSetFlag;
    DWORD       dwSaveFlag;
    int             nValue;
    int             nParam;
}DMS_NET_CHANNEL_COLOR_SINGLE;

//ͨ����ɫ����
typedef struct tagDMS_NET_CHANNEL_COLOR
{
    DWORD   dwSize;
    DWORD   dwChannel;
    DWORD   dwSetFlag;      //0,���õ�������;1,�������;2,�ָ���һ�α����
    int     nHue;           //ɫ�� 0-255
    int     nSaturation;        //���Ͷ� 0-255
    int     nContrast;      //�Աȶ� 0-255
    int     nBrightness;    //���� 0-255
    int     nDefinition;    //������ 0-255
}DMS_NET_CHANNEL_COLOR, *PDMS_NET_CHANNEL_COLOR;


typedef struct tagDMS_NET_DAY_NIGHT_DETECTION_EX
{
    DWORD dwSize;
    DWORD dwChannel;
    BYTE byMode;      // 0���Զ���⣬1�����������⣬2����Ƶ��⣬3��ʱ����
    //����������
    BYTE byTrigger;    // ����������-0���͵�ƽ��Ч��1���ߵ�ƽ��Ч
    //��Ƶ���
    BYTE byAGCSensitivity;// ������,0-5
    BYTE byDelay;      // �л���ʱʱ��0-10s��

    BYTE byIRCutLevel;// IRCUT����-0���͵�ƽ��Ч��1���ߵ�ƽ��Ч
    BYTE byLedLevel; // ����ƿ���: 0:�͵�ƽ 1:�ߵ�ƽ
    BYTE reserve[2];
    //ʱ����
    DMS_SCHEDTIME stColorTime; //��ɫ��ʱ��Σ����ڴ�ʱ�������Ϊ�Ǻڰס�
}DMS_NET_DAY_NIGHT_DETECTION_EX;

/**UPNP**/
typedef struct tagDMS_UPNP_CONFIG
{
    DWORD       dwSize;
    BOOL        bEnable;                /*�Ƿ�����upnp*/
    DWORD       dwMode;                 /*upnp������ʽ.0Ϊ�Զ��˿�ӳ�䣬1Ϊָ���˿�ӳ��*/
    DWORD       dwLineMode;             /*upnp����������ʽ.0Ϊ��������,1Ϊ��������*/
    char            csServerIp[32];         /*upnpӳ������.������·����IP*/
    DWORD       dwDataPort;             /*upnpӳ�����ݶ˿�*/
    DWORD       dwWebPort;              /*upnpӳ������˿�*/
    DWORD       dwMobilePort;           /*upnpӳ���ֻ��˿�*/

    WORD        dwDataPort1;            /*upnp��ӳ��ɹ������ݶ˿�*/
    WORD        dwWebPort1;             /*upnp��ӳ��ɹ�������˿�*/
    WORD        dwMobilePort1;          /*upnpӳ��ɹ����ֻ��˿�*/
    WORD            wDataPortOK;
    WORD            wWebPortOK;
    WORD            wMobilePortOK;
}DMS_UPNP_CONFIG, *LPDMS_UPNP_CONFIG;

/* ftp�ϴ�����*/
typedef struct tagDMS_NET_FTP_PARAM
{
    DWORD   dwSize;
    BOOL    bEnableFTP;                             /* �Ƿ�����ftp�ϴ�����*/
    char    csFTPIpAddress[DMS_MAX_DOMAIN_NAME];    /* ftp ��������������IP��ַ������*/
    DWORD   dwFTPPort;                              /* ftp �˿�*/
    char    csUserName[DMS_NAME_LEN];               /* �û���*/
    char    csPassword[DMS_PASSWD_LEN];             /* ����      */
    WORD    wTopDirMode;                /* 0x0 = ʹ���豸ip��ַ, 0x1 = ʹ���豸��,0x2 = OFF*/
    WORD    wSubDirMode;                /* 0x0 = ʹ��ͨ���� ,        0x1 = ʹ��ͨ����,0x2 = OFF*/
    BYTE    byTestStatus;			    /* 0: ���ӳɹ�  1������ʧ��    BYTE    reservedData[27];  */
    BYTE    reservedData[27];           /* ���� */
}DMS_NET_FTP_PARAM, *LPDMS_NET_FTP_PARAM;

typedef struct tagDMS_NET_CHANNEL_OSDINFO
{
    DWORD   dwSize;
    DWORD   dwChannel;
    BOOL    bShowTime;
    BOOL    bDispWeek; /*�Ƿ���ʾ����*/
    DWORD   dwTimeTopLeftX; /*OSD���Ͻǵ�x����*/
    DWORD   dwTimeTopLeftY;/*OSD���Ͻǵ�y����*/
    BYTE    byReserve1;// 2011-01-21 yxhu: ��Ч�ֶΣ�ֱ�Ӳ���tagDMS_NET_DEVICE_INFO �ṹ��
                 /*OSD����(�����ո�ʽ)��
                */
    BYTE       byOSDAttrib;
                /*
                OSD���ԣ�͸��/��˸����
                1��͸������˸
                2��͸��������˸
                3����˸����͸��
                4����͸��������˸
                */
    BYTE    byReserve2;  // 2011-01-21 yxhu: ��Ч�ֶΣ�ֱ�Ӳ���tagDMS_NET_DEVICE_INFO �ṹ��

    BYTE    byShowChanName ;    //Ԥ����ͼ�����Ƿ���ʾͨ�����ƣ�0-����ʾ��1-��ʾ�������С704*576��
    DWORD   dwShowNameTopLeftX ;    //ͨ��������ʾλ�õ�x����
    DWORD   dwShowNameTopLeftY ;    //ͨ��������ʾλ�õ�y����
    char    csChannelName[DMS_NAME_LEN];    //ͨ������
    DMS_NET_OSD_S stOsd[DMS_MAX_OSD_NUM];/**Դͨ������OSD 2011-01-21 yxhu: �ֶ�Ԥ��*/
}DMS_NET_CHANNEL_OSDINFO, *LPDMS_NET_CHANNEL_OSDINFO;


/**�ֻ�ƽ̨ע�����**/
typedef struct tagDMS_MOBILE_CENTER_INFO
{
    DWORD   dwSize;
    BOOL    bEnable;
    char        szIp[64];                   //������Ip
    DWORD   dwCenterPort;               //���Ķ˿�
    char        csServerNo[64];             //���������к�
    DWORD   dwStatus;                   //����������״̬ 0Ϊδ���� 1Ϊ������ 2���ӳɹ�
    char    csUserName[32];             //�û���
    char    csPassWord[32];             //����
    BYTE    byAlarmPush;
    BYTE    reservedData[3];
    DWORD   reserve1[1];            //����
}DMS_MOBILE_CENTER_INFO, *LPDMS_MOBILE_CENTER_INFO;

//RTSP����
typedef struct tagDMS_NET_RTSP_CFG
{
    DWORD       dwSize;
    DWORD       dwPort;             //RTSP�˿�
    BYTE        byReserve[32];
}DMS_NET_RTSP_CFG,*PDMS_NET_RTSP_CFG;

typedef struct tagDMS_NET_NETWORK_CFG
{
    DWORD       dwSize;
    DMS_NET_ETHERNET    stEtherNet[DMS_MAX_ETHERNET];

    DMS_NET_IPADDR    stMulticastIpAddr;
    DMS_NET_IPADDR    stGatewayIpAddr;
    DMS_NET_IPADDR    stManagerIpAddr;    //Զ�̹���������ַ(�������澯ʱ���Զ����ӵĶԽ�IP)
    DMS_NET_IPADDR    stDnsServer1IpAddr;
    DMS_NET_IPADDR    stDnsServer2IpAddr;

    BYTE        byEnableDHCP;         //
    BYTE        byMaxConnect;         //�������������
    BYTE        byReserve[2];         //����
    WORD            wHttpPort;            //Http�˿�
    WORD        wReserve1;            //yxhu 2013-03-07 Ԥ��
    WORD            wManagerPort;         //Զ�̹��������˿�
    WORD            wMulticastPort;       //�ಥ�˿�
    //BYTE        byAutoDHCPDNS;      //yxhu ������dns��ip��ַ����byEnableDHCP
    DWORD       dwReserve2;
}DMS_NET_NETWORK_CFG, *PDMS_NET_NETWORK_CFG;

typedef struct
{
    DWORD dwSize;
    BOOL  bPPPoEEnable;              //0-������,1-����
    char  csPPPoEUser[DMS_NAME_LEN]; //PPPoE�û���
    char  csPPPoEPassword[DMS_PASSWD_LEN]; //PPPoE����
    DMS_NET_IPADDR  stPPPoEIP; //PPPoE IP��ַ(ֻ��)
    DWORD dwSecurityProtocol; /**< ����Э�� ֵ��Χ�ο�:HI_CFG_PPPOE_SECURITY_PROTOCOL_E*/
}DMS_NET_PPPOECFG, *LPDMS_NET_PPPOECFG;

typedef struct tagDMS_NET_P2P_CFG
{
	DWORD dwSize;
	DWORD dwEnable;
	char  	  csID[64];
	BYTE 	  byStatus;
	//char      csRes[63];
	BYTE	u8p2ptype;	//0:tutk  1:langtao
	char      u8mac[20];
	char      csRes[42];
}DMS_NET_P2P_CFG;

typedef struct tagDMS_NET_FAC_CFG
{
    DWORD dwSize;
    char  language; //0:Chinese; 1:English
    char  csRes[128];
}DMS_NET_FAC_CFG;


//�豸�Զ�ά������
typedef struct tagDMS_NET_DEVICEMAINTAIN
{
	DWORD dwSize;
	BYTE byEnable;
	BYTE byIndex;
	SYSTEMTIME stDeviceMaintain;
}DMS_NET_DEVICEMAINTAIN;

typedef struct tagDMS_NET_LOG_INFO
{
    DMS_TIME   stSysStartTime;
    DMS_TIME   stSysRunTime;
    char       szReserve[16];
}DMS_NET_LOG_INFO;


typedef struct tagDMS_NET_RECDAYINMONTH
{
	DWORD dwSize;
	DWORD nChannel;
	unsigned char	year;//��ѯ��,0~255ȡֵ��Χ������2000~2255��
	unsigned char	month;//��ѯ�£�1~12Ϊȡֵ��Χ
	unsigned char	type;//�ļ����� ��0xff �� ȫ����0 �� ��ʱ¼��1 - �ƶ���⣬2 �� ����������3  �� �ֶ�¼��
	LONG lCalendarMap;
}DMS_NET_RECDAYINMONTH, *LPDMS_NET_RECDAYINMONTH;


typedef struct tagDMS_NET_EMAIL_PARAM
{
    DWORD	dwSize;

    BOOL	bEnableEmail;                   //�Ƿ�����

    BYTE	byAttachPicture;                //�Ƿ������
    BYTE	bySmtpServerVerify;             //���ͷ�����Ҫ�������֤
    BYTE	byMailInterval;                 //����2s��(1-2�룻2-3�룻3-4�룻4-5��)
    BYTE	byTestStatus;                   //0: ���ͳɹ�  1������ʧ��

    WORD	wServicePort;					/**< �������˿�,һ��Ϊ25���û����ݾ������������ */
    WORD	wEncryptionType;				/**< �������� ssl*/

    char	csEMailUser[DMS_NAME_LEN];      //�˺�
    char	csEmailPass[DMS_PASSWD_LEN];    //����

    char    csSmtpServer[DMS_MAX_DOMAIN_NAME]; //smtp������ //���ڷ����ʼ�
    char    csPop3Server[DMS_MAX_DOMAIN_NAME]; //pop3������ //���ڽ����ʼ�,��IMAP��������

    DMS_NET_EMAIL_ADDR stToAddrList[DMS_EMAIL_ADDR_MAXNUM]; /**< �ռ��˵�ַ�б�  */
    DMS_NET_EMAIL_ADDR stCcAddrList[DMS_EMAIL_ADDR_MAXNUM]; /**< ���͵�ַ�б� */
    DMS_NET_EMAIL_ADDR stBccAddrList[DMS_EMAIL_ADDR_MAXNUM];/**< ���͵�ַ�б� */
    DMS_NET_EMAIL_ADDR stSendAddrList;                          /**< �����˵�ַ */

	BYTE    byEmailType[1];					//����Ϊ�����ʼ����� 8 - Motion 4 - Video Loss  2 - IOAlarm  1- Other
	char    csReserved[31];
}DMS_NET_EMAIL_PARAM, *PDMS_NET_EMAIL_PARAM;


/*
¼�����߼�

1. ¼��ģʽ��Ϊ: �ƻ�¼��/����¼��/�ֹ�¼��/��¼��
	��DMS_NET_CHANNEL_RECORD �ṹ���е�byRecordMode ���塣
	����ֵ���óɲ�¼��ʱ����ͨ�����������κ�¼��ָ������������ֹ�¼��
2. ��ʱ¼���߼�
	�ƻ�¼��: ����stRecordSched ʱ���¼��ƻ�����¼��
	����1: ¼��ģʽbyRecordMode �Ƕ�ʱ¼��ģʽ��¼��ʱ�����жϵ�ǰʱ����¼������(byRecordType) �ֶ�ֵ��λ��0x000001 Ϊ��
3. �ƶ����¼���߼�
	����1: ��Ƶͨ�������ƶ���������������ͨ��¼��
	����2: a. ¼��ģʽbyRecordMode �Ƕ�ʱ¼��ģʽ��¼��ʱ�����жϵ�ǰʱ����¼������byRecordType �ֶ�ֵ��λ��0x000002 Ϊ��
			����b. ¼��ģʽbyRecordMode �ǿ���¼��ģʽ
4. ̽ͷ����¼���߼�
	����1: ̽ͷ������������������������Ƶͨ��¼��
	����2: a.¼��ģʽbyRecordMode �Ƕ�ʱ¼��ģʽ��¼��ʱ�����жϵ�ǰʱ����¼������byRecordType �ֶ�ֵ��λ��0x000004 Ϊ��
				����b. ¼��ģʽbyRecordMode �ǿ���¼��ģʽ
*/

typedef struct
{
    DWORD			dwSize;    				/* �˽ṹ�Ĵ�С */
    DWORD			dwChannel;
    DWORD			dwReserve;  			/* 2014-12-23 yxhu: ֱ����byRecordMode�� ���ֶ�ֹͣʹ�ã������ֶ�*/
    DMS_RECORDSCHED stRecordSched;			/* ¼��ʱ��Σ�����һ��������*/
    DWORD			dwPreRecordTime;		/* Ԥ¼ʱ�䣬��λ��s��0��ʾ��Ԥ¼*/
    DWORD			dwRecorderDuration; 	/* ¼�񱣴���ʱ��, 2014-12-25 yxhu: IPC �汾��֧��*/
    BYTE				byRedundancyRec;		/* �Ƿ�����¼��,��Ҫ����˫���ݣ�0/1, Ĭ��Ϊ������2014-12-25 yxhu: Ŀǰ�汾��֧��*/
    BYTE				byAudioRec;				/* ¼��ʱ����������ʱ�Ƿ��¼��Ƶ���ݣ������д˷���,Ŀǰ�汾��֧��*/
    BYTE				byRecordMode;			//0:�Զ�ģʽ������ʱ¼�����ã�1:����¼�� 2:�ֶ�¼��ģʽ3:��ֹ���з�ʽ����¼��
    BYTE  				byStreamType;			// ¼���������� 0:������ 1:������
    BYTE  				byReserve[8];
}DMS_NET_CHANNEL_RECORD, *LPDMS_NET_CHANNEL_RECORD;

typedef struct tagDMS_NET_RECORD_STREAMMODE
{
    DWORD         dwSize;
    DWORD         dwStreamType;//0:first stream,1:second stream,2,third stream
    unsigned char byRes[16];
}DMS_NET_RECORD_STREAMMODE;

// SNMP
typedef struct tagDMS_SNMP_CFG{
	DWORD	dwSize;
	BYTE	byEnable;				//�Ƿ���Ч
	BYTE	nSnmpVersion;			//v1=0 v2=1 v3=2
	int  nSnmpPort;					//1-65535
	char szROCommunity[32];
	char szRWCommunity[32];
	char szTrapAddr[16];
	int  nTrapPort;					//1-65535
	char szUserName[32];			//v3 using
	char szPassword[16];
}DMS_SNMP_CFG,*PDMS_SNMP_CFG;


typedef struct
{
    DWORD			dwSize;				//���ṹ��С
    char			csNTPServer[64];	//NTP��������������IP��ַ
    WORD			wInterval;			//Уʱ���ʱ�䣨��СʱΪ��λ��
    BYTE			byEnableNTP;		//NTPУʱ�Ƿ����ã�0����1����
    signed char		cTimeDifferenceH;	//����ʱ�׼ʱ���ʱ�Сʱ����-12 ... +13
    signed char		cTimeDifferenceM;	//����ʱ�׼ʱ���ʱ����ӣ���0, 30, 45
    BYTE			res1;
    WORD			wNtpPort;			//NTP�������˿ڣ��豸Ĭ��Ϊ123
    BYTE			res2[8];
}DMS_NET_NTP_CFG,*LDMS_NET_NTP_CFG;

typedef struct
{
    DWORD				dwSize;			//���ṹ��С
    int					nTimeZone;		//ʱ��
    BYTE                byDstMode;		//����ʱģʽ��0-Ĭ�ϣ�1-�Զ���
    BYTE                byStartDst;     //�Ƿ��Ѿ���ʼִ��Dst;
    BYTE				byRes1[10];		//����
    DWORD				dwEnableDST;	//�Ƿ�������ʱ�� 0�������� 1������
    BYTE				byDSTBias;		//����ʱƫ��ֵ��30min, 60min, 90min, 120min, �Է��Ӽƣ�����ԭʼ��ֵ
    BYTE				byRes2[3];		//����
    DMS_NET_TIMEPOINT	stBeginPoint;	//��ʱ�ƿ�ʼʱ��
    DMS_NET_TIMEPOINT	stEndPoint;		//��ʱ��ֹͣʱ��
}DMS_NET_ZONEANDDST, *LPDMS_NET_ZONEANDDST;

/* ��ʱץͼ */
typedef struct tagDMS_NET_SNAP_TIMER
{
    DWORD       	dwSize;
    BOOL                bEnable;
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
    DWORD          	dwInterval;      //��λ������
    int                 	nPictureQuality;//��������(0-4),0Ϊ���
    int                 	nImageSize;     // �����С����Чֵ������������֧�ֵķֱ���
    BYTE			byStoragerMode; //�洢ģʽ(0:����, 1:FTP, 2: FTP|LOCAL(����FTP,FTPʧ�ܺ�¼�񱾵�))
    BYTE			byRes1;
    WORD			wChannel;	//0813
    BYTE        		byRes[4];
}DMS_NET_SNAP_TIMER, *LPDMS_NET_SNAP_TIMER;

typedef struct tagDMS_NET_SNAP_EVENT
{
    DWORD       	dwSize;
    BOOL                bEnable;
    DWORD          	dwInterval;           //ץ�ļ������λ������
    int                 	nPictureQuality;   //��������(0-4),0Ϊ���
    int                 	nImageSize;         // �����С����Чֵ������������֧�ֵķֱ���
    BYTE                 byPicNum;           //ץ������
    BYTE        		byRes[3];
}DMS_NET_SNAP_EVENT, *LPDMS_NET_SNAP_EVENT;

/*232���ڲ���*/
typedef struct
{
    DWORD dwSize; /* �˽ṹ�Ĵ�С */
    DWORD dwBaudRate; /* ������(bps) */
    BYTE byDataBit; /* �����м�λ 0��5λ��1��6λ��2��7λ��3��8λ; */
    BYTE byStopBit; /* ֹͣλ 0��1λ��1��2λ; */
    BYTE byParity; /* У�� 0����У�飬1����У�飬2��żУ��; */
    BYTE byFlowcontrol; /* 0���ޣ�1��������,2-Ӳ���� */
    DWORD dwWorkMode; /* ����ģʽ��0��խ�����䣨232��������PPP���ţ���1������̨��232�������ڲ������ƣ���2��͸��ͨ�� */
}DMS_NET_RS232CFG, *LPDMS_NET_RS232CFG;


typedef struct
{
    DWORD   dwSize;
    int             bEnable;              /*0:��Ч�û�������, 1:����*/
    DWORD   dwIndex;          /*�û����*/

    char        csUserName[DMS_NAME_LEN]; /* �û������32�ֽ�*/
    char        csPassword[DMS_PASSWD_LEN]; /* ���� */

    DWORD   dwUserRight; /* Ȩ�� */

    BYTE  byLocalPreviewRight[DMS_MAX_CHANNUM]; //���ؿ���Ԥ����ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byNetPreviewRight[DMS_MAX_CHANNUM]; //Զ�̿���Ԥ����ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byLocalPlaybackRight[DMS_MAX_CHANNUM]; //���ؿ��Իطŵ�ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byNetPlaybackRight[DMS_MAX_CHANNUM]; //Զ�̿��Իطŵ�ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byLocalRecordRight[DMS_MAX_CHANNUM]; //���ؿ���¼���ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byNetRecordRight[DMS_MAX_CHANNUM];//Զ�̿���¼���ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byLocalPTZRight[DMS_MAX_CHANNUM];//���ؿ���PTZ��ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byNetPTZRight[DMS_MAX_CHANNUM];//Զ�̿���PTZ��ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byLocalBackupRight[DMS_MAX_CHANNUM];//���ر���Ȩ��ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    BYTE  byNetBackupRight[DMS_MAX_CHANNUM];//Զ�̱���Ȩ��ͨ�� 1-��Ȩ�ޣ�0-��Ȩ��

    DMS_NET_IPADDR   stIPAddr;
    BYTE    byMACAddr[DMS_MACADDR_LEN]; /* �����ַ */
    BYTE    byPriority;                                 /* ���ȼ���0xff-�ޣ�0--�ͣ�1--�У�2--�ߣ�3--��� */
                                                                    /*
                                                                    �ޡ�����ʾ��֧�����ȼ�������
                                                                    �͡���Ĭ��Ȩ��:�������غ�Զ�̻ط�,���غ�Զ�̲鿴��־��״̬,���غ�Զ�̹ػ�/����
                                                                    �С����������غ�Զ�̿�����̨,���غ�Զ���ֶ�¼��,���غ�Զ�̻ط�,�����Խ���Զ��Ԥ��
                                                                                    ���ر���,����/Զ�̹ػ�/����
                                                                    �ߡ�������ִ�г���Ϊ Administrators �鱣����������������κβ���ϵͳ����
                                                                    ��ߡ�������Ա
                                                                    */

    BYTE    byRes[17];
}DMS_NET_USER_INFO,*LPDMS_NET_USER_INFO;

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

//�źŶ�ʧ����
typedef struct{
    DWORD    dwSize;
    DWORD   dwChannel;
    BOOL     bEnable; /* �Ƿ����źŶ�ʧ���� */
    DMS_HANDLE_EXCEPTION   stHandle; /* ����ʽ */
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
}DMS_NET_CHANNEL_VILOST,*LPDMS_NET_CHANNEL_VILOST;

//�ڵ����������С704*576
typedef struct
{
    DWORD dwSize;
    DWORD   dwChannel;
    BOOL bEnable; /* �Ƿ������ڵ����� ,0-��,1-�������� 2-�������� 3-��������*/
    WORD wHideAlarmAreaTopLeftX; /* �ڵ������x���� */
    WORD wHideAlarmAreaTopLeftY; /* �ڵ������y���� */
    WORD wHideAlarmAreaWidth; /* �ڵ�����Ŀ� */
    WORD wHideAlarmAreaHeight; /*�ڵ�����ĸ�*/
    DMS_HANDLE_EXCEPTION stHandle; /* ����ʽ */
    DMS_SCHEDTIME stScheduleTime[7][4];/**��ͨ����videoloss�Ĳ���ʱ��*/
}DMS_NET_CHANNEL_HIDEALARM,*LPDMS_NET_CHANNEL_HIDEALARM;

typedef struct tagDMS_NET_WIFI_CONFIG
{
    DWORD           dwSize;
    BYTE            bWifiEnable;
    BYTE			byWifiMode;         // 0:station, 1:ap
    BYTE			byWpsStatus;        // wps״̬(0:�ر�,1:����)
	BYTE			byConnectStatus;    // 0: ether connect,  1: wifi connect.
    DMS_NET_IPADDR  dwNetIpAddr;
    DMS_NET_IPADDR  dwNetMask;
    DMS_NET_IPADDR  dwGateway;
    DMS_NET_IPADDR  dwDNSServer;
    char            csEssid[32];
    BYTE            nSecurity;
						// 0: none  (wifi enable)
						// 1: wep 64 assii (wifi enable)
						// 2: wep 64 hex (wifi enable)
						// 3: wep 128 assii (wifi enable)
						// 4: wep 128 hex (wifi enable)
						// 5: WPAPSK-TKIP
						// 6: WPAPSK-AES
						// 7: WPA2PSK-TKIP
						// 8: WPA2PSK-AES
    BYTE        	byNetworkType;  // 1. Infra 2. ad-hoc
    BYTE            byEnableDHCP;
    BYTE            byStatus; //    0:  �ɹ�,����ֵ�Ǵ�����
    char            csWebKey[32];
}DMS_NET_WIFI_CONFIG, *LPDMS_NET_WIFI_CONFIG;

typedef struct tagDMS_NET_DDNSCFG
{
    DWORD   dwSize;
    BOOL    bEnableDDNS;                //�Ƿ�����DDNS
    struct {
        DMS_DDNS_TYPE    enDDNSType;                  //DDNS����������, �����������ͣ�DMS_DDNS_TYPE
        char    csDDNSUsername[DMS_NAME_LEN];
        char    csDDNSPassword[DMS_PASSWD_LEN];
        char    csDDNSDomain[DMS_MAX_DOMAIN_NAME];          //��DDNS������ע���������ַ
        char    csDDNSAddress[DMS_MAX_DOMAIN_NAME];         //DNS��������ַ��������IP��ַ������
        int nDDNSPort;                  //DNS�������˿ڣ�Ĭ��Ϊ6500
        int nUpdateTime;              /*����ʱ��*/
        int     nStatus;                        /*״̬,0:�ɹ�,����ֵ��������*/

        BYTE byReserve[32];               /**< ������,�����۽��Ԥ��32�ֽ� */
    }stDDNS[DMS_MAX_DDNS_NUMS];
}DMS_NET_DDNSCFG, *PDMS_NET_DDNSCFG;

typedef struct tagDMS_NET_SNAP_DATA_V2
{
    DWORD   dwSize;
    int     nChannel;
    int     nLevel;       // 0-5 :0 ���
    DWORD   dwFileType;   // 0:jpgͼƬ��1:bmpͼƬ
    DWORD   dwBufSize;    // ���ݳ���
    char   *pDataBuffer;  //����
    int     nWidth;
    int     nHeight;
}DMS_NET_SNAP_DATA_V2;

///////////////////////////////////////////////////////////
///////////////////// ���Ӳ���2 ///////////////////////////
typedef enum __RESTORE_MASK_E
{
    NORMAL_CFG = 0x00000001,       //�豸��������
    VENCODER_CFG = 0x00000002,     //��Ƶ�������
    RECORD_CFG = 0x00000004,           //¼�����
    RS232_CFG = 0x00000008,            //RS232����
    NETWORK_CFG = 0x00000010,          //�������
    ALARM_CFG = 0x00000020,          //��������/�������
    VALARM_CFG = 0x00000040,           //��Ƶ����������
    PTZ_CFG = 0x00000080,              //��̨����
    PREVIEW_CFG = 0x00000100,          //�����������
    USER_CFG = 0x00000200,            //�û�����
    TVMODE_CFG = 0x00000400,		  //��ʽ
    COLOR_CFG = 0x00000800, 		  //������ɫ
} RESTORE_MASK_E;

typedef enum emGKSysFlag
{
    DMS_SYS_FLAG_ENCODE_D1 =         0x00000001,
    DMS_SYS_FLAG_HD_RECORD =         0x00000002,
    #ifdef MODULE_SUPPORT_RF433
	DMS_SYS_FLAG_RF433_ALARM =		0x00000003,
	#endif
    DMS_SYS_FLAG_CI_IDENTIFY =        0x00000004,
    DMS_SYS_FLAG_MD_RECORD	=		0x00000008,	//����¼��
    DMS_SYS_FLAG_PPPOE_DDNS =        0x00000010,
    DMS_SYS_FLAG_DECODE_H264 =       0x00000020,
    DMS_SYS_FLAG_ITEV =			0x00000040,  //�������ܷ���
    DMS_SYS_FLAG_VIDEO_ALARM_MODE2 =  0x00010000,
    DMS_SYS_FLAG_FTPUP =              0x00020000,  // FTPUP
    DMS_SYS_FLAG_EMAIL =              0x00040000,	// EMAIL
    DMS_SYS_FLAG_WIFI =               0x00080000, /* WLAN */
    DMS_SYS_FLAG_ACTIVE_CONNECTION =  0x00100000,  //֧����������
    DMS_SYS_FLAG_TELALARM =           0x00200000,  //֧�ֵ绰����
    DMS_SYS_FLAG_TEMPHUM =            0x00400000,  //֧����ʪ�ȣ���Դ
    DMS_SYS_FLAG_UPNP =               0x00800000,	// upnp
    DMS_SYS_FLAG_PLATFORM =           0x01000000,	/* ֧��ƽ̨����*/
    DMS_SYS_FLAG_3G =				  0x02000000, /* 3G */
    DMS_SYS_FLAG_PTZ =				  0x04000000,	//PTZ
    DMS_SYS_FLAG_DEVICEMAINTAIN =     0x08000000,  //�豸ά��ʹ��
    DMS_SYS_FLAG_NETWORK =			  0x20000000,	//��������
    DMS_SYS_FLAG_SNMP =   			  0x40000000,	//SNMP
    DMS_SYS_FLAG_EXCEPTALARM =		  0x80000000,	//�쳣��������ʹ��
} eSysFlag;

typedef enum __DMS_SERVER_TYPE_E
{
    //���汾
    NVS_FLAG=100001	,  // ���汾DVS
    NVD_FLAG=100002, //  ���汾NVD
    NVR_FLAG=100003, //  ���汾NVR
    IPC_FLAG=100004, //  ���汾IPCAM
    DVR_D3_0001_FLAG=0x10D30001, // D3�����豸
    DVR_D2_0001_FLAG=0x10D30002, // D3�����豸
    DMS_HVR_FLAG=0x10D30003,//HVR
    //IPC
    IPC_6C_D	=0xA1010002,
    IPC_6C_P	=0xA1010003,
    IPC_6D_M	=0xA1020004,
    IPC_6E_M	=0xA1030004,
    IPC_6E_3M	=0xA1030005,
    IPC_6F_P	=0xB1000003,
    IPC_6F_10P	=0xA1080003,	//3518 720P
    IPC_6F_13P	=0xA1080006,	//3518 960P
    IPC_6F_20P	=0xA1080004,	//3518 1080P
    IPC_6F_30P	=0xA1080005,	//3518 300M
    IPC_6H_P	=0xC1080003,

    //DVS
    DVS_S6C00D=0xA2010002,
    DVS_S6D00D=0xA2020002,
    DVS_S6D00C=0xA2020001,
    DVS_S6D00D_S=0xA2020002,
    //NVD
    NVD_S7D00D=0xA5050002,
    //DVR
    DVR_DA3700FC=0xA3060001,
    DVR_DA3700FD=0xA3070002,
}DMS_SERVER_TYPE_E;

typedef enum {
    DMS_NTSC = 0,
    DMS_PAL = 1,
} DMS_FORMAT;

typedef enum {
    DMS_VIDEO_FORMAT_CIF = 0,
    DMS_VIDEO_FORMAT_D1 = 1,
    DMS_VIDEO_FORMAT_HD1 = 2,
    DMS_VIDEO_FORMAT_QCIF = 3,
    DMS_VIDEO_FORMAT_VGA = 4,
    DMS_VIDEO_FORMAT_QVGA = 5,
    DMS_VIDEO_FORMAT_720P = 6,
    DMS_VIDEO_FORMAT_130H = 7,
    DMS_VIDEO_FORMAT_300H = 8,
    DMS_VIDEO_FORMAT_QQVGA = 9,
    DMS_VIDEO_FORMAT_UXGA = 10,
    DMS_VIDEO_FORMAT_SVGA = 11,
    DMS_VIDEO_FORMAT_1080P = 12,
    DMS_VIDEO_FORMAT_XGA = 13,
    DMS_VIDEO_FORMAT_960P = 14,
    DMS_VIDEO_FORMAT_640 = 15,
    DMS_VIDEO_FORMAT_5M = 17,
    DMS_VIDEO_FORMAT_4M = 18,
    DMS_VIDEO_FORMAT_2048x1520 = 19,
} DMS_VIDEO_FORMAT_TYPE;

//Set Flag
typedef enum emColorSetFlag
{
	DMS_COLOR_SET_BRIGHTNESS =			0x00000001,//����
	DMS_COLOR_SET_HUE =					0x00000002,//ɫ��
	DMS_COLOR_SET_SATURATION =			0x00000004,//���Ͷ�
	DMS_COLOR_SET_CONTRAST =			0x00000008,//�Աȶ�
	DMS_COLOR_SET_IRISBASIC =			0x00000010,//��Ȧ��׼ֵ
	DMS_COLOR_SET_RED =					0x00000020,//
	DMS_COLOR_SET_BLUE =				0x00000040,//
	DMS_COLOR_SET_GREEN =				0x00000080,//
	DMS_COLOR_SET_GAMMA =				0x00000100,//
	DMS_COLOR_SET_DEFINITION =			0x00000200,//�����ȡ����
	DMS_COLOR_SET_SCENE =				0x00000400, //��������
	DMS_COLOR_SET_ADNSWITCH =			0x00000800, //��������
	DMS_COLOR_SET_DN = 					0x00001000, //���뷧ֵ,����������Ч
	DMS_COLOR_SET_AWBSWITCH = 			0x00002000, //�Զ���ƽ���л�
	DMS_COLOR_SET_AECSWITCH = 			0x00004000, //�Զ��ع��л�
	DMS_COLOR_SET_EC = 					0x00008000, //�عֵⷧ
	DMS_COLOR_SET_AGCSWITCH = 			0x00010000, //�Զ������л�
	DMS_COLOR_SET_GC = 					0x00020000, //���淧ֵ
	DMS_COLOR_SET_MIRROR = 				0x00040000, //����
	DMS_COLOR_SET_BAW = 				0x00080000, //��ɫת�ڰ�
	DMS_COLOR_SET_EWD = 				0x00100000, //�򿪿�̬
	DMS_COLOR_SET_WD = 					0x00200000, //��̬����
	DMS_COLOR_SET_AIRIS = 				0x00400000, //�Զ���Ȧʹ��
	DMS_COLOR_SET_BLC = 				0x00800000, // ���ⲹ��
} eColorSetFlag;

typedef enum emRecordType
{
    GK_NET_RECORD_TYPE_SCHED   = 0x000001,   //��ʱ¼��
    GK_NET_RECORD_TYPE_MOTION  = 0x000002,  //�Ƶ����¼��
    GK_NET_RECORD_TYPE_ALARM   = 0x000004,  //����¼��
    GK_NET_RECORD_TYPE_CMD     = 0x000008, //����¼��
    GK_NET_RECORD_TYPE_MANU    = 0x000010,  //�ֹ�¼��
    GK_NET_RECORD_TYPE_SCHED_2 = 0x000011 //�ֹ�¼��
} eRecordType;

#define REC_ACTION_STOP   0
#define REC_ACTION_START  1
//¼�����
typedef struct tagDMS_NET_REC_CONTROL
{
	DWORD	dwSize;
	int    	nChannel;
	int    	nRecordType; //¼�����ͣ��μ�DMS_NET_RECORD_TYPE_
	WORD    	wAction;     //������0 ֹͣ¼��1 ��ʼ¼��
	WORD     wDuration;  //0xFFFFFFFF��ʾ��ֹͣ��ֱ�������ٴ�ֹͣ���
}DMS_NET_REC_CONTROL;

typedef enum tagWIRELESS_STATUS{
    WL_OK = 0,
    WL_NOT_CONNECT,     //û��������
    WL_DEVICE_NOT_EXIST,    //ipc��wifiӲ��������
    WL_ESSID_NOT_EXIST, //essid������
    WL_DHCP_ERROR,          //dhcp��ȡ����ip
    WL_ENCRYPT_FAIL,        //������֤����
    WL_IP_CONFLICT,         //IP��ַ��ͻ
	WL_UNSUPPORT_WPS,
}WIRELESS_STATUS_E;

typedef enum
{
    PT_PCMU          = 0,
    PT_1016          = 1,
    PT_G721          = 2,
    PT_GSM           = 3,
    PT_G723          = 4,
    PT_DVI4_8K       = 5,
    PT_DVI4_16K      = 6,
    PT_LPC           = 7,
    PT_PCMA          = 8,
    PT_G722          = 9,
    PT_S16BE_STEREO  = 10,
    PT_S16BE_MONO    = 11,
    PT_QCELP         = 12,
    PT_CN            = 13,
    PT_MPEGAUDIO     = 14,
    PT_G728          = 15,
    PT_DVI4_3        = 16,
    PT_DVI4_4        = 17,
    PT_G729          = 18,
    PT_G711A         = 19,
    PT_G711U         = 20,
    PT_G726          = 21,
    PT_G729A         = 22,
    PT_LPCM          = 23,
    PT_CelB          = 25,
    PT_JPEG          = 26,
    PT_CUSM          = 27,
    PT_NV            = 28,
    PT_PICW          = 29,
    PT_CPV           = 30,
    PT_H261          = 31,
    PT_MPEGVIDEO     = 32,
    PT_MPEG2TS       = 33,
    PT_H263          = 34,
    PT_SPEG          = 35,
    PT_MPEG2VIDEO    = 36,
    PT_AAC           = 37,
    PT_WMA9STD       = 38,
    PT_HEAAC         = 39,
    PT_PCM_VOICE     = 40,
    PT_PCM_AUDIO     = 41,
    PT_AACLC         = 42,
    PT_MP3           = 43,
    PT_ADPCMA        = 49,
    PT_AEC           = 50,
    PT_X_LD          = 95,
    PT_H264          = 96,
    PT_D_GSM_HR      = 200,
    PT_D_GSM_EFR     = 201,
    PT_D_L8          = 202,
    PT_D_RED         = 203,
    PT_D_VDVI        = 204,
    PT_D_BT656       = 220,
    PT_D_H263_1998   = 221,
    PT_D_MP1S        = 222,
    PT_D_MP2P        = 223,
    PT_D_BMPEG       = 224,
    PT_MP4VIDEO      = 230,
    PT_MP4AUDIO      = 237,
    PT_VC1           = 238,
    PT_JVC_ASF       = 255,
    PT_D_AVI         = 256,
    PT_DIVX3		= 257,
    PT_AVS		= 258,
    PT_REAL8		= 259,
    PT_REAL9		= 260,
    PT_VP6		= 261,
    PT_VP6F		= 262,
    PT_VP6A		= 263,
    PT_SORENSON	 =264,
    PT_MAX           = 265,
    /* add by hisilicon */
    PT_AMR           = 1001,
    PT_MJPEG         = 1002,
	PT_H264_HIGHPROFILE = 1003,
	PT_H264_MAINPROFILE = 1004,
	PT_H264_BASELINE = 1005,
	PT_H265 = 1006,
}PAYLOAD_TYPE_E;

typedef struct tagPAYLOAD_NAME{
	int		nPayloadID;
	char	*lpName;
}PAYLOAD_NAME;

typedef struct tagDMS_VIDEO_FORMAT{
	int nVideoFormat;
	int nWidth;
	int nHeight;
	int nPAL;
	char *lpName;
}DMS_VIDEO_FORMAT;

/*      WIFI   */
typedef struct tagDMS_NET_WIFI_SITE
{
    DWORD           dwSize;
    int             nType;
    char            csEssid[DMS_NAME_LEN];
    int             nRSSI;    //wifi �ź�ǿ��
    BYTE            byMac[DMS_MACADDR_LEN];
    BYTE            byRes1[2];
    BYTE            bySecurity;
								//0: none  (wifi enable)
								//1: wep 64 assii (wifi enable)
								//2: wep 64 hex (wifi enable)
								//3: wep 128 assii (wifi enable)
								//4: wep 128 hex (wifi enable)
								//5: WPAPSK-TKIP
								//6: WPAPSK-AES
								//7: WPA2PSK-TKIP
								//8: WPA2PSK-AES
    BYTE            byRes2[3];
    int             nChannel;
}DMS_NET_WIFI_SITE;

typedef struct tagDMS_NET_WIFI_SITE_LIST
{
    DWORD           dwSize; //== sizeof(DMS_NET_WIFI_SITE)*nCount
    int             nCount;
    DMS_NET_WIFI_SITE    stWifiSite[100];// ʵ�ʴ���
}DMS_NET_WIFI_SITE_LIST;

/*Ӳ����Ϣ*/
/*����Ӳ����Ϣ*/
typedef struct
{
	DWORD  dwSize;
	DWORD  dwHDNo;		//Ӳ�̺�, ȡֵ0��DMS_MAX_DISKNUM-1
	DWORD  dwHDType;    //Ӳ������(��������) 0:SD��,1:U��,2:Ӳ��
    DWORD  dwCapacity;	//Ӳ������(��������)
    DWORD  dwFreeSpace; //Ӳ��ʣ��ռ�(��������)
    DWORD  dwHdStatus;	//Ӳ��״̬(��������)�� 0��������1��δ��ʽ����2������3��S.M.A.R.T״̬��4����ƥ�䣻5������
    BYTE  byHDAttr;		//0��Ĭ�ϣ�1�����ࣻ2��ֻ��
    BYTE  byRecStatus;  //�Ƿ�����¼��--0:����,1:����¼��
    BYTE  byRes1[2];	//��������
    DWORD  dwHdGroup;	//�����ĸ����飬ȡֵ 0��DMS_MAX_HD_GROUP-1
    BYTE  byRes2[120];	//����
}DMS_NET_SINGLE_HD, *LPDMS_NET_SINGLE_HD;
/*����Ӳ����Ϣ����*/
typedef struct
{

	DWORD  dwSize;
    DWORD  dwHDCount; //Ӳ����
	DMS_NET_SINGLE_HD  stHDInfo[DMS_MAX_DISKNUM];
}DMS_NET_HDCFG, *LPDMS_NET_HDCFG;
/*��ʽ����������*/
typedef struct
{
	DWORD  dwSize;
    DWORD  dwHDNo; //Ӳ�̺�
	BYTE  byRes2[32];//����
}DMS_NET_DISK_FORMAT, *LPDMS_NET_DISK_FORMAT;
//��ʽ��Ӳ��״̬�Լ�����
typedef struct
{
	DWORD  dwSize;
    DWORD  dwHDNo; //Ӳ�̺�
    DWORD  dwHdStatus;//Ӳ��״̬�� 0����ʽ����ʼ��1�����ڸ�ʽ�����̣�2����ʽ�����
    DWORD  dwProcess;//��ʽ������ 0-100
    DWORD  dwResult;//��ʽ����� 0���ɹ���1��ʧ��
	BYTE  byRes[16];//����
}DMS_NET_DISK_FORMAT_STATUS, *LPDMS_NET_DISK_FORMAT_STATUS;


/*����������Ϣ ��һ�汾��֧��*/
typedef struct
{

	DWORD dwHDGroupNo;			//�����(��������) 0��DMS_MAX_HD_GROUP-1
    BYTE  byHDGroupChans[64];  //�����Ӧ��¼��ͨ��, 0����ʾ��ͨ����¼�󵽸����飻1����ʾ¼�󵽸�����
    BYTE  byRes[8];
}DMS_NET_SINGLE_HDGROUP, *LPDMS_NET_SINGLE_HDGROUP;

/*����������Ϣ���� ��һ�汾��֧��*/
typedef struct
{
    DWORD  dwSize;
    DWORD  dwHDGroupCount;
    DMS_NET_SINGLE_HDGROUP  stHDGroupAttr[DMS_MAX_HD_GROUP];
}DMS_NET_HDGROUP_CFG, *LPDMS_NET_HDGROUP_CFG;


typedef struct tagGK_NET_CMS_FRAME_HEADER
{
    DWORD  dwSize;
    DWORD   dwFrameIndex;
    DWORD   dwVideoSize;
    DWORD   dwTimeTick;
    WORD    wAudioSize;
    BYTE    byFrameType; //0��I֡��1��P֡��2��B֡��ֻ�����Ƶ
    BYTE    byVideoCode;
    BYTE    byAudioCode;
    BYTE    byReserved1;// ��4λ���� ;0: ������  1�� ������
    BYTE    byReserved2; // ��4λ����  1: I 2:p
    BYTE    byReserved3; // ��4λ����
    WORD    wVideoWidth;
    WORD    wVideoHeight;
} GK_NET_CMS_FRAME_HEADER;

typedef enum
{
	GK_CMS_PT_PCMU = 0,
	GK_CMS_PT_1016 = 1,
	GK_CMS_PT_G721 = 2,
	GK_CMS_PT_GSM = 3,
	GK_CMS_PT_G723 = 4,
	GK_CMS_PT_DVI4_8K = 5,
	GK_CMS_PT_DVI4_16K = 6,
	GK_CMS_PT_LPC = 7,
	GK_CMS_PT_PCMA = 8,
	GK_CMS_PT_G722 = 9,
	GK_CMS_PT_S16BE_STEREO,
	GK_CMS_PT_S16BE_MONO = 11,
	GK_CMS_PT_QCELP = 12,
	GK_CMS_PT_CN = 13,
	GK_CMS_PT_MPEGAUDIO = 14,
	GK_CMS_PT_G728 = 15,
	GK_CMS_PT_DVI4_3 = 16,
	GK_CMS_PT_DVI4_4 = 17,
	GK_CMS_PT_G729 = 18,
	GK_CMS_PT_G711A = 19,
	GK_CMS_PT_G711U = 20,
	GK_CMS_PT_G726 = 21,
	GK_CMS_PT_G729A = 22,
	GK_CMS_PT_LPCM = 23,
	GK_CMS_PT_CelB = 25,
	GK_CMS_PT_JPEG = 26,
	GK_CMS_PT_CUSM = 27,
	GK_CMS_PT_NV = 28,
	GK_CMS_PT_PICW = 29,
	GK_CMS_PT_CPV = 30,
	GK_CMS_PT_H261 = 31,
	GK_CMS_PT_MPEGVIDEO = 32,
	GK_CMS_PT_MPEG2TS = 33,
	GK_CMS_PT_H263 = 34,
	GK_CMS_PT_SPEG = 35,
	GK_CMS_PT_MPEG2VIDEO = 36,
	GK_CMS_PT_AAC = 37,
	GK_CMS_PT_WMA9STD = 38,
	GK_CMS_PT_HEAAC = 39,
	GK_CMS_PT_PCM_VOICE = 40,
	GK_CMS_PT_PCM_AUDIO = 41,
	GK_CMS_PT_AACLC = 42,
	GK_CMS_PT_MP3 = 43,
	GK_CMS_PT_ADPCMA = 49,
	GK_CMS_PT_AEC = 50,
	GK_CMS_PT_X_LD = 95,
	GK_CMS_PT_H264 = 96,
	GK_CMS_PT_D_GSM_HR = 200,
	GK_CMS_PT_D_GSM_EFR = 201,
	GK_CMS_PT_D_L8 = 202,
	GK_CMS_PT_D_RED = 203,
	GK_CMS_PT_D_VDVI = 204,
	GK_CMS_PT_D_BT656 = 220,
	GK_CMS_PT_D_H263_1998 = 221,
	GK_CMS_PT_D_MP1S = 222,
	GK_CMS_PT_D_MP2P = 223,
	GK_CMS_PT_D_BMPEG = 224,
	GK_CMS_PT_MP4VIDEO = 230,
	GK_CMS_PT_MP4AUDIO = 237,
	GK_CMS_PT_VC1 = 238,
	GK_CMS_PT_JVC_ASF = 255,
	GK_CMS_PT_D_AVI = 256,
	GK_CMS_PT_MAX = 257,

	GK_CMS_PT_AMR = 1001, /* add by mpp */
	GK_CMS_PT_MJPEG = 1002,
}GK_CMS_PT_TYPE_E;

typedef struct tagDMS_NET_USER
{
	DWORD dwSize;
	DMS_NET_USER_INFO stUser[DMS_MAX_USERNUM];
} DMS_NET_USER,*LPDMS_NET_USER;

typedef struct tagJBNV_TIME{
	DWORD dwYear;
	DWORD dwMonth;
	DWORD dwDay;
	DWORD dwHour;
	DWORD dwMinute;
	DWORD dwSecond;
}JBNV_TIME;

typedef struct tagJBNV_FindFileReq{		//��ѯ����
	DWORD		dwSize;
	DWORD		nChannel;				//0xff��ȫ��ͨ����0��1��2 ......
	DWORD		nFileType;				//�ļ����� ��0xff �� ȫ����0 �� ��ʱ¼��1 - �ƶ���⣬2 �� ����������3  �� �ֶ�¼��
	JBNV_TIME	BeginTime;				//
	JBNV_TIME	EndTime;				//StartTime StopTime ��ֵ��Ϊ0000-00-00 00:00:00��ʾ��������ʱ��
}JBNV_FIND_FILE_REQ;

typedef struct tagJBNV_FILE_DATA_INFO{
	char		sFileName[256];			//�ļ���
	JBNV_TIME	BeginTime;				//
	JBNV_TIME	EndTime;
	DWORD		nChannel;
	DWORD		nFileSize;				//�ļ��Ĵ�С
	DWORD		nState;					//�ļ�ת��״̬
}JBNV_FILE_DATA_INFO;

typedef struct tagJBNV_FindFileResp{
	DWORD		dwSize;
	DWORD		nResult;		//0:success ;1:find file error ; 2:the number of file more than the memory size, and the result contains part of the data
	DWORD		nCount;
	DWORD		dwReserved[3];
}JBNV_FIND_FILE_RESP;

/*��̨������485���ڲ���*/
typedef struct
{
    DWORD dwSize; /* �˽ṹ�Ĵ�С */
    DWORD   dwChannel;
    DWORD dwBaudRate; /* ʵ�ʲ�����(bps)*/
    BYTE byDataBit; /* �����м�λ 0��5λ��1��6λ��2��7λ��3��8λ */
    BYTE byStopBit; /* ֹͣλ 0��1λ��1��2λ  */
    BYTE byParity; /* У�� 0����У�飬1����У�飬2��żУ��; */
    BYTE byFlowcontrol; /* 0���ޣ�1��������,2-Ӳ���� */
    char  csDecoderType[DMS_NAME_LEN]; /* ����������, ���±�*/
    WORD wDecoderAddress; /*��������ַ:0 - 255*/
    BYTE byHSpeed;      //��̨H�ٶ�
    BYTE byVSpeed;      //��̨V�ٶ�
    BYTE byWatchPos;  //����λ
    BYTE byRes;
    WORD dwRes;
 //   DWORD dwRes;
}DMS_NET_DECODERCFG,*LPDMS_NET_DECODERCFG;

//��̨Э���ṹ����
typedef struct
{
    DWORD dwType;               /*����������ֵ����1��ʼ��������*/
    char  csDescribe[DMS_NAME_LEN]; /*��������������*/
}DMS_PTZ_PROTOCOL;

typedef struct
{
    DWORD   dwSize;
    DMS_PTZ_PROTOCOL stPtz[DMS_PTZ_PROTOCOL_NUM];/*���200��PTZЭ��*/
    DWORD   dwPtzNum;           /*��Ч��ptzЭ����Ŀ*/
    BYTE    byRes[8];           // ��������
}DMS_NET_PTZ_PROTOCOLCFG, *LDMS_NET_PTZ_PROTOCOLCFG;


/*NAS*/
typedef struct
{
    char   csServerIP[DMS_MAX_IP_LENGTH];
    char   byPath[64];  //�������ļ�·��
    BYTE   byEnable;	//0:�ر�,1:����
    BYTE   byRes2[79];   //����
}DMS_NAS_SERVER_INFO;

typedef struct
{
    DWORD  dwSize;
    DMS_NAS_SERVER_INFO stServerInfo[8];
}DMS_NET_NAS_CFG;


typedef struct _OPEN_FILE
{
	DWORD		nFileType; // 1 open by filename ,2 open by time
	char		csFileName[256];
	DWORD		nChannel;
	DWORD		nProtocolType;
	DWORD		nStreamType;

	JBNV_TIME	struStartTime;
	JBNV_TIME	struStopTime;
}OPEN_FILE,*POPEN_FILE;

typedef struct _OPEN_FILE_EX
{
	DWORD	nFlag2;
	DWORD	nID;
	DWORD		nFileType; // 1 open by filename ,2 open by time
	char		csFileName[256];
	DWORD		nChannel;
	DWORD		nProtocolType;
	DWORD		nStreamType;
	JBNV_TIME	struStartTime;
	JBNV_TIME	struStopTime;
}OPEN_FILE_EX,*POPEN_FILE_EX;

typedef struct tagFilePlay_INFO
{
    DWORD   dwSize;
    DWORD   dwStream1Height;	//��Ƶ��(1)
    DWORD   dwStream1Width;		//��Ƶ��
	DWORD   dwStream1CodecID;	//��Ƶ�������ͺţ�MPEG4Ϊ0��JPEG2000Ϊ1,H264Ϊ2��
    DWORD   dwAudioChannels;	//��Ƶͨ����
    DWORD   dwAudioBits;		//��Ƶ������
    DWORD   dwAudioSamples;		//��Ƶ������
    DWORD   dwWaveFormatTag;	//��Ƶ�������ͺ�
	char	csChannelName[32];	//ͨ������
	DWORD	dwFileSize;
	DWORD	dwTotalTime;
	DWORD	dwPlayStamp;		//ʱ���
}FilePlay_INFO,*PFilePlay_INFO;

typedef struct tagNET_AUDIO_INFO
{
	ULONG	nFlag;
	ULONG	nChannel;
	ULONG	nBitRate;
	ULONG	nSmapleRate;
}NET_AUDIO_INFO,*PNET_AUDIO_INFO;

typedef enum
{
	DMS_NET_PLAYSTART = 0,
	DMS_NET_PLAYPAUSE,
	DMS_NET_PLAYRESTART,
	DMS_NET_FASTFORWARD2X,
	DMS_NET_FASTFORWARD4X,
	DMS_NET_FASTFORWARD8X,
	DMS_NET_FASTFORWARD16X,
	DMS_NET_FASTFORWARD32X,
	DMS_NET_SLOWPLAY2X,
	DMS_NET_SLOWPLAY4X,
	DMS_NET_SLOWPLAY8X,
	DMS_NET_SLOWPLAY16X,
	DMS_NET_SLOWPLAY32X,
	DMS_NET_PLAYNORMAL,
	DMS_NET_PLAYFRAME,
	DMS_NET_PLAYSETPOS,
	DMS_NET_PLAYGETPOS,
	DMS_NET_PLAYGETTIME,
	DMS_NET_PLAYGETFRAME,
	DMS_NET_GETTOTALFRAMES,
	DMS_NET_GETTOTALTIME,
	DMS_NET_THROWBFRAME,
} DMS_PLAY_BACK_CONTROL_E;


typedef enum
{
    JB_PLAY_CMD_PLAY = 1,
    JB_PLAY_CMD_PLAYPREV = 2,
    JB_PLAY_CMD_STEPIN = 3,
    JB_PLAY_CMD_STEPOUT = 4,
    JB_PLAY_CMD_PAUSE = 5,
    JB_PLAY_CMD_RESUME = 6,
    JB_PLAY_CMD_FASTPLAY = 7,
    JB_PLAY_CMD_FASTBACK = 8,
    JB_PLAY_CMD_STOP = 9,
    JB_PLAY_CMD_SLOWPLAY = 10,
    JB_PLAY_CMD_FIRST = 11,   /*��λ���ļ�ͷ*/
    JB_PLAY_CMD_LAST = 12,    /*��λ���ļ�ĩβ*/
    JB_PLAY_CMD_SEEK = 13,    /*��λ����*/
    JB_PLAY_CMD_SOUND = 14,
}  JB_PLAY_CMD_E;

typedef struct tagVODActionReq{
	DWORD		dwSize;
	DWORD		dwAction; //1.2......
	DWORD		dwData;
}VODACTION_REQ ,*pVODACTION_REQ;

typedef enum
{
    JB_TEST_RESULT_DNSUP = 0,   // ��֧��
    JB_TEST_RESULT_DNT,         // ��δ����
    JB_TEST_RESULT_SUCCESS,     // ���Գɹ�
    JB_TEST_RESULT_FAILED,      // ����ʧ��
    JB_TEST_RESULT_BUTT,
}  JB_TEST_RESULT_E;

typedef struct tagNET_TEST_RESULT_INFO
{
    JB_TEST_RESULT_E emSpeaker;
    JB_TEST_RESULT_E emMic;
    JB_TEST_RESULT_E emLed;
    JB_TEST_RESULT_E emSd;
    JB_TEST_RESULT_E emIRCut;
    JB_TEST_RESULT_E emWifi;
    JB_TEST_RESULT_E emPTZ;
    JB_TEST_RESULT_E emRestore;
    JB_TEST_RESULT_E emView;
    JB_TEST_RESULT_E emWhiteLed;
    JB_TEST_RESULT_E emResetBtn;
    JB_TEST_RESULT_E reserved1;
    JB_TEST_RESULT_E reserved2;
}NET_TEST_RESULT_INFO,*PNET_TEST_RESULT_INFO;

typedef struct tagNET_AF_OFFSET_INFO
{
	DWORD		dwValue;
	DWORD		dwReserved[3];
}NET_AF_OFFSET_INFO;

typedef struct tagNET_TEST_CMD_INFO
{
	Char cmd[128];
}NET_TEST_CMD_INFO;

#define		TMNV4NET_FLAG			904
#define		TMNVHISINET_FLAG		931
#define		DMS_DVR_HI_FLAG			932
#define		DMS_DVR_HI2_FLAG		933
#define		DMS_G711A_160			934


#define ALARM_IN_NUM 0
#define ALARM_OUT_NUM 0
#define CMS_FUNC_MASK1 (DMS_SYS_FLAG_ENCODE_D1 | DMS_SYS_FLAG_HD_RECORD \
                       | DMS_SYS_FLAG_DECODE_H264 | DMS_SYS_FLAG_NETWORK \
                       | DMS_SYS_FLAG_PTZ | DMS_SYS_FLAG_DEVICEMAINTAIN)

#define CMS_FUNC_MASK2 (DMS_SYS_FLAG_ENCODE_D1 | DMS_SYS_FLAG_HD_RECORD \
                       | DMS_SYS_FLAG_DECODE_H264 | DMS_SYS_FLAG_FTPUP \
                       | DMS_SYS_FLAG_EMAIL | DMS_SYS_FLAG_WIFI \
                       | DMS_SYS_FLAG_DEVICEMAINTAIN \
                       | DMS_SYS_FLAG_NETWORK)


void PrintDmsTime(DMS_SYSTEMTIME *ptime);
int Gk_LogOn(int sock);
int GkDealCmdDispatch(int sock);
int GkHandleMediaRequest(int sock);
int GkCmsCmdResq(int sock, void *body, int body_size, int cmd);
int GkCmsCmdResqByIndex(int sock, void *body, int body_size, int cmd, int stream_no);
int Gk_CmsNotify(int sock, unsigned long type, void *data, int data_size);
int cms_notify_to_all_client(unsigned long type);

#ifdef __cplusplus
}
#endif
#endif


