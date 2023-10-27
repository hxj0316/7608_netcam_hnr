/*
 * AVIOCTRLDEFs.h
 *	Define AVIOCTRL Message Type and Context
 *  Created on: 2011-08-12
 *  Author: TUTK
 *
 */

//Change Log:
//  2013-12-25 - 1> Add set and get dropbox feature of device.
//                      Add IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_REQ
//                          Client request device to return dropbox status
//                      Add IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_RESP
//                          Device return to client the dropbox status
//                      Add IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_REQ
//                          Client request device to set dropbox info
//                      Add IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_RESP
//                          Device acknowledge the set dropbox info
//
//  2013-06-25 - 1> Add set and get time zone of device.
//                      Add IOTYPE_USER_IPCAM_GET_TIMEZONE_REQ
//                          Client request device to return time zone
//	                    Add IOTYPE_USER_IPCAM_GET_TIMEZONE_RESP
//	                        Device return to client the time zone
//	                    Add IOTYPE_USER_IPCAM_SET_TIMEZONE_REQ
//	                        Client request device to set time zone
//	                    Add IOTYPE_USER_IPCAM_SET_TIMEZONE_RESP
//	                        Device acknowledge the set time zone request
//
//  2013-06-05 - 1> Add customer defined message type, start from #FF000000
//                  Naming role of message type : IOTYPE_[Company_name]_[function_name]
//                      ex : IOTYPE_TUTK_TEST_REQ, IOTYPE_TUTK_TEST_RESP
//                  Naming role of data structure : [Company_name]_[data_structure_name]
//                      ex : TUTK_SMsgTestReq, TUTK_SMsgTestResp
//
//
//  2013-03-10 - 1> Add flow information collection mechanism.
//						Add IOTYPE_USER_IPCAM_GET_FLOWINFO_REQ
//							Device request client to collect flow information.
//						Add IOTYPE_USER_IPCAM_GET_FLOWINFO_RESP
//							Client acknowledge device that request is received.
//						Add IOTYPE_USER_IPCAM_CURRENT_FLOWINFO
//							Client send collected flow information to device.
//				 2> Add IOTYPE_USER_IPCAM_RECEIVE_FIRST_IFRAME command.
//
//	2013-02-19 - 1> Add more detail of status of SWifiAp
//				 2> Add more detail description of STimeDay
//
//	2012-10-26 - 1> SMsgAVIoctrlGetEventConfig
//						Add field: externIoOutIndex, externIoInIndex
//				 2> SMsgAVIoctrlSetEventConfig, SMsgAVIoctrlGetEventCfgResp
//						Add field: externIoOutStatus, externIoInStatus
//
//	2012-10-19 - 1> SMsgAVIoctrlGetWifiResp: -->SMsgAVIoctrlGetWifiResp2
//						Add status description
//				 2> SWifiAp:
//				 		Add status 4: selected but not connected
//				 3> WI-FI Password 32bit Change to 64bit
//				 4> ENUM_AP_ENCTYPE: Add following encryption types
//				 		AVIOTC_WIFIAPENC_WPA_PSK_TKIP		= 0x07,
//						AVIOTC_WIFIAPENC_WPA_PSK_AES		= 0x08,
//						AVIOTC_WIFIAPENC_WPA2_PSK_TKIP		= 0x09,
//						AVIOTC_WIFIAPENC_WPA2_PSK_AES		= 0x0A,
//
//				 5> IOTYPE_USER_IPCAM_SETWIFI_REQ_2:
//						Add struct SMsgAVIoctrlSetWifiReq2
//				 6> IOTYPE_USER_IPCAM_GETWIFI_RESP_2:
//						Add struct SMsgAVIoctrlGetWifiResp2

//  2012-07-18 - added: IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_REQ, IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_RESP
//	2012-05-29 - IOTYPE_USER_IPCAM_DEVINFO_RESP: Modify firmware version
//	2012-05-24 - SAvEvent: Add result type
//  2014-07-07 - Change Dropbox struct SMsgAVIoctrlSetDropbox from 32 bytes to 128byes
//

#ifndef _AVIOCTRL_DEFINE_H_
#define _AVIOCTRL_DEFINE_H_

#define USE_HTS_PROTOCOL   0
#define USE_APP_XUHUI


/////////////////////////////////////////////////////////////////////////////////
/////////////////// Message Type Define//////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// AVIOCTRL Message Type
typedef enum
{
	IOTYPE_USER_IPCAM_START 					= 0x01FF,
	
	IOTYPE_USER_IPCAM_SEND_META_START           = 0x200,
    IOTYPE_USER_IPCAM_SEND_META_STOP            = 0x201,
	
    IOTYPE_USER_IPCAM_PUSH_STREAMING_START      = 0x210,
	IOTYPE_USER_IPCAM_PUSH_STREAMING_START_RESP = 0x211,
    IOTYPE_USER_IPCAM_PUSH_STREAMING_FINISH     = 0x212,
	IOTYPE_USER_IPCAM_PUSH_STREAMING_FINISH_RESP= 0x213,
	IOTYPE_USER_IPCAM_PUSH_STREAMING_CLOSE      = 0x214,
	
	IOTYPE_USER_IPCAM_STOP	 					= 0x02FF,
	IOTYPE_USER_IPCAM_AUDIOSTART 				= 0x0300,
	IOTYPE_USER_IPCAM_AUDIOSTOP 				= 0x0301,

	IOTYPE_USER_IPCAM_SPEAKERSTART 				= 0x0350,
	IOTYPE_USER_IPCAM_SPEAKERSTOP 				= 0x0351,

	IOTYPE_USER_IPCAM_SETSTREAMCTRL_REQ			= 0x0320,
	IOTYPE_USER_IPCAM_SETSTREAMCTRL_RESP		= 0x0321,
	IOTYPE_USER_IPCAM_GETSTREAMCTRL_REQ			= 0x0322,
	IOTYPE_USER_IPCAM_GETSTREAMCTRL_RESP		= 0x0323,

	IOTYPE_USER_IPCAM_SETMOTIONDETECT_REQ		= 0x0324,
	IOTYPE_USER_IPCAM_SETMOTIONDETECT_RESP		= 0x0325,
	IOTYPE_USER_IPCAM_GETMOTIONDETECT_REQ		= 0x0326,
	IOTYPE_USER_IPCAM_GETMOTIONDETECT_RESP		= 0x0327,

	IOTYPE_USER_IPCAM_GETSUPPORTSTREAM_REQ		= 0x0328,	// Get Support Stream
	IOTYPE_USER_IPCAM_GETSUPPORTSTREAM_RESP		= 0x0329,

	IOTYPE_USER_IPCAM_DEVINFO_REQ				= 0x0330,
	IOTYPE_USER_IPCAM_DEVINFO_RESP				= 0x0331,

	IOTYPE_USER_IPCAM_SETPASSWORD_REQ			= 0x0332,
	IOTYPE_USER_IPCAM_SETPASSWORD_RESP			= 0x0333,

	IOTYPE_USER_IPCAM_LISTWIFIAP_REQ			= 0x0340,
	IOTYPE_USER_IPCAM_LISTWIFIAP_RESP			= 0x0341,
	IOTYPE_USER_IPCAM_SETWIFI_REQ				= 0x0342,
	IOTYPE_USER_IPCAM_SETWIFI_RESP				= 0x0343,
	IOTYPE_USER_IPCAM_GETWIFI_REQ				= 0x0344,
	IOTYPE_USER_IPCAM_GETWIFI_RESP				= 0x0345,
	IOTYPE_USER_IPCAM_SETWIFI_REQ_2				= 0x0346,
	IOTYPE_USER_IPCAM_GETWIFI_RESP_2			= 0x0347,

	IOTYPE_USER_IPCAM_SETRECORD_REQ				= 0x0310,
	IOTYPE_USER_IPCAM_SETRECORD_RESP			= 0x0311,
	IOTYPE_USER_IPCAM_GETRECORD_REQ				= 0x0312,
	IOTYPE_USER_IPCAM_GETRECORD_RESP			= 0x0313,

	IOTYPE_USER_IPCAM_SETRCD_DURATION_REQ		= 0x0314,
	IOTYPE_USER_IPCAM_SETRCD_DURATION_RESP  	= 0x0315,
	IOTYPE_USER_IPCAM_GETRCD_DURATION_REQ		= 0x0316,
	IOTYPE_USER_IPCAM_GETRCD_DURATION_RESP  	= 0x0317,

	IOTYPE_USER_IPCAM_LISTEVENT_REQ				= 0x0318,
	IOTYPE_USER_IPCAM_LISTEVENT_RESP			= 0x0319,

	IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL 		= 0x031A,
	IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP 	= 0x031B,

	IOTYPE_USER_IPCAM_SEACH_RECORD_IN_MONTH_REQ	= 0x031C,
	IOTYPE_USER_IPCAM_SEACH_RECORD_IN_MONTH_RESP= 0x031D,

	IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_REQ		= 0x032A,
	IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_RESP	= 0x032B,

	IOTYPE_USER_IPCAM_GET_EVENTCONFIG_REQ		= 0x0400,	// Get Event Config Msg Request
	IOTYPE_USER_IPCAM_GET_EVENTCONFIG_RESP		= 0x0401,	// Get Event Config Msg Response
	IOTYPE_USER_IPCAM_SET_EVENTCONFIG_REQ		= 0x0402,	// Set Event Config Msg req
	IOTYPE_USER_IPCAM_SET_EVENTCONFIG_RESP		= 0x0403,	// Set Event Config Msg resp

	IOTYPE_USER_IPCAM_SET_ENVIRONMENT_REQ		= 0x0360,
	IOTYPE_USER_IPCAM_SET_ENVIRONMENT_RESP		= 0x0361,
	IOTYPE_USER_IPCAM_GET_ENVIRONMENT_REQ		= 0x0362,
	IOTYPE_USER_IPCAM_GET_ENVIRONMENT_RESP		= 0x0363,

	IOTYPE_USER_IPCAM_SET_VIDEOMODE_REQ			= 0x0370,	// Set Video Flip Mode
	IOTYPE_USER_IPCAM_SET_VIDEOMODE_RESP		= 0x0371,
	IOTYPE_USER_IPCAM_GET_VIDEOMODE_REQ			= 0x0372,	// Get Video Flip Mode
	IOTYPE_USER_IPCAM_GET_VIDEOMODE_RESP		= 0x0373,

	IOTYPE_USER_IPCAM_FORMATEXTSTORAGE_REQ		= 0x0380,	// Format external storage
	IOTYPE_USER_IPCAM_FORMATEXTSTORAGE_RESP		= 0x0381,

	IOTYPE_USER_IPCAM_PTZ_COMMAND				= 0x1001,	// P2P PTZ Command Msg

	IOTYPE_USER_IPCAM_EVENT_REPORT				= 0x1FFF,	// Device Event Report Msg
	IOTYPE_USER_IPCAM_RECEIVE_FIRST_IFRAME		= 0x1002,	// Send from client, used to talk to device that
															// client had received the first I frame

	IOTYPE_USER_IPCAM_GET_FLOWINFO_REQ			= 0x0390,
	IOTYPE_USER_IPCAM_GET_FLOWINFO_RESP			= 0x0391,
	IOTYPE_USER_IPCAM_CURRENT_FLOWINFO			= 0x0392,

	IOTYPE_USER_IPCAM_GET_TIMEZONE_REQ          = 0x3A0,
	IOTYPE_USER_IPCAM_GET_TIMEZONE_RESP         = 0x3A1,
	IOTYPE_USER_IPCAM_SET_TIMEZONE_REQ          = 0x3B0,
	IOTYPE_USER_IPCAM_SET_TIMEZONE_RESP         = 0x3B1,


    // dropbox support
    IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_REQ      = 0x500,
    IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_RESP     = 0x501,
    IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_REQ      = 0x502,
    IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_RESP     = 0x503,

    	//ptz preset
	IOTYPE_HICHIP_CRUISE_START                  = 0x600,
	IOTYPE_HICHIP_CRUISE_STOP                   = 0x601,

	IOTYPE_BRIGHT_GETBRIGHT_REQ					= 0x602,
	IOTYPE_BRIGHT_GETBRIGHT_RESP 				= 0x603,

	IOTYPE_BRIGHT_SETBRIGHT_REQ					= 0x604,
	IOTYPE_BRIGHT_SETBRIGHT_RESP 				= 0x605,
	IOTYPE_CONTRAST_GETCONTRAST_REQ 			= 0x606,
	IOTYPE_CONTRAST_GETCONTRAST_RESP 			= 0x607,
	IOTYPE_CONTRAST_SETCONTRAST_REQ 			= 0x608,
	IOTYPE_CONTRAST_SETCONTRAST_RESP 			= 0x609,
	IOTYPE_USER_IPCAM_GET_BATTERY_REQ 			= 0x60A,
	IOTYPE_USER_IPCAM_GET_BATTERY_RESP	 		= 0x60B,

    //ADD 20160615
	IOTYPE_USER_IPCAM_SETPRESET_REQ             = 0x440,
	IOTYPE_USER_IPCAM_SETPRESET_RESP            = 0x441,
	IOTYPE_USER_IPCAM_GETPRESET_REQ             = 0x442,
	IOTYPE_USER_IPCAM_GETPRESET_RESP            = 0x443,

	IOTYPE_USER_IPCAM_DOOROPEN_REQ              = 0x800,	// ����
	IOTYPE_USER_IPCAM_DOOROPEN_RESP             = 0x801,	// ����Ӧ��
    IOTYPE_XM_SETNIGHTMODE_REQ                  = 0x722,
    IOTYPE_XM_SETNIGHTMODE_RESP                 = 0x723,

	IOTYPE_XM_GETNIGHTMODE_REQ 					= 0x724,
	IOTYPE_XM_GETNIGHTMODE_RESP 				= 0x725,

	// for IP02/IPC product
	IOTYPE_USER_IPCAM_SET_PUSH_REQ              = 0x802,	// ��Ϣ����ע��
	IOTYPE_USER_IPCAM_SET_PUSH_RESP             = 0x803,	// ��Ϣ����ע��Ӧ��

	IOTYPE_USER_IPCAM_DEL_PUSH_REQ              = 0x804,	// ��Ϣ����ע��
	IOTYPE_USER_IPCAM_DEL_PUSH_RESP             = 0x805,	// ��Ϣ����ע��Ӧ��

	IOTYPE_USER_IPCAM_SET_MDP_REQ				= 0x806,	// �����ƶ�������������Ϣ
	IOTYPE_USER_IPCAM_SET_MDP_RESP				= 0x807,	// �����ƶ�������Ӧ����Ϣ
	IOTYPE_USER_IPCAM_GET_MDP_REQ				= 0x808,	// ��ȡ�ƶ�������������Ϣ
	IOTYPE_USER_IPCAM_GET_MDP_RESP				= 0x809,	// ��ȡ�ƶ�������Ӧ����Ϣ

	IOTYPE_USER_IPCAM_DOORPASS_REQ              = 0x812,	// ������������
	IOTYPE_USER_IPCAM_DOORPASS_RESP             = 0x813,	// ������������Ӧ��

	IOTYPE_USER_IPCAM_SET_DEVICETIME_REQ        = 0x816,
	IOTYPE_USER_IPCAM_SET_DEVICETIME_RESP       = 0x817,
	
    IOTYPE_USER_IPCAM_GET_DEVICETIME_REQ        = 0x818,    //��ȡ�����ı���ʱ��
    IOTYPE_USER_IPCAM_GET_DEVICETIME_RESP       = 0x819,

	IOTYPE_USER_IPCAM_ALARMING_REQ				= 0x850,	// ��Ϣ֪ͨ����
	IOTYPE_USER_IPCAM_ALARMING_RESP				= 0x851,	// ��Ϣ֪ͨӦ��

    IOTYPE_USER_IPCAM_SET_SYSTEM_REQ			= 0x852,	// ����ϵͳ
    IOTYPE_USER_IPCAM_SET_SYSTEM_RESP			= 0x853,

    IOTYPE_USER_IPCAM_GET_SYSTEM_REQ			= 0x854,	// ��ȡϵͳ����
	IOTYPE_USER_IPCAM_GET_SYSTEM_RESP			= 0x855,

    IOTYPE_USER_IPCAM_GET_SDCARD_REQ            = 0x856,    // ��ȡSD��״̬ 
    IOTYPE_USER_IPCAM_GET_SDCARD_RESP           = 0x857,    // ��ȡSD��״̬Ӧ��

    IOTYPE_USER_IPCAM_GET_OSD_REQ               = 0x858,    // ��ȡ OSD ����
    IOTYPE_USER_IPCAM_GET_OSD_RESP              = 0x859,    // ��ȡ OSD ����Ӧ��
    IOTYPE_USER_IPCAM_SET_OSD_REQ               = 0x860,    // ���� OSD ����
    IOTYPE_USER_IPCAM_SET_OSD_RESP              = 0x861,    // ���� OSD ����Ӧ��
	IOTYPE_USER_IPCAM_SET_433_REQ				= 0x862,	// ���� 433 �豸
	IOTYPE_USER_IPCAM_SET_433_RESP				= 0x863,	// ���� 433 �豸Ӧ��
	IOTYPE_USER_IPCAM_GET_433_REQ				= 0x864,	// ��ȡ 433 �豸�б�
	IOTYPE_USER_IPCAM_GET_433_RESP				= 0x865,	// ��ȡ 433 �豸�б�Ӧ��
	IOTYPE_USER_IPCAM_CFG_433_REQ				= 0x866,	// ��ʼ 433 ���
	IOTYPE_USER_IPCAM_CFG_433_RESP				= 0x867,	// ��ʼ 433 ���Ӧ��
	IOTYPE_USER_IPCAM_DEL_433_REQ				= 0x868,	// ɾ�� 433 �豸
	IOTYPE_USER_IPCAM_DEL_433_RESP				= 0x869,	// ɾ�� 433 �豸Ӧ��
	IOTYPE_USER_IPCAM_CFG_433_EXIT_REQ			= 0x870,	// �˳� 433 �豸���
	IOTYPE_USER_IPCAM_CFG_433_EXIT_RESP			= 0x871,	// �˳� 433 �豸���Ӧ��

    // NEW�豸����
    IOTYPE_USER_IPCAM_GET_CAPACITY_REQ          = 0x880,    //��ȡ�����Ĳ���,�糧�̺ţ���Ƶ��ʽ����Ƶ��ʽ��
    IOTYPE_USER_IPCAM_GET_CAPACITY_RESP         = 0x881,    //��ȡ�����Ĳ�����Ӧ��
    // �ƶ���ⱨ��
    IOTYPE_USER_IPCAM_GET_MD_ALAM_REQ           = 0x882,
    IOTYPE_USER_IPCAM_GET_MD_ALAM_RESP          = 0x883,
    IOTYPE_USER_IPCAM_SET_MD_ALAM_REQ           = 0x884,
    IOTYPE_USER_IPCAM_SET_MD_ALAM_RESP          = 0x885,
    // ��Ƶ�������
    IOTYPE_USER_IPCAM_GET_AUDIO_REQ             = 0x886,
    IOTYPE_USER_IPCAM_GET_AUDIO_RESP            = 0x887,
    IOTYPE_USER_IPCAM_SET_AUDIO_REQ             = 0x888,
    IOTYPE_USER_IPCAM_SET_AUDIO_RESP            = 0x889,
    // �̼�����
    IOTYPE_USER_IPCAM_UPDATE_REQ                = 0x88a,
    IOTYPE_USER_IPCAM_UPDATE_RESP               = 0x88b,
    IOTYPE_USER_IPCAM_UPDATE_PROG_REQ           = 0x88c,
    IOTYPE_USER_IPCAM_UPDATE_PROG_RESP          = 0x88d,
    // Զ��ִ��shell    
    IOTYPE_USER_IPCAM_SHELL_REQ                 = 0x88e,    
    IOTYPE_USER_IPCAM_SHELL_RESP                = 0x88f,
    IOTYPE_USER_IPCAM_HEARTBEAT_REQ             = 0x890,
    IOTYPE_USER_IPCAM_HEARTBEAT_RESP            = 0x891,

    IOTYPE_USER_IPCAM_GET_AUDIO_VOLUME_REQ      = 0x8a0,
    IOTYPE_USER_IPCAM_GET_AUDIO_VOLUME_RESP     = 0x8a1,
    IOTYPE_USER_IPCAM_SET_AUDIO_VOLUME_REQ      = 0x8a2,
    IOTYPE_USER_IPCAM_SET_AUDIO_VOLUME_RESP     = 0x8a3,

	IOTYPE_USER_IPCAM_GET_CLOUDRECORD_REQ 		= 0X0732,
	IOTYPE_USER_IPCAM_GET_CLOUDRECORD_RESP 		= 0X0733,
	IOTYPE_USER_IPCAM_SET_CLOUDRECORD_REQ 		= 0X0734,
	IOTYPE_USER_IPCAM_SET_CLOUDRECORD_RESP 		= 0X0735,

	IOTYPE_USER_IPCAM_SET_LIVEVIEWTIMESTAMP_REQ = 0X802,
	IOTYPE_USER_IPCAM_SET_LIVEVIEWTIMESTAMP_RESP = 0X803,
	IOTYPE_USER_IPCAM_GET_LIVEVIEWTIMESTAMP_REQ  = 0X804,
	IOTYPE_USER_IPCAM_GET_LIVEVIEWTIMESTAMP_RESP  = 0X805,

	IOTYPE_USER_IPCAM_GET_WAKEUP_FUN_REQ		= 0X1802,
	IOTYPE_USER_IPCAM_GET_WAKEUP_FUN_RESP		= 0X1803,
	IOTYPE_USER_IPCAM_SET_WAKEUP_FUN_REQ		= 0X1804,
	IOTYPE_USER_IPCAM_SET_WAKEUP_FUN_RESP		= 0X1805,
	// ��ѯ¼������
	IOTYPE_USER_IPCAM_LISTEVENT_DATE_REQ		= 0x0892,
	IOTYPE_USER_IPCAM_LISTEVENT_DATE_RESP		= 0x0893,

	// �����豸����Ϣ
	IOTYPE_USER_IPCAM_UPDATE_CLOUD_REQ			= 0x08B4,
	IOTYPE_USER_IPCAM_UPDATE_CLOUD_REQP			= 0x08B5,

    IOTYPE_USER_CMD_MAX
}ENUM_AVIOCTRL_MSGTYPE;
typedef enum
{
    DEVICE_TYPE_38              =0,//��Ƭ��         ��Ӧ�ַ���     :  MODEL_38
    DEVICE_TYPE_YTJ_NORMAL      =1,//��ͨҡͷ��     ��Ӧ�ַ���     :  YTJ_NORMAL
    DEVICE_TYPE_PANORAMA        =2,//ȫ��           ��Ӧ�ַ���     :  IPC_PANORAMA_V1
    DEVICE_TYPE_YTJ_EGG         =3,//����ҡͷ��     ��Ӧ�ַ���     :  YTJ_EGG

	DEVICE_TYPE_MAX
}ENUM_DEVICE_TYPE;
typedef enum
{
    ODM_DS              =0,//��Ӧ�ַ���     :  ODM_DS
    ODM_TCL             =1,//��Ӧ�ַ���     :  ODM_TCL
    ODM_KONKA           =2,//��Ӧ�ַ���     :  ODM_KONKA

	ODM_MAX
}ENUM_ODM;

typedef enum
{

    P2P_TYPE_NONE              =0,//��Ӧ�ַ���      :  NONE

    P2P_TYPE_MTWS              =1,//��Ӧ�ַ���      :  P2P_MTWS
    P2P_TYPE_TUTK              =2,//��Ӧ�ַ���      :  P2P_TUTK

	P2P_TYPE_MAX
}ENUM_P2P_TYPE;

typedef enum
{
    LANG_TYPE_AUTO              =0,//��Ӧ�ַ���   :  LANG_ATUO
    LANG_TYPE_SIMPLE_CH         =1,//��Ӧ�ַ���:SIMPLE_CH
    LANG_TYPE_COMPLEX_CH        =2,//��Ӧ�ַ���:COMPLEX_CH
    LANG_TYPE_EN                =3,//��Ӧ�ַ���:EN

	LANG_TYPE_MAX
}ENUM_LANG_TYPE;
typedef enum
{
    PANORAMA_TOP              =0,//��Ӧ�ַ���:PANORAMA_TOP
    PANORAMA_WALL             =1,//��Ӧ�ַ���:PANORAMA_WALL

	PANORAMA_MAX
}ENUM_PANORAMA_TYPE;


/////////////////////////////////////////////////////////////////////////////////
/////////////////// Type ENUM Define ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	AVIOCTRL_OK					= 0x00,
	AVIOCTRL_ERR				= -0x01,
	AVIOCTRL_ERR_PASSWORD		= AVIOCTRL_ERR - 0x01,
	AVIOCTRL_ERR_STREAMCTRL		= AVIOCTRL_ERR - 0x02,
	AVIOCTRL_ERR_MONTIONDETECT	= AVIOCTRL_ERR - 0x03,
	AVIOCTRL_ERR_DEVICEINFO		= AVIOCTRL_ERR - 0x04,
	AVIOCTRL_ERR_LOGIN			= AVIOCTRL_ERR - 5,
	AVIOCTRL_ERR_LISTWIFIAP		= AVIOCTRL_ERR - 6,
	AVIOCTRL_ERR_SETWIFI		= AVIOCTRL_ERR - 7,
	AVIOCTRL_ERR_GETWIFI		= AVIOCTRL_ERR - 8,
	AVIOCTRL_ERR_SETRECORD		= AVIOCTRL_ERR - 9,
	AVIOCTRL_ERR_SETRCDDURA		= AVIOCTRL_ERR - 10,
	AVIOCTRL_ERR_LISTEVENT		= AVIOCTRL_ERR - 11,
	AVIOCTRL_ERR_PLAYBACK		= AVIOCTRL_ERR - 12,
    AVIOCTRL_ERR_USERNAME		= AVIOCTRL_ERR - 13,

	AVIOCTRL_ERR_INVALIDCHANNEL	= AVIOCTRL_ERR - 0x20,
}ENUM_AVIOCTRL_ERROR; //APP don't use it now


// ServType, unsigned long, 32 bits, is a bit mask for function declareation
// bit value "0" means function is valid or enabled
// in contract, bit value "1" means function is invalid or disabled.
// ** for more details, see "ServiceType Definitation for AVAPIs"
//
// Defined bits are listed below:
//----------------------------------------------
// bit		fuction
// 0		Audio in, from Device to Mobile
// 1		Audio out, from Mobile to Device
// 2		PT function
// 3		Event List function
// 4		Play back function (require Event List function)
// 5		Wi-Fi setting function
// 6		Event Setting Function
// 7		Recording Setting function
// 8		SDCard formattable function
// 9		Video flip function
// 10		Environment mode
// 11		Multi-stream selectable
// 12		Audio out encoding format

// The original enum below is obsoleted.
typedef enum
{
	SERVTYPE_IPCAM_DWH					= 0x00,
	SERVTYPE_RAS_DWF					= 0x01,
	SERVTYPE_IOTCAM_8125				= 0x10,
	SERVTYPE_IOTCAM_8125PT				= 0x11,
	SERVTYPE_IOTCAM_8126				= 0x12,
	SERVTYPE_IOTCAM_8126PT				= 0x13,
}ENUM_SERVICE_TYPE;

// AVIOCTRL Quality Type
typedef enum
{
	AVIOCTRL_QUALITY_UNKNOWN			= 0x00,
	AVIOCTRL_QUALITY_MAX				= 0x01,	// ex. 640*480, 15fps, 320kbps (or 1280x720, 5fps, 320kbps)
	AVIOCTRL_QUALITY_HIGH				= 0x02,	// ex. 640*480, 10fps, 256kbps
	AVIOCTRL_QUALITY_MIDDLE				= 0x03,	// ex. 320*240, 15fps, 256kbps
	AVIOCTRL_QUALITY_LOW				= 0x04, // ex. 320*240, 10fps, 128kbps
	AVIOCTRL_QUALITY_MIN				= 0x05,	// ex. 160*120, 10fps, 64kbps
}ENUM_QUALITY_LEVEL;


typedef enum
{
	AVIOTC_WIFIAPMODE_NULL				= 0x00,
	AVIOTC_WIFIAPMODE_MANAGED			= 0x01,
	AVIOTC_WIFIAPMODE_ADHOC				= 0x02,
}ENUM_AP_MODE;


typedef enum
{
	AVIOTC_WIFIAPENC_INVALID			= 0x00,
	AVIOTC_WIFIAPENC_NONE				= 0x01, //
	AVIOTC_WIFIAPENC_WEP				= 0x02, //WEP, for no password
	AVIOTC_WIFIAPENC_WPA_TKIP			= 0x03,
	AVIOTC_WIFIAPENC_WPA_AES			= 0x04,
	AVIOTC_WIFIAPENC_WPA2_TKIP			= 0x05,
	AVIOTC_WIFIAPENC_WPA2_AES			= 0x06,

	AVIOTC_WIFIAPENC_WPA_PSK_TKIP  = 0x07,
	AVIOTC_WIFIAPENC_WPA_PSK_AES   = 0x08,
	AVIOTC_WIFIAPENC_WPA2_PSK_TKIP = 0x09,
	AVIOTC_WIFIAPENC_WPA2_PSK_AES  = 0x0A,

}ENUM_AP_ENCTYPE;


// AVIOCTRL Event Type
typedef enum
{
	AVIOCTRL_EVENT_ALL					= 0x00,	// all event type(general APP-->IPCamera)
	AVIOCTRL_EVENT_MOTIONDECT			= 0x01,	// motion detect start//==s==
	AVIOCTRL_EVENT_VIDEOLOST			= 0x02,	// video lost alarm
	AVIOCTRL_EVENT_IOALARM				= 0x03, // io alarmin start //---s--

	AVIOCTRL_EVENT_MOTIONPASS			= 0x04, // motion detect end  //==e==
	AVIOCTRL_EVENT_VIDEORESUME			= 0x05,	// video resume
	AVIOCTRL_EVENT_IOALARMPASS			= 0x06, // IO alarmin end   //---e--
	AVIOCTRL_EVENT_MOVIE				= 0x07,
	AVIOCTRL_EVENT_TIME_LAPSE			= 0x08,
	AVIOCTRL_EVENT_EMERGENCY			= 0x09,
	AVIOCTRL_EVENT_EXPT_REBOOT			= 0x10, // system exception reboot
	AVIOCTRL_EVENT_SDFAULT				= 0x11, // sd record exception
	AVIOCTRL_EVENT_FULLTIME_RECORDING	= 0x12,
	AVIOCTRL_EVENT_PIR					= 0x13,
	AVIOCTRL_EVENT_RINGBELL				= 0x14,
	AVIOCTRL_EVENT_SOUND				= 0x15,
	AVIOCTRL_EVENT_RINGBELL_TUTK_PRIVATE		= 2000,

}ENUM_EVENTTYPE;

// AVIOCTRL Record Type
typedef enum
{
	AVIOTC_RECORDTYPE_OFF				= 0x00,
	AVIOTC_RECORDTYPE_FULLTIME			= 0x01,
	AVIOTC_RECORDTYPE_ALARM				= 0x02,
	AVIOTC_RECORDTYPE_MANUAL			= 0x03,
	AVIOTC_RECORDTYPE_SCHEDULE			= 0x04,
}ENUM_RECORD_TYPE;

// AVIOCTRL Play Record Command
typedef enum
{
	AVIOCTRL_RECORD_PLAY_PAUSE			= 0x00,
	AVIOCTRL_RECORD_PLAY_STOP			= 0x01,
	AVIOCTRL_RECORD_PLAY_STEPFORWARD	= 0x02, //now, APP no use
	AVIOCTRL_RECORD_PLAY_STEPBACKWARD	= 0x03, //now, APP no use
	AVIOCTRL_RECORD_PLAY_FORWARD		= 0x04, //now, APP no use
	AVIOCTRL_RECORD_PLAY_BACKWARD		= 0x05, //now, APP no use
	AVIOCTRL_RECORD_PLAY_SEEKTIME		= 0x06, //now, APP no use
	AVIOCTRL_RECORD_PLAY_END			= 0x07,
	AVIOCTRL_RECORD_PLAY_START			= 0x10,
}ENUM_PLAYCONTROL;

// AVIOCTRL Environment Mode
typedef enum
{
	AVIOCTRL_ENVIRONMENT_INDOOR_50HZ 	= 0x00,
	AVIOCTRL_ENVIRONMENT_INDOOR_60HZ	= 0x01,
	AVIOCTRL_ENVIRONMENT_OUTDOOR		= 0x02,//auto�Զ��жϰ����ҹ��
	AVIOCTRL_ENVIRONMENT_NIGHT			= 0x03,//ҹ��ģʽ���������
	AVIOCTRL_ENVIRONMENT_DAY		    = 0x04,//����ģʽ���غ����
	
}ENUM_ENVIRONMENT_MODE;

// AVIOCTRL Video Flip Mode
typedef enum
{
	AVIOCTRL_VIDEOMODE_NORMAL 			= 0x00,
	AVIOCTRL_VIDEOMODE_FLIP				= 0x01,
	AVIOCTRL_VIDEOMODE_MIRROR			= 0x02,
	AVIOCTRL_VIDEOMODE_FLIP_MIRROR 		= 0x03,
}ENUM_VIDEO_MODE;

// AVIOCTRL PTZ Command Value
typedef enum
{
	AVIOCTRL_PTZ_STOP					= 0,
	AVIOCTRL_PTZ_UP						= 1,
	AVIOCTRL_PTZ_DOWN					= 2,
	AVIOCTRL_PTZ_LEFT					= 3,
	AVIOCTRL_PTZ_LEFT_UP				= 4,
	AVIOCTRL_PTZ_LEFT_DOWN				= 5,
	AVIOCTRL_PTZ_RIGHT					= 6,
	AVIOCTRL_PTZ_RIGHT_UP				= 7,
	AVIOCTRL_PTZ_RIGHT_DOWN				= 8,
	AVIOCTRL_PTZ_AUTO					= 9,
	AVIOCTRL_PTZ_SET_POINT				= 10,
	AVIOCTRL_PTZ_CLEAR_POINT			= 11,
	AVIOCTRL_PTZ_GOTO_POINT				= 12,

	AVIOCTRL_PTZ_SET_MODE_START			= 13,
	AVIOCTRL_PTZ_SET_MODE_STOP			= 14,
	AVIOCTRL_PTZ_MODE_RUN				= 15,

	AVIOCTRL_PTZ_MENU_OPEN				= 16,
	AVIOCTRL_PTZ_MENU_EXIT				= 17,
	AVIOCTRL_PTZ_MENU_ENTER				= 18,

	AVIOCTRL_PTZ_FLIP					= 19,
	AVIOCTRL_PTZ_START					= 20,

	AVIOCTRL_LENS_APERTURE_OPEN			= 21,
	AVIOCTRL_LENS_APERTURE_CLOSE		= 22,

	AVIOCTRL_LENS_ZOOM_IN				= 23,
	AVIOCTRL_LENS_ZOOM_OUT				= 24,

	AVIOCTRL_LENS_FOCAL_NEAR			= 25,
	AVIOCTRL_LENS_FOCAL_FAR				= 26,

	AVIOCTRL_AUTO_PAN_SPEED				= 27,
	AVIOCTRL_AUTO_PAN_LIMIT				= 28,
	AVIOCTRL_AUTO_PAN_START				= 29,

	AVIOCTRL_PATTERN_START				= 30,
	AVIOCTRL_PATTERN_STOP				= 31,
	AVIOCTRL_PATTERN_RUN				= 32,

	AVIOCTRL_SET_AUX					= 33,
	AVIOCTRL_CLEAR_AUX					= 34,
	AVIOCTRL_MOTOR_RESET_POSITION		= 35,

	AVIOCTRL_PTZ_LEFT_RIGHT_AUTO		= 36,	// ����Ѳ��
	AVIOCTRL_PTZ_UP_DOWN_AUTO			= 37	// ����Ѳ��
}ENUM_PTZCMD;



/////////////////////////////////////////////////////////////////////////////
///////////////////////// Message Body Define ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////

/*
IOTYPE_USER_IPCAM_START 				= 0x01FF,
IOTYPE_USER_IPCAM_STOP	 				= 0x02FF,
IOTYPE_USER_IPCAM_AUDIOSTART 			= 0x0300,
IOTYPE_USER_IPCAM_AUDIOSTOP 			= 0x0301,
IOTYPE_USER_IPCAM_SPEAKERSTART 			= 0x0350,
IOTYPE_USER_IPCAM_SPEAKERSTOP 			= 0x0351,
IOTYPE_USER_IPCAM_PUSH_STREAMING_START  = 0x600,
IOTYPE_USER_IPCAM_PUSH_STREAMING_STOP   = 0x601,
** @struct SMsgAVIoctrlAVStream
*/
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned char streamNo;	// �������:0-������,1-������
	unsigned char sendAudio;// ������Ƶ:1-������Ƶ,0-��������Ƶ
	unsigned char reserved[2];
} SMsgAVIoctrlAVStream;


/*
IOTYPE_USER_IPCAM_GETSTREAMCTRL_REQ		= 0x0322,
** @struct SMsgAVIoctrlGetStreamCtrlReq
*/
typedef struct
{
	unsigned int channel;	// Camera Index
	unsigned char streamNo;	// �������:0-������,1-������
	unsigned char reserved[3];
}SMsgAVIoctrlGetStreamCtrlReq;

/*
IOTYPE_USER_IPCAM_SETSTREAMCTRL_REQ		= 0x0320,
IOTYPE_USER_IPCAM_GETSTREAMCTRL_RESP	= 0x0323,
** @struct SMsgAVIoctrlSetStreamCtrlReq, SMsgAVIoctrlGetStreamCtrlResq
*/
typedef struct
{
	unsigned int  channel;	// Camera Index
	unsigned char quality;	//refer to ENUM_QUALITY_LEVEL
	unsigned char streamNo;	// �������:0-������,1-������
	unsigned char reserved[2];
} SMsgAVIoctrlSetStreamCtrlReq, SMsgAVIoctrlGetStreamCtrlResq;

/*
IOTYPE_USER_IPCAM_SETSTREAMCTRL_RESP	= 0x0321,
** @struct SMsgAVIoctrlSetStreamCtrlResp
*/
typedef struct
{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetStreamCtrlResp;


/*
IOTYPE_USER_IPCAM_GETMOTIONDETECT_REQ	= 0x0326,
** @struct SMsgAVIoctrlGetMotionDetectReq
*/
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetMotionDetectReq;


/*
IOTYPE_USER_IPCAM_SETMOTIONDETECT_REQ		= 0x0324,
IOTYPE_USER_IPCAM_GETMOTIONDETECT_RESP		= 0x0327,
** @struct SMsgAVIoctrlSetMotionDetectReq, SMsgAVIoctrlGetMotionDetectResp
*/
typedef struct
{
	unsigned int channel; 		// Camera Index
	unsigned int sensitivity; 	// 0(Disabled) ~ 100(MAX):
								// index		sensitivity value
								// 0			0
								// 1			25
								// 2			50
								// 3			75
								// 4			100
}SMsgAVIoctrlSetMotionDetectReq, SMsgAVIoctrlGetMotionDetectResp;


/*
IOTYPE_USER_IPCAM_SETMOTIONDETECT_RESP	= 0x0325,
** @struct SMsgAVIoctrlSetMotionDetectResp
*/
typedef struct
{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetMotionDetectResp;


/*
IOTYPE_USER_IPCAM_DEVINFO_REQ			= 0x0330,
** @struct SMsgAVIoctrlDeviceInfoReq
*/
typedef struct
{
	unsigned char reserved[4];
}SMsgAVIoctrlDeviceInfoReq;


/*
IOTYPE_USER_IPCAM_DEVINFO_RESP			= 0x0331,
** @struct SMsgAVIoctrlDeviceInfo
*/
typedef struct
{
	unsigned char model[16];	// IPCam mode
	unsigned char vendor[16];	// IPCam manufacturer
	unsigned int version;		// IPCam firmware version	ex. v1.2.3.4 => 0x01020304;  v1.0.0.2 => 0x01000002
	unsigned int channel;		// Camera index
	unsigned int total;			// 0: No cards been detected or an unrecognizeable sdcard that could not be re-formatted.
								// -1: if camera detect an unrecognizable sdcard, and could be re-formatted
								// otherwise: return total space size of sdcard (MBytes)

	unsigned int free;			// Free space size of sdcard (MBytes)
	unsigned char reserved[8];	// reserved
}SMsgAVIoctrlDeviceInfoResp;

/*
IOTYPE_USER_IPCAM_SETPASSWORD_REQ		= 0x0332,
** @struct SMsgAVIoctrlSetPasswdReq
*/
#ifdef USE_APP_XUHUI
typedef struct
{
	char user[32];			   // The user name  /* tutk only one user, the RL support mutli user*/
	char oldpasswd[32];			// The old security code
	char newpasswd[32];			// The new security code
}SMsgAVIoctrlSetPasswdReq;

#else
/*
IOTYPE_USER_IPCAM_SETPASSWORD_REQ		= 0x0332,
** @struct SMsgAVIoctrlSetPasswdReq
*/
typedef struct
{
    
	char oldpasswd[32];			// The old security code
	char newpasswd[32];			// The new security code
	char user[32];
}SMsgAVIoctrlSetPasswdReq;

#endif
/*
IOTYPE_USER_IPCAM_SETPASSWORD_RESP		= 0x0333,
** @struct SMsgAVIoctrlSetPasswdResp
*/
typedef struct
{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetPasswdResp;


/*
IOTYPE_USER_IPCAM_LISTWIFIAP_REQ		= 0x0340,
** @struct SMsgAVIoctrlListWifiApReq
*/
typedef struct
{
	unsigned char reserved[4];
}SMsgAVIoctrlListWifiApReq;

typedef struct
{
	char ssid[32]; 				// WiFi ssid
	char mode;	   				// refer to ENUM_AP_MODE
	char enctype;  				// refer to ENUM_AP_ENCTYPE
	char signal;   				// signal intensity 0--100%
	char status;   				// 0 : invalid ssid or disconnected
								// 1 : connected with default gateway
								// 2 : unmatched password
								// 3 : weak signal and connected
								// 4 : selected:
								//		- password matched and
								//		- disconnected or connected but not default gateway
}SWifiAp;

/*
IOTYPE_USER_IPCAM_LISTWIFIAP_RESP		= 0x0341,
** @struct SMsgAVIoctrlListWifiApResp
*/
typedef struct
{
	unsigned int number; // MAX number: 1024(IOCtrl packet size) / 36(bytes) = 28
	SWifiAp stWifiAp[20];
}SMsgAVIoctrlListWifiApResp;

/*
IOTYPE_USER_IPCAM_SETWIFI_REQ			= 0x0342,
** @struct SMsgAVIoctrlSetWifiReq
*/
typedef struct
{
	unsigned char ssid[32];			//WiFi ssid
	unsigned char password[32];		//if exist, WiFi password
	unsigned char mode;				//refer to ENUM_AP_MODE
	unsigned char enctype;			//refer to ENUM_AP_ENCTYPE
	unsigned char reserved[10];
}SMsgAVIoctrlSetWifiReq;

//IOTYPE_USER_IPCAM_SETWIFI_REQ_2		= 0x0346,
typedef struct
{
	unsigned char ssid[32];		// WiFi ssid
	unsigned char password[64];	// if exist, WiFi password
	unsigned char mode;			// refer to ENUM_AP_MODE
	unsigned char enctype;		// refer to ENUM_AP_ENCTYPE
	unsigned char reserved[10];
}SMsgAVIoctrlSetWifiReq2;

/*
IOTYPE_USER_IPCAM_SETWIFI_RESP			= 0x0343,
** @struct SMsgAVIoctrlSetWifiResp
*/
typedef struct
{
	int result; //0: wifi connected; 1: failed to connect
	unsigned char reserved[4];
}SMsgAVIoctrlSetWifiResp;

/*
IOTYPE_USER_IPCAM_GETWIFI_REQ			= 0x0344,
** @struct SMsgAVIoctrlGetWifiReq
*/
typedef struct
{
	unsigned char reserved[4];
}SMsgAVIoctrlGetWifiReq;

/*
IOTYPE_USER_IPCAM_GETWIFI_RESP			= 0x0345,
** @struct SMsgAVIoctrlGetWifiResp //if no wifi connected, members of SMsgAVIoctrlGetWifiResp are all 0
*/
typedef struct
{
	unsigned char ssid[32];		// WiFi ssid
	unsigned char password[32]; // WiFi password if not empty
	unsigned char mode;			// refer to ENUM_AP_MODE
	unsigned char enctype;		// refer to ENUM_AP_ENCTYPE
	unsigned char signal;		// signal intensity 0--100%
	unsigned char status;		// refer to "status" of SWifiAp
}SMsgAVIoctrlGetWifiResp;

//changed: WI-FI Password 32bit Change to 64bit
//IOTYPE_USER_IPCAM_GETWIFI_RESP_2    = 0x0347,
typedef struct
{
 unsigned char ssid[32];	 // WiFi ssid
 unsigned char password[64]; // WiFi password if not empty
 unsigned char mode;	// refer to ENUM_AP_MODE
 unsigned char enctype; // refer to ENUM_AP_ENCTYPE
 unsigned char signal;  // signal intensity 0--100%
 unsigned char status;  // refer to "status" of SWifiAp
}SMsgAVIoctrlGetWifiResp2;

#if 0
/*
IOTYPE_USER_IPCAM_GETRECORD_REQ			= 0x0312,
** @struct SMsgAVIoctrlGetRecordReq
*/
typedef struct
{
	unsigned int channel; // Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetRecordReq;

/*
IOTYPE_USER_IPCAM_SETRECORD_REQ			= 0x0310,
IOTYPE_USER_IPCAM_GETRECORD_RESP		= 0x0313,
** @struct SMsgAVIoctrlSetRecordReq, SMsgAVIoctrlGetRecordResq
*/
typedef struct
{
	unsigned int channel;		// Camera Index
	unsigned int recordType;	// Refer to ENUM_RECORD_TYPE
	unsigned char reserved[4];
}SMsgAVIoctrlSetRecordReq, SMsgAVIoctrlGetRecordResq;

/*
IOTYPE_USER_IPCAM_SETRECORD_RESP		= 0x0311,
** @struct SMsgAVIoctrlSetRecordResp
*/
typedef struct
{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetRecordResp;
#endif

/*
IOTYPE_USER_IPCAM_GETRCD_DURATION_REQ	= 0x0316,
** @struct SMsgAVIoctrlGetRcdDurationReq
*/
typedef struct
{
	unsigned int channel; // Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetRcdDurationReq;

/*
IOTYPE_USER_IPCAM_SETRCD_DURATION_REQ	= 0x0314,
IOTYPE_USER_IPCAM_GETRCD_DURATION_RESP  = 0x0317,
** @struct SMsgAVIoctrlSetRcdDurationReq, SMsgAVIoctrlGetRcdDurationResp
*/
typedef struct
{
	unsigned int channel; 		// Camera Index
	unsigned int presecond; 	// pre-recording (sec)
	unsigned int durasecond;	// recording (sec)
}SMsgAVIoctrlSetRcdDurationReq, SMsgAVIoctrlGetRcdDurationResp;


/*
IOTYPE_USER_IPCAM_SETRCD_DURATION_RESP  = 0x0315,
** @struct SMsgAVIoctrlSetRcdDurationResp
*/
typedef struct
{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetRcdDurationResp;


typedef struct
{
	unsigned short year;	// The number of year.
	unsigned char month;	// The number of months since January, in the range 1 to 12.
	unsigned char day;		// The day of the month, in the range 1 to 31.
	unsigned char wday;		// The number of days since Sunday, in the range 0 to 6. (Sunday = 0, Monday = 1, ...)
	unsigned char hour;     // The number of hours past midnight, in the range 0 to 23.
	unsigned char minute;   // The number of minutes after the hour, in the range 0 to 59.
	unsigned char second;   // The number of seconds after the minute, in the range 0 to 59.
}STimeDay;

/*
IOTYPE_USER_IPCAM_LISTEVENT_REQ			= 0x0318,
** @struct SMsgAVIoctrlListEventReq
*/
typedef struct
{
	unsigned int channel; 		// Camera Index
	STimeDay stStartTime; 		// Search event from ...
	STimeDay stEndTime;	  		// ... to (search event)
	unsigned char event;  		// event type, refer to ENUM_EVENTTYPE
	unsigned char status; 		// 0x00: Recording file exists, Event unreaded
								// 0x01: Recording file exists, Event readed
								// 0x02: No Recording file in the event
	unsigned char reserved[2];
}SMsgAVIoctrlListEventReq;

#if USE_HTS_PROTOCOL
typedef struct
{
	STimeDay stBeginTime;
	unsigned char event;
	unsigned char status;	// 0x00: Recording file exists, Event unreaded
							// 0x01: Recording file exists, Event readed
							// 0x02: No Recording file in the event
	unsigned char reserved[2];
}SAvEvent;
#else

typedef struct
{
	STimeDay stBeginTime;
    STimeDay stEndTime;
	unsigned char event;
	unsigned char status;	// 0x00: Recording file exists, Event unreaded
							// 0x01: Recording file exists, Event readed
							// 0x02: No Recording file in the event
	unsigned char reserved[2];
}SAvEvent;

#endif

/*
IOTYPE_USER_IPCAM_LISTEVENT_RESP		= 0x0319,
** @struct SMsgAVIoctrlListEventResp
*/
typedef struct
{
	unsigned int  channel;		// Camera Index
	unsigned int  total;		// Total event amount in this search session
	unsigned char index;		// package index, 0,1,2...;
								// because avSendIOCtrl() send package up to 1024 bytes one time, you may want split search results to serveral package to send.
	unsigned char endflag;		// end flag; endFlag = 1 means this package is the last one.
	unsigned char count;		// how much events in this package
	unsigned char reserved[1];
	SAvEvent stEvent[1];		// The first memory address of the events in this package
}SMsgAVIoctrlListEventResp;


/*
IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL 	= 0x031A,
** @struct SMsgAVIoctrlPlayRecord
*/
typedef struct
{
	unsigned int channel;	// Camera Index
	unsigned int command;	// play record command. refer to ENUM_PLAYCONTROL
	unsigned int Param;		// command param, that the user defined
	STimeDay stTimeDay;		// Event time from ListEvent
	unsigned char reserved[4];
} SMsgAVIoctrlPlayRecord;

/*
IOTYPE_USER_IPCAM_RECORD_PLAYCONTROL_RESP 	= 0x031B,
** @struct SMsgAVIoctrlPlayRecordResp
*/
typedef struct
{
	unsigned int command;	// Play record command. refer to ENUM_PLAYCONTROL
	unsigned int result; 	// Depends on command
							// when is AVIOCTRL_RECORD_PLAY_START:
							//	result>=0   real channel no used by device for playback
							//	result <0	error
							//			-1	playback error
							//			-2	exceed max allow client amount
	unsigned char reserved[4];
} SMsgAVIoctrlPlayRecordResp; // only for play record start command

/*

IOTYPE_USER_IPCAM_SEACH_RECORD_IN_MONTH_REQ = 0x031C,
IOTYPE_USER_IPCAM_SEACH_RECORD_IN_MONTH_RESP= 0x031D,

*/
typedef struct 
{
	unsigned char	year;//��ѯ��,0~255ȡֵ��Χ������2000~2255��,��2017�꣬��yearΪ17
	unsigned char	month;//��ѯ�£�1~12Ϊȡֵ��Χ
	long lCalendarMap;//bit1~bit31����1�ŵ�31��,��¼�����bitλ=1��
}SMsgAVIoctrlRecordInMonthReq,SMsgAVIoctrlRecordInMonthResp;


/*
IOTYPE_USER_IPCAM_PTZ_COMMAND	= 0x1001,	// P2P Ptz Command Msg
** @struct SMsgAVIoctrlPtzCmd
*/
typedef struct
{
	unsigned char control;	// PTZ control command, refer to ENUM_PTZCMD
	unsigned char speed;	// PTZ control speed
	unsigned char point;	// no use in APP so far. preset position, for RS485 PT
	unsigned char limit;	// no use in APP so far.
	unsigned char aux;		// no use in APP so far. auxiliary switch, for RS485 PT
	unsigned char channel;	// camera index
	unsigned char reserve[2];
} SMsgAVIoctrlPtzCmd;

/*
IOTYPE_USER_IPCAM_EVENT_REPORT	= 0x1FFF,	// Device Event Report Msg
*/
/** @struct SMsgAVIoctrlEvent
 */
typedef struct
{
	STimeDay stTime;
	unsigned long time; 	// UTC Time
	unsigned int  channel; 	// Camera Index
	unsigned int  event; 	// Event Type
	unsigned char reserved[4];
} SMsgAVIoctrlEvent;



#if 0

/* 	IOTYPE_USER_IPCAM_GET_EVENTCONFIG_REQ	= 0x0400,	// Get Event Config Msg Request
 */
/** @struct SMsgAVIoctrlGetEventConfig
 */
typedef struct
{
	unsigned int	channel; 		  //Camera Index
	unsigned char   externIoOutIndex; //extern out index: bit0->io0 bit1->io1 ... bit7->io7;=1: get this io value or not get
    unsigned char   externIoInIndex;  //extern in index: bit0->io0 bit1->io1 ... bit7->io7; =1: get this io value or not get
	char reserved[2];
} SMsgAVIoctrlGetEventConfig;

/*
	IOTYPE_USER_IPCAM_GET_EVENTCONFIG_RESP	= 0x0401,	// Get Event Config Msg Response
	IOTYPE_USER_IPCAM_SET_EVENTCONFIG_REQ	= 0x0402,	// Set Event Config Msg req
*/
/* @struct SMsgAVIoctrlSetEventConfig
 * @struct SMsgAVIoctrlGetEventCfgResp
 */
typedef struct
{
	unsigned int    channel;        // Camera Index
	unsigned char   mail;           // enable send email
	unsigned char   ftp;            // enable ftp upload photo
	unsigned char   externIoOutStatus;   // enable extern io output //bit0->io0 bit1->io1 ... bit7->io7; 1:on; 0:off
	unsigned char   p2pPushMsg;			 // enable p2p push msg
	unsigned char   externIoInStatus;    // enable extern io input  //bit0->io0 bit1->io1 ... bit7->io7; 1:on; 0:off
	char            reserved[3];
}SMsgAVIoctrlSetEventConfig, SMsgAVIoctrlGetEventCfgResp;

/*
	IOTYPE_USER_IPCAM_SET_EVENTCONFIG_RESP	= 0x0403,	// Set Event Config Msg resp
*/
/** @struct SMsgAVIoctrlSetEventCfgResp
 */
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned int result;	// 0: success; otherwise: failed.
}SMsgAVIoctrlSetEventCfgResp;

#endif


/*
IOTYPE_USER_IPCAM_SET_ENVIRONMENT_REQ		= 0x0360,
** @struct SMsgAVIoctrlSetEnvironmentReq
*/
typedef struct
{
	unsigned int channel;		// Camera Index
	unsigned char mode;			// refer to ENUM_ENVIRONMENT_MODE
	unsigned char TwolightMode;//��ͨҹ��:0; ȫ��ҹ��:1; ����ҹ��:2
	unsigned char reserved[2];
}SMsgAVIoctrlSetEnvironmentReq;


/*
IOTYPE_USER_IPCAM_SET_ENVIRONMENT_RESP		= 0x0361,
** @struct SMsgAVIoctrlSetEnvironmentResp
*/
typedef struct
{
	unsigned int channel; 		// Camera Index
	unsigned char result;		// 0: success; otherwise: failed.
	unsigned char reserved[3];
}SMsgAVIoctrlSetEnvironmentResp;


/*
IOTYPE_USER_IPCAM_GET_ENVIRONMENT_REQ		= 0x0362,
** @struct SMsgAVIoctrlGetEnvironmentReq
*/
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetEnvironmentReq;

#ifndef USE_APP_XUHUI
/*
IOTYPE_USER_IPCAM_GET_ENVIRONMENT_RESP		= 0x0363,
** @struct SMsgAVIoctrlGetEnvironmentResp
*/
typedef struct
{
	unsigned int channel; 		// Camera Index
	unsigned char mode;			// refer to ENUM_ENVIRONMENT_MODE
	unsigned char reserved[3];
}SMsgAVIoctrlGetEnvironmentResp;

#else

typedef struct
{
	unsigned int channel;		  // Camera Index
	unsigned char mode; 		   // refer to ENUM_ENVIRONMENT_MODE
	unsigned char TwolightMode;//��ͨҹ��:0; ȫ��ҹ��:1; ����ҹ��:2
	unsigned char reserved[2];
}SMsgAVIoctrlGetEnvironmentResp;

#endif

/*
IOTYPE_USER_IPCAM_SET_VIDEOMODE_REQ			= 0x0370,
** @struct SMsgAVIoctrlSetVideoModeReq
*/
typedef struct
{
	unsigned int channel;	// Camera Index
	unsigned char mode;		// refer to ENUM_VIDEO_MODE
	unsigned char reserved[3];
}SMsgAVIoctrlSetVideoModeReq;


/*
IOTYPE_USER_IPCAM_SET_VIDEOMODE_RESP		= 0x0371,
** @struct SMsgAVIoctrlSetVideoModeResp
*/
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned char result;	// 0: success; otherwise: failed.
	unsigned char reserved[3];
}SMsgAVIoctrlSetVideoModeResp;


/*
IOTYPE_USER_IPCAM_GET_VIDEOMODE_REQ			= 0x0372,
** @struct SMsgAVIoctrlGetVideoModeReq
*/
typedef struct
{
	unsigned int channel; 	// Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetVideoModeReq;


/*
IOTYPE_USER_IPCAM_GET_VIDEOMODE_RESP		= 0x0373,
** @struct SMsgAVIoctrlGetVideoModeResp
*/
typedef struct
{
	unsigned int  channel; 	// Camera Index
	unsigned char mode;		// refer to ENUM_VIDEO_MODE
	unsigned char reserved[3];
}SMsgAVIoctrlGetVideoModeResp;


/*
/IOTYPE_USER_IPCAM_FORMATEXTSTORAGE_REQ			= 0x0380,
** @struct SMsgAVIoctrlFormatExtStorageReq
*/
typedef struct
{
	unsigned int storage; 	// Storage index (ex. sdcard slot = 0, internal flash = 1, ...)
	unsigned char reserved[4];
}SMsgAVIoctrlFormatExtStorageReq;


/*
IOTYPE_USER_IPCAM_FORMATEXTSTORAGE_REQ		= 0x0381,
** @struct SMsgAVIoctrlFormatExtStorageResp
*/
typedef struct
{
	unsigned int  storage; 	// Storage index
	unsigned char result;	// 0: success;
							// -1: format command is not supported.
							// otherwise: failed.
	unsigned char reserved[3];
}SMsgAVIoctrlFormatExtStorageResp;


typedef struct
{
	unsigned short index;		// the stream index of camera
	unsigned short channel;		// the channel index used in AVAPIs, that is ChID in avServStart2(...,ChID)
	char reserved[4];
}SStreamDef;


/*	IOTYPE_USER_IPCAM_GETSUPPORTSTREAM_REQ			= 0x0328,
 */
typedef struct
{
	unsigned char reserved[4];
}SMsgAVIoctrlGetSupportStreamReq;


/*	IOTYPE_USER_IPCAM_GETSUPPORTSTREAM_RESP			= 0x0329,
 */
typedef struct
{
	unsigned int number; 		// the quanity of supported audio&video stream or video stream
	SStreamDef streams[1];
}SMsgAVIoctrlGetSupportStreamResp;


/* IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_REQ			= 0x032A, //used to speak. but once camera is connected by App, send this at once.
 */
typedef struct
{
	unsigned int channel;		// camera index
	char reserved[4];
}SMsgAVIoctrlGetAudioOutFormatReq;

/* IOTYPE_USER_IPCAM_GETAUDIOOUTFORMAT_RESP			= 0x032B,
 */
typedef struct{
    unsigned int channel;        // camera index
    int codecId;                    // refer to ENUM_CODECID in AVFRAMEINFO.h
    char sample_rate;
    char bitdata;
    char channels;
    char reserved[1];
}SMsgAVIoctrlGetAudioOutFormatResp;

/* IOTYPE_USER_IPCAM_RECEIVE_FIRST_IFRAME			= 0x1002,
 */
typedef struct
{
	unsigned int channel;		// camera index
	char reserved[4];
}SMsgAVIoctrlReceiveFirstIFrame;

/* IOTYPE_USER_IPCAM_GET_FLOWINFO_REQ              = 0x390
 */
typedef struct
{
	unsigned int channel;			// camera index
	unsigned int collect_interval;	// seconds of interval to collect flow information
									// send 0 indicates stop collecting.
}SMsgAVIoctrlGetFlowInfoReq;

/* IOTYPE_USER_IPCAM_GET_FLOWINFO_RESP            = 0x391
 */
typedef struct
{
	unsigned int channel;			// camera index
	unsigned int collect_interval;	// seconds of interval client will collect flow information
}SMsgAVIoctrlGetFlowInfoResp;

/* IOTYPE_USER_IPCAM_CURRENT_FLOWINFO              = 0x392
 */
typedef struct
{
	unsigned int channel;						// camera index
	unsigned int total_frame_count;				// Total frame count in the specified interval
	unsigned int lost_incomplete_frame_count;	// Total lost and incomplete frame count in the specified interval
	unsigned int total_expected_frame_size;		// Total expected frame size from avRecvFrameData2()
	unsigned int total_actual_frame_size;		// Total actual frame size from avRecvFrameData2()
	unsigned int timestamp_ms;					// Timestamp in millisecond of this report.
	char reserved[8];
}SMsgAVIoctrlCurrentFlowInfo;

/* IOTYPE_USER_IPCAM_GET_TIMEZONE_REQ               = 0x3A0
 * IOTYPE_USER_IPCAM_GET_TIMEZONE_RESP              = 0x3A1
 * IOTYPE_USER_IPCAM_SET_TIMEZONE_REQ               = 0x3B0
 * IOTYPE_USER_IPCAM_SET_TIMEZONE_RESP              = 0x3B1
 */

#if USE_HTS_PROTOCOL

typedef struct
{
	int cbSize;							// the following package size in bytes, should be sizeof(SMsgAVIoctrlTimeZone)
	int nIsSupportTimeZone;
	int nGMTDiff;						// the difference between GMT in hours
	char szTimeZoneString[256];			// the timezone description string in multi-bytes char format
}SMsgAVIoctrlTimeZone;

#else

typedef struct
{
	int cbSize;							// the following package size in bytes, should be sizeof(SMsgAVIoctrlTimeZone)
	int nIsSupportTimeZone;
	int nTimeZone;						// the difference between GMT in hours
	char szTimeZoneString[256];			// the timezone description string in multi-bytes char format
}SMsgAVIoctrlTimeZone;

#endif


/*
// dropbox support
IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_REQ      = 0x500,
IOTYPE_USER_IPCAM_GET_SAVE_DROPBOX_RESP     = 0x501,
*/
typedef struct
{
    unsigned short nSupportDropbox;     // 0:no support/ 1: support dropbox
    unsigned short nLinked;             // 0:no link/ 1:linked
    char szLinkUDID[64];                // Link UDID for App
}SMsgAVIoctrlGetDropbox;


/*
 // dropbox support
 IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_REQ      = 0x502,
 IOTYPE_USER_IPCAM_SET_SAVE_DROPBOX_RESP     = 0x503,
 */
typedef struct
{
    unsigned short nLinked;             // 0:no link/ 1:linked
    char szLinkUDID[64];                // UDID for App
    char szAccessToken[128];             // Oauth token
    char szAccessTokenSecret[128];       // Oauth token secret
	char szAppKey[128];                  // App Key (reserved)
	char szSecret[128];                  // Secret  (reserved)
}SMsgAVIoctrlSetDropbox;

/*
IOTYPE_USER_IPCAM_PUSH_STREAMING_START		= 0x202,
** @struct SMsgAVIoctrlPushStream
*/
typedef struct
{
	unsigned int channel;	// Camera Channel
	char szDeviceUID[20];	// Device UID
	unsigned int  event; 	// Event Type, refer to ENUM_EVENTTYPE
	unsigned char reserved[4];
} SMsgAVIoctrlPushStream;

//
// ����ʹ�õ����ݽṹ
//
typedef struct{
	int			AlarmType;		// ��������
	long long 	AlarmTime;		// ����ʱ��
	char		AlarmDID[64];	// �����豸ID
}SMsgAVIoctrlAlarmingReq;
//
// ������Ϣ����ע��ʹ�õ����ݽṹ
//
typedef struct{

	char	AppKey[64];
	char	Master[64];
	char	Alias[64];
	int		Type;
	char    FcmKey[64]; // for google push
#ifdef USE_APP_XUHUI
	char	endPointUrl[64];
	char	accessKeyId[64];
	char	accessKeySecret[64];
	char	appId[64];
	char	bucketName[64];
	char	objectName[64];
#endif

}SMsgAVIoctrlSetPushReq,SMsgAVIoctrlDelPushReq;



//NEW ADD 20161209
/**
 * IOTYPE_USER_GET_CAPACITY_REQ
 * IOTYPE_USER_GET_CAPACITY_RESP
 */
 
#ifdef USE_APP_XUHUI
typedef struct
{
	unsigned char 	devType;			// �豸����: 0-��Ƭ��, 1-ҡͷ��, 2-���������...
	unsigned char 	netFamily;			// �����ṩ��: 1-��������, 2-tutk...
	unsigned char 	serverID;			// ������ID: ����,Ĭ��Ϊ0
	unsigned char 	language;			// �豸�̼����԰汾: 0Ϊ�Զ���Ӧ(��������),1Ϊ��������,2Ϊ��������,3ΪӢ��
	unsigned int 	odmID;				// odm��ID: 0-��˴�Լ�, 01-TCL, 02-����
	char 			hid[32];			// �豸���к�
	char			reserve0[28];		// ����0
	unsigned char 	suportPTZ;			// ֧��ptz����: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportAudioIn;		// ֧����Ƶ�ɼ�: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportAudioOut;		// ֧����������: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportStorage;		// ֧�ִ洢: ֧��Ϊ1,��֧��Ϊ0
	unsigned int 	reservel;			// ����1
	char 			version[16];		// �汾��:  ���ñ�������201612121816(������ʱ��)
	char 			model[16];			// ��Ʒ����ͺ�
	unsigned int 	suportChannelNum;	// ��ǰ�豸�ܵ����������
	unsigned int 	manufacture;		// �����̺�: ����������ϵ��ȡ
	unsigned char	supportPir;		    // ֧��PIR
	unsigned char	supportDemolAlarm;	// ֧�ַ��𱨾�
	unsigned char	supportWakeUpControl;	// ��ʾ���ѹ��ܿ����1 ֧�֣�0 ��֧��˫��ĵ������壬�������в���ʾ
	unsigned char	supportOpenDoor;	// ֧���ֻ�����, 1 ֧�֣�0 ��֧��
	unsigned char	is_owner;			// ֧���ֻ�����, 1 ֧�֣�0 ��֧��
	unsigned char	supportLight;		// �Ƿ�֧��˫��Դ���ƣ�1 ֧�֣�0 ��֧��
	char 			reserve2[34];		// ����44
}SMsgAVIoctrlGetCapacityResp;


typedef struct
{
	char 		user_sign[32];			// �û�ǩ��������ô���洢��ȣ����ж��Ƿ�Ϊ��������һ����������Ҫ�洢��flash��
	char 		isWaveConfig;			// ����������־��1�������������������������ʽĬ��Ϊ0
	char 		reserve[223];			//����
}SMsgAVIoctrlGetCapcityReq;

#else

typedef struct
{
	unsigned char 	devType;			// �豸����: 0-��Ƭ��, 1-ҡͷ��, 2-���������...
	unsigned char 	netFamily;			// �����ṩ��: 1-��������, 2-tutk...
	unsigned char 	serverID;			// ������ID: ����,Ĭ��Ϊ0
	unsigned char 	language;			// �豸�̼����԰汾: 0Ϊ�Զ���Ӧ(��������),1Ϊ��������,2Ϊ��������,3ΪӢ��
	unsigned int 	odmID;				// odm��ID: 0-��˴�Լ�, 01-TCL, 02-����
	char 			hid[32];			// �豸���к�
	char 			videoCodec[8];		// ��Ƶ����: ����Сд, ��: "h264", "h265"
	char 			audioCodec[8];		// ��Ƶ����: ����Сд, ��: "aac", "mp3", "711a", "711u", "pcm"
	unsigned int 	audioSampleRate;	// ��Ƶ����: 8000,16000
	unsigned int 	audioChannelNum;	// ��Ƶͨ��: 1-��ͨ, 2-˫ͨ��
	unsigned int 	audioBitsPerSample;	// ��Ƶbits: 8, 16
	unsigned char 	suportPTZ;			// ֧��ptz����: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportAudioIn;		// ֧����Ƶ�ɼ�: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportAudioOut;		// ֧����������: ֧��Ϊ1,��֧��Ϊ0
	unsigned char 	suportStorage;		// ֧�ִ洢: ֧��Ϊ1,��֧��Ϊ0
	unsigned int 	panoramaMode;		// ȫ����װģʽ: 0-����ʽ,1-�ڹ�ʽ
	char 			version[16];		// �汾��:  ���ñ�������201612121816(������ʱ��)
	char 			model[16];			// ��Ʒ����ͺ�
	char			supportPir;		    // ֧��PIR
	char			supportDemolAlarm;	// ֧�ַ��𱨾�
	char			supportWakeUpControl;	// ��ʾ���ѹ��ܿ����1 ֧�֣�0 ��֧��˫��ĵ������壬�������в���ʾ
	char			supportOpenDoor;	// ֧���ֻ�����, 1 ֧�֣�0 ��֧��
	char 			reserve2[36];		// ����44
}SMsgAVIoctrlGetCapacityResp;
#endif

//
// ͨ��Ӧ�����ݽṹ
//
typedef struct{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlGetMDPReq,
SMsgAVIoctrlSetMDPResp,
SMsgAVIoctrlSetPushResp,
SMsgAVIoctrlDelPushResp,
SMsgAVIoctrlAlarmingResp,
SMsgAVIoctrlDoorOpenResp,
SMsgAVIoctrlDoorPassResp;




typedef struct						// 0x602
{
    unsigned int channel; // Camera Index
    unsigned char reserved[4];
} SMsgAVIoctrlGetBrightReq;


typedef struct					//0x603  0x604
{
    unsigned int channel; // Camera Index
    unsigned char bright; // refer to ENUM_BRIGHT_LEVEL
    unsigned char reserved[3];
} SMsgAVIoctrlSetBrightReq, SMgAVIoctrlGetBrightResp;


typedef struct				//0x605
{
    unsigned int result; // 0: success; otherwise: failed.
    unsigned char reserved[4];
} SMsgAVIoctrSeltBrightResp;


typedef struct				//0x606
{
    unsigned int channel; // Camera Index
    unsigned char reserved[4];
} SMsgAVIoctrlGetContrastReq;


typedef struct				//0x607 0x608
{
    unsigned int channel;    // Camera Index
    unsigned char contrast; // refer to ENUM_CONTRAST_LEVEL [1-5]
    unsigned char reserved[3];
} SMsgAVIoctrlSetContrastReq, SMgAVIoctrlGetContrastResp;


typedef struct				//0x609
{
    unsigned int result; // 0: success; otherwise: failed.
    unsigned char reserved[4];
} SMsgAVIoctrSetContrastResp;


typedef struct					// 0x723
{
        unsigned int channel;                 // Camera Index
        unsigned char result;                 // refer to NIGHTMODE
        unsigned char reserved[3];
}SMsgXMSetNightmodeResp;

typedef struct		//0x724
{
        unsigned int channel;                // Camera Index
        unsigned char reserved[4];
}SMsgXMGetNightmodeReq;


typedef struct					// 0x722 0x725
{
        unsigned int channel;                 // Camera Index
        unsigned char mode;                 // refer to NIGHTMODE
        unsigned char reserved[3];
}SMsgXMGetNightmodeResp, SMsgXMSetNightmodeReq;

typedef struct				//0x60A
{
    unsigned int channel; // Camera Index
    unsigned char reserved[4];
} SMsgAVIoctrlGetBatteryReq;


//P.S. IP camera is allowed to send specific RESP IO command without getting the REQ first.
typedef struct				//0x60B
{
    unsigned int channel; // Camera Index
    unsigned char battery; // 0 �C 100 (%) ; -1 : charging
    unsigned char reserved[3];
} SMsgAVIoctrlGetBatteryResp;


typedef struct			//0X732
{
	unsigned int channel;                // Camera Index
	unsigned char reserved[4];
}SMsgAVIoctrlGetCloudRecordReq;



typedef struct			//0X733
{
	int channel; // Camera Index
	unsigned char recordmode;// 0 10s // 1 20s //2 30s
	unsigned char reserved[3];
}SMsgAVIoctrlGetCloudRecordResp;


typedef struct //0x734
{
	int channel; // Camera Index
	unsigned char recordmode;// 0 10s // 1 20s //2 30s
	unsigned char reserved [3];
}SMsgAVIoctrlSetCloudRecordReq;


typedef struct			//0x735
{
	int channel; //Camera Index
	unsigned char result;  // 0: success; otherwise: failed
	unsigned char reserved[3];
}SMsgAVIoctrlSetCloudRecordResp;


typedef struct   // 0x802  0x805 tk kalay door phone
{
       int channel; // Camera Index
        unsigned char TimeStamp; // 0: Liveview timestamp OFF
                                    // 1: Liveview timestamp ON  year/month/day
                                    // 2: Liveview timestamp ON  month/day/year
                                    // 3: Liveview timestamp ON  day/month/year
        unsigned char reserved[3];
}SMsgAVIoctrlSetLivevieTimeStampReq,SMsgAVIoctrlGetLivevieTimeStampResp;

typedef struct	// 0x803  for tutk kalay door phone
{
        int channel; //Camera Index
        unsigned char result;  // 0: success; otherwise: failed
        unsigned char reserved[3];
}SMsgAVIoctrlSetLivevieTimeStampResp;

typedef struct // 0x804
{
        unsigned int channel;                // Camera Index
        unsigned char reserved[4];
}SMsgAVIoctrlGetLivevieTimeStampReq;


typedef enum{
	AVIOCTRL_LANG_EN,
	AVIOCTRL_LANG_ZH,
	AVIOCTRL_LANG_JA,
}ENUM_LANGUAGE_TYPE;

typedef struct
{
	int power_ctrl;	// 0x1: reboot 0x3: reset
	int lang;
	unsigned char reserved[4];
}SMsgAVIoctrlSetSystemReq,
SMsgAVIoctrlSetSystemResp,
SMsgAVIoctrlGetSystemReq,
SMsgAVIoctrlGetSystemResp;

#ifdef MODULE_SUPPORT_RF433
/*
	433 device:
	                id,type,name
*/
typedef enum{
	DEV_433_TYPE_REMOTE_CONTROL = 10,	// ң��
	DEV_433_TYPE_ALARM,					// ����
	DEV_433_TYPE_OTHER,					// ����
}ENUM_DEV_433_TYPE;

typedef struct{
	int 	id;				// 433 ��ʶ
	int		type;			// 433 �豸���� ENUM_DEV_433_TYPE
	char	name[64];		// �豸����
}SMsgAVIoctrlCfg433Req,SMsgAVIoctrlSet433Req,SMsgAVIoctrlDel433Req,S433Dev;;
typedef struct{
	int		num;			// �豸����
	S433Dev	dev[32];		// �豸
}SMsgAVIoctrlGet433Resp;

#if 0
typedef enum{
	AVIOCTRL_CFG_433_OK = 0,	// ��Գɹ�
	AVIOCTRL_CFG_433_TIMEOUT,	// ��Գ�ʱ
	AVIOCTRL_CFG_433_MAX,		// �豸����
	AVIOCTRL_CFG_433_WAITING,			// ����ѧϰ
	AVIOCTRL_CFG_433_EXISTS,			// �豸�Ѵ���
}ENUM_AVIOTC_CFG_433_RET;
#endif
typedef struct{
	int		result;			// ENUM_AVIOTC_CFG_433_RET
	int		none;			// Ԥ��
}SMsgAVIoctrlGet433Req,SMsgAVIoctrlCfg433Resp,SMsgAVIoctrlSet433Resp,SMsgAVIoctrlDel433Resp,SMsgAVIoctrlCfg433ExitReq,SMsgAVIoctrlCfg433ExitResp;
#endif


/**
 * IOTYPE_USER_IPCAM_HEARTBEAT_REQ    // 0X890
 * IOTYPE_USER_IPCAM_HEARTBEAT_RESP  // 0X891
 */
typedef struct
{
	int		seqNo;				// ���
	double 	timeStamp;			// ʱ���
	char 	reserve[12];		// ����ֵ
}SMsgAVIoctrlHeartbeatReq, SMsgAVIoctrlHeartbeatResp;


/*
 * IOTYPE_USER_IPCAM_GET_AUDIO_VOLUME_REQ 			// 0x8a0,
 * IOTYPE_USER_IPCAM_GET_AUDIO_VOLUME_RESP			// 0x8a1,
 * IOTYPE_USER_IPCAM_SET_AUDIO_VOLUME_REQ 			// 0x8a2,
 * IOTYPE_USER_IPCAM_SET_AUDIO_VOLUME_RESP			// 0x8a3,
*/

typedef struct{
	unsigned int level; // 0~4,5������0������ͣ�4�������
	unsigned char reserved[4];
}SMsgAVIoctrlAudioVolumeSetReq, SMsgAVIoctrlAudioVolumeSetResp, SMsgAVIoctrlAudioVolumeGetReq, SMsgAVIoctrlAudioVolumeGetResp;


//NEW ADD ��˴�������ƶ���ⱨ�������ṹ��
/**
 * IOTYPE_USER_IPCAM_GET_MD_ALAM_REQ
 * IOTYPE_USER_IPCAM_GET_MD_ALAM_RESP
 * IOTYPE_USER_IPCAM_SET_MD_ALAM_REQ
 * IOTYPE_USER_IPCAM_SET_MD_ALAM_RESP
 */
typedef struct{
	unsigned char 	enable;			// �ƶ����ʹ��: 1Ϊ����,0�ر�
	unsigned char	level;			// �ƶ���⼶��: 0-4, 0Ϊ�ٶ�,4Ϊ����
	unsigned char	trigAudioOut;	// �����������: 1Ϊ����,0�ر�
	unsigned char	trigRecord;		// ����¼��
	unsigned char	trigMsgPush;	// ������Ϣ����
	unsigned char   alarmPirEnable; // PIR ����
	unsigned char   alarmDelEnable; // ���𱨾�����
	unsigned char	resrver[10];	// ����12 -10
	unsigned char	frequency;		// �ƶ����Ƶ�ʣ���λ��:0Ϊ�̶�Ĭ��,����ָ��������

	unsigned char	startHour;		// ÿ����ʼʱ
	unsigned char 	startMins;		// ÿ����ʼ��
	unsigned char 	closeHour;		// ÿ�����ʱ
	unsigned char 	closeMins;		// ÿ�������
}SMsgAVIoctrlMDAlarmReq,SMsgAVIoctrlMDAlarmResp;



//
// OSD �������ȡ
//
typedef struct {
	int 	channel;

	int		channel_name_enbale;
	char 	channel_name_text[128];
	float	channel_name_x;
	float	channel_name_y;

	int		datetime_enable;
	int		date_fmt;	// 0:XXXX-XX-XX year mon date; 1:XX-XX-XXXX mon date year 2:XX-XX-XXXX date mon year
	int		date_sprtr;	// 0 :, 1 -, 2 //, 3 .
	int		time_fmt;	// 0 - 24,1 - 12
	int		display_week;
	float	datatime_x;
	float	datatime_y;

	int		channel_id_enable;
	char	channel_id_text[128];
	float	channel_id_x;
	float	channel_id_y;
}SMsgAVIoctrlSetOSDReq,SMsgAVIoctrlGetOSDReq,SMsgAVIoctrlGetOSDResp;

typedef struct{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
}SMsgAVIoctrlSetOSDResp;



typedef enum
{
    AVIOCTRL_CRUISEMODE_VERTICAL =0x00,//����Ѳ��
    AVIOCTRL_CRUISEMODE_HORIZONTAL =0x01,//����Ѳ��
}ENUM_CRUISE_MODE;

typedef struct
{
    unsigned int channel; // camera index
    unsigned char mode; // refer to ENUM_CRUISE_MODE
    unsigned char reserved[3];
}SMsgAVIoctrlCruiseStart;

typedef struct
{
    unsigned int channel; // camera index
    unsigned char reserved[4];
}SMsgAVIoctrlCruiseStop;

typedef struct
{
    unsigned int channel;  // camera index
    unsigned int nPresetIdx;	// �O�����A���cindex	��ֵ���� 0~3(��ʾ�A���c1~4)
}SMsgAVIoctrlSetPresetReq;

typedef struct
{
    int result;	// �؂�ֵ	0: success; otherwise: failed
    unsigned char reserved[4];
}SMsgAVIoctrlSetPresetResp;

typedef struct
{
    unsigned int channel;  // camera index
    unsigned int nPresetIdx;  // �d���A���cindex	��ֵ���� 0~3(��ʾ�A���c1~4)
}SMsgAVIoctrlGetPresetReq,SMsgAVIoctrlGetPresetResp;

//�̼���������ṹ��
/*
IOTYPE_USER_IPCAM_UPDATE_REQ
 */
typedef struct 
{
	int 		updateType;			// ������ʽ:�Զ�Ϊ0,���Ϊ1��ʹ��urlAddrָ���ĵ�ַ
	char 		md5[64];			// ��updateType=1ʱ,�˵�ַ��Ч,�ļ���md5ֵ
	char 		urlAddr[250];		// ��updateType=1ʱ,�˵�ַ��Ч,ʹ�ô˵�ַָ���������ļ� 
}SMsgAVIoctrlUpdateReq;



//�̼��������Ȳ�ѯ�ṹ��
/*
IOTYPE_USER_IPCAM_UPDATE_PROG_REQ
IOTYPE_USER_IPCAM_UPDATE_PROG_RESP
 */
typedef struct 
{
	char 			version[16];		// �汾��:  ���ñ�������201612121816(������ʱ��)
	unsigned int 	progress;			// ��������:0-100
	int 			status;				// ����״̬: =0Ϊ����,<0Ϊ����
}SMsgAVIoctrlUpdateProgressReq, SMsgAVIoctrlUpdateProgressResp;

/**
 * IOTYPE_USER_IPCAM_SHELL_REQ
 */
typedef struct 
{
	int				waitEnd;			// �ȴ�ִ�н���: 1Ϊ�ȴ�,0Ϊ���ȴ�
	char 			value[250];			// shell������ 
}SMsgAVIoctrlShellReq;

/**
 * IOTYPE_USER_IPCAM_SHELL_RESQ
 */
typedef struct 
{
	int		result;				// �ȴ�ִ�н���
	char 	msg[640];			// shellִ�еķ�����Ϣ 
}SMsgAVIoctrlShellResp;

/*
/IOTYPE_USER_IPCAM_GET_SDCARD_REQ			= 0x0856,
** @struct SMsgAVIoctrlGetSdcardReq
*/
typedef struct
{
	unsigned int storage; 	// Storage index (ex. sdcard slot = 0, internal flash = 1, ...)
	unsigned char reserved[4];
}SMsgAVIoctrlGetSdcardReq;

/*
/IOTYPE_USER_IPCAM_GET_SDCARD_RESQ			= 0x0857,
** @struct SMsgAVIoctrlGetSdcardReq
*/

typedef struct{
	int	status;	//sd 0,δ���룬1����
	int size; 	//sd �ܴ�СMB
	int free;	//sd ʣ��ռ�MB
	int format_process; //   0 -100
}SMsgAVIoctrlGetSDCardResp;


typedef struct
{
	unsigned int channel;		// Camera Index
	unsigned int recordType;	// Refer to ENUM_RECORD_TYPE

	// schedule for every day
	int	startHour;				// ��ʼʱ
	int startMins;				// ��ʼ��
	int closeHour;				// ����ʱ
	int closeMins;				// ������

	unsigned char reserved[4];  // reserved[0],¼��ʱ�䣬��λ�룬0��Ĭ��Ϊ15��
}SMsgAVIoctrlGetRecordReq,
SMsgAVIoctrlGetRecordResp,SMsgAVIoctrlGetRecordResq,SMsgAVIoctrlSetRecordReq, SMsgAVIoctrlSetRecordResp;

//
// �ƶ��������ʹ�õ����ݽṹ
//
typedef struct{
	int	MotionEnable;			// �ƶ����ʹ��
	int	MotionLevel;			// �ƶ���⼶��
	int	MotionAlarmFrequency;	// �ƶ����Ƶ�ʣ���λ��
	int	MotionAlarmType;		// �ƶ��������

	// schedule for every day
	int	MotionStartHour;		// ÿ����ʼʱ
	int MotionStartMins;		// ÿ����ʼ��
	int MotionCloseHour;		// ÿ�����ʱ
	int MotionCloseMins;		// ÿ�������
}SMsgAVIoctrlSetMDPReq,SMsgAVIoctrlGetMDPResp;


enum{
    TUTK_SD_STATUS_NOTINIT = 0, //sd ��������
    TUTK_SD_STATUS_OK,          //sd ״̬����
    TUTK_SD_STATUS_NOTFORMAT,   //sd δ��ʽ��
    TUTK_SD_STATUS_FORMAT_OK,   //sd ��ʽ��OK
    TUTK_SD_STATUS_READONLY,    //sd ֻ��
    TUTK_SD_STATUS_FORMATING    //sd ���ڸ�ʽ��
};


typedef struct{

unsigned int channel; // Camera Index  //0: �ѽ���TF��   otherwise:δ����TF��
unsigned char reserved[4];

}SMsgAVIoctrlGetTFCardStateReq,SMsgAVIoctrlGetTFCardStateResp;


typedef struct{

unsigned int enable; // 0,�ر�˫���ѹ��ܣ�1 ��˫����
unsigned char reserved[4];

}SMsgAVIoctrlGetWakeUpStateReq,SMsgAVIoctrlGetWakeUpStateResp,SMsgAVIoctrlSetWakeUpStateReq,SMsgAVIoctrlSetWakeUpStateResp;

/*IOTYPE_USER_IPCAM_GET_AUDIO_REQ
IOTYPE_USER_IPCAM_GET_AUDIO_RESP
IOTYPE_USER_IPCAM_SET_AUDIO_REQ
IOTYPE_USER_IPCAM_SET_AUDIO_RESP
 */
typedef struct {
	unsigned char 	enableIn;			// ��������(��Ƶ�ɼ�)
	unsigned char 	enableOut;			// �������(��Ƶ���)
}SMsgAVIoctrlAudioDevice;

//IOTYPE_USER_IPCAM_SET_DEVICETIME_REQ (0x816)
//    IOTYPE_USER_IPCAM_GET_DEVICETIME_RESP       = 0x819,
// ����ʱ��ṹ��
typedef struct{
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;
	unsigned char nIsSupportSync;  /* 1 supported; 0 unsupported  */
	int nGMTOffset;  //the offset between GMT in minutes
}SMsgAVIoctrlSetDeviceTimeReq,
SMsgAVIoctrlGetDeviceTimeResp;

//IOTYPE_USER_IPCAM_SET_DEVICETIME_RESP (0x817)
typedef struct{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4]; 
} SMsgAVIoctrlSetDeviceTimeResp;

//IOTYPE_USER_IPCAM_UPDATE_CLOUD_REQ (0x8B4)
typedef struct{
	int result;	// 0: success; otherwise: failed.
	unsigned char reserved[4];
} SMsgGeneralResp;

#endif
