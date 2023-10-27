
/*!
* @file
*  HieServer_PrivateCfg.h
* @brief
*  ˽��������Ϣ����
* @author
*  caoli
* @date
*  2009��4��25��
* @version
*  1.0.0
*/

#ifndef HIE_SERVER_CFG_H
#define HIE_SERVER_CFG_H

/*!
*\file		HieServer_Cfg.h
*\brief	Զ������
*\author	����
*\date		2009/8/20

���ܣ�
-#	Զ�����ù���ϵͳ���ýṹ�嶨��

˵����
-#	���ļ�ʹ���µ������ʵ��
*/

/**************************************************************************
 *                           ���Ͷ���                                     *
 **************************************************************************/

//1500ϵ��
#define	DVR_1500_4CIF		1	/*! 4· CIFǶ��ʽ */
#define	DVR_1500_8CIF		2	/*! 8· CIFǶ��ʽ */
#define	DVR_1500_12CIF		3	/*! 12· CIFǶ��ʽ */
#define	DVR_1500_16CIF		4	/*! 16· CIFǶ��ʽ */
#define	DVR_1500_4D1		5	/*! 4· D1Ƕ��ʽ */
#define	DVR_1500_8D1		6	/*! 8· D1Ƕ��ʽ */

//8550ϵ��
#define	DVR_8550_4CIF		7	/*! 4· CIFǶ��ʽ */

//1700ϵ��
#define	DVR_1700_8CIF		8	/*! 8· 1700 CIFǶ��ʽ */
#define	DVR_1700_16CIF		9	/*! 16· 1700 CIFǶ��ʽ */
#define	DVR_1700_4D1		10	/*! 4· 1700 D1Ƕ��ʽ */
#define	DVR_1700_8D1		11	/*! 8· 1700 D1Ƕ��ʽ */
#define	DVR_1700_16D1		12	/*! 16· 1700 D1FǶ��ʽ */


/**************************************************************************
 *                           ��������                                     *
 **************************************************************************/
#define DVR_MAX_ID_LEN					(32)
#define DVR_MAX_VIDEOIN_NUM				(16)
#define DVR_MAX_OSD_NUM                 (8)
/** @brief ���PTZ��											                    			 */
#define DVR_MAX_PTZ_NUM					(DVR_MAX_VIDEOIN_NUM)
#define DVR_MAX_VIDEOENC_NUM			(DVR_MAX_VIDEOIN_NUM)
#define MAX_PTZ_SHIELD_NUMBER			(32)  //���ֻ����24 
#define DVR_MAX_ALARMIN_NUM				(16)
#define DVR_MAX_ALARMOUT_NUM			(8)
#define DVR_MAX_EXCEPTION_NUM			(8)
#define DVR_MAX_VIDEOOUT_NUM			(8)

#define DVR_MAX_DAYS					(7)
#define DVR_MAX_TIMESEGMENT				(24 * 60 / 5)	//ÿ5����һ��ʱ���

#define DVR_MAX_IP_STR_LEN				(16)
#define DVR_MAX_MAC_STR_LEN				(18)
#define DVR_MAX_MAC_NUM_LEN				(6)
#define DVR_MAX_DDNSPRO_NUM				(32)
#define DVR_MAX_DDNSPRONAME_LEN			(128)
#define DVR_MAX_DOMAINNAME_LEN 			(256)
#define DVR_MAX_ALARM_AGENT_ID          (32)
#define DVR_MAX_EMAIL_LEN				(256)

#define DVR_MAX_USER_NUM 				(16)
#define DVR_MAX_USERNAME_LEN 			(64)
#define DVR_MAX_PASSWORD_LEN 			(64)
#define DVR_MAX_RIGHT_NUM				(32)

#define DVR_MAX_OWNER_LEN				(128)
#define DVR_MAX_CIVILCODE_LEN			(128)
#define DVR_MAX_ADDRESS_LEN				(128)
#define DVR_MAX_BLOCK_LEN				(128)
/** @brief ��������û��� */
#define DVR_MAX_ONLINEUSER_NUM			(16)

/** @brief IP�������� */
#define DVR_MAX_WHITEIP_NUM				(10)

#define DVR_MAX_PRESET_NUM				(128)
#define DVR_MAX_MOTIONSCOPE_NUM			(32)
#define DVR_MAX_PICSEC_NUM				(24)
#define DVR_MAX_VIDEOSHELTER_NUM		(8)
#define DVR_MAX_FONTNAME_LEN			(32)	//����������
#define DVR_MAX_CHANNELNAME_LEN			(20 + 1)	//����ͨ������󳤶�,��������
#define DVR_MAX_PTZPRONAME_LEN			(48)	//PTZЭ�����Ƴ���
#define DVR_MAX_PTZPRO_NUM				(512)	//PTZЭ�����(��װ��δ��װ��256)
#define DVR_MAX_LATTICE_WIDTH			(24 * 10)	//������
#define DVR_MAX_LATTICE_HEIGHT			(24)	//����߶�
#define DVR_MAX_LATTICE_BUFSIZE			(24 * 24 * 10 / 8)	//���������С

#define DVR_MAX_UERDEF_LATTICE_BUFSIZE  (5 * DVR_MAX_LATTICE_BUFSIZE)
#define DVR_MAX_UERDEF_INFO_BUFSIZE     (6 * DVR_MAX_CHANNELNAME_LEN)

#define DVR_MAX_PARTITION_DEVNODE_LEN	(16)
#define DVR_MAX_PARTITION_IN_HARDDISK	(15)
#define DVR_MAX_PARTITION_IN_DISKGROUP	(32)
#define DVR_MAX_HARDDISK_NUM 			(8)
#define DVR_MAX_DISKGROUP_NUM 			(8)

#define DVR_MAX_VGARESOLVING_NUM		(16)

#define DVR_MAX_VERSION_LEN				(32)

/** @brief Hxht��ͨ�ַ�������																*/
#define DVR_COMMON_STRING_LEN			(32)
#define TOKEN_LENGTH                    (64)


/** @brief ֧�ֵ�Э����������																*/
#define DVR_MAX_SUPPORT_PROTOCOL_NUM	(32)

/** @brief WIFI�ȵ�������󳤶� */
#define DVR_MAX_SSID_LEN				(128)

/** @brief ����ʶ���WIFI�ȵ������� */
#define DVR_MAX_AP_NUM					(10)

/** @brief WIFI��Կ��󳤶� */
#define DVR_MAX_WIFI_KEY_LEN			(128)

/** @brief ���Դ�豸����																			*/
#define MAX_SOURCE_DEVICE_NUMBER		(16)

/** @brief �������豸����																			*/
#define MAX_DEVICE_OUTPUT				(16)

//֧�����VGA����
#define MAX_VGA_NUM						(8)

//TV���
#define MAX_TV_NUM						(8) 

/** @brief ISP ����ģ����� */
#define DVR_MAX_ISP_TEMPLATE		(6)

/** @brief ISP ������AEWindow ���� */
#define DVR_MAX_ISP_AEWINDOW		(6)

/** @brief ISP ������ƽ��ģ����� */
#define DVR_MAX_ISP_WHITEBALANCE	(6)


/** @brief ͼ��ģʽ���� */
#define DVR_MAX_IMAGE_MODE_NUMBER	(16)


#define DVR_MAX_NTP_SVR_LEN          (64)

#define DVR_NUM_abyGammaTbl	(256*3)

/**************************************************************************
*								Э�����ƶ���			                  *
**************************************************************************/
//˽��Э��
#define	PROTOCOL_PRIVATE				0

//�ֻ�Э��
#define	PROTOCOL_TIDEMOBILE		    	1

//���Ż�ͨЭ��
#define	PROTOCOL_HXHT						(1<<1)

//RTSPЭ��
#define	PROTOCOL_RTSP						(1<<2)

//OnvifЭ��
#define	PROTOCOL_ONVIF						(1<<3)

//AlarmMonitorSHЭ��
#define PROTOCOL_ALARMMONITORSH             (1<<4)

//GB28181Э��
#define PROTOCOL_GB28181					(1<<5)

//VSIPЭ��
#define PROTOCOL_VSIP					(1<<6)

//xxxЭ��
//#define PROTOCOL_XXX						(1<<7)

/**************************************************************************
 *                             ������Ϣ���������Ͷ���                     *
 **************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_NETCFG 			101	/*��ȡ�������          */
#define HY_DVR_SET_NETCFG 			102	/*�����������          */
//������
#define NETCFG_ALL			       	0xFFFFFFFF	/*ȫ����������          */
#define NETCFG_DHCP_CONF			1	/*Dhcp������Ϣ          */
#define NETCFG_DHCP_STATE			(1 << 1)	/*Dhcp״̬��Ϣ          */
#define NETCFG_ETH_IF				(1 << 2)	/*Eth������Ϣ           */
#define NETCFG_PPPOE_CONF			(1 << 3)	/*PPPoE����             */
#define NETCFG_PPPOE_IF				(1 << 4)	/*PPPoE������Ϣ         */
#define NETCFG_DNS_CONF				(1 << 5)	/*DNS������Ϣ           */
#define NETCFG_DDNS_CONF			(1 << 6)	/*DDNS������Ϣ          */
#define NETCFG_HTTP_CONF			(1 << 7)	/*http����              */
#define NETCFG_LISTENPORT_CONF		(1 << 8)	/*����˿�������Ϣ      */
#define NETCFG_MOBILE_CONF			(1 << 9)	/*�ֻ��˿�������Ϣ      */
#define NETCFG_AUTODETECT_CONF		(1 << 10)	/*�Զ����ֶ˿�������Ϣ  */
#define NETCFG_WLAN_IF				(1 << 11)	/*WIFI������Ϣ          */
#define NETCFG_WLAN_DHCP_CONF		(1 << 12)	/*WIFI DHCP����         */
#define NETCFG_WLAN_DHCP_STATE		(1 << 13)	/*WIFI Dhcp״̬��Ϣ     */
#define NETCFG_WLAN_DNS_CONF		(1 << 14)	/*WIFI DNS������Ϣ      */
#define NETCFG_WLAN_ACCESSPOINTS	(1 << 15)	/*WIFI �ȵ��б�         */
#define NETCFG_WLAN_CONNECTION		(1 << 16)	/*WIFI ��������         */
#define NETCFG_WLAN_STATE			(1 << 17)	/*WIFI ����״̬         */
#define NETCFG_UPNP_CONF			(1 << 18)	/*UPNP ������Ϣ         */
#define NETCFG_ETH_IF2				(1 << 19)	/*Eth0������Ϣ           */

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_NETSERVERCFG 	103	/*��ȡ���������ò���            */
#define HY_DVR_SET_NETSERVERCFG 	104	/*���÷��������ò���            */
//������
#define NETSERVERCFG_ALL			0xFFFFFFFF	/*ȫ����������          */
#define NETSERVERCFG_CMS_CONF		1	/*CMS������Ϣ               */
#define NETSERVERCFG_CMS_STATE		2	/*CMS״̬��Ϣ               */
#define NETSERVERCFG_AMS_CONF		4	/*AMS������Ϣ               */
#define NETSERVERCFG_NTP_CONF		8	/*NTP������Ϣ               */
#define NETSERVERCFG_EML_CONF		16	/*EML������Ϣ               */
#define NETSERVERCFG_EML_1_CONF		32	/*EML������Ϣ(1.1�汾) */
#define NETSERVERCFG_CMS_PROTOCOL	64	/*CMSЭ����Ϣ */
#define NETSERVERCFG_AMS_SH_CONF    128 /* AMS �Ϻ�������Ϣ         */
#define NETSERVERCFG_AMS_SH_TIME    256 /* AMS �Ϻ��ϴ�ʱ��������Ϣ */



///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_PICCFG 					105	/*��ȡ���Ĺ�����������ò���            */
#define HY_DVR_SET_PICCFG 					106	/*�������Ĺ�����������ò���            */
//������
#define PICCFG_ALL							0xFFFFFFFF	/*ȫ����������                                      */
#define PICCFG_WORKMODE_CONF				1	/*����ģʽ                                              */
#define PICCFG_OSD_CONF						2	/*OSD��Ϣ                                       */
#define PICCFG_CHNAME_CONF					4	/*ͨ��������Ϣ                                      */
#define PICCFG_TIMESEC_CONF					8	/*ʱ�����Ϣ                                        */
#define PICCFG_VIDEOLOST_CONF				16	/*��Ƶ��ʧ������Ϣ                                      */
#define PICCFG_MOTION_CONF					32	/*�ƶ����������Ϣ                                      */
#define PICCFG_MOSAIC_CONF					64	/*������������Ϣ                                        */
#define PICCFG_CHNAME_UNICODE_CONF		    128	/*ͨ��������Ϣ(unicode)                                      */
#define PICCFG_USERDEF_MODE_CONF            512 /*�û��Զ����ع����*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_COMPRESSCFG 					107	/*��ȡ���Ĺ�����������ò���            */
#define HY_DVR_SET_COMPRESSCFG 					108	/*�������Ĺ�����������ò���            */
#define COMPRESSCFG_ROI_CONF 8                      //���ӵ�������

//������
#define COMPRESSCFG_ALL								0xFFFFFFFF	/*ȫ����������                                      */
#define COMPRESSCFG_WORKMODE_CONF				    1	/*����ģʽ������Ϣ                                  */
#define COMPRESSCFG_COMPRESS_CAP					2	/*����������Ϣ                                      */
#define COMPRESSCFG_COMPRESS_CONF					4	/*����������Ϣ                                      */


//added 20141209 JPG����Ϣ��Ҫ��������, added by tupeng, Ϊ�˱����hedandi�ĳ�ͻ,������˴�ı��
#define COMPRESSCFG_JPG								10000 	
typedef struct CGI_JPG_SETTING
{
    int   frame_rate;
    int   frame_number;
    int   nJpgQuality;
}CGI_JPG_SETTING;


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_RECORDCFG 	    109	/*��ȡ����¼����� */
#define HY_DVR_SET_RECORDCFG 	    110	/*���ñ���¼����� */
//������
#define RECORDCFG_ALL				0xFFFFFFFF	/*ȫ��¼������                                      */
#define RECORD_CTRL_STATE			1	/*¼�����״̬����*/
#define RECORD_STREAM				2	/*¼������ѡ�� */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_SYSTIME 					111	/*��ȡϵͳʱ�����          */
#define HY_DVR_SET_SYSTIME 					112	/*����ϵͳʱ�����          */
//������
#define SYSTIME_ALL							0xFFFFFFFF	/*ϵͳʱ������          */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_PTZCFG 					113	/*��ȡ��̨����                  */
#define HY_DVR_SET_PTZCFG 					114	/*������̨����                  */
//������
#define PTZCFG_ALL							0xFFFFFFFF	/*��̨��������                  */
#define PTZCFG_PRESET                       0x3 /*PTZ Ԥ�õ�*/
#define PTZCFG_BALLPARAM                    8

typedef struct tagHY_DVR_PTZ_BALL_PARAM
{
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
    int nType;
    float fFov;
    int nOffsetX;
    int nOffsetY;
    int nMaxZoom;
}HY_DVR_PTZ_BALL_PARAM, *LPHY_DVR_PTZ_BALL_PARAM;


/*PTZ Ԥ�õ�Ľṹ��*/
typedef struct 
{
    int   m_dwChannel; //!<ͨ����
    int   index;       //!<������
    char  token[DVR_MAX_CHANNELNAME_LEN];
    char  name[DVR_MAX_CHANNELNAME_LEN];
} ONVIF_PTZ_SETTING, *LPONVIF_PTZ_SETTING;

typedef struct 
{
    ONVIF_PTZ_SETTING struPtzSetting[DVR_MAX_PRESET_NUM];               
    int     iPresetNum;                                          
} ONVIF_PTZ_ALL_SETTING, *LPONVIF_PTZ_ALL_SETTING;



//20140522 ��ȡPTZ��PZ��Ϣ
#define PTZCFG_ABSCOORD 4
typedef struct 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 
    unsigned long dwPanTiltX; 
    unsigned long dwPanTiltY; 
    unsigned long dwZoomX; 

    //��λ: um
    int nSensorWidth;  //Ŀǰ�̶��⼸��ֵ
    int nSensorHeight; 
    int nSensorMinFocus; 
    int nSensorMaxFocus;

    int nCompassOffset; //��������ƫ�� 

}HY_DVR_PTZ_ABSCOORD_CFG, *LPHY_DVR_PTZ_ABSCOORD_CFG;


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_SERIALCFG 				115	/*��ȡ�������ò���          */
#define HY_DVR_SET_SERIALCFG 				116	/*���ô������ò���          */
//������
#define SERIALCFG_ALL						0xFFFFFFFF	/*ȫ�����ڲ�������          */
#define SERIALCFG_PTZ_ELCTRONIC_CFG			1			/*����PTZ����	          */
#define SERIALCFG_PTZ_REPLENISH_1_CFG		2			/*PTZ��������	          */
#define PTZCFG_SHIELD						3		/** @brief ��̨�ڱ�����(HY_DVR_PTZ_SHIELD_SET)	 */ //20120426
#define PTZCFG_REGION						4		/** @brief �Ŵ�����(HY_DVR_PTZ_REGION_ZOOM) */ //20120426
#define SERIALCFG_PTZ_REPLENISH_2_CFG		5			/*PTZ��������	          */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_ALARMCFG 					117	/*��ȡ�������ò���          */
#define HY_DVR_SET_ALARMCFG 					118	/*���ñ������ò���          */
//������
#define ALARMCFG_ALL									0xFFFFFFFF	/*ȫ��������������          */
#define ALARMCFG_WORKMODE_CONF				1	/*��������ģʽ                  */
#define ALARMCFG_IMGCAPTURE_CONF				2	/*��������                          */
#define ALARMCFG_ALARMIN_CONF					4	/*������������                  */
#define ALARMCFG_ALARMOUT_CONF				8	/*�����������                  */
#define ALARMCFG_EXCEPTION_CONF				16	/*�쳣��������                  */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_VIDEOOUTCFG 					119	/*��ȡ��ʾ������ò���          */
#define HY_DVR_SET_VIDEOOUTCFG 					120	/*������ʾ������ò���          */
//������
#define VIDEOOUTCFG_ALL								0xFFFFFFFF	/*ȫ����ʾ�����������          */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_USERCFG 					121	/*��ȡ�û���Ϣ����          */
#define HY_DVR_SET_USERCFG 					122	/*�����û���Ϣ����          */
//������
#define USERCFG_ALL							0xFFFFFFFF	/*�û���Ϣ����                  */
#define USERCFG_ONE							1	/*�����û���Ϣ                  */
#define USERCFG_CREATE						2	/*����û�                  */
#define USERCFG_REMOVE						4	/*ɾ���û�                  */

#define USERCFG_USER_ONLINE					8	/** @brief �����û�	 */

#define USERCFG_USER_ONLINE_ONE				16	/** @brief ���������û�	 */

#define USERCFG_WHITE_IP_ENABLE				32	/** @brief �Ƿ������û�������	 */

#define USERCFG_WHITE_IP					64	/** @brief �û�������			 */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_DEVICEINFO 			123	/*��ȡ�豸��Ϣ����                  */
#define HY_DVR_SET_DEVICEINFO			124	/*�����豸��Ϣ������ֻ������ */
//������
#define DEVICEINFO_ALL					0xFFFFFFFF	/*ȫ���豸��Ϣ����                  */

#define DEVICEINFO_ONVIF_SPC            0x01
//onvifλ�ú�name��Ϣ
typedef struct  
{
    char szManufacturer[TOKEN_LENGTH];  //IPC
    char szName[DVR_COMMON_STRING_LEN]; //IPCamera
    char szCountry[DVR_COMMON_STRING_LEN]; 
    char szCity[DVR_COMMON_STRING_LEN]; 
}ONVIF_SPC_DEVINFO, *LPONVIF_SPC_DEVINFO;


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_DEVICECFG 		125	/*��ȡ�豸���ò���                  */
#define HY_DVR_SET_DEVICECFG		126	/*�����豸���ò���                  */
//������
#define DEVICECFG_ALL						0xFFFFFFFF	/*ȫ���豸���ò���                  */
#define DEVICECFG_LOCKSCREENTIME_CONF		1	/*����ʱ�����                          */
#define DEVICECFG_LANGUAGE_CONF			2	/*����                                          */
#define DEVICECFG_DATETIME_CONF				4	/*����ʱ���ʽ                          */
#define DEVICECFG_DST_CONF					8	/*��ʱ��                                        */
#define DEVICECFG_REMOTECONTROL_ID			16	/*�豸ID������ң����                */
#define DEVICECFG_TIMEZONE_CONF			32	/*ʱ��                */
#define DEVICECFG_VIDEOSTANDARD_CONF		64	/*��Ƶ��ʽ������Ϣ                                      */

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_STORAGEINFO			127	/*��ȡ�洢ϵͳ��Ϣ                  */
#define HY_DVR_SET_STORAGEINFO			128	/*���ô洢ϵͳ��Ϣ��ֻ������ */
//������
#define STORAGEINFO_ALL					0xFFFFFFFF	/*ȫ���豸��Ϣ����                  */
#define STORAGEINFO_WORKMODE_CONF	1	/*�洢ϵͳ����ģʽ����          */
#define STORAGEINFO_DISK_INFO			2	/*Ӳ����Ϣ����                      */
#define STORAGEINFO_DISKGROUP_INFO	4	/*��������Ϣ����                        */
#define STORAGEINFO_DISK_MARK			8	/*����Ч����                        */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_DEVICESTATE		129	/*��ȡ�豸״̬��Ϣ                  */
#define HY_DVR_SET_DEVICESTATE		130	/*�����豸״̬��Ϣ���ճ����� */
//������
#define DEVICESTATE_ALL				0xFFFFFFFF	/*ȫ���豸״̬��Ϣ                      */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_DEVICEMAINTENANCE	131	/*��ȡ�豸�Զ�ά������                  */
#define HY_DVR_SET_DEVICEMAINTENANCE	132	/*�����豸�Զ�ά������                  */
//������
#define DEVICEMAINTENANCE_ALL			0xFFFFFFFF	/*ȫ���豸�Զ�ά������                  */


///////////////////////////////////////////////////////////////////////////////////////////////////////
#define HY_DVR_GET_DEVICECUSTOM	133	/*��ȡ�豸���ܶ������� */
#define HY_DVR_SET_DEVICECUSTOM	134	/*�����豸���ܶ������� */
//������
#define DEVICECUSTOM_ALL		0xFFFFFFFF	/*ȫ���豸���ܶ������� */
#define DEVICECUSTOM_HCRESMODE_CONF				8	/*HC���ͱ������Դ����ģʽ��Ϣ */

///////////////////////////////////////////////////////////////////////////////////////////////////////
//������
#define HY_DVR_GET_HXHT			135	/*��ȡHXHT���� */
#define HY_DVR_SET_HXHT			136	/*����HXHT���� */
//������
#define HXHT_ALL				0xFFFFFFFF	/* ȫ��HXHT���� */
#define	HXHT_NATSTATUS			1	/* NAT״̬����    */
#define	HXHT_VOICEPARAM			2	/* ������������ */
#define HXHT_CMS_STORAGE		4	/*CMS�洢ע��������Ϣ       */
#define HXHT_DEVICE_ID			8	/*ǰ���豸ID               */

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define	HY_DVR_GET_PROTOCOL		137	/* ��ȡҪ���ص�Э�����ƣ�˽��Э���Ǳ�����صģ��������У� */
#define	HY_DVR_SET_PROTOCOL		138	/* ����Ҫ���ص�Э������ */
//������
#define PROTOCOL_ALL			0xFFFFFFFF	/* ȫ��Ҫ���ص�Э������ */


#define	HY_DVR_GET_SOURCE_DEVICE	139	/* ��ȡԴ�豸���� */
#define	HY_DVR_SET_SOURCE_DEVICE	140	/* ����Դ�豸���� */
//������
#define SOURCE_DEVICE_ALL			0xFFFFFFFF	/* ȫ��Դ�豸���� */

///////////////////////////////////////////////////////////////////////////////////////////////////////
////������
///** @brief ��ȡ�豸���															 */
//#define	HY_DVR_GET_DEVICE_OUTPUT			141
//
///** @brief �����豸���(��Ч���ֻ�ܻ�ȡ����������)							 */
//#define	HY_DVR_SET_DEVICE_OUTPUT			142
//
////������
///** @brief ȫ���豸���(HY_DVR_DEVICE_OUTPUT)									 */
//#define DEVICE_OUTPUT_ALL					0xFFFFFFFF

///////////////////////////////////////////////////////////////////////////////////////////////////////
//������
/** @brief ��ȡ��������ʾ�豸��Ϣ												*/
#define	HY_DVR_GET_DISP_DEV					143

/** @brief ���ý�������ʾ�豸��Ϣ												*/
#define	HY_DVR_SET_DISP_DEV					144

//������
/** @brief ���н�������ʾ�豸��Ϣ(HY_DVR_DISPDEV_CFG)						 */
#define DISP_DEV_ALL						0xFFFFFFFF

/** @brief VGA��Ϣ(HY_DVR_VGAPARA_CFG)										 */
#define DISP_DEV_VGA						1

/** @brief TV ��Ϣ(HY_DVR_VOOUT_CFG)										 */
#define DISP_DEV_TV							2

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define	HY_DVR_GET_NXP8850ISP		145	/* ��ȡNXP8850 ISP���� */
#define	HY_DVR_SET_NXP8850ISP		146	/* ����NXP8850 ISP���� */
//������
#define NXP8850ISP_ALL				0xFFFFFFFF	/* ȫ��ISP���� */


///////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief ��ȡӦ�ò����Բ���													*/
#define	HY_DVR_GET_APP_FEATURE				200
/** @brief ����Ӧ�ò����Բ���													*/
#define	HY_DVR_SET_APP_FEATURE				201
//������
/** @brief ��Чֵ																*/
#define APP_FEATURE_ALL						0xFFFFFFFF

//////////////////////////////////////////////////////////////////////////
//������
/** @brief ��ȡGB28181��Ϣ												*/
#define HY_DVR_GET_GB28181_INFO		203
/** @brief ����GB28181��Ϣ												*/
#define HY_DVR_SET_GB28181_INFO		204
//������
#define GB28181_INFO_ALL			0xFFFFFFFF /* ȫ������ */
///////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
*                         ̽��������Ϣ���������Ͷ���                      *
************************************************************************* */

///////////////////////////////////////////////////////////////////////////////////////////////////////
//������
/** @brief ��ȡ�������												          	 */
#define HY_DVR_PROBE_GET_NETCFG 			17

/** @brief �����������												          	 */
#define HY_DVR_PROBE_SET_NETCFG 			18

//������
/** @brief ������������(HY_DVR_PROBE_NET_CFG)						          	 */
#define PROBE_NETCFG_BASE					1

/** @brief ���������ȵ�����(HY_DVR_PROBE_WLAN_CONNECTION)						 */
#define PROBE_NETCFG_WLAN_CONNECTION		2

/** @brief �ȵ��б�����(HY_DVR_PROBE_WLAN_ACCESSPOINT_CFG)						 */
#define PROBE_NETCFG_WLAN_AP_LIST			3





/** @brief ��ȡ������Բ���													*/
#define	HY_DVR_GET_CCM				205
	/** @brief ����������Բ���													*/
#define	HY_DVR_SET_CCM				206
	//������
	/** @brief 																*/
#define CCM_ALL						0xFFFFFFFF


/** @brief ��ȡ���ͼ��ģʽ													*/
#define	HY_DVR_GET_IMGMODE			207
	/** @brief �������ͼ��ģʽ															*/
#define	HY_DVR_SET_IMGMODE				208
	//������
	/** @brief 																*/
#define IMGMODE_ALL						0xFFFFFFFF

///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************************
 *                        ������Ϣ�ṹ�嶨��                              *
 **************************************************************************/

////////////////////////////////////////////////////////////////////////////

//ʱ��ƻ�, 24Сʱ�ȷֳ�48��ʱ���
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//! һ�������0Ϊ��������1Ϊ����
	unsigned char bySetType[DVR_MAX_DAYS][DVR_MAX_TIMESEGMENT];

	//! ״̬,0Ϊ����(ȫ�첼��),1Ϊ����,2Ϊ��֧��
	unsigned char byState;

	//! ����λ
	unsigned char byReserve[3];

} HY_DVR_SCHEDTIME, *LPHY_DVR_SCHEDTIME;

//����ģʽ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nWorkMode;				//�߼�ģʽ��0Ϊ��ͨģʽ��1Ϊ�߼�ģʽ

} HY_DVR_WORKMODE, *LPHY_DVR_WORKMODE;

/*������Ϣ*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szFontName[DVR_MAX_FONTNAME_LEN];	//������
	int nFontStyle;				//������
	int nFontSize;				//�����С

} HY_DVR_FONT_INFO, *LPHY_DVR_FONT_INFO;

/*��������*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char strName[DVR_MAX_CHANNELNAME_LEN];	//ͨ������(�ַ���)
	int nCharSet;				//ͨ�����ַ��� 0 GB, 1 unicode

	//����� DVR_MAX_LATTICE_WIDTH
	//���߶� DVR_MAX_LATTICE_HEIGHT
	int nWidth;					//ͨ�����������ַ������(������Ϊ��λ)
	int nHeight;				//ͨ�����������ַ����߶�(������Ϊ��λ)

	//���󣬰���ɨ��
	//��Ϊ (nWidth + 7)/8
	//��ΪnHeight
	//Ŀǰ���֧��10��24*24�ĵ���
	unsigned char byLattice[DVR_MAX_LATTICE_BUFSIZE];

	HY_DVR_FONT_INFO struFontInfo;	//ͨ����������Ϣ

} HY_DVR_ALIAS_CFG, *LPHY_DVR_ALIAS_CFG;

/*��������,unicode�汾!!!!!!*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//Ϊunicode��
	unsigned short strName[DVR_MAX_CHANNELNAME_LEN];	//ͨ������(�ַ���)
	int nCharSet; 

	//����� DVR_MAX_LATTICE_WIDTH
	//���߶� DVR_MAX_LATTICE_HEIGHT
	int nWidth;					//ͨ�����������ַ������(������Ϊ��λ)
	int nHeight;				//ͨ�����������ַ����߶�(������Ϊ��λ)

	//���󣬰���ɨ��
	//��Ϊ (nWidth + 7)/8
	//��ΪnHeight
	//Ŀǰ���֧��10��24*24�ĵ���
	unsigned char byLattice[DVR_MAX_LATTICE_BUFSIZE];

	HY_DVR_FONT_INFO struFontInfo;	//ͨ����������Ϣ

} HY_DVR_ALIAS_CFG_1, *LPHY_DVR_ALIAS_CFG_1;

/*!
*  @struct	tagHY_DVR_OSD_USER_DEF_INFO
*  @brief	�û��Զ���OSD��Ϣ
 */
typedef struct tagHY_DVR_OSD_USER_DEF_INFO
{
	int bValid;													/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	char UserDefInfo[DVR_MAX_UERDEF_INFO_BUFSIZE];						/*!< �û��Զ�����Ϣ */
	int nCharSet;												/*!< ͨ�����ַ���: 0-GB, 1-unicode */
	int nWidth;													/*!< ͨ�����������ַ������(������Ϊ��λ)������� DVR_MAX_LATTICE_WIDTH */
	int nHeight;												/*!< ͨ�����������ַ����߶�(������Ϊ��λ)�����߶� DVR_MAX_LATTICE_HEIGHT */
	unsigned char byLattice[DVR_MAX_UERDEF_LATTICE_BUFSIZE];			/*!< ���󣬰���ɨ�裬��Ϊ(nWidth+7)/8����ΪnHeight��Ŀǰ���֧��10��24*24�ĵ��� */
	int nLatticeLen;
    HY_DVR_FONT_INFO struFontInfo;								/*!< ͨ����������Ϣ */

} HY_DVR_OSD_USER_DEF_INFO, *LPHY_DVR_OSD_USER_DEF_INFO;

/*!
*  @struct	tagHY_DVR_OSD_USER_DEF_INFO_1
*  @brief	�û��Զ���OSD��Ϣ(Unicode)
*/
typedef struct tagHY_DVR_OSD_USER_DEF_INFO_1
{
	int bValid;													/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	unsigned short UserDefInfo[DVR_MAX_UERDEF_INFO_BUFSIZE];			/*!< �û��Զ�����Ϣ */
	int nCharSet;												/*!< ͨ�����ַ���: 0-GB, 1-unicode */
	int nWidth;													/*!< ͨ�����������ַ������(������Ϊ��λ)������� DVR_MAX_LATTICE_WIDTH */
	int nHeight;												/*!< ͨ�����������ַ����߶�(������Ϊ��λ)�����߶� DVR_MAX_LATTICE_HEIGHT */
	unsigned char byLattice[DVR_MAX_UERDEF_LATTICE_BUFSIZE];			/*!< ���󣬰���ɨ�裬��Ϊ(nWidth+7)/8����ΪnHeight��Ŀǰ���֧��10��24*24�ĵ��� */
	int nLatticeLen;
    HY_DVR_FONT_INFO struFontInfo;								/*!< ͨ����������Ϣ */

} HY_DVR_OSD_USER_DEF_INFO_1, *LPHY_DVR_OSD_USER_DEF_INFO_1;


////////////////////////////////////////////////////////////////////////////

//DDNSЭ����Ϣ�ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bDdnsEnable;			/* 0-�������Զ�ע��, 1-�����Զ�ע�� */

	/* ��ǰDDNS��״̬ (ֻ��) */
	//0��ʾ�Ѿ�ֹͣ��1��ʾ����������2��ʾ�Ѿ�������3��ʾ����ֹͣ
	int nDdnsState;

	char szProtocolName[DVR_MAX_DDNSPRONAME_LEN];	/*Э������  (ֻ��) */
	char szUserName[DVR_MAX_USERNAME_LEN];	/*ע���û��� */
	char szUserPassword[DVR_MAX_PASSWORD_LEN];	/*ע���û����� */
	char szDomainName[DVR_MAX_DOMAINNAME_LEN];	/*ע���û����� */

	unsigned short wRegisterPort;	/*����˿� */

	int nAutoRegIntervals;		/*�Զ�ע����ʱ�䣬��λΪ�� */

} HY_DVR_DDNS, *LPHY_DVR_DDNS;

//DDNS�������ýṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDdnsProtocolCount;		//֧�ֵ�DDNSЭ������(ֻ��)

	HY_DVR_DDNS struDdnsProtocol[DVR_MAX_DDNSPRO_NUM];

} HY_DVR_DDNS_CONF, *LPHY_DVR_DDNS_CONF;

//PPPoE����������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bPPPoEEnable;			/* 0-������PPPoE,1-���� PPPoE */

	//PPPoE�����Ϣ
	char szPPPoEUser[DVR_MAX_USERNAME_LEN];	/* PPPoE�û��� */
	char szPPPoEPassword[DVR_MAX_PASSWORD_LEN];	/* PPPoE���� */

} HY_DVR_PPPOE_CONF, *LPHY_DVR_PPPOE_CONF;

//PPPoE������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	/*PPPoE ����״̬(ֻ��) */
	//0��ʾû�����ӣ�1��ʾ�������ӣ�2��ʾ���ӳɹ���3��ʾ���ڹҶ�
	int nPPPoEState;

	char szPPPoEIP[DVR_MAX_IP_STR_LEN];	/*PPPoE IP��ַ(ֻ��) */

	char szPPPoESubnetMask[DVR_MAX_IP_STR_LEN];	/*PPPoE ��������(ֻ��) */

	char szPPPoEGateway[DVR_MAX_IP_STR_LEN];	/*PPPoE ����(ֻ��) */

} HY_DVR_PPPOE_IF, *LPHY_DVR_PPPOE_IF;

//DHCP������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bDhcpEnable;			/* 0-������Dhcp,1-����Dhcp */

} HY_DVR_DHCP_CONF, *LPHY_DVR_DHCP_CONF;

//DHCP״̬��Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDhcpState;				/*0-δ����,1-��ȡ�ɹ�,2-���ڻ�ȡ,3-��ȡʧ��,������ȡ*/

} HY_DVR_DHCP_STATE, *LPHY_DVR_DHCP_STATE;

//DNS������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szPrimaryDNS[DVR_MAX_IP_STR_LEN];	/*��DNS������ */
	char szSecondaryDNS[DVR_MAX_IP_STR_LEN];	/*����DNS */

} HY_DVR_DNS_CONF, *LPHY_DVR_DNS_CONF;

//http������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				//���û����http����

	unsigned short wHttpPort;	/* Http�˿� */

} HY_DVR_HTTP_CONF, *LPHY_DVR_HTTP_CONF;

/*!
*  @struct	tagHY_DVR_LISTENPORT_CONF
*  @brief	��������˿���Ϣ
 */
typedef struct tagHY_DVR_LISTENPORT_CONF
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned short wLocalCmdPort;	/*��������˿� */
	unsigned short wLocalMediaPort;	/*����ý��˿� */

	unsigned short wRtpPort;	/*����rtp�˿� */

} HY_DVR_LISTENPORT_CONF, *LPHY_DVR_LISTENPORT_CONF;

#define LISTENPORT_CONF_LEN		sizeof(HY_DVR_LISTENPORT_CONF)

//�˳��ֻ��ͻ�������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned short wMobilePort;	/*�ֻ��˿� */

} HY_DVR_TIDE_MOBILE, *LPHY_DVR_TIDE_MOBILE;

//�ֻ������˿���Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_TIDE_MOBILE struTide;	/*�˳��ֻ����� */

} HY_DVR_MOBILE_CONF, *LPHY_DVR_MOBILE_CONF;

#define MOBILE_CONF_LEN		sizeof(HY_DVR_MOBILE_CONF)


//ETH0���ýṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//����������������Ϣ
	char szIPAddress[DVR_MAX_IP_STR_LEN];	/* DVR IP��ַ */
	char szSubnetMask[DVR_MAX_IP_STR_LEN];	/* DVR �������� */
	char szGateway[DVR_MAX_IP_STR_LEN];	/* ���ص�ַ */
	char szMacAddress[DVR_MAX_MAC_STR_LEN];	/* ֻ�����������������ַ */

} HY_DVR_ETH_IF, *LPHY_DVR_ETH_IF;

//�Զ����ֶ˿���Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned short wAutoDetectPort;	/*�Զ����ֶ˿� */

} HY_DVR_AUTODETECT_CONF, *LPHY_DVR_AUTODETECT_CONF;

#define AUTODETECT_CONF_LEN		sizeof(HY_DVR_AUTODETECT_CONF)

typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//!\brief WIFI״̬ 0 ������ 1 ����, 2 ����δ�ҵ�
	int bEnable; 

} HY_DVR_WLAN_STATE, *LPHY_DVR_WLAN_STATE;

typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	// !\brief �ȵ�����
	char szSSID[DVR_MAX_SSID_LEN];

	//!\brief �ȵ�MAC��ַ
	char szMacAddress[DVR_MAX_MAC_STR_LEN];

	//!�ȵ��ź�ǿ��
	int nSignal;

	//!0-Open,1-Shared,2-WPAPSK, 3-WPA2PSK, 4-WPANONE(ad-hocģʽ),5-WPA,6-WPA2
	int nAuthentication;

	//! 0��ʾNONE 1��ʾWEP 2��ʾTKIP 3��ʾAES
	int nEncrypType;

	//!\brief �ȵ�֧�ֵĹ淶0��ʾ802.11b,1��ʾ802.11b/g,2��ʾ802.11b/g/n
	// 1: legacy 11B only
	// 2: legacy 11A only
	// 3: legacy 11a/b/g mixed
	// 4: legacy 11G only
	// 5: 11ABGN mixed
	// 6: 11N only
	// 7: 11GN mixed
	// 8: 11AN mixed
	// 9: 11BGN mixed
	int nWirelessMode;
} HY_DVR_WLAN_ACCESSPOINT, *LPHY_DVR_WLAN_ACCESSPOINT;

/*!
*  @struct	tagHY_DVR_WLAN_ACCESSPOINT_CFG
*  @brief	WIFI�ȵ�����
*/
typedef struct tagHY_DVR_WLAN_ACCESSPOINT_CFG
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WLAN_ACCESSPOINT struAccessPoints[DVR_MAX_AP_NUM];
} HY_DVR_WLAN_ACCESSPOINT_CFG, *LPHY_DVR_WLAN_ACCESSPOINT_CFG;

typedef struct 
{
	//���ṹ���Ƿ���Ч��=0��ʾ��Ч
	int bValid;

	// ����
	char szKey[DVR_MAX_WIFI_KEY_LEN];

	// ��Կ���� 0-��ʾ16���� 1��ʾ�ַ�������
	int nKeyType;
} HY_DVR_WLAN_KEY, *LPHY_DVR_WLAN_KEY;

typedef struct 
{
	//���ṹ���Ƿ���Ч��=0��ʾ��Ч
	int bValid;

	// WEP��֤��ǰ�õ�KEY������,1~4
	int nDefKeyID;

	// WEP KEY1
	HY_DVR_WLAN_KEY Key1;

	// WEP KEY2
	HY_DVR_WLAN_KEY Key2;

	// WEP KEY3
	HY_DVR_WLAN_KEY Key3;

	// WEP KEY4
	HY_DVR_WLAN_KEY Key4;

	// WPA KEY
	HY_DVR_WLAN_KEY WPAKey;
} HY_DVR_WLAN_KEY_CFG, *LPHY_DVR_WLAN_KEY_CFG;

/*!
*  @struct	tagHY_DVR_WLAN_CONNECTION
*  @brief	WIFI��������
*/
typedef struct tagHY_DVR_WLAN_CONNECTION
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//!\brief �Ƿ������� 0��ʾδ������ 1��ʾ������(ֻ��) 2��ʾ��������
	int bConnect;

	// !\brief �ȵ�����
	char szSSID[DVR_MAX_SSID_LEN];

	//!\brief �ȵ�MAC��ַ
	char szMacAddress[DVR_MAX_MAC_STR_LEN];

	// ��������
	// 0��ʾManaged, 1��ʾAd-hoc
	int nNetworkType;
	
	// ��֤ģʽ
	// 0-Open,1-Shared,2-WPAPSK, 3-WPA2PSK, 4-WPANONE(ad-hocģʽ),5-WPA,6-WPA2
	int nAuthMode;

	// �����㷨 0-NONE,1-WEP,2-TKIP,3-AES
	int nEncrypType;

	// ��������
	HY_DVR_WLAN_KEY_CFG struKeyCfg;
} HY_DVR_WLAN_CONNECTION, *LPHY_DVR_WLAN_CONNECTION;

/*!
*  @struct	tagHY_DVR_UPNP_CONF
*  @brief	UPNP����
*/
typedef struct
{
	int bValid;														/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч	*/

	int bEnable;													/*!< UPNP���� 0 ������ 1 ����		*/
	char szWanAddress[DVR_MAX_IP_STR_LEN];							/*!< ���ڵ�ַ(ֻ��)					*/
	int nLocalHttpPort;												/*!< ����HTTP�˿�					*/
	int nExportHttpPort;											/*!< ����HTTP�˿�					*/
	int nHttpPortState;												/*!< HTTP״̬ 0 �ɹ� 1 ʧ�� 2 ӳ����*/
	int nLocalMobilePort;											/*!< �����ֻ��˿�					*/
	int nExportMobilePort;											/*!< �����ֻ��˿�					*/
	int nMobilePortState;											/*!< �ֻ�״̬ 0 �ɹ� 1 ʧ�� 2 ӳ����*/
	int nLocalCommandPort;											/*!< ��������˿�					*/
	int nExportCommandPort;											/*!< ��������˿�					*/
	int nCommandPortState;											/*!< ����״̬ 0 �ɹ� 1 ʧ�� 2 ӳ����*/
	int nLocalMediaPort;											/*!< ����ý��˿�					*/
	int nExportMediaPort;											/*!< ����ý��˿�					*/
	int nMediaPortState;											/*!< ý��״̬ 0 �ɹ� 1 ʧ�� 2 ӳ����*/
	int nReserve[8];												/*!< ����							*/
} HY_DVR_UPNP_CONF, *LPHY_DVR_UPNP_CONF;

#define NETCFG_P2P_CONF (1 << 20)
typedef struct  
{
    int bValid;
    int bEnable;//0 ���� 1 ����
    int nReserve[8];//����
}HY_DVR_P2P_CONF, *LPHY_DVR_P2P_CONF;
////////////////////////////////////////////////////////////////////////////
//�����������������ýṹ
typedef struct
{
    int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

    HY_DVR_DHCP_CONF struDhcp_Conf;	//Dhcp������Ϣ
    HY_DVR_DHCP_STATE struDhcp_State;	//Dhcp״̬��Ϣ

    HY_DVR_ETH_IF struEth;		//Eth������Ϣ

    HY_DVR_PPPOE_CONF struPPPoE_Conf;	//PPPoE����
    HY_DVR_PPPOE_IF struPPPoE_IF;	//PPPoE������Ϣ

    HY_DVR_DNS_CONF struDNS;	//DNS������Ϣ

    HY_DVR_DDNS_CONF struDDNS;	//DDNS������Ϣ

    HY_DVR_HTTP_CONF struHttp;	//http����

    HY_DVR_LISTENPORT_CONF struListenPort;	//����˿�������Ϣ

    HY_DVR_MOBILE_CONF struMobilePort;	//�ֻ��˿�������Ϣ

    HY_DVR_AUTODETECT_CONF struAutoDetectPort;	//�Զ����ֶ˿�����

    ///////////////////////////////////////////////////////////////////////////////////////////////
    HY_DVR_DHCP_CONF struWIFIDhcp_Conf; //WLAN DHCP
    HY_DVR_DHCP_STATE struWIFIDhcp_State;						/*!< WIFI Dhcp״̬��Ϣ */
    HY_DVR_ETH_IF struWIFICommon; //WIFI ������Ϣ
    HY_DVR_DNS_CONF struWIFIDNS;								/*!< WIFI DNS������Ϣ */
    HY_DVR_WLAN_ACCESSPOINT_CFG struWIFIAccessPoint;/*!< WIFI�ȵ����� */

    HY_DVR_WLAN_CONNECTION struWIFIConnectCfg;	/*!< WIFI�������� */

    HY_DVR_WLAN_STATE struWIFIState;	/*!< WIFI ״̬ */

    HY_DVR_UPNP_CONF	struUPNPConf;			/*!< UPNP������Ϣ				*/
} HY_DVR_NET_CFG, *LPHY_DVR_NET_CFG;

#define NET_CFG_LEN   sizeof (HY_DVR_NET_CFG)
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////

//���Ĺ��������������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bCmsEnable;				/*0-�����ù��������, 1-���� */

	char szManageHostAddress[DVR_MAX_DOMAINNAME_LEN];	/*Զ�̹���������ַ�����ȿ������� */

	unsigned short wManageHostPort;	/*Զ�̹��������˿� */

	int nAutoRegIntervals;		/*ע����ʱ�䣬��λΪ�� */

} HY_DVR_NET_CMS, *LPHY_DVR_NET_CMS;

//���Ĺ������������״̬
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nCmsState;				/*CMSע��״̬,Ŀǰϵͳ��֧�� */

} HY_DVR_CMS_STATE, *LPHY_DVR_CMS_STATE;

//�������������������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bAmsEnable;				/*0-�����ñ���������, 1-���� */

	char szAlarmHostAddress[DVR_MAX_DOMAINNAME_LEN];	/*���������������ȿ������� */

	unsigned short wAlarmHostPort;	/*�����������˿� */

} HY_DVR_NET_AMS, *LPHY_DVR_NET_AMS;


//�Ϻ��ı������������������Ϣ
typedef struct  
{
	int bValid;              //!���ṹ���Ƿ���Ч�� =0��ʾ��Ч

	int bAmsEnable;         /*0-�����ñ����������� 1-����*/

	char szAlarmAgentID[DVR_MAX_ALARM_AGENT_ID];    /*����ID */

	char szAlarmHostAddress[DVR_MAX_DOMAINNAME_LEN];	/*���������������ȿ������� */

	unsigned short wAlarmHostPort;	/*�����������˿� */

}HY_DVR_NET_AMS_SH, *LPHY_DVR_NET_AMS_SH;

//�Ϻ��ı������������ʱ��������Ϣ

typedef struct  
{
	int bValid;              //!���ṹ���Ƿ���Ч�� =0��ʾ��Ч
	int iPrePicTime;          /* Ԥ¼ʱ��*/
	int iContinualUploadTime; /*�����ϴ�ʱ��*/
	int bAlarmUploadPicEnable[DVR_MAX_ALARMIN_NUM];/*�Ƿ�������ͼƬ�ϴ�*/
}HY_DVR_NET_AMS_SH_TIME, *LPHY_DVR_NET_AMS_SH_TIME;


//Ntp������������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bNtpEnable;				/*0-������, 1-���� */

	char szNtpHostAddress[DVR_MAX_DOMAINNAME_LEN];	/*Ntp�����������ȿ������� */
	unsigned short wNtpHostPort;	/*Ntp���������˿� */

	int nAutoRegIntervals;		/*Ntp������ͬ�����ʱ�䣬��λΪ�� */

} HY_DVR_NET_NTP, *LPHY_DVR_NET_NTP;

//Email������������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEmlEnable;				/*0-������, 1-���� */

	char szSmtpServer[DVR_MAX_DOMAINNAME_LEN];	/*smtp��������ַ */
	unsigned short wSmtpPort;	/*�˿� */

	char szSenderAddress[DVR_MAX_EMAIL_LEN];	/*�����������ַ */

	char szSmtpUserName[DVR_MAX_USERNAME_LEN];	/*smtp�������ʺ�(��Ч����) */
	char szSmtpPassword[DVR_MAX_PASSWORD_LEN];	/*smtp���������� (��������������) */

} HY_DVR_NET_EML, *LPHY_DVR_NET_EML;

//Email������������Ϣ(1.1.0�汾ʹ��)
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEmlEnable;				/*0-������, 1-���� */

	char szSmtpServer[DVR_MAX_DOMAINNAME_LEN];	/*smtp��������ַ(����Ϊ������IP��ַ) */
	unsigned short wSmtpPort;	/*�˿� */
	int nSmtpAuth;				/*��֤��ʽ(0ΪNONE, 1ΪLOGIN, 2ΪPLAIN) */
	int nSmtpTLS;				/*�Ƿ�ʹ�ð�ȫ��½(0Ϊoff, 1Ϊon) */

	char szSenderEmail[DVR_MAX_EMAIL_LEN];	/*�����������ַ */
	char szSenderPassword[DVR_MAX_PASSWORD_LEN];	/*�������������� */

	//���²����ݲ�ʹ��
	int nProtocol;				//(ĿǰΪsmtp)
	int nTimeout;				//��ʱ����
	int nAuto_from;

	//����ΪTLS��ظ߼��������ݲ�ʹ��
	int nTls_starttls;
	int nTls_certcheck;
	int nTls_force_sslv3;
	int nTls_min_dh_prime_bits;
	int nTls_priorities;

} HY_DVR_NET_EML_1, *LPHY_DVR_NET_EML_1;

//////////////////////////////////////////////////////////////////////////////
//Э������
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned long dwProtocolName;	/*!< Э�����ƣ�Э���ǰ�λ���úͶ�ȡ�ģ�����ֻ���ֻ�Э��ʱ����ֵΪ1��
									   ֻ�л��Ż�ͨЭ��ʱ����ֵΪ2������Э�鶼��ʱ����ֵΪ3��
									   ��˽��Э���Ǳ�����صģ��������У� */
} HY_DVR_PROTOCOL_NAME, *LPHY_DVR_PROTOCOL_NAME;

//CMS��Ӧ��Э������
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned long dwCMSProtocol;	/*!< Э�����ƣ���CMS����Ҫ��дIP��ַ����Ҫ�����豸�ˣ����
									   CMS�е�IP��ַ��Ϊ��һ��Э��׼���ģ������˽��Э�飬��ô�豸
									   ��ע�ᵽNVMS�У������HxhtЭ�飬���豸ע�ᵽHxht�Ľ���������С�
									   �ò���ֻ�ܱ�ʾһ��Э�顣 */
} HY_DVR_CMS_FOR_PROTOCOL, *LPHY_DVR_CMS_FOR_PROTOCOL;

//////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//�������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_NET_CMS struCMS;		//���Ĺ����������Ϣ    
	HY_DVR_NET_AMS struAMS;		//������������Ϣ    
	HY_DVR_NET_NTP struNTP;		//NTP��������Ϣ 
	HY_DVR_NET_EML struEML;		//Email��������Ϣ  

	HY_DVR_CMS_FOR_PROTOCOL struCMSForProtocol;	//���Ĺ����������Ϣ��Ӧ��Э�飨��CMS�ǰ󶨵ģ�

	HY_DVR_CMS_STATE struCmsState;	//���Ĺ������������״̬

	HY_DVR_NET_EML_1 struEML_1;	//Email��������Ϣ  1.1��

	HY_DVR_NET_AMS_SH struAMSSH;   //�Ϻ�����������������Ϣ

} HY_DVR_NET_MANAGER, *LPHY_DVR_NET_MANAGER;

#define NET_MANAGER_LEN   sizeof (HY_DVR_NET_MANAGER)
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////

//ͼ��������ýṹ(ÿͨ��֧��24��ʱ���)
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bTimeSegEnable;			/*����ʱ��Ƭ */

	int nStartTime;				/*��ʼʱ�� ���00:00ʱ������  */
	int nStopTime;				/*����ʱ�� ���00:00ʱ������  */

	int nBrightness;			/*����,0-255 */
	int nContrast;				/*�Աȶ�,0-255 */
	int nSaturation;			/*���Ͷ�,0-255 */
	int nHue;					/*ɫ��,0-255 */

} HY_DVR_CHROMA_SEC, *LPHY_DVR_CHROMA_SEC;

//ͼ��������ýṹ(ÿͨ��֧��24��ʱ���)
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//ɫ�Ȳ���
	HY_DVR_CHROMA_SEC struChromaSec[DVR_MAX_PICSEC_NUM];

} HY_DVR_CHROMA, *LPHY_DVR_CHROMA;


/*�쳣����ʽ*/
#define DVR_ALARM_TRIGGER_NONE		0	/*����Ӧ */
#define DVR_ALARM_TRIGGER_PREVIEW	(0x1)	/*������Ļ��ʾ��ͨ��������¼��������� */
#define DVR_ALARM_TRIGGER_AUDIOOUT	(0x1 << 1)	/*�������� */
#define DVR_ALARM_TRIGGER_RECORD	(0x1 << 2)	/*����¼�� */
#define DVR_ALARM_TRIGGER_SNAPSHOT	(0x1 << 3)	/*�������� */
#define DVR_ALARM_TRIGGER_ALARMOUT	(0x1 << 4)	/*����������� */
#define DVR_ALARM_TRIGGER_NOTIFYAMS	(0x1 << 5)	/*�ϴ��������� */
#define DVR_ALARM_TRIGGER_PTZ		(0x1 << 6)	/*����PTZ */
#define DVR_ALARM_TRIGGER_EMAIL		(0x1 << 7)	/*����EMail֪ͨ */
#define DVR_ALARM_TRIGGER_NOTIFYGUI	(0x1 << 8)	/*��Ļ��ʾ */
#define DVR_ALARM_TRIGGER_LIGHT  	(0x1 << 9)	/*�������� */
#define DVR_ALARM_TRIGGER_V12  	    (0x1 << 0xa)	/*�������� */

/*��������ṹ��*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned long dwHandleType;	/*����ʽ,����ʽ��"��"��� */

	int nRecordChannelMask;		//����¼��ͨ����ʶ����λ��ʾ
	int nSnapshotChannelMask;	//��������ͨ����ʶ����λ��ʾ
	int nAlarmOutChannelMask;	//�����������ͨ����ʶ����λ��ʾ

	//�����������ͨ��
	int nPtzAction[DVR_MAX_PTZ_NUM];	//nPtzAction �����Ķ���,  0��ʾ�޶��� , 1����Ԥ�õ�, 2����Ѳ�� 3�����켣
	int nPtzIndex[DVR_MAX_PTZ_NUM];	//nPtzIndex  ������Ԥ�õ�/Ѳ��/�켣�ı��

	int nMonitorMask;			//���ԡ����ԡ�����+����,spot

} HY_DVR_HANDLEEXCEPTION, *LPHY_DVR_HANDLEEXCEPTION;


//�ƶ����
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				/* �Ƿ����ƶ���� */

	int nSenstive;				/*�ƶ����������, 0 - 5,Խ��Խ����,0xff�ر� */

	int nXScope;
	int nYScope;				/*X,Y�����鷶Χ */

	/*�������,����32*32��С���,Ϊ1��ʾ�ĺ�����ƶ��������,0-��ʾ���� */
	unsigned char byMotionScope[DVR_MAX_MOTIONSCOPE_NUM][DVR_MAX_MOTIONSCOPE_NUM];

	HY_DVR_HANDLEEXCEPTION struHandleType;	/* ����ʽ */

	HY_DVR_SCHEDTIME struSchedTime;	/* 0��ʾ��������1��ʾ���� */

} HY_DVR_MOTION, *LPHY_DVR_MOTION;


//�źŶ�ʧ���� 
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				/* �Ƿ����źŶ�ʧ���� */

	HY_DVR_HANDLEEXCEPTION struHandleType;	/* ����ʽ */

	HY_DVR_SCHEDTIME struSchedTime;	/* 0��ʾ��������1��ʾ���� */

} HY_DVR_VILOST, *LPHY_DVR_VILOST;


//�ڵ����� �����С704*576 
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				/* �Ƿ����� */

	//����������������ʵ����������
	int nHideAreaTopLeftX;		/* �����������x������� */
	int nHideAreaTopLeftY;		/* �����������y������� */
	int nHideAreaBottomRightX;	/* �����������x������� */
	int nHideAreaBottomRightY;	/* �����������y������� */

} HY_DVR_MOSAIC_INFO, *LPHY_DVR_MOSAIC_INFO;

//�ڵ����� �����С704*576 
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//������
	int bEnableMosaic;			/* �Ƿ������ڵ� ,0-��,1-�� ����Ϊ704*576 */

	HY_DVR_MOSAIC_INFO struMosaicInfo[DVR_MAX_VIDEOSHELTER_NUM];

} HY_DVR_MOSAIC, *LPHY_DVR_MOSAIC;

typedef struct tagHY_DVR_OSD_ADD_INFO
{
    int bValid;													/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

    int nAddInfoLeftX;											/*!< ������Ϣ��x���꣨0-703��  */
    int nAddInfoLeftY;											/*!< ������Ϣ��y���꣨0-575��  */

} HY_DVR_OSD_ADD_INFO, *LPHY_DVR_OSD_ADD_INFO;

//OSD������Ϣ
typedef struct tagHY_DVR_OSD_CFG
{
    int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

    int nShowOsd;				/* 0Ϊͨ��ʱ�������ʾ��1Ϊ��ʾͨ����2��Ϊ��ʾʱ�䡢3Ϊ���߾���ʾ */

    int nTimeTopLeftX;			/* ʱ���x���� */
    int nTimeTopLeftY;			/* ʱ���y���� */

    int nNameTopLeftX;			/* ���Ƶ�x���� */
    int nNameTopLeftY;			/* ���Ƶ�y���� */
    HY_DVR_OSD_ADD_INFO struAddInfo;

} HY_DVR_OSD_CFG, *LPHY_DVR_OSD_CFG;

/////////////////////////////////////////////////////////////////////////////////////////////////
//ͼ������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struWorkMode;	//����ģʽ(ֻ��)

	//��ʾOSD
	HY_DVR_OSD_CFG struOsd[DVR_MAX_VIDEOIN_NUM];

	//ͨ����
	HY_DVR_ALIAS_CFG struChName[DVR_MAX_VIDEOIN_NUM];

	HY_DVR_OSD_USER_DEF_INFO struOSDUserDef[DVR_MAX_VIDEOIN_NUM];			/*!< ͨ���������� */

	//ͼ���������24��ʱ���
	HY_DVR_CHROMA struChroma[DVR_MAX_VIDEOIN_NUM];

	//�źŶ�ʧ���� 
	HY_DVR_VILOST struViLost[DVR_MAX_VIDEOIN_NUM];

	//�ƶ���� 
	HY_DVR_MOTION struMotion[DVR_MAX_VIDEOIN_NUM];

	//������
	HY_DVR_MOSAIC struMosaic[DVR_MAX_VIDEOIN_NUM];

	//ͨ������unicode��
	HY_DVR_ALIAS_CFG_1 struChName_1[DVR_MAX_VIDEOIN_NUM];

	HY_DVR_OSD_USER_DEF_INFO_1 struOSDUserDef_1[DVR_MAX_VIDEOIN_NUM];		/*!< ͨ����������(Unicode) */

} HY_DVR_PIC_CFG, *LPHY_DVR_PIC_CFG;


/////////////////////////////////////////////////////////////////////////////////////////////////
//2013.10.26 ����ͨ��OSD�ṹ��Ϣ
typedef struct tagHY_DVR_OSD_USER_DEF_ALL_CFG
{
    int bValid; 
    HY_DVR_OSD_ADD_INFO      struAddInfo[DVR_MAX_OSD_NUM];
    HY_DVR_OSD_USER_DEF_INFO struOSDUserDef[DVR_MAX_OSD_NUM];
    HY_DVR_OSD_USER_DEF_INFO_1 struOSDUserDef_1[DVR_MAX_OSD_NUM];
}HY_DVR_OSD_USER_DEF_ALL_CFG, *LPHY_DVR_OSD_USER_DEF_ALL_CFG;

#define PIC_CFG_LEN				sizeof (HY_DVR_PIC_CFG)

/////////////////////////////////////////////////////////////////////////////////////////////////
//20140311 �����û��Զ���������Ϣ
typedef struct tagHY_DVR_USERDEF_MODE
{
    int bValid;
    int nWhiteBalance;    /*(�Զ�/Auto, ��/Simple, �ֶ�/Manual), �ֶ�ģʽ�¿��� ���� ��ɫ����Rgain / ��ɫ����Bgain ��Χ0-100*/ 
    int nRedGain;
    int nBlueGain;        /*������*/
    int nDeNoiseLv2D;     /*2D����ȼ� 0-100 */
    int nDeNoiseLv3D;     /*3D����ȼ� 0-100*/ 
    int nSharpness;       /*���SHARPNESS (0-100)*/
    int nMaxGain;         /*����GAIN (0-100)*/
    int nMaxShutter;      /* �ع�ʱ�� (1/2,1/4,1/12.5,1/25,1/50,1/100,1/250,1/500,1/750,1/1000,1/2000,1/4000,1/10000,1/100000) 50Hz 
                             (1/3,1/6,1/15, 1/30,1/60,1/120,1/250,1/500,1/750,1/1000,1/2000,1/4000,1/10000,1/100000) 60Hz */
    int nExposalMode;     /*�ع�ģʽ (�Զ�,���ⲹ��,�̶�����)*/
    int nWideDynLv;       /*��̬WDR (��/��), �ȼ� 0-100*/ 
}HY_DVR_USERDEF_MODE, *LPHY_DVR_USERDEF_MODE;
typedef struct tag_HY_DVR_USERDEF_MODE_CFG
{
    int bValid;
    HY_DVR_USERDEF_MODE struUserDefMode[DVR_MAX_VIDEOIN_NUM];
 
}HY_DVR_USERDEF_MODE_CFG, *LPHY_DVR_USERDEF_MODE_CFG;
//////////////////////////////////////////////////////////////////////////

typedef struct tagHY_DVR_ENABLE_EPTZ
{
	int bValid;													/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	unsigned long dwEnable ;												/*!< 0 ������ 1 ���� */

} HY_DVR_ENABLE_EPTZ, *LPHY_DVR_ENABLE_EPTZ;
/* NXP8850 Image Sensor Processing������Ϣ: ���ֽ��� ��De-Noise��*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	/*
	   ����ģ������ :
	   ���壺 ͨ�����������Ĳ�ͬ���в�ͬ�����ű��ĵ���
	   0    ��ͨ 
	   1    ���� 
	   2 ���ڵ����� 
	   3 ����
	   4 �����
	   5 �Զ���
	   6��7 ����
	 */
	int nWorkMode;

	/*
	   ͼ����ɫ ��Video Color��
	   0    ��ɫ
	   1    �ڰ�
	   2    �Զ�
	   3    �ⲿ����
	 */
	int nVideoColor;

	/*
	   ͼ���� (Video orientation)
	   0    ����
	   1    ��ת      
	   2    ����
	   3    ��ת+ ����
	 */
	int nVideoOrientation;

	/*
	   ǰ������Ƶ��
	   0    60Hz
	   1    50Hz
	 */
	int nInputFrequency;

	HY_DVR_ENABLE_EPTZ struEnableEPTZ;		/*!�Ƿ����õ���ptz */

} HY_DVR_ISP_COMMON, *LPHY_DVR_ISP_COMMON;

/* NXP8850 Image Sensor Processing������Ϣ: AE Mode ����*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nMode;					//AEģʽ��0-�Զ� 1-���� 2-�ֶ�

}HY_DVR_AE_MODE, *LPHY_DVR_AE_MODE;

/* NXP8850 Image Sensor Processing������Ϣ: AE Window ����*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//����(on /off)
	int bOpen;

	//X (0-4095) ��Ҫʶ�����Ŀ�ȣ���������720p,����1280
	int nX;

	//Y (0-4095) ��Ҫʶ�������Ч���ظ߶�
	int nY;

	//width (1-4095) ��Ҫʶ�����Ŀ��
	int nWidth;

	//height(1-4095) ��Ҫʶ�������Ч���ظ߶�
	int nHeight;

	//Weighting (1X �� 8X��ѡ) ����0-7
	int nWeighting;
	
} HY_DVR_AEWINDOW_CFG, *LPHY_DVR_AEWINDOW_CFG;

typedef struct tagHY_DVR_EXPOSAL_LENS_LEVEL
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	int nLensLevel;   //-5~+5

}HY_DVR_EXPOSAL_LENS_LEVEL,*LPHY_DVR_EXPOSAL_LENS_LEVEL;


typedef struct tagHY_DVR_EXPOSAL_CFG_MODETIME 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 
    int nStartTime; /*!< ��ʼʱ�� ���00:00ʱ������ */ 
    int nStopTime; /*!< ����ʱ�� ���00:00ʱ������ */ 
    int nMode; //���¼�����ʹ�õ�ģʽ
}HY_DVR_EXPOSAL_CFG_MODETIME, *LPHY_DVR_EXPOSAL_CFG_MODETIME; 

typedef struct tagHY_DVR_EXPOSAL_CFG_MODETIMEALL 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 
    int bTimeModeEnable; /*!< ����ʱ��Ƭ */ 
    HY_DVR_EXPOSAL_CFG_MODETIME struModeTime[4]; 
}HY_DVR_EXPOSAL_CFG_MODETIMEALL, *LPHY_DVR_EXPOSAL_CFG_MODETIMEALL; 


/* NXP8850 Image Sensor Processing������Ϣ: �ع����*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//��ͷ��Ȧ��(Lens Type)     0Ϊ �����Զ���Ȧ  1-256Ϊ �ֶ���Ȧ��Ӧ(0-255)
	int bLensType;
	HY_DVR_EXPOSAL_LENS_LEVEL strLensLevel;
	/*
		Ŀ������  (Target Lumiance)	0-255
	*/
	int nTargetLumiance;

	/*
		ƫ�� (Offset)	0-255
	*/
	int nOffset;

	/*
		�����ٶ�  (Shutter Speed)
		��С�ٶȣ�0��1/2
		����ٶȣ�1/120��1
		�ٶȿ�ѡ
		0,1/120,1/60,1/30,1/15,1/8,1/4,1/2,1 
	*/
	int nMinShutterSpeed;
	int nMaxShutterSpeed;

	/*
		����ϵ�� (Gain)
		��С���� 1x ~ 16x 
		������� 2x ~ 32x
	*/
	int nMinAutoGain;
	int nMaxAutoGain;


    HY_DVR_EXPOSAL_CFG_MODETIMEALL struModeTimeAll; 
	
} HY_DVR_EXPOSAL_CFG, *LPHY_DVR_EXPOSAL_CFG;

/* NXP8850 Image Sensor Processing������Ϣ: ��ƽ�� (White Balance)*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	/*
		1 �Զ�
		2 �ֶ��� �ֶ� ��Ҫ�޸� RED Gain (0-255),Blue Gain(0-255)
		3 ����
		4 ̫��
		5 ����
		6 ���ӫ���
	*/
	int bWorkMode;

	int nRedGain[DVR_MAX_ISP_WHITEBALANCE];

	int nBlueGain[DVR_MAX_ISP_WHITEBALANCE];

} HY_DVR_WHITEBALANCE_CFG, *LPHY_DVR_WHITEBALANCE_CFG;

/* NXP8850 Image Sensor Processing������Ϣ: ���ֽ��� ��De-Noise��*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	/*
	   0    �ر�
	   1    ����
	   Y  Strength ��0-255��
	   Cb strength (0-255)
	   Cr strength (0-255)
	 */
	int bWorkMode;

	int nYStrength;

	int nCbStrength;

	int nCrStrength;

} HY_DVR_DENOISE_CFG, *LPHY_DVR_DENOISE_CFG;

/* NXP8850 Image Sensor Processing������Ϣ*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч!!!!!!!!!!!!!!!!

	/*
	AE ģʽ��Ϊ�ֶ�ģʽʱ��AE Window�������ò�����Ч��
	*/
	HY_DVR_AE_MODE struAEMode;

	/*
		AE Window ����,�ֳ�6��Window,�ֱ��� 1 -6 
	*/
	HY_DVR_AEWINDOW_CFG struAEWindow[DVR_MAX_ISP_AEWINDOW];

	/*
		�ع����
	*/
	HY_DVR_EXPOSAL_CFG struExposal;
		
	/*
		��ƽ�� (White Balance)
	*/
	HY_DVR_WHITEBALANCE_CFG struWhiteBalance;

	/*
		���ֽ��� ��De-Noise��
	*/
	HY_DVR_DENOISE_CFG struDeNoise;

} HY_DVR_ISP_TEMPLATE, *LPHY_DVR_ISP_TEMPLATE;


//20120711
//IPC����ͼ�����
// typedef struct
// {
// 	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
// 	int EdgeSensitivity; //������
// 	int EdgeStrength; //ƫ����
// 	HY_DVR_DENOISE_CFG strNoiseFilter;
// }HY_DVR_DENOISE_CFG_Ex,LPHY_DVR_DENOISE_CFG_Ex;
// 
// typedef float FLOAT;
// 
// 
// typedef struct 
// {
// 	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
// 	BOOL        bModified;
// 	FLOAT       flCoeff_RR;
// 	FLOAT       flCoeff_GR;
// 	FLOAT       flCoeff_BR;
// 	FLOAT       flCoeff_RG;
// 	FLOAT       flCoeff_GG;
// 	FLOAT       flCoeff_BG;
// 	FLOAT       flCoeff_RB;
// 	FLOAT       flCoeff_GB;
// 	FLOAT       flCoeff_BB;
// 	FLOAT       flOffset_R;
// 	FLOAT       flOffset_G;
// 	FLOAT       flOffset_B;
// }HY_DVR_COLORMATRIX_CFG,LPHY_DVR_COLORMATRIX_CFG;
// 
// typedef struct
// {
// 	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
// 
// 	unsigned char abyGammaTbl[DVR_NUM_abyGammaTbl];  //gamma��
// 
// } HY_DVR_GAMMA_CFG,LPHY_DVR_GAMMA_CFG;
// 
// 
// /* NXP8850 Image Sensor Processing������Ϣ*/
// typedef struct
// {
// 	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
// 
// 	//ISP�������ò���
// 	HY_DVR_ISP_COMMON struCommon;
// 
// 	//ISP����ģ��
// 	HY_DVR_ISP_TEMPLATE struTemplate[DVR_MAX_ISP_TEMPLATE];
// 
// 	HY_DVR_DENOISE_CFG_Ex strNoise;
// 
// 	HY_DVR_COLORMATRIX_CFG strColorMatrix;
// 
// 	HY_DVR_GAMMA_CFG strGamma;
// 
// } HY_DVR_NXP8850ISP_CFG, *LPHY_DVR_NXP8850ISP_CFG;


typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	int EdgeSensitivity; //������
	int EdgeStrength; //ƫ����
	HY_DVR_DENOISE_CFG strNoiseFilter;
}HY_DVR_DENOISE_CFG_Ex,*LPHY_DVR_DENOISE_CFG_Ex;

typedef struct 
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	float       flCoeff_RR;
	float       flCoeff_GR;
	float       flCoeff_BR;
	float       flCoeff_RG;
	float       flCoeff_GG;
	float       flCoeff_BG;
	float       flCoeff_RB;
	float       flCoeff_GB;
	float       flCoeff_BB;
	float       flOffset_R;
	float       flOffset_G;
	float       flOffset_B;
}HY_DVR_COLORMATRIX_CFG,*LPHY_DVR_COLORMATRIX_CFG;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned char abyGammaTbl[256*3];  //gamma��

} HY_DVR_GAMMA_CFG,*LPHY_DVR_GAMMA_CFG;



/* NXP8850 Image Sensor Processing������Ϣ*/
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//ISP�������ò���
	HY_DVR_ISP_COMMON struCommon;

	//ISP����ģ��
	HY_DVR_ISP_TEMPLATE struTemplate[DVR_MAX_ISP_TEMPLATE];


	HY_DVR_DENOISE_CFG_Ex strNoise;

	HY_DVR_COLORMATRIX_CFG strColorMatrix;

	HY_DVR_GAMMA_CFG strGamma;

} HY_DVR_NXP8850ISP_CFG, *LPHY_DVR_NXP8850ISP_CFG;
#define NXP8850ISP_CFG_LEN		(sizeof (HY_DVR_NXP8850ISP_CFG))
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//��������
#define   DVR_MAX_STREAM_TYPE		2
#define   DVR_CAP_NORMAL_H264    	0x01
#define   DVR_CAP_ADVANCED_H264		0x02

#define   DVR_CAP_RESOLUTION_QCIF   0x01
#define   DVR_CAP_RESOLUTION_CIF    0x02
#define   DVR_CAP_RESOLUTION_D1   	0x04			//ʵʱ2CIF
#define   DVR_CAP_RESOLUTION_SD    	0x08			//ʵʱD1

#define DVR_CAP_RESOLUTION_4CIF_SIM   0x10		//��ʵʱD1
#define DVR_CAP_RESOLUTION_2CIF_SIM   0x20		//��ʵʱ2CIF

#define DVR_CAP_RESOLUTION_QVGA		0x40		//320*240
#define DVR_CAP_RESOLUTION_VGA			0x80		//640*480
#define DVR_CAP_RESOLUTION_SVGA		0x100		//800*600
#define DVR_CAP_RESOLUTION_XVGA		0x200		//1024*768
#define DVR_CAP_RESOLUTION_HD720		0x400	//1280*720
#define DVR_CAP_RESOLUTION_QuadVGA	0x800		//1280*960
#define DVR_CAP_RESOLUTION_SXGA		0x1000		//1280*1024
#define DVR_CAP_RESOLUTION_UXGA		0x2000		//1600*1200
#define DVR_CAP_RESOLUTION_HD1080		0x4000		//1920*1080
#define DVR_CAP_RESOLUTION_SD1		0x8000		//960*576


//���������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nStreamFormatCap;		/*֧�ֵ�������ʽ�� ����֧�ֵĸ�ʽ���Ľ�� */

	unsigned long dwResolutionCap[DVR_MAX_STREAM_TYPE];	/*ÿ�ֱ����ʽ��֧�ֵķֱ��ʸ�ʽ,֧�����еĸ�ʽ���Ľ�� */

} HY_DVR_COMPRESS_CAP, *LPHY_DVR_COMPRESS_CAP;

//����GOV length�����ṹ
typedef struct
{
    int bValid; //! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
    //GOV length I frame֮��ļ��
    int nGovLength;
} HY_DVR_EX_COMPRESSION_INF, *LPHY_DVR_EX_COMPRESSION_INF;

//�������ýṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//֡�� 1-20��ffΪȫ֡��
	int nVideoFrameRate;

	//�������ޡ���λK��
	int nVideoBitrateH;

	//��������
	int nVideoBitrateL;

	//����ϵ��:0-6
	int nQuotiety;

	//�ֱ���0-CIF, 1-D1
	int nResolution;

	//��������0-��Ƶ��,1-������
	int nStreamType;

    //¼������ 0-������ 1-��׼ 2-�� 3-�Զ���
    int nPicQuality;

    //��������0: �������ʣ�1: ���룬2���޶�����
    int nBitrateType;

    HY_DVR_EX_COMPRESSION_INF struExCompInf;

} HY_DVR_COMPRESSION_INFO, *LPHY_DVR_COMPRESSION_INFO;


//��������������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnalbe;				/* �Ƿ����������� (ֻ������) */

	//֡�� 1-20��ffΪȫ֡��
	int nVideoFrameRate;

	//����
	int nVideoBitrate;

    HY_DVR_EX_COMPRESSION_INF struExCompInf;

} HY_DVR_AUX_COMPRESSION_INFO, *LPHY_DVR_AUX_COMPRESSION_INFO;


//ͨ����������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDelayRecordTime;		/* ¼���ӳ�ʱ�䳤�� */
	int nPreRecordTime;			/* Ԥ¼ʱ�� 0-��Ԥ¼ 1-5�� 2-10�� 3-15�� 4-20�� 5-25�� */

	HY_DVR_COMPRESS_CAP struCompressCap;	/*ѹ�������� ���ڻ�ȡ����ʱ��Ч��ֻ���� */

	HY_DVR_AUX_COMPRESSION_INFO struNetPara;	/* ������ */

	HY_DVR_COMPRESSION_INFO struRecordPara[3];	/*0��������,1�¼�¼��,2�ֶ�¼�� */

	HY_DVR_COMPRESSION_INFO struHDNetPara;	/* ���������� */

} HY_DVR_COMPRESSION_CHANNEL, *LPHY_DVR_COMPRESSION_CHANNEL;
/*!
*  @struct	tagHY_DVR_VGA_NET_CFG
*  @brief	VGA�������ṹ
*/
typedef struct tagHY_DVR_HD_NET_CFG
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_COMPRESSION_INFO struVGANetPara[DVR_MAX_VIDEOIN_NUM];				/*!< VGA������ */

}HY_DVR_VGA_NET_CFG, *LPHY_DVR_VGA_NET_CFG;
/////////////////////////////////////////////////////////////////////////////////////////////////
//��������ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struWorkMode;	//����ģʽ(ֻ��)

	HY_DVR_COMPRESSION_CHANNEL struCompressChannel[DVR_MAX_VIDEOIN_NUM];

	HY_DVR_VGA_NET_CFG	struVGANet;											/*!< ��ͨ��VGA������ */

} HY_DVR_COMPRESSION_CFG, *LPHY_DVR_COMPRESSION_CFG;

#define COMPRESS_CFG_LEN   sizeof (HY_DVR_COMPRESSION_CFG)
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//¼�����״̬
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//0��ʾ�ֶ�¼��,1��ʾ�Զ�¼��,2��ʾ�ر�¼��
	unsigned int uRecordMode[DVR_MAX_VIDEOIN_NUM];
} HY_DVR_RECORD_CTRLSTATE, *LPHY_DVR_RECORD_CTRLSTATE;

//¼������ѡ��
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//0��ʾ������,1��ʾ������(CIF),2��ʾ����������(VGA)
	unsigned int uStreamIndex[DVR_MAX_VIDEOIN_NUM];
} HY_DVR_RECORD_STREAM, *LPHY_DVR_RECORD_STREAM;

//¼��ƻ��ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//0��ʾ��¼��1��ʾ��ʱ¼��2��ʾ�ƶ�¼��
	HY_DVR_SCHEDTIME struRecordTime[DVR_MAX_VIDEOIN_NUM];

	//¼�����״̬
	HY_DVR_RECORD_CTRLSTATE struRecordCtlState;

	//¼������ѡ��
	HY_DVR_RECORD_STREAM struRecordStream;

} HY_DVR_RECORD_SCHED, *LPHY_DVR_RECORD_SCHED;

#define RECORD_SCHED_LEN sizeof(HY_DVR_RECORD_SCHED)
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//����ϵͳʱ�����
typedef struct
{
	int nYear;					//��
	int nMonth;					//��
	int nDay;					//��
	int nHour;					//ʱ
	int nMinute;				//��
	int nSecond;				//��

} HY_DVR_TIME, *LPHY_DVR_TIME;

#define TIME_CFG_LEN sizeof(HY_DVR_TIME)
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//��̨Э����Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nUseable;				/*PTZЭ���״̬��0��δ��װ��1���Ѱ�װ�� */
	int bSystem;				/*PTZЭ���״̬��0����ϵͳ���еģ���ɾ����1��ϵͳ���еģ�����ɾ���� */

	char szPTZName[DVR_MAX_PTZPRONAME_LEN];	/*PTZЭ������ */

} HY_DVR_PTZPRO, *LPHY_DVR_PTZPRO;

//��̨Э�������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nPtzCount;				/* ϵͳ֧��PTZЭ��ĸ��� */

	HY_DVR_PTZPRO struPTZProtocol[DVR_MAX_PTZPRO_NUM];	/*ϵͳPTZ���� */

} HY_DVR_PTZPRO_CFG, *LPHY_DVR_PTZPRO_CFG;

#define PTZPRO_CFG_LEN sizeof(HY_DVR_PTZPRO_CFG)
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//��̨��Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned char bySetPreset[DVR_MAX_PRESET_NUM];	/* Ԥ�õ��Ƿ�����,0-û������,1-���� */
	unsigned char bySetCruise[DVR_MAX_PRESET_NUM];	/* Ѳ���Ƿ�����: 0-û������,1-���� */
	unsigned char bySetTrack[DVR_MAX_PRESET_NUM];	/* �켣�Ƿ�����,0-û������,1-���� */

} HY_DVR_PTZ, *LPHY_DVR_PTZ;

//��������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nBaudRate;				/* ������(bps)��0��50��1��75��2��110��3��150��4��300��5��600��6��1200��7��2400��8��4800��9��9600��10��19200�� 11��38400��12��57600��13��76800��14��115.2k; */
	int nDataBit;				/* �����м�λ 0��5λ��1��6λ��2��7λ��3��8λ; */
	int nStopBit;				/* ֹͣλ 0��1λ��1��2λ; */
	int nParity;				/* У�� 0����У�飬1����У�飬2��żУ��; */
	int nFlowcontrol;			/* 0���ޣ�1��������,2-Ӳ���� */
	int nDecoderAddress;		/*��������ַ:0 - 255 */
	int nComType;				/*0��COM1 RS232��1��COM2 RS485 */

} HY_DVR_SERIAL, *LPHY_DVR_SERIAL;

/////////////////////////////////////////////////////////////////////////////////////////////////
//��̨��������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nUseable;				//! �Ƿ�������������
	int nCycle;					//! ��������������, 0��ʾ����
	int nAction;				//! �������� 0��ʾԤ�õ� 1��ʾѲ�� 2��ʾ�켣 3��ʾ�Զ�ɨ��
	int nIndex;					//! ������(Ԥ�õ��,Ѳ����,�켣��,�Զ�ɨ����·��)
} HY_DVR_PTZACTION, *LPHY_DVR_PTZACTION;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szPTZName[DVR_MAX_PTZPRONAME_LEN];	/*PTZЭ������ */

	HY_DVR_SERIAL struSerial;	//��������

	HY_DVR_PTZACTION struPTZAction;

} HY_DVR_DECODER_CHANNEL, *LPHY_DVR_DECODER_CHANNEL;

//sony�ĵ���ptz��о����
typedef struct  
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDigitZoom;				/*!< ���ֱ䱶 (0-�� 1-��)			                        */
	int nLowLum;				/*!< ���ն� (0-�� 1-��)										*/
	int nBackLight;				/*!< ���ⲹ�� (0-�� 1-��)									*/
	int nAutoShutter;			/*!< �Զ����� (0-�ֶ� 1-�Զ�)			                    */
	int nWhiteBalance;			/*!< ��ƽ�� (0-�ֶ� 1-�Զ� 2-���� 3-���� 4-ATW 5-ONE PUSH)	*/
	int nRedGain;				/*!< ������ (0~255)											*/
	int nBlueGain;				/*!< ������ (0~255)											*/
	int nAcutance;				/*!< ��� (0~15)											*/
	int nExpMode;				/*!< �ع�ģʽ (0-�Զ� 1-�ֶ� 2-�������� 3-��Ȧ����)			*/
	int nShutterSpeed;			/*!< �����ٶ� (0~15)				                        */
	int nIrisSpeed;				/*!< ��Ȧ (0-�ֶ� 1-�Զ���									*/
	int nWideDyn;				/*!< ��̬ (0-�� 1-��)										*/
	int n3DFilter;				/*!< 3D����	(0-�� 1-��)										*/
	int nMirror;				/*!< ����  (0-�� 1-��)										*/
	int nReverse;				/*!< ��ת  (0-�� 1-��)										*/
	int nInputFrequency;		/*!< ����Ƶ��  (0-60Hz 1-50Hz)                              */
	int nLongRestrict;			/*!< �������٣�1-����0-�أ�									*/
	int nNorthSet;				/*!< ��λ�趨��1-�趨��										*/

} HY_DVR_PTZ_ELCTRONIC_SONY, *LPHY_DVR_PTZ_ELCTRONIC_SONY;

//����ͨ���ĵ���ptz����
typedef struct  
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_PTZ_ELCTRONIC_SONY struPtzElcSony;

} HY_DVR_PTZ_ELCTRONIC_CHANNEL, *LPHY_DVR_PTZ_ELCTRONIC_CHANNEL;


//--------------------- begin of new 20140730 �����ṹ������ -------------------------
typedef struct tagHY_DVR_PTZ_REPLENISH_CONTENT_2 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 

    //��̬�ڴ�
    int iWideDyn; 
    int iWDRParaProbeLv; 
    int iWDRParaShadowLv; 
    int iWDRParaHighlightLv; 
    int iWDRParaExposureLv; 

    int iFocusMode; /*!< �۽�ģʽ 0�Զ� 1�ֶ� 2 �Զ�+�ֶ� */ 
    int iFocusLowerLimit; /*!< �۽����� */ 
    int iHighSensitivity; /*!<�������� 0�أ�1�� */ 
    int iGain; /*!< ���棺0��λ��1�ӣ�2���� */ 
    int iLum; /*!<���ȣ�0��λ��1���ӣ� 2,���� */ 
    int iExpComp; /*!<�عⲹ����0��λ��1���ӣ� 2,���� */ 
    int iCompass; /*!<������0�أ�1�� */ 
    int iDenoiseMode; // ����ģʽ0:�� 1-5��2D��3D-1,3D-2��ǿ���룬2D+ǿ���� 
    int iDenoiseIntensity; //2D/3D����ǿ�� 0-100 
    int iDenoiseLv; //2D����ȼ� 0-5 
    int iStrongDenoiseDynaicAdaptEn; //ǿ���룺0�� 1 �� 
    int iStrongDenoiseCurFrame; //ǿ���� ��ǰ֡ ��0-9 
    int iStrongDenoiseRefFrame; //�ο�֡Ȩ�� 0-9 
} HY_DVR_PTZ_REPLENISH_CONTENT_2, *LPHY_DVR_PTZ_REPLENISH_CONTENT_2; 

typedef struct tagHY_DVR_PTZ_REPLENISH_2 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 
    HY_DVR_PTZ_REPLENISH_CONTENT_2 struContent[DVR_MAX_PTZ_NUM]; /*!< �������� */ 
} HY_DVR_PTZ_REPLENISH_2, *LPHY_DVR_PTZ_REPLENISH_2;

//--------------- end of new 20140730 ------------------------------


typedef struct 
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	int	iWorkMode;															/*!< ����ģʽ: 0�Զ���1�ֶ�									*/
	int	iManualSwitch;														/*!< �ֶ�����: 0�ֶ��أ�1�ֶ���								*/
	int	iOpenThreshold;														/*!< �����¶ȣ����ڴ��¶�ʱ����								*/
	int	iCloseThreshold;													/*!< �ر��¶ȣ����ڴ��¶�ʱ����								*/
} HY_DVR_PTZ_TEMPERATURE_DEVICE, *LPHY_DVR_PTZ_TEMPERATURE_DEVICE;

/*!
*  @struct	HY_DVR_PTZ_ELCTRONIC_CH_TEMPERATURE
*  @brief	����ͨ���ĵ���ptz�¶��豸����
*/
typedef struct  
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	HY_DVR_PTZ_TEMPERATURE_DEVICE struPtzFanDevice;							/*!< �����豸���� */
	HY_DVR_PTZ_TEMPERATURE_DEVICE struPtzHeaterDevice;						/*!< �����豸���� */
	HY_DVR_PTZ_TEMPERATURE_DEVICE struPtzReserveDevice[14];					/*!< �¶��豸����(�����豸ʱ��һ) */

} HY_DVR_PTZ_ELCTRONIC_CH_TEMPERATURE, *LPHY_DVR_PTZ_ELCTRONIC_CH_TEMPERATURE;

//����ͨ���ĵ���ptz����
typedef struct  
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_PTZ_ELCTRONIC_CHANNEL   struPtzElcChannel[DVR_MAX_PTZ_NUM];
	HY_DVR_PTZ_ELCTRONIC_CH_TEMPERATURE   struPtzElcChTemperature[DVR_MAX_PTZ_NUM];

} HY_DVR_PTZ_ELCTRONIC, *LPHY_DVR_PTZ_ELCTRONIC;

/*!
*  @struct	tagHY_DVR_PTZACTION
*  @brief	��̨��������
*/
typedef struct
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_PTZACTION struPTZAction[DVR_MAX_PTZ_NUM];						/*!< ��̨���� */

} HY_DVR_PTZACTION_CFG, *LPHY_DVR_PTZACTION_CFG;

/*!
*  @struct	HY_DVR_PTZ_REPLENISH_CONTENT_1
*  @brief	ptz�豸������������
*/
typedef struct 
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	int	iLowPowerConsumption;												/*!< �͹���: 0�أ�1��										*/
	int	iScreenWiper;														/*!< ��ˢ��: 0�أ�1���Σ�2����								*/
	int	iNightVision;														/*!< ҹ�ӣ�0�أ�1����2�Զ�									*/
	int	iDemist;															/*!< ����0�أ�1��											*/
} HY_DVR_PTZ_REPLENISH_CONTENT_1, *LPHY_DVR_PTZ_REPLENISH_CONTENT_1;

/*!
*  @struct	HY_DVR_PTZ_REPLENISH_1
*  @brief	ptz�豸��������
*/
typedef struct
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_PTZ_REPLENISH_CONTENT_1 struContent[DVR_MAX_PTZ_NUM];			/*!< �������� */

} HY_DVR_PTZ_REPLENISH_1, *LPHY_DVR_PTZ_REPLENISH_1;

/*!
*  @struct	tagHY_DVR_DECODER_CFG
*  @brief	�豸��������
 */
typedef struct
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_DECODER_CHANNEL struDecoder[DVR_MAX_PTZ_NUM];					/*!< ������ͨ������ */
	HY_DVR_PTZACTION_CFG	struPtzAction;									/*!< ��ͨ����̨�������� */
	HY_DVR_PTZ_ELCTRONIC	struPtzElc;										/*!< ����ptz���� */
	HY_DVR_PTZ_REPLENISH_1	struReplenish1;									/*!< ��������1		*/
    HY_DVR_PTZ_REPLENISH_2  struReplenish2; /*!< ��������2 */ 

} HY_DVR_DECODER_CFG, *LPHY_DVR_DECODER_CFG;

#define  DECODER_CFG_LEN  sizeof(HY_DVR_DECODER_CFG)

//20120426
/*!
*  @struct	HY_DVR_PTZ_SHIELD
*  @brief	��̨�ڱ���Ϣ
 */

#define  DVR_MAX_RESERVE_NUM 4
typedef struct
{
	int bValid;    /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	int iColor;  /*�ڸ���ɫ*/
	int iStart_x;  /*��ʼ����x*/
	int iStart_y; /*��ʼ����y*/
	int iEnd_x;  /*���½�����x */
	int iEnd_y;  /*���½�����y*/
	int iReserved[DVR_MAX_RESERVE_NUM];
}HY_DVR_PTZ_SHIELD,*LPHY_DVR_PTZ_SHIELD;
/*!
*  @struct	HY_DVR_PTZ_SHIELD_SET
*  @brief	��̨�ڱ�����
 */
typedef struct
{
	int bValid;    /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	int Enable;  /*ʹ���ڱ� �ر� 0  ���� 1*/
	int iLookup;	/*��ǰλ��*/
	int SetDellocal;  /*���û�ɾ���ĵ�*/
	HY_DVR_PTZ_SHIELD struShield[MAX_PTZ_SHIELD_NUMBER];
}HY_DVR_PTZ_SHIELD_SET,*LPHY_DVR_PTZ_SHIELD_SET;

/*!
*  @struct	HY_DVR_PTZ_REGION_ZOOM
*  @brief	�Ŵ�����
 */
typedef struct
{
	int bValid;/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	int iStatus;  /*״̬: 0�ָ�  1�Ŵ�*/
	int iStart_x; /*ˮƽ��ʼ����x  */
	int iStart_y; /*��ʼ����y*/
	int iEnd_x;   /*���½�����x */
	int iEnd_y;   /*���½�����y*/
	int iReserved[4];
}HY_DVR_PTZ_REGION_ZOOM,*LPHY_DVR_PTZ_REGION_ZOOM;

//20120426 end

///////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////
//��������������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nAlarmInType;			/*����������,0������,1������ */

	HY_DVR_HANDLEEXCEPTION struHandleType;	/* ����ʽ */

	HY_DVR_SCHEDTIME struSchedTime;	//ʱ��ƻ�

} HY_DVR_ALARMIN_CHANNEL, *LPHY_DVR_ALARMIN_CHANNEL;

//���������ͨ������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nAlarmOutDelay;			/* �������ʱ��(-1Ϊ���ޣ��ֶ��ر�) */

	HY_DVR_SCHEDTIME struSchedTime;	//�����������ʱ��

} HY_DVR_ALARMOUT_CHANNEL, *LPHY_DVR_ALARMOUT_CHANNEL;

//��������ץͼ����
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nCapPicCount;			//������������
	int nCapPicIntervalTime;	//�������ռ��ʱ��

} HY_DVR_ALARM_CAPTURE_PIC, *LPHY_DVR_ALARM_CAPTURE_PIC;

//ϵͳ�쳣���
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				//�Ƿ��������

	int nCheckTime;				//���ʱ��������λΪ��

	HY_DVR_HANDLEEXCEPTION struHandleType;	/* ��������ʽ */

} HY_DVR_SYSTEM_EXCEPTION, *LPHY_DVR_SYSTEM_EXCEPTION;

//��������
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struWorkMode;	//����ģʽ(ֻ��)

	HY_DVR_ALARM_CAPTURE_PIC struCapPic;	//��������

	HY_DVR_ALARMIN_CHANNEL struAlarmIn[DVR_MAX_ALARMIN_NUM];	//������������

	HY_DVR_ALARMOUT_CHANNEL struAlarmOut[DVR_MAX_ALARMOUT_NUM];	//������������

	/*
	   0    Ӳ�̹���
	   1    IP��ͻ
	   2    �Ƿ�����
	   3    ���߶�
	   4    ������
	   5    �����쳣
	 */
	HY_DVR_SYSTEM_EXCEPTION struSystemException[DVR_MAX_EXCEPTION_NUM];

	//��������ͨ������
	HY_DVR_ALIAS_CFG struAlarmInAlias[DVR_MAX_ALARMIN_NUM];

	//�������ͨ������
	HY_DVR_ALIAS_CFG struAlarmOutAlias[DVR_MAX_ALARMOUT_NUM];

	//��������ͨ������(unicode)
	HY_DVR_ALIAS_CFG_1 struAlarmInAlias_1[DVR_MAX_ALARMIN_NUM];

	//�������ͨ������(unicode)
	HY_DVR_ALIAS_CFG_1 struAlarmOutAlias_1[DVR_MAX_ALARMOUT_NUM];

	//!һ����������״̬,0Ϊ����,1Ϊ����
	int byState;
	

} HY_DVR_ALARM_CFG, *LPHY_DVR_ALARM_CFG;

#define ALARM_CFG_LEN   sizeof (HY_DVR_ALARM_CFG)
//////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
//��ʾ����
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nScreenSaveTime;		/* ��Ļ����ʱ�� */

	int nMenuAlphaValue;		/* �˵��뱳��ͼ��Աȶ� */

	//int nResolvingPowerCount; //֧�ֵ����Էֱ�����

	//int nCurrentResolvingPower[3];    //��ǰ���Էֱ���

	//int nResolvingPowerList[3][DVR_MAX_VGARESOLVING_NUM]; //֧�ֵ����Էֱ����б�

} HY_DVR_VO_CFG, *LPHY_DVR_VO_CFG;

#define VO_CFG_LEN   sizeof (HY_DVR_VO_CFG)
/////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////

//�û�ʹ����Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				//�Ƿ�����

} HY_DVR_USER_ENABLE, *LPHY_DVR_USER_ENABLE;


//�û�������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szPassword[DVR_MAX_PASSWORD_LEN];	/* ���� */

	unsigned char byLimitMac[DVR_MAX_MAC_NUM_LEN];	/* ���Ʒ��������ַ */

	char szLimitIP[DVR_MAX_IP_STR_LEN];	/* ���Ʒ���IP��ַ */

	char szEmail[DVR_MAX_EMAIL_LEN];

} HY_DVR_USER_INFO, *LPHY_DVR_USER_INFO;

//�û�Ȩ����Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nLocalRight[DVR_MAX_RIGHT_NUM];	/* Ȩ�� */
	/*����0: ����ϵͳ���� */
	/*����1: �ļ����� */
	/*����2: ¼�񱸷� */
	/*����3: ϵͳ�ر� */
	/*����4: �ʼ�֪ͨ */

	int nRemoteRight[DVR_MAX_RIGHT_NUM];	/* Ȩ�� */
	/*����0: ϵͳ���� */
	/*����1: �����Խ� */
	/*����2: ������� */
	/*����3: ������� */
	/*����4: Զ������ */
	/*����5: �ر�ϵͳ */

	int nChannelRight[DVR_MAX_RIGHT_NUM];	/* Ȩ�� */
	/*����0: ���ػط� */
	/*����1: ����Ԥ�� */
	/*����2: ������̨���� */
	/*����3: �����ֶ�¼�� */
	/*����4: Զ�̻ط� */
	/*����5: Զ��Ԥ�� */
	/*����6: Զ����̨���� */
	/*����7: Զ���ֶ�¼�� */

} HY_DVR_USER_RIGHT, *LPHY_DVR_USER_RIGHT;


//�û���ϢȨ����Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szUserName[DVR_MAX_USERNAME_LEN];	/* �û��� ���16λ */

	//�û�ʹ����Ϣ
	HY_DVR_USER_ENABLE struUserEnable;

	//�û�������Ϣ
	HY_DVR_USER_INFO struUserInfo;

	//�û�Ȩ����Ϣ
	HY_DVR_USER_RIGHT struUserRight;

} HY_DVR_USER, *LPHY_DVR_USER;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	// �Ƿ�ΪԶ���û�
	int bRemote;

	// �Ƿ�������״̬
	int bConnect;

	// �û����
	unsigned long dwUserID;

	// ��½Э��
	unsigned long dwProtocolID;

	// �û���
	char szUserName[DVR_MAX_USERNAME_LEN];

	// ��½IP
	unsigned long dwIPAddress;

	// ��½ʱ��
	HY_DVR_TIME struLoginTime;

	// �������� b/s
	unsigned long dwBitrate;
}HY_DVR_ONLINEUSER, *LPHY_DVR_ONLINEUSER; 

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	// �����û��б�
	HY_DVR_ONLINEUSER struOnlineUser[DVR_MAX_ONLINEUSER_NUM];
} HY_DVR_ONLINEUSER_CFG, *LPHY_DVR_ONLINEUSER_CFG;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned long dwWhiteIP;
} HY_DVR_WHITEIP, *LPHY_DVR_WHITEIP;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				// �Ƿ�����IP������
} HY_DVR_WHITEIP_ENABLE, *LPHY_DVR_WHITEIP_ENABLE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struWorkMode;	//����ģʽ(ֻ��)

	HY_DVR_USER struUser[DVR_MAX_USER_NUM];

	HY_DVR_ONLINEUSER_CFG struUserOnline;

	HY_DVR_WHITEIP struWhiteIP[DVR_MAX_WHITEIP_NUM];

	HY_DVR_WHITEIP_ENABLE struWhiteIPEnable;

} HY_DVR_USER_CFG, *LPHY_DVR_USER_CFG;

#define USER_CFG_LEN	sizeof(HY_DVR_USER_CFG)
/////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
//�豸��Ϣ�ṹ(ֻ��)
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//���ͱ��
	unsigned long dwBoardType;

	//�豸�������ƣ���HY-2016HC
	char szTypeName[DVR_MAX_VERSION_LEN];

	unsigned long dwSerialNum;	/* ���к� */

	//!\brief   Ӳ���汾
	char szHardwareVersion[DVR_MAX_VERSION_LEN];

	//!\brief   ����汾
	char szSoftwareVersion[DVR_MAX_VERSION_LEN];

	//!\brief   ����汾
	char szDspVersion[DVR_MAX_VERSION_LEN];

	unsigned char byVideoInChannels;	/*��Ƶ����·�� */
	unsigned char byAudioInChannels;	/*��Ƶ����·�� */
	unsigned char byAlarmInChannels;	/*��������·�� */
	unsigned char byAlarmOutChannels;	/*�������·�� */

	unsigned char byVGAChannels;	/*VGA�ӿڸ��� */

	unsigned char bySpotOutChannles;	/*SPOT���·�� */
	unsigned char byVoiceInChannels;	/*�����Խ�����·�� */

} HY_DVR_DEVICE_INFO, *LPHY_DVR_DEVICE_INFO;

#define	DEVICE_INFO_LEN	sizeof(HY_DVR_DEVICE_INFO)
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
///*!
//*  @struct	tagHY_DVR_DEVICE_OUTPUT_INDEX
//*  @brief	����豸���(ֻ��)
//*/
//typedef struct tagHY_DVR_DEVICE_OUTPUT_INDEX
//{
//	int bValid;															/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
//
//	unsigned long	dwDeviceType;										/*!< �豸����(0: ��Ч, 1: VGA, 2 :TV) */
//	unsigned long	dwDeviceIndex;										/*!< �豸��� */
//
//} HY_DVR_DEVICE_OUTPUT_INDEX, *LPHY_DVR_DEVICE_OUTPUT_INDEX;
//
///*!
//*  @struct	tagHY_DVR_DEVICE_OUTPUT
//*  @brief	����豸��Ϣ(ֻ��)
//*/
//typedef struct tagHY_DVR_DEVICE_OUTPUT
//{
//	int bValid;															/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
//
//	HY_DVR_DEVICE_OUTPUT_INDEX cOutputDevice[MAX_DEVICE_OUTPUT];
//
//} HY_DVR_DEVICE_OUTPUT, *LPHY_DVR_DEVICE_OUTPUT;

//////////////////////////////////////////////////////////////////////////////
//��ʱ����Ϣ�ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//!\brief   �Ƿ�������ʱ��
	int bEnableDST;

	HY_DVR_TIME struStart;		//��ʼʱ�䣬����Ч

	HY_DVR_TIME struEnd;		//����ʱ�䣬����Ч

	int nDSTBias;				//��������ʱ�䣬����Ϊ��λ

} HY_DVR_DST, *LPHY_DVR_DST;

//�豸�������ã�����ʱ���ʽ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//!\brief   �Զ�����ʱ�䣨��λΪ�룩
	int nAutoLockScreenTime;

	//!\brief   ����
	// 0(Default) - �������� 1 - �������� 2 - Ӣ��
	int nMenuLanguage;

	//!\brief   ���ڸ�ʽ
	// 0(Default) - YYYY-MM-DD W hh:mm:ss, 1 - YYYY-MM-DD hh:mm:ss, 2 - MM-DD-YYYY W hh:mm:ss, 3 - MM-DD-YYYY hh:mm:ss
	int nDateFormat;

	//!\brief   ʱ���ʽ��12Сʱ�ƣ�24Сʱ�ƣ�
	int nTimeFormat;

} HY_DVR_GUI, *LPHY_DVR_GUI;

//�豸�������ã�ʱ��
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	//!\brief   ʱ��ʱ��
	int nTimeZone;

} HY_DVR_TIMEZONE, *LPHY_DVR_TIMEZONE;

//�豸�������ã�ʱ��
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDvrID;					/* DVR ID,����ң���� */

} HY_DVR_DVRID, *LPHY_DVR_DVRID;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nVideoInStandard;		//��Ƶ������ʽ 

    /*!< ��Ƶ������ʽ:1-NTSC,2-PAL,4-SECAM */
	int nVideoOutStandard;		//��Ƶ�����ʽ��ͨ����������ʽһ����

} HY_DVR_STANDARD, *LPHY_DVR_STANDARD;


typedef struct tagNEW_HY_DVR_NTP_INTV 
{ 
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */ 

    int nNTPInterval; /*!<0:ÿ�� 1��ÿ�� 2�����ζ�ʱ 3��ÿ��> */ 

} HY_DVR_NTP_INTV, *LPHY_DVR_NTP_INTV; 


//20131109 ����NTP����
typedef struct tagNEW_HY_DVR_NTP
{
    int bValid; /*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

    int bEnableNTP; /*!<1:enable 0:disable> */

    char timeSvr[DVR_MAX_NTP_SVR_LEN];

    HY_DVR_NTP_INTV struNTPIntv; //�����½ӿ�����

} HY_DVR_NTP, *LPHY_DVR_NTP;



//�豸�������ýṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_DVRID struDvrID;

	//!\brief   �Զ�����ʱ�䣨��λΪ�룩
	HY_DVR_GUI struGUI;

	HY_DVR_TIMEZONE struTimeZone;

	//��ʱ����Ϣ
	HY_DVR_DST struDST;

	//�豸����
	HY_DVR_ALIAS_CFG struDvrAlias;

	//�豸��ʽ
	HY_DVR_STANDARD struStandard;

	//�豸����(unicode)
	HY_DVR_ALIAS_CFG_1 struDvrAlias_1;

    //2013.11.9 ����NTP�Ļ�ȡ��Ϣ
    HY_DVR_NTP struNTP; 

} HY_DVR_DEVICE_CFG, *LPHY_DVR_DEVICE_CFG;

#define	DEVICE_CFG_LEN	sizeof(HY_DVR_DEVICE_CFG)
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
//������Ϣ�ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szDevNode[DVR_MAX_PARTITION_DEVNODE_LEN];	//�豸�ڵ���, �� /dev/sda1

	unsigned long dwCapacity;	//������, MΪ��λ
	unsigned long dwSpare;		//ʣ������

	unsigned long dwStartCHS;	// ������ʼ����

	unsigned long dwEndtCHS;	// ������������

} HY_DVR_DISK_PARTITION, *LPHY_DVR_DISK_PARTITION;

//Ӳ�̱��
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nVerifyMark;			//��֤��(���øı�ʱ����֤���ͬ���ı�)

} HY_DVR_DISK_MARK, *LPHY_DVR_DISK_MARK;

#define	DISK_MARK_LEN	sizeof(HY_DVR_DISK_MARK)

//�洢�豸���ͣ�HD��SD��USB��IPSAN��NAS��
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int nDiskType;				//! 0- unkonown��1-HDD��2-USB��4-CDROM��8-SD��16-IPSAN��32-NAS
}HY_DVR_DISK_TYPE, *LPHY_DVR_DISK_TYPE;

//Ӳ����Ϣ�ṹ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				//�Ƿ�װ�д�Ӳ��

	HY_DVR_DISK_MARK struVerifyMark;	//��֤��(���øı�ʱ����֤���ͬ���ı�)

	int nRestState;				//Ӳ��״̬���Ƿ����ߣ�
	int nWriteState;			//Ӳ��״̬���Ƿ��д��

	char szSerialNum[DVR_MAX_VERSION_LEN];	//���к�
	char szTypeName[DVR_MAX_VERSION_LEN];	//Ӳ������
	int nBusNum;				//����λ��,SATA1

	unsigned long dwCapacity;	//����,MΪ��λ
	//unsigned long dwSpare;        //��ʣ������
	int nPartitionCount;		//��������
	HY_DVR_DISK_PARTITION struPartitionInfo[DVR_MAX_PARTITION_IN_HARDDISK];	//������Ϣ

	HY_DVR_DISK_TYPE struDiskType;

} HY_DVR_DISK, *LPHY_DVR_DISK;

//������Ϣ�ṹ
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int bEnable;				//�Ƿ����ñ�����

	int nGroupType;				//��������

	int nBindChannelMask;		//�󶨵�ͨ��

	int nDataKeepTime;			//���ݱ���ʱ��

	int nPartitionCount;		//����ӵķ�������

	char szPartitionList[DVR_MAX_PARTITION_IN_DISKGROUP][DVR_MAX_PARTITION_DEVNODE_LEN];	//�豸�ڵ��б�,֧��32������

} HY_DVR_DISK_GROUP, *LPHY_DVR_DISK_GROUP;

/*!
*  @struct	tagHY_DVR_DISK_TYPE_CFG
*  @brief	�洢�豸��������
*/
typedef struct tagHY_DVR_DISK_TYPE_CFG
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_DISK_TYPE struDiskType[DVR_MAX_HARDDISK_NUM];					/*!< �洢�豸���� */

}HY_DVR_DISK_TYPE_CFG, *LPHY_DVR_DISK_TYPE_CFG;
//�洢���� ��Ϣ
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struWorkMode;	//����ģʽ(ֻ��)

	//Ӳ����Ϣ
	HY_DVR_DISK struDisk[DVR_MAX_HARDDISK_NUM];	//8��Ӳ�̣�ֻ����; SD�̵���Ϣ������

	//������Ϣ
	HY_DVR_DISK_GROUP struDiskGroup[DVR_MAX_DISKGROUP_NUM];	//8�����飨ֻ����

	HY_DVR_DISK_TYPE_CFG struDiskType;										/*!< �洢�豸�������ã�ֻ���� */

} HY_DVR_STORAGE_CFG, *LPHY_DVR_STORAGE_CFG;

#define	STORAGE_CFG_LEN	sizeof(HY_DVR_STORAGE_CFG)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/*!
*	@struct tagHY_DVR_VGA_VIDEO_BITRATE
*	@brief	VGA����������
*/
typedef struct tagHY_DVR_VGA_VIDEO_BITRATE
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	int nVGAVideoBitrate;													/*!< ���������ʣ���λkbit/s�� */

}HY_DVR_VGA_VIDEO_BITRATE, *LPHY_DVR_VGA_VIDEO_BITRATE;

//�豸ͨ��״̬
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч(��ϵͳû�и�ͨ��)

	int nVideoBitrate;			//����������
	int nChildVideoBitrate;		//����������

	int bUseAuxVideoStream;		//�Ƿ�����������

	//��¼��0
	//����¼������1
	//��ͨ����2
	//�ֶ�¼������3
	//�ƶ�¼��4
	int nRecordType;			//¼��״̬(������)
	int bViLost;				//�Ƿ�������Ƶ�źű���(0��ʾ���źţ�1��ʾ���ź�)
	int bMotion;				//�Ƿ����ƶ�����(�в�֧��)

	HY_DVR_VGA_VIDEO_BITRATE struVGABitrate;								/*!< VGA���������� */

} HY_DVR_CHANNEL_STATE, *LPHY_DVR_CHANNEL_STATE;

/*!
*  @struct	tagHY_DVR_VGA_VIDEO_BITRATE_CFG
*  @brief	VGA��������������
*/
typedef struct tagHY_DVR_VGA_VIDEO_BITRATE_CFG
{
	int bValid;																/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */
	HY_DVR_VGA_VIDEO_BITRATE struVGABitrate[DVR_MAX_VIDEOIN_NUM];			/*!< VGA���������� */

}HY_DVR_VGA_VIDEO_BITRATE_CFG, *LPHY_DVR_VGA_VIDEO_BITRATE_CFG;

//�豸״̬��Ϣ(ֻ��)
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_CHANNEL_STATE struChannelState[DVR_MAX_VIDEOIN_NUM];	//ͨ��״̬

	HY_DVR_VGA_VIDEO_BITRATE_CFG struVGABitrate;							/*!< VGA�������������� */

} HY_DVR_DEVICE_STATE, *LPHY_DVR_DEVICE_STATE;

#define	DEVICE_STATE_LEN	sizeof(HY_DVR_DEVICE_STATE)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//�豸�Զ�ά����������
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч(��ϵͳû�и�ͨ��)

	int bEnable;				//���ã����ǽ���

	int nMode;					//ά��ģʽ 0Ϊÿ�졢1Ϊÿ�ܡ�2Ϊ����

	//����ģʽʱ����0λ��ʾ������,��1λ��ʾ����1,��ӦλΪ1��ʾ������Ч
	int nWeekDayMask;

	//��Ϊ����ģʽ����������ʱ����Ч
	//����ģʽ��ʱ����Ч
	HY_DVR_TIME struTime;

} HY_DVR_MAINTENANCE_COMMON, *LPHY_DVR_MAINTENANCE_COMMON;

//�豸�Զ�ά��
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_MAINTENANCE_COMMON struCommon;	//��������


} HY_DVR_DEVICE_MAINTENANCE, *LPHY_DVR_DEVICE_MAINTENANCE;

#define	DEVICE_MAINTENANCE_LEN	sizeof(HY_DVR_DEVICE_MAINTENANCE)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////

/*!
*  @struct	tagHY_DVR_HC_RESMODE
*  @brief	HC���ͱ������Դ����ģʽ
*/
typedef struct tagHY_DVR_HC_RESMODE
{
	int bValid;					/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	//�ýṹ��ֻ��HC��HS��������Ч

	/*  ����ģʽȡֵ��ΧΪ(0 - 2)
	   16·����: 
	   ģʽ0ʱ��ϵͳ֧��16·ʵʱCIF���ʵʱ2CIF��D1��ʽ�����
	   ģʽ1ʱ��ϵͳ֧��8·ʵʱ2CIF ��8·ʵʱCIF ��⣬4·2CIFʵʱ����
	   ģʽ2ʱ��ϵͳ֧��4·ʵʱD1 ��12·ʵʱCIF���룬1·D1ʵʱ����
	   8·����: 
	   ģʽ0ʱ��ϵͳ֧��8·ʵʱCIF���ʵʱ2CIF��D1��ʽ�����
	   ģʽ1ʱ��ϵͳ֧��2·ʵʱD1 ��6·ʵʱCIF ��⣬4·D1ʵʱ����
	   ģʽ2ʱ��ϵͳ֧��6·ʵʱD1 ��2·ʵʱCIF���룬1·D1ʵʱ����
	 */
	int nMode;

	unsigned long dwChMask;		/*!< ͨ�����룬��0λΪ1��ʾͨ��1֧��2CIF��D1��ʽ���룬�������� */

} HY_DVR_HC_RESMODE, *LPHY_DVR_HC_RESMODE;

//�豸���ܶ���
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_WORKMODE struUser;	//�߼��û�����ģʽ

	HY_DVR_WORKMODE struAlarm;	//�߼���������ģʽ

	HY_DVR_WORKMODE struPic;	//�߼�ͼ���������

	HY_DVR_WORKMODE struEmail;	//Email��������

	HY_DVR_WORKMODE struRecord;	//�߼�¼���������

	HY_DVR_WORKMODE struCMS;	//���Ĺ������������

	HY_DVR_WORKMODE struAMS;	//��������������

	HY_DVR_WORKMODE struNTP;	//NTP����������

	HY_DVR_WORKMODE struStorage;	//�߼��洢����

	HY_DVR_WORKMODE struMaintenance;	//�Զ�ά������

	HY_DVR_HC_RESMODE struHCResMode;	/*!< HC���ͱ������Դ����ģʽ */

	//HY_DVR_WORKMODE struBeep; //�����澯�Զ����(Ŀǰ��δ֧��)

} HY_DVR_DEVICE_CUSTOM, *LPHY_DVR_DEVICE_CUSTOM;

#define	DEVICE_CUSTOM_LEN	sizeof(HY_DVR_DEVICE_CUSTOM)
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//�澯��ֵ���ݲ�ʵ�֣�
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	float fCPUUseRatio;			/*!< CPU������ */
	float fMemoryUseRatio;		/*!< �ڴ������� */
	float fDiskSpaceRatio;		/*!< Ӳ�������� */
	unsigned long dwDiskSpaceBalance;	/*!< Ӳ��ʣ��ռ� */

} HY_DVR_THRESHOLD, *LPHY_DVR_THRESHOLD;

//���ط�ʽ����������NAT�����������֣��ݲ�ʵ�֣�
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч
	unsigned long dwDownloadMode;	/*!< ���ط�ʽ�� 0-hxht, 1-STREAM_TCP */

} HY_DVR_DOWNLOAD_MODE, *LPHY_DVR_DOWNLOADMODE;

//////////////////////////////////////////////////////////////////////////////

//���Ĺ���������洢ע��������Ϣ
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szManageHostAddress[DVR_MAX_DOMAINNAME_LEN];	/*Զ�̹���������ַ�����ȿ������� */

	unsigned short wManageHostPort;	/*Զ�̹��������˿� */

} HY_DVR_NET_CMS_STOREAGE, *LPHY_DVR_NET_CMS_STOREAGE;

//ǰ���豸ID
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szDeviceID[DVR_COMMON_STRING_LEN];	/* ǰ���豸ID */

} HY_DVR_DEVICE_ID, *LPHY_DVR_DEVICE_ID;

//NAT����
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned long dwIsNAT;		/*!< �Ƿ���NAT���棬0��ʾ���ں��棬1��ʾ��NAT���� */

} HY_DVR_NATSTATUS, *LPHY_DVR_NATSTATUS;

/*
ѹ����ʽ ����ͨ���� �������   ��������    
G711U.HX      1        16        8000  
G711A.HX      1        16        8000  
G722.HX       1        16    8000��16000
*/
//������������
typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	char szAudioEncoder[DVR_COMMON_STRING_LEN];	/*!< ֧�ֵ����������ʽ�������ж�����Զ��Ÿ�����G711U.HX G711A.HX G722.HX */
	unsigned long dwChannel;	/*!< ͨ���� */
	unsigned long dwBitsPerSample;	/*!< ������� */
	unsigned long dwSamplesPerSec;	/*!< ������ */

} HY_DVR_VOICEPARAM, *LPHY_DVR_VOICEPARAM;

typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_NET_CMS_STOREAGE struCmsStorage;	//���Ĺ���������洢ע����Ϣ
	HY_DVR_DEVICE_ID struDeviceID;	//ǰ���豸ID
	HY_DVR_NATSTATUS struNATStatus;	//NAT״̬
	HY_DVR_VOICEPARAM struVoiceParam;	//�������в���

} HY_DVR_HXHT_CFG, *LPHY_DVR_HXHT_CFG;

typedef struct
{
	int bValid;					//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	unsigned int dwFileMap; //0~30λ��ʾ1��~31�ŵ��ļ��ֲ����,0��ʾ���ļ�,1��ʾ���ļ�
} HY_DVR_FILEMAP_CFG, * LPHY_DVR_FILEMAP_CFG;


//////////////////////////////////////////////////////////////////////////////

/** @brief ��ַ����(��������)																		*/
#define HY_DVR_ADDRESS_LEN				(256)

/** @brief �û�������																	*/
#define HY_DVR_USERNAME_LEN				(32)

/** @brief �û����볤��																	*/
#define HY_DVR_USERPASS_LEN				(32)

/** @brief Ĭ�ϱ����ֶθ���																*/
#define HY_DVR_DEF_RESERVE_NUM			(3)

/*!
*  @struct	tagHY_DVR_USERLOGIN_INFO
*  @brief	�û���¼����
*/
typedef struct tagHY_DVR_USERLOGIN
{
	int bValid;													//���ṹ���Ƿ���Ч��=0��ʾ��Ч
	char	sServerIP[HY_DVR_ADDRESS_LEN];						/*!< ��������IP��ַ							*/
	unsigned long	dwCommandPort;								/*!< ��¼���ӵ�����˿�						*/
	char	sUName[HY_DVR_USERNAME_LEN];						/*!< ��¼�û����û�����						*/
	char	sUPass[HY_DVR_USERPASS_LEN];						/*!< ��¼�û����û�����						*/
	unsigned long	dwReserve[HY_DVR_DEF_RESERVE_NUM];			/*!< ����									*/
}HY_DVR_USERLOGIN_INFO, *LPHY_DVR_USERLOGIN_INFO;

/*!
*  @struct	tagHY_DVR_SOURCE_DEVICE_INFO
*  @brief	Դ�豸��Ϣ
*/
typedef struct tagHY_DVR_SOURCE_DEVICE_INFO
{
	int bValid;													//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	int						nEnable;							/*!< 0��ʾ�����ã�1��ʾ����			*/
	HY_DVR_USERLOGIN_INFO	cUserPara;							/*!< �û�����						*/
	unsigned long			dwVideoInNumber;					/*!< ��Ƶ������						*/
	unsigned long			dwReserve[HY_DVR_DEF_RESERVE_NUM];	/*!< ����							*/
}HY_DVR_SOURCE_DEVICE_INFO, *LPHY_DVR_SOURCE_DEVICE_INFO;

/*!
*  @struct	tagHY_DVR_SOURCE_DEVICE_INFO_CONFIG
*  @brief	Դ�豸����
*/
typedef struct tagHY_DVR_SOURCE_DEVICE_INFO_CONFIG
{
	int bValid;														//���ṹ���Ƿ���Ч��=0��ʾ��Ч

	HY_DVR_SOURCE_DEVICE_INFO	cSourceDevice[MAX_SOURCE_DEVICE_NUMBER];	/*!< Դ�豸��Ϣ					*/
	unsigned long				dwReserve[HY_DVR_DEF_RESERVE_NUM];			/*!< ����				*/
}HY_DVR_SOURCE_DEVICE_INFO_CONFIG, *LPHY_DVR_SOURCE_DEVICE_INFO_CONFIG;

//////////////////////////////////////////////////////////////////////////////
//��ʾ�豸����
//��ȡ��ʾ�豸��Ϣ

/*!
*  @struct	tagHY_DVR_VGAPARA
*  @brief	VGA����
*/
typedef struct tagHY_DVR_VGAPARA
{
	int bValid;									/*!< ���ṹ���Ƿ���Ч��0��ʾ��Ч */

	int nResolution;							/*!< �ֱ��� ,����λ��ʾ */
	int nResolutionCap;							/*!< ����������ֻ������ֱ����й�,��λ��ʾ�������ھ�λΪ1��ʾ
												֧��9���� */
	unsigned long	dwDeviceIndex;				/*!< �豸��� */
	int nBrightness;											/*!< ����,0-255  */
	int nContrast;												/*!< �Աȶ�,0-255  */
	int nHue;													/*!< ɫ��,0-255  */
	int nSaturation;											/*!< ���Ͷ�,0-255  */

}HY_DVR_VGAPARA, *LPHY_DVR_VGAPARA;

/*!
*  @struct	tagHY_DVR_VGAPARA_CFG
*  @brief	VGA��������
*/
typedef struct tagHY_DVR_VGAPARA_CFG
{
	int bValid;									/*!< 0��ʾ��Ч 1��ʾ��Ч */

	int nVGACount;								/*!< VGA Outʵ�ʸ��� */
	HY_DVR_VGAPARA struVga[MAX_VGA_NUM];

}HY_DVR_VGAPARA_CFG, *LPHY_DVR_VGAPARA_CFG;

/*!
*  @struct	tagHY_DVR_VOOUT
*  @brief	��Ƶ���
*/
typedef struct tagHY_DVR_VOOUT
{
	int bValid; 								/*!< 0��ʾ��Ч 1��ʾ��Ч */

	int byVideoFormat;							/*!< �����ʽ,0-PAL,1-NTSC */
	int wLeftffset;								/*!< ��Ƶ�����ƫ�� */
	int wRightffset;							/*!< ��Ƶ�����ƫ�� */
	int wTopffset;								/*!< ��Ƶ�����ƫ�� */
	int wBottomffset;							/*!< ��Ƶ�����ƫ�� */
	int dwResolutionCap;						/*!< ����������ֻ����*/
	unsigned long	dwDeviceIndex;				/*!< �豸��� */
	int nBrightness;											/*!< ����,0-255  */
	int nContrast;												/*!< �Աȶ�,0-255  */
	int nHue;													/*!< ɫ��,0-255  */
	int nSaturation;											/*!< ���Ͷ�,0-255  */

}HY_DVR_VOOUT, *LPHY_DVR_VOOUT;

/*!
*  @struct	tagHY_DVR_VOOUT_CFG
*  @brief	��Ƶ�������
*/
typedef struct tagHY_DVR_VOOUT_CFG
{
	int bValid;									/*!< 0��ʾ��Ч 1��ʾ��Ч */
	int nTVCount;								/*!< TV Out ����*/
	HY_DVR_VOOUT struTv[MAX_TV_NUM];

}HY_DVR_VOOUT_CFG, *LPHY_DVR_VOOUT_CFG;

/*!
*  @struct	tagHY_DVR_DISPDEV_CFG
*  @brief	��������ʾ�豸��Ϣ
*/
typedef struct tagHY_DVR_DISPDEV_CFG
{
	int bValid; 								/*!< 0��ʾ��Ч 1��ʾ��Ч */

	HY_DVR_VGAPARA_CFG struVGA;					/*!< VGA���� */
	HY_DVR_VOOUT_CFG struTV;					/*!< TV���� */

} HY_DVR_DISPDEV_CFG,  * LPHY_DVR_DISPDEV_CFG;

//////////////////////////////////////////////////////////////////////////////
/*!
*  @enum	eDeviceCapability
*  @brief	�豸��������
*/
typedef enum eDeviceCapability
{
	//�����ֶ�0
	eCapabilityLocalNetwork			= 0x00,							/*!< ��������								*/
	eCapabilityWirelessNetwork		= 0x01,							/*!< ��������								*/
	eCapabilityDHCP					= 0x02,							/*!< DHCP									*/
}eDeviceCapability;
//////////////////////////////////////////////////////////////////////////////
/*!
*  @struct	tagHY_DVR_PROBE_NET_INTERFACE
*  @brief	����ӿ�����
*/
typedef struct tagHY_DVR_PROBE_NET_INTERFACE
{ 
	unsigned char	m_bytEnable;									/*!< �Ƿ����ã�0-�����ã�1-���ã�2-δ�ҵ�	*/
	unsigned char	m_bytGetIPMode;									/*!< ��ȡ��ַ��0-�ֶ��趨��1-DHCP��ȡ		*/
	unsigned char	m_bytDevMac[DVR_MAX_MAC_NUM_LEN];				/*!< �豸MAC��ַ(Read)						*/
	unsigned char	m_bytDevIP[DVR_MAX_IP_STR_LEN];					/*!< �豸��ǰIP��ַ							*/
	unsigned char	m_bytDevMask[DVR_MAX_IP_STR_LEN];				/*!< �豸IP��ַ����							*/
	unsigned char	m_bytDevGateway[DVR_MAX_IP_STR_LEN];			/*!< �豸���ص�ַ							*/
	unsigned long	m_dwNetworkState;								/*!< ����״̬:
																			0-���ӳɹ�,
																			1-δ����,
																			2-������������,
																			3-��������ʧ��,��������,
																			4-���ڻ�ȡIP,
																			5-��ȡIPʧ��,������ȡ				*/
	unsigned long	m_dwReserve;									/*!< ��������								*/
}HY_DVR_PROBE_NET_INTERFACE, *LPHY_DVR_PROBE_NET_INTERFACE;
/*!
*  @struct	tagHY_DVR_PROBE_NET_CFG
*  @brief	̽����������
*/
typedef struct tagHY_DVR_PROBE_NET_CFG
{ 
	unsigned char	m_bytDevType;									/*!< ����������(Read)						*/
	unsigned char	m_bytDevChan;									/*!< �豸ͨ������(Read)						*/
	unsigned char	m_bytDevAutoReg;								/*!< �Ƿ������Զ�ע��						*/
	unsigned char	m_bytDevRegInterval;							/*!< �豸ע����ʱ��						*/
	HY_DVR_PROBE_NET_INTERFACE	m_cInterface[3];					/*!< ����ӿ�����							*/
	unsigned char	m_bytDevManHost[DVR_MAX_IP_STR_LEN];			/*!< �豸����������ַ						*/
	unsigned short	m_bytDevDetectPort;								/*!< �豸�Զ����ּ����˿�					*/
	unsigned short	m_wManHostPort;									/*!< �豸���������˿�						*/
	unsigned short	m_wCmdPort;										/*!< �豸����˿�							*/
	unsigned short	m_wMediaPort;									/*!< �豸ý��˿�							*/
	unsigned long	m_dwDeviceCapability[4];						/*!< �豸������λ�����eDeviceCapability����*/
	unsigned long	m_dwReserve[3];									/*!< ��������								*/
}HY_DVR_PROBE_NET_CFG, *LPHY_DVR_PROBE_NET_CFG;
//////////////////////////////////////////////////////////////////////////////
/*!
*  @struct	tagHY_DVR_WLAN_CONNECTION
*  @brief	WIFI��������
*/
typedef struct tagHY_DVR_PROBE_WLAN_CONNECTION
{
	int		m_bEnable;												/*!< WIFI״̬ 0 ������ 1 ����, 2 ����δ�ҵ�*/
	int		m_bConnect;												/*!< �Ƿ������ϣ�0��ʾδ������ 1��ʾ������(ֻ��) */
	int		m_nKeyIndex;											/*!< ��������: 1 - 4						*/
	char	m_szSSID[DVR_MAX_SSID_LEN];								/*!< �ȵ�����								*/
	char	m_szKey[DVR_MAX_WIFI_KEY_LEN];							/*!< ����									*/
	char	m_szMacAddress[DVR_MAX_MAC_STR_LEN];					/*!< �ȵ�MAC��ַ							*/
	int		m_nNetworkType;											/*!< ��������: 0��ʾManaged, 1��ʾAd-hoc	*/
	int		m_nAuthMode;											/*!< ��֤ģʽ: 0-OPEN,1-SHARED,2-WEPAUTO,3-WPAPSK,4-WPA2PSK,5-WPANONE,6-WPA,7-WPA2*/
	int		m_nEncrypType;											/*!< �����㷨 0-NONE,1-WEP,2-TKIP,3-AES		*/
	unsigned long	m_dwReserve[3];									/*!< ��������								*/
} HY_DVR_PROBE_WLAN_CONNECTION, *LPHY_DVR_PROBE_WLAN_CONNECTION;
//////////////////////////////////////////////////////////////////////////////
/*!
*  @struct	tagHY_DVR_PROBE_WLAN_ACCESSPOINT
*  @brief	WIFI�ȵ�
*/
typedef struct tagHY_DVR_PROBE_WLAN_ACCESSPOINT
{
	char szSSID[DVR_MAX_SSID_LEN];									/*!< �ȵ����� */
	char szMacAddress[DVR_MAX_MAC_STR_LEN];							/*!< �ȵ�MAC��ַ */
	int nSignal;													/*!< �ȵ��ź�ǿ�� */
	int nAuthentication;											/*!< ��֤ģʽ 0-Open,1-Shared,2-WPAPSK, 3-WPA2PSK, 4-WPANONE(ad-hocģʽ),5-WPA,6-WPA2 */
	int nEncrypType;												/*!< ����ģʽ  0��ʾNONE 1��ʾWEP 2��ʾTKIP 3��ʾAES */
	int nWirelessMode;												/*!< 1: legacy 11B only 2: legacy 11A only 3: legacy 11a/b/g mixed
																		 4: legacy 11G only 5: 11ABGN mixed    6: 11N only
																		 7: 11GN mixed      8: 11AN mixed      9: 11BGN mixed */
	unsigned long	m_dwReserve[3];									/*!< ��������								*/
} HY_DVR_PROBE_WLAN_ACCESSPOINT, *LPHY_DVR_PROBE_WLAN_ACCESSPOINT;

/*!
*  @struct	tagHY_DVR_PROBE_WLAN_ACCESSPOINT_CFG
*  @brief	WIFI�ȵ�����
*/
typedef struct tagHY_DVR_PROBE_WLAN_ACCESSPOINT_CFG
{
	int		m_nNumber;												/*!< ��Ч�ȵ����							*/
	HY_DVR_PROBE_WLAN_ACCESSPOINT m_struAccessPoints[DVR_MAX_AP_NUM];/*!< WIFI�ȵ�								*/
	unsigned long	m_dwReserve[3];									/*!< ��������								*/
} HY_DVR_PROBE_WLAN_ACCESSPOINT_CFG, *LPHY_DVR_PROBE_WLAN_ACCESSPOINT_CFG;
////////////////////////////////////////////////////////////////////////////////
///*!
//*  @struct	tagGB28181Info
//*  @brief	GB28181������Ϣ
//*/





typedef struct tagHY_DVR_GB28181_IDInfo
{
	int bValid; 
	char sID[DVR_MAX_ID_LEN];
	unsigned long	m_dwReserve[3];									/*!< ��������								*/
}HY_DVR_GB28181_IDInfo,*LPHY_DVR_GB28181_IDInfo;

typedef struct tagHY_DVR_GB28181_Info
{

	int bValid; 
	char sServerPassword[DVR_MAX_PASSWORD_LEN];			/*ע�����������*/	
	HY_DVR_GB28181_IDInfo cDeviceID;									/*�豸ID*/
	HY_DVR_GB28181_IDInfo cServerID;									/*������ID*/
	HY_DVR_GB28181_IDInfo cChannelID[DVR_MAX_VIDEOIN_NUM];				/*ͨ��ID*/
	HY_DVR_GB28181_IDInfo cAlarminID[DVR_MAX_ALARMIN_NUM];				/*��������ID*/
	HY_DVR_GB28181_IDInfo cAlarmoutID[DVR_MAX_ALARMOUT_NUM];			/*�������ID*/
	char sOwner[DVR_MAX_OWNER_LEN]; 									/*�豸����*///csst	   //+
	char sCivilCode[DVR_MAX_CIVILCODE_LEN]; 							/*��������*///100000	 //+
	char sAddress[DVR_MAX_ADDRESS_LEN]; 								/*��װ��ַ*///beijing	//+
	char sBlock[DVR_MAX_BLOCK_LEN];

	unsigned long	dwReserve[HY_DVR_DEF_RESERVE_NUM];			/*!< ����									*/
}HY_DVR_GB28181_Info,*LPHY_DVR_GB28181_Info;

#define GB28181_INFO_LEN		sizeof(HY_DVR_GB28181_Info)

#define DVR_MAX_OWNER_LEN				(128)
#define DVR_MAX_CIVILCODE_LEN			(128)
#define DVR_MAX_ADDRESS_LEN				(128)
#define DVR_MAX_BLOCK_LEN				(128)

//����ͨѶЭ��
//���1/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int portSelect; //0:Port0 1:Port1
	int ProtocolSelect;  //0:Original  1:Pelco-D
	int BaudRate;        //0:4800 bps 1:9600 2:19200 3:38400 4:57600 5:115200 15:2400
}HY_DVR_CCM_BLOCK1_SERIAL,LPHY_DVR_CCM_BLOCK1_SERIAL;
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int CommunicationID; //ͨ��ID 0~127
	int DefaultcommuunicationID; //Ĭ��ͨ��ID 0~127
	HY_DVR_CCM_BLOCK1_SERIAL strserial;
}HY_DVR_CCM_BLOCK1,*LPHY_DVR_CCM_BLOCK1;

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


//���2////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int FixShutterLevel;  //0~12(Megapixel Model Only) 0~10(WDR/Normal Model)
	int LowLightLevel; //0~12 (Megapixel Model Only)
}HY_DVR_CCM_BLOCK2_FIXSHUTTER,*LPHY_DVR_CCM_BLOCK2_FIXSHUTTER;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int LowLight; //0:Off 1:On(Megapixel Model Only)
	int AGCsetup; //0:Off 1:On(Megapixel Model Only)
	int ExtraGain; //0:Off 1:On(Megapixel Model Only)
	int SenseUpPriority; // 0:Normal 1:Swap
	int SlowShutter;  //0:Off 1:Auto
	int EnAGC;  //0:Fix 1:Auto	
}HY_DVR_CCM_BLOCK2_SENSEUP,*LPHY_DVR_CCM_BLOCK2_SENSEUP;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int WDRSetting; //0:Off 1:On
	int WDRLevel;  //0~12
}HY_DVR_CCM_BLOCK2_WDR,*LPHY_DVR_CCM_BLOCK2_WDR;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int PeakSetting; //0:Off 1:On
	int PeakLevel;  //0~8
}HY_DVR_CCM_BLOCK2_PEAK,*LPHY_DVR_CCM_BLOCK2_PEAK;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int BackLightSetting; //0:Off 1:On
	char BackLightPosition[12][16];  //
}HY_DVR_CCM_BLOCK2_BACKLIGHT,*LPHY_DVR_CCM_BLOCK2_BACKLIGHT;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int Luminance; //�ع���� 0:Manual Shutter Control 1:ALC Control 2:AES Control 3:ALC Alignment(Megapixel Model Only) Iris Adjust(AF Model Only) 4:Tracking Adjust(Af Model Only)
	int ALCControl; //�Զ����߲��� 0~16
	int AESControl; //�Զ����ӿ���
	HY_DVR_CCM_BLOCK2_FIXSHUTTER strFixShutter; //
	HY_DVR_CCM_BLOCK2_SENSEUP strSense_up;
	int AGCLevel; //0~7;
	int ManualGain; //�ֶ����� 0~15
	int SlowShutter; //������ 0~7
	HY_DVR_CCM_BLOCK2_WDR strWDR;
	HY_DVR_CCM_BLOCK2_PEAK strPeak;
	int IrisSpeed; //��Ȧ�ٶȿ��� 0:Slow 1:Middle 2:Fast
	HY_DVR_CCM_BLOCK2_BACKLIGHT strBackLight;
	int IRFilter; //0:IR Day 1:IR NIght 2:IR Compulsion 3:IR Auto
	int IRDelayTime; // 0~7
	int ALCSpeed; //0~9(Megapixel Model Only)
	int PIris; //ALC Speed Level (AF Model Only)
}HY_DVR_CCM_BLOCK2,*LPHY_DVR_CCM_BLOCK2;
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//���3////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int En_3DDNR; //3D���� 0:Off 1:On
	int En_2DDNR; //2D���� 0:Off 1:On
	int DNRLevel; //0:Off 1:Low 2:Middle 3:High
	
}HY_DVR_CCM_BLOCK3_DNR,*LPHY_DVR_CCM_BLOCK3_DNR;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int H_HDVLevel; //ˮƽ Enhance0~12
	int H_SDVLevel; //ˮƽ Enhance0~12
	int V_HDVLevel; //��ֱ Enhance0~12
	int V_SDVLevel; //��ֱ Enhance0~12
	
}HY_DVR_CCM_BLOCK3_ENHANCE,*LPHY_DVR_CCM_BLOCK3_ENHANCE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int XDRLevel; //0:Off 1:Low 2:Middle 3:High
	int FogLevel; //0:Off 1:Low 2:Middle 3:High
	int EclipseLevel; //0:Off 1:Low 2:Middle 3:High
	HY_DVR_CCM_BLOCK3_DNR strDNR;
	HY_DVR_CCM_BLOCK3_ENHANCE strEnhance;
	int Gamma;  //0:CRT Gamma 1:LCD Gamma  2:1.00
	int sharpness; //0:Off 1:On(Megapixel Model Only)
 }HY_DVR_CCM_BLOCK3,*LPHY_DVR_CCM_BLOCK3;

//���4////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	char U_matrix[4]; // U Mareix Offset Control -128~127 
	char V_matrix[4]; // V Mareix Offset Control -128~127 
	char R_matrix[4]; // R Mareix Offset Control -128~127 (Complimentary Model Only) 
	char G_matrix[4]; // G Mareix Offset Control -128~127 (Complimentary Model Only)
	char B_matrix[4]; // B Mareix Offset Control -128~127 (Complimentary Model Only)
	
	
}HY_DVR_CCM_BLOCK4_MATRIX,*LPHY_DVR_CCM_BLOCK4_MATRIX;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int ColorMode; //0:Mono 1:OFF 2:ON 3:Auto
	int ATWSet; // 0:ATW 1:ATW1 2:ATW2 (WDR/Normal Model Only)	
	int AGCLevel; //0~10 AGC Color Suppress Level Setting
	int ATWLevel; //ATW Offset Level Setting 0~16
	int AWBLevel; //AWB Offset Level Setting 0~16
	int ColorGainLevel; //Color Gain Level Setting 0~32
	int R_Y; //R-Y Offset Control 0~16
	int B_Y; //B-Y Offset Control 0~16 
	HY_DVR_CCM_BLOCK4_MATRIX strMatrix;
}HY_DVR_CCM_BLOCK4_WHITEBALANCE,*LPHY_DVR_CCM_BLOCK4_WHITEBALANCE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	HY_DVR_CCM_BLOCK4_WHITEBALANCE strWhiteBlance;
		
		
}HY_DVR_CCM_BLOCK4,*LPHY_DVR_CCM_BLOCK4;


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


//���5//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


typedef struct 
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int ZoomMode; //0:D-zoom Off 1:D-Zoom On 2:Manul Pan/Tilt 3:Auto Pan/Tilt
	int dzoomLimit; //0~5	
	int ZoomMagnitude; // Mega Pixel Model:100~400  Normal/WDR Model:100~200  AF x10 Model:0~5636 , *Optical 0~1136 ,D-Zoom 1136~5636
	int ZoomHPosition; // +/- 1/2 H Size
	int ZoomVPosition; // +/- 1/2 V Size
	int AutoHStep; //Auto PAN/TILT horizontal move speed 1~10 (Megapixel Model Only)
	int AutoVStep; //Auto PAN/TILT vertical move speed 1~10 (Megapixel Model Only)
	int ZoomSpeed; //Zoom Speed Setting 0~4 (AF Model Only)
	int ZoomResetPosition; // +/- 999 (AF Model Only)
}HY_DVR_CCM_BLOCK5_ZOOM,*LPHY_DVR_CCM_BLOCK5_ZOOM;

typedef struct 
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int FocusMode; //0:Off 1:Auto Always 2:Auto Sleep 3:After Zoom 4:One Time
	int SleepTime; //0~6
	int FocusPosition; // +/-999 (AF Model Only)
	int FocusSpeed; //0~4 (AF Model Only)
	int FocusResetPosition; // +/- 999 (AF Model Only)
	int FocusGetPosition; // +/-999 (AF Model Only)
}HY_DVR_CCM_BLOCK5_FOCUS,*LPHY_DVR_CCM_BLOCK5_FOCUS;

typedef struct 
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	HY_DVR_CCM_BLOCK5_ZOOM strZoom;
	HY_DVR_CCM_BLOCK5_FOCUS strFocus;
}HY_DVR_CCM_BLOCK5,*LPHY_DVR_CCM_BLOCK5;


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//���6////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int MaskZoomConnect; //0:Off 1:On
	char EnMask[16];// 0:Off 1:On
	char AreaMaskPosition[4][16]; 
	
}HY_DVR_CCM_BLOCK6,*LPHY_DVR_CCM_BLOCK6;




////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//���7////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int MotionDetect; // 0:Off 1:On
	int DetectTime;  // 0~2
	int SensLevel; // 0~8
	char MotionDetectArea[12][16];
}HY_DVR_CCM_BLOCK7_MOTIONDETECT,*LPHY_DVR_CCM_BLOCK7_MOTIONDETECT;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	HY_DVR_CCM_BLOCK7_MOTIONDETECT strMotionDetect;
	int Phase; //0~24 (Line-Lock Model Only)
	int VolumePriority; //0:OSD 1:Volume
	int SwitchPriority; //0:OSD 1:Switch
}HY_DVR_CCM_BLOCK7,*LPHY_DVR_CCM_BLOCK7;


//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//���8////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int DisplayAspect;  //0:Normal 1:Full
	int Display;  //0:NTSC  1:PAL
}HY_DVR_CCM_BLOCK8_DISPLAY,*LPHY_DVR_CCM_BLOCK8_DISPLAY;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int PosiNega; //0:Posi 1:Nega
	int Vreverse; //0:Off 1:On
	int HReverse; //0:Off 1:On
}HY_DVR_CCM_BLOCK8_REVERSE,*LPHY_DVR_CCM_BLOCK8_REVERSE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int FacDeMenu; // Factory Default Menu Disable 0:Off 1:On
	int Line_Lock;  //Line-Lock Menu Disable 0:Off 1:On
	int AFSwitch;  //0:Off 1:On
	int ColorBar;  // 0:Off 1:On
	int CrossLine; // 0:Off 1:On
	int DigitalZoom; //0:Off 1:On
	int MotionDetect; //0:Off 1:On
	
}HY_DVR_CCM_BLOCK8_OSD,*LPHY_DVR_CCM_BLOCK8_OSD;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int TotalCount; // 0:A 1:A~B 2:A~C 3:A~D
	int Select; //0:A 1:B 2:C 3:D
	
}HY_DVR_CCM_BLOCK8_LANGUAGE,*LPHY_DVR_CCM_BLOCK8_LANGUAGE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int TitlePosition; //0:Upper Position 1:Lower Position
	int TitleDisplay; //0:Off 1:On
	char TitleBuffer[24];
}HY_DVR_CCM_BLOCK8_TILLE,*LPHY_DVR_CCM_BLOCK8_TITLE;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int HDisplayMode; //0~6
	int SyncCode;  //0~3
	int Houtput; //0:On 1:Off
}HY_DVR_CCM_BLOCK8_HDV,*LPHY_DVR_CCM_BLOCK8_HDV;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int HDisplayMode; //0: 4:3 1: Full 2: Wide
	int Houtput; //0:On 1:Off
}HY_DVR_CCM_BLOCK8_SDV,*LPHY_DVR_CCM_BLOCK8_SDV;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	int ID; // 0:Disable 1:Enable
	int ClockPhase; // 0:posi 1:Nega
	int DigitalRange; //0:Normal 1:Full
	int DigitalFormat; //0:UYVY 1:VYUY 2:YUYV 3:YVYU(WDR/Normal)  0:UV 1:YU 2:Reserve 3:Reserve(Magapixel)
}HY_DVR_CCM_BLOCK8_DIGITAL,*LPHY_DVR_CCM_BLOCK8_DIGITAL;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
		char LuminanceMenu[2];
		char ColorMenu;
		char SetupMenu;
		char AFSpecialMenu[3];//(AF Model)
		char NMSpecialMenu[2];//(Normal Model)		
}HY_DVR_CCM_BLOCK8_OSDMENU,*LPHY_DVR_CCM_BLOCK8_OSDMENU;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
		int BlueBack; //0:Off 1:On
		int PositionLoad; //0:Off 1:On
		int AFInitEn; // AF Initialize Enable	0:Off 1:On
		int PowerPosition;  //0~63
}HY_DVR_CCM_BLOCK8_POWER,*LPHY_DVR_CCM_BLOCK8_POWER;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
		int AlarmModel; //0:Off 1:Motion Detect 2:Port Switch 3:Motion Detect & Port Switch
		int Freeze; // 0:Off 1:On
		int AlarmPosition; //0~63
}HY_DVR_CCM_BLOCK8_ALARM,*LPHY_DVR_CCM_BLOCK8_ALARM;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
		int DemoMode; //0:Off 1:On
		int StartPosition; //0~63
		int EndPosition;  //0~63
		int IntervalTime;   //Demo Interval Time  0~60
}HY_DVR_CCM_BLOCK8_DEMO,*LPHY_DVR_CCM_BLOCK8_DEMO;

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
		int WBhold;   // 0:Off 1:On
		int Irishold; //0:Off 1:On
}HY_DVR_CCM_BLOCK8_HOLD,*LPHY_DVR_CCM_BLOCK8_HOLD;


typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	
	HY_DVR_CCM_BLOCK8_DISPLAY strDisplay; // (Megapixel Model Only)
	HY_DVR_CCM_BLOCK8_REVERSE strReverse;
	HY_DVR_CCM_BLOCK8_OSD strOSD;
	int CrossLinePosition; // HP Cross Line Position VP Cross Line V Position (Megapixel Model Only)
	HY_DVR_CCM_BLOCK8_LANGUAGE strLanguage;
	HY_DVR_CCM_BLOCK8_HDV strHDV;// (Megapixel Model Only)
	HY_DVR_CCM_BLOCK8_SDV strSDV;// (Megapixel Model Only)
	HY_DVR_CCM_BLOCK8_DIGITAL strDigital;
	int Freeze;  //0:Off 1:On
	HY_DVR_CCM_BLOCK8_OSDMENU strOSDMenu;
	HY_DVR_CCM_BLOCK8_ALARM strAlarm;
	HY_DVR_CCM_BLOCK8_DEMO strDemo;
	HY_DVR_CCM_BLOCK8_HOLD strHold;
}HY_DVR_CCM_BLOCK8,*LPHY_DVR_CCM_BLOCK8;
//�ܽṹ��//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	int bValid;					//! ���ṹ���Ƿ���Ч��=0��ʾ��Ч
	HY_DVR_CCM_BLOCK1 strBlock1;
	HY_DVR_CCM_BLOCK2 strBlock2;
	HY_DVR_CCM_BLOCK3 strBlock3;
	HY_DVR_CCM_BLOCK4 strBlock4;
	HY_DVR_CCM_BLOCK5 strBlock5;
	HY_DVR_CCM_BLOCK6 strBlock6;
	HY_DVR_CCM_BLOCK7 strBlock7;
	HY_DVR_CCM_BLOCK8 strBlock8;

}HY_DVR_PTZ_CCM,*LPHY_DVR_PTZ_CCM;
//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/*!
*  @struct	tagHY_DVR_IMAGE_MODE_INFO_TIME
*  @brief	ͼ��ģʽʱ����Ϣ
*/
typedef struct tagHY_DVR_IMAGE_MODE_INFO_TIME
{
	int nHour;					//ʱ
	int nMinute;				//��
	int nSecond;				//�룬����Ϊ0
} HY_DVR_IMAGE_MODE_INFO_TIME, *LPHY_DVR_IMAGE_MODE_INFO_TIME;

/*!
*  @struct	tagHY_DVR_IMAGE_MODE_INFO
*  @brief	ͼ��ģʽ��Ϣ
*/
typedef struct tagHY_DVR_IMAGE_MODE_INFO
{
	int bValid;
	int bEnable;
	HY_DVR_IMAGE_MODE_INFO_TIME cBeginTime;
	HY_DVR_IMAGE_MODE_INFO_TIME cEndTime;
	int nBrightness;			/*����,0-255 */
	int nContrast;				/*�Աȶ�,0-255 */
	int nSaturation;			/*���Ͷ�,0-255 */
	int nHue;					/*ɫ��,0-255 */
	unsigned long	dwReserve[3];									/*!< ��������								*/
} HY_DVR_IMAGE_MODE_INFO, *LPHY_DVR_IMAGE_MODE_INFO;

/*!
*  @struct	tagHY_DVR_IMAGE_MODE
*  @brief	ͼ��ģʽ
*/
typedef struct tagHY_DVR_IMAGE_MODE
{
	int bValid;
	HY_DVR_IMAGE_MODE_INFO cImageMode[DVR_MAX_IMAGE_MODE_NUMBER];	/*!< 0: ����ģʽ 1: ҹ��ģʽ 3-15: ���� */
	unsigned long	dwReserve[3];									/*!< ��������								*/
} HY_DVR_IMAGE_MODE, *LPHY_DVR_IMAGE_MODE;
//////////////////////////////////////////////////////////////////////////////



//RIO���� 
typedef struct 
{ 
    int bValid; //! ���ṹ���Ƿ���Ч��=0��ʾ��Ч 
    int nROIQuality; //ROI ����  0������ 1-5�������ȼ� 

    HY_DVR_MOSAIC struROIArea; 
} HY_DVR_ROI_CHANNEL, *LPHY_DVR_ROI_CHANNEL;  //������һ��������������


typedef struct 
{ 
    int bValid; //! ���ṹ���Ƿ���Ч��=0��ʾ��Ч 

    HY_DVR_ROI_CHANNEL struCompressROIChannel[DVR_MAX_VIDEOIN_NUM]; 

} HY_DVR_ROI_CFG, *LPHY_DVR_ROI_CFG;

#define ROI_CFG_LEN   sizeof (HY_DVR_ROI_CFG)

#endif
