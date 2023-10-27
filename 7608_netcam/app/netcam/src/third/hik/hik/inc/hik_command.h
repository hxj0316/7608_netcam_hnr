#ifndef _HIK_COMMAND_H_
#define _HIK_COMMAND_H_

#include <linux/if_ether.h>  
#include <arpa/inet.h>

#include "hik_configtypes.h"
//#include "QMAPIType.h"

#define CURRENT_NETSDK_VERSION	NETSDK_VERSION3_1

/* network command defines */
/* �¡�������SDK�Ľӿ� */
#define NEW_NETSDK_INTERFACE	90
#define OLD_NETSDK_INTERFACE	60
#define CLIENT_SDK_VERSION		99

#define MAKE_NETSDK_VERSION(_major, _minor, _year, _month, _day)	\
	(	\
	(((_major)&0xff)<<24) | 	\
	(((_minor)&0xff)<<16) |	\
	((((_year)-2000)&0x3f)<<10)	|	\
	(((_month)&0xf)<<6) |	\
	((_day)&0x3f)	\
	)

#define GET_NETSDK_MAJOR(version)	(((version)>>24)&0xff)
#define GET_NETSDK_MINOR(version)	(((version)>>16)&0xff)
#define GET_NETSDK_YEAR(version)	((((version)>>10)&0x3f)+2000)
#define GET_NETSDK_MONTH(version)	(((version)>>6)&0xf)
#define GET_NETSDK_DAY(version)		((version)&0x3f)

#define GET_NETSDK_BUILDDATE(version)	((version)&0xffff)
#define RELOGIN_FLAG					0x80000000
#define NETSDK_VERSION1_0	MAKE_NETSDK_VERSION(1, 0, 2004, 10, 4)
#define NETSDK_VERSION1_2	MAKE_NETSDK_VERSION(1, 2, 2005, 3, 15)		/* �û���/������ܣ�Ӳ����Ϊ16 */
#define NETSDK_VERSION1_3	MAKE_NETSDK_VERSION(1, 3, 2005, 4, 1)		/* �ƶ����/��Ƶ��ʧ/�ڵ���������ʱ��� */
#define NETSDK_VERSION1_4	MAKE_NETSDK_VERSION(1, 4, 2005, 5, 30)		/* Ԥ�õ���Ŀ 16 -> 128 */
#define NETSDK_VERSION1_5	MAKE_NETSDK_VERSION(1, 5, 2005, 12, 28)		/* �û���Ȩ��ϸ����ͨ��(���ػطš�Զ�̻طš�Զ��Ԥ��)*/
#define NETSDK_VERSION2_0   MAKE_NETSDK_VERSION(2, 0, 2006, 4, 27)
#define NETSDK_VERSION2_1	MAKE_NETSDK_VERSION(2, 1, 2006, 8, 14)		/* ����汾���ϵ�SDK��Ҫ������֤ */
#define NETSDK_VERSION3_0	MAKE_NETSDK_VERSION(3, 0, 2008, 2, 28)		
#define NETSDK_VERSION3_1	MAKE_NETSDK_VERSION(3, 1, 2009, 7, 30)		
#define NETSDK_VERSION4_0   MAKE_NETSDK_VERSION(4, 0, 2010, 3, 1)
#define NETSDK_VERSION5_1   MAKE_NETSDK_VERSION(5, 1, 2015, 5, 11)

/* define status that return to client */
#define NETRET_QUALIFIED		1
#define NETRET_EXCHANGE			2
#define NETRET_ERRORPASSWD		3
#define NETRET_LOWPRI			4
#define NETRET_OPER_NOPERMIT	5
#define NETRET_VER_DISMATCH		6
#define NETRET_NO_CHANNEL		7
#define NETRET_NO_SERIAL		8
#define NETRET_NO_ALARMIN		9
#define NETRET_NO_ALARMOUT		10
#define NETRET_NO_DISK			11
#define NETRET_NO_HARDDISK		12
#define NETRET_NOT_SUPPORT		13
#define NETRET_ERROR_DATA		14
#define NETRET_CHAN_ERROR		15
#define NETRET_DISK_ERROR		16
#define NETRET_CMD_TIMEOUT		17
#define NETRET_MAXLINK			18
#define NETRET_NEEDRECVHEADER	19
#define NETRET_NEEDRECVDATA		20
#define NETRET_NEEDRECVDATA_V30	104
#define NETRET_IPCCFG_CHANGE	105
#define NETRET_IPCCFG_CHANGE_V31	106
#define NETRET_KEEPAUDIOTALK    NETRET_EXCHANGE
#define NETRET_RECVKEYDATA      90       	/* SETPOS����շ��������ǹؼ�֡���ݣ���ǰ��I֡��*/
#define NETRET_RECVSETPOSDATA   91       	/* SETPOS����ոı�λ�ú�ʵ��λ�õ����� */
#define NETRET_SAME_USER        49
#define NETRET_DEVICETYPE_ERROR 50			/*�������ʱ�豸�ͺŲ�ƥ��*/
#define NETRET_LANGUAGE_ERROR   51 			/*�������ʱ������ƥ��*/
#define NETRET_PARAVERSION_ERROR	52		/*�������ʱ����汾��ƥ��*/
#define NETRET_IPC_NOTCONNECT   53			/*IPC������*/
/*ȥ��NETRET_720P_SUB_NOTSUPPORT, ��Ҫʱ���� NETRET_NOT_SUPPORT*/
//#define NETRET_720P_SUB_NOTSUPPORT         54/*720P ��������֧��*/
#define NETRET_IPC_COUNT_OVERFLOW	54		/*IPC�������*/
#define NETRET_EMAIL_TEST_ERROR		55		/*�ʼ�����ʧ�� 9000_1.1*/

#define NETRET_PLAY_END			21
#define NETRET_WRITEFLASHERROR	22
#define NETRET_UPGRADE_FAILED	23
#define NETRET_UPGRADING		24
#define NETRET_NEEDWAIT			25
#define NETRET_FILELISTOVER		26
#define NETRET_RECVFILEINFO		27
#define NETRET_FORMATING		28
#define NETRET_FORMAT_END		29
#define NETRET_NO_USERID		30
#define NETRET_EXCEED_MAX_USER	31
#define NETRET_DVR_NO_RESOURCE	32
#define NETRET_DVR_OPER_FAILED	33			/* operation failed */
#define NETRET_IP_MISMATCH		34
#define NETRET_SOCKIP_MISMATCH	35
#define NETRET_MAC_MISMATCH		36
#define NETRET_ENC_NOT_STARTED	37
#define NETRET_LANG_MISMATCH	38			/* ����ʱ���Բ�ƥ�� */
#define NETRET_NEED_RELOGIN		39			/* ��Ҫ��������/�û���������֤ */
#define NETRET_HD_READONLY		48


#define NETRET_IPCAMERA 		30 			/*IP �����*/
#define NETRET_MEGA_IPCAMERA 	31  		/*��������IP �����*/
#define NETRET_IPDOME 			40 			/*IP ����*/
#define NETRET_IPMOD 			50 			/*IP ģ��*/

#define ABILITY_NOT_SUPPORT		1000		//�豸��������֧��

typedef struct _MAC_FRAME_HEADER
{
	char m_cDstMacAddress[6];		//Ŀ��mac��ַ
	char m_cSrcMacAddress[6];		//Դmac��ַ
	short m_cType;					//��һ��Э�����ͣ���0x0800������һ����IPЭ�飬0x0806Ϊarp
}MAC_FRAME_HEADER,*PMAC_FRAME_HEADER;

#pragma pack(2)
typedef struct
{
	int		m_ServiceType;
	int		m_Seqnum;
	int		m_CmdType;
	short	m_Checksum;

	char	m_SrcMac[6];
	int		m_SrcIp;
	char	m_DstMac[6];
	int		m_DstIp;
	char	m_res[32];
}NET_CLIENT_BROADCAST;

typedef struct
{
	int		m_ServiceType;
	int		m_Seqnum;
	int		m_CmdType;
	short	m_Checksum;

	char	m_SrcMac[6];
	int		m_SrcIp;
	char	m_DstMac[6];
	int		m_DstIp;

	int		m_DstMaskip;
	char    m_unkown[18];
	short		m_port;

	char	m_res[8];
}NET_BROADCAST_FIX_IP;


typedef struct
{
	int		m_ServiceType;
	int		m_Seqnum;
	int		m_CmdType;
	unsigned short	m_Checksum;

	char	m_SrcMac[6];
	int		m_SrcIp;
	char	m_DstMac[6];
	int		m_DstIp;
	int		m_SrcNetmask;
	char	m_DevSerialNum[48];

	int		m_unknown1;
	int		m_ServicePort;
	int		m_ChannelNum;
	int		m_unuse1;

	char	m_SoftwareVersion[48];
	char	m_DspVersion[48];
	char	m_Starttime[48];	//У����㵽�˳�ԱΪֹ

	int		m_unknown3;
	int		m_SrcGateway;

	char	m_unuse2[32];

	int		m_unknown4;
	char	m_unuse3[28];
	int		m_unknown5;
	char	m_unuse4[32];

	char	m_DeviceType[48];
}NET_BROADCAST;

//���ظ��ͻ���״̬��ͷ 
typedef struct{
	UINT32	length;
	UINT32	checkSum;
	UINT32	retVal;
	UINT8	res[4];
}NETRET_HEADER;

//login
typedef struct{
	UINT32 	length;
	UINT8	ifVer;
	UINT8	res1[3];
	UINT32	checkSum;
	UINT32	netCmd;
	UINT32	version;
	UINT8	res2[4];
	UINT32	clientIp;
	UINT8	clientMac[6];
	UINT8	res3[2];
	UINT8	username[NAME_LEN];
	UINT8	password[PASSWD_LEN];
}NET_LOGIN_REQ;

typedef struct{
	UINT32	length;
	UINT32	checkSum;
	UINT32	retVal;
	UINT32  devSdkVer;
	DWORD	dwUserID;
	BYTE	sSerialNumber[SERIALNO_LEN];
	BYTE	byDVRType;
	BYTE	byChanNum;
	BYTE	byStartChan;
	BYTE	byAlarmInPortNum;
	BYTE	byAlarmOutPortNum;
	BYTE	byDiskNum;
	BYTE	byRes1[2];
	BYTE    byRes2[32];		//��32�ֽ�����Ӧ�úͿͻ��˵İ汾�й�
}NET_LOGIN_RET;

typedef struct
{
	DWORD	netCmd;
	char    CmdType[100];		//�豸�������������������,�����С��֪���ݶ�
}NET_DEVABILITY_RET;

typedef struct{
	UINT32 	length;
	UINT8  	ifVer;
	UINT8	res1[3];
	UINT32	checkSum;
	UINT32	netCmd;
	UINT32	clientIp;
	UINT32	userID;	
	UINT8	clientMac[6];
	UINT8	res2[2];
	UINT8	res3[20];
	char	CmdType[100];
}NET_ISAPIABILITY;

typedef struct{
	DWORD	dwsize;
	DWORD	unknow1;
	DWORD	unknow2;
	char	data[4096];
}NET_ISAPIABILITY_RET;

typedef struct {
		UINT32	length;			/* ���������ܳ��� */
		UINT32  checkSum;		/* ����У��� */
		UINT32  retVal;			/* ����״̬���ض��ı�ʶ��ʾ��9000�豸 */
		UINT32  devSdkVer;
		UINT8	challenge[60];	/* BASE64����֮�����ս�� */
		//UINT8	challenge[80];	/* BASE64����֮�����ս�� */ //ץ�������˽ṹ���СΪ96
} NET_LOGIN_CHALLENGE;

typedef struct{
	UINT32	userID;
	UINT8	serialno[SERIALNO_LEN];
	UINT8	devType;
	UINT8	channelNums;
	UINT8	firstChanNo;
	UINT8	alarmInNums;
	UINT8	alarmOutNums;
	UINT8	hdiskNums;
	UINT8	res2[2];
}NET_LOGIN_RETDATA;

/*
 * ========================================================
 *			IPC parameter 
 * ========================================================
 */
typedef struct{
	UINT32 enable;
	char username[NAME_LEN];
	char password[PASSWD_LEN];	
	U_IN_ADDR ip;
	UINT16 cmdPort;
	UINT8 factory;
	UINT8 res[33];
}SINGLE_IPDEV_CFG;

typedef struct{
	UINT8 enable;
	UINT8 factory;
	UINT8 res1[2];
	char username[NAME_LEN];
	char password[PASSWD_LEN];
	char domainname[MAX_DOMAIN_NAME];
	U_IN_ADDR ip;
	UINT16 cmdPort;
	UINT8 res2[34];
}SINGLE_IPDEV_CFG_V31;

typedef struct{
	SINGLE_IPDEV_CFG  ipDevInfo[MAX_IP_DEVICE];
}NETPARAM_IPDEV_CFG;

typedef struct{
	SINGLE_IPDEV_CFG_V31  ipDevInfo[MAX_IP_DEVICE];
}NETPARAM_IPDEV_CFG_V31; 

typedef struct{
	UINT8 enable;
	UINT8 ipID;
	UINT8 channel;
	UINT8 res[33];
}SINGLE_IPCHAN_CFG;

typedef struct{
	UINT8 analogChanEnable[MAX_ANALOG_CHANNUM/8];
	SINGLE_IPCHAN_CFG  ipChanInfo[MAX_IP_CHANNUM];
}NETPARAM_IPCHAN_CFG;

typedef struct{
	UINT32 length;
	NETPARAM_IPDEV_CFG  ipDevCfg;
	NETPARAM_IPCHAN_CFG ipChanCfg;
}NETPARAM_IPCORE_CFG;

typedef struct{
	UINT32 length;
	NETPARAM_IPDEV_CFG_V31  ipDevCfg;
	NETPARAM_IPCHAN_CFG ipChanCfg;
}NETPARAM_IPCORE_CFG_V31;



typedef struct{
	UINT8 ipID;
	UINT8 alarmIn;
	UINT8 res[18];
}SINGLE_IPALARMIN_CFG;

typedef struct{
	UINT32 length;
	SINGLE_IPALARMIN_CFG ipAlarmInInfo[MAX_IP_ALARMIN];
}NETPARAM_IPALARMIN_CFG;

typedef struct{
	UINT8 ipID;
	UINT8 alarmOut;
	UINT8 res[18];
}SINGLE_IPALARMOUT_CFG;

typedef struct{
	UINT32 length;
	SINGLE_IPALARMOUT_CFG ipAlarmOutInfo[MAX_IP_ALARMOUT];
}NETPARAM_IPALARMOUT_CFG;

typedef struct{
	NETPARAM_IPDEV_CFG  ipDevCfg;
	NETPARAM_IPCHAN_CFG ipChanCfg;
	SINGLE_IPALARMIN_CFG ipAlarmInInfo[MAX_IP_ALARMIN];
	SINGLE_IPALARMOUT_CFG ipAlarmOutInfo[MAX_IP_ALARMOUT];
}NETRET_IPC_ALARMINFO;

typedef struct{
	NETPARAM_IPDEV_CFG_V31  ipDevCfg;
	NETPARAM_IPCHAN_CFG ipChanCfg;
	SINGLE_IPALARMIN_CFG ipAlarmInInfo[MAX_IP_ALARMIN];
	SINGLE_IPALARMOUT_CFG ipAlarmOutInfo[MAX_IP_ALARMOUT];
}NETRET_IPC_ALARMINFO_V31;

typedef struct{
	UINT32	alarmType;
	UINT8	alarmInStatus[32];	    /* ��λ,��0λ��Ӧ��0������˿� */
	UINT8	alarmOutStatus[16];		/* �������봥���ı����������λ,��0λ��Ӧ��0������˿�, alarmTypeΪ0ʱ��Ҫ */
	UINT8   triggeredRecChan[16];		/* �������봥����¼��ͨ������λ,��0λ��ʾ��0��ͨ�� */
	UINT8	channelNo[16];				/* ��λ,��0λ��Ӧ��0��ͨ����alarmTypeΪ2��3,6ʱ��Ҫ���� */
	UINT8	diskNo[16];					/* ��λ,��0λ��Ӧ��0��Ӳ��,dwAlarmTypeΪ4,5ʱ��Ҫ���� */
}NETRET_ALARMINFO_V30;

typedef struct{
	UINT32	alarmType;
	UINT32	alarmInNumber;			/* ��λ,��0λ��Ӧ��0������˿� */
	UINT32	triggeredAlarmOut;		/* �������봥���ı����������λ,��0λ��Ӧ��0������˿�, alarmTypeΪ0ʱ��Ҫ */
	UINT16	triggeredIPCAlarmOut[MAX_IPCHANNUM];		/* IPC �������봥���ı����������λ,��0λ��Ӧ��0������˿�, alarmTypeΪ0ʱ��Ҫ */
	UINT64	triggeredRecChan;		/* �������봥����¼��ͨ������λ,��0λ��ʾ��0��ͨ�� */
	UINT64	channelNo;				/* ��λ,��0λ��Ӧ��0��ͨ����alarmTypeΪ2��3,6ʱ��Ҫ���� */
	UINT64	diskNo;					/* ��λ,��0λ��Ӧ��0��Ӳ��,dwAlarmTypeΪ4,5ʱ��Ҫ���� */
}NETRET_ALARMINFO;


#define MAX_ALARMINFO_MSGQ_NUM	32	
#define MAX_ALARMINFO_MSGQ_LEN	sizeof(NETRET_ALARMINFO)
#define ALARMINFO_MSG_PATH  "/alarmInfo-msg"
#if 0
typedef struct{
	NODE node;
	UINT32 userID;
	int  connfd;
	mqd_t alarmInfoMsgQ;
}ALARM_UP_CONNS;
#endif

/* DVR ����״̬ */
typedef struct{
	UINT8	bRecStarted;		/* �Ƿ�¼��1--¼��/0--��¼�� */
	UINT8	bViLost;			/* VI�źŶ�ʧ��0--����/1--��ʧ */
	UINT8	chanStatus;			/* ͨ��Ӳ��״̬��0--����/1--�쳣 */
	UINT8	res1;
	UINT32	bitRate;			/* ʵ������ */
	UINT32	netLinks;			/* �ͻ��������� */
	UINT32	clientIP[6];	/* �ͻ���IP��ַ */
}ENC_CHAN_STATUS;

typedef struct{
	UINT8	bRecStarted;		/* �Ƿ�¼��1--¼��/0--��¼�� */
	UINT8	bViLost;			/* VI�źŶ�ʧ��0--����/1--��ʧ */
	UINT8	chanStatus;			/* ͨ��Ӳ��״̬��0--����/1--�쳣 */
	UINT8	res1;
	UINT32	bitRate;			/* ʵ������ */
	UINT32	netLinks;			/* �ͻ��������� */
	U_IN_ADDR clientIP[6];	/* �ͻ���IP��ַ */
	UINT32	ipcNetLinks;			/* IPC ������ */
	UINT8  	res[12];
}ENC_CHAN_STATUS_V30;

typedef struct{
	UINT32	totalSpace;
	UINT32	freeSpace;
	UINT32	diskStatus;			/* bit 0 -- idle, bit 1 -- error */
}HDISK_STATUS;

typedef struct{
	//NODE node;
	UINT32 userID;
	int  connfd;
	//mqd_t alarmInfoMsgQ;
}ALARM_UP_CONNS;


typedef struct{
	UINT32	deviceStatus;			/* �豸��״̬,0-����,1-CPUռ����̫��,����85%,2-Ӳ������,���紮������ */
	HDISK_STATUS hdStatus[16];
	ENC_CHAN_STATUS chanStatus[MAX_CHANNUM_8000];
	UINT8  	alarmInStatus[MAX_ALARMIN_8000];
	UINT8	alarmOutStatus[MAX_ALARMOUT_8000];
	UINT32  localDispStatus;		/* ������ʾ״̬,0-����,1-������ */
}DVR_WORKSTATUS;

typedef struct{
	UINT32		 deviceStatus;			/* �豸��״̬,0-����,1-CPUռ����̫��,����85%,2-Ӳ������,���紮������ */
	HDISK_STATUS hdStatus[16];
	ENC_CHAN_STATUS_V30 chanStatus[16];
	UINT8  	alarmInStatus[16];
	UINT8	alarmOutStatus[16];
	UINT32  localDispStatus;		/* ������ʾ״̬,0-����,1-������ */
	UINT8   audioInChanStatus;
	UINT8   res[35];
}DVR_WORKSTATUS_V30;


//device parameter
typedef struct{
	UINT32 	length;
	UINT8	DVRName[NAME_LEN];
	UINT32	deviceID;
	UINT32	recycleRecord;
	UINT8	serialno[SERIALNO_LEN];
	UINT32	softwareVersion;
	UINT32	softwareBuildDate;
	UINT32	dspSoftwareVersion;
	UINT32	dspSoftwareBuildDate;
	UINT32	panelVersion;
	UINT32	hardwareVersion;
	UINT8	alarmInNums;
	UINT8	alarmOutNums;
	UINT8	rs232Nums;
	UINT8	rs485Nums;
	UINT8	netIfNums;
	UINT8	hdiskCtrlNums;
	UINT8	hdiskNums;
	UINT8	devType;
	UINT8	channelNums;
	UINT8	firstChanNo;
	UINT8	decodeChans;
	UINT8	vgaNums;
	UINT8	usbNums;
	UINT8	auxOutNum;
	UINT8	audioNum;
	UINT8 	ipChanNum;
}NETPARAM_DEVICE_CFG;

typedef struct{
    UINT32  length;
    UINT8   DVRName[NAME_LEN];
    UINT32  deviceID;
    UINT32  recycleRecord;
    UINT8   serialno[SERIALNO_LEN];
    UINT32  softwareVersion;
    UINT32  softwareBuildDate;
    UINT32  dspSoftwareVersion;
    UINT32  dspSoftwareBuildDate;
    UINT32  panelVersion;
    UINT32  hardwareVersion;
    UINT8   alarmInNums;
    UINT8   alarmOutNums;
    UINT8   rs232Nums;
    UINT8   rs485Nums;
    UINT8   netIfNums;
    UINT8   hdiskCtrlNums;
    UINT8   hdiskNums;
    UINT8   unknown1;
    UINT8   channelNums;
    UINT8   firstChanNo;
    UINT8   decodeChans;
    UINT8   vgaNums;
    UINT8   usbNums;
    UINT8   auxOutNum;
    UINT8   audioNum;
    UINT8   ipChanNum;
    UINT8   zeroChanNum;        /* ��ͨ��������� */
    UINT8   supportAbility;     /* ������λ����Ϊ0��ʾ��֧�֣�1��ʾ֧�֣�
                                 supportAbility & 0x1, ��ʾ�Ƿ�֧����������
                                supportAbility & 0x2, ��ʾ�Ƿ�֧��һ������
                                supportAbility & 0x4, ��ʾ�Ƿ�֧��ѹ������������ȡ
                                supportAbility & 0x8, ��ʾ�Ƿ�֧��˫���� 
                                supportAbility & 0x10, ��ʾ֧��Զ��SADP 
                                supportAbility & 0x20  ��ʾ֧��Raid����*/
    
    UINT8   bESataFun;          /* eSATA��Ĭ����;��0-Ĭ��¼��1-Ĭ�ϱ��� */
    UINT8   bEnableIPCPnp;      /* 0-��֧�ּ��弴�ã�1-֧�ּ��弴�� */
    UINT8   unknown2;
    UINT8   unknown3;
    UINT8   unknown4;
	UINT8   CPUType;
    UINT8   devType[40];            /* ���� */
}NETPARAM_DEVICE_CFG_V40;

/*
 * ====================================================================================
 *									 net data struct
 * ====================================================================================
 */
typedef struct{	//sizeof = 32
	UINT32 	length;				/* total length */
	UINT8  	ifVer;				/* version: 90 -- new interface/60 -- old interface */
	UINT8	res1[3];
	UINT32	checkSum;			/* checksum */
	UINT32	netCmd;				/* client request command */
	UINT32	clientIp;			/* clinet IP address */
	UINT32	userID;				/* user ID */
	UINT8	clientMac[6];
	UINT8	res[2];
}NETCMD_HEADER;

typedef struct{
	NETCMD_HEADER header;
	UINT32 channel;
}NETCMD_CHAN_HEADER;

typedef struct{
	UINT32 		handleType;							/* �쳣����,�쳣����ʽ��"��"��� */
	UINT8		alarmOutTrigStatus[16];				/* �������ı������(�˴�������󱨾������Ϊ32) */
}NETPARAM_EXCEPTION_V30;

typedef struct{
	UINT8	startHour;
	UINT8	startMin;
	UINT8	stopHour;
	UINT8	stopMin;
}NETPARAM_TIMESEG;

typedef struct{
	UINT8		alarmInName[NAME_LEN];				/* ���� */
	UINT8		sensorType;							/* ���������ͣ�0Ϊ������1Ϊ���� */
	UINT8		bEnableAlarmIn;						/* ���������� */
	UINT8		res[2];
	NETPARAM_EXCEPTION_V30 alarmInAlarmHandleType;				/* �������봦�� */
	NETPARAM_TIMESEG armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ����ʱ��� */
	UINT8		recordChanTriggered[16];			/* ��������¼��(�˴��������ͨ����Ϊ128) */
	UINT8		bEnablePreset[16];					/* �Ƿ����Ԥ�õ� */
	UINT8		presetNo[16];						/* ���õ���̨Ԥ�õ����,һ������������Ե���
												   	   ���ͨ������̨Ԥ�õ�, 0xff��ʾ������Ԥ�õ㡣
												   	 */
	UINT8		bEnablePresetRevert[16];	
	UINT16		presetRevertDelay[16];
	UINT8       bEnablePtzCruise[16];				/* �Ƿ����Ѳ�� */
	UINT8		ptzCruise[16];						/* Ѳ�� */
	UINT8       bEnablePtzTrack[16];				/* �Ƿ���ù켣 */
	UINT8		trackNo[16];						/* ��̨�Ĺ켣���(ͬԤ�õ�) */
}NETPARAM_ALARMIN_V30;

typedef struct{
	UINT32	length;
	NETPARAM_ALARMIN_V30 alarmIn;
}NETPARAM_ALARMIN_CFG_V30;

/*
 * ========================================================
 *			 alarmOut parameter 
 * ========================================================
 */
typedef struct{
	UINT8	alarmOutName[NAME_LEN];				/* ���� */
	UINT32	alarmOutDelay;
	NETPARAM_TIMESEG alarmOutTimeSeg[MAX_DAYS][MAX_TIMESEGMENT_8000];		/* �����������ʱ��� */
}NETPARAM_ALARMOUT;

typedef struct{
	UINT32 length;
	NETPARAM_ALARMOUT alarmOut;
}NETPARAM_ALARMOUT_CFG;

typedef struct{
	UINT8	alarmOutName[NAME_LEN];				/* ���� */
	UINT32	alarmOutDelay;
	NETPARAM_TIMESEG alarmOutTimeSeg[MAX_DAYS][MAX_TIMESEGMENT];		/* �����������ʱ��� */
}NETPARAM_ALARMOUT_V30;

typedef struct{
	UINT32 length;
	NETPARAM_ALARMOUT_V30 alarmOut;
	UINT8 res[16];
}NETPARAM_ALARMOUT_CFG_V30;


#define CLOSE_ALL		0	/* �ر�(��Ͽ�)���п��� */
#define CAMERA_PWRON	1	/* ��ͨ�������Դ */
#define LIGHT_PWRON		2	/* ��ͨ�ƹ��Դ */
#define WIPER_PWRON		3	/* ��ͨ��ˢ���� */
#define FAN_PWRON		4	/* ��ͨ���ȿ��� */
#define HEATER_PWRON	5	/* ��ͨ���������� */
#define AUX_PWRON1		6	/* ��ͨ�����豸���� */
#define AUX_PWRON2		7	/* ��ͨ�����豸���� */
#define SET_PRESET		8	/* ����Ԥ�õ� */
#define CLE_PRESET		9	/* ���Ԥ�õ� */

#define STOP_ALL		10	/* ֹͣ����������(��ͷ, ��̨)���� */
#define ZOOM_IN			11	/* �������ٶ�SS���(���ʱ��) */
#define ZOOM_OUT		12	/* �������ٶ�SS��С(���ʱ�С) */
#define FOCUS_IN		13	/* �������ٶ�SSǰ�� */
#define FOCUS_OUT		14	/* �������ٶ�SS��� */
#define IRIS_ENLARGE	15	/* ��Ȧ���ٶ�SS���� */
#define IRIS_SHRINK		16	/* ��Ȧ���ٶ�SS��С */
#define AUTO_ZOOM		17	/* ���Զ�����(�Զ�����) */
#define AUTO_FOCUS		18	/* ���Զ����� */
#define AUTO_IRIS		19	/* ���Զ���Ȧ */

#define TILT_UP			21	/* ��̨��SS���ٶ����� */
#define TILT_DOWN		22	/* ��̨��SS���ٶ��¸� */
#define PAN_LEFT		23	/* ��̨��SS���ٶ���ת */
#define PAN_RIGHT		24	/* ��̨��SS���ٶ���ת */
#define UP_LEFT			25	/* ��̨��SS���ٶ���������ת */
#define UP_RIGHT		26	/* ��̨��SS���ٶ���������ת */
#define DOWN_LEFT		27	/* ��̨��SS���ٶ��¸�����ת */
#define DOWN_RIGHT		28	/* ��̨��SS���ٶ��¸�����ת */
#define AUTO_PAN		29	/* ��̨��SS���ٶ������Զ�ɨ�� */

#define FILL_PRE_SEQ	30	/* ��Ԥ�õ����Ѳ������ */
#define SET_SEQ_DWELL	31	/* ����Ѳ����ͣ��ʱ�� */
#define SET_SEQ_SPEED	32	/* ����Ѳ���ٶ� */
#define CLE_PRE_SEQ		33	/* ��Ԥ�õ��Ѳ���ٶ���ɾ�� */
#define STA_MEM_CRUISE	34	/* Set Cruise start memorize */
#define STO_MEM_CRUISE	35	/* Set Cruise stop memorize */
#define RUN_CRUISE		36	/* ��ʼ�켣 */
#define RUN_SEQ			37	/* ��ʼѲ�� */
#define STOP_SEQ		38	/* ֹͣѲ�� */
#define GOTO_PRESET		39  /* ����ת��Ԥ�õ� */
#define SYSTEM_RESET	40	/* ϵͳ��λ */

//#define CLICK_ZOOM		41	/*������¼�*/
//#define BEG_ADD_KEYPT	42	/*��ʼ��ӹؼ���*/
#define BEG_ADD_PRESET_TO_CRUISE    41 /* ��Ԥ�õ����Ѳ������ */
#define END_ADD_PRESET_TO_CRUISE	42 /* ��Ԥ�õ����Ѳ������ */

#define END_ADD_KEYPT	43	/*������ӹؼ���*/
#define STOP_CRUISE     	44 	/*ɾ��Ѳ��·��*/
#define MENU_MODE       45 	/*	����˵�ģʽ*/
#define Clean_All_PreSet  46 /*	������е�Ԥ�õ�����*/
/* PTZ control when preview*/
typedef struct{
	NETCMD_HEADER header;
	UINT32 channel;
	UINT32 command;
	union{
	UINT32 presetNo;
	UINT32 speed;
	};
}NET_PTZ_CTRL_DATA;


/*
 * ========================================================
 *					network parameter
 * ========================================================
 */
//8000 network parameter 
typedef struct{
	UINT32	devIp;
	UINT32	devIpMask;
	UINT32	mediaType;		/* network interface type */
	UINT16	ipPortNo;		/* command port */
	UINT16	mtu;			/* MTU */
	UINT8	macAddr[6];
	UINT8	res2[2];
}NETPARAM_ETHER_CFG;

typedef struct{
	UINT32	length;
	NETPARAM_ETHER_CFG etherCfg[MAX_ETHERNET];
	UINT32	manageHostIp;
	UINT16	manageHostPort;
	UINT16	httpPort;
	UINT32	ipResolverIpAddr;
	UINT32	mcastAddr;
	UINT32	gatewayIp;
	UINT32	nfsIp;
	UINT8	nfsDirectory[PATHNAME_LEN];
	UINT32	bEnablePPPoE;
	UINT8	pppoeName[NAME_LEN];
	UINT8	pppoePassword[PASSWD_LEN];
	UINT8	res2[4];
	UINT32	pppoeIp;
}NETPARAM_NETWORK_CFG;



typedef struct{
	UINT8	streamType;
	UINT8	resolution;
	UINT8	bitrateType;
	UINT8	quality;
	UINT32	maxBitRate;
	UINT32	videoFrameRate;
	UINT16  intervalFrameI;
	UINT8	BFrameNum;			/* B֡����: 0:BBP, 1:BP, 2:P */
	UINT8   EFrameNum;
	UINT8   videoEncType; //��Ƶ�����ʽ
	UINT8   audioEncType;
	UINT8   res[10];
}NETPARAM_COMP_PARA_V30;

typedef struct{
	UINT32	length;
	NETPARAM_COMP_PARA_V30 normHighCompPara;
	NETPARAM_COMP_PARA_V30 normLowCompPara;
	NETPARAM_COMP_PARA_V30 eventCompPara;
	NETPARAM_COMP_PARA_V30 netCompPara;
}NETPARAM_COMPRESS_CFG_V30;



typedef struct{
	BYTE byStreamType; //�������� 1-��Ƶ��,3-������
	BYTE byResolution; //�ֱ���0-DCIF, 1-CIF, 2-QCIF, 3-4CIF, 4-2CIF
	BYTE byBitrateType; //�������� 0:�����ʣ� 1:������
	BYTE byPicQuality; //ͼ������ 0-��� 1-�κ� 2-�Ϻ� 3-һ�� 4-�ϲ� 5-��
	DWORD dwVideoBitrate; //��Ƶ���� 0-������ 1-16K 2-32K 3-48k 4-64K 5-80K
							//6-96K 7-128K 8-160k 9-192K 10-224K 11-256K 12-320K
							//13-384K 14-448K 15-512K 16-640K 17-768K 18-896K
							//19-1024K 20-1280K 21-1536K 22-1792K 23-2048K
							//���λ(32 λ)�ó� 1 ��ʾ���Զ�������, 0-31 λ��ʾ����ֵ(��Сֵ 16k�����ֵ 8192k)��
	DWORD dwVideoFrameRate; //֡�� 0-ȫ��; 1-1/16; 2-1/8; 3-1/4; 4-1/2; 5-1; 6-2; 7-4; 8-6;
							//9-8; 10-10; 11-12; 12-16; 13-20
}INTER_COMPRESSION_INFO, *LPINTER_COMPRESSION_INFO;

typedef struct{
	DWORD dwSize;
	INTER_COMPRESSION_INFO struRecordPara; /* ¼������¼�����¼����� */
	INTER_COMPRESSION_INFO struNetPara; /* �������¼�����¼�����ʱ������� */
}INTER_COMPRESSIONCFG, *LPINTER_COMPRESSIONCFG;


typedef struct
{
	BYTE byStreamType; //�������� 1-��Ƶ��,3-������
	BYTE byResolution; //�ֱ���0-DCIF, 1-CIF, 2-QCIF, 3-4CIF, 4-2CIF
						//16-VGA, 17-UXGA, 18-SVGA, 19-HD720p (16-19�����
						//����)
	BYTE byBitrateType; //�������� 0:�����ʣ� 1:������
	BYTE byPicQuality; //ͼ������ 0-��� 1-�κ� 2-�Ϻ� 3-һ�� 4-�ϲ� 5-��
	DWORD dwVideoBitrate; //��Ƶ���� 0-���� 1-16K 2-32K 3-48k 4-64K 5-80K 6-96K
							//7-128K 8-160k 9-192K 10-224K 11-256K 12-320K
							// 13-384K 14-448K 15-512K 16-640K 17-768K 18-896K
							//19-1024K 20-1280K 21-1536K 22-1792K 23-2048K
							//���λ(31λ)�ó�1��ʾ���Զ�������, 0-30λ��ʾ����ֵ��
	DWORD dwVideoFrameRate; //֡�� 0-ȫ��; 1-1/16; 2-1/8; 3-1/4; 4-1/2; 5-1; 6-2; 7-4;
							//8 - 6; 9 - 8; 10 - 10; 11 - 12; 12 - 16; 13 - 20; V2.0�汾���¼�14 - 15; 15 - 18; 16 - 22;
	WORD wIntervalFrameI; //I֡���
							//BYTE byRes[2];
							//2006-08-11 ���ӵ�P֡�����ýӿڣ����Ը���ʵʱ����ʱ����
	BYTE byIntervalBPFrame;//0-BBP֡; 1-BP֡; 2-��P֡
	BYTE byENumber;//E֡����
}INTER_COMPRESSION_INFO_EX, *LPINTER_COMPRESSION_INFO_EX;

typedef struct
{
	DWORD dwSize;
	INTER_COMPRESSION_INFO_EX struRecordPara; //¼��
	INTER_COMPRESSION_INFO_EX struNetPara;//����
}INTER_COMPRESSIONCFG_EX, *LPINTER_COMPRESSIONCFG_EX;


////////////////////////////////////////////////////////////////
typedef struct{
	DWORD dwAlarmType;/*0-�ź�������,1-Ӳ����,2-�źŶ�ʧ�� 3���ƶ���⣬ 4��Ӳ��δ
					  ��ʽ��,5-дӲ�̳���,6-�ڵ������� 7-��ʽ��ƥ��, 8-�Ƿ�����*/
	DWORD dwAlarmInputNumber;/*��λ,�� 0 λ��Ӧ�� 0 ������˿�,dwAlarmType Ϊ 0 ʱ
							 ��Ҫ����*/
	DWORD dwAlarmOutputNumber;/*��λ,�� 0 λ��Ӧ�� 0 ������˿�, */
	DWORD dwAlarmRelateChannel;/*��λ���� 0 λ��Ӧ�� 0 ��ͨ��*/
	DWORD dwChannel;/*��λ,�� 0 λ��Ӧ�� 0 ��ͨ���� dwAlarmType Ϊ 2 �� 3,6 ʱ��Ҫ��
					��, */
	DWORD dwDiskNumber;/*��λ,�� 0 λ��Ӧ�� 0 ��Ӳ��,dwAlarmType Ϊ 4,5 ʱ��Ҫ����
					   */
}INTER_ALARMINFO, *LPINTER_ALARMINFO;


///////////////////////////////////////////////////////////////
#define SDK_V13
typedef struct{
	//��ʼʱ��
	BYTE byStartHour;
	BYTE byStartMin;
	//����ʱ��
	BYTE byStopHour;
	BYTE byStopMin;
} INTER_SCHEDTIME,*LPINTER_SCHEDTIME;
/*�쳣����ʽ*/
//#define NOACTION 0x0 /*����Ӧ*/
//#define WARNONMONITOR 0x1 /*�������Ͼ���*/
//#define WARNONAUDIOOUT 0x2 /*��������*/
//#define UPTOCENTER 0x4 /*�ϴ�����*/
//#define TRIGGERALARMOUT 0x8 /*�����������*/
typedef struct
{
	DWORD dwHandleType; //�쳣����,�쳣����ʽ��"��"���
	DWORD dwAlarmOutTriggered; //�����ı����������λ����� 32 �����
}INTER_HANDLEEXCEPTION;

typedef struct { /* Each day shares the same motion detection parameter */
	DWORD dwMotionScope[18]; /*�������,0-21 λ,��ʾ 22 ��,���� 22*18 ��С��
	��,Ϊ 1 ��ʾ���ƶ��������,0-��ʾ����*/
	BYTE byMotionSenstive; /*�ƶ����������, 0 - 5,Խ��Խ����,oxff �ر�*/
	BYTE byEnableHandleMotion; /* �Ƿ����ƶ���� 0���� 1����*/
	char reservedData[2];
	INTER_HANDLEEXCEPTION struMotionHandleType; /* ����ʽ */
#ifdef SDK_V13
	INTER_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT]; /*����ʱ
	��*/
#endif
	DWORD dwRelRecordChan; /* ����������¼��ͨ�� ��λ */
	}INTER_MOTION,*LPINTER_MOTION;
	typedef struct{
	DWORD dwEnableHideAlarm; /* �Ƿ������ڵ����� ,0-��,1-��*/
	WORD wHideAlarmAreaTopLeftX; /* �ڵ������ x ���� */
	WORD wHideAlarmAreaTopLeftY; /* �ڵ������ y ���� */
	WORD wHideAlarmAreaWidth; /* �ڵ�����Ŀ� */
	WORD wHideAlarmAreaHeight; /*�ڵ�����ĸ�*/
	INTER_HANDLEEXCEPTION strHideAlarmHandleType; /* ����ʽ */
#ifdef SDK_V13
	INTER_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT]; /*����ʱ��*/
#endif
}INTER_HIDEALARM,*LPINTER_HIDEALARM;

//�źŶ�ʧ����
typedef struct{
	DWORD dwEnableVILostAlarm; /* �Ƿ������źŶ�ʧ���� ,0-��,1-��*/
	INTER_HANDLEEXCEPTION strVILostAlarmHandleType; /* ����ʽ */
#ifdef SDK_V13
	INTER_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT]; /*����ʱ ��*/
#endif
}INTER_VILOST,*LPINTER_VILOST;

typedef struct
{
	DWORD dwSize;
	BYTE sChanName[NAME_LEN]; /* ͨ���� */
	DWORD dwVideoFormat; /* ��Ƶ��ʽ */
	BYTE byBrightness; /*����,������*/
	BYTE byContrast; /*�Աȶ�,������*/
	BYTE bySaturation; /*���Ͷ�,������ */
	BYTE byHue; /*ɫ��,������*/
	//��ʾͨ����
	DWORD dwShowChanName; /* Ԥ����ͼ�����Ƿ���ʾͨ������,0-����ʾ,1-��ʾ */
	WORD wShowNameTopLeftX; /* ͨ��������ʾλ�õ� x ���� */
	WORD wShowNameTopLeftY; /* ͨ��������ʾλ�õ� y ���� */
	//�źŶ�ʧ����
	INTER_VILOST struVILost;
	//�ƶ����
	INTER_MOTION struMotion;
	//�ڵ�����
	INTER_HIDEALARM struHideAlarm;
	//�ڵ�
	DWORD dwEnableHide; /* �Ƿ������ڵ� ,0-��,1-��*/
	WORD wHideAreaTopLeftX; /* �ڵ������ x ���� */
	WORD wHideAreaTopLeftY; /* �ڵ������ y ���� */
	WORD wHideAreaWidth; /* �ڵ�����Ŀ� */
	WORD wHideAreaHeight; /*�ڵ�����ĸ�*/
	//OSD
	DWORD dwShowOsd; /* Ԥ����ͼ�����Ƿ���ʾ OSD,0-����ʾ,1-��ʾ */
	WORD wOSDTopLeftX; /* OSD �� x ���� */
	WORD wOSDTopLeftY; /* OSD �� y ���� */
	BYTE byOSDType; /* OSD����(��Ҫ�������ո�ʽ) */
					/* 0: XXXX-XX-XX ������ */
					/* 1: XX-XX-XXXX ������ */
					/* 2: XXXX��XX��XX�� */
					/* 3: XX��XX��XXXX�� */
					/* 4: XX-XX-XXXX ������*/
					/* 5: XX �� XX �� XXXX �� */
	BYTE byDispWeek; /* �Ƿ���ʾ���� */
	BYTE byOSDAttrib; /* OSD ����:͸������˸ */
	/*
#define OSD_DISABLE 0 //����ʾ OSD
#define OSD_TRANS_WINK 1 //͸������˸
#define OSD_TRANS_NO_WINK 2 //͸��������˸
#define OSD_NO_TRANS_WINK 3 //��͸������˸
#define OSD_NO_TRANS_NO_WINK 4 //��͸��������˸
	*/
	char reservedData2;
}INTER_PICCFG,*LPINTER_PICCFG;

#define MAX_SHELTERNUM 4	

typedef struct _INTER_SHELTER
{
	WORD wHideAreaTopLeftX; /* �ڵ������x���� */
	WORD wHideAreaTopLeftY; /* �ڵ������y���� */
	WORD wHideAreaWidth; /* �ڵ�����Ŀ� */
	WORD wHideAreaHeight; /*�ڵ�����ĸ�*/
}INTER_SHELTER,*LPINTER_SHELTER;

typedef struct _INTER_PICCFG_EX
{
	DWORD dwSize;
	BYTE sChanName[NAME_LEN]; 
	DWORD dwVideoFormat; /*ֻ�� ��Ƶ��ʽ 1-NTSC 2-PAL */
	BYTE byBrightness; /*����,0-255*/
	BYTE byContrast; /*�Աȶ�,0-255*/
	BYTE bySaturation; /*���Ͷ�,0-255 */
	BYTE byHue; /*ɫ��,0-255*/
	//��ʾͨ����
	DWORD dwShowChanName; // Ԥ����ͼ�����Ƿ���ʾͨ������,0-����ʾ,1-��ʾ
	WORD wShowNameTopLeftX; /* ͨ��������ʾλ�õ�x���� */
	WORD wShowNameTopLeftY; /* ͨ��������ʾλ�õ�y���� */
	//�źŶ�ʧ����
	INTER_VILOST struVILost;
	//�ƶ����
	INTER_MOTION struMotion;
	//�ڵ�����
	INTER_HIDEALARM struHideAlarm;
	//�ڵ�
	DWORD dwEnableHide; /* �Ƿ������ڵ� ,0-��,1-��*/
	INTER_SHELTER struShelter[MAX_SHELTERNUM];
	//OSD
	DWORD dwShowDate;// Ԥ����ͼ�����Ƿ���ʾ����
	WORD wShowDateX; /* ʱ����ʾ��x���� */
	WORD wShowDateY; /* ʱ����ʾ��y���� */
	BYTE byDateType; /* ���ڸ�ʽ(��Ҫ�������ո�ʽ) */
					/* 0: XXXX-XX-XX ������ */
					/* 1: XX-XX-XXXX ������ */
					/* 2: XXXX��XX��XX�� */
					/* 3: XX��XX��XXXX�� */
					/* 4: XX-XX-XXXX ������*/
					/* 5: XX��XX��XXXX�� */
	BYTE byDispWeek; /* �Ƿ���ʾ���� */
	BYTE byOSDAttrib; /* OSD����:͸������˸ */
						/* 1: ͸������˸ */
						/* 2: ͸��������˸ */
						/* 3: ��͸������˸ */
						/* 4: ��͸��������˸ */
	char byTimeFmt;	//ʱ���ʽ0-24,1-12
}INTER_PICCFG_EX,*LPINTER_PICCFG_EX;


////////////////////////////ʵʱͼ�����///////////////////////////////////////
typedef struct {
	BYTE byBrightness; /*����,������*/
	BYTE byContrast; /*�Աȶ�,������*/
	BYTE bySaturation; /*���Ͷ�,������ */
	BYTE byHue; /*ɫ��,������*/
}INTER_VIDEOPARA, *LPINTER_VIDEOPARA;

typedef struct{
	DWORD	unknown0;
	BYTE	unknown1;
	BYTE	unknown2;
	BYTE	unknown3;
	BYTE	unknown4;
	BYTE	unknown5;
	BYTE	res1[3];
	BYTE	unknown6;
	BYTE	res2[7];
	BYTE	unknown7;
	BYTE	res3[7];
	DWORD	unknown8;
	DWORD	unknown9;
	BYTE	res4[16];
	DWORD	unknown10;
	BYTE	res5[16];
	BYTE	unknown11;
	BYTE	unknown12; 
	BYTE	unknown13;
	BYTE	unknown14;
	DWORD	unknown15;
	BYTE	res6[4];
	DWORD	unknown16;
	BYTE	res7[20];
	DWORD	unknown17;
	BYTE	res8[6];
	BYTE	Mirror;
	BYTE	res9;
	DWORD	unknown18;
	BYTE	res10[28];
	DWORD	unknown19;
	DWORD	unknown20;
	BYTE	res11[12];
	BYTE	spin;
	BYTE	res12[3];
	BYTE	res13[12];
	DWORD	unknown21;
	DWORD	unknown22;
	BYTE	res14[4];
	DWORD	unknown23;
	DWORD	unknown24;
	BYTE	res15[288];
}INTER_COLORENHANCE, *LINTER_COLORENHANCE;

typedef struct { 
	BYTE sNTPServer[64];    	/* Domain Name or IP addr of NTP server */ 
	WORD wInterval;    		/* adjust time interval(hours) */ 
	BYTE byEnableNTP; 			/*enable NPT client */ 
	signed char cTimeDifferenceH; /* ����ʱ�׼ʱ���ʱ�� -12 ... +13 */ 
	signed char cTimeDifferenceM; 
    BYTE res1;
    WORD wNtpPort;			//NTP�������˿ڣ��豸Ĭ��Ϊ123 
    BYTE res2[8];
}INTER_NTPPARA, *LPINTER_NTPPARA; 

/*ʱ������ʱ�Ʋ���*/
typedef struct 
{
    DWORD dwMonth;  //�� 0-11��ʾ1-12����
    DWORD dwWeekNo;  //�ڼ��ܣ�0����1�ܣ�1����2�ܣ�2����3�ܣ� 3����4�ܣ�4�����һ�� 
    DWORD dwWeekDate;  //���ڼ���0�������գ�1������һ��2�����ڶ���3����������4�������ģ�5�������壻6��������
    DWORD dwHour; //Сʱ����ʼʱ��ȡֵ��Χ0��23�� ����ʱ��ȡֵ��Χ1��23 
    DWORD dwMin; //��0��59
}INTER_TIMEPOINT;

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
    INTER_TIMEPOINT		stBeginPoint;	//��ʱ�ƿ�ʼʱ��
    INTER_TIMEPOINT		stEndPoint;		//��ʱ��ֹͣʱ��
}INTER_DSTPARA, *LPINTER_DSTPARA;

typedef struct{
	UINT32	U32IpV4;
	BYTE  	byRes[20]; 
}INTER_NET_IPADDR,*LINTER_NET_IPADDR;

typedef struct{
	INTER_NET_IPADDR strIPAddr;
	INTER_NET_IPADDR strIPMask;
	DWORD	dwNetInterface;
	WORD	wDataPort;
    WORD	wMTU;
	BYTE	macAddr[6];
	BYTE	res2[2];
}INTER_NET_ETHERNET;

typedef struct{
	DWORD	dwSize;
    INTER_NET_ETHERNET    stEtherNet[MAX_ETHERNET];
	BYTE	res1[64];
	BYTE	res2[14];
    BYTE    byIpV4Enable;          
    BYTE    byIpV6Enable;  
	WORD	wMulticastSearchEnable; //0x0101����,0x0202�ر�
	BYTE	res3[2];
	BYTE	res4[4];
    INTER_NET_IPADDR    stDnsServer1IpAddr;
    INTER_NET_IPADDR    stDnsServer2IpAddr;                               
	BYTE	res5[64];
	BYTE	res6[2];
    WORD    wHttpPort;	//Http�˿�    
	BYTE	res7[4];
    INTER_NET_IPADDR    stMulticastIpAddr;    
    INTER_NET_IPADDR    stGatewayIpAddr;
	BOOL	dwPPPoEEnable;
	char 	csPPPoEUser[32];
	char	csPPPoEPassword[32];
	BYTE	res8[36];
}INTER_NET_ETHER_CFG, *LINTER_NET_ETHER_CFG;

typedef struct{
	DWORD	dwSize;
	BYTE	byEnableFTP;
	char 	res1[2];
	BYTE	byFTPPort;
	char	csFTPIpAddress[MAX_DOMAIN_NAME];
	char    csUserName[NAME_LEN];
	char    csPassword[PASSWD_LEN];
	char	byCustomTopDirMode[MAX_DOMAIN_NAME]; //�Զ���һ��Ŀ¼
	char	byCustomSubDirMode[MAX_DOMAIN_NAME]; //�Զ������Ŀ¼
	BYTE	byDirMode;		//Ŀ¼�ṹ,0:�����ڸ�Ŀ¼,1:ʹ��һ��Ŀ¼,2:ʹ�ö���Ŀ¼
	BYTE	byTopDirMode;	//һ��Ŀ¼,1:ʹ���豸��,2:ʹ���豸��,3:ʹ���豸IP,6:�Զ���
	BYTE	bySubDirMode;	//����Ŀ¼,1:ʹ��ͨ����,2:ʹ��ͨ����,3:�Զ���
	char 	res3;
	BYTE	byIsAnonymity;	//�Ƿ�����
	BYTE	byServerType;	//����������
	char 	res4[2];
	char 	res5[252];	
}INTER_NET_FTP_CFG, *LINTER_NET_FTP_CFG;

typedef struct{
	DWORD	dwSize;
	WORD	wPort;			//RTSP�˿�
	BYTE	byReserve[54];
}INTER_NET_RTSP_CFG,*LINTER_NET_RTSP_CFG;

typedef struct{
	DWORD	length;	//�ļ�ͷ�ĳ���
	BYTE	byFileHead[40]; //�ļ�ͷ
	BYTE	byReserve[4];
}INTER_NET_P2PTCP_CFG,*LINTER_NET_P2PTCP_CFG;

typedef struct{
	DWORD	dwSize;
	BYTE	res1;
	BYTE	res2[3];
	BYTE	res3;
	BYTE	res4;
	BYTE	res5[2];
	BYTE	res6[64];
}NET_UNKNOWN113305;

#pragma pack()

#endif

