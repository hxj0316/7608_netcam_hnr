#ifndef _HIK_CONFIGTYPES_H_
#define _HIK_CONFIGTYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

//#include "QMAPIType.h"
#include "hik_lst.h"
//#include "hik_netfun.h"

#ifndef _HAVE_TYPE_FALSE
#define _HAVE_TYPE_FALSE
	#define FALSE 0
#endif

#ifndef _HAVE_TYPE_TRUE
#define _HAVE_TYPE_TRUE
	#define TRUE 1
#endif



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
	typedef unsigned long		DWORD;
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

#ifndef _HAVE_TYPE_SINT16
#define _HAVE_TYPE_SINT16
	typedef signed short 		SINT16;
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


#define NAME_LEN				32				/* �û������� */
#define PASSWD_LEN				16				/* ���볤�� */
#define MAX_DOMAIN_NAME			64				/* ����������� */
#define MACADDR_LEN				6				/* MAC��ַ���� */
#define PATHNAME_LEN			128				/* ·�������� */
#define PRODUCT_MODEL_LEN		64				/* ��Ʒ�ͺų���(�����ͺ�>16λ) */

#define MAX_VIDEOOUT			4				/* ���CVBS���� */
#define MAX_VIDEOOUT_8000		2				/* ���CVBS���� */
#define MAX_VGA					4				/* ���VGA���� */
#define MAX_VGA_8000			1				/* ���VGA���� */
#define MAX_AUXOUT				16				/* ���ģ������������ */
#define MAX_ETHERNET			2				/* ���ڸ��� */
#define MAX_ANALOG_CHANNUM		32				/* maximum analog channel count */
#define MAX_CHANNUM				32				/* �������ͨ���� */
#define MAX_CHANNUM_8000	    16				/* �������ͨ���� */
#define	MAX_DECODE_CHAN     	16				/* ������ͨ���� */
#define MAX_ALARMIN				32				/* ��󱨾����� */
#define MAX_ALARMIN_8000	    16				/* ��󱨾����� */
#define MAX_ALARMOUT			32				/* ��󱨾���� */
#define MAX_ALARMOUT_8000		4				/* ��󱨾���� */
#define MAX_SERIAL_PORT			8				/* ��󴮿ڸ��� */
#define MAX_USB_NUM         	2				/* USB���������� */

#define MAX_IPCHANNUM       	32				/* IPC������ */
#define MAX_IPC_ALARMIN     	6				/* IPC��󱨾�������� */
#define MAX_IPC_ENCCHAN     	4				/* IPC��󱨱���ͨ������ */
#define MAX_IPC_ALARMOUT    	2				/* IPC��󱨾�������� */

#define MAX_USERNUM				32				/* ����û��� */
#define MAX_USERNUM_8000		16				/* ����û��� */
#define MAX_EXCEPTIONNUM		32				/* ����쳣���� */
#define MAX_EXCEPTIONNUM_8000	16				/* ����쳣���� */
// fix by tw
//#define MAX_TIMESEGMENT			8				/* һ�����8��ʱ��� */
#define MAX_TIMESEGMENT			4				/* һ�����8��ʱ��� */
#define MAX_TIMESEGMENT_8000    4				/* 8000һ�����4��ʱ��� */
#define MAX_DAYS				7				/* 1��7�� */
#define PHONENUMBER_LEN			32				/* �绰���볤�� */

#define MAX_HD_COUNT			33				/* ���33��Ӳ��(����16������SATAӲ�̡�1��eSATAӲ�̺�16��NFS��) */
#define MAX_HD_COUNT_8000	    16				/* 8000���16��Ӳ��(����16������SATAӲ�̡�1��eSATAӲ�̺�16��NFS��) */
#define MAX_NFS_DISK			16				/* ���16��NFS�� */
#define MAX_NFS_DISK_8000	    8				/* 8000���8��NFS�� */
#define MAX_HD_GROUP			16				/* ���16������ */

#define MAX_AUXOUT_COUNT		4				/* ���������ڸ��� */

#define MAX_OSD_LINE			8				/* ���OSD�ַ�����(������ͨ����������) */
#define MAX_OSD_LINE_8000		4				/* ���OSD�ַ�����(������ͨ����������) */
#define MAX_MASK_REGION_NUM		4				/* ����ڵ�������� */
#define MAX_PREV_WINDOWS		32				/* ��󱾵�Ԥ�������� */
#define MAX_PREV_WINDOWS_8000	16				/* ��󱾵�Ԥ�������� */
#define MAX_PREV_MOD			8				/* ���Ԥ��ģʽ��Ŀ*/

#define MAX_PRESET				256				/* ��̨��Ԥ�õ� */
#define MAX_PRESET_8000			128				/* ��̨��Ԥ�õ� */
#define MAX_TRACK				32				/* ��̨�Ĺ켣 */
#define MAX_TRACK_8000			16				/* ��̨�Ĺ켣 */
#define MAX_CRUISE				32				/* ��̨��Ѳ�� */
#define MAX_CRUISE_8000			16				/* ��̨��Ѳ�� */
#define MAX_KEYPOINT			64				/* һ��Ѳ�����Ĺؼ��� */

#define PRODDATELEN         8			/* �������� */
#define PRODNUMLEN          9			/* 9λ�������к� */
#define SERIALNO_LEN        48			/* ���кų��� */


#define PARAVER_071122			0x071122			/* V1.0.0�Ĳ����ṹ */
#define PARAVER_090309			0x090309			/* V1.0.1�Ĳ����ṹ */
#define PARAVER_091203			0x091203			/* V1.0.3 */
#define PARAVER_100316			0x100316			/* V3.1.0 */
#define PARAVER_101010			0x101010			/* V3.2.0 */
#define PARAVER_100511			0x100511			/* 81st�Ĳ����ṹ */
#ifdef DZ_BEITONG_VER
#define PARAVER_110108          0x110108            /* ��ͨЭ��Ĳ��� */
#endif

#ifdef HI3515
#define CURRENT_PARAVER	PARAVER_100511
#else
#ifndef DZ_BEITONG_VER
#define CURRENT_PARAVER	PARAVER_101010
#else
#define CURRENT_PARAVER	PARAVER_110108
#endif
#endif

#define MAX_VO_CHAN				MAX_VIDEOOUT	/*������ͨ������*/
#define MAX_VO_CHAN_8000		MAX_VIDEOOUT_8000/*������ͨ������*/
#define CHANNEL_MAIN			1				/*����*/
#define CHANNEL_SEC				2				/*����*/
#define CHANNEL_VGA				3				/*VGA */
#define CHANNEL_AUX			    0				/*�������*/

#define MAX_AO_CHAN				2				/* maximum audio out ports */
/*
  Ԥ��ģʽ
  */
#define PREVIEW_MODE_1		1
#define PREVIEW_MODE_2		2
#define PREVIEW_MODE_4		4
#define PREVIEW_MODE_6		6
#define PREVIEW_MODE_7		7
#define PREVIEW_MODE_8		8
#define PREVIEW_MODE_9		9
#define PREVIEW_MODE_10		10
#define PREVIEW_MODE_12     12
#define PREVIEW_MODE_13		13
#define PREVIEW_MODE_16		16
#define PREVIEW_MODE_402	0x42

/*
 * Definitions for OSD
 */
#define OSD_DISABLE					0		/* ����ʾOSD */
#define OSD_TRANS_WINK				1		/* ͸������˸ */
#define OSD_TRANS_NO_WINK			2		/* ͸��������˸ */
#define OSD_NO_TRANS_WINK			3		/* ��͸������˸ */
#define OSD_NO_TRANS_NO_WINK		4		/* ��͸��������˸ */

#define OSD_TYPE_ENGYYMMDD			0		/* XXXX-XX-XX ������ */
#define OSD_TYPE_ENGMMDDYY			1		/* XX-XX-XXXX ������ */
#define OSD_TYPE_ENGDDMMYY			2		/* XX-XX-XXXX ������ */
#define OSD_TYPE_CHNYYMMDD			3		/* XXXX��XX��XX�� */
#define OSD_TYPE_CHNMMDDYY			4		/* XX��XX��XXXX�� */
#define OSD_TYPE_CHNDDMMYY			5		/* XX��XX��XXXX�� */

#define OSD_24H_TYPE			    0		/* 24 Сʱ��*/
#define OSD_12H_TYPE			    1		/* 12 Сʱ��*/

/*
 * Definitions for DEVTime
 */
#define DATE_TYPE_YYMMDD			0		/* XXXX-XX-XX ������ */
#define DATE_TYPE_MMDDYY			1		/* XX-XX-XXXX ������ */
#define DATE_TYPE_DDMMYY			2		/* XX-XX-XXXX ������ */

/*
 * Definitions for LOGO
 */
#define LOGO_DISABLE				0		/* ����ʾLOGO */
#define LOGO_TRANS					1		/* ͸�� */
#define LOGO_NO_TRANS				2		/* ��͸�� */

/*
 * Definitions for streamtype and resolution��
 */
#define STREAM_TYPE_VO		(STREAM_VIDEO-CONST_NUMBER_BASE)	/* Video only */
#define STREAM_TYPE_AO		(STREAM_AUDIO-CONST_NUMBER_BASE)	/* Audio only */
#define STREAM_TYPE_AV		(STREAM_MULTI-CONST_NUMBER_BASE)	/* Audio & video */

#define RESOLUTION_CIF		(CIF_FORMAT-CONST_NUMBER_BASE)
#define RESOLUTION_QCIF		(QCIF_FORMAT-CONST_NUMBER_BASE)
#define RESOLUTION_2CIF		(HCIF_FORMAT-CONST_NUMBER_BASE)
#define RESOLUTION_4CIF		(FCIF_FORMAT-CONST_NUMBER_BASE)
#define RESOLUTION_QQCIF	(QQCIF_FORMAT-CONST_NUMBER_BASE)	/* PAL:96*80, NTSC:96*64 */
#define RESOLUTION_NCIF		(NCIF_FORMAT-CONST_NUMBER_BASE)		/* 320*240 */
#define RESOLUTION_DCIF		(DCIF_FORMAT-CONST_NUMBER_BASE)		/* PAL:528*384 NTSC:528*320 */
#define RESOLUTION_UXGA		(UXGA_FORMAT-CONST_NUMBER_BASE)  	/* 1600*1200 !!!��QQCIF����*/
#define RESOLUTION_VGA		(VGA_FORMAT-CONST_NUMBER_BASE)		/*VGA format added by lj 2007-1-27*/
#define RESOLUTION_HD720P  	(HD720p_FORMAT-CONST_NUMBER_BASE)

/*
 *definitions for videoStandard, �� DSP/common.h�е�VIDEO_STANDARDһ��
 */
#define VIDEOS_NON		0
#define VIDEOS_NTSC 	1			/* NTSC��ʽ */
#define VIDEOS_PAL		2			/* PAL ��ʽ */

/* ¼������ */
#define TIMING_REC				0x0				/* ��ʱ¼�� */
#define MOTION_DETECT_REC		0x1				/* �ƶ����¼�� */
#define ALARM_REC				0x2				/* ����¼�� *//* �ӽ����� */
#define ALARMORMOTION_REC		0x3				/* ���� | �ƶ����¼�� */ /* �������� */
#define ALARMANDMOTION_REC		0x4				/* ���� & �ƶ����¼�� */ /* װ������ */
#define COMMAND_REC				0x5				/* ����� */
#define MANUAL_REC				0x6				/* �ֶ�¼�� */
#define SHAKEALARM_REC			0x7				/* �𶯱���¼�� */
#define ENVIRONMENT_ALARM_REC 	0x8				/* ������������¼�� */
#define ALL_RECORD_TYPE			0xff			/* ����¼������ */


/*�쳣��������*/
#define NO_HARDDISK_FLAG				0x80000000

#define EXCEPTIONCOUNT					5		/*�쳣��������*/
#define HARDDISKFULL_EXCEPTION			0x0		/*����*/
#define HARDDISKERROR_EXCEPTION			0x1		/*�̴�*/
#define ETHERNETBROKEN_EXCEPTION		0x2		/*���߶�*/
#define IPADDRCONFLICT_EXCEPTION		0x3		/*IP��ַ��ͻ*/
#define ILLEGALACCESS_EXCEPTION			0x4		/*�Ƿ�����*/
#define VI_EXCEPTION					0x5		/*��Ƶ�ź��쳣*/
#define VS_MISMATCH_EXCEPTION			0x6		/*����/�����Ƶ��ʽ��ƥ�� */
#define VIDEOCABLELOSE_EXCEPTION		0x7		/*��Ƶ���ź�*/
#define AUDIOCABLELOSE_EXCEPTION        0x8		/*��Ƶ���ź�*/
#define ALARMIN_EXCEPTION				0x9		/*��������*/
#define MASKALARM_EXCEPTION				0xa		/*�ڵ�����*/
#define MOTDET_EXCEPTION				0xb		/*�ƶ����*/
#define RECORDING_EXCEPTION				0xc		/*¼���쳣--���벻���ļ���¼������г����쳣*/
#define IPC_IP_CONFLICT_EXCEPTION       0Xd     /*ipc ip ��ַ��ͻ*/
#define MAX_EXCEPTION_TYPE				(RECORDING_EXCEPTION+1)

#define MANUAL_TRIGGER_ALARMOUT			0x10
#define MANUAL_CLEAR_ALARMOUT			0x11
#define IPC_INNERCHAN_NOTEXIST          0x20
/*�쳣����ʽ*/
#define NOACTION						0x0		/*����Ӧ*/
#define WARNONMONITOR					0x1		/*�������Ͼ���*/
#define WARNONAUDIOOUT					0x2		/*��������*/
#define UPTOCENTER						0x4		/*�ϴ�����*/
#define TRIGGERALARMOUT					0x8		/*�����������*/
#define SENDEMAIL						0x10	/*Send Email*/

/* RS232��ʹ��ģʽ */
#define SER_MODE_PPP		0			/* PPP ģʽ */
#define SER_MODE_CONSOLE	1			/* ���ƿ�ģʽ dvrShell */
#define SER_MODE_TRANS		2			/* ͸�����������ͨ�� */
#define SER_MODE_DIRECT		3			/* ����ֱ�� */

/* baud rate defines */
#define S50     	0
#define S75     	1
#define S110    	2
#define S150    	3
#define S300    	4
#define S600    	5
#define S1200   	6
#define S2400   	7
#define S4800   	8
#define S9600   	9
#define S19200  	10
#define S38400  	11
#define S57600  	12
#define S76800		13
#define S115200 	14

/* data bits defines */
#define DATAB5     	0
#define DATAB6     	1
#define DATAB7     	2
#define DATAB8     	3

/* stop bits defines */
#define STOPB1		0
#define STOPB2		1

/* parity defines */
#define NOPARITY	0
#define ODDPARITY	1
#define EVENPARITY	2

/* flow control defines */
#define	NOCTRL		0
#define SOFTCTRL	1		/* xon/xoff flow control */
#define HARDCTRL	2		/* RTS/CTS flow control */

#define SER_SPEED_SHIFT		0
#define SER_SPEED_MASK		0xf
#define SER_DATAB_SHIFT		4
#define SER_DATAB_MASK		0x3
#define SER_STOPB_SHIFT		6
#define SER_STOPB_MASK		0x1
#define SER_PARITY_SHIFT	7
#define SER_PARITY_MASK 	0x3
#define SER_FLOWCTRL_SHIFT	9
#define SER_FLOWCTRL_MASK	0x3

#ifdef EVDO
#define EVDO_MAXCELLNUM 5
#define EVDO_CELLNUMLEN 16
#endif

/*
  �û�����
 */
#define LOGIN_NO_LOGIN		0			/*δ��¼*/
#define LOGIN_DEFAULT_RIGHT	0x00014014
				/* Ĭ��Ȩ��:�������غ�Զ�̻ط�,���غ�Զ�̲鿴��־��״̬,���غ�Զ�̹ػ�/����
					LOCALPLAYBACK|LOCALSHOWSTATUS|REMOTEPLAYBACK|REMOTESHOWSTATUS
				*/
#define LOGIN_SUPERUSER		0x000d7117
				/* �߼�����Ա:�������غ�Զ�̿�����̨,���غ�Զ���ֶ�¼��,���غ�Զ�̻ط�,�����Խ���Զ��Ԥ��
				    ���ر���,����/Զ�̹ػ�/����
					LOCALPTZCONTROL|LOCALMANUALREC|LOCALPLAYBACK|REMOTEPTZCONTROL
					|REMOTEMANUALREC|REMOTEPLAYBACK|REMOTEVOICETALK|REMOTEPREVIEW
				*/
#define LOGIN_ADMIN			0xffffffff	/*����Ա*/

/*�û�Ȩ�޵ȼ�*/
#define USER_PRIO_HIGH     	2
#define USER_PRIO_MIDDLE	1
#define USER_PRIO_LOW		0


/*
 * Definitions for user permission
 */
#define LOCALPTZCONTROL		0x1					/* ���ؿ�����̨ */
#define LOCALMANUALREC		0x2					/* �����ֶ�¼�� */
#define LOCALPLAYBACK		0x4					/* ���ػط� */
#define LOCALSETPARAMTER	0x8					/* �������ò��� */
#define LOCALSHOWSTATUS		0x10				/* ���ز鿴״̬����־ */
#define LOCALHIGHOP			0x20				/* ���ظ߼�����(��������ʽ��) */
#define LOCALSHOWPARAMETER	0x40				/* ���ز鿴���� */
#define LOCALMANAGECAMERA	0x80				/* ���ع���ģ���IP camera */
#define LOCALBACKUP		    0x100				/* ���ر��� */
#define LOCALREBOOT		    0x200				/* ���عػ�/���� */

#define REMOTEPTZCONTROL	0x1000				/* Զ�̿�����̨ */
#define REMOTEMANUALREC		0x2000				/* Զ���ֶ�¼�� */
#define REMOTEPLAYBACK		0x4000				/* Զ�̻ط� */
#define REMOTESETPARAMETER	0x8000				/* Զ�����ò��� */
#define REMOTESHOWSTATUS	0x10000				/* Զ�̲鿴״̬����־ */
#define REMOTEHIGHOP		0x20000				/* Զ�̸߼�����(��������ʽ��) */
#define REMOTEVOICETALK		0x40000				/* Զ�̷��������Խ� */
#define REMOTEPREVIEW		0x80000				/* Զ��Ԥ�� */
#define REMOTEALARM			0x100000			/* Զ�����󱨾��ϴ���������� */
#define REMOTECTRLLOCALOUT	0x200000			/* Զ�̿��ƣ�������� */
#define REMOTESERIALCTRL	0x400000			/* Զ�̿��ƴ��� */
#define REMOTESHOWPARAMETER	0x800000			/* Զ�̲鿴���� */
#define REMOTEMANAGECAMERA	0x1000000			/* Զ�̹���ģ���IP camera */
#define REMOTEREBOOT		0x2000000			/* Զ�̹ػ�/���� */

/*��������*/
#define PARA_VIDEOOUT	0x1
#define PARA_IMAGE		0x2
#define PARA_ENCODE		0x4
#define PARA_NETWORK	0x8
#define PARA_ALARM		0x10
#define PARA_EXCEPTION	0x20
#define PARA_DECODER	0x40				/*������*/
#define PARA_RS232		0x80
#define PARA_PREVIEW	0x100
#define PARA_SECURITY	0x200
#define PARA_DATETIME	0x400
#define PARA_FRAMETYPE	0x800				/*֡��ʽ*/
#define PARA_HARDDISK   0x1000
#define PARA_AUTOBOOT   0x2000
#define PARA_HOLIDAY    0x4000
#define PARA_IPC	    0x8000 				/*IPͨ������*/
#define PARA_DEVICE     0X10000				/*�豸������Ϣ*/

/*ppp ����*/
#define PPPMODE_MASK		0x1					/* PPPģʽ��0Ϊ������1Ϊ���� */
#define CALLBACK_MASK		0x2					/* �ز���0Ϊ����Ҫ��1Ϊ��Ҫ */
#define CALLBACKMODE_MASK	0x4					/* �ز�ģʽ��0Ϊ�̶����룻1Ϊ�û�ָ�� */
#define ENCRYPTION_MASK		0x8					/* ���ܣ�0Ϊ����Ҫ��1��Ϊ��Ҫ */

/* ���뷽�� */
#define NET_INTERCEPTION	0
#define NET_RECEIVE			1
#define SERIAL_DIRECT_INPUT	2
#define SERIAL_ATM_INPUT	3

//#define NET_INTERCEPTION		0
#define SERIAL_INTERCEPTION 	1
#define NET_PROTOCOL			2
#define SERIAL_PROTOCOL		3
#define SERIAL_SERVER			4

#define OLD_ATM_ALARMIN_NUMS		5
/* ATM������ */
#define ATM_NCR				0
#define ATM_DIEBOLD			1
#define ATM_WINCOR_NIXDORF	2
#define ATM_SIEMENS			3
#define ATM_OLIVETTI		4
#define ATM_FUJITSU			5
#define ATM_HITACHI			6
#define ATM_SMI				7
#define ATM_IBM				8
#define ATM_BULL			9
#define ATM_YIHUA			10
#define ATM_LIDE			11
#define ATM_GUANGDIAN		12
#define ATM_MINIBANL		13
#define ATM_GUANGLI			14
#define ATM_EASTCOM			15
#define ATM_CHENTONG		16
#define ATM_NANTIAN			17
#define ATM_XIAOXING		18
#define ATM_YUYIN			19
#define ATM_TAILITE			20
#define ATM_DRS918			21

#define ATM_KALATEL			22
#define ATM_NCR_2			23

#define ATM_NXS				24

#define MAX_ATM_TYPE		ATM_NXS

/* IP camera factory */
#define FAC_HIKVISION       0
#define FAC_PANASONIC		1
#define FAC_SONY			2	
#define FAC_AXIS	       	3
#define FAC_INFINOVA      	4
#define FAC_SANYO			5
#define FAC_ZAVIO           6

/* backup device type */
#define BACKUP_USB_DEV		1
#define BACKUP_USB_HD_DEV	2
#define BACKUP_USB_CDR_DEV	3
#define BACKUP_IDE_CDR_DEV	4
#define BACKUP_SATA_DEV		5
#define BACKUP_SATA_CDR_DEV	6
/*�����ļ�*/
#define ITEM_FILE_COPY  30		/*���ɸ����ļ�����Ҳ�������ɸ����ļ�Ƭ��*/

/* parameter of searching file */
#define MAX_DATE_TIME	(0x7fffffff)
#define MIN_DATE_TIME	(0)
#define MAX_DATE_YEAR	(2037)
#define NO_MATCH_CHAN	(-1)
#define NO_MATCH_TYPE	(0xff)
#define NO_MATCH_TIME	(0)

/* NIC type */
#define NET_IF_10M_HALF		1					/* 10M ethernet */
#define NET_IF_10M_FULL		2
#define NET_IF_100M_HALF	3					/* 100M ethernet */
#define NET_IF_100M_FULL	4
#define NET_IF_1000M_FULL	5					/* 1000M ethernet */
#define NET_IF_AUTO			6


/* minimum and maximum value of MTU */
#define MIN_MTU_VALUE		500
#define MAX_MTU_VALUE		1500

/* ͨ��֧�ֵ����ֱ��� */
#define MAX_CIF_FORMAT		0			/* QCIF, CIF */
#define MAX_DCIF_FORMAT 	1			/* QCIF, CIF, 2CIF, DCIF */
#define MAX_4CIF_FORMAT 	2			/* QCIF, CIF, 2CIF, DCIF, 4CIF */
#define MAX_2CIF_FORMAT 	3			/* QCIF, CIF, 2CIF */
#define MAX_QCIF_FORMAT 	4			/* QCIF */

/*
  �ṹ�ĳ�Ա����ע�����: UINT64����8�ֽڶ��룬UINT32����4�ֽڶ��롣
*/
#define CFG_MAGIC				0x484b5753		/* "HKWS", */
#define PARAM_START_OFFSET 		12

/*
 	��Ϣ����Ĭ��ģʽ
*/
#define DEFAULT_MSG_MODE   		0600

/*
	��ֹ�ƶ����
*/
#define DISABLE_MOTDET	0xff

#ifdef DZ_BEITONG_VER
#define MAX_BT_CARD_LEN      160                //���屴ͨ���ŵĳ���      
#endif


/*
	����
*/
typedef struct
{		/* 4 bytes */
	UINT16	x;									/* ������ */
	UINT16	y;									/* ������ */
} COORDINATE;

/*
	�ߴ�
*/
typedef struct
{		/* 4 bytes */
	UINT16		width;							/* ��� */
	UINT16		height;							/* �߶� */
} SIZE;

/*
	��������
*/
typedef struct
{		/* 8 bytes */
	COORDINATE	upLeft;							/* ���Ͻ����� */
	SIZE		size;							/* ��С(��Ⱥ͸߶�) */
} ZONE;


/*
	��������
*/
typedef struct
{		/* 12 bytes */
	COORDINATE	upLeft;							/* ���Ͻ����� */
	COORDINATE	dwRight;						/* ���½�����*/
	UINT32		enableWork;						/* �Ƿ�����*/
} ZONE_EX;


/*
	�����ʼ���Ϣ
*/
typedef struct{
	UINT32	bSendEmail;
	UINT32  haveSendEmail;
	UINT32	exceptionType;
	UINT64	inputChannel;
	UINT64  trigChannel;
	UINT64	saveInputChannel;
	UINT16	IPCInputAlarmNo[MAX_IPCHANNUM];
	UINT16	IPCsaveInputAlarmNo[MAX_IPCHANNUM];
	UINT64	saveTrigChannel;
	time_t  eventTime;
} CTRL_EMAILINFO;

/*
 * ������ʾ��Ϣ�ṹ
 */
typedef struct
{
	NODE node;
	int alarmType;
	UINT32 channel;				/* channel no. */
	UINT8 ipStr[MAX_DOMAIN_NAME];   //������IP��ַ
}ALARM_TIP;

/* HD_INFO: ���ڱ�����ʾӲ��״̬
*/
#define HD_STAT_OK			0	/* ���� */
#define HD_STAT_UNFORMATTED		1	/* δ��ʽ�� */
#define HD_STAT_ERROR			2	/* ���� */
#define HD_STAT_SMART_FAILED		3	/* SMART״̬ */
#define HD_STAT_MISMATCH			4	/* ��ƥ�� */
#define HD_STAT_IDLE				5	/* ����*/
#define HD_STAT_NOT_ONLINE			6	/* ����Ӳ�̲�����*/

typedef struct
{	/* 24 bytes */
	UINT16 ctrl;
	UINT16 drive;
	UINT32 capacity;
	UINT32 freeSpace;
	UINT32 hdStatus;
	UINT16 bIdle;
	UINT16 bReadOnly;
	UINT16 bRedund;
	UINT16 hdGroup;
	UINT32 hdType;
}HD_INFO;

/*
	ʱ���
*/
#define MAKE_HM(hour, min) 		(((hour)<<8) | (min))
#define GET_HOUR_FROM_HM(hm)	(((hm)>>8) & 0xff)
#define GET_MIN_FROM_HM(hm)		((hm) & 0xff)
typedef struct
{		/* 4 bytes */
	UINT16		startTime;						/* ��ʼʱ�� */
	UINT16		stopTime;						/* ֹͣʱ�� */
} TIMESEGMENT;

typedef struct
{		/* 104 bytes */
	COORDINATE	upLeft;							/* ���Ͻ����� */
	UINT8		dispLine;						/* �Ƿ���ʾ���� */
	UINT8		charCnt;						/* �ַ����� */
	UINT8		osdCode[88];					/* OSD�ַ������ǰ������ص�IP Camera������ÿ�е�����ַ���Ϊ88 */
	UINT8		res[10];
}OSD_LINE;

/*
	OSD���ݽṹ
*/
typedef struct
{		/* 848 bytes */
	UINT8       osdAttrib;						/* OSD����:͸������˸ */
	UINT8		hourOsdType;					/* 24Сʱ�ƻ���12Сʱ��*/
	UINT8		res[2];	
	/* ����/ͨ���� */
	UINT8		dispDate;						/* �Ƿ���ʾ���� */
	UINT8		dateOsdType;					/* �����ո�ʽ���� */
	UINT8		dispWeek;						/* �Ƿ���ʾ���� */
	UINT8		dispNameOsd;					/* �Ƿ���ʾͨ������ */
	COORDINATE	dateUpLeft;						/* �������Ͻ����� */
	COORDINATE	nameOsdUpLeft;					/* ͨ��������ʾλ�� */
	/* ����OSD�ַ� */
	OSD_LINE	additOsdLine[MAX_OSD_LINE];		/* ����OSD�ַ� */
} OSDPARA;

/*
	LOGO����
*/
typedef struct
{		/* 8 bytes */
	UINT8		dispLogo;						/* �Ƿ����LOGO */
	UINT8		logoAttrib;						/* LOGO����:͸������˸ */
	UINT8		res[2];
	COORDINATE	logoUpLeft;						/* LOGOλ�� */
}LOGOPARA;

/*
 * Compression parameters
 */
typedef struct
{		/* 32 bytes */
	UINT8		streamType;						/* ��������: ���ϡ���Ƶ����Ƶ */
	UINT8		resolution;						/* �ֱ��ʣ�QCIF��CIF��2CIF��DCIF��4CIF��D1 */
	UINT8		bitrateType;					/* �����ʡ������� */
	UINT8		videoEncType;					/* ��Ƶ�����׼ */
	UINT8		audioEncType;					/* ��Ƶ�����׼ */
	UINT8		BFrameNum;						/* B֡����: 0:BBP, 1:BP, 2:P */
	UINT8		EFNumber;						/* E֡���� */
	UINT8		res[13];
	UINT16	 	intervalFrameI;					/* I֡��� */
	UINT16		quality;						/* ͼ������ */
	UINT32 		maxVideoBitrate;				/* ��Ƶ��������(��λ��bps) */
	UINT32 		videoFrameRate;					/* ��Ƶ֡�ʣ�PAL��1/16-25��NTCS��1/16-30 */
} COMPRESSION;

/*
	¼��ƻ����ݽṹ
*/
typedef struct
{		/* 8 bytes */
	TIMESEGMENT	recActTime;						/* ʱ��� */
	UINT8		type;							/* ����(��ͬʱ��ο��в�ͬ������) */
	UINT8		compParaType;					/* ѹ���������� */
	UINT8		res[2];
} RECORDSCHED;

/*
	ȫ��¼�����ݽṹ
*/
typedef struct
{		/* 8 bytes */
	UINT8		bAllDayRecord;					/* �Ƿ�ȫ��¼�� */
	UINT8		recType;						/* ¼������ */
	UINT8		compParaType;					/* ѹ���������� */
	UINT8		res[5];
}RECORDDAY;
#if 0
/*
*IPcamrea record parameters
*/
typedef struct
{	/*546 bytes*/
	UINT8			enableRecord;				/* �Ƿ�¼�� */
	UINT8			enableRedundancyRec;		/* �Ƿ�����¼�� */
	UINT8			enableAudioRec;				/* ����������ʱ�Ƿ��¼��Ƶ���� */
	UINT8			res[17];
	RECORDDAY		recordDay[MAX_DAYS];
	RECORDSCHED		recordSched[MAX_DAYS][MAX_TIMESEGMENT];	/* ��¼���� */
	UINT32			recordDelay;				/* �ƶ����/����¼�����ʱ */
	UINT32          preRecordTime;				/* Ԥ¼ʱ��*/
	UINT32			recorderDuration;			/* ¼�񱣴���ʱ�� */
} IPCRECORD;
#endif

/* backup device type */
#define BACKUP_USB_DEV		1
#define BACKUP_USB_HD_DEV	2
#define BACKUP_USB_CDR_DEV	3
#define BACKUP_IDE_CDR_DEV	4
#define BACKUP_SATA_DEV		5
#define BACKUP_SATA_CDR_DEV	6
#define ITEM_FILE_COPY  	30		/*���ɸ����ļ�����Ҳ�������ɸ����ļ�Ƭ��*/
typedef struct
{	/*368 bytes*/
	UINT32 saveDeviceType;	                /*�洢�豸����*/
	UINT32 frameNumber;						/*�ļ�Ƭ�ϸ���*/
	UINT32 file[ITEM_FILE_COPY];          	/*�ļ����*/
	UINT32 startTime[ITEM_FILE_COPY];		/*����Ƭ�ϵĿ�ʼʱ�䣬����Ƭ�ϸ��ƣ���λΪ��*/
	UINT32 endTime[ITEM_FILE_COPY];			/*����Ƭ�ϵĽ���ʱ�䣬����Ƭ�ϸ��ƣ���λΪ��*/
}copyFile_t;

/*
	�ڼ������ݽṹ
*/
typedef struct
{		/* 32 bytes */
	UINT8		weekendStart;					/* ��ĩ��ʼ�� */
	UINT8		weekendDays;					/* ���� */
	UINT16		res[15];						/* ���������������ڼ��� */
}HOLIDAYPARA;

/*
	�쳣�������ݽṹ
*/
typedef struct
{		/* 40 bytes */
	UINT32 		handleType;						/* �쳣����,�쳣����ʽ��"��"��� */
	UINT32		alarmOutTriggered;				/* �������ı������(�˴�������󱨾������Ϊ32) */
	UINT8		IPCalarmOutTriggered[MAX_IPCHANNUM]; /* ��������IPC ������� */
}EXCEPTION;

/*
	���ؼ������������ݽṹ
*/
typedef struct
{		/* 32 bytes */
	UINT32		warnOutChan;					/* 0:�����, 1:�����, 2:�������1, 3:�������2, 4:�������3, 5:�������4 */
	UINT32		warnChanSwitchTime;				/* ����ͨ��������Ԥ���л�ʱ�� */
	UINT8		res[24];
}LOCALWARNPARA;

/*
	�ƶ�����쳣����ṹ
*/
typedef struct
{
	TIMESEGMENT	armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ����ʱ�� */
	EXCEPTION	handleType;							/* �쳣����ʽ */
	UINT64		motTriggerRecChans;					/* �ƶ���ⴥ����¼��ͨ�� */
	UINT8		enableMotDetAlarm;					/* �ƶ���ⱨ�� */
	UINT8		precision;							/* �ƶ�����㷨�Ľ���: 0--16*16, 1--32*32, 2--64*64 ... */
	UINT8		res[6];
}MOTDETEXECPARA;

/*
	�ƶ�������ݽṹ
*/
typedef struct
{		/* 1024 bytes */
#if 0
	UINT64		motionLine[36];					/* 36��44/45�з���ÿ�������ʾ16*16���ء�*/
												/* ÿ��64λ���������ݵ�0-43/44λ�ֱ��Ӧÿ���д����ҵ�44/45���顣*/
#endif
	UINT32		motionLine[64][3];				/* ���ǵ��������ص�IP Camera��������Ļ����Ϊ(96*64)���ƶ���⡮�顯��
												   ÿ����Ĵ�С�ɡ�precision��ȷ����
												   ��������ʹ��500������(2592*1944)��IP Camera�����ÿ������32*32���أ�
												   ������Ļ��Ϊ81*61�顣
												 */
	UINT8		motionLevel;					/* ��̬��������ȣ�ȡ0xffʱ�ر�;(6���;0���) -> (0���;5���) */
				/* �ƶ�����㷨�Ľ���: 0--16*16, 1--32*32, 2--64*64 ... */
	UINT8		res[7];
	MOTDETEXECPARA motdetExec;
} MOTDETECTPARA;

/*
	�ƶ�������ݽṹ��չ(�����������ҵ�IPC)
*/
typedef struct
{		/* 1024 bytes */

	ZONE_EX     motionArea[8];					/* ���ǵ�������չ���ݶ�Ϊ8������*/
	UINT8       res2[672];
	UINT8		motionLevel;					/* ��̬��������ȣ�ȡ0xffʱ�ر�;(6���;0���) -> (0���;5���) */
				/* �ƶ�����㷨�Ľ���: 0--16*16, 1--32*32, 2--64*64 ... */
	UINT8		res[7];
	MOTDETEXECPARA motdetExec;
} MOTDETECTPARA_EX;

/*
	�ڵ��������ݽṹ
*/
typedef struct
{		/* 256 bytes */
	ZONE		area;							/* �ڵ��������� */
	TIMESEGMENT	armTime[MAX_DAYS][MAX_TIMESEGMENT];/* �ڵ���������ʱ�� */
	EXCEPTION	handleType;						/* �ڵ����� */
	UINT8		maskAlarmLevel;					/* ������ */
	UINT8		enableMaskAlarm;				/* �ڵ����� */
	UINT8		res[14];
}MASKALARMPARA;

/*
	��Ƶ/��Ƶ�źŶ�ʧ���ݽṹ
*/
typedef struct
{		/* 240 bytes */
	TIMESEGMENT	armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ��Ƶ�źŶ�ʧ����ʱ�� */
	EXCEPTION	handleType;						/* ��Ƶ�źŶ�ʧ���� */
	UINT8		enableAlarm;					/* ��Ƶ�źŶ�ʧ���� */
	UINT8		res[7];
}SIGNALLOSTPARA;

/*
	�ڵ����ݽṹ
*/
typedef struct
{		/* 40 bytes */
	ZONE		hideArea[MAX_MASK_REGION_NUM];		/* �ڵ����� */
	UINT8		enableHide;						/* �Ƿ������ڵ� */
	UINT8		res[7];
}VIHIDEPARA;

/*
	RS232 parameter
*/
typedef struct
{		/* 8 bytes */
	UINT8		speed;							/* ������ */
	UINT8		databits;						/* ����λ */
	UINT8		stopbits;						/* ֹͣλ */
	UINT8		parity;							/* У�� */
	UINT8		flowctrl;						/* ���� */
	UINT8		rs232Usage;						/* RS232��ʹ�÷�ʽ */
	UINT8		res[2];
} RS232PARA;

/*
	DVRʵ��Ѳ�����ݽṹ
*/
#define CRUISE_MAX_PRESET_NUMS	32
typedef struct
{		/* 192 bytes */
	UINT8 		presetNo[CRUISE_MAX_PRESET_NUMS];	/* Ԥ�õ�� */
	UINT8 		cruiseSpeed[CRUISE_MAX_PRESET_NUMS];/* Ѳ���ٶ� */
	UINT16 		dwellTime[CRUISE_MAX_PRESET_NUMS];	/* ͣ��ʱ�� */
	UINT8		enableThisCruise;					/* �Ƿ����� */
	UINT8		res[63];
}CRUISE_PARA;

/*
	PTZ���ݽṹ
*/
typedef struct
{		/* 344 bytes */
	UINT8		speed;							/* ������ */
	UINT8		databits;						/* ����λ */
	UINT8		stopbits;						/* ֹͣλ */
	UINT8		parity;							/* У�� */
	UINT8		flowctrl;						/* ���� */
	UINT8		res1[3];
    UINT16		decoderModel;					/* ���������� */
	UINT16		decoderAddress;					/* ��������ַ */
	UINT8 		ptzPreset[MAX_PRESET/8];		/* Ԥ�õ��Ƿ�����: 0--û�����ã� 1--������ */
	UINT8   	ptzCruise[MAX_CRUISE];			/* Ѳ���Ƿ�����  : 0--û�����ã� 1--������ */
	UINT8		ptzTrack[MAX_TRACK];			/* �켣�Ƿ�����  : 0--û�����ã� 1--������ */
	CRUISE_PARA	ptzCruisePara;					/* DVRʵ��PTZѲ�����ܲ��� */
	UINT8		res2[44];
} PANTILTZOOM;

/*
* ��Ƶ��ɫ�ṹ
*/
typedef struct
{	/*4 bytes*/
	UINT8		brightness; 				/* ����   (0-255) */
	UINT8		contrast;					/* �Աȶ� (0-255) */
	UINT8		tonality;					/* ɫ��   (0-255) */
	UINT8		saturation; 				/* ���Ͷ� (0-255) */
}COLOR;

/*
* ��Ƶ��ɫ���ýṹ
*/
typedef struct
{	/*64 bytes*/
	COLOR color[MAX_TIMESEGMENT];
	TIMESEGMENT handleTime[MAX_TIMESEGMENT];
}VICOLOR;

/*
* ¼�����ýṹ
*/
typedef struct
{	/*528 bytes*/
	UINT8			enableRecord;				/* �Ƿ�¼�� */
	UINT8			enableRedundancyRec;		/* �Ƿ�����¼�� */
	UINT8			enableAudioRec;				/* ����������ʱ�Ƿ��¼��Ƶ���� */
#ifdef HI3515
	UINT8 			res1;						/* ��һ���ֽ�*/
#endif
	RECORDDAY		recordDay[MAX_DAYS];		/* ȫ��¼����� */
	RECORDSCHED		recordSched[MAX_DAYS][MAX_TIMESEGMENT];	/* ¼��������� */
	UINT32			preRecordTime;				/* Ԥ¼ʱ�� */
	UINT32			recordDelay;				/* �ƶ����/����¼�����ʱ */
	UINT32			recorderDuration;			/* ¼�񱣴���ʱ�� */
	UINT8		    res2[12];					/* ��Ϊ12�ֽ�*/
}RECORDPARA;

/*
 * Video in(camera) parameters
 */
typedef struct
{		/* 3904 bytes */
	UINT8			channelName[NAME_LEN];		/* ͨ������ */
	VICOLOR 		viColor;					/* ��Ƶ��ɫ���ýṹ*/
	OSDPARA			osdPara;					/* OSD���� */
	LOGOPARA		logoPara;					/* LOGO���� */

	UINT8			videoStandard;				/* ��Ƶ��ʽ*/
	UINT8			enableWork;					/* �Ƿ�ʹ�ܸ�ģ��ͨ��*/
	UINT8			res1[6];

	RECORDPARA		recPara;					/* ¼����� */

	COMPRESSION		normalHighCompPara;			/* ��ʱ��ѹ������ */
	COMPRESSION		normalLowCompPara;			/* ��ʱ��ѹ������ */
	COMPRESSION		eventCompPara;				/* �¼�ѹ������ */
	COMPRESSION		netCompPara;				/* ����ѹ������ */

	SIGNALLOSTPARA	viLostPara;					/* ��Ƶ�źŶ�ʧ���� */
	SIGNALLOSTPARA	aiLostPara;					/* ��Ƶ�źŶ�ʧ���� */
	union
	{
		MOTDETECTPARA	para;					/* �ƶ������� */
		MOTDETECTPARA_EX para_ex;				
	}motDetPara;
	MASKALARMPARA	maskAlarmPara;				/* �ڵ��������� */
	VIHIDEPARA		videoHidePara;				/* ��Ƶ�ڵ����� */

	PANTILTZOOM		ptzPara;					/* ���������� */
	UINT8			res2[240];
} VIDEOIN;

/*
 * Video out���ݽṹ
 */
typedef struct
{		/* 32 bytes */
	UINT8		videoStandard;					/* �����ʽ	*/
	UINT8		menuAlphaValue;					/* �˵��뱳��ͼ��Աȶ� */
	UINT16		idleTime;						/* ��Ļ����ʱ�� */
	UINT16		voOffset;						/* ��Ƶ���ƫ�� */
	UINT16		voBrightnessLevel;				/* ��Ƶ������� */
	UINT8		voMode;							/* ��������Ƶ���ģʽ(0--�˵�/1--Ԥ��) */
	UINT8		enableScaler;					/* �������� */
	UINT8		res[22];
} VIDEOOUT;

/*
	VGA���ݽṹ
*/
typedef struct
{		/* 16 bytes */
	UINT8		resolution;						/* �ֱ��� */
	UINT8		freq;							/* ˢ��Ƶ�� */
	UINT16		brightness;						/* ���� */
	UINT8		res[12];
} VGAPARA;

/*
 * MATRIX��������ṹ
 */
typedef struct
{		/* 80 bytes */
	UINT16		order[MAX_CHANNUM];				/* Ԥ��˳��, 0xff��ʾ��Ӧ�Ĵ��ڲ�Ԥ�� */
	UINT16		switchTime;						/* Ԥ���л�ʱ�� */
	UINT8		res[14];
}MATRIXPARA;

/*
 * Preview parameters
 */
typedef struct
{	/* 280 bytes */
	UINT16		switchInterval;					/* Ԥ���л�������ʱ��(��λ����)��0Ϊ���л� */
	UINT8		mode;							/* ͨ��Ԥ��ģʽ 1: 1���� 2: 4���� 3: 6���� 4: 8����
												                                     5: 9���� 6: 12���� 7: 16���� */
	UINT8		enableAudio;					/* �Ƿ�Ԥ����Ƶ */
	UINT8  		order[MAX_PREV_MOD][MAX_PREV_WINDOWS];				/* Ԥ��˳��, 0xff��ʾ��Ӧ�Ĵ��ڲ�Ԥ�� */
	UINT8		res[20];
} PREVIEW;

/*
 * Monitoralarm parameters
 */
typedef struct
{   /*24 bytes*/
	UINT16      alarmChanSwitchTime;			/* ����ͨ���л�ʱ�� */
	UINT16 		alarmOutChan;					/* ѡ�񱨾�������������ͨ��: 1:�����/ 2:�����/3:VGA*/
	UINT8		res[20];
}MONITORALARM;

/*
 * Ӳ�����ݽṹ
 */
typedef struct
{		/* 640 bytes */
	UINT64		chansUseHD[MAX_HD_COUNT];		/* ����:ʹ�ø�Ӳ�̵�ͨ�� */
	UINT8		HDProperty[MAX_HD_COUNT];		/* Ӳ������:������--0/ֻ����--1/������--2*/
	UINT8		res1[MAX_HD_COUNT];		
	UINT8		checkHDMatch[MAX_HD_COUNT];		/* �Ƿ���Ӳ�̸��� */
	UINT8		cyclicRecord[MAX_HD_COUNT];		/* Ӳ��ѭ��д�� */
	UINT32		HDGroup[MAX_HD_COUNT];		    /* ����ֲ��������Ӧbit λ��ֵΪ��Ӳ�������*/
	UINT8		sataFunc;						/* eSATA���÷�:�̿�/���� */
	UINT8		hdSpaceThreshold;				/* 0-100��Ӳ��ʣ��ռ䲻�㱨�� */
	UINT8		bEnableSMART;					/* S.M.A.R.T���� */
	UINT8  		res2[109];
}HDPARA;

/*
	IP��ַ������IPv4��IPv6��ʽ�ĵ�ַ
*/
typedef struct
{
	/* 24 bytes */
	struct in_addr	v4;							/* IPv4��ַ */
	struct in6_addr	v6;							/* IPv6��ַ */
	UINT8			res[4];
}U_IN_ADDR;

/* ���̵�IP ��ַ������
�ṹ��ĳ�����U_IN_ADDR ����һ��*/
typedef struct
{
	struct in_addr	v4;							/* IPv4��ַ */
	struct in6_addr	v6;							/* IPv6��ַ */
	UINT16			port;						/* �˿ں�*/
	UINT8			type;						/* ���̵�����*/
	UINT8			res;
}NETHD_ADDR_TYPE;

/*
	����ӿ����ݽṹ
*/
typedef struct
{		/* 64 bytes */
	UINT32		mtu;							/* MTU */
   	U_IN_ADDR	ipAddress;						/* IP��ַ */
   	U_IN_ADDR	ipMask;							/* IP���� */
	UINT16		ipPortNo;						/* IP�˿ں� */
	UINT8		macAddr[MACADDR_LEN];			/* �����ַ��ֻ������ʾ */
	UINT8		mediaType;						/* �������� */
	UINT8		res2[23];
} ETHERNET;

/*
	NTP���ݽṹ
*/
typedef struct
{		/* 80 bytes */
	UINT8		ntpServer[64];					/* Domain Name or IP addr of NTP server */
	UINT16		interval;						/* adjust time interval(hours) */
	UINT8		enableNTP;						/* enable NPT client */
	UINT8       res1;
	UINT16      ntpPort;							/*IP port for NTP server*/
	UINT8	    res2[10];
}NTPPARA;

/*
	DDNS���ݽṹ
*/
typedef struct{
	UINT8		username[NAME_LEN];				/* DDNS�˺��û���/���� */
	UINT8		password[PASSWD_LEN];
	UINT8		domainName[MAX_DOMAIN_NAME];		/* ���� */
	UINT8		serverName[MAX_DOMAIN_NAME];	/* DDNS server ����/IP */
	UINT16		ddnsPort;                       /* DDNS port*/
	UINT8       res[10];
}SINGLE_DDNS;

#define MAX_DDNS_NUMS        10
typedef struct
{		/* 144 bytes */
	UINT8		enableDDNS;						/* �Ƿ����� */
	UINT8		hostIdx;								/* ѡ������DNS Э��*/
	UINT8       res[2];
	SINGLE_DDNS ddns[MAX_DDNS_NUMS];
}DDNSPARA;

/*
	SMTP���ݽṹ
*/
typedef struct
{		/* 640 bytes */
	UINT8		account[NAME_LEN];				/* �˺�/���� */
	UINT8		password[NAME_LEN];
	struct
	{
		UINT8	name[NAME_LEN];					/* ���������� */
		UINT8	address[MAX_DOMAIN_NAME];		/* �����˵�ַ */
	}sender;
	UINT8		smtpServer[MAX_DOMAIN_NAME];	/* smtp������ */
	UINT8		pop3Server[MAX_DOMAIN_NAME];	/* pop3������ */
	struct
	{
		UINT8	name[NAME_LEN];					/* �ռ������� */
		UINT8	address[MAX_DOMAIN_NAME];		/* �ռ��˵�ַ */
	}receiver[3];								/* ����������3���ռ��� */
	UINT8		bAttachment;					/* �Ƿ������ */
	UINT8		bSmtpServerVerify;				/* ���ͷ�����Ҫ�������֤ */
	UINT8       mailinterval;					/* ʱ���� */
	UINT8 	bSslServerVerify;                   /*�Ƿ�����ssl�����ʼ�*/
	UINT32    smtpServerPort;                      /*ssl �������˿�*/
	UINT8   res[56];
}EMAILPARA;

/*
	NFS���ݽṹ
	attention:
	caofeng 2009.6 ����ISCSI ����Ӳ�̣�Ϊ����NAS ����һ�²���С�Ķ���������Ȼʹ��nfs
*/
typedef struct
{		/* 152 bytes */
	NETHD_ADDR_TYPE nfsHostIPaddr;				/* ���̵�IP ��ַ������(NAS, ISCSI...)*/
	UINT8		nfsDirectory[128];				/* NFS�����׳���Ŀ¼�� */
}NFSPARA;

typedef NFSPARA ISCSIPARA;

/*
	�������ݽṹ
*/
typedef struct
{		/* 3936 bytes */
	ETHERNET	etherNet[MAX_ETHERNET];			/* ��̫���� */
	U_IN_ADDR	manageHost1IpAddr;				/* ����������IP��ַ */
	U_IN_ADDR	manageHost2IpAddr;				/* ����������IP��ַ */
	U_IN_ADDR	alarmHostIpAddr;				/* ��������IP��ַ */
	UINT16 		manageHost1Port;				/* �����������˿ں� */
	UINT16 		manageHost2Port;				/* �����������˿ں� */
	UINT16		alarmHostIpPort;				/* ���������˿ں� */
	UINT8       	bUseDhcp;						/* �Ƿ�����DHCP */
	UINT8		res1[9];
	U_IN_ADDR	dnsServer1IpAddr;				/* ����������1��IP��ַ */
	U_IN_ADDR	dnsServer2IpAddr;				/* ����������2��IP��ַ */
	UINT8		ipResolver[MAX_DOMAIN_NAME];	/* IP����������������IP��ַ */
	U_IN_ADDR	ipResolverAddr;                 /* IP����������IP��ַ */
	UINT16		ipResolverPort;					/* IP�����������˿ں� */
	UINT16		httpPortNo;						/* HTTP�˿ں� */
	UINT8		res2[4];
	U_IN_ADDR	multicastIpAddr;				/* �ಥ���ַ */
	U_IN_ADDR	gatewayIpAddr;					/* ���ص�ַ */
	NFSPARA		nfsDiskParam[MAX_NFS_DISK]; 	/* ����Ӳ��(NAS, ISCSI ...) */
	NTPPARA		ntpClientParam;					/* NTP���� */
	DDNSPARA	ddnsClientParam;				/* DDNS���� */
	EMAILPARA	emailParam;						/* EMAIL���� */
	UINT8		res[256];						/* !!! SNMP�Ƿ���Ҫ���� */
} NETWORK;

/*
	PPP���ݽṹ
*/
typedef struct
{		/* 160 bytes */
	U_IN_ADDR	remoteIpAddr;					/* Զ��IP��ַ */
	U_IN_ADDR	localIpAddr;					/* ����IP��ַ */
	UINT32 		localIpMask;					/* ������������ */
	UINT8		username[NAME_LEN];				/* �û��� */
	UINT8		password[PASSWD_LEN];			/* ���� */
	UINT16		mru;							/* �����ճ��� */
	UINT16		pppAttribute;					/* PPPģʽ���ز����ز�ģʽ������ */
	UINT8		phoneNumber[PHONENUMBER_LEN];	/* �绰����/�ز����� */
	UINT8		res[24];
} PPPPARA;

/*
	PPPoE���ݽṹ
*/
typedef struct
{		/* 80 bytes */
	UINT8		username[NAME_LEN];				/* �û��� */
	UINT8		password[PASSWD_LEN];			/* ���� */
	UINT8		enablePPPoE;					/* ����PPPoE */
	UINT8		res[31];
} PPPOEPARA;

/*
	�����������ݽṹ
*/
typedef struct
{		/* 632 bytes */
	UINT8		alarmInName[NAME_LEN];			/* ���� */
	UINT8		sensorType;						/* ���������ͣ�0Ϊ������1Ϊ���� */
	UINT8		bEnableAlarmIn;					/* ���������� */
	UINT8		res1[6];
	UINT64		alarmInTriggerRecChans;			/* �������봥����¼��ͨ�� */
	TIMESEGMENT armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ����ʱ��� */
	EXCEPTION	alarmInAlarmHandleType;			/* �������봦�� */
	UINT8		enablePreset[MAX_CHANNUM];		/* �Ƿ����Ԥ�õ� */
	UINT8		presetNo[MAX_CHANNUM];			/* ���õ���̨Ԥ�õ���� */
	UINT8		enablePresetRevert[MAX_CHANNUM];/* �Ƿ�ָ�������Ԥ�õ�ǰ��λ�� */
	UINT16		presetRevertDelay[MAX_CHANNUM];	/* �ָ�Ԥ�õ���ʱ */
	UINT8      	enablePtzCruise[MAX_CHANNUM];	/* �Ƿ����Ѳ�� */
	UINT8		ptzCruise[MAX_CHANNUM];			/* Ѳ�� */
	UINT8      	enablePtzTrack[MAX_CHANNUM];	/* �Ƿ���ù켣 */
	UINT8		trackNo[MAX_CHANNUM];			/* ��̨�Ĺ켣���(ͬԤ�õ�) */
	UINT8		res2[64];
} ALARMIN;

/*
	����������ݽṹ
*/
typedef struct
{		/* 272 bytes */
	UINT8		alarmOutName[NAME_LEN];			/* ���� */
	TIMESEGMENT	alarmOutTimeSeg[MAX_DAYS][MAX_TIMESEGMENT];	/* �����������ʱ��� */
	UINT32		alarmOutDelay;					/* ���������ʱ */
	UINT8		res[12];
} ALARMOUT;

/*
	�û����ݽṹ
*/
typedef struct
{		/* 184 bytes */
	UINT8		username[NAME_LEN];				/* �û��� */
	UINT8		password[PASSWD_LEN];			/* ���� */
	UINT32		permission;						/* Ȩ�� */
	U_IN_ADDR	ipAddr;							/* �û�IP��ַ(Ϊ0ʱ��ʾ�����κε�ַ) */
	UINT8		macAddr[MACADDR_LEN];			/* MAC address */
	UINT8		priority;						/* ���ȼ���0--�ͣ�1--�У�2--�� */
	UINT8		res1[13];
	UINT64		localPrevPermission;			/* Ԥ��Ȩ�� */
	UINT64		netPrevPermission;
	UINT64		localRecordPermission;			/* ¼��Ȩ�� */
	UINT64		netRecordPermission;
	UINT64		localPlayPermission;			/* �ط�Ȩ�� */
	UINT64		netPlayPermission;
	UINT64		localPtzCtrlPermission;			/* PTZ����Ȩ�� */
	UINT64		netPtzCtrlPermission;
	UINT64		localBackupPermission;			/* ���ر���Ȩ�� */
	UINT8		res2[16];
} USER;

/* ATM���Ų�׽���ݽṹ
*/
typedef struct
{
	UINT8 code[12];
}FRAMECODE;

typedef struct
{		/* 240 bytes */
	UINT32 		inputMode;						/* ���뷽ʽ */
	U_IN_ADDR   atmip;							/* ATM��IP */
	UINT32 		atmtype;						/* ATM������ */
	UINT32 		frameSignBeginPos;				/* ���ı�־λ����ʼλ�� */
	UINT32 		frameSignLength;				/* ���ı�־λ�ĳ��� */
	UINT8  		frameSignContent[12];			/* ���ı�־λ������ */
	UINT32 		cardLengthInfoBeginPos;			/* ���ų�����Ϣ����ʼλ�� */
	UINT32 		cardLengthInfoLength;			/* ���ų�����Ϣ�ĳ��� */
	UINT32 		cardNumberInfoBeginPos;			/* ���ŵ���ʼλ�� */
	UINT32 		cardNumberInfoLength;			/* ���ŵĳ��� */
	UINT32 		businessTypeBeginPos;			/* �������͵���ʼλ�� */
	UINT32 		businessTypeLength;				/* �������͵ĳ��� */
	FRAMECODE	frameTypeCode[10];				/* �� */
	UINT16		ATMPort;						/* ���Ų�׽�˿ں�(����Э�鷽ʽ) */
	UINT16		protocolType;					/* ����Э������ */
	UINT8		res[60];
}ATM_FRAMETYPE;

/*
	DM6467 ATM���Ų�׽���ݽṹ
*/
#define MAX_ATM				4
#define MAX_ATM_NUM		1
#define MAX_ACTION_TYPE	12	
#if 0
typedef struct
{
	UINT8					code[12];
}FRAMECODE;
#endif
typedef struct	/*ɸѡ����*/
{	/*32 bytes*/
	UINT8					benable;					/*0,������;1,����*/
	UINT8					mode;					/*0,ASCII;1,HEX*/	
	UINT8					textPos;					/*�ַ���λ�ã�λ�ù̶�ʱʹ��*/
	UINT8					res1[1];
	UINT8 					filterText[16];			/*�����ַ���*/
	UINT8					res[12];
}FILTER;

typedef struct	/*��ʼ��ʶ����*/
{	/*40 bytes*/
	UINT8					startMode;				/*0,ASCII;1,HEX*/
	UINT8					endMode;				/*0,ASCII;1,HEX*/
	UINT8					res1[2];
	FRAMECODE				startCode;				/*��ʼ�ַ�*/
	FRAMECODE				endCode;				/*�����ַ�*/
	UINT8					res[12];
}IDENTIFICAT;

typedef struct	/*������Ϣλ��*/
{	/*32 bytes*/
	UINT32 					offsetMode;				/*0,token;1,fix*/
	UINT32  					offsetPos;				/*modeΪ1��ʱ��ʹ��*/
	FRAMECODE				tokenCode;				/*��־λ*/
	UINT8					multiplierValue;			/*��־λ���ٴγ���*/
	UINT8					externOffset;			/*���ӵ�ƫ����*/
	UINT8					codeMode;				/*0,ASCII;1,HEX*/
	UINT8 					res[9];					
}PACKAGE_LOCATION;

typedef struct	/*������Ϣ����*/
{	/*48 bytes*/
	UINT32					lengthMode;				/*�������ͣ�0,variable;1,fix;2,get from package(���ÿ��ų���ʹ��)*/
	UINT32  					fixLength;				/*modeΪ1��ʱ��ʹ��*/
	UINT32					maxLength;
	UINT32					minLength;
	UINT32					endMode;				/*�ս��0,ASCII;1,HEX*/	
	FRAMECODE				endCode;				/*�ս��*/
	UINT32					lengthPos;				/*lengthModeΪ2��ʱ��ʹ�ã����ų����ڱ����е�λ��*/
	UINT32					lengthLen;				/*lengthModeΪ2��ʱ��ʹ�ã����ų��ȵĳ���*/
	UINT8					res[8];
}PACKAGE_LENGTH;

typedef struct	/*OSD ���ӵ�λ��*/
{	/*20 bytes*/
	UINT32					positionMode;			/*���ӷ�񣬹�2�֣�0������ʾ��1��Custom*/
	UINT32 					pos_x;					/*x���꣬positionmodeΪCustomʱʹ��*/
	UINT32					pos_y;					/*y���꣬positionmodeΪCustomʱʹ��*/
	UINT8					res[8];
}OSD_POSITION;

typedef struct	/*������ʾ��ʽ*/
{	/*60 bytes*/
	UINT8					item1;					/*Month,0.mm;1.mmm;2.mmmm*/							
	UINT8 					item2;					/*Day,0.dd;*/									
	UINT8 					item3;					/*Year,0.yy;1.yyyy*/	
	UINT8					dateform;				/*0~5��3��item���������*/
	UINT8					res1[20];
	char						seprator[4];				/*�ָ���*/
	char	    					displayseprator[4];		/*��ʾ�ָ���*/
	UINT32					displayform;				/*0~5��3��item���������*/
	UINT8					res[24];
}DATE_FORMAT;
typedef struct	/*ʱ����ʾ��ʽ*/
{	/*60 bytes*/
	UINT32					timeform;				/*1. HH MM SS;0. HH MM*/
	char 					res1[20];
	UINT32					hourmode;				/*0,12;1,24*/
	char						seprator[4]; 				/*���ķָ�������ʱû��*/
	char						displayseprator[4];		/*��ʾ�ָ���*/
	UINT32	    				displayform;				/*0~5��3��item���������*/
	UINT32					displayhourmode;			/*0,12;1,24*/
	UINT8					res[16];
}TIME_FORMAT;

typedef struct
{	/*24 bytes*/
	UINT64					channel;					/*���ӵ�ͨ��*/
	UINT32					delayTime;				/*������ʱʱ��*/
	UINT8					benableDelayTime;		/*�Ƿ����õ�����ʱ�������˿�����ʱ����*/
	UINT8					res[11];
}OVERLAY_CHANNAL;

typedef struct
{	/*84 bytes*/
	PACKAGE_LOCATION		location;
	OSD_POSITION			position;
	FRAMECODE				actionCode;				/*�������͵ȶ�Ӧ����*/
	FRAMECODE				preCode;				/*�����ַ�ǰ���ַ�*/
	UINT8					actionCodeMode;			/*�������͵ȶ�Ӧ����0,ASCII;1,HEX*/
	UINT8					res[7];
}ATM_PACKAGE_ACTION;

typedef struct
{	/*120 bytes*/
	PACKAGE_LOCATION		location;
	DATE_FORMAT			dateForm;
	OSD_POSITION			position;
	UINT8					res[8];
}ATM_PACKAGE_DATE;

typedef struct
{	/*120 bytes*/
	PACKAGE_LOCATION		location;
	TIME_FORMAT				timeForm;
	OSD_POSITION			position;
	UINT8					res[8];
}ATM_PACKAGE_TIME;

typedef struct
{	/*120 bytes*/
	PACKAGE_LOCATION		location;
	PACKAGE_LENGTH			length;
	OSD_POSITION			position;
	FRAMECODE				preCode;				/*�����ַ�ǰ���ַ�*/
	UINT8					res[8];
}ATM_PACKAGE_OTHERS;

typedef struct
{		/* 1804 bytes*/
	UINT8					benable;					/*�Ƿ�����0,������;1,����*/
	UINT8					inputMode;				/*���뷽ʽ:������������ڼ���������Э�顢����Э�顢���ڷ�����*/
	UINT8					res1[2];
	UINT8					atmName[32];			/*ATM ����*/
	U_IN_ADDR				atmIp;					/*ATM ��IP  */
	UINT16					atmPort;					/* ���Ų�׽�˿ں�(����Э�鷽ʽ) �򴮿ڷ������˿ں�*/
	UINT8					res2[2];
	UINT32					atmType;				/*ATM ������*/
	IDENTIFICAT				identification;				/*���ı�־*/
	FILTER					filter;					/*��������*/
	ATM_PACKAGE_OTHERS		cardNoPara;				/*���ӿ�������*/
	ATM_PACKAGE_ACTION		tradeActionPara[MAX_ACTION_TYPE];	/*���ӽ�����Ϊ����*/
	ATM_PACKAGE_OTHERS		amountPara;				/*���ӽ��׽������*/
	ATM_PACKAGE_OTHERS		serialNoPara;			/*���ӽ����������*/
	OVERLAY_CHANNAL		overlayChan;				/*����ͨ������*/
	ATM_PACKAGE_DATE		datePara;				/*�����������ã���ʱ����*/
	ATM_PACKAGE_TIME		timePara;				/*����ʱ�����ã���ʱ����*/
	UINT8					res[32];
}ATM_FRAMETYPE_NEW;


/*
	�Զ��������ݽṹ�����԰�ͨ������ʱ���
*/
typedef struct
{		/* 208 bytes */
	UINT8		enableAutoBackup;				/* �Ƿ����Զ����� */
	UINT8		backupDelay;					/* ������ʱ */
	UINT8		days;							/* ��������,1��ʾ���� */
	UINT8		res[5];
	UINT64  	channels;						/* ��Ҫ���ݵ�ͨ��,��λ��ʾ */
	UINT8		startHour[MAX_CHANNUM];			/* ��ʼʱ��-Сʱ */
	UINT8		startMinute[MAX_CHANNUM];		/* ��ʼʱ��-���� */
	UINT8		startSecond[MAX_CHANNUM];		/* ��ʼʱ��-�� */
	UINT8		endHour[MAX_CHANNUM];			/* ����ʱ��-Сʱ */
	UINT8		endMinute[MAX_CHANNUM];			/* ����ʱ��-���� */
	UINT8		endSecond[MAX_CHANNUM];			/* ����ʱ��-�� */
}AUTOBACKUP;


/*
	����IPcamera ���� ��Ƶ�źŶ�ʧ,�ڵ�����,�ƶ����
	�쳣��������ݽṹ
*/
typedef struct
{
	SIGNALLOSTPARA  viLostPara;					/* ��Ƶ�źŶ�ʧ���� */
	MASKALARMPARA   maskAlarmPara;				/* �ڵ��������� */
	MOTDETEXECPARA	motDetPara;					/* �ƶ������� */
}IPCPICPARA;

/*
	IPcamrea �����ṹ
*/
#define HD_IPC 			31
#define HD_IPC_862F 	32
#define IP_DOME     	40
#define IP_DOME_HD     	41
#define IP_DOME_HD_B    42

typedef struct
{
	UINT32 bExist;
	UINT32 bValid;
	UINT32 ip; 			/* host byte ordered */
	UINT8 mac[MACADDR_LEN];
	UINT16 port; 	/* rtsp port */
	UINT16 cmdPort;		/* http port */
	UINT16 factory;
	UINT32 channel;
	UINT8 mode;			/*0-main stream; 1-sub stream*/
	UINT8 devType;
	UINT8 alarmInNums;
	UINT8 alarmOutNums;
	UINT8 usr[NAME_LEN];
	UINT8 pwd[PASSWD_LEN];
	VIDEOIN	ipcVideo;
	ALARMIN  	alarmInPara[MAX_IPC_ALARMIN];
	ALARMOUT	alarmOutPara[MAX_IPC_ALARMOUT];
	UINT8 ipcDomain[MAX_DOMAIN_NAME];   /*IPC ����*/
	UINT8 res2[64];
}IPCAMERA_PARA;

/*
	����ʱ���ݽṹ
*/
typedef struct
{		/* 8 bytes */
	UINT8 		mon;
	UINT8 		weekNo;
	UINT8 		date;
	UINT8 		hour;
	UINT8 		min;
	UINT8		res[3];
}TIMEPOINT;

typedef struct
{		/* 40 bytes */
	UINT32		zoneIndex;  					/* 0:PST, 1:MST, 2:CST, 3:EST, 4:HST, 5:AST, 6:AKST */
	UINT8		res1[12];
	UINT32		DSTBias;						/* ʱ�����ֵ */
	UINT8		enableDST;						/* ��������ʱ */
	UINT8		res[3];
	TIMEPOINT	startpoint;						/* ����ʱ��ֹʱ�� */
	TIMEPOINT   endpoint;
}ZONEANDDST;

/*
	��ʱ���ػ����ݽṹ
*/
typedef struct
{		/* 96 bytes */
	TIMESEGMENT autoBootShut[MAX_DAYS];			/* ����/�ػ�ʱ�� */
	UINT8		enableAutoBootShut;				/* �Ƿ��Զ����ػ� */
	UINT8		res[67];
}AUTOBOOT;

/*
	������Ƶ����
*/
/* ��ͼ��ʽ */
typedef enum _VCA_PICTURE_FORMAT_
{
  	VCA_PICTURE_JPEG	=  0,					/* JPEG��ʽ */
 	VCA_PICTURE_YUV420  =  1					/* YUV420 ��ʽ */
}VCA_PICTURE_FORMAT;

/**** ����ʶ�� ****/
/* ʶ������ */
#define  VCA_IMAGE_RECOGNIZE	0				/* ��̬ͼ��ʶ�� */
#define  VCA_VIDEO_RECOGNIZE	1				/* ��̬��Ƶʶ�� */

/* ʶ�𳡾� */
#define  VCA_LOW_SPEED_SCENE	0				/* ����ͨ���������շ�վ��С���ſڡ�ͣ������*/
#define  VCA_HIGH_SPEED_SCENE	1				/* ����ͨ�����������ڡ����ٹ�·���ƶ����飩*/

/* ʶ������־ */
	/* ͼ��ʶ��ģʽ */
#define  VCA_IMAGE_RECOGNIZE_FAILURE	0		/* ʶ��ʧ�� */
#define  VCA_IMAGE_RECOGNIZE_SUCCESS	1		/* ʶ��ɹ� */
	/* ��Ƶʶ��ģʽ */
#define  VCA_VIDEO_RECOGNIZE_FAILURE					0	/* ʶ��ʧ�� */
#define  VCA_VIDEO_RECOGNIZE_SUCCESS_OF_BEST_LICENSE	1	/* ʶ�𵽱��ϴ�׼ȷ�Ը��ߵĳ��� */
#define  VCA_VIDEO_RECOGNIZE_SUCCESS_OF_NEW_LICENSE		2	/* ʶ���µĳ��� */

/* ����λ��*/
typedef struct _VCA_PLATE_RECT_
{	/* 16 bytes */
	int		left;
	int		top;
	int		right;
	int		bottom;
} VCA_PLATE_RECT;

/* �������� */
typedef enum _VCA_PLATE_TYPE_
{
	VCA_STANDARD92_PLATE,						/* ��׼���ó������ */
	VCA_STANDARD02_PLATE,						/* 02 ʽ���ó���  */
	VCA_WJPOLICE_PLATE,							/* �侯�� */
	VCA_JINGCHE_PLATE							/* ���� */
}VCA_PLATE_TYPE;

/* ������ɫ */
typedef enum _VCA_PLATE_COLOR_
{
	VCA_BLUE_PLATE,								/* ��ɫ���� */
	VCA_YELLOW_PLATE,							/* ��ɫ���� */
	VCA_WHITE_PLATE,							/* ��ɫ���� */
	VCA_BLACK_PLATE								/* ��ɫ���� */
}VCA_PLATE_COLOR;

/* ����ʶ����� */
typedef struct _VCA_PLATE_PARAM_
{	/* 320 bytes */
	UINT8		bEnable;						/* ��������ʶ�� */
	UINT8		bCapturePic;					/* �Ƿ��ͼ */
	UINT8 		picFormat;						/* ��ͼ��ѹ����ʽ,VCA_PICTURE_FORMAT */
	UINT8		res1;
	UINT16		jpegSize;						/* JPEGͼƬ�Ĵ�С */
	UINT16		jpegQuality;					/* JPEGͼƬ������ */
	UINT64		vcaTriggerRecChans;				/* ����ʶ�𴥷���¼��ͨ�� */
	TIMESEGMENT	armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ����ʶ�𲼷�ʱ�� */
	EXCEPTION	handleType;						/* ����ʶ���� */
	UINT32		frameInterval;					/* ��� frmInterval ��һ��ʶ��,��ͼ��ʶ��ģʽ��Ч,������СΪ 20 */
	VCA_PLATE_RECT	search_rect;				/* �������� */
	VCA_PLATE_RECT	invalidate_rect;			/* ��Ч���������������ڲ� */
	UINT32		minPlateWidth;					/* ��С���ƿ�ȣ�������Ϊ 80 */
	UINT8		recogniseScene;					/* ʶ�𳡾�(���ٺ͸���) */
	UINT8		recogniseType;					/* ʶ������(��̬�Ͷ�̬) */
	UINT16		firstChar;						/* ʶ��Ӧ�ö�Ӧ�ĵ���ʡ�ݼ��,��"��"��*/
	UINT8		res2[28];
}VCA_PLATE_PARAM;

/* ����ʶ���� */
typedef struct _VCA_PLATE_RESULT_
{
	VCA_PLATE_RECT	plate_rect;					/* ����λ�� */
	UINT8	plate_class;						/* ��������, VCA_PLATE_TYPE */
	UINT8	plate_color;						/* ������ɫ, VCA_PLATE_COLOR */
	UINT	res[2];
	UINT8	license[16];						/* ���ƺ��� */
	UINT8	believe[16];						/* ����ʶ���ַ������Ŷȣ����⵽����"�� A12345",����
                                 				   Ϊ 10,20,30,40,50,60,70�����ʾ"��"����ȷ�Ŀ�����
                                 				   ֻ�� 10%��"A"�ֵ���ȷ�Ŀ������� 20% */
}VCA_PLATE_RESULT;
/*********/

/**** ��Ϊ���� ****/
#define  VCA_MAX_ALARM_LINE_NUM   	5			/* ���֧������������ */
#define  VCA_MAX_ALARM_AREA_NUM  	5			/* ���֧������������� */

/* ������ */
typedef struct _VCA_POINT_
{	/* 8 bytes */
 	UINT32		x;								/* X������ */
 	UINT32		y;								/* Y������ */
}VCA_POINT;

/* ����Ŀ������ */
typedef enum _VCA_TARGET_CLASS_
{
  	VCA_TARGET_NOTHING	= 0,					/* ��ЧĿ�� */
  	VCA_TARGET_HUMAN	= 1,					/* �� */
  	VCA_TARGET_VEHICLE	= 2,					/* �� */
  	VCA_TARGET_ANYTHING	= 3						/* �˻� */
}VCA_TARGET_CLASS;

/* �����߱�������;ע�⣺���������Ϊˮƽ�ߣ�����"���϶���"��Ϊ"��������"  */
typedef enum _VCA_CROSS_DIRECTION_
{
  	VCA_LEFT_GO_RIGHT	= 0,					/* �������� */
  	VCA_RIGHT_GO_LEFT	= 1,					/* �������� */
 	VCA_BOTH_DIRECTION  = 2						/* ˫�� */
}VCA_CROSS_DIRECTION;

/* ������ */
typedef struct _VCA_ALARM_LINE_
{	/* 24 bytes */
  	UINT8		bActive;						/* �˾������Ƿ���Ч */
   	UINT8		alarm_target;					/* ����Ŀ������, VCA_TARGET_CLASS */
  	UINT8		alarm_direction;				/* �����߱�������, VCA_CROSS_DIRECTION */
	UINT8		res[5];
  	VCA_POINT	start;							/* ��������� */
  	VCA_POINT	end;							/* �������յ� */
}VCA_ALARM_LINE;

/* �������������� */
typedef enum _VCA_AREA_ALARM_CLASS_
{
 	VCA_ENTER_AREA	= 0,						/* Ŀ��������� */
  	VCA_EXIT_AREA	= 1,						/* Ŀ���뿪���� */
  	VCA_INSIDE_AREA = 2							/* ��������Ŀ�� */
}VCA_AREA_ALARM_CLASS;

/* ������ */
typedef struct _VCA_ALARM_AREA_
{	/* 88 bytes */
	UINT8		bActive;						/* �˾��������Ƿ���Ч */
	UINT8		alarm_target;					/* ����Ŀ������, VCA_TARGET_CLASS */
	UINT8  		alarm_class;					/* ��������������, VCA_AREA_ALARM_CLASS */
	UINT8		res;
	UINT32		pointNum;						/* ���������10 ������ */
	VCA_POINT	pos[10];						/* �������� */
}VCA_ALARM_AREA;

/* �û��趨�������� */
typedef struct _VCA_RULE_PARAM_
{	/* 832 bytes */
	UINT8			bEnable;					/* ������Ϊ���� */
	UINT8			bCapturePic;				/* �Ƿ��ͼ */
	UINT8 			picFormat;					/* ��ͼ��ѹ����ʽ,VCA_PICTURE_FORMAT */
	UINT8			res1;
	UINT16			jpegSize;					/* JPEGͼƬ�Ĵ�С */
	UINT16			jpegQuality;				/* JPEGͼƬ������ */
	UINT64			vcaTriggerRecChans;			/* ��Ϊ����������¼��ͨ�� */
	TIMESEGMENT		armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ��Ϊ��������ʱ�� */
	EXCEPTION		handleType;					/* ��Ϊ�������� */
	UINT16 			human_width; 				/* �˵Ĵ��¿�� */
	UINT16 			human_height; 				/* �˵Ĵ��¸߶� */
	UINT16			vehicle_width; 				/* ���Ĵ��¿�� */
	UINT16			vehicle_height;				/* ���Ĵ��¸߶� */
 	VCA_ALARM_AREA	alarm_area[VCA_MAX_ALARM_AREA_NUM];	/* �����߹��� */
 	VCA_ALARM_LINE	alarm_line[VCA_MAX_ALARM_LINE_NUM];	/* ����������� */
	UINT8			res2[16];
}VCA_RULE_PARAM;

/* ��Ϊ������� */
typedef struct _VCA_BEHAVIOR_ANALYSE_RESULT_
{
	struct{
		UINT8		bAlert;						/* 1-���� */
		UINT8		alarm_target;				/* ����Ŀ������, VCA_TARGET_CLASS */
		UINT8		alarm_direction;			/* �����߱�������, VCA_CROSS_DIRECTION */
		UINT8		res;
		COORDINATE 	start;
		COORDINATE 	end;
	}line_alert[VCA_MAX_ALARM_LINE_NUM];
	struct{
		UINT8		bAlert;						/* 1-���� */
		UINT8		alarm_target;				/* ����Ŀ������, VCA_TARGET_CLASS */
		UINT8		alarm_class;				/* ��������������, VCA_AREA_ALARM_CLASS */
		UINT8		pointNum;					/* ������� */
		COORDINATE	pos[10];					/* �������� */
	}area_alert[VCA_MAX_ALARM_AREA_NUM];
} VCA_BEHAVIOR_ANALYSE_RESULT;
/********/

/**** ������� ****/
#define VCA_MAX_FACE_CNT         10              /*ÿ�μ��������������*/
typedef struct
{	/* 8 bytes */
	short x,y,w,h;
}VCA_FACE_AREA;

/* ���������� */
typedef struct _VCA_FD_PARAM_
{	/*  272 bytes */
	UINT8		bEnable;						/* ��������ʶ�� */
	UINT8		bCapturePic;					/* �Ƿ��ͼ */
	UINT8 		picFormat;						/* ��ͼ��ѹ����ʽ,VCA_PICTURE_FORMAT */
	UINT8		res;
	UINT16		jpegSize;						/* JPEGͼƬ�Ĵ�С */
	UINT16		jpegQuality;					/* JPEGͼƬ������ */
	UINT64		vcaTriggerRecChans;				/* ������ⴥ����¼��ͨ�� */
	TIMESEGMENT	armTime[MAX_DAYS][MAX_TIMESEGMENT];	/* ������Ⲽ��ʱ�� */
	EXCEPTION	handleType;						/* ������⴦�� */
	UINT32		frameInterval;					/* ֡��� */
	UINT8		bLocateEyePos;					/* �Ƿ�λ���� */
	UINT8		bClusterProcess;				/* �Ƿ���о������(�����ظ�����) */
	UINT8		res1[18];
}VCA_FD_PARAM;
/********/

/*
   ʱ����Ϣ
*/
typedef struct
{
	SINT16		zoneHour;  						/*ʱ��Сʱ��Ϣ*/
	SINT16      	zoneMin;						/*ʱ��������Ϣ*/
}TIMEZONE;


/*
   �豸ʱ��
*/
typedef struct
{
	TIMEZONE    timeZone;
	UINT8       dateType;
	UINT8       zoneno;	
	UINT8       res[2];
}TIMEPARA;

/*
	����IPC���ýṹ
*/
#define MAX_IP_DEVICE		32
#define MAX_IP_CHANNUM      32
#define MAX_IP_ALARMIN		128
#define MAX_IP_ALARMOUT		64
typedef struct{
	UINT32 enable;
	char username[NAME_LEN];
	char password[PASSWD_LEN];
	U_IN_ADDR ip;
	UINT16 cmdPort;
	UINT8 devType;
	UINT8 factory;
	UINT8 res[32];
}NET_IP_DEV_PARA;

typedef struct{
	UINT8 enable;
	UINT8 ipID;
	UINT8 channel;
	UINT8 factory;
	UINT8 res[32];
}NET_IP_CHAN_INFO;

typedef struct{
	UINT8 analogChanEnable[MAX_ANALOG_CHANNUM/8];
	NET_IP_CHAN_INFO  ipChanInfo[MAX_IP_CHANNUM];
}NET_IP_CHAN_PARA;

typedef struct{
	UINT8 ipID;
	UINT8 alarmIn;
	UINT8 res[18];
}NET_IP_ALARMIN_PARA;

typedef struct{
	UINT8 ipID;
	UINT8 alarmOut;
	UINT8 res[18];
}NET_IP_ALARMOUT_PARA;

typedef struct{
	NET_IP_DEV_PARA  ipDevPara[MAX_IP_DEVICE];
	NET_IP_CHAN_PARA ipChanPara;
	NET_IP_ALARMIN_PARA ipAlarmInPara[MAX_IP_ALARMIN];
	NET_IP_ALARMOUT_PARA ipAlarmOutPara[MAX_IP_ALARMOUT];
}NET_IP_PARA;

typedef struct{
	UINT16 scale;             /* ���� */
	UINT16 offset;            /* ƫ����*/
}MOUSEPARA;

typedef struct{
	UINT8 bUseWizard;
	UINT8 res[15];
}GUIPARA;

#ifdef PIC_PART_MANAGE
/**@struct PICPARA	  
 * @brief  ͼƬ����
 */
typedef struct
{
    UINT16 picResolution;   /* ץͼ�ֱ��� �豸�˶���:2=D1,3=CIF,4=QCIF, sdk�˶���:0=CIF, 1=QCIF, 2=D1 */
    UINT16 picQuality;      /* ץͼ����   �豸�˶���:80,60,40, sdk�˶���:0-��ã�1-�Ϻã�2-һ�� */ 

}PICPARA;
#endif
/*
	�豸�����ṹ
*/
typedef struct
{
	//UINT32		magicNumber;					/* ���� */
	//UINT32		paraChecksum;					/* У��� */
	//UINT32		paraLength;						/* �ṹ����* У��͡����ȴ�'paraVersion'��ʼ���� */
	//UINT32		paraVersion;					/* �����ļ��İ汾�� */

	//UINT16		deviceType;						/* �豸�ͺţ����ڵ����豸����ʱ����Ƿ���� */
	//UINT8		res1[6];							/* ����6�ֽڣ���������չ�����ļ����Լ���� */

	//UINT32		language;						/* �豸���� */
	//UINT8		voutAndTip;						/* bit0:(�Ƿ���ʾ�¼���ʾ)*/
												/*    bit1:(�Ƿ��VGA ����)
												    bit2:(�Ƿ���ʾmain/aux/spot��Ϣ)*/
	//UINT8		res2[3];

	/* VGA��CVBS��������ڲ���������ǰ�棬���ں�����ʱ��ʼ����Щ���� */
	//VGAPARA		vgaPara[MAX_VGA];				/* VGA���� */
	//VIDEOOUT	videoOut[MAX_VIDEOOUT];			/* �����Ƶ���� */
	//RS232PARA	rs232Ppara[MAX_SERIAL_PORT];	/* RS232�ڲ��� */

	//UINT8		deviceName[NAME_LEN];			/* �豸���� */
	//UINT32		deviceId;						/* �豸��, ����ң���� */
	//UINT32		bEnablePassword;				/* �Ƿ��������������½ */

	//HOLIDAYPARA	holidayPara;						/* ������/�ڼ��ղ��� */
	//ZONEANDDST	zoneDst;						/* ����ʱ���� */
	//TIMEPARA    devTime;							/* �豸ʱ�������Ϣ*/
	//MOUSEPARA   mousePara;                      			/* ������*/
	//VIDEOIN		videoIn[MAX_CHANNUM];			/* ������Ƶ���� */
	//MATRIXPARA 	matrixPara[MAX_AUXOUT];			/* MATRIX���Ԥ������ */
	//PREVIEW		previewPara[MAX_VIDEOOUT];		/* Ԥ������ */
	//MONITORALARM monitorAlarm;					/* �������澯����*/

	//HDPARA		hdiskPara;						/* Ӳ����ز��� */
	NETWORK	networkPara;					/* ������� */
	//PPPPARA		pppPara;						/* PPP���� */
	//PPPOEPARA	pppoePara;						/* PPPoE���� */

	//ALARMIN		alarmInPara[MAX_ALARMIN];		/* ����������� */
	//ALARMOUT	alarmOutPara[MAX_ALARMOUT];	/* ����������� */
	//EXCEPTION	exceptionHandleType[MAX_EXCEPTIONNUM];	/* �쳣����ʽ */

	//USER		user[MAX_USERNUM];				/* �û����� */
	//ATM_FRAMETYPE	frameType;					/* ATM���Ų�׽���� */
	//AUTOBOOT	autoBootShutPara;				/* ��ʱ���ػ����� */
	//AUTOBACKUP	autoBackupPara;					/* �Զ����ݲ��� */
	//IPCAMERA_PARA ipCamera[MAX_IPCHANNUM];	/* IPcamera �豸���� */
#if 0
	NET_IP_PARA netIpPara;                     /* ����IP�豸��������*/
	GUIPARA   guiPara;
#ifdef EVDO
      UINT32 enableEvdo;
      UINT32 enableSms;
      UINT32 EvdoOffLineTime;
      UINT8   cellNumber[EVDO_MAXCELLNUM][EVDO_CELLNUMLEN];
#endif
	/* ���� */
#ifdef EHOME
	struct in_addr ehServerIpaddr;
	UINT16 ehServerPort;
	UINT16 ehAlarmEnable;
	UINT8 ehRes[16];		
	ZONE motionArea[MAX_CHANNUM][MASK_MAX_REGION]; 	/*�ƶ�����ĸ�������ϣ����Ӻ���Ӳ���*/
	UINT8 eHFrontId[32];
#endif
#if 1
	ATM_FRAMETYPE_NEW frameTypeNew[MAX_ATM]; /*��ATM���Ų�׽����*/
#endif
#if defined(DM6467) || defined(HI3515)
	UINT32 colorSharpness[MAX_CHANNUM][MAX_TIMESEGMENT];
#endif

#ifdef PIC_PART_MANAGE
    PICPARA picPara[MAX_CHANNUM];
#endif
#ifdef DZ_BEITONG_VER
    int delayTime;
#endif
#endif
} DEVICECONFIG;

/* extern globals */
//extern DEVICECONFIG *pDevCfgParam;
//extern DEV_HARD_INFO devHardInfo;
//extern BOOT_PARMS bootParms;
extern pthread_mutex_t globalMSem;

#ifdef __cplusplus
}
#endif


#endif

