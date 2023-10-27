#ifndef _HIK_NETFUN_H_
#define _HIK_NETFUN_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h> 
#include <malloc.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/time.h>

#include <net/if.h>
#include <net/route.h>
#include <netinet/tcp.h>
#include <netinet/in.h> 
#include <netinet/if_ether.h>

#include <linux/if_packet.h>
#include <arpa/inet.h>

//#include "QMAPIType.h"
//#include "QMAPINetSdk.h"
//#include "QMAPI.h"

#include "hik_configtypes.h"
//#include "hik_command.h"
//#include "hik_netlogin.h"
#include "hik_netutil.h"
#include "hik_security.h"
#include "pwrapper.h"
#include "d3des.h"
#include "hmac.h"

#define HIK_DEBUG 0
#if HIK_DEBUG
#define HPRINT_INFO(fmt, args...)  printf("[HikInfo|%s:%d(%s)] "fmt,__FILE__,__LINE__,__FUNCTION__, ##args)
#else
#define HPRINT_INFO(fmt, args...) {}
#endif
#define HPRINT_ERR(fmt, args...)   printf("\033[0;31m[HikErr|%s:%d(%s)]\033[0;0m " fmt,__FILE__,__LINE__,__FUNCTION__, ##args)


#define MIN_CLIENT_DATA			sizeof(NETCMD_HEADER)
#define MAX_CLIENT_DATA			(1024*8)//2048
#define MAX_ALARMUP_CONNS		10

//login/logout 
#define NETCMD_LOGIN						0x010000	/* login */
#define NETCMD_RELOGIN						0x010010	/* verify username/password again */
#define NETCMD_LOGOUT						0x010100	/* logout */
#define NETCMD_USEREXCHANGE					0x010200	/* user keep alive */

#define NETCMD_GET_NETWORKCFG				0x110000	//��ȡ�����������
#define NETCMD_SET_NETWORKCFG				0x110001	//���û����������

#define NETCMD_GET_FTPCFG					0x116282	//��ȡFTP����
#define NETCMD_SET_FTPCFG					0x116283	//����FTP����
#define NETCMD_GET_RTSPCFG					0x020c02	//��ȡRTSP����
#define NETCMD_SET_RTSPCFG					0x020c03	//����RTSP����

#define NETCMD_GET_IPCORECFG				0x110150
#define NETCMD_SET_IPCORECFG				0x110151
#define NETCMD_GET_IPCORECFG_V31			0x110152 	/*SDK V3.1���������֣�֧��ʹ��IPC  ��������*/
#define NETCMD_SET_IPCORECFG_V31			0x110153 	/*SDK V3.1���������֣�֧��ʹ��IPC  ��������*/

#define NETCMD_GET_IPALARMINCFG		    	0x110160
#define NETCMD_SET_IPALARMINCFG		    	0x110161
#define NETCMD_GET_IPALARMOUTCFG			0x110170
#define NETCMD_SET_IPALARMOUTCFG			0x110171

#define NETCMD_GET_PICCFG					0x020200	/* picture parameter */
#define NETCMD_SET_PICCFG					0x020201
#define NETCMD_GET_PICCFG_EX				0x020232
#define NETCMD_SET_PICCFG_EX				0x020233
#define NETCMD_GET_PICCFG_V30   			0x110010
#define NETCMD_SET_PICCFG_V30   			0x110011

//PTZ control 
#define NETCMD_PTZ							0x030200	/* ��̨���� */
#define NETCMD_TRANSPTZ						0x030201	/* ͸����̨���� (û��ʹ��)*/
#define NETCMD_GETPTZCONTROL				0x030202	/* ��ȡ��̨����Ȩ */
#define DVR_PTZWITHSPEED					0x030203    /*���ٶȵ���̨����*/

#define GET_CURISE_INFO	                    0x030210   /*��ȡ����̨Ѳ������Ϣ*/

/* alarm */
#define NETCMD_ALARMCHAN					0x030400	/* ���������ϴ�ͨ�� */
#define NETCMD_ALARMCHAN_V30				0x111020	/* ���������ϴ�ͨ�� */
#define NETCMD_SETALARMOUT					0x030401	/* ���Ʊ������ */
#define NETCMD_GETALARMOUT					0x030402	/* ��ȡ������� */
#define NETCMD_GETALARMOUT_V30				0x111010	/* ��ȡ������� */
/* voice talk */
#define NETCMD_STARTVOICECOM	 			0x030500	/* ��ʼ�����Խ� */
#define NETCMD_STARTVOICECOM_V30 			0x111030	/* ��ʼ�����Խ� */
/* transparent channel */
#define NETCMD_SERIALSTART					0x030600	/* ����͸��ͨ�� */
#define NETCMD_SERIALSEND					0x030601	/* ������͸��ͨ��ֱ�ӷ������� */
/* control */
#define NETCMD_SOFTRESTORE_CFG				0x030700	/* �ָ�Ĭ�� ���� */
#define NETCMD_SAVE_CFG						0x030800	/* ������� */
#define NETCMD_REBOOT						0x030900	/* reboot DVR */
#define NETCMD_SHUTDOWN						0x030a00	/* shutdown DVR */
#define NETCMD_UPGRADE						0x030b00	/* upgrade software */
#define NETCMD_FORMAT						0x030c00	/* format hard disk */
#define NETCMD_GET_HDMANAGE  				0x111050	/* format hard disk */
#define NETCMD_SET_HDATTRIBUTE				0x111051	/* format hard disk */
#define NETCMD_GET_HDGROUP  				0x111052	/* format hard disk */
#define NETCMD_SET_HDGROUP					0x111053	/* format hard disk */
#define NETCMD_STARTRECORD					0x030d00	/* start record */
#define NETCMD_STOPRECORD					0x030d01	/* stop record */
#define DVR_SETALARMINPUT   				0x030d02  	/*�ֶ�����*/
#define NETCMD_GET_LOG						0x030e00	/* get log */
#define NETCMD_GET_LOG_V30					0x111000	/* get log */
#define NETCMD_GET_SMART_LOG				0x111001	/* get HD SMART log */

#define NETCMD_GET_WORKSTATUS				0x040000	/* ��ȡ����״̬ */
#define NETCMD_GET_WORKSTATUS_V30			0x111070	/* ��ȡ����״̬ */
#define NETCMD_GET_FRAMEFORMAT  			0x040200  	/* ��ȡ������Ϣ */
#define NETCMD_SET_FRAMEFORMAT  			0x040201  	/* ���ý�����Ϣ */
#define NETCMD_GET_FRAMEFORMAT_NEW  		0x110080  	/* ��ȡ������Ϣ */
#define NETCMD_SET_FRAMEFORMAT_NEW  		0x110081  	/* ���ý�����Ϣ */


#define NETCMD_SET_PIPMODE					0x040400	/* ���û��л�ģʽ */
#define NETCMD_GETBMPBYTIME					0x090004	/* ��ʱ��ץBMPͼ */
#define NETCMD_MAKEIFRAME					0x090100	/* ǿ��I֡���� */
#define NETCMD_MAKESUBIFRAME				0x090101	/* ǿ��I֡���� */

#define NETCMD_GET_DEVICECFG				0x020000	/* device parameter */
#define NETCMD_SET_DEVICECFG				0x020001
#define NETCMD_GET_DEVICECFG_V40    		0x1110c2     /* ��ȡ��չ�豸���� */
#define NETCMD_SET_DEVICECFG_V40    		0x1110c3     /* ��ȡ��չ�豸���� */

#define NETCMD_GET_ALARMINCFG_V30			0x110050	/* alarm in parameter */
#define NETCMD_SET_ALARMINCFG_V30			0x110051

#define NETCMD_GET_DEVICECAPACITY			0x011000
#define NETCMD_GET_ISAPICAPABILITY			0x117000	
#define NETCMD_UNKNOWN_11300b				0x11300b
#define NETCMD_UNKNOWN_02010c				0x02010c
#define NETCMD_UNKNOWN_113305				0x113305

#define NETCMD_UNKNOWN_113004				0x113004	//��ͷ��ʼ��
#define NETCMD_UNKNOWN_113003				0x113003	//�����۽�

#define NETCMD_GET_COMPRESSCFG_V30			0x110040	/* compression parameter */
#define NETCMD_SET_COMPRESSCFG_V30			0x110041

// #define DVR_GET_NETCFG						0x020100 //��ȡ�������
// #define DVR_SET_NETCFG						0x020101 //�����������
#define NETCMD_GET_NETCFG					0x020100	/* network parameter */
#define NETCMD_SET_NETCFG					0x020101	//�����������

#define NETCMD_GET_NETAPPCFG    			0x020110
#define NETCMD_SET_NETAPPCFG    			0x020111
#define NETCMD_GET_NTPCFG					0x020112  	/*��ȡ����Ӧ�ò��� NTP*/
#define NETCMD_SET_NTPCFG					0x020113  	/*��������Ӧ�ò��� NTP*/
#define NETCMD_GET_DDNSCFG					0x020114  	/*��ȡ����Ӧ�ò��� DDNS*/
#define NETCMD_SET_DDNSCFG					0x020115  	/*��������Ӧ�ò��� DDNS*/
#define NETCMD_GET_DDNSCFG_EX   			0x02011a  	/*��ȡ����Ӧ�ò�����չDDNS*/
#define NETCMD_SET_DDNSCFG_EX   			0x02011b  	/*��������Ӧ�ò�����չDDNS*/
#define NETCMD_GET_DDNSCFG_V30 	 			0x110140  	/*��ȡ����Ӧ�ò���9000 DDNS*/
#define NETCMD_SET_DDNSCFG_V30  			0x110141  	/*��������Ӧ�ò���9000 DDNS*/
#define NETCMD_GET_EMAILCFG					0x020116  	/*��ȡ����Ӧ�ò��� EMAIL*/
#define NETCMD_SET_EMAILCFG					0x020117  	/*��������Ӧ�ò��� EMAIL*/
#define NETCMD_GET_EMAILCFG_V30  			0x1100b0
#define NETCMD_SET_EMAILCFG_V30 			0x1100b1

#define NETCMD_EMAIL_TEST					0x111081 	/*Email���Խӿ�*/
#define NETCMD_GET_NFSCFG       			0x020120
#define NETCMD_SET_NFSCFG       			0x020121
#define NETCMD_GET_NASCFG_V30   			0x110090
#define NETCMD_SET_NASCFG_V30   			0x110091


#define NETCMD_SET_PICCFG_ENHANCE			0x020234

#define NETCMD_GET_HDCFG_NEW    			0x10001a
#define NETCMD_SET_HDCFG_NEW    			0x10001b

#define NETCMD_GET_COMPRESSCFG_AUD			0x110042
#define NETCMD_SET_COMPRESSCFG_AUD			0x110043
#define NETCMD_GET_COMPRESSCFG_AUD_CURRENT	0x110044

#define NETCMD_GET_COMPRESSCFG				0x020210	/* compression parameter */
#define NETCMD_SET_COMPRESSCFG				0x020211
#define NETCMD_GET_COMPRESSCFG_EX			0x020218	/* compression parameter */
#define NETCMD_SET_COMPRESSCFG_EX			0x020219
#define NETCMD_GET_EVENTCOMPCFG				0x020212	/* compression parameter */
#define NETCMD_SET_EVENTCOMPCFG				0x020213

#define NETCMD_GET_RELATIONCFG				0x020214	/* relation between alarm and motion */
#define NETCMD_SET_RELATIONCFG				0x020215

#define NETCMD_GET_RECORDCFG				0x020220	/* record parameter */
#define	NETCMD_SET_RECORDCFG				0x020221
#define NETCMD_GET_RECORDCFG_V30			0x110020
#define NETCMD_SET_RECORDCFG_V30			0x110021

#define NETCMD_GET_DECODERCFG				0x020230	/* decoder parameter */
#define NETCMD_SET_DECODERCFG				0x020231
#define NETCMD_GET_PTZPROTOCOL      		0x111080
#define NETCMD_GET_DECODERCFG_V30			0x110070	/* decoder parameter */
#define NETCMD_SET_DECODERCFG_V30			0x110071

#define NETCMD_GET_SHOWSTRING				0x020234	/* ��ȡ�����ַ����� */
#define NETCMD_SET_SHOWSTRING				0x020235	/* ���õ����ַ����� */
#define NETCMD_GET_SHOWSTRING_V30			0x110130	/* ��ȡ�����ַ����� */
#define NETCMD_SET_SHOWSTRING_V30			0x110131	/* ���õ����ַ����� */
#define NETCMD_GET_RS232CFG					0x020310	/* rs232 parameter */
#define NETCMD_SET_RS232CFG					0x020311
#define NETCMD_GET_RS232CFG_V30				0x110060	/* rs232 parameter */
#define NETCMD_SET_RS232CFG_V30				0x110061
#define NETCMD_GET_ALARMINCFG				0x020410	/* alarm in parameter */
#define NETCMD_SET_ALARMINCFG				0x020411
#define NETCMD_GET_ALARMOUTCFG				0x020420	/* alarm out parameter */
#define NETCMD_SET_ALARMOUTCFG				0x020421
#define NETCMD_GET_ALARMOUTCFG_V30			0x1100a0	/* alarm out parameter */
#define NETCMD_SET_ALARMOUTCFG_V30			0x1100a1


#define NETCMD_GET_TIMECFG					0x020500	/* DVR date/time */
#define NETCMD_SET_TIMECFG					0x020501
#define NETCMD_GET_DSTCFG          		 	0x090009
#define NETCMD_SET_DSTCFG          		 	0x09000a
#define NETCMD_GET_RTCTYPE					0x090352	/*rtc ʱ������0 no rtc;1 has */

#define NETCMD_GET_PREVIEWCFG				0x020600	/* preview parameter */
#define NETCMD_SET_PREVIEWCFG				0x020601
#define NETCMD_GET_PREVIEWCFG_V30			0x110100	/* preview parameter */
#define NETCMD_SET_PREVIEWCFG_V30			0x110101
#define NETCMD_GET_AUXPREVCFG				0x020602	/* aux Vout preview parameter,for HF-S */
#define NETCMD_SET_AUXPREVCFG				0x020603
#define NETCMD_GET_AUXPREVCFG_V30			0x110102	/* aux Vout preview parameter,for HF-S */
#define NETCMD_SET_AUXPREVCFG_V30			0x110103

#define NETCMD_GET_AUXVOUTCFG				0x020610	/* aux video out parameter */
#define NETCMD_SET_AUXVOUTCFG				0x020611
#define NETCMD_GET_VIDEOOUTCFG				0x020700	/* video out parameter */
#define NETCMD_SET_VIDEOOUTCFG				0x020701
#define NETCMD_GET_VIDEOOUTCFG_V30			0x110110	/* video out parameter */
#define NETCMD_SET_VIDEOOUTCFG_V30			0x110111

#define NETCMD_GET_MATRIXPARA_V30   		0x100008
#define NETCMD_SET_MATRIXPARA_V30   		0x100009

#define NETCMD_GET_USERCFG					0x020800	/* user parameter */
#define NETCMD_SET_USERCFG					0x020801
#define NETCMD_GET_USERCFG_EX				0x020802  	/* user parameter */
#define NETCMD_SET_USERCFG_EX				0x020803
#define NETCMD_GET_USERCFG_V30				0x110030
#define NETCMD_SET_USERCFG_V30				0x110031

#define NETCMD_GET_EXCEPTIONCFG				0x020900	/* exception parameter */
#define NETCMD_SET_EXCEPTIONCFG				0x020901
#define NETCMD_GET_EXCEPTIONCFG_V30			0x110120	/* exception parameter */
#define NETCMD_SET_EXCEPTIONCFG_V30			0x110121

#define NETCMD_GET_ATMPROTOCOL				0x110082

#define NETCMD_GET_HDCFG					0x100018
#define NETCMD_SET_HDCFG					0x100019
#define NETCMD_GET_AUTOBOOTCFG		    	0x1100c0
#define NETCMD_SET_AUTOBOOTCFG		    	0x1100c1
#define NETCMD_GET_CRUISECFG				0x1100e0
#define NETCMD_SET_CRUISECFG				0x1100e1
#define NETCMD_GET_HOLIDAYCFG				0x1100f0
#define NETCMD_SET_HOLIDAYCFG				0x1100f1

#define DVR_GET_SCALECFG					0x020a04 	/* ��ȡ��������*/
#define DVR_SET_SCALECFG					0x020a05  	/*������������*/
#define DVR_GET_SCALECFG_V30				0x110180 	/* ��ȡ��������*/
#define DVR_SET_SCALECFG_V30				0x110181  	/*������������*/

/*ATM���˿�����*/
#define DVR_GET_ATMPORT    					0x020a06  	/*��ȡATM PORT����*/
#define DVR_SET_ATMPORT    					0x020a07  	/*����ATM PORT����*/
#define DVR_PTZSELZOOMIN 					0x030290	/*�������ѡ������Ŵ�*/

#define NETCMD_GET_JPEGCFG					0x020004	/*��ȡͼƬ����	*/
#define NETCMD_SET_JPEGCFG					0x020005	/*����ͼƬ����*/
#define NETCMD_GET_JPEGPICTURE_EX			0x03000a	/*����jpegץͼ*/

#define NETCMD_GET_DEVCFGFILE   			0x020A00	/*��ȡ��ǰ�����������ļ�*/
#define NETCMD_PUT_DEVCFGFILE				0x020A01	/*���������ļ���������*/

/*
 * ============================================================================
 *                            3. remote  operation
 * ============================================================================
 */
/* preview */
#define NETCMD_TCP_PREV						0x030000	/* TCP preview */
#define NETCMD_UDP_PREV						0x030001	/* UDP preview */
#define NETCMD_MCAST_PREV					0x030002	/* MCAST preview */
#define NETCMD_RTP_PREV						0x030003	/* RTP preview */
#define NETCMD_VIDEO_ONLY					0x030004	/* ����Ƶ�ֿ� */
#define NETCMD_AUDIO_ONLY					0x030005
#define NETCMD_DATAEXCHANGE					0x030006	/* data exchange */
#define NETCMD_GET_VIDEOEFFECT				0x030007	/* ʵʱͼ�����*/
#define NETCMD_SET_VIDEOEFFECT				0x030008
#define NETCMD_GET_COLORENHANCE				0x111096	/*��ȡ��ת��������ز���*/
#define NETCMD_SET_COLORENHANCE				0x111095	/*������ת��������ز���*/

#define NETCMD_GET_VIDEOEFFECT_V30			0x053943 	/*��ʱʵ�֣���Ҫ��SDK��������*/
#define NETCMD_SET_VIDEOEFFECT_V30			0x073456

#define NETCMD_GET_VOLUME					0x030010	/*��ȡÿ��ͨ����������*/

/* playback */
#define NETCMD_FINDFILE						0x030100	/* �����ļ� */
#define NETCMD_FINDFILE_V30					0x111040	/* �����ļ� */
#define NETCMD_PLAYBYFILE					0x030101	/* ���ļ��ط� */
#define NETCMD_PLAYBYTIME					0x030102	/* ��ʱ��ط� */
#define NETCMD_STARTPLAY					0x030103	/* ��ʼ�ط� */
#define NETCMD_STOPPLAY						0x030104	/* ֹͣ�ط�,�ͷ����� */
#define NETCMD_SETPOS						0x030105	/* ���ļ��ط�,�ı䲥�ŵĽ��� */
#define NETCMD_BACKWARD						0x030106	/* ����n�� */
#define NETCMD_PAUSE						0x030107	/* ��ͣ */
#define NETCMD_RESTART						0x030108	/* ���¿�ʼ */
#define NETCMD_EXCHANGE						0x030109	/* �طŽ������� */
#define NETCMD_STREAMSPEED      			0x03010d    /* ���ص��ٶ� */
#define NETCMD_LOCK_FILE        			0x030112
#define NETCMD_UNLOCK_FILE      			0x030113
#define NETCMD_LOCK_BYTIME     				0x030114
#define NETCMD_UNLOCK_BYTIME    			0x030115
#define NETCMD_FINDPICTURE      			0x03010a	/*����ͼƬ*/
#define NETCMD_GETPICTURE					0x03010b	/*����ͼƬ*/
#define NETCMD_GET_JPEGPICTURE				0x030009	/*ץJPEGͼ��*/

//remote control local display 
#define NETCMD_LOCALPREV					0x030300
#define NETCMD_LOCALPLAYBYNAME				0x030301
#define NETCMD_LOCALPLAYBYTIME				0x030302
#define NETCMD_LOCALFAST					0x030303
#define NETCMD_LOCALSLOW					0x030304
#define NETCMD_LOCALSINGLEFRAME				0x030305
#define NETCMD_LOCALNORMAL					0x030306
#define NETCMD_LOCALFORWARD					0x030307
#define NETCMD_LOCALBACKWARD				0x030308
#define NETCMD_LOCALSETPOS					0x030309
#define NETCMD_LOCALGETPOS					0x03030a
#define NETCMD_LOCALSTOPPLAY				0x03030b
#define NETCMD_CLICKKEY						0x03030c
#define NETCMD_START_PANELKEY				0x111060
#define NETCMD_STOP_PANELKEY				0x111061
#define NETCMD_PREVIEWONECHAN				0x03030d

#define NETCMD_GET_JPEGCFG					0x020004	/*��ȡͼƬ����	*/
#define NETCMD_SET_JPEGCFG					0x020005	/*����ͼƬ����*/
#define NETCMD_GET_JPEGPICTURE_EX			0x03000a	/*����jpegץͼ*/

#define NETCMD_GET_JPEGPICTURE				0x030009	/*ץJPEGͼ��*/


#define NETCMD_FINDFILE_EVENTSEARCH			0x090413	/*�¼������ӿ�*/
#define NETCMD_GET_RTSPPORT     			0x020c04

typedef enum HikResoloution_size_t {
    RESOLOUTION_SIZE_QQVGA   = 0x00, // 160*128
    RESOLOUTION_SIZE_QCIF,           // 176*144
    RESOLOUTION_SIZE_LTF,            // 240*192
    RESOLOUTION_SIZE_QVGA    = 0x06, // 320*240
    RESOLOUTION_SIZE_QNATIVE,        // 320*256
    RESOLOUTION_SIZE_CIF     = 0x01, // 352*288
    RESOLOUTION_SIZE_HD1,            // 352*576
    RESOLOUTION_SIZE_SVCD,           // 480*480
    RESOLOUTION_SIZE_HVGAW   = 0x56, // 640*360
    RESOLOUTION_SIZE_VGA     = 0x10, // 640*480
    RESOLOUTION_SIZE_NATIVE,         // 640*512
    RESOLOUTION_SIZE_BCIF,           // 704*288
    RESOLOUTION_SIZE_4CIF    = 0x03, // 704*576
    RESOLOUTION_SIZE_BVGA    = 0x04, // 720*480
    RESOLOUTION_SIZE_D1      = 0x05, // 720*576
    RESOLOUTION_SIZE_SVGA    = 0x12, // 800*600, 800*592
    RESOLOUTION_SIZE_960H,           // 960*576    
    RESOLOUTION_SIZE_XVGA,           // 1024*768
    RESOLOUTION_SIZE_WXGA,           // 1280*800
    RESOLOUTION_SIZE_720p    = 0x13, // 1280*720
    RESOLOUTION_SIZE_1_3M    = 0x14, // 1280*960
    RESOLOUTION_SIZE_1_2M,           // 1216*1024
    RESOLOUTION_SIZE_SXGA,           // 1280*1024
    RESOLOUTION_SIZE_1_5M,           // 1408*1024
    RESOLOUTION_SIZE_WSXGA,          // 1600*1024
    RESOLOUTION_SIZE_UXGA    = 0x11, // 1600*1200
    RESOLOUTION_SIZE_WUXGA,          // 1920*1200
    RESOLOUTION_SIZE_1080p   = 0x1b, // 1920*1080
    RESOLOUTION_SIZE_2_5M,           // 1872*1408
    RESOLOUTION_SIZE_3M,             // 2048*1536
    RESOLOUTION_SIZE_3M_1    = 0x1e, // 2048*1520
    RESOLOUTION_SIZE_3M_2    = 0x1f, // 2304*1296
    RESOLOUTION_SIZE_4M      = 0x46, // 2560*1440
    RESOLOUTION_SIZE_4M_1    = 0x47, // 2688*1520
    RESOLOUTION_SIZE_5M,             // 3744*1408
    RESOLOUTION_SIZE_5_0M,           // 2432*2048
    RESOLOUTION_SIZE_5M_1    = 0x43, // 2592*1440
    RESOLOUTION_SIZE_5M_2    = 0x43, // 2592*1944
    RESOLOUTION_SIZE_5M_3,           // 2560*1920
    RESOLOUTION_SIZE_8M,             // 3296*2472
    RESOLOUTION_SIZE_NR      = 0xFF  //< ö�ٵ�ͼ�δ�С�������Ŀ��
}HikResoloution_SIZE_T;


/* defines */
//#define MAX_LOGIN_USERS		128		/* maximum logined user numbers */
#define MAX_LOGIN_USERS		32		/* maximum logined user numbers */

#define BASE_USER_ID		0x10000
#define KEEP_ALIVE_INTERVAL	(5*60)		/* logined user must send keep alive message */
#define CHECK_USER_INTERVAL	5		/* check user timeout at interval 5 seconds */
#define	NO_USER				(-1)

#define	ACTUAL_MAX_NAME_LEN	32
#define LIMITED_NAME_LEN	16			/* Now we limit username length to 32 bytes */

/* user login return status */
#define INVALID_USER		0x1000
#define EXCEED_MAX_USER		0x1001
#define EMPTY_USER			0x1002
#define USER_NO_PERMISSION 	0x1003

#define BUFF_ALIGNMENT 	 0x10
/* typedefs */
typedef struct
{
	int 	userID;				/* user ID, DVR assign an exclusive ID when user login */
	int     cfgUserID;			/*��userID��Ӧ�����ò����е��û�ID*/
	UINT8	userName[NAME_LEN];		/* user name */
	UINT32	permission;			/* permission */
	UINT64  localPrevPermission;	
	UINT64	netPrevPermission;			/* Զ��Ԥ��ͨ�� */
	UINT64	localPlayPermission;			/* ���ػط�ͨ�� */
	UINT64	netPlayPermission;			/* Զ�̻ط�ͨ�� */
	UINT64	localRecordPermission;		/* ���ػط�ͨ�� */
	UINT64	netRecordPermission;		/* Զ�̻ط�ͨ�� */
	UINT64	localPtzCtrlPermission;		/* ���ػط�ͨ�� */
	UINT64	netPtzCtrlPermission;		/* Զ�̻ط�ͨ�� */
	UINT64	localBackupPermission;		/* ���ػط�ͨ�� */
	UINT32	priority;
	BOOL	bLocalUser;			/* local user or network user */
	time_t	loginTime;			/* timer of user login */
	time_t	expireTime;			/* client should send a 'keepalive' at intervals of 'KEEP_ALIVE_INTERVAL' seconds when idle */
	struct in_addr peerClientIp;		/* peer's IP address get from client program */
	struct in_addr peerSocketIp;		/* peer's Ip address get form socket accept() */
	UINT16  persistOpNums;			/* persistent operation(e.g. preview, download file) numbers of this user */
	char	peerMacAddr[MACADDR_LEN];	/* peer's MAC address */
	UINT32 sdkVersion;			/* SDK version of network user */
}HikUSER_INFO;

typedef struct{
	int hikhandle;
	HikUSER_INFO puserinfo[MAX_LOGIN_USERS];	
}HikUser;

typedef struct
{
	int connfd;
	struct sockaddr_in clientAddr;
}ThreadInfo;

int netClientLogin(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientReLogin(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientLogout(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientUserExchange(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetIpCoreCfgV31(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetIpAlarmInCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetIpAlarmInCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
void netClientStartAlarmUpChannel(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr, BOOL b9000Sdk);
int netClientGetWorkStatus(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetWorkStatusV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetDeviceCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetDeviceCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetDeviceCfgV40(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetDeviceCfgV40(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetAlarmInCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetAlarmInCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetAlarmOutCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetAlarmOutCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientPTZControl(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientPTZControlWithSpeed(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetDeviceCapacity(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetCompressCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetCompressCfgV30(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetNetCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetNetCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetPicCfgEx(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetPicCfgEx(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetRtspPort(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetVideoEffect(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetVideoEffect(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetColorEnhance(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetColorEnhance(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetAlarmIncfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetAlarmIncfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetNtpcfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetNtpcfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetDstcfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetDstcfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetSoftStore(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetReBoot(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetNetWorkCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetNetWorkCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientGetRtspCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetRtspCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientTcpPrev(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientUNKNOWN113305(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetDeviceDatetime(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientCmd11300b(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
void convertNetCmdHeaderByteOrder(NETCMD_HEADER *pHeader);
int netClientGetFtpCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netClientSetFtpCfg(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);
int netMakeIFrame(int connfd, char *recvbuff, struct sockaddr_in *pClientSockAddr);




#endif


