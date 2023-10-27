
#ifndef ITE_SIP_API_H
#define ITE_SIP_API_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <ite_typedef.h>

#include <osipparser2/osip_message.h>
#include <osipparser2/osip_parser.h>
#include <osipparser2/osip_port.h>
//#include <eXosip2/eXosip2.h>
#include <eXosip2/eXosip.h>
#include <eXosip2/eX_setup.h>
#include <eXosip2/eX_register.h>
#include <eXosip2/eX_options.h>
#include <eXosip2/eX_message.h>
#include <osip2/osip_mt.h>

//#include "ITENetSDK.h"
#include "itg_config_info.h"

#ifdef MODULE_SUPPORT_MOJING
#include "mojing_cloud_storage.h"
#endif

#define GB_MSG_ALARM_TYPE 0x10
typedef struct _ite_alarm_t{
    int  method; /*Ϊ����������ı���0����������1ΪAlarmMethod��Ƶ����, 2ΪAlarmMethod�豸����*/
    int  type;   /*�������ͣ�2Ϊ�ƶ���� 6Ϊ��������*/
    char time[32]; //ʱ���ʽ2020-09-30T14:00:00
    char para[128]; 
} ite_alarm_t;

typedef struct _gb28181_msg_queue_t{
    long mtype;
    ite_alarm_t mmsg;
}gb28181_msg_queue_t;

typedef struct _sessionId
{
    int cid;
    int did;

} sessionId;


typedef struct _ite_gb28181Obj
{
    struct eXosip_t *excontext ;
    IPNC_SipIpcConfig *sipconfig;
} ite_gb28181Obj;


typedef struct _ite_rtp_Info
{
    char localip[64];
    char remoteip[64];
    unsigned short videoPort;
    unsigned short audioPort;
    unsigned int enOverTcp;
    unsigned int mediaChannel;
    int veRtpSock;
    int veRtcpSock;
    int auRtpSock;
    int auRtcpSock;

    unsigned int playLoadType;
    int ssrc;
    char *frameBuffer;
    int frameBufferSize;
    int eventId;
    char rtpType;  //play:0,  download:1, playback:2, audio:3, cloudstorage 4,
    char isUsed;
    char isRun;

#ifdef MODULE_SUPPORT_MOJING
	CloudStorageControlT cloudCtrl;
#endif
    int selfCloudTestFd;
	unsigned int rtpResv[56];
}ite_rtp_Info;

struct _device_info/*�豸��Ϣ�ṹ��*/
{
    char *server_id;/*SIP������ID*//*Ĭ��ֵ��34020000002000000001*/
    char *server_ip;/*SIP������IP��ַ*//*Ĭ��ֵ��192.168.1.178*/
    char *server_port;/*SIP������IP�˿�*//*Ĭ��ֵ��5060*/

    char *ipc_id;/*ý����������ID*//*Ĭ��ֵ��34020000001180000002*/
    char *ipc_pwd;/*ý��������������*//*Ĭ��ֵ��12345678*/
    char *ipc_ip;/*ý����������IP��ַ*//*Ĭ��ֵ��192.168.1.144*/
    char *ipc_port;/*ý����������IP�˿�*//*Ĭ��ֵ��6000*/

    char *ipc_alarmid;/*����ͨ��*/
    char *ipc_voiceid;/*�������ͨ��*/

    char *device_name;/*�豸/����/ϵͳ����*//*Ĭ��ֵ��IPC*/
    char *device_manufacturer;/*�豸����*//*Ĭ��ֵ��iTarge*/
    char *device_model;/*�豸�ͺ�*//*Ĭ��ֵ��GB28181*/
    char *device_firmware;/*�豸�̼��汾*//*Ĭ��ֵ��V1.0*/
    char *device_encode;/*�Ƿ����*//*ȡֵ��Χ��ON/OFF*//*Ĭ��ֵ��ON*/
    char *device_record;/*�Ƿ�¼��*//*ȡֵ��Χ��ON/OFF*//*Ĭ��ֵ��OFF*/
    char *device_gbsn;/*�豸����*/
    char *device_mac;/*�豸Mac��ַ*/
    char *device_ctei;/*�豸Ctei*/
} device_info;
struct _device_status/*�豸״̬�ṹ��*/
{
    char *status_on;/*�豸��״̬*//*ȡֵ��Χ��ON/OFF*//*Ĭ��ֵ��ON*/
    char *status_ok;/*�Ƿ���������*//*ȡֵ��Χ��OK/ERROR*//*Ĭ��ֵ��OFF*/
    char *status_online;/*�Ƿ�����*//*ȡֵ��Χ��ONLINE/OFFLINE*//*Ĭ��ֵ��ONLINE*/
    char *status_guard;/*����״̬*//*ȡֵ��Χ��ONDUTY/OFFDUTY/ALARM*//*Ĭ��ֵ��OFFDUTY*/
    char *status_time;/*�豸���ں�ʱ��*//*��ʽ��xxxx-xx-xxTxx:xx:xx*//*Ĭ��ֵ��2012-12-20T12:12:20*/
} device_status;
enum _device_control
{
    EXOSIP_CTRL_RMT_LEFT = 1,/*����*/
    EXOSIP_CTRL_RMT_RIGHT,   /*����*/
    EXOSIP_CTRL_RMT_UP,      /*����*/
    EXOSIP_CTRL_RMT_DOWN,    /*����*/
    EXOSIP_CTRL_RMT_LARGE,   /*�Ŵ�*/
    EXOSIP_CTRL_RMT_SMALL,   /*��С*/
    EXOSIP_CTRL_RMT_STOP,    /*ֹͣң��*/
    EXOSIP_CTRL_REC_START,   /*��ʼ�ֶ�¼��*/
    EXOSIP_CTRL_REC_STOP,    /*ֹͣ�ֶ�¼��*/
    EXOSIP_CTRL_GUD_START,   /*����*/
    EXOSIP_CTRL_GUD_STOP,    /*����*/
    EXOSIP_CTRL_ALM_RESET,   /*������λ*/
    EXOSIP_CTRL_TEL_BOOT,    /*�豸Զ������*/
};

/*�ص�����*/
struct _ite_eXosip_callback
{
    /*��ȡ�豸��Ϣ*/
    /*device_info���豸��Ϣ�ṹ��ָ��*/
    /*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_getDeviceInfo)(struct _device_info *device_info);

    /*��ȡ�豸״̬*/
    /*device_info���豸״̬�ṹ��ָ��*/
    /*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_getDeviceStatus)(struct _device_status *device_status);

    /*��ȡ¼���ļ�����ʼʱ�������ʱ��*/
    /*ʱ���ʽ��xxxx-xx-xxTxx:xx:xx*/
    /*period_start��¼��ʱ�����ʼֵ*/
    /*period_end��¼��ʱ��ν���ֵ*/
    /*start_time����ǰ����¼���ļ�����ʼʱ��*/
    /*end_time����ǰ����¼���ļ��Ľ���ʱ��*/
    /*����ֵ���ɹ�ʱ���ط���ʱ���������ʣ��¼���ļ�������ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_getRecordTime)(char *period_start, char *period_end, char *start_time, char *end_time);

    /*�豸���ƣ��������ҡ����ϡ����¡��Ŵ���С��ֹͣң��/��ʼ�ֶ�¼��ֹͣ�ֶ�¼��/����������/������λ/�豸Զ������*/
    /*ctrl_cmd���豸�������_device_control���͵�ö�ٱ���*/
    /*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_deviceControl)(enum _device_control ctrl_cmd);

    /*ý����ƣ�ʵʱ�㲥/�ط�/����*/
    /*control_type��ý��������ͣ�ʵʱ�㲥/Play���ط�/Playback������/Download*/
    /*media_ip��ý�������IP��ַ*/
    /*media_port��ý�������IP�˿�*/
    /*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_mediaControl)(char *control_type, char *media_ip, char *media_port);

    /*���ſ��ƣ�����/���/����/��ͣ*/
    /*control_type�����ſ��ƣ�����/���/����/PLAY����ͣ/PAUSE*/
    /*play_speed�������ٶȣ�1Ϊ���ţ�����1Ϊ��ţ�С��1Ϊ����*/
    /*pause_time����ͣʱ�䣬��λΪ��*/
    /*range_start�����ŷ�Χ����ʼֵ*/
    /*range_end�����ŷ�Χ�Ľ���ֵ*/
    /*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
    int (*ite_eXosip_playControl)(char *control_type, char *play_speed, char *pause_time, char *range_start, char *range_end);
} ite_eXosip_callback;

/*��������*/
/*������ע��eXosip*/
void ite_eXosip_launch(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig);

int  ite_eXosip_init(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig);/*��ʼ��eXosip*/
/*ע��eXosip*/
/*expires������ע��ĳ�ʱʱ�䣬��λΪ��*/
/*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
int ite_eXosip_register(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig,int expires);

/*ע��eXosip*/
/*����ֵ���ɹ�ʱ����0��ʧ��ʱ���ظ�ֵ*/
int ite_eXosip_unregister(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig);

/*����MESSAGE������*/
/*ʱ���ʽ��xxxx-xx-xxTxx:xx:xx*/
/*alarm_time������ʱ��*/
void ite_eXosip_sendEventAlarm(struct eXosip_t *excontext,ite_alarm_t *alarm_time,IPNC_SipIpcConfig * SipConfig);

/*����MESSAGE���ļ�����*/
void ite_eXosip_sendFileEnd(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig,int session_id);

/* send invite message */
//int ite_eXosip_sendinvite(char * to,char * sdpMessage, char ** responseSdp,sessionId * id);//by jzw
int ite_eXosip_invit(struct eXosip_t *excontext,sessionId * id, char * to, char * sdpMessage, char * responseSdp,IPNC_SipIpcConfig * SipConfig);
/* send bye message */
int ite_eXosip_bye(struct eXosip_t *excontext,sessionId id);

void *ite_eXosip_register_Task(void *pData);

void * ite_eXosip_Kalive_Task(void *pData);
void *ite_eXosip_processEvent_Task(void *pData);/*��Ϣѭ������*/
void *ite_eXosip_NotifyAlarm_Task(void *pData);
void *ite_eXosip_PositionAlert_Task(void *pData);
void *ite_eXosip_ftp_upload_log_Task(void *pData);
//OS_S32 ite_gb28181_init_comm(OS_HANDLE *pt_data);
int ite_logUploadParam_init(void *pData);
int ite_eXosip_log_bplus_tree_init(void);
int ite_eXosip_log_bplus_tree_deinit(void);
#endif //ITE_SIP_API_H


