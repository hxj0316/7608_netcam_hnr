/*!
*****************************************************************************
** FileName     : phone_mtws.c
**
** Description  : api for mtws app.
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Create Date  : 2016-4-8
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/
#ifdef __cplusplus
    extern "C" {
#endif

#include <stdarg.h>
#include "cfg_all.h"
#include "netcam_api.h"
#include "common.h"
#include "media_fifo.h"
#include "mmc_api.h"
#include "flash_ctl.h"
#include "avi_rec.h"
#include "utility_api.h"
#include "phone_mtws.h"
#include "eventalarm.h"
#include "ipcsearch.h"
#include "gk_cms_utility.h"
#include "netcam_network.h"
#include "cfg_all.h"
#include "g711.h"
#include "gk_cms_protocol.h"
#include "xml.h"
#include "mtws_base64.h"
#include "md5.h"
#include "avi_utility.h"


static int is_sta = 0;
static int is_get_mtws_p2p_id = 0;
static char devId[256] = {0};

static void *mtws_alarm_handle = NULL;

#define MTWS_REC_AUDIO 0
#define MTWS_REC_TALK 0

static void * mtws_p2p_thread(void *para);
static void * mtws_stream1_thread(void *para);
static void * mtws_stream0_thread(void *para);
static void * mtws_local_listen_thread(void *para);

#if MTWS_REC_AUDIO
static FILE *fp = NULL;
#endif

#if MTWS_REC_TALK
static FILE *fp_talk = NULL;
#endif

#define MTWS_MAX_LINK_NUM 8

typedef struct
{
	int video_stream_type;			// �������ͣ�������������������
	int session_id;					// ����������ʱ������ID.
	int channel_id;					// ͨ��ID.
} P2P_CONNECT_INFO;

typedef struct _P2P_CLIENT_s
{
    NET_EVENT_CALLBACK m_evtCB;
    AUDIO_TALKBACK_CALLBACK m_talkCB;
    SYSTEMCONTROL_CALLBACK m_controlCB;
    int m_bHasClients;
    int m_bHasGetMainVol;
    int m_bHasGetSubVol;
    char m_mainVol[256];
    char m_subVol[256];
    pthread_mutex_t m_mainMutex;
    pthread_mutex_t m_subMutex;
    int m_bLoginSvr;

    int m_mainOpen;
    int m_subOpen;
    int m_ConnectNum;
    pthread_mutex_t m_ConnectMutex;
    P2P_CONNECT_INFO m_ConnectList[MTWS_MAX_LINK_NUM];
} P2P_CLIENT_T;


static P2P_CLIENT_T g_p2pClient;

static int mtws_thread_running = 0;

typedef struct{
	int	status;	//sd ״̬
	int size; //sd ����
	int used;
	int free; //sd ʣ��ռ�
	int format_process; // sd ��ʽ������
}sdStatus;

static int get_vol(char *dst, int stream_type)
{
    if (stream_type == 0) {
        pthread_mutex_lock(&g_p2pClient.m_mainMutex);
        strcpy(dst, g_p2pClient.m_mainVol);
        pthread_mutex_unlock(&g_p2pClient.m_mainMutex);
    } else if (stream_type == 1) {
        pthread_mutex_lock(&g_p2pClient.m_subMutex);
        strcpy(dst, g_p2pClient.m_subVol);
        pthread_mutex_unlock(&g_p2pClient.m_subMutex);
    }

    return 0;
}

static int set_vol(char *src, int stream_type)
{
    if (stream_type == 0) {
        pthread_mutex_lock(&g_p2pClient.m_mainMutex);
        strcpy(g_p2pClient.m_mainVol, src);
        pthread_mutex_unlock(&g_p2pClient.m_mainMutex);
    } else if (stream_type == 1) {
        pthread_mutex_lock(&g_p2pClient.m_subMutex);
        strcpy(g_p2pClient.m_subVol, src);
        pthread_mutex_unlock(&g_p2pClient.m_subMutex);
    }

    return 0;
}

static int mtws_register_video(int session_id, int channel_id, int video_stream_type)
{
	int i;
	int ret = -1;

	pthread_mutex_lock(&g_p2pClient.m_ConnectMutex);
	//�ж��Ƿ��Ѿ�����������
    for (i = 0; i < MTWS_MAX_LINK_NUM; i++) {
        if(g_p2pClient.m_ConnectList[i].session_id == 0) {
            g_p2pClient.m_ConnectList[i].session_id = session_id;
            g_p2pClient.m_ConnectList[i].channel_id = channel_id;
            g_p2pClient.m_ConnectList[i].video_stream_type = video_stream_type;
			ret = 0; //��ӳɹ�����0
			break;
        }
    }
    pthread_mutex_unlock(&g_p2pClient.m_ConnectMutex);

    return ret;
}

static int mtws_unregister_video(int session_id)
{
	int i;
	int ret = -1;

    pthread_mutex_lock(&g_p2pClient.m_ConnectMutex);
	//�ж��Ƿ��Ѿ�����������
    for (i = 0; i < MTWS_MAX_LINK_NUM; i++) {
        if(g_p2pClient.m_ConnectList[i].session_id == session_id) {
            g_p2pClient.m_ConnectList[i].session_id = 0;
            g_p2pClient.m_ConnectList[i].channel_id = 0;
            g_p2pClient.m_ConnectList[i].video_stream_type = 0;
			ret = 0; //��ӳɹ�����0
			break;
        }
    }
    pthread_mutex_unlock(&g_p2pClient.m_ConnectMutex);

    return ret;
}


static void mtws_p2p_log_func(const char* message)
{
	if (message)
	{
		printf("[p2p log] %s\n",message);
	}
}


int SystemGetTimeofRun(struct timeval *tv, struct timezone *tz)
{
//#if GET_SYSTEM_TIME_RELATIVE
#if 0
	struct timespec runtime;

	if(0 > clock_gettime(CLOCK_MONOTONIC, &runtime))
	{
		__ERR("clock gettime err: %s\n", strerror(errno));
		return -1;
	}

	tv->tv_sec 	= runtime.tv_sec;
	tv->tv_usec = runtime.tv_nsec/1000;
#else
	gettimeofday(tv, tz);
#endif
	return 0;
}


#define DEBUG_MAX_INFO_LEN 1024

void DebugPrint(int level, int way, const char* format, ...)
{
    //static char function_buf[DEBUG_MAX_INFO_LEN]=""; //buf to remember funtion name

    char content_buf[1024];

    va_list ap;
#if 0
    char debug_buf[1024];
    if(level > debug_level)
    return 0;

    if((way>0) && ((way & debug_way) == 0))
    return 0;
#endif
    va_start(ap, format);
    vsnprintf(content_buf, 1024,  format, ap);
    va_end(ap);

    printf(content_buf);
}


static int mtws_user_auth(REQ_USER_AUTH_T *pAuthMsg)
{
    if (pAuthMsg == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("login, session id:%d user:%s passwd:%s\n", pAuthMsg->sessionId, pAuthMsg->username, pAuthMsg->password);
    return 0;
}

static int mtws_open_stream(REQ_CHANNEL_OPEN_T *pChannelOpenMsg)
{
    if (pChannelOpenMsg == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("open stream, sessionId:%d  channelId:%d streamtype=%d\n",
            pChannelOpenMsg->sessionId, pChannelOpenMsg->channelId, pChannelOpenMsg->videoStreamType);


    /* û��SPS PPS������ȴ� */
    if (0 == pChannelOpenMsg->videoStreamType) {
        while (g_p2pClient.m_bHasGetMainVol == 0) {
            sleep(1);
        }
    } else {
        while (g_p2pClient.m_bHasGetSubVol == 0) {
            sleep(1);
        }
    }

    if (0 == pChannelOpenMsg->videoStreamType) {
		VIDEO_STREAM_PARAM mainVideoParam;
		memset(&mainVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
		mainVideoParam.bitrate = 2000;
		strcpy(mainVideoParam.codec, "H264");
		mainVideoParam.width = 1280;
		mainVideoParam.height = 720;
		mainVideoParam.framerate= 25;
        get_vol(mainVideoParam.config, 0);
		CloudSetVideoParam(0, STREAM_VIDEO_MAIN, mainVideoParam);
        //g_p2pClient.m_mainOpen = 1;
        //mtws_register_video
    } else {
		VIDEO_STREAM_PARAM subVideoParam;
		memset(&subVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
		subVideoParam.bitrate = 1000;
		strcpy(subVideoParam.codec, "H264");
		subVideoParam.width = 640;
		subVideoParam.height = 360;
		subVideoParam.framerate= 25;
        get_vol(subVideoParam.config, 1);
        CloudSetVideoParam(0, STREAM_VIDEO_SUB, subVideoParam);
        //g_p2pClient.m_subOpen = 1;
        //mtws_register_video
    }

	AUDIO_STREAM_PARAM  audioParam;
	memset(&audioParam, 0, sizeof(AUDIO_STREAM_PARAM));
	audioParam.enable = 1;
	strcpy(audioParam.codec, "PCMU");
	audioParam.samplerate = 8000;
	audioParam.samplewidth = 16;
	audioParam.channels = 1;
	audioParam.bitrate = 64;
    CloudSetAudioParam(0, audioParam);

    //init_video_stream_param

    //���ó�1������Է�����
    g_p2pClient.m_bHasClients = 1;
	return 0;
}

static int mtws_close_stream(REQ_CHANNEL_CLOSE_T *pChannelCloseMsg)
{
    if (pChannelCloseMsg == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("close stream, sessionId:%d  channelId:%d\n", pChannelCloseMsg->sessionId, pChannelCloseMsg->channelId);

    //g_p2pClient.m_mainOpen = 0;
    //g_p2pClient.m_subOpen = 0;

    //���ó�0���򲻷�����
    g_p2pClient.m_bHasClients = 0;
    return 0;
}

static int mtws_ptz_control(REQ_PTZ_ACTION_T* ptz_control)
{
    if (ptz_control == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("PTZ, sessionid = %d xml = [%s]\n", ptz_control->sessionId, ptz_control->xml);

	/*	�������XML�ı�ʾ��

	<xml>
	<cmd>left</cmd>
	<panspeed>5</panspeed>
	<tiltspeed>5</tiltspeed>
	<duration>3000</duration>
	</xml>

    <xml>
    <cmd>up</cmd>
    <duration>3000</duration>
    <panspeed>5</panspeed>
    <tiltspeed>5</tiltspeed>
    </xml>

    <xml>
        <cmd>stop</cmd>
    </xml>

	*/

    /*
	PTZ��������
	<xml>
	<cmd>SetOrbit</cmd>
	<preset>presetnum</preset>
	<r>
	<b>bhour:bmin</b>
	<e>ehour:emin</e>
	</r>
	</xml>
    */
    int ret = 0;
    int ptz_speed = 0;
    int ptz_step  = 0;
    int ptz_preset_id = 0;

    int ptz_curis_grp_id = 0;
    int grp_idx = 0;
    int pot_idx = 0;
    int cnt;
    GK_NET_PRESET_INFO   gkPresetCfg;
    DMS_NET_CRUISE_POINT cruisePoint;

    GK_NET_CRUISE_CFG    gkCruiseCfg;
    GK_NET_CRUISE_GROUP  cruise_info;


	char operation_name[32] = {0};
	char duration[32] = {0};
	char pan_speed[32] = {0};
	char titl_speed[32] = {0};
	char preset_id[32] = {0};

	char * cmd_str_begin = strstr(ptz_control->xml, "<cmd>");
	char * cmd_str_end   = strstr(ptz_control->xml, "</cmd>");
	int len = cmd_str_end - cmd_str_begin - 5;
	if (len > 0) {
		strncpy(operation_name, cmd_str_begin+5, len);
	}
	if (strlen(operation_name) <=0) {
        PRINT_ERR("xml error.");
		return -1;
	}

	cmd_str_begin = strstr(ptz_control->xml, "<panspeed>");
	cmd_str_end   = strstr(ptz_control->xml, "</panspeed>");
	len = cmd_str_end - cmd_str_begin - 10;
	if (len > 0) {
		strncpy(pan_speed, cmd_str_begin+10, len);
	}

	cmd_str_begin = strstr(ptz_control->xml, "<tiltspeed>");
	cmd_str_end   = strstr(ptz_control->xml, "</tiltspeed>");
	len = cmd_str_end - cmd_str_begin - 11;
	if (len > 0) {
		strncpy(titl_speed, cmd_str_begin+11, len);
	}

	cmd_str_begin = strstr(ptz_control->xml, "<duration>");
	cmd_str_end   = strstr(ptz_control->xml, "</duration>");
	len = cmd_str_end - cmd_str_begin - 10;
	if (len > 0) {
		strncpy(duration, cmd_str_begin+10, len);
	}

	cmd_str_begin = strstr(ptz_control->xml, "<preset>");
	cmd_str_end   = strstr(ptz_control->xml, "</preset>");
	len = cmd_str_end - cmd_str_begin - 8;
	if (len > 0) {
		strncpy(preset_id, cmd_str_begin+8, len);
	}




	PRINT_INFO("--------------------------  cmd = [%s] panspeed = [%s] titlspped [%s] duration [%s] preset_id [%s]\n",
		operation_name, pan_speed, titl_speed, duration,preset_id);

    //�򴮿�1��������
    //PRINT_INFO("to send xml = [%s]\n", ptz_control->xml);

    /*
    ����������ҪѲ�����ܡ�
    ��Ҫ���Ԥ�õ㵽Ѳ��·��(Ѳ����)��
    ɾ��Ѳ��·��(Ѳ����)��Ѳ����(ԭ�������Ԥ�õ�)��
    ����Ѳ��·��(Ѳ����) ��������
    ����Ҫ����xml�ű�����
    Ѳ����:group_id
    Ԥ�õ�:preset_id

    */
    ptz_preset_id = atoi(preset_id);
    printf("FFFFF  ptz cmd %s ,preset_id:%d\n",operation_name,ptz_preset_id);
    ptz_step = 20;
    ptz_speed = 32;
    if(!strcmp(operation_name,"stop"))
    {
        netcam_ptz_stop();
        printf("stop ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"up"))
    {
        netcam_ptz_up(ptz_step, ptz_speed);
        printf(" up ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"down"))
    {
        netcam_ptz_down(ptz_step, ptz_speed);
        printf(" down ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"left"))
    {
        netcam_ptz_left(ptz_step, ptz_speed);
        printf(" left ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"left_up"))
    {
        netcam_ptz_left_up(ptz_step, ptz_speed);
        printf(" left_up ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"left_down"))
    {
        netcam_ptz_left_down(ptz_step, ptz_speed);
        printf(" left_down ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"right"))
    {
        netcam_ptz_right(ptz_step, ptz_speed);
        printf(" right ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"right_up"))
    {
        netcam_ptz_right_up(ptz_step, ptz_speed);
        printf(" right_up ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"right_down"))
    {
        netcam_ptz_right_down(ptz_step, ptz_speed);
        printf(" right_down ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"auto"))/*�Զ�Ѳ��*/
    {
        netcam_ptz_hor_ver_cruise(ptz_speed);
        printf("AUTO  ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"setpreset"))/*����Ԥ�õ�*/
    {
            /*id�ţ���0��ʼ����*/
            if (ptz_preset_id > PTZ_MAX_PRESET)
                ptz_preset_id = PTZ_MAX_PRESET - 1;
            else if (ptz_preset_id > 0)
                ptz_preset_id -= 1;
            else
                ptz_preset_id = 0;

            get_param(PTZ_PRESET_PARAM_ID, &gkPresetCfg);
            gkPresetCfg.nPresetNum++;
            if (gkPresetCfg.nPresetNum >= NETCAM_PTZ_MAX_PRESET_NUM)
            {
                gkPresetCfg.nPresetNum = NETCAM_PTZ_MAX_PRESET_NUM;
            }

            /*no��ţ���0��ʼ����*/
            gkPresetCfg.no[gkPresetCfg.nPresetNum-1] = ptz_preset_id;
            set_param(PTZ_PRESET_PARAM_ID, &gkPresetCfg);
            PresetCruiseCfgSave();

            if ((ret = netcam_ptz_set_preset(ptz_preset_id, NULL)))
            {
                PRINT_ERR("call  set PRESET error!\n");
                return -1;
            }
        printf("setpreset  ptz cmd!!\n");
    }
    else if(!strcmp(operation_name,"clearpreset"))/*ɾ��Ԥ�õ�*/
    {
        /*id�ţ���0��ʼ����*/
        if (ptz_preset_id > PTZ_MAX_PRESET)
            ptz_preset_id = PTZ_MAX_PRESET - 1;
        else if (ptz_preset_id > 0)
            ptz_preset_id -= 1;
        else
            ptz_preset_id = 0;

        get_param(PTZ_PRESET_PARAM_ID, &gkPresetCfg);
        gkPresetCfg.nPresetNum--;
        if (gkPresetCfg.nPresetNum <= 0)
        {
            gkPresetCfg.nPresetNum = 0;
        }
        set_param(PTZ_PRESET_PARAM_ID, &gkPresetCfg);
        PresetCruiseCfgSave();
        if ((ret = netcam_ptz_clr_preset(ptz_preset_id)))
        {
            PRINT_ERR("call  DMS_PTZ_CMD_PRESET_CLS error!\n");
            return -1;
        }

        printf("clearpreset  ptz cmd!!\n");
    }

    else if(!strcmp(operation_name,"callpreset"))/*����Ԥ�õ�*/
    {
        /*id�ţ���0��ʼ����*/
        if (ptz_preset_id > PTZ_MAX_PRESET)
            ptz_preset_id = PTZ_MAX_PRESET - 1;
        else if (ptz_preset_id > 0)
            ptz_preset_id -= 1;
        else
            ptz_preset_id = 0;

        cruise_info.byPointNum    = 1;
        cruise_info.byCruiseIndex = 0;
        cruise_info.struCruisePoint[0].byPointIndex = 0;
        cruise_info.struCruisePoint[0].byPresetNo   = ptz_preset_id;
        cruise_info.struCruisePoint[0].byRemainTime = 0;
        cruise_info.struCruisePoint[0].bySpeed      = -1;
        if ((ret = netcam_ptz_stop()))
        {
            PRINT_ERR("call  netcam_ptz_stop error!\n");
            return -1;
        }
        if ((ret = netcam_ptz_preset_cruise(&cruise_info)))
        {
            PRINT_ERR("call  DMS_PTZ_CMD_AUTO_STRAT error!\n");
            return -1;
        }
        printf("XXXcallpreset  ptz cmd!!\n");

    }

    #if 1
    else if(!strcmp(operation_name,"addcurise"))/*Ԥ�õ����Ѳ����*/
    {

        grp_idx = ptz_curis_grp_id;
        memset(&cruisePoint, 0, sizeof(DMS_NET_CRUISE_POINT));
        if (grp_idx >= PTZ_MAX_CRUISE_GROUP_NUM)
        {
            PRINT_ERR("index:%d > PTZ_MAX_CRUISE_GROUP_NUM!\n", grp_idx);
            return -1;
        }

        get_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);

#if 0
        PRINT_INFO("ADD index:%d   byPointNum:%d  byPointIndex:%d  byPresetNo:%d  byRemainTime:%d  bySpeed:%d \n",
                                                        grp_idx,
                                                        gkCruiseCfg.struCruise[grp_idx].byPointNum,
                                                        cruisePoint.byPointIndex,
                                                        cruisePoint.byPresetNo,
                                                        cruisePoint.byRemainTime,
                                                        cruisePoint.bySpeed);
#endif
        /*û����һ��Ѳ�ӵ㣬��Ӧ��ļ�����1��byPointNum++*/
        if(gkCruiseCfg.struCruise[grp_idx].byPointNum < PTZ_MAX_CRUISE_POINT_NUM)
        {
            gkCruiseCfg.struCruise[grp_idx].byPointNum++;
        }
        else
        {
            gkCruiseCfg.struCruise[grp_idx].byPointNum = PTZ_MAX_CRUISE_POINT_NUM;
            PRINT_INFO("total preset point num:PTZ_MAX_CRUISE_POINT_NUM ,don't add preset point!\n");

        }
#if 0
        /* Ѳ�����е��±�,���ֵ����PTZ_MAX_CRUISE_POINT_NUM ��ʾ��ӵ�ĩβ */
        if (cruisePoint.byPointIndex >= gkCruiseCfg.struCruise[grp_idx].byPointNum)
        {
            cruisePoint.byPointIndex = gkCruiseCfg.struCruise[grp_idx].byPointNum - 1;
        }
#endif
        /*************
        ��Ҫ�޸ĵĵط�

        **************/
        cruisePoint.byPointIndex = gkCruiseCfg.struCruise[grp_idx].byPointNum - 1;
        cruisePoint.byPresetNo = ptz_preset_id;
        cruisePoint.byRemainTime = 1;
        cruisePoint.bySpeed = 4;

        pot_idx = cruisePoint.byPointIndex;

        /*ΪʲôҪ������gkCruiseCfg����һ��Ѳ���������ú�??*/
        for (cnt = gkCruiseCfg.struCruise[grp_idx].byPointNum -1; cnt > pot_idx; cnt--)
        {
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byPointIndex =
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPointIndex;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byPresetNo   =
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPresetNo;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byRemainTime =
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byRemainTime;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].bySpeed      =
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].bySpeed;
        }

        /*����Ѳ��������*/
        gkCruiseCfg.nChannel =0;
        gkCruiseCfg.struCruise[grp_idx].byCruiseIndex = grp_idx;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byPointIndex =
                                                                                cruisePoint.byPointIndex;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byPresetNo   =
                                                                                cruisePoint.byPresetNo;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byRemainTime =
                                                                                cruisePoint.byRemainTime;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].bySpeed      =
                                                                                cruisePoint.bySpeed;

        set_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);
        PresetCruiseCfgSave();
    }
    else if(!strcmp(operation_name,"deletecurise"))
    {
        grp_idx = ptz_curis_grp_id;
        /*
        ��Ҫ���õĵط�
        */
        memset(&cruisePoint, 0, sizeof(DMS_NET_CRUISE_POINT));
        cruisePoint.byPointIndex = 0;



        if (grp_idx >= PTZ_MAX_CRUISE_GROUP_NUM)
        {
            PRINT_ERR("index:%d > PTZ_MAX_CRUISE_GROUP_NUM!\n", grp_idx);
            return -1;
        }


        /*ͨ��pointindexɾ��Ѳ����??*/
        if (cruisePoint.byPointIndex > PTZ_MAX_CRUISE_POINT_NUM)
        {
            PRINT_ERR("cruisePoint.byPointIndex:%d > PTZ_MAX_CRUISE_GROUP_NUM!\n", cruisePoint.byPointIndex);
            cruisePoint.byPointIndex = PTZ_MAX_CRUISE_POINT_NUM - 1;
        }
        else if (cruisePoint.byPointIndex < 0)
        {
            PRINT_ERR("cruisePoint.byPointIndex:%d < 0\n", cruisePoint.byPointIndex);
            cruisePoint.byPointIndex = 0;
        }

        get_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);
#if 0
        PRINT_INFO("DELETE  index:%d   byPointNum:%d  byPointIndex:%d  byPresetNo:%d  byRemainTime:%d  bySpeed:%d \n",
                                                        grp_idx,
                                                        gkCruiseCfg.struCruise[grp_idx].byPointNum,
                                                        cruisePoint.byPointIndex,
                                                        cruisePoint.byPresetNo,
                                                        cruisePoint.byRemainTime,
                                                        cruisePoint.bySpeed);
#endif
        gkCruiseCfg.nChannel = 0;
        /*�����Ƕ���ģ�������*/
        if (cruisePoint.byPointIndex > gkCruiseCfg.struCruise[grp_idx].byPointNum)
        {
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byPointIndex = 0;
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byPresetNo   = 0;
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byRemainTime = 0;
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].bySpeed      = 0;
        }

        gkCruiseCfg.struCruise[grp_idx].byPointNum--;
        pot_idx = cruisePoint.byPointIndex;
        gkCruiseCfg.struCruise[grp_idx].byCruiseIndex = grp_idx;


        /*�����Ƕ���ģ�������*/
        for (cnt = pot_idx; cnt < gkCruiseCfg.struCruise[grp_idx].byPointNum; cnt++)
        {
            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPointIndex =
                                        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byPointIndex;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPresetNo   =
                                        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byPresetNo;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byRemainTime =
                                        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].byRemainTime;

            gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].bySpeed      =
                                        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt+1].bySpeed;
        }

        pot_idx = gkCruiseCfg.struCruise[grp_idx].byPointNum;
        //printf("----------> pot_idx:%d \n", pot_idx);
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byPointIndex = 0;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byPresetNo   = 0;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].byRemainTime = 0;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[pot_idx].bySpeed      = 0;

        set_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);
        PresetCruiseCfgSave();

    }
    else if(!strcmp(operation_name,"modifycurise"))/*�޸�Ѳ��·����ĳһ��Ѳ����*/
    {
        grp_idx = ptz_curis_grp_id;

        /*
        ��Ҫ���õĵط�
        */
        memset(&cruisePoint, 0, sizeof(DMS_NET_CRUISE_POINT));
        cruisePoint.byPointIndex = 0;

        //memcpy(&cruisePoint, ptz_control.byRes, sizeof(DMS_NET_CRUISE_POINT));
        if (grp_idx >= PTZ_MAX_CRUISE_GROUP_NUM)
        {
            PRINT_ERR("index:%d > PTZ_MAX_CRUISE_GROUP_NUM!\n", grp_idx);
            return -1;
        }
        if (cruisePoint.byPointIndex > PTZ_MAX_CRUISE_POINT_NUM)
        {
            PRINT_ERR("cruisePoint.byPointIndex:%d > PTZ_MAX_CRUISE_GROUP_NUM!\n", cruisePoint.byPointIndex);
            return -1;
        }

        get_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);

        PRINT_INFO("MOD  index:%d   byPointNum:%d  byPointIndex:%d  byPresetNo:%d  byRemainTime:%d  bySpeed:%d \n",
                                                            grp_idx,
                                                            gkCruiseCfg.struCruise[grp_idx].byPointNum,
                                                            cruisePoint.byPointIndex,
                                                            cruisePoint.byPresetNo,
                                                            cruisePoint.byRemainTime,
                                                            cruisePoint.bySpeed);

        gkCruiseCfg.nChannel =0;
        gkCruiseCfg.struCruise[grp_idx].byCruiseIndex = grp_idx;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byPointIndex =
                                                                                cruisePoint.byPointIndex;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byPresetNo   =
                                                                                cruisePoint.byPresetNo;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].byRemainTime =
                                                                                cruisePoint.byRemainTime;
        gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cruisePoint.byPointIndex].bySpeed      =
                                                                                cruisePoint.bySpeed;
        set_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);
        PresetCruiseCfgSave();
    }
    else if(!strcmp(operation_name,"startcurise"))/*��ʼĳһѲ��·����Ѳ��*/
    {
        grp_idx = ptz_curis_grp_id;
        if (grp_idx >= PTZ_MAX_CRUISE_GROUP_NUM)
        {
            PRINT_ERR("START_CRU grp_idx:%d > PTZ_MAX_CRUISE_GROUP_NUM:%d\n", grp_idx, PTZ_MAX_CRUISE_GROUP_NUM);
            return -1;
        }
        get_param(PTZ_CRUISE_PARAM_ID, &gkCruiseCfg);
        for (cnt = 0; cnt < PTZ_MAX_CRUISE_POINT_NUM; cnt++)
        {
            if (gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPresetNo > 0)
            {
                gkCruiseCfg.struCruise[grp_idx].struCruisePoint[cnt].byPresetNo--;
            }
        }
        ret = netcam_ptz_preset_cruise(&gkCruiseCfg.struCruise[grp_idx]);
        if (ret < 0)
        {
            PRINT_ERR("Call netcam_ptz_preset_cruise error!\n");
            return -1;
        }


    }
#endif
    else
    {
        printf("error ptz cmd!!\n");

    }



#if 0
    /* ��ʼ������1 */
    if (serial1_fd < 0) {
        serial1_fd = SerialInit(1); //������1·����
        if (serial1_fd < 0) {
            PRINT_ERR("SerialInit error.");
            return -1;
        }
    }

    /* �������� */
    if (serial1_fd > 0)
        SerialSend(serial1_fd, (unsigned char *)ptz_control->xml, (int)(strlen(ptz_control->xml) + 1));
#endif

    return 0;
}

/**********************************************************************
* �������ܣ� ����ϵͳʱ��
* ����1:     (in)ʱ��(ʱ��ΪCSTʱ��)
* ����ֵ:    ��
**********************************************************************/
int mtws_adjust_time(char *str_time)
{
    if (strlen(str_time) <= 0) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("str_time:%s\n", str_time);

	// ����ϵͳʱ��
    char command[128] = {0};
    sprintf(command, "date -s %s", str_time);
    system(command);
    system("hwclock -w");

    time_t timep;
    time(&timep);
    struct tm tt = {0};
    struct tm *p = localtime_r(&timep, &tt);

    time_t timep2;
    time(&timep2);
    struct tm tt2 = {0};
    struct tm *p2 = gmtime_r(&timep2, &tt2);


	PRINT_INFO("localtime: %04d%02d%02d%02d%02d%02d\n", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday,
                                                   p->tm_hour, p->tm_min, p->tm_sec);

	PRINT_INFO("gmtime: %04d%02d%02d%02d%02d%02d\n", p2->tm_year + 1900, p2->tm_mon + 1, p2->tm_mday,
                                                   p2->tm_hour, p2->tm_min, p2->tm_sec);

    return 0;
}

int mtws_send_md_alarm()
{
    time_t timep;
    time(&timep);
    struct tm tt = {0};
    struct tm *p = localtime_r(&timep, &tt);

    char str[100] = {0};
    sprintf(str, "motion detect: %04d.%02d.%02d-%02d:%02d:%02d",
                 p->tm_year + 1900, p->tm_mon + 1,
                 p->tm_mday, p->tm_hour,
                 p->tm_min, p->tm_sec);

    PRINT_INFO("alarm time: %s\n", str);
    CloudSendAlarm(0, 8202, 0, str);
    return 0;
}

int mtws_alarm_cb(int nChannel, int nAlarmType, int nAction, void* pParam)
{
	if (nAction == 0) {
		//PRINT_INFO("\n");
		return 0;
	}

    char eventType[20] = {0};
	char strAction[20] = {0};

    switch (nAlarmType)
    {
        case GK_ALARM_TYPE_ALARMIN:     //0:�ź���������ʼ
        	sprintf(eventType ,"%s", "LocalIO");
			sprintf(strAction ,"%s", "Start");
			break;

        case GK_ALARM_TYPE_DISK_FULL:       //1:Ӳ����
			sprintf(eventType ,"%s", "StorrageLowSpace");
			break;

        case GK_ALARM_TYPE_VLOST:            //2:�źŶ�ʧ
        	sprintf(eventType ,"%s", "VideoLoss");
			sprintf(strAction ,"%s", "Start");
			break;

        case GK_ALARM_TYPE_VMOTION:          //3:�ƶ����
			sprintf(eventType ,"%s", "VideoMotion");
			sprintf(strAction ,"%s", "Start");
            mtws_send_md_alarm();
            break;
        case GK_ALARM_TYPE_DISK_UNFORMAT:    //4:Ӳ��δ��ʽ��
			sprintf(eventType ,"%s", "StorageNotExist");
			break;

        case GK_ALARM_TYPE_DISK_RWERR:       //5:��дӲ�̳���,
			sprintf(eventType ,"%s", "StorageFailure");
			break;

        case GK_ALARM_TYPE_VSHELTER:         //6:�ڵ�����
			sprintf(eventType ,"%s", "VideoBlind");
			break;

        case GK_ALARM_TYPE_ALARMIN_RESUME:         //9:�ź��������ָ�
			sprintf(eventType ,"%s", "LocalIO");
			sprintf(strAction ,"%s", "Stop");
			break;

        case GK_ALARM_TYPE_VLOST_RESUME:         //10:��Ƶ��ʧ�����ָ�
			sprintf(eventType ,"%s", "VideoLoss");
			sprintf(strAction ,"%s", "Stop");
			break;

        case GK_ALARM_TYPE_VMOTION_RESUME:         //11:��Ƶ�ƶ���ⱨ���ָ�
			sprintf(eventType ,"%s", "VideoMotion");
			sprintf(strAction ,"%s", "Stop");
			break;

        case GK_ALARM_TYPE_NET_BROKEN:       //12:����Ͽ�
			sprintf(eventType ,"%s", "NetAbort");
			break;

        case GK_ALARM_TYPE_IP_CONFLICT:      //13:IP��ͻ
			sprintf(eventType ,"%s", "IPConfict");
			break;

        default:
			PRINT_INFO("Alarm: Type = %d", nAlarmType);
        	break;
    }

    //PRINT_INFO("alarm_type:%s action:%s\n", eventType, strAction);

    return 0;
}

static int mtws_start_sd_rec()
{
    PRINT_INFO("start sd rec.\n");
    // ����������¼��
    PRINT_INFO("open stream0. one pack len = %d minutes.", runRecordCfg.recordLen);
    manu_rec_start_alltime(0, runRecordCfg.recordLen);

    return 0;
}

static int mtws_stop_sd_rec()
{
    PRINT_INFO("stop sd rec.\n");
    // �ر�������¼��
    PRINT_INFO("close stream0. one pack len = %d minutes.", runRecordCfg.recordLen);
    manu_rec_stop(0);

    return 0;
}

static int mtws_enable_md()
{
    PRINT_INFO("enable md.\n");
    runMdCfg.enable = 1;
    return 0;
}

static int mtws_disable_md()
{
    PRINT_INFO("disable md.\n");
    runMdCfg.enable = 0;
    return 0;
}

static int mtws_config_set(REQ_CONFIG_SET_T* config_set)
{

    //RSP_CONFIG_GET_T rsp_config_get;

    if (config_set == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("config_set, sessionid = %d configId = %d xml = [%s]\n",
               config_set->sessionId, config_set->configId, config_set->xml);

    int ret = -1;
    switch (config_set->configId)
    {

        case 820:
        {
            printf("mtws_config_set set \n");
            break;
        }

        case 10003:
        {

            printf("mtws_adjust_time set \n");
            ret = mtws_adjust_time(config_set->xml + 1); //ȥ����ͷ�� \n
            break;
        }
        case 10004:
        {

            printf("mtws_start_sd_rec set \n");
            ret = mtws_start_sd_rec();
            break;
        }
        case 10005:
        {

            printf("mtws_stop_sd_rec set \n");
            ret = mtws_stop_sd_rec();
            break;
        }
        case 10007:
        {

            printf("mtws_enable_md set \n");
            ret = mtws_enable_md();
            break;
        }
        case 10008:
        {

            printf("mtws_disable_md set \n");
            ret = mtws_disable_md();
            break;
        }

        default:
        {
            PRINT_ERR("unknown cmd:%d !\n", config_set->configId);
            break;
        }
    }

    return ret;
}



static const char *config_xml_write_ws_cb(XMLN *node, int where)
{
    XMLN *parent = NULL;
    const char *name = NULL;
    int level;
    static const char *tabs = "\t\t\t\t\t\t\t\t";

    name = node->name;

    if (!strncmp(name, "?xml", 4))
    {
        if (where == XML_WS_AFTER_OPEN)
            return ("\n");
        else
            return (NULL);
    }
    else if (where == XML_WS_BEFORE_OPEN)
    {
        for (level = 0, parent = node->parent;parent;level++, parent = parent->parent);
        if ( level > 8)
            level = 8;
        return (tabs + 8 - level);
    }
    else if(where == XML_WS_BEFORE_CLOSE && (node->f_child != NULL && node->f_child->type == NTYPE_TAG))
    {
        for (level = 0, parent = node->parent;parent;level++, parent = parent->parent);
        if ( level > 8)
            level = 8;
        return (tabs + 8 - level);
    }
    else if (where == XML_WS_AFTER_CLOSE)
    {
        return ("\n");
    }
    else if (where == XML_WS_AFTER_OPEN && (node->f_child != NULL && node->f_child->type == NTYPE_TAG))
    {
        return ("\n");
    }

    return (NULL);
}

/********************
ִ��shell�������ִ�н������retErr��ִ��������ݷ���buff
********************/
static int mtws_getStringBySystem(const char *cmd, char *buff, int bufsize,int *retErr)
{
    FILE *fp = NULL;
    int ret=0;
    memset(buff, 0, sizeof(bufsize));
    if(NULL == buff)
    {
        printf("cmd buf is null\n");
        return -1;
    }

    fp = popen(cmd, "r");
    if(fp != NULL)
    {
        *retErr = 0;

    }
    else
    {
        *retErr = errno;
    }

    if (NULL == fp)
    {
        perror("error popen");
        return -1;
    }
    fread(buff,1,bufsize,fp);

    //printf("%d cmd exe display :: %s \n", __LINE__, buff);
    pclose(fp);
}

static int  mtws_config_get(REQ_CONFIG_GET_T* config_get)
{
    if (config_get == NULL) {
        PRINT_ERR("param error.");
        return -1;
    }

    PRINT_INFO("config_get, configid = %d session id= %d \n",
                config_get->configId, config_get->sessionId);

    RSP_CONFIG_GET_T rsp_config_get;
    memset(&rsp_config_get, 0, sizeof(rsp_config_get));
    rsp_config_get.sessionId = config_get->sessionId;
    rsp_config_get.configId = config_get->configId;
    char str[10000] = {0};
    switch (config_get->configId)
    {
        case 10006:
        {
            sprintf(str, "<md_enable>%d<\/md_enable>", runMdCfg.enable);
            rsp_config_get.xmllen = strlen(str) + 1;
            rsp_config_get.pxml = str;
            break;
        }

        case 800:/*��ȡ������Ϣ*/
        {
            printf("mtws_config_get get \n");
            break;
        }

        case 202:/*ʱ������*/
        {
            printf("mtws_config_get get timezone\n");
            break;
        }


        case 203:/*��ȡ�û�������*/
        {
            printf("mtws_config_get get user name\n");
            break;
        }

         case 204:/*��ȡtf��״̬*/
         {
             printf("mtws_config_get sd \n");
             break;
         }


        default:
        {
            PRINT_ERR("unknown cmd:%d !\n", config_get->configId);
            break;
        }

    }



    CloudSendConfig(config_get->sessionId, rsp_config_get);
    return 0;
}
int mtws_control_alarm(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;
    cJSON *alarmJson = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson)
    {
        printf("Unable to read rootJson\n");
        return -1;
    }
    /********************************
    1,����alarm
    ********************************/
    /*
    "alarm": {
        "isset": 0,
        "result": 0,
        "errMsg": "",
          // <!-- �ƶ���� -->
          // <!-- enable(ȡֵ0,1),1��ʾ����,=0��ʾ�ر� -->
          // <!-- level��ʾ�����ȵȼ�(ȡֵ1-5) Խ��Խ����-->
          // <!-- trigger_speaker��������ʱ��Ӧ�����б����������(ȡֵ0,1),0��ʾ������,1��ʾ���� -->
        "motion": {
          "enable": 1,
          "level": 1,
          "triggerSpeaker": 0
        }
      },
    */
    alarmJson = cJSON_GetObjectItem(rootJson,"alarm");
    if(alarmJson != NULL)
    {
        int sensi;
        isset = cJSON_GetObjectItem(alarmJson,"isset")->valueint;/*'0'=get,'1'=set*/

        printf("dddd alarm    isset:[%d]\n",isset);
        if(0==isset)//get
        {

            netcam_md_get_sensitivity(0, &sensi);
            sensi +=1;

            snprintf(rsp_pxml, 600,
                "{\n"
                    "\"alarm\":\n"
                    "{\n"
                        "\"isset\": 0,\n"
                        "\"result\": 0,\n"
                        "\"errMsg\": \"\",\n"
                        "\"motion\": \n"
                        "{\n"
                          "\"enable\": %d,\n"
                          "\"level\": %d,\n"
                          "\"triggerSpeaker\": %d\n"
                        "}\n"
                     "}\n"
                "}\n",
                runMdCfg.enable,
                sensi,
                runMdCfg.handle.is_beep
                );

        }
        else if(1==isset)//set
        {
            printf("ddd %s,%d\n",__FUNCTION__,__LINE__);
            cJSON *mdJson = NULL;
            cJSON *mdEnableJson = NULL;
            cJSON *mdLevelJson = NULL;
            cJSON *mdTrgSpkJson = NULL;
            int mdenable,sensi,triggerSpeaker;

            mdJson = cJSON_GetObjectItem(alarmJson,"motion");
            if(mdJson ==NULL)
            {
                printf("get md json failed\n");
            }
            mdEnableJson = cJSON_GetObjectItem(mdJson,"enable");
            if(mdEnableJson==NULL)
            {
                printf("cJSON_GetObjectItem  mdEnableJson failed\n");
                return -1;
            }
            mdLevelJson = cJSON_GetObjectItem(mdJson,"level");
            if(mdLevelJson==NULL)
            {
                printf("cJSON_GetObjectItem  mdLevelJson failed\n");
                return -1;
            }

            mdTrgSpkJson = cJSON_GetObjectItem(mdJson,"triggerSpeaker");
            if(mdTrgSpkJson==NULL)
            {
                printf("cJSON_GetObjectItem  mdTrgSpkJson failed\n");
                return -1;
            }

            mdenable = mdEnableJson->valueint;
            sensi = mdLevelJson->valueint;
            triggerSpeaker = mdTrgSpkJson->valueint;


            printf("md set -> enable :%d,sensitivity %d,triggerSpeaker:%d\n",
                mdenable,
                sensi,
                triggerSpeaker
                );
            /*1,����mdʹ�ܿ���*/
            runMdCfg.enable =mdenable;

            /*2,��������������*/
            if(sensi>5 || sensi<1)
            {
                printf("!!!md set sensitivity is %d,out of range\n",sensi);
                sensi = 3;
            }
            netcam_md_set_sensitivity(0, sensi-1);

            /*3����������*/
            runMdCfg.handle.is_alarmout = triggerSpeaker;// triggerSpeaker;
            runMdCfg.handle.is_beep = triggerSpeaker;
            MdCfgSave();
            /*������Ҫ���ñ���ʱ��*/
            //runMdCfg.handle.duration = 100;

            //cJSON_Delete(mdJson);
            strcpy(rsp_pxml,pxml);
        }

        else
        {
            printf("alarm isset value err :%c\n",isset);
        }
        //cJSON_Delete(alarmJson);
    }

    cJSON_Delete(rootJson);

    return 0;


}

int mtws_control_timezone(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *tzJson = NULL;
    rootJson = cJSON_Parse(pxml);
    if (!rootJson)
    {
        printf("Unable to read rootJson\n");
        return -1;
    }


    /********************************
    2,ʱ�����úͻ�ȡ
    ********************************/
    /*
    "time": {
        "isset": 0,
        "result": 0,
        "errMsg": "",
        // <!-- ʱ������,���������ʱ��һ��:���� GTM +8:00��480,�÷��ӱ�ʾ-->
        "zone": {
          "value": 480
        }
      },
    */
    tzJson = cJSON_GetObjectItem(rootJson, "time");
    if(tzJson != NULL)
    {

        printf("DDDD %s,%d\n",__FUNCTION__,__LINE__);
        isset = cJSON_GetObjectItem(tzJson,"isset")->valueint;/*'0'=get,'1'=set*/
        printf("dddd alarm    isset:[%d]\n",isset);
        if(0==isset)//get
        {

            snprintf(rsp_pxml, 600,
                "{\n"
                    "\"time\":\n"
                    "{\n"
                        "\"isset\": 0,\n"
                        "\"result\": 0,\n"
                        "\"errMsg\": \"\",\n"
                        "\"zone\":\n"
                        "{\n"
                           "\"value\": %d\n"
                        "}\n"
                    "}\n"
                "}\n",
                runSystemCfg.timezoneCfg.timezone
                );
        }
        else if(1==isset)//set
        {

            cJSON *zoneJson = NULL;
            cJSON *zoneValueJson = NULL;
            int zoneValue;


            zoneJson = cJSON_GetObjectItem(tzJson, "zone");
            if(zoneJson ==NULL)
            {
               printf("get zoneJson  failed\n");
               return -1;
            }


            zoneValueJson = cJSON_GetObjectItem(zoneJson, "value");
            if(zoneValueJson ==NULL)
            {
               printf("get zoneValueJson  failed\n");
               return -1;
            }
            zoneValue= zoneValueJson->valueint;

            printf("timezone set -> zoneValue :%d\n",zoneValue);

            /*1,����ʱ�����÷��ӵķ�ʽ-720~720*/
            runSystemCfg.timezoneCfg.timezone = zoneValue;
            SystemCfgSave();
            strcpy(rsp_pxml,pxml);
        }
        else
        {

            printf("time isset value err :%d\n",isset);
        }

    }

    cJSON_Delete(rootJson);

    return 0;


}

int mtws_control_sound(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *soundJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    3,�������úͻ�ȡ
    ********************************/
    #if 0
    "sound": {
       "isset": 0,
       "result": 0,
       "errMsg": "",
       // <!-- �������뼴mic�Ĳɼ� -->
       // <!-- enable(ȡֵ0,1),1��ʾ����,=0��ʾ�ر� -->
       "in": {
         "enable": 0
       },

       // <!-- �������,�������ڶԽ����ǲ��ű�������ʱ������ -->
       // <!-- value(ȡֵ0-10),0��ʾ����,10Ϊ��� -->
       "out": {
         "value": 0
       }
     },

    #endif
    soundJson = cJSON_GetObjectItem(rootJson,"sound");
    if(soundJson != NULL)
    {

        printf("DDDD %s,%d\n",__FUNCTION__,__LINE__);
        isset = cJSON_GetObjectItem(soundJson,"isset")->valueint;/*'0'=get,'1'=set*/
        if(0==isset)//get
        {
            snprintf(rsp_pxml, 600,
            "{\n"
                "\"sound\": \n"
                "{\n"
                    "\"isset\": 0,\n"
                    "\"result\": 0,\n"
                    "\"errMsg\": \"\",\n"
                    "\"in\":\n"
                    "{\n"
                        "\"enable\": %d\n"
                    "},\n"
                    "\"out\":\n"
                    "{\n"
                        "\"value\": %d\n"
                    "}\n"
                "}\n"
            "}\n",
            0,
            0
            );

        }
        else if(1==isset)//set
        {
            cJSON *soundInJson = NULL;
            cJSON *soundOutJson = NULL;

            cJSON *soundInValueJson = NULL;
            cJSON *soundOutValueJson = NULL;
            int soundInEnable,soundOutValue;

            soundInJson = cJSON_GetObjectItem(soundJson, "in");
            if(soundInJson==NULL)
            {
                printf("cJSON_GetObjectItem  soundInJson failed\n");
                return -1;
            }
            soundOutJson = cJSON_GetObjectItem(soundJson, "out");
            if(soundOutJson==NULL)
            {
                printf("cJSON_GetObjectItem  soundOutJson failed\n");
                return -1;
            }


            soundInValueJson = cJSON_GetObjectItem(soundInJson, "enable");
            if(soundInValueJson==NULL)
            {
                printf("cJSON_GetObjectItem  soundInValueJson failed\n");
                return -1;
            }

            soundOutValueJson = cJSON_GetObjectItem(soundOutJson, "value");
            if(soundOutValueJson==NULL)
            {
                printf("cJSON_GetObjectItem  soundOutValueJson failed\n");
                return -1;
            }

            soundInEnable= soundInValueJson->valueint;
            soundOutValue= soundOutValueJson->valueint;

            printf("sound set -> in enable :%d,out value:%d\n",soundInEnable,soundOutValue);

            /*����????????????????????*/

            strcpy(rsp_pxml,pxml);

        }
        else
        {

            printf("time isset value err :%d\n",isset);
        }

    }

    cJSON_Delete(rootJson);

    return 0;


}

int mtws_control_account(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *accountJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    4,�˺Ż�ȡ
    ********************************/
    #if 0
    "account": {
       "result": 0,
       "errMsg": "",
       // <!-- user��ʾ�˺�,pass��ʾ����,��ɾ�����޸�ʱ,��Ҫ�õ�id -->
       "list": [
         {
           "id": 1,
           "user": "admin",
           "pass": "admin"
         },
         {
           "id": 2,
           "user": "guest",
           "pass": "123456"
         }
       ]
     },
    #endif
    accountJson = cJSON_GetObjectItem(rootJson,"account");
    if(accountJson != NULL)
    {
        int i,userNum=0;
        char arr[2000];
        char temStr[2000];
        memset(arr,0,sizeof(arr));

        for (i = 0; i < GK_MAX_USER_NUM; i ++)
        {
            if (runUserCfg.user[i].enable == 1)
            {

                memset(temStr,0,sizeof(temStr));
                snprintf(temStr, 600,
                    "{\n"
                        "\"id\": %d,\n"
                        "\"user\": \"%s\",\n"
                        "\"pass\": \"%s\"\n"
                    "},\n",
                    i,
                    runUserCfg.user[i].userName,
                    runUserCfg.user[i].password
                    );
                printf("temStr:%s\n",temStr);
                strcat(arr,temStr);
                userNum++;
            }
        }
        printf("arr %s,userNum =%d\n",arr,userNum);

        printf("DDDD %s,%d\n",__FUNCTION__,__LINE__);
        snprintf(rsp_pxml, 600,
            "{\n"
                "\"account\":\n"
                "{\n"
                   "\"result\": 0,\n"
                   "\"errMsg\": \"\",\n"
                   "\"list\":\n"
                   "[\n"
                     "%s\n"
                   "]\n"
                 "}\n"
             "}\n",
             arr
             );
    }

    cJSON_Delete(rootJson);

    return 0;


}

int mtws_control_accountAed(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *accountAedJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    5,�˺����ӡ��޸ġ�ɾ��
    ********************************/
    #if 0
    // <!-- �˺��������ɾ�� -->
    "accountAED": {
      // <!-- oper��ʾ������(a��ʾ����,e�޸�,dɾ��)-->
      // a����Ҫ����id
      // e,d�������id>0,��ֱ��ƥ��id�Ŷ�Ӧ���˺�(���˺�Ҳ�����޸�)�����id=0��ֱ�Ӱ�userȥƥ���û�
		  // <!-- result��ʾ�������,ͬ��������,<0����errMsg���Ա�ʾ������Ϣ -->
		  // <!-- �����ö��item -->
      "list": [{
        "id": 1,
        "oper": "a",
        "user": "admin",
        "pass": "admin",
        "result": 0,
        "errMsg": "",
      },]
    },

    #endif

    accountAedJson = cJSON_GetObjectItem(rootJson, "accountAED");
    if(accountAedJson != NULL)
    {

        int index;
        cJSON *accountArrJson=NULL;
        cJSON *accountArrItem = NULL;


        printf("DDDD  account_aed %s,%d\n",__FUNCTION__,__LINE__);
        accountArrJson = cJSON_GetObjectItem(accountAedJson, "list");
        if(accountArrJson == NULL)
        {
            printf("accountArrJson cmdjson is null\n");
            return -1;
        }

        int arraySize = cJSON_GetArraySize(accountArrJson);
        PRINT_INFO("accountArrJson arraySize=%d\n", arraySize);
        for( index = 0; index < arraySize; index++)
        {
            accountArrItem = cJSON_GetArrayItem(accountArrJson, index);
            if(NULL!=accountArrItem)
            {
                cJSON *idJson=NULL;
                cJSON *cmdJson=NULL;
                cJSON *userJson=NULL;
                cJSON *passJson=NULL;

                GK_NET_USER_INFO userInfo;

                cmdJson = cJSON_GetObjectItem(accountArrItem, "oper");
                if(cmdJson==NULL)
                {
                    printf("accountaed cmdjson is null\n");
                    return -1;
                }
                else
                {
                    idJson = cJSON_GetObjectItem(accountArrItem, "id");
                    if(idJson==NULL)
                    {
                        printf("accountaed idJson is null\n");
                        return -1;
                    }

                    userJson = cJSON_GetObjectItem(accountArrItem, "user");
                    if(userJson==NULL)
                    {
                        printf("accountaed userJson is null\n");
                        return -1;
                    }

                    passJson = cJSON_GetObjectItem(accountArrItem, "pass");
                    if(passJson==NULL)
                    {
                        printf("accountaed passJson is null\n");
                        return -1;
                    }

                    if('a'==*cmdJson->valuestring)//�����û�������Ĳ���
                    {



                    	memset(&userInfo, 0, sizeof(GK_NET_USER_INFO));
                    	strncpy(userInfo.userName, userJson->valuestring, sizeof(userInfo.userName));
                        strncpy(userInfo.password, passJson->valuestring, sizeof(userInfo.password));

                    	UserAdd(&userInfo);
                    	UserCfgSave();
                        UserCfgPrint();
                    }
                    else if('d'==*cmdJson->valuestring)//ɾ���û�������Ĳ���
                    {
                        if(userJson->valuestring!=NULL)
                        {
                            ret = UserDel(userJson->valuestring);
                            if(0!=ret)
                            {
                                printf("user %s not exist,failed to delete user\n",userJson->valuestring);
                            }

                        }
                        else
                        {
                            printf("NULL userName ,failed to delete user\n");
                        }
                        UserCfgSave();
                    }
                    else if('e'==*cmdJson->valuestring)//�޸��û�������Ĳ���
                    {
                    	GK_NET_USER_INFO userInfo;

                    	memset(&userInfo, 0, sizeof(GK_NET_USER_INFO));
                    	strncpy(userInfo.userName, userJson->valuestring, sizeof(userInfo.userName));
                        strncpy(userInfo.password, passJson->valuestring, sizeof(userInfo.password));

                    	if (UserModify(&userInfo))
                    		UserAdd(&userInfo);
                    	UserCfgSave();
                        CloudSdkUpdateAccount(userInfo.userName,userInfo.password);
                    }
                    else
                    {
                        printf("account cmd err :%s\n",cmdJson->valuestring);
                        return -1;
                    }
                }
            }
        }



        strcpy(rsp_pxml,pxml);
    }

    cJSON_Delete(rootJson);

    return 0;

}

int mtws_control_storage(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *storageJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    6,tf������
    ********************************/
    #if 0
    // <!-- TF�������ȡ -->
    "storage": {
      "result": 0,
      "errMsg": "",
      // <!-- name��ʾһ��Ψһ������,��ʱ����Ҫ��ʽ����ʱ����Ҫ�õ� -->
	  // <!-- total��free�ֱ��ʾ��������ʣ������ -->
	  // <!-- format��ʾ���̸�ʽ,unknow��ʾδ֪,��û�з������ʽ����none��ʾû�в���sdk�� -->
      "list": [
        {
          "name": "sd0",
          "total": 1024,
          "free": 300,
          "status": 0
        },
        {
          "name": "sd1",
          "total": 0,
          "free": 0,
          "status": -1
        }
      ]
    },
    #endif

    storageJson = cJSON_GetObjectItem(rootJson, "storage");
    if(storageJson != NULL)
    {

        printf("DDDD  storage %s,%d\n",__FUNCTION__,__LINE__);
        sdStatus res;
        res.status = mmc_get_sdcard_stauts();
        res.size = grd_sd_get_all_size();
        res.free = grd_sd_get_free_size();
        res.format_process = grd_sd_get_format_process();
        res.used = res.size -res.free;


        printf("get sdcard, status:%d all_size:%d used:%d free_size:%d process:%d\n", res.status, res.size, res.used,res.free, res.format_process);
        /********************************
        1,��ȡ��ز���ֵ
        ********************************/
        snprintf(rsp_pxml, 600,
            "{\n"
                "\"storage\":\n"
                "{\n"
                  "\"result\": 0,\n"
                  "\"errMsg\": \"\",\n"
                  "\"list\": \n"
                  "[\n"
                    "{\n"
                      "\"name\": \"sd0\",\n"
                      "\"total\": %d,\n"
                      "\"free\": %d,\n"
                      "\"status\": 0\n"
                    "}\n"
                  "]\n"
                "}\n"
            "}\n",
            res.size,
            res.free
            );
    }

    cJSON_Delete(rootJson);

    return 0;


}

int mtws_control_storageFmt(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *storageFmtJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    7,tf����ʽ��
    ********************************/
    #if 0
    // <!-- TF����ʽ�� -->
     "storageFormat": {
       "result": 0,
       "errMsg": "",
       // <!-- name��ʾҪ��ʽ���ĸ�sd�� -->
       "item": {
         "name": "sd0"
       }
     },
    #endif

    storageFmtJson = cJSON_GetObjectItem(rootJson, "storageFormat");
    if(storageFmtJson != NULL)
    {
        cJSON *fmtJson = NULL;
        cJSON *fmtItemJson = NULL;
        cJSON *fmtNameJson = NULL;
        char *pfmtName;

        printf("DDDD  storage_format %s,%d\n",__FUNCTION__,__LINE__);
        fmtItemJson = cJSON_GetObjectItem(storageFmtJson, "item");
        if(fmtItemJson==NULL)
        {
            printf("cJSON_GetObjectItem  fmtItemJson failed\n");
            return -1;
        }

        fmtNameJson = cJSON_GetObjectItem(fmtItemJson, "name");
        if(fmtNameJson==NULL)
        {
            printf("cJSON_GetObjectItem  fmtNameJson failed\n");
            return -1;
        }
        pfmtName= fmtNameJson->valuestring;

        printf("format stroage device is %s \n",pfmtName);
        ret = 0;
        ret=grd_sd_format();


        snprintf(rsp_pxml, 600,
            "{\n"
                "\"storageFormat\":\n"
                "{\n"
                  "\"result\": %d,\n"
                  "\"errMsg\": \"\",\n"
                  "\"item\": \n"
                  "{\n"
                    "\"name\": \"sd0\"\n"
                  "}\n"
                "}\n"
            "}\n",
            ret
            );
    }

    cJSON_Delete(rootJson);

    return 0;


}

static int mtws_get_ver(char *s_ver, unsigned long long *n_ver)
{
    char s_tmp[32] = {0};
    int i = last_index_at(s_ver, '.');
    strcpy(s_tmp, s_ver + i + 1);
    printf("s_tmp:%s\n", s_tmp);
    *n_ver = (unsigned long long)atoll(s_tmp);
}


int mtws_control_version(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *versionJson  = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    8,��ȡ�̼��汾
    ********************************/
    #if 0
    // <!-- ��ȡ�̼��汾 -->
     "verion": {
         "result": 0,
         "errMsg": "",
         // <!-- devid��ʾ�豸id�� -->
         // <!-- cloudid��ʾ��id��,���װ��һ�� -->
         // <!-- curver��ʾ��ǰ�汾�� -->
         // <!-- model��ʾ����ͺ� -->
         // <!-- odm��ʾodm�̺� -->
         // <!-- ���langΪ�ձ�ʾ������������,����̼�����ֻ����ָ�������Ը��� -->
         "devid": "1",
         "cloudid": "1",
         "curver": 1,
         "model": "1",
         "odm": 1,
         "lang": 0
     },
    #endif
    versionJson = cJSON_GetObjectItem(rootJson, "verion");
    if(versionJson != NULL)
    {
        printf("DDDD  verion %s,%d\n",__FUNCTION__,__LINE__);

        unsigned long long n_ver = 0;
        mtws_get_ver(runSystemCfg.deviceInfo.upgradeVersion, &n_ver);
        printf("now -- s_ver:%s n_ver:%lld\n", runSystemCfg.deviceInfo.upgradeVersion, n_ver);

        snprintf(rsp_pxml, 600,
            "{\n"
                "\"verion\":\n"
                "{\n"
                    "\"result\": 0,\n"
                    "\"errMsg\": \"\",\n"
                    "\"devid\": \"%s\",\n"
                    "\"cloudid\": \"%s\",\n"
                    "\"curver\": %lld,\n"
                    "\"model\": \"1\",\n"
                    "\"odm\": 1,\n"
                    "\"lang\": %d\n"
                "}\n"
            "}\n",
            runSystemCfg.deviceInfo.serialNumber,
            devId,
            n_ver,
            runSystemCfg.deviceInfo.languageType);
        #if 0
        runSystemCfg.deviceInfo.hardwareVersion
        #endif

    }

    cJSON_Delete(rootJson);

    return 0;


}
int md5_check(unsigned char *inEncrypt,unsigned int fileSize,unsigned char *outDecrypt)
{

    MD5_CTX md5;
    MD5Init(&md5);
    int i;
    //unsigned char encrypt[] ="admin";//21232f297a57a5a743894a0e4a801fc3
    //unsigned char decrypt[16];
    MD5Update(&md5,inEncrypt,fileSize);
    MD5Final(&md5,outDecrypt);

    //printf("\n befor md5:%s\n after md5 32 bit:\n",inEncrypt);
    printf("\n after md5 32bit out Decypty :\n");
#if 0
    for(i=0;i<16;i++)
    {
        printf("%02x",outDecrypt[i]);  //02xǰ��Ҫ���� %
    }

    printf("\n");
#endif
    return 0;

}
int mtws_control_update(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;
    cJSON *updateJson  = NULL;
    char *purl = NULL;
    char *ver_str = NULL;
    char *updateFile="/tmp/update.bin";
    char *fileData = NULL;

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }



    /********************************
    9,���� ����
    ********************************/
    #if 0
    // <!-- �������� -->
      "update": {
        "result": 0,
        "errMsg": "",
        // url��һ���ǿ�ѡ��,���������,��ʾ��ָ����urlֱ��download������
        // ���url����valueΪ��,��ͨ��http://ts.vs98.com:8080/api/v1/update?devid=...���urlȥ��ȡ����
        "url": "http://aaa/bin/a1000.bin",
      },
    #endif
    updateJson = cJSON_GetObjectItem(rootJson, "update");
    if(updateJson != NULL)
    {
        unsigned char *recv_md5=NULL;
        unsigned char mtws_md5_arr[16];

        cJSON *urlJson = NULL;
        //cJSON *urlValueJson = NULL;

        urlJson = cJSON_GetObjectItem(updateJson, "url");
        if(urlJson != NULL &&  strlen(urlJson->valuestring)>1)
        {
            purl = urlJson->valuestring;
            printf("update from url : %s\n",purl);
        }
        else
        {
            cJSON *updateJson, *verJson, *urlJson,*md5Json;
            char cmd[500];

            //char *tem_purl = "http://ts.vs98.com:8080/api/v1/update?devid=2903987&cloudid=2903987&curver=1012&model=1&odm=1";
            char tem_purl[500];

            unsigned long long now_n_ver = 0;
            mtws_get_ver(runSystemCfg.deviceInfo.upgradeVersion, &now_n_ver);
            printf("now -- s_ver:%s n_ver:%lld\n", runSystemCfg.deviceInfo.upgradeVersion, now_n_ver);

            #if 1

            snprintf(tem_purl,500,"http://ts.vs98.com:8080/api/v1/update?devid=%s&cloudid=%s&curver=%lld&model=1&odm=1"
            ,runSystemCfg.deviceInfo.serialNumber,
            devId,
            now_n_ver);
            printf("tem_purl:%s\n",tem_purl);
            #endif
            snprintf(cmd,500,"wget '%s' -O /tmp/update.json"
            ,tem_purl);


            printf("system call: %s\n",cmd);
            ret = system(cmd);
            printf("wget json cmd ret :%d\n",ret);


            updateJson=cJSON_ReadFile("/tmp/update.json");//���ļ�ϵͳ�ж�ȡ JSON ��Ŀ
            if(updateJson != NULL)
            {

                char * tmpPri = cJSON_Print(updateJson);
                printf("update  JSON pri:%s\n",tmpPri);

                /* ��ȡ�����ļ��İ汾�� */
                verJson = cJSON_GetObjectItem(updateJson, "ver");
                if (verJson == NULL)
                {
                    printf("verJson null \n\n");
                    ret = UPDATE_JSON_ERR;
                    goto err0;
                }
                else
                {
                    ver_str = verJson->valuestring;
                }

                /* �ж������ļ��İ汾���¾��������Ƿ����� */
                char upgrade_s_ver[MAX_SYSTEM_STR_SIZE] = {0};
                snprintf(upgrade_s_ver, "%s", ver_str);
                unsigned long long upgrade_n_ver = 0;
                mtws_get_ver(upgrade_s_ver, &upgrade_n_ver);
                printf("upgrade -- s_ver:%s n_ver:%lld\n", upgrade_s_ver, upgrade_n_ver);
#if 0
                unsigned long long now_n_ver = 0;
                mtws_get_ver(runSystemCfg.deviceInfo.upgradeVersion, &now_n_ver);
                printf("now -- s_ver:%s n_ver:%lld\n", runSystemCfg.deviceInfo.upgradeVersion, now_n_ver);
#endif
                if (now_n_ver >= upgrade_n_ver)
                {
                    PRINT_INFO("no need upgrade, now is the newest version.");
                    ret = UPDATE_VERSION_ERR;
                    goto err0;
                }
                else
                {
                    PRINT_INFO("to upgrade: the newest version is %lld, local version is %lld.", upgrade_n_ver, now_n_ver);
                }


                urlJson = cJSON_GetObjectItem(updateJson, "url");
                if(urlJson != NULL)
                {
                    purl = urlJson->valuestring;
                }
                else
                {
                    printf("urlJson null \n\n");
                    ret = UPDATE_JSON_ERR;
                    goto err0;

                }

                md5Json = cJSON_GetObjectItem(updateJson, "md5");
                if(urlJson != NULL)
                {
                    recv_md5 = md5Json->valuestring;
                    printf("recv md5 %s\n",recv_md5);
                }
                else
                {
                    printf("md5 null \n\n");
                    ret = UPDATE_JSON_ERR;
                    goto err0;
                }
            }
            else
            {
                printf("updateJson /tmp/update.json not exit \n\n");
                ret = UPDATE_JSON_ERR;
                goto err0;
            }
            printf("update from default url : %s\n",purl);
            /*to do ??????????*/
        }
        char cmd[500];
        long fileSize = 0;
        long fileLen = 0;
        long offset = 0;
        int cnt;
        //wget http://ts.vs98.com:8080/firmware/gk7108v1_fw.bin

        /*�����ļ�������*/
        snprintf(cmd,500,"wget '%s' -O %s",
        purl,
        updateFile);

        printf("system call: %s\n",cmd);
        ret = system(cmd);
        printf("update ret :%d\n");

        /*1 �ͷ�ϵͳ��Դ*/

        netcam_update_relase_system_resource();

        sdk_video_enc_stop();
        sdk_audio_enc_stop();
        //mtws_app_exit();
        mtws_thread_running = 0;


        //upgrade_flash_by_file(updateFile);

        FILE *fp = NULL;
        fp = fopen(updateFile, "rb");
        if (fp == NULL)
        {
            PRINT_ERR("fopen error %s not exist!", updateFile);
            ret = UPDATE_FILE_ERR;
            goto err;
        }
        //��ȡ�ļ���С
        if (0 != fseek(fp, 0, SEEK_END))
        {
            fclose(fp);
            ret = UPDATE_FILE_ERR;
            goto err;
        }
        fileSize = ftell(fp);
        printf("update fileSize:%d\n",fileSize);
        fileData = netcam_update_malloc(fileSize);
        if(!fileData)
        {
            PRINT_ERR();
            fclose(fp);
            ret = UPDATE_MEM_ERR;
            goto err;
        }
        memset(fileData, 0, fileSize);

        if(0 != fseek(fp, 0, SEEK_SET))
        {
            PRINT_ERR();
            free(fileData);
            fileData = NULL;
            fclose(fp);
            ret = UPDATE_FILE_ERR;
            goto err;
        }

        //��ȡ�����ļ�
        fileLen = fileSize;
        while (fileLen > 0)
        {
            ret = fread(fileData + offset, 1, fileLen, fp);
            if (ret >= 0)
            {
                offset += ret;
                fileLen -= ret;
            }
            else
            {
                PRINT_INFO("offset:%d  len:%d  ret:%d\n", offset, fileLen, ret);
                //break;
            }

            //PRINT_INFO("offset:%d  len:%d  ret:%d\n", offset, fileLen, ret);
        }
        fclose(fp);

        if(recv_md5 != NULL)
        {
            int ii;

            md5_check(fileData,fileSize,mtws_md5_arr);
            #if 0
            printf("\nxxxx check md5 :\n");
            for(ii = 0;ii < 16;ii++)
            {
                printf("%2x",mtws_md5_arr[ii]);
            }
            printf("\nxxxx recv md5 :%s\n",recv_md5);
            #endif

            char md5Str[32]={'\0'};
            snprintf(md5Str,500,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
            mtws_md5_arr[0],mtws_md5_arr[1],mtws_md5_arr[2],mtws_md5_arr[3],
            mtws_md5_arr[4],mtws_md5_arr[5],mtws_md5_arr[6],mtws_md5_arr[7],
            mtws_md5_arr[8],mtws_md5_arr[9],mtws_md5_arr[10],mtws_md5_arr[11],
            mtws_md5_arr[12],mtws_md5_arr[13],mtws_md5_arr[14],mtws_md5_arr[15]
            );
            printf("final md5str:%s\n",md5Str);
            /*�Ƚϼ���õ���md5ֵ(md5Str)���ͽ��յ���md5ֵ(recv_md5)������ͬ��˵���ļ����صĲ���ȷ*/
            if(strcmp(md5Str,recv_md5) != 0)
            {
                printf("md5 check failed,download file have error\n");
                ret = UPDATE_MD5_ERR;
                free(fileData);
                fileData = NULL;
                goto err;

            }
        }
        //��ʼ����

        //ret = upgrade_flash_by_file(updateFile);
        ret = netcam_update(fileData, fileSize, NULL);
        if (ret < 0)
        {
            PRINT_ERR("Check updating package, failed: %d \n", ret);
            free(fileData);
            fileData = NULL;
            ret = UPDATE_UP_ERR;
            goto err;
        }


        cnt = 0;
        while (cnt < 99)
        {
            cnt = netcam_update_get_process();
            fileLen = (fileSize * cnt)/100/2;
            usleep(218 * 1000);
            PRINT_INFO("-----> len:%d  cnt:%d  ret:%d \n", fileLen, cnt, ret);
        }

        //usleep(100 * 1000);
        if(fileData != NULL)
        {
            free(fileData);
            fileData = NULL;
        }
        PRINT_INFO("-----> update success ,reboot now! \n");
		#if 0
        netcam_exit(90);
        new_system_call("reboot -f");//force REBOOT
        system("reboot");
		#endif
        netcam_sys_operation(0,SYSTEM_OPERATION_REBOOT);
        strcpy(rsp_pxml,pxml);
    }

    cJSON_Delete(rootJson);

    printf("DDDD %s,%d\n",__FUNCTION__,__LINE__);
    return 0;
err0:
    snprintf(rsp_pxml, 600,
    "{\n"
        "\"update\":\n"
        "{\n"
            "\"url\": \"%s\",\n"
            "\"result\": %d,\n"
        "}\n"
    "}\n",
    purl,
    ret
    );
    return ret;
err:

    snprintf(rsp_pxml, 600,
        "{\n"
            "\"update\":\n"
            "{\n"
                "\"url\": \"%s\",\n"
                "\"result\": %d,\n"
            "}\n"
        "}\n",
        purl,
        ret
        );
    //updateFile
    new_system_call("rm -fr /tmp/update.bin");
	#if 0
    netcam_exit(90);
    new_system_call("reboot -f");//force REBOOT
	#else
	netcam_sys_operation(0,SYSTEM_OPERATION_REBOOT);
	#endif
    #if 0
	mtws_thread_running = 1;
	CreateDetachThread(mtws_stream0_thread, NULL, NULL);
    CreateDetachThread(mtws_stream1_thread, NULL, NULL);
    CreateDetachThread(mtws_local_listen_thread, NULL, NULL);
    #endif
    return ret;

}

int mtws_control_systemCall(char *pxml,char *rsp_pxml,int rsp_maxLen)
    {
        int ret=0;
        int isset;
        int xmlLen;
        cJSON  *rootJson = NULL;

        cJSON *systemJson  = NULL;

        rootJson = cJSON_Parse(pxml);
        if (!rootJson) {
            printf("Unable to read rootJson\n");
            return -1;
        }



        /********************************
        10,ϵͳ����
        ********************************/
    #if 0
        // <!-- ϵͳ���� -->
         "system": {
             //  <!-- �����ж��shell�� -->
             // <!-- value��ʾshellָ��,ֱ�����м��� -->
             // <!-- result��ʾshell�ķ���ֵ -->
             // <!-- msg�Ǳ�ʾ�ն˵���� -->
           "shell": {
             "value": "reboot",
             "result": 0,
             "msg" : "",
           }
         }
	#endif
        systemJson = cJSON_GetObjectItem(rootJson, "system");
        if(systemJson != NULL)
        {

            printf("DDDD  system %s,%d\n",__FUNCTION__,__LINE__);
            cJSON *shellJson = NULL;
            cJSON *shellValueJson = NULL;

            char *pshellString;
            shellJson = cJSON_GetObjectItem(systemJson, "shell");
            if(shellJson==NULL)
            {
                printf("cJSON_GetObjectItem  shellJson failed\n");
                return -1;
            }

            shellValueJson = cJSON_GetObjectItem(shellJson, "value");
            if(shellValueJson==NULL)
            {
                printf("cJSON_GetObjectItem  shellValueJson failed\n");
                return -1;
            }
            pshellString = shellValueJson->valuestring;
            //system(pshellString);
            char cret[5000];
            ret=0;
            mtws_getStringBySystem(pshellString,cret,sizeof(cret),&ret);

            snprintf(rsp_pxml, 600,
                "{\n"
                    "\"system\":\n"
                    "{\n"
                        "\"shell\":\n"
                        "{\n"
                            "\"value\": \"%s\",\n"
                            "\"result\": %d,\n"
                            "\"msg\" : \"%s\"\n"
                        "}\n"
                    "}\n"
                "}\n",
                pshellString,
                ret,
                cret
                );

        }

        cJSON_Delete(rootJson);

        return 0;
}

int mtws_control_json(char *pxml,char *rsp_pxml,int rsp_maxLen)
{
    int ret=0;
    int isset;
    int xmlLen;
    cJSON  *rootJson = NULL;

    cJSON *alarmJson = NULL;
    cJSON *tzJson = NULL;
    cJSON *soundJson  = NULL;
    cJSON *accountJson  = NULL;
    cJSON *accountAedJson  = NULL;
    cJSON *storageJson  = NULL;

    cJSON *storageFmtJson  = NULL;
    cJSON *versionJson  = NULL;
    cJSON *updateJson  = NULL;

    cJSON *systemJson  = NULL;
    /******************
    ��xml�ж�ȡ�������
    *******************/
    /*
  	// <!-- ==================���б��Ĺ���===================== -->
  	// 				***********ע��������Ҫ������*******
  	// <!-- isset(ȡֵ0,1),=0��ʾget,=1��ʾset-->
  	// <!-- result(ȡֵ0,-n)=0��ʾ�ɹ�,<0��ʾʧ��,errMsg���潫�д�����Ϣ -->
  	// <!-- ע���е�get��ʾ���ݴ� -->
  	// ʾ��:
  	// 		�ֻ��˱�������(get):
  	// 			{"arlam":{"isset":0}}
  	// 		IPCӦ��:
  	// 			{"arlam":{"isset":0,"result": 0,"errMsg": "","motion": {"enable": 1, "level": 1, "triggerSpeaker": 0 }}}
  	//
  	// 		�ֻ������ñ���(set):
  	// 			{"arlam":{"isset":1,"motion": {"enable": 1, "level": 5, "triggerSpeaker": 1 }}}
  	// 		IPCӦ��:
  	// 			{"arlam":{"isset":1,"result": 0,"errMsg": ""}}
  	// <!-- =================================================== -->

    */
    /*������*/

    rootJson = cJSON_Parse(pxml);
    if (!rootJson) {
        printf("Unable to read rootJson\n");
        return -1;
    }

    char * tmpPri = cJSON_Print(rootJson);
    printf("input xml to JSON pri:%s\n",tmpPri);
    /********************************
    1,����alarm
    ********************************/
    alarmJson = cJSON_GetObjectItem(rootJson,"alarm");
    if(alarmJson != NULL)
    {
        mtws_control_alarm(pxml,rsp_pxml,rsp_maxLen);
    }

    /********************************
    2,ʱ�����úͻ�ȡ
    ********************************/
    tzJson = cJSON_GetObjectItem(rootJson, "time");
    if(tzJson != NULL)
    {
        mtws_control_timezone(pxml,rsp_pxml,rsp_maxLen);
    }


    /********************************
    3,�������úͻ�ȡ
    ********************************/
    soundJson = cJSON_GetObjectItem(rootJson,"sound");
    if(soundJson != NULL)
    {
        mtws_control_sound(pxml,rsp_pxml,rsp_maxLen);
    }

    /********************************
    4,�˺Ż�ȡ
    ********************************/
    accountJson = cJSON_GetObjectItem(rootJson,"account");
    if(accountJson != NULL)
    {
        mtws_control_account(pxml,rsp_pxml,rsp_maxLen);
    }

    /********************************
    5,�˺����ӡ��޸ġ�ɾ��
    ********************************/
    accountAedJson = cJSON_GetObjectItem(rootJson, "accountAED");
    if(accountAedJson != NULL)
    {
        mtws_control_accountAed(pxml,rsp_pxml,rsp_maxLen);
    }


    /********************************
    6,tf������
    ********************************/
    storageJson = cJSON_GetObjectItem(rootJson, "storage");
    if(storageJson != NULL)
    {
        mtws_control_storage(pxml,rsp_pxml,rsp_maxLen);
    }

    /********************************
    7,tf����ʽ��
    ********************************/
    storageFmtJson = cJSON_GetObjectItem(rootJson, "storageFormat");
    if(storageFmtJson != NULL)
    {
        mtws_control_storageFmt(pxml,rsp_pxml,rsp_maxLen);
    }


    /********************************
    8,��ȡ�̼��汾
    ********************************/
    versionJson = cJSON_GetObjectItem(rootJson, "verion");
    if(versionJson != NULL)
    {
        mtws_control_version(pxml,rsp_pxml,rsp_maxLen);
    }
    /********************************
    9,���� ����
    ********************************/
    updateJson = cJSON_GetObjectItem(rootJson, "update");
    if(updateJson != NULL)
    {
        mtws_control_update(pxml,rsp_pxml,rsp_maxLen);
    }

    /********************************
    10,ϵͳ����
    ********************************/
    systemJson = cJSON_GetObjectItem(rootJson, "system");
    if(systemJson != NULL)
    {
        mtws_control_systemCall(pxml,rsp_pxml,rsp_maxLen);
    }

    cJSON_Delete(rootJson);

    return 0;


}



int mtws_system_control_cb(int msg_code, char *xml, char *rsp_xml, int rsp_len, void *pUserContext)
{
    int ret;
    printf("CCC mtws_system_control_cb msg_code:%d,xml:%s\n",msg_code,xml);
    char tmp_rsp_xml[1000];
    char tmp_xml[1000];
    char tmp_xml2[1000];
    char tmp_xml3[1000];
    memset(tmp_xml,0,sizeof(tmp_xml));
    memset(tmp_xml2,0,sizeof(tmp_xml));
    memset(tmp_xml3,0,sizeof(tmp_xml));
    memset(tmp_rsp_xml,0,sizeof(tmp_rsp_xml));

    printf("@@@@in put xml strlen:%d,sizeof rsp_xml size:%d\n",strlen(xml),sizeof(rsp_xml));
    /*���粻��ֱ�Ӵ�json��ʽ����Ҫ������������xml�ַ�����ת��base64��ʽ*/
    mtws_base64_decode(xml,tmp_xml,sizeof(tmp_xml));
    printf("@@@@mtws_base64_decode intput xml %s  ,%d\n",tmp_xml,strlen(tmp_xml));

    /****************
    Э�鴦��
    ****************/
    mtws_control_json(tmp_xml, tmp_rsp_xml,rsp_len);
    printf("!!!!!!! out put tmp_rsp_xml !!!!!\n");
    printf("%s\n",tmp_rsp_xml);

    ret = mtws_base64_encode(tmp_rsp_xml,strlen(tmp_rsp_xml),rsp_xml,strlen(rsp_xml));
    printf("@@@@ret :%d,mtws_base64_encode %s  ,%d\n",ret,rsp_xml,strlen(rsp_xml));


    return 0;
}


int mtws_p2p_talk_cb(char *audio_data, int audio_len, void *pUserContext)
{
    //printf("!!!!!!!!!!!!!!!!! call mtws_p2p_talk_cb\n");
#if MTWS_REC_TALK
    if (fp_talk == NULL)
    {
        fp_talk = fopen("/mnt/sd_card/talk_test.ulaw", "wb+");
        if (fp_talk == NULL)
        {
            PRINT_ERR("fopen error.\n");
            netcam_exit(90);
        }
        else
        {
            PRINT_INFO("1, open ok.");
        }
    }

    static int tmp_no = 0;
    if (tmp_no < 1500)
    {
        int ret = fwrite(audio_data, 1, audio_len, fp_talk);
        if (ret <= 0)
        {
            PRINT_ERR("write error.\n");
            netcam_exit(90);
        }
        PRINT_INFO("2, write ulaw, len:%d\n.", audio_len);
    }
    tmp_no ++;

    if (tmp_no == 1500)
    {
        PRINT_INFO("write ulaw, close\n.");
        fclose(fp_talk);
    }
#endif

    /* ��� ulaw ���� */
    netcam_audio_output(audio_data, audio_len, NETCAM_AUDIO_ENC_MU_LAW, SDK_TRUE);
    return 0;
}


int p2p_evt_process(int msgType,  void *pMsgContext, void *pUserContext)
{
    PRINT_INFO("p2pcmd: %d\n", msgType);

    int ret = 0;
    switch (msgType)
    {
        case REQ_USER_AUTH:
        {
    		REQ_USER_AUTH_T *pAuthMsg = (REQ_USER_AUTH_T *)(pMsgContext);
            ret = mtws_user_auth(pAuthMsg);
            break;
        }
        case REQ_STREAM_OPEN:
        {
            REQ_CHANNEL_OPEN_T *pChannelOpenMsg = (REQ_CHANNEL_OPEN_T *)(pMsgContext);
            ret = mtws_open_stream(pChannelOpenMsg);
            break;
        }
        case REQ_STREAM_CLOSE:
        {
    		REQ_CHANNEL_CLOSE_T *pChannelCloseMsg = (REQ_CHANNEL_CLOSE_T *)(pMsgContext);
            ret = mtws_close_stream(pChannelCloseMsg);
            break;
        }
        case REQ_PTZ_CONTROL:
        {
    		REQ_PTZ_ACTION_T* ptz_control = (REQ_PTZ_ACTION_T*)(pMsgContext);
            ret = mtws_ptz_control(ptz_control);
            break;
        }
        case REQ_CONFIG_GET:
        {

            printf("REQ_CONFIG_GET \n");
            REQ_CONFIG_GET_T* config_get = (REQ_CONFIG_GET_T*)(pMsgContext);
            ret = mtws_config_get(config_get);
            break;
        }
        case REQ_CONFIG_SET:
        {

            printf("REQ_CONFIG_SET \n");
            REQ_CONFIG_SET_T* config_set = (REQ_CONFIG_SET_T*)(pMsgContext);
            ret = mtws_config_set(config_set);
            break;
        }

        case REQ_REPLAY_QUERY:
        {
            PRINT_INFO("REQ_REPLAY_QUERY.\n");
            break;
        }
        case REQ_REPLAY_START:
        {
            PRINT_INFO("REQ_REPLAY_START.\n");
            break;
        }
        case REQ_REPLAY_STOP:
        {
            PRINT_INFO("REQ_REPLAY_STOP.\n");
            break;
        }
        case REQ_REPLAY_PAUSE:
        {
            PRINT_INFO("REQ_REPLAY_PAUSE.\n");
            break;
        }
        case REQ_REPLAY_RESUME:
        {
            PRINT_INFO("REQ_REPLAY_RESUME.\n");
            break;
        }
        case REQ_REPLAY_SEEK:
        {
            PRINT_INFO("REQ_REPLAY_SEEK.\n");
            break;
        }

        default:
        {
            PRINT_ERR("unknown cmd:%d !\n", msgType);
            break;
        }
    }

	return ret;
}


static int rtsp_get_pps_sps(char *buf, int len, char *pps, int *pps_len, char *sps, int *sps_len)
{
	//unsigned char flag[5];
	unsigned char *flag;//20130725
	int offset = 0;

	int config_len = 0;
	int iframe_offset = 0;

	while(offset < (len - 5))
	{
		//memcpy(flag, buf + offset, 5);
		flag=(unsigned char *)(buf + offset); //20130725

		if(flag[0]== 0 && flag[1]==0 && flag[2] == 0 && flag[3] == 1)
		{
			//printf("flag[4] =%x\n", flag[4]);

			if((flag[4] & 0x1F) == 0x06) //nal_type == SEI
			{
				config_len = offset;
				//break;
			}
			else if((flag[4] & 0x1F) == 0x05) //nal_type == I frame
			{
				if(config_len == 0)
					config_len = offset;

				iframe_offset = offset + 4;
				break;
			}
		}

		offset++;
	}

	//printf("config len = %d, iframe_offset = %d\n", config_len, iframe_offset);

	offset = 0;
	while(offset < (config_len - 5))
	{
		//memcpy(flag, buf + offset, 5);
		flag=(unsigned char *)(buf + offset); //20130725

		if(flag[0]== 0 && flag[1]==0 && flag[2] == 0 && flag[3] == 1)
		{
			if((flag[4] & 0x1F) == 0x08) //nal_type == PPS, johnnyling 20100628
			{
				memcpy(sps, buf + 4, offset - 4);
				*sps_len = offset - 4;

				//printf("offset = %d, size = %d, len = %03d, sps:", offset, config_len, *sps_len);
				//rtsp_print_hex_len(sps, *sps_len);
				//printf("\n");

				memcpy(pps, buf + offset + 4, config_len - offset - 4);
				*pps_len = config_len - offset - 4;

				//printf("offset = %d, size = %d, len = %03d, pps:", offset, config_len, *pps_len);
				//rtsp_print_hex_len(pps, *pps_len);
				//printf("\n");

				return iframe_offset;
			}
		}

		offset++;
	}

	return iframe_offset;
}

static int main_stream_send(char *frame, int len, unsigned char frame_type, unsigned long timestamp, void *pcontext)
{
	//printf("main_stream_callback, len = %d, isKey = %d\n", len, isKey);

	if (GK_NET_FRAME_TYPE_I == frame_type)
	{
		int iframe_offset;
		char pps[1024];
		char sps[1024];
		int pps_len;
		int sps_len;
		iframe_offset = rtsp_get_pps_sps(frame, len, pps, &pps_len, sps, &sps_len);
	    //printf("###########pps_len=%d, sps_len=%d, iframeoff=%d\n", pps_len, sps_len, iframe_offset);

		if(iframe_offset > 0)
		{
			VIDEO_STREAM_PARAM mainVideoParam;
			memset(&mainVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
			mainVideoParam.bitrate = 2000;
			strcpy(mainVideoParam.codec, "H264");
			mainVideoParam.width = 1280;
			mainVideoParam.height = 720;
			mainVideoParam.framerate= 25;

			int j;
			for(j=0;j<iframe_offset;j++)
			{
				char tmp[32];
				memset(tmp, 0, 32);
				sprintf(tmp, "%02x", frame[j]);
				strcat(mainVideoParam.config,tmp);
			}

			//printf("main config :%s\n", mainVideoParam.config);
		    set_vol(mainVideoParam.config, 0);
            #if 0
			CloudSetVideoParam(0, STREAM_VIDEO_MAIN, mainVideoParam);
			#endif
			//CloudSetVideoParam(1, STREAM_VIDEO_MAIN, mainVideoParam);
			//CloudSetVideoParam(2, STREAM_VIDEO_MAIN, mainVideoParam);
			//CloudSetVideoParam(3, STREAM_VIDEO_MAIN, mainVideoParam);

			g_p2pClient.m_bHasGetMainVol = 1;

		}
	}

	if (!g_p2pClient.m_bHasClients)
	{
		return 0;
	}

    int frameType = 0;
    switch (frame_type)
    {
        case GK_NET_FRAME_TYPE_I:
        {
    		frameType = FRAME_TYPE_I;
            break;
        }

        case GK_NET_FRAME_TYPE_P:
        {
    		frameType = FRAME_TYPE_P;
            break;
        }

        case GK_NET_FRAME_TYPE_A:
        {
    		frameType = FRAME_TYPE_AUDIO;
            break;
        }

        default:
        {
            PRINT_ERR("unknown frame type:%d !\n", frame_type);
            break;
        }
    }

    if ((frameType == FRAME_TYPE_I) || (frameType == FRAME_TYPE_P))
	    CloudWriteMainStreamData(0, frame, len, frameType, timestamp);
    else if (frameType == FRAME_TYPE_AUDIO)
    {
        /* ��Ϊû��ʹ�� g_p2pClient.m_mainOpen��m_subOpen ����������������
         * ����Ĭ��ֱ�Ӿ����������ʡ�
         * CloudWriteMainStreamData �� CloudWriteSubStreamData �����Լ����ơ�
         * CloudWriteAudioStreamData ������Ƶ���Ͳ�ͬ���̡߳�
         * ���� CloudWriteAudioStreamData ����һ�μ��ɡ�
         * �ʷ��ڷ��������̻߳��߷��ڷ����������߳�����ɣ�
         * ������ֻ���ڷ����������̡߳�
         */
        #if 1
        char a_buf[1024] = {0};
        int a_len = audio_ulaw_encode((unsigned char *)a_buf, (short *)frame, len);
        CloudWriteAudioStreamData(0, a_buf, a_len, timestamp);
        #else
        static unsigned char tmp_no = 1;
        char a_buf[160];
        memset(a_buf, tmp_no++, sizeof(a_buf));
        CloudWriteAudioStreamData(0, a_buf, 160, timestamp);
        tmp_no %= 9;
        #endif

        #if MTWS_REC_AUDIO
        if (fp == NULL)
        {
            fp = fopen("/mnt/sd_card/audio_test.ulaw", "wb+");
            if (fp == NULL)
            {
                PRINT_ERR("fopen error.\n");
                netcam_exit(90);
            }
            else
            {
                PRINT_INFO("1, open ok.");
            }
        }

        static int tmp_no = 0;
        if (tmp_no < 1500)
        {
            int ret = fwrite(a_buf, 1, a_len, fp);
            if (ret <= 0)
            {
                PRINT_ERR("write error.\n");
                netcam_exit(90);
            }
            PRINT_INFO("write ulaw, len:%d\n.", a_len);
        }
        tmp_no ++;

        if (tmp_no == 1500)
        {
            PRINT_INFO("write ulaw, close\n.");
            fclose(fp);
        }
        #endif

    }
    else
        PRINT_ERR("frame_type error:%d !\n", frame_type);

	return 0;
}

static int  sub_stream_send(char *frame, int len, unsigned char frame_type, unsigned long timestamp, void *pcontext)
{
	//printf("sub_stream_callback, len = %d, iskey = %d\n", len, isKey);

	if (GK_NET_FRAME_TYPE_I == frame_type)
	{
		int iframe_offset;
		char pps[1024];
		char sps[1024];
		int pps_len;
		int sps_len;
		iframe_offset = rtsp_get_pps_sps(frame, len, pps, &pps_len, sps, &sps_len);

		if(iframe_offset > 0)
		{
			//printf("@@@@@@@@@@@ pps_len=%d, sps_len=%d, iframeoff=%d\n", pps_len, sps_len, iframe_offset);

			VIDEO_STREAM_PARAM subVideoParam;
			memset(&subVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
			subVideoParam.bitrate = 1000;
			strcpy(subVideoParam.codec, "H264");
			subVideoParam.width = 640;
			subVideoParam.height = 360;
			subVideoParam.framerate= 25;

			int j;
			for(j=0;j< iframe_offset; j++)
			{
				char tmp[32];
				memset(tmp, 0, 32);
				sprintf(tmp, "%02x", frame[j]);
				strcat(subVideoParam.config, tmp);
			}
			//strcpy(subVideoParam.config, "");

			//printf("sub config :%s\n", subVideoParam.config);
            set_vol(subVideoParam.config, 1);
            #if 0
			CloudSetVideoParam(0, STREAM_VIDEO_SUB, subVideoParam);
			#endif
			//CloudSetVideoParam(1, STREAM_VIDEO_SUB, subVideoParam);
			//CloudSetVideoParam(2, STREAM_VIDEO_SUB, subVideoParam);
			//CloudSetVideoParam(3, STREAM_VIDEO_SUB, subVideoParam);

			g_p2pClient.m_bHasGetSubVol = 1;

		}
	}


	if (!g_p2pClient.m_bHasClients)
	{
		return 0;
	}

    int frameType = 0;
    switch (frame_type)
    {
        case GK_NET_FRAME_TYPE_I:
        {
    		frameType = FRAME_TYPE_I;
            break;
        }

        case GK_NET_FRAME_TYPE_P:
        {
    		frameType = FRAME_TYPE_P;
            break;
        }

        case GK_NET_FRAME_TYPE_A:
        {
    		frameType = FRAME_TYPE_AUDIO;
            break;
        }

        default:
        {
            PRINT_ERR("unknown frame type:%d !\n", frame_type);
            break;
        }
    }

    if ((frameType == FRAME_TYPE_I) || (frameType == FRAME_TYPE_P))
	    CloudWriteSubStreamData(0, frame, len, frameType, timestamp);
    else if (frameType == FRAME_TYPE_AUDIO)
    {
		/* �����������̷߳��ͼ��ɣ������ٷ��;��ظ��� */
        //char a_buf[1024] = {0};
        //int a_len = audio_alaw_encode(a_buf, frame, len);
        //CloudWriteAudioStreamData(0, a_buf, a_len, timestamp);
    }
    else
        PRINT_ERR("frame_type error:%d !\n", frame_type);

	return 0;
}

#if 0
static int audio_callback(char *frame, int len, double timestamp, void *pcontext)
{
	return CloudWriteAudioStreamData(0, frame, len, timestamp);
}
#endif

static void init_video_stream_param()
{
	int i;
	for (i = 0; i < 1; i++) {
		VIDEO_STREAM_PARAM mainVideoParam;
		memset(&mainVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
		mainVideoParam.bitrate = 2000;
		strcpy(mainVideoParam.codec, "H264");
		mainVideoParam.width = 1280;
		mainVideoParam.height = 720;
		mainVideoParam.framerate= 25;
        get_vol(mainVideoParam.config, 0);
		CloudSetVideoParam(i, STREAM_VIDEO_MAIN, mainVideoParam);


		VIDEO_STREAM_PARAM subVideoParam;
		memset(&subVideoParam, 0, sizeof(VIDEO_STREAM_PARAM));
		subVideoParam.bitrate = 1000;
		strcpy(subVideoParam.codec, "H264");
		subVideoParam.width = 640;
		subVideoParam.height = 360;
		subVideoParam.framerate= 25;
        get_vol(subVideoParam.config, 1);
		CloudSetVideoParam(i, STREAM_VIDEO_SUB, subVideoParam);

		AUDIO_STREAM_PARAM  audioParam;
		memset(&audioParam, 0, sizeof(AUDIO_STREAM_PARAM));
		audioParam.enable = 1;
		strcpy(audioParam.codec, "PCMU");
		audioParam.samplerate = 8000;
		audioParam.samplewidth = 16;
		audioParam.channels = 1;
		audioParam.bitrate = 64;

		CloudSetAudioParam(i, audioParam);
	}
}

#if 0
static int load_mtws_config(MTWS_CONFIG *pconfig)
{
    #if 1
    int ret = get_from_mtd("/dev/mtd5", 0x8000, pconfig, sizeof(MTWS_CONFIG));
    if (ret != 0) {
        PRINT_ERR("get_from_mtd failed.\n");
        return -1;
    }
    if (pconfig->size != sizeof(MTWS_CONFIG)) {
        //д��Ĭ�� AP name
        strcpy(pconfig->ap_name, MTWS_AP_NAME);
        pconfig->size = sizeof(MTWS_CONFIG);
        PRINT_INFO("write to mtd, default ap name:%s\n", pconfig->ap_name);
        ret = set_to_mtd("/dev/mtd5", 0x8000, pconfig, sizeof(MTWS_CONFIG));
        if (ret != 0) {
            PRINT_ERR("set_to_mtd failed.\n");
            return -1;
        }

        //����APģʽ��SSID
        char ap_buf[32] = {0};
        netcam_wifi_ap_get_ssid(ap_buf, sizeof(ap_buf));
        PRINT_INFO("1 cfg file, ap name:%s\n", ap_buf);
        netcam_wifi_ap_set_ssid(pconfig->ap_name);
        netcam_wifi_ap_get_ssid(ap_buf, sizeof(ap_buf));
        PRINT_INFO("2 cfg file, ap name:%s\n", ap_buf);

        //��MTD�ж�ȡ��֤
        memset(pconfig, 0, sizeof(MTWS_CONFIG));
        get_from_mtd("/dev/mtd5", 0x8000, pconfig, sizeof(MTWS_CONFIG));
        PRINT_INFO("3 size:%d, ap_name:%s\n", pconfig->size, pconfig->ap_name);

    } else {
        PRINT_INFO("0, get from mtd, size:%d ap_name:%s", pconfig->size, pconfig->ap_name);
    }

    #endif

    return 0;
}
#endif

static int mtws_send(MEDIABUF_HANDLE reader, int stream_id)
{
    //PRINT_INFO();
    if (reader == NULL) {
        PRINT_ERR("not add reader.\n");
        return -1;
    }

    GK_NET_FRAME_HEADER frame_header = {0};
    void *frame_data = NULL;
    int frame_len = 0;
    int ret;

    ret = mediabuf_read_frame(reader, (void **)&frame_data, &frame_len, &frame_header);
    if (ret < 0) {
		PRINT_ERR("cms read stream%d : mediabuf_read_frame fail.\n", stream_id);
		return -1;
    } else if (ret == 0) {
        PRINT_INFO("stream%d, mediabuf_read_frame = 0\n", stream_id);
        return 0;
    } else {
        //PRINT_INFO("cms read stream%d : mediabuf_read_frame OK.\n", tBST);
    }

    struct timeval timev;
	gettimeofday(&timev, NULL);
	int timv_stamp = timev.tv_sec * 1000 + timev.tv_usec / 1000;

    if (stream_id == 0) {
        main_stream_send((char *)frame_data, frame_len, frame_header.frame_type, (unsigned long)timv_stamp, NULL);
    } else if (stream_id == 1) {
        sub_stream_send((char *)frame_data, frame_len, frame_header.frame_type, (unsigned long)timv_stamp, NULL);
    } else {
        PRINT_ERR("%d not support.\n", stream_id);
    }

    return 0;
}


int mtws_send_stream(int stream_id)
{
    MEDIABUF_HANDLE reader = mediabuf_add_reader((GK_NET_STREAM_TYPE)stream_id);
    if (reader == NULL) {
        PRINT_ERR("Add reader to stream%d error.\n", stream_id);
        return -1;
    }

    while (1) {
        if (mtws_thread_running == 0)
            break;

        mtws_send(reader, stream_id);
    }

    mediabuf_del_reader(reader);
    PRINT_INFO("mtws send stream%d process end.\n", stream_id);
    return 0;
}


static void * mtws_stream0_thread(void *para)
{
    SetThreadName("mtws_stream0_thread");

    mtws_send_stream(0);

    PRINT_INFO("mtws_stream0_thread end !\n");
    return NULL;
}

static void * mtws_stream1_thread(void *para)
{
    SetThreadName("mtws_stream1_thread");

    mtws_send_stream(1);

    PRINT_INFO("mtws_stream1_thread end !\n");
    return NULL;
}

static int mtws_mac_to_devid(char *dev_id, char *mac)
{
    char *src = mac;
    char *dst = dev_id;

    /* device Id: F0 + MAC */
    *dst ++ = 'F';
    *dst ++ = '0';
    *dst ++ = 'G';
    *dst ++ = 'K';

    while (*src != '\0') {
        if (*src != ':') {
            *dst = *src;
            src ++;
            dst ++;
        } else {
            src ++;
        }
    }
    *dst = '\0';

    return 0;
}

static int read_mtws_p2p_id()
{
    struct device_info_mtd info;
    if(load_info_to_mtd_reserve(MTD_MTWS_P2P, &info, sizeof(info)))
    {
        PRINT_ERR("[p2p] Fail to load p2p id from flash\n");
        return -1;
    }

    if(info.device_id[0] == 0)
    {
    	PRINT_ERR("[p2p] no p2p device id!\n");
    	return -1;
    }

    strncpy(runSystemCfg.deviceInfo.serialNumber, info.device_id,
    	sizeof(runSystemCfg.deviceInfo.serialNumber));

    SystemCfgSave();
    PRINT_INFO("[p2p] get from mtd, serialNumber:%s\n", runSystemCfg.deviceInfo.serialNumber);

    return 0;
}

static int mtwx_load_p2p_id()
{
    #if 1
    struct device_info_mtd info;
    if(load_info_to_mtd_reserve(MTD_TUTK_P2P, &info, sizeof(info)))
    {
        PRINT_ERR("[p2p] Fail to load p2p id from flash\n");
        return -1;
    }

    if(info.device_id[0] == 0)
    {
    	PRINT_ERR("[p2p] no p2p device id!\n");
    	return -1;
    }

    strncpy(runSystemCfg.deviceInfo.deviceType, info.device_id,
    	sizeof(runSystemCfg.deviceInfo.deviceType));

    #else
    sprintf(runSystemCfg.deviceInfo.serialNumber, "%s", "EFPA9D5WKV74AG6GY1XJ");
    #endif
    SystemCfgSave();
    PRINT_INFO("[p2p] get from mtd, deviceType:%s\n", runSystemCfg.deviceInfo.deviceType);

    return 0;
}


static int mtws_p2p_func()
{
    int ret = -1;

    /* �ȴ�stream�̵߳õ�VOL�����洢�� g_p2pClient ȫ�ֱ����� */
    while(1) {
        if (mtws_thread_running == 0) {
            PRINT_INFO("mtws thread stop all. \n");
            break;
        }

        if ((g_p2pClient.m_bHasGetMainVol == 1) && (g_p2pClient.m_bHasGetSubVol == 1)) {
            break;
        } else {
            PRINT_INFO("not get vol. \n");
            sleep(1);
            continue;
        }
    }

    printf("location --1.\n");
    //CloudSdkSeLogFunc(mtws_p2p_log_func, 0);

    #if 0
    printf("location --2.\n");
    /* �����APģʽ��������ȴ���ȡ·����SSID �� ����, ���л�STAģʽ���ӳɹ� */
    if (runNetworkCfg.wireless.mode == 0) {
        /* �ȴ���ȡ·����SSID �� ���� */
        while (1) {
            if (is_sta == 1)
                break;
            else {
                sleep(1);
                continue;
            }
        }
        #if 0
        printf("after ap to sta, test ping.\n");
        ret = netcam_ping_ip(TEST_IP);
        if (ret != 0) {
            printf("after ap to sta, ping error, so to reboot.\n");
            /* ���粻ͨ���л�APģʽ�������� */
            runNetworkCfg.wifi.mode = 0;
            NetworkCfgSave();
            system("reboot");
        } else {
            printf("after ap to sta, ping ok.\n");
            /* ����ͨ��,���´�������ʱ������STA���� */
            runNetworkCfg.wifi.mode = 1;
            NetworkCfgSave();
        }
        #else
        /* ����ͨ��,���´�������ʱ������STA���� */
        runNetworkCfg.wireless.mode = 1;
        NetworkCfgSave();
        #endif
    }
    #endif

    /* �ж�wifi ��sta ģʽ�Ƿ������� */
    printf("location --2.\n");

    /* ����wifi �����󣬻�ȡwifi mac ��ַ */
    char mac[20] = {0};

    //netcam_net_get()
    WIFI_HANDLE wifi_apt_name = net_wifi_open();
    sdk_net_get_hwaddr(wifi_apt_name, mac);

    PRINT_INFO("%s, cfg mac:%s, actual mac:%s\n", wifi_apt_name, runNetworkCfg.wifi.mac, mac);

    /* ����mac ��ַ�������ļ��� */
    if (strcmp(runNetworkCfg.wifi.mac, mac) != 0) {
        strcpy(runNetworkCfg.wifi.mac, mac);
        NetworkCfgSave();
    }

    //200 means NVR
    /* 4 means 4 channels */
    printf("location --3.\n");
    char dev_id[32] = {0};
    #if 1
    mtws_mac_to_devid(dev_id, runNetworkCfg.wifi.mac);
    PRINT_INFO("wifi_mac:%s  dev_id:%s\n", runNetworkCfg.wifi.mac, dev_id);
    printf("g, device_name:%s device_type:%s   run, deviceType:%s serialNumber:%s\n",
        g_systemInfo.device_name,
        g_systemInfo.device_type,
        runSystemCfg.deviceInfo.deviceType,
        runSystemCfg.deviceInfo.serialNumber);

    strncpy(runSystemCfg.deviceInfo.deviceType, dev_id, sizeof(runSystemCfg.deviceInfo.deviceType));
    #else
    mtwx_load_p2p_id();
    strncpy(dev_id, runSystemCfg.deviceInfo.deviceType, sizeof(runSystemCfg.deviceInfo.deviceType));
    #endif
    PRINT_INFO("deviceType:%s dev_id:%s\n", runSystemCfg.deviceInfo.deviceType, dev_id);

    //CloudSdkInit("37", "te918168", 100, 2, "03ABCE9B9872B59F");
    //CloudSdkInit("""37", "te918168", 100, 2, dev_id);
    //CloudSdkInit("37", "te918168", 100, 2, dev_id, "admin", "123456");
    PRINT_INFO("user:%s, psd:%s\n", runUserCfg.user[0].userName, runUserCfg.user[0].password);
    CloudSdkInit("37", "te918168", 100, 2, dev_id, runUserCfg.user[0].userName, runUserCfg.user[0].password);
    printf("location --4.\n");
    CloudRegisterNetEventCallBack(g_p2pClient.m_evtCB, &g_p2pClient);
    CloudRegisterAudioTalkbackCallBack(g_p2pClient.m_talkCB, &g_p2pClient);
    CloudRegisterSystemcontrolCallBack(g_p2pClient.m_controlCB, &g_p2pClient);

    sleep(5);
    printf("location --5.\n");
    init_video_stream_param();

    printf("location --6.\n");
    /* �ȴ��豸��¼�ɹ� */
    while (1) {
        if (mtws_thread_running == 0) {
            PRINT_INFO("mtws thread stop all. \n");
            break;
        }

        if (CloudLoginServer() == 0) {
            printf("login svr success\n");
            g_p2pClient.m_bLoginSvr = 1;
            break;
        } else {
            printf("login svr fail\n");
            sleep(5);
            continue;
        }
    }

    /* ��ȡ�豸id */
    ret = CloudGetDeviceId(devId);
    if(ret == 0) {
        //send_p2p_device_id_msg(devId);
        printf("CloudGetDeviceId success. ret = %d devId = %s\n", ret, devId);
        #if 0
        ret = read_mtws_p2p_id();
        if (ret != 0) {
        }
        #endif

        if (strcmp(runSystemCfg.deviceInfo.serialNumber, devId) != 0) {
            strncpy(runSystemCfg.deviceInfo.serialNumber, devId, sizeof(runSystemCfg.deviceInfo.serialNumber));
        }
    } else {
        printf("CloudGetDeviceId fail. ret = %d devId = %s\n", ret, devId);
    }

    is_get_mtws_p2p_id = 1; //1���Ѿ���ȡdevice id
    printf("EEEEXIT %s %d\n",__FUNCTION__,__LINE__);
    return 0;
}


static void * mtws_p2p_thread(void *para)
{
    SetThreadName("mtws_p2p_thread");

	PRINT_INFO("=============TUTK P2PName_add! begin========\n");
	P2PName_add(MTD_TUTK_P2P);
    PRINT_INFO("=============TUTK P2PName_add! end========\n");

    mtws_p2p_func();

    PRINT_INFO("mtws_p2p_thread end !\n");
    return NULL;
}



static int mtws_connect_ssid(int sock, int size)
{
    PRINT_INFO("mtws_connect_ssid start !\n");
    int ret;
    char ssid[100] = {0};
    char passwd[100] = {0};

    char *content = (char *)malloc(size);
    if (content == NULL) {
        PRINT_ERR("malloc error.\n");
        return -1;
    }

    ret = recv(sock, content, size, 0);
    if (ret < 0) {
        PRINT_ERR("recv content error.");
        return -1;
    }
    sscanf(content, "SSID=%[^&]&passwd=%s", ssid, passwd);
    PRINT_INFO("ssid:%s passwd:%s.\n", ssid, passwd);

#if 1
    //FILE *fp = fopen("/tmp/ssid.conf", "wb+");
    FILE *fp = fopen(SSID_CONF, "wb+");
    if (NULL == fp) {
        PRINT_ERR("fopen error.\n");
        free(content);
        return -1;
    }

    fprintf(fp, "ap_scan=1\n\n\n");
    fprintf(fp, "network={\n");
    fprintf(fp, "    ssid=\"%s\"\n", ssid);
    fprintf(fp, "    psk=\"%s\"\n", passwd);
    fprintf(fp, "}\n\n");

    fclose(fp);
#endif

#if 1
    //ret = netcam_wifi_ap_to_sta();
    ret = netcam_net_wifi_switchWorkMode(NETCAM_WIFI_STA);
    if (ret < 0) {
        PRINT_ERR("netcam_wifi_ap_to_sta error.\n");
        free(content);
        netcam_exit(90);
        return -1;
    } else {
        PRINT_INFO("netcam_wifi_ap_to_sta success.\n");
    }

#endif

    free(content);
    is_sta = 1;

    #if 0
    /* �ȴ���ȡ�豸ID */
    while (1) {
        if (is_device_id == 1)
            break;
        else {
            sleep(1);
            continue;
        }
    }

    /* �����豸ID */
    PRINT_INFO("to send devId: %s.\n", devId);
    ret = send(sock, devId, strlen(devId), 0);
    if (ret < 0) {
        PRINT_ERR("send devId error.");
        return -1;
    }
    #endif

    return 0;
}

static int MtwsLocalSessionLoop(int sock)
{
    int ret = 0;

    MTWS_LOCAL_MSG_HEAD msg_head = {0};
    ret = recv(sock, (char *)&msg_head, sizeof(MTWS_LOCAL_MSG_HEAD), 0);
    if (ret != sizeof(MTWS_LOCAL_MSG_HEAD)) {
        PRINT_ERR();
        close(sock);
        return -1;
    }

    switch (msg_head.cmd) {
        case CMD_CONNECT_SSID:
            PRINT_INFO("sock:%d, CMD_CONNECT_SSID\n", sock);
            //mtws_connect_ssid(sock, msg_head.size);
            break;

        default:
            PRINT_ERR("sock:%d, cmd error: %d\n", sock, msg_head.cmd);
            break;
    }

    close(sock);
    return 0;
}


static void * MtwsLocalSessionThread(void *param)
{
    SetThreadName("MtwsLocalSessionThread");

    MtwsLocalSessionLoop((int)param);

    return NULL;
}


static int mtws_local_listen(int port)
{
    int ret, listenfd, connectfd;    //socket�ļ�������
    listenfd = CreateTcpSock(NULL, port);
	if (listenfd < 0) {
		PRINT_ERR("create listen socket fail");
		return -1;
	}

    //���óɷ�����,������
    ret = SetSockAttr(listenfd);
    if (ret < 0) {
        PRINT_ERR();
        return -1;
    }

    //ѭ���������ȴ�����
    struct sockaddr_in addr;
    int sin_size;  //��ַ��Ϣ�ṹ���С
    struct timeval timeout;
    fd_set readfd; //���ļ�����������
    while (1) {
        if (mtws_thread_running == 0) {
            PRINT_INFO("listen, g_cms_pthread_run is 0\n");
            break;
        }

        timeout.tv_sec = 3;  //��ʱʱ��Ϊ3��
        timeout.tv_usec = 0;
        //�ļ���������0
        FD_ZERO(&readfd);
        //���׽����ļ����������뵽�ļ�������������
        FD_SET(listenfd, &readfd);

        select(listenfd + 1, &readfd, NULL, NULL, &timeout);
        if (!FD_ISSET(listenfd, &readfd)) {
            usleep(100000);
            continue;
        }

        sin_size = sizeof(struct sockaddr_in);
        if ((connectfd = accept(listenfd, (struct sockaddr *)&addr, (socklen_t *) & sin_size)) == -1) {
            PRINT_ERR("Server: accept failed%s\n", strerror(errno));
            continue;
        }
        PRINT_INFO("accept session_socket:%d, remote ip:%s, remote port:%d\n",
                    connectfd, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));

        //ÿ��һ�����Ӿʹ���һ���Ự�߳�
        CreateDetachThread(MtwsLocalSessionThread, (void *)connectfd, NULL);

    }

    PRINT_INFO("GkListenLoop end\n");
    //�رշ���������socket
	if (listenfd > 0) {
    	close(listenfd);
		listenfd = -1;
	}

    return 0;
}


static void * mtws_local_listen_thread(void *para)
{
    SetThreadName("mtws_local_listen_thread");

    mtws_local_listen(MTWS_LOCAL_TCP_LISTEN_PORT);

    PRINT_INFO("mtws_local_listen_thread end !\n");
    return NULL;
}




/* ���������ֻ�APP P2P�Խ� */
int mtws_app_init()
{
	PRINT_INFO("mtws_app_init.\n");

    #if 0
    int ret = 0;
    /* ��ȡ�û�˽��������Ϣ: ap name��wifi mac */
    MTWS_CONFIG config;
    memset(&config, 0, sizeof(MTWS_CONFIG));
    int ret = load_mtws_config(&config);
    if (ret != 0) {
        PRINT_ERR("load_ds_config error. \n");
        netcam_exit(90);
    }
    #endif

    #if 0
    if (runNetworkCfg.wifi.mode == 0) {
        /* AP ģʽ����*/
        netcam_wifi_ap_start();
    } else {
        /* STA ģʽ����*/

        netcam_wifi_sta_start();
    }
    #endif


    #if 0
    /*�����STAģʽ���������ж������Ƿ�ͨ���������ͨ�����л�APģʽ������ */
    if (runNetworkCfg.wireless.mode == 1) {
        /* ���������Ƿ�ͨ�� */
        ret = netcam_ping_ip(TEST_IP);
        if (ret != 0) {
            /* ���粻ͨ���л�APģʽ�������� */
            runNetworkCfg.wifi.mode = 0;
            NetworkCfgSave();
            system("reboot");
        } else {
            /* ����ͨ���������һ�� */
        }
    }
    #endif



    /* ע�ᱨ���ص����� */
    PRINT_INFO("=============MTWS P2PName_add! begin========\n");
	P2PName_add(MTD_MTWS_P2P);
    PRINT_INFO("=============MTWS P2PName_add! end========\n");
    mtws_alarm_handle = event_alarm_open(mtws_alarm_cb);

	//ע��private smartlink
	if(netcam_net_private_smartlink())
    {
        PRINT_ERR("TUTK Register private smartlink fail.\n");
        return -1;
    }


    /* ��ʼ��ȫ�ֱ��� */
	memset(&g_p2pClient, 0, sizeof(P2P_CLIENT_T));
	g_p2pClient.m_evtCB = p2p_evt_process;
    g_p2pClient.m_talkCB = mtws_p2p_talk_cb;
    g_p2pClient.m_controlCB = mtws_system_control_cb;
    pthread_mutex_init(&g_p2pClient.m_mainMutex, NULL);
    pthread_mutex_init(&g_p2pClient.m_subMutex, NULL);
    pthread_mutex_init(&g_p2pClient.m_ConnectMutex, NULL);

    /* �����߳� */
	mtws_thread_running = 1;
	CreateDetachThread(mtws_stream0_thread, NULL, NULL);
    CreateDetachThread(mtws_stream1_thread, NULL, NULL);
    CreateDetachThread(mtws_p2p_thread, NULL, NULL);
    CreateDetachThread(mtws_local_listen_thread, NULL, NULL);

    printf("DDD %s,%d\n",__FUNCTION__,__LINE__);

    return 0;
}

void mtws_app_exit()
{
    PRINT_INFO("mtws_app_exit");
#if 0
    //pthread_mutex_destroy();
    pthread_mutex_destroy(&g_p2pClient.m_mainMutex);
    pthread_mutex_destroy(&g_p2pClient.m_subMutex);
    pthread_mutex_destroy(&g_p2pClient.m_ConnectMutex);
#endif

    mtws_thread_running = 0;
    CloudSdkRelease();

    event_alarm_close(mtws_alarm_handle);
}

#ifdef __cplusplus
}
#endif





