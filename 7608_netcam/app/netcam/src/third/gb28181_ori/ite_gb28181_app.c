#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ite_sip_api.h>
//#include <ite_uac.h>
#include <ite_uas.h>
#include <arpa/inet.h>
//#include <ite_sip_interface.h>
#include <eXosip2/eXosip.h>
#include "media_fifo.h"
#include "ite_gb28181_playback.h"
#include "cfg_gb28181.h"
#include "flash_ctl.h"
#include "ipcsearch.h"
#include "eventalarm.h"
#include "cfg_alarm.h"
#include "cfg_network.h"

OS_S32 g_bterminate = ITE_OSAL_FALSE;
int g_alarmMsgId = 0;
extern int g_register_status;

#if 1
char serverIp[50] = "182.145.195.235";   //gateway.mj.sctel.com.cn  192.168.10.186
char gDeviceId[50] = "08101021122A3A00000"; //08101021122A3A00000 34020000001320000001
#else
char serverIp[50] = "gateway.mj.sctel.com.cn";
char gDeviceId[50] = "08101021122A3A00000";
#endif

int serverPort = 5060;
static unsigned int lastAlarmTime = 0;
static unsigned int lastAreaAlarmTime = 0;

extern int rtpInit(void);
extern int uac_init(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig);
extern int uac_UNregister(struct eXosip_t *excontext,IPNC_SipIpcConfig * SipConfig);

/* ============================================================================================= */
static void gb28181_handle_signal(OS_S32 signum)
{
    gb28181_msg_queue_t alarmMsg;
    memset(&alarmMsg, 0, sizeof(alarmMsg));
    printf("recevice signal:%d\r\n", signum);
    switch (signum)
    {
        case SIGHUP:
            g_bterminate = ITE_OSAL_FALSE;
            break;
        case SIGTERM:
        case SIGINT:
            g_bterminate = ITE_OSAL_TRUE;
            break;
        case 35://SIGRTMIN+1
            alarmMsg.mtype = GB_MSG_ALARM_TYPE;
            alarmMsg.mmsg.method = 1; //为报警通道报警
            break;
        case 36://SIGRTMIN+2            
            alarmMsg.mtype = GB_MSG_ALARM_TYPE;
            alarmMsg.mmsg.method = 2; //为设备通道报警
            break;
        default:
            break;
    }
    printf("g_recvalarm :%d\n", alarmMsg.mmsg.method);
    if(alarmMsg.mtype == GB_MSG_ALARM_TYPE)
    {
        struct timezone Ztest;
        struct tm timeNow;
        struct timeval systime;
        gettimeofday(&systime,&Ztest);
        localtime_r(&systime.tv_sec, &timeNow);
        sprintf(alarmMsg.mmsg.time, "%04d-%02d-%02dT%02d:%02d:%02d",
                                                timeNow.tm_year + 1900, timeNow.tm_mon + 1, timeNow.tm_mday,
                                                timeNow.tm_hour, timeNow.tm_min, timeNow.tm_sec);
        if(msgsnd(g_alarmMsgId, (void*)&alarmMsg, sizeof(alarmMsg)- sizeof(alarmMsg.mtype), 0) < 0)
        {
            perror("msgsnd");
            printf("[%s][%d]msgsnd g_alarmMsgId failed.\n", __func__, __LINE__);
        }
    }
    return;
}

/* ============================================================================================= */
void gb28181_init_signal(void)
{
    struct sigaction sigaInst;
    sigaInst.sa_flags = 0;
    sigemptyset(&sigaInst.sa_mask);

    sigaddset(&sigaInst.sa_mask, SIGTERM);
    sigaddset(&sigaInst.sa_mask, SIGINT);
    sigaddset(&sigaInst.sa_mask, SIGHUP);
    sigaddset(&sigaInst.sa_mask, SIGRTMIN+1);//自定义实时信号
    sigaddset(&sigaInst.sa_mask, SIGRTMIN+2);

    sigaInst.sa_handler = gb28181_handle_signal;
    sigaction(SIGTERM, &sigaInst, NULL);
    sigaction(SIGINT, &sigaInst, NULL);
    sigaction(SIGHUP, &sigaInst, NULL);
    sigaction(SIGRTMIN+1, &sigaInst, NULL);
    sigaction(SIGRTMIN+2, &sigaInst, NULL);
}


int  UAC_TryConnectServer(IPNC_SipIpcConfig * ipnc_conf)
{
    int onLineRet = -1;
    int sockfd = -1;
    struct timeval timeo = {1, 0};
    struct sockaddr_in serv_addr;
    socklen_t socklen = sizeof(timeo);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd < 0)
    {
        sleep(1);
        onLineRet = -1;
        return onLineRet;
    }
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeo, socklen);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr  = inet_addr(ipnc_conf->sipRegServerIP);
    serv_addr.sin_port = htons (ipnc_conf->sipRegServerPort);
    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        onLineRet = -2;
    }
    else
    {
        onLineRet = 0;
    }
    close(sockfd);
    printf("UAC_TryConnect [%s][%d] onLineRet[%d]  [offLine<0)]\n", ipnc_conf->sipRegServerIP, ipnc_conf->sipRegServerPort, onLineRet);
    return onLineRet;
}

static int gb28181_alarm_cb_func(int nChannel, int nAlarmType, int nAction, void* pParam)
{
    gb28181_msg_queue_t alarmMsg;
    memset(&alarmMsg, 0, sizeof(alarmMsg));
    printf("gb28181_alarm_cb_func: Type = %d\n", nAlarmType);
	switch(nAlarmType)
    {
        case GK_ALARM_TYPE_ALARMIN:     //0:信号量报警开始
			break;

        case GK_ALARM_TYPE_DISK_FULL:       //1:硬盘满
			break;

        case GK_ALARM_TYPE_VLOST:            //2:信号丢失
			break;

        case GK_ALARM_TYPE_VMOTION:          //3:移动侦测
        {
            if (runAlarmCfg.alarmInfo[0].gbStatus == 1)
            {
                alarmMsg.mtype = GB_MSG_ALARM_TYPE;
                alarmMsg.mmsg.method = 1;
                alarmMsg.mmsg.type   = 2;
            }

#ifdef MODULE_SUPPORT_MOJING

			time_t time_stamp;
		    struct tm  *tv;

		    time_stamp = time(NULL);
		    tv = localtime(&time_stamp);
				
			time_stamp = time(NULL);
			if(tv->tm_year+1900  < 2020)
			{
                printf("####mojing_get_cloud_storage_addr_request: %04d%02d%02d %02d:%02d:%02d, wait time sync!\n", tv->tm_year+1900,tv->tm_mon+1,
					tv->tm_mday,tv->tm_hour,tv->tm_min,tv->tm_sec);
				break;
			}
			
			mojing_get_cloud_storage_addr_request(CLOUD_STORAGE_DYNAMIC);
#endif
            break;
        }
        case GK_ALARM_TYPE_DISK_UNFORMAT:    //4:硬盘未格式化
			break;

        case GK_ALARM_TYPE_DISK_RWERR:       //5:读写硬盘出错,
			break;

        case GK_ALARM_TYPE_VSHELTER:         //6:遮挡报警
			break;

        case GK_ALARM_TYPE_ALARMIN_RESUME:         //9:信号量报警恢复
			break;

        case GK_ALARM_TYPE_VLOST_RESUME:         //10:视频丢失报警恢复
			break;

        case GK_ALARM_TYPE_VMOTION_RESUME:         //11:视频移动侦测报警恢复
			break;

        case GK_ALARM_TYPE_NET_BROKEN:       //12:网络断开
			break;

        case GK_ALARM_TYPE_IP_CONFLICT:      //13:IP冲突
			break;

        case GK_ALARM_TYPE_AREA_ALARM:
        {
            if (runAlarmCfg.alarmInfo[1].gbStatus == 1)
            {
                alarmMsg.mtype = GB_MSG_ALARM_TYPE;
                alarmMsg.mmsg.method = 5;
                alarmMsg.mmsg.type   = 6;
                if(pParam)
                    strcpy(alarmMsg.mmsg.para, pParam);
            }
            break;
        }
        case GK_ALARM_TYPE_UPGRADE_ALARM:
        {            
            alarmMsg.mtype = GB_MSG_ALARM_TYPE;
            alarmMsg.mmsg.method = 2;
            alarmMsg.mmsg.type   = 6;
            if(pParam)
                strcpy(alarmMsg.mmsg.para, pParam);
            break;
        }
        case GK_ALARM_TYPE_UPLOAD_ALARM:
        {            
            alarmMsg.mtype = GB_MSG_ALARM_TYPE;
            alarmMsg.mmsg.method = 2;
            alarmMsg.mmsg.type   = 10;
            if(pParam)
                strcpy(alarmMsg.mmsg.para, pParam);
            break;
        }

        default:
			PRINT_INFO("Alarm: Type = %d", nAlarmType);
        	break;
    }
    if(alarmMsg.mtype == GB_MSG_ALARM_TYPE)
    {
        struct timezone Ztest;
        struct tm timeNow;
        struct timeval systime;
        gettimeofday(&systime,&Ztest);
        localtime_r(&systime.tv_sec, &timeNow);
        sprintf(alarmMsg.mmsg.time, "%04d-%02d-%02dT%02d:%02d:%02d",
                                                timeNow.tm_year + 1900, timeNow.tm_mon + 1, timeNow.tm_mday,
                                                timeNow.tm_hour, timeNow.tm_min, timeNow.tm_sec);
        if(msgsnd(g_alarmMsgId, (void*)&alarmMsg, sizeof(alarmMsg)- sizeof(alarmMsg.mtype), 0) < 0)
        {
            perror("msgsnd");
            printf("[%s][%d]msgsnd g_alarmMsgId failed.\n", __func__, __LINE__);
        }
    }

	return 0;
}

iConfigInfo sys_conf;
ite_gb28181Obj sip_gb28281Obj = {NULL, NULL};

int rtp_get_data_init(unsigned int streamId)
{
    MEDIABUF_HANDLE handle = mediabuf_add_reader(streamId);
    if(handle)
        mediabuf_set_newest_frame(handle);
    else
    {
        printf("add reader error.\n");
    }
    return (int)handle;
}

int rtp_get_data(int handle, void **data, int *size, int *type, unsigned int *pts)
{
    int readLen = 0;
    GK_NET_FRAME_HEADER header = {0};

    readLen = mediabuf_read_frame((MEDIABUF_HANDLE)handle, data, size, &header);
    if((readLen > 0))
    {
        *type = header.frame_type;
        *pts = header.pts;
    }
    else
    {
        return 0;
    }
    return 1;
}

int rtp_get_data_close(int handle)
{
    mediabuf_del_reader((MEDIABUF_HANDLE)handle);
    return 0;
}

int gb28181_load_id_check(char *id)
{
    int i; 
    int len = strlen(id);
    for (i = 0; i < len; i++)
    {
        if ((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'A' && id[i] <= 'F') || (id[i] >= 'a' && id[i] <= 'f'))
        {
            continue;
        }
        else
        {
            return 1;
        }
    }
    return 0;
}

void gb28181_set_sip_info(void)
{
    #if 0
	strcpy(sys_conf.SipIpcParam.sipRegServerIP, "182.145.195.243");
	sys_conf.SipIpcParam.sipRegServerPort = 30000;
    #else
	strcpy(sys_conf.SipIpcParam.sipRegServerIP, runGB28181Cfg.ServerIP);
	sys_conf.SipIpcParam.sipRegServerPort = runGB28181Cfg.ServerPort;
    #endif
	strcpy(sys_conf.SipIpcParam.sipRegServerUID, runGB28181Cfg.ServerUID);
	strcpy(sys_conf.SipIpcParam.sipRegServerPwd, runGB28181Cfg.ServerPwd);
	strcpy(sys_conf.SipIpcParam.sipRegServerDomain, runGB28181Cfg.ServerDomain);
	sys_conf.SipIpcParam.sipDevicePort = runGB28181Cfg.DevicePort;
	strcpy(sys_conf.SipIpcParam.sipDeviceUID, runGB28181Cfg.DeviceUID);
	sys_conf.SipIpcParam.regExpire_s = runGB28181Cfg.Expire;
	sys_conf.SipIpcParam.sipDevHBCycle_s = runGB28181Cfg.DevHBCycle;
	sys_conf.SipIpcParam.sipDevHBOutTimes = runGB28181Cfg.DevHBOutTimes;
	strcpy(sys_conf.SipIpcParam.sipAlarmID, runGB28181Cfg.AlarmID);//daiding
	sys_conf.SipIpcParam.bRedirect = 1;
	strcpy(sys_conf.SipIpcParam.sipDevGBSN, runGB28181Cfg.DeviceUID);
    strcpy(sys_conf.SipIpcParam.sipDevMac, runNetworkCfg.lan.mac);
    struct device_info_mtd gb28181_info;
    int ret = load_info_to_mtd_reserve(MTD_GB28181_INFO,&gb28181_info,sizeof(struct device_info_mtd));
    if(ret !=0)
    {
        PRINT_ERR("[p2p] get p2p error: type[MTD_GB28181_INFO].\n");
        strcpy(sys_conf.SipIpcParam.sipDevCtei, "123456789012345");
    }
    else
    {
        PRINT_INFO("[p2p] get p2p info: type[MTD_GB28181_INFO] ID[%s].\n",gb28181_info.device_id);
        strcpy(sys_conf.SipIpcParam.sipDevCtei, gb28181_info.device_id);
    }
    g_register_status = 0;
}

void gb28181_load_id(void)
{
    struct device_info_mtd yunni_info;
    memset(&yunni_info,0,sizeof(struct device_info_mtd));
    int ret = load_info_to_mtd_reserve(MTD_TUTK_P2P, &yunni_info, sizeof(struct device_info_mtd));
    if((ret < 0) || (yunni_info.device_id[0] == 0))
    {
        printf("Fail to load gb28181 id from flash!\n");
    }
    else
    {
        printf("load gb28181 id:%s\n", yunni_info.device_id);
        if (strcmp(runGB28181Cfg.DeviceUID, yunni_info.device_id) != 0)
        {
            if (strcmp(yunni_info.device_id, "000000000") == 0 || strcmp(yunni_info.device_id, "0000000000000000000") == 0)
            {
                printf("recover id from %s to %s\n", yunni_info.device_id, runGB28181Cfg.DeviceUID);
                //保存的id为0，但是魔镜id不为0，说明flash存储的id丢失，以魔镜id为准
                strcpy(yunni_info.device_id, runGB28181Cfg.DeviceUID);                
                save_info_to_mtd_reserve(MTD_TUTK_P2P, &yunni_info, sizeof(yunni_info));
            }
            else
            {
                strcpy(runGB28181Cfg.DeviceUID, yunni_info.device_id);
                GB28181CfgSave();
            }
        }
    }
}

int gb28181_msg_queue_init(int proj_id)
{
	int ret = -1;
    
    key_t key = ftok("/tmp", proj_id);
    if(key < 0)
    {
        perror("ftok");
        return -1;
    }

    g_alarmMsgId = msgget(key, IPC_CREAT|0666);
    if(g_alarmMsgId < 0)
    {
        perror("msgget");
        printf("[%s][%d]msgget failed.\n", __func__, __LINE__);
    	return -1;
    }
    return 0;
}

void* gb28181_start_thread(void *pData)
{
    pthread_t pthid[5];

    /*
    char send_sdp_data[1024];
    char recieve_sdp_data[1024];
    char rtsp_data[1024];
    char EOF_message[1024];
    struct st_rtsptype rtsptype;
    */
    gb28181_msg_queue_init(0x88);

    rtpInit();
    memset(&sys_conf, 0, sizeof(sys_conf));

	sys_conf.SipIpcParam.bUseSipDevice = 1;

#if 0
	sys_conf.SipIpcParam.regExpire_s = 3600;
	strcpy(sys_conf.SipIpcParam.sipAlarmID, "1");//daiding
	sys_conf.SipIpcParam.sipDevHBCycle_s = 30;
	sys_conf.SipIpcParam.sipDevHBOutTimes = 3;
	sys_conf.SipIpcParam.sipDevicePort = 5060;
	strcpy(sys_conf.SipIpcParam.sipDeviceUID, gDeviceId);//34020000001320000001  08101021122A3A00000
	strcpy(sys_conf.SipIpcParam.sipRegServerDomain, "34020000");
	strcpy(sys_conf.SipIpcParam.sipRegServerIP, serverIp);
	sys_conf.SipIpcParam.sipRegServerPort = serverPort;
	strcpy(sys_conf.SipIpcParam.sipRegServerPwd, "2KE496r8a");
	strcpy(sys_conf.SipIpcParam.sipRegServerUID, "51000000002006000002");
#else

	if (!strcmp(runGB28181Cfg.DeviceUID,"0000000000000000000") || (strstr(runGB28181Cfg.DeviceUID, "00") == NULL))
	{
        gb28181_load_id();
	}

    gb28181_set_sip_info();
#endif
   // int UAS_ONLINE = 0, tryTimes = 0;

    //gb28181_init_signal();
    //ite_gb28181_init_comm((OS_HANDLE )&sys_conf);
    sip_gb28281Obj.excontext = (struct eXosip_t *)eXosip_malloc();
    sip_gb28281Obj.sipconfig  = &(sys_conf.SipIpcParam);

    g_bterminate = ITE_OSAL_FALSE;
    //printf("sip_gb28281Obj [%p  %ld    %p]\n", sip_gb28281Obj.excontext, sizeof(struct eXosip_t) , sip_gb28281Obj.sipconfig);

    uac_init(sip_gb28281Obj.excontext, sip_gb28281Obj.sipconfig);

    printf("sip_gb28281Obj [%p    %p]\n", sip_gb28281Obj.excontext,  sip_gb28281Obj.sipconfig);

    if (0 != pthread_create(&pthid[0], NULL, ite_eXosip_register_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_register_Task is failed\n");
        return (void*)-1;
    }
    sleep(2);
    printf("******ite_eXosip_Kalive_Task******\r\n");
    if (0 != pthread_create(&pthid[1], NULL, ite_eXosip_Kalive_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_KaliveTask is failed\n");
        return (void*)-1;
    }
    printf("******ite_eXosip_processEvent_Task******\r\n");
    if (0 != pthread_create(&pthid[2], NULL, ite_eXosip_processEvent_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_processEvent_Task is failed\n");
        return (void*)-1;
    }
    printf("******ite_eXosip_NotifyAlarm_Task******\r\n");
    if (0 != pthread_create(&pthid[3], NULL, ite_eXosip_NotifyAlarm_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_NotifyAlarm_Task is failed\n");
        return (void*)-1;
    }
    printf("******ite_eXosip_ftp_upload_log_Task******\r\n");
    if (0 != pthread_create(&pthid[4], NULL, ite_eXosip_ftp_upload_log_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_NotifyAlarm_Task is failed\n");
        return (void*)-1;
    }
#if 0
    printf("******ite_eXosip_PositionAlert_Task******\r\n");
    if (0 != pthread_create(&pthid[4], NULL, ite_eXosip_PositionAlert_Task, (void*) &sip_gb28281Obj))
    {
        printf("pthread_create ite_eXosip_PositionAlert_Task is failed\n");
        return (void*)-1;
    }
#endif
    printf("******ite_logUploadParam_init******\r\n");
    if (0 != ite_logUploadParam_init((void*) &sip_gb28281Obj))
    {
        printf("ite_logUploadParam_init is failed\n");
        return (void*)-1;
    }

	gk_gb28181_playback_init();


	P2PName_add(MTD_TUTK_P2P);


	void *mojing_alarm_handle = NULL;
    mojing_alarm_handle = event_alarm_open(gb28181_alarm_cb_func);
#if 1

    while (g_bterminate == ITE_OSAL_FALSE)
    {
        if(sys_conf.SipIpcParam.bUseSipDevice == 0)
        {
            uac_UNregister(sip_gb28281Obj.excontext, sip_gb28281Obj.sipconfig);
            g_bterminate = ITE_OSAL_TRUE;
        }
        sleep(2);

    }
    sleep(2);
#endif
  //  goto bEnStart;
    printf("gb28181_start end\n");
    event_alarm_close(mojing_alarm_handle);
    return 0;
}

int gb28181_start(void)
{
	//P2PName_add(MTD_GB28181_INFO);

    pthread_t pthid;
    pthread_create(&pthid, NULL, gb28181_start_thread, NULL);
    return 0;
}

int gb28181_close(void)
{
	g_bterminate = ITE_OSAL_TRUE;
}

int gb28181_get_reg_serverip_port(char *ip, int *port)
{
	if(!ip || !port)
		return -1;

	strcpy(ip,sys_conf.SipIpcParam.sipRegServerIP);
	*port = sys_conf.SipIpcParam.sipRegServerPort;
}

