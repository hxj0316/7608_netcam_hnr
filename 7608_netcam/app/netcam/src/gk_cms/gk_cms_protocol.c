/*!
*****************************************************************************
** FileName     : gk_cms_protocol.c
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
#include "gk_cms_common.h"
#include "gk_cms_media.h"
#include "gk_cms_sock.h"
#include "gk_cms_protocol.h"
#include "gk_cms_utility.h"
#include "gk_cms_net_api.h"
#include "gk_cms_pb.h"
#include "media_fifo.h"
#include "cfg_all.h"
#include "netcam_api.h"
#include "flash_ctl.h"
#ifdef MODULE_SUPPORT_RF433
#include "rf433.h"
#endif

static int g_session_pthread_run = 0;
static char g_session_reques_i_freame[2] = {0};


int Gk_CmsNotify(int sock, unsigned long type, void *data, int data_size)
{
    char *buf = malloc(data_size + sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG));
    if (buf == NULL) {
        PRINT_ERR("Gk_CmsNotify malloc error\n");
        return -1;
    }

    JB_NET_PACK_HEAD *cmd_head = (JB_NET_PACK_HEAD *)buf;
    cmd_head->nFlag = 9000;
    cmd_head->nCommand = GK_NET_CMD_NOTIFY;
    cmd_head->nChannel = 0;
    cmd_head->nErrorCode = 0;
    cmd_head->nBufSize = sizeof(JB_SERVER_MSG) + data_size;

    JB_SERVER_MSG *msg_head = (JB_SERVER_MSG *)(buf + sizeof(JB_NET_PACK_HEAD));
    msg_head->dwMsg = type;
    msg_head->dwChannel = 0;
    msg_head->cbSize = data_size;

    long ts = time(NULL);
    struct tm tt = {0};
    struct tm *pTm = localtime_r(&ts, &tt);
    struct timeval time_tmp;
    gettimeofday(&time_tmp, NULL);

    msg_head->st.wYear = pTm->tm_year + 1900;
    msg_head->st.wMonth = pTm->tm_mon + 1;
    msg_head->st.wDay = pTm->tm_mday;
    msg_head->st.wHour = pTm->tm_hour;
    msg_head->st.wMinute = pTm->tm_min;
    msg_head->st.wSecond = pTm->tm_sec;
    msg_head->st.wDayOfWeek = pTm->tm_wday;
    msg_head->st.wMilliseconds = time_tmp.tv_usec/1000;

    char *msg = (char *)(buf  + sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG));
    if ((data != NULL) && (data_size != 0))
        memcpy(msg, data, data_size);

    int ret = GkSockSend(sock, buf, sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG) + data_size, 100);
    if (ret != (sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG) + data_size)) {
        PRINT_ERR("send:%d size:%d", ret, sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG) + data_size);
        free(buf);
        return -1;
    } else {
        //PRINT_INFO("send %d data_size:%d, head_size:%d\n", ret, data_size, sizeof(JB_NET_PACK_HEAD) + sizeof(JB_SERVER_MSG));
    }

    free(buf);
    return 0;
}

int cms_notify_to_all_client(unsigned long type)
{
    int i = 0;
    int sock = 0;
    int ret = 0;
    for (i = 0; i < GK_MAX_LINK_NUM; i++) {
        sock = GkGetMainSockByIndex(i);
        if (sock > 0) {
            ret = Gk_CmsNotify(sock, type, NULL, 0);
            if (ret < 0) {
                GkCloseMainSockByIndex(i);
            }
        }
    }
    return 0;
}


int GkCmsCmdResq(int sock, void *body, int body_size, int cmd)
{
    char *buf = malloc(body_size + sizeof(JB_NET_PACK_HEAD));
    if (buf == NULL) {
        PRINT_ERR("malloc failed\n");
        return -1;
    }

    JB_NET_PACK_HEAD *cmd_head = (JB_NET_PACK_HEAD *)buf;
    cmd_head->nFlag = 9000;
    cmd_head->nCommand = cmd;
    cmd_head->nChannel = 0;
    cmd_head->nErrorCode = 0;
    cmd_head->nBufSize = body_size;
    memcpy(buf + sizeof(JB_NET_PACK_HEAD), body, body_size);

    int ret = GkSockSend(sock, buf, sizeof(JB_NET_PACK_HEAD) + body_size, 100);
    if (ret != (sizeof(JB_NET_PACK_HEAD) + body_size)) {
        PRINT_ERR("send:%d size:%d error", ret, sizeof(JB_NET_PACK_HEAD) + body_size);
        free(buf);
        return -1;
    } else {
        //PRINT_INFO("send %d body_size:%d, head_size:%d\n", ret, body_size, sizeof(JB_NET_PACK_HEAD));
    }

    free(buf);
    return 0;
}

int GkCmsCmdResqByIndex(int sock, void *body, int body_size, int cmd, int stream_no)
{
    char *buf = malloc(body_size + sizeof(JB_NET_PACK_HEAD));
    if (buf == NULL) {
        PRINT_ERR("malloc failed\n");
        return -1;
    }

    JB_NET_PACK_HEAD *cmd_head = (JB_NET_PACK_HEAD *)buf;
    cmd_head->nFlag = 9000;
    cmd_head->nCommand = cmd;
    cmd_head->nChannel = stream_no;
    cmd_head->nErrorCode = 0;
    cmd_head->nBufSize = body_size;
    memcpy(buf + sizeof(JB_NET_PACK_HEAD), body, body_size);

    int ret = GkSockSend(sock, buf, sizeof(JB_NET_PACK_HEAD) + body_size, 100);
    if (ret != (sizeof(JB_NET_PACK_HEAD) + body_size)) {
        PRINT_ERR("send:%d size:%d", ret, sizeof(JB_NET_PACK_HEAD) + body_size);
        free(buf);
        return -1;
    } else {
        PRINT_INFO("send %d body_size:%d, head_size:%d\n", ret, body_size, sizeof(JB_NET_PACK_HEAD));
    }

    free(buf);
    return 0;
}

int GkHandleMediaRequest(int sock)
{
    OPEN_HEAD open_head = {0};
	int ret = GkSockRecv(sock, (char *)&open_head, sizeof(OPEN_HEAD));
    if (ret != sizeof(OPEN_HEAD)) {
        PRINT_ERR("recv %d not equal %d\n", ret, sizeof(OPEN_HEAD));
        return -1;
    }

    GK_CMS_STREAM_TYPE type = GK_UNKNOWN;
    PRINT_INFO("session id : %lu\n", open_head.nID);
    if (open_head.nStreamType == 0) {
        type = GK_MAIN_STREAM;
    } else if (open_head.nStreamType == 1) {
        type = GK_SUB_STREAM;
    } else {
        PRINT_ERR("request media type error.\n");
        return -1;
    }

	/* ����ǰsocket��ӽ����飬������ý�������� */
	ret = GkAddLvSock(sock, open_head.nID, type);
	if (ret == -1)  {// �����Ҳ�����Ӧ��session_id
		PRINT_ERR();
        JB_NET_PACK_HEAD req = {0};
        req.nCommand = NETCMD_NOT_LOGON;
        GkSockSend(sock, (char *)&req, sizeof(JB_NET_PACK_HEAD), 100);
		return -1;
    }

    GkSetLvSockStatus(open_head.nID, 1, type);

    return 0;
}


/*************************************************************
* �������ܣ���¼
* ���������
* ���������
* ����ֵ  ��
* ��ע    :
*************************************************************/
int Gk_LogOn(int sock)
{
    USER_INFO user_info;
    struct sockaddr_in name;
    socklen_t nameLen;
    int ret = GkSockRecv(sock, (char *)&user_info, sizeof(USER_INFO));
    if (ret != sizeof(USER_INFO)) {
        PRINT_ERR("logon, recv error. recv:%d, size:%d\n", ret, sizeof(USER_INFO));
        return -1;
    }

    char send_msg[GK_CMS_BUFLEN] = {0};
    JB_NET_PACK_HEAD *cmd_head = (JB_NET_PACK_HEAD *)send_msg;
    cmd_head->nFlag = JBNV_FLAG;
    cmd_head->nCommand = JBCMD_SERVER_INFO;
    cmd_head->nChannel = 0;
    cmd_head->nErrorCode = 0;
    cmd_head->nBufSize = sizeof(CMD_SERVER_INFO) + sizeof(JBNV_CHANNEL_INFO);
    CMD_SERVER_INFO *server_info = (CMD_SERVER_INFO *) (send_msg + sizeof(JB_NET_PACK_HEAD));

    //�û���������֤
    //PRINT_INFO("login, user:%s password:%s mac:%s\n", user_info.szUserName, user_info.szUserPsw, user_info.MacCheck);
    PRINT_INFO("login, user:%s password:%s\n", user_info.szUserName, user_info.szUserPsw);
    GK_NET_USER_INFO user_check = {0};
    strncpy(user_check.userName, user_info.szUserName, sizeof(user_check.userName));
    strncpy(user_check.password, user_info.szUserPsw, sizeof(user_check.password));
    ret = UserCheck(&user_check);

    if (ret <0) {
        PRINT_INFO("user:%s password:%s is not match.\n", user_info.szUserName, user_info.szUserPsw);
        cmd_head->nCommand = JBCMD_PASSWORD_UNMATCH;
        server_info->dwServerIndex = 0;
        goto err;
    }

    //����sessionid
    int fSessionInt = GenSessionInt();
    //���Ự��socket���� gGkSessionGlobal ����
    ret = GkAddMainSock(sock, fSessionInt);
    if (ret == -1) { // ���Ӵﵽ�����������-1
        PRINT_INFO("AddMainSock��array is full.\n");
        cmd_head->nCommand = JBCMD_MAX_LINK;
        server_info->dwServerIndex = 0;
        goto err;
    } else {
        server_info->dwServerIndex = fSessionInt;
        PRINT_INFO("fSessionInt is %d.\n", fSessionInt);
    }


    PRINT_INFO("multiip:%s\n", GK_CMS_MC_GROUP_IP);
    server_info->dwMultiAddr = ip_to_dw(GK_CMS_MC_GROUP_IP);

    server_info->ServerInfo.dwSize = sizeof(JBNV_SERVER_INFO);

    server_info->ServerInfo.dwServerFlag = IPC_FLAG;
    nameLen = sizeof(name);
    if(getsockname(sock,(struct sockaddr *)&name,&nameLen) != 0)
    {
        PRINT_INFO("getsockname failed,use cfg file\n");
        strncpy(server_info->ServerInfo.szServerIp, runNetworkCfg.lan.ip, sizeof(runNetworkCfg.lan.ip));
        PRINT_INFO("ip:%s\n", runNetworkCfg.lan.ip);
        server_info->ServerInfo.dwServerIp = ip_to_dw(runNetworkCfg.lan.ip);
        PRINT_INFO("mac:%s\n", runNetworkCfg.lan.mac);
        mac_to_dw(server_info->ServerInfo.byMACAddr, runNetworkCfg.lan.mac);
        PRINT_INFO("mac: %x:%x:%x:%x:%x:%x \n", server_info->ServerInfo.byMACAddr[0], server_info->ServerInfo.byMACAddr[1],
                                               server_info->ServerInfo.byMACAddr[2], server_info->ServerInfo.byMACAddr[3],
                                               server_info->ServerInfo.byMACAddr[4], server_info->ServerInfo.byMACAddr[5]);
    }
    else
    {
        strncpy(server_info->ServerInfo.szServerIp, inet_ntoa(name.sin_addr),16);
        PRINT_INFO("ip:%s\n", server_info->ServerInfo.szServerIp);
        server_info->ServerInfo.dwServerIp = ip_to_dw(server_info->ServerInfo.szServerIp);
        if(netcam_net_get_detect("eth0") == 0)
        {
            PRINT_INFO("mac:%s\n", runNetworkCfg.lan.mac);
            mac_to_dw(server_info->ServerInfo.byMACAddr, runNetworkCfg.lan.mac);
            PRINT_INFO("mac: %x:%x:%x:%x:%x:%x \n", server_info->ServerInfo.byMACAddr[0], server_info->ServerInfo.byMACAddr[1],
                                                   server_info->ServerInfo.byMACAddr[2], server_info->ServerInfo.byMACAddr[3],
                                                   server_info->ServerInfo.byMACAddr[4], server_info->ServerInfo.byMACAddr[5]);
        }
        else
        {
            PRINT_INFO("mac:%s\n", runNetworkCfg.wifi.mac);
            mac_to_dw(server_info->ServerInfo.byMACAddr, runNetworkCfg.wifi.mac);
            PRINT_INFO("mac: %x:%x:%x:%x:%x:%x \n", server_info->ServerInfo.byMACAddr[0], server_info->ServerInfo.byMACAddr[1],
                                                   server_info->ServerInfo.byMACAddr[2], server_info->ServerInfo.byMACAddr[3],
                                                   server_info->ServerInfo.byMACAddr[4], server_info->ServerInfo.byMACAddr[5]);

        }
    }
    server_info->ServerInfo.wServerPort = GK_CMS_TCP_LISTEN_PORT;
    server_info->ServerInfo.wChannelNum = 1;

    server_info->ServerInfo.dwVersionNum = 1;

    strncpy(server_info->ServerInfo.szServerName, runSystemCfg.deviceInfo.deviceName, sizeof(runSystemCfg.deviceInfo.deviceName));

    server_info->ServerInfo.dwServerCPUType = 1;

    strncpy((char *)server_info->ServerInfo.bServerSerial, runSystemCfg.deviceInfo.serialNumber, sizeof(runSystemCfg.deviceInfo.serialNumber));


    server_info->ServerInfo.dwAlarmInCount = ALARM_IN_NUM;
    server_info->ServerInfo.dwAlarmOutCount = ALARM_OUT_NUM;

    server_info->ServerInfo.dwSysFlags = CMS_FUNC_MASK2;
    #ifdef MODULE_SUPPORT_LOCAL_REC
    server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_MD_RECORD;
    #endif
    //server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_PTZ;
    //server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_PPPOE_DDNS;
    //server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_ACTIVE_CONNECTION;
    //server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_EXCEPTALARM;
	#ifdef MODULE_SUPPORT_RF433
	server_info->ServerInfo.dwSysFlags |= DMS_SYS_FLAG_RF433_ALARM;
	#endif
    server_info->ServerInfo.dwUserRight = 1;
    server_info->ServerInfo.dwNetPriviewRight = 1;
    strcpy(server_info->ServerInfo.csServerDes, "GK-IPC-V1");

    JBNV_CHANNEL_INFO *channel_info = (JBNV_CHANNEL_INFO *) (send_msg + sizeof(JB_NET_PACK_HEAD) + sizeof(CMD_SERVER_INFO));
    channel_info->dwSize = sizeof(JBNV_CHANNEL_INFO);
    channel_info->dwStream1Height = runVideoCfg.vencStream[0].h264Conf.height;
    channel_info->dwStream1Width = runVideoCfg.vencStream[0].h264Conf.width;
    channel_info->dwStream1CodecID = runVideoCfg.vencStream[0].enctype == 3 ? VIDEO_DECODE_H265 : VIDEO_DECODE_H264_V2;
    channel_info->dwStream2Height = runVideoCfg.vencStream[1].h264Conf.height;
    channel_info->dwStream2Width = runVideoCfg.vencStream[1].h264Conf.width;
    channel_info->dwStream2CodecID = runVideoCfg.vencStream[1].enctype == 3 ? VIDEO_DECODE_H265 : VIDEO_DECODE_H264_V2;

    channel_info->dwAudioChannels = runAudioCfg.chans;
    channel_info->dwAudioBits = runAudioCfg.sampleBitWidth;
    channel_info->dwAudioSamples = runAudioCfg.sampleRate;
    channel_info->dwWaveFormatTag = WAVE_FORMAT_G711A;
    strcpy(channel_info->csChannelName, "gk-ipc");

    GkSockSend(sock, send_msg, sizeof(JB_NET_PACK_HEAD) + sizeof(CMD_SERVER_INFO) + sizeof(JBNV_CHANNEL_INFO), 100);
    return 0;

err:
    GkSockSend(sock, send_msg, sizeof(JB_NET_PACK_HEAD) + sizeof(CMD_SERVER_INFO) + sizeof(JBNV_CHANNEL_INFO), 100);
    return -1;
}

/*************************************************************
* �������ܣ�����
* ���������
* ���������
* ����ֵ  ��
* ��ע    :
*************************************************************/
int Gk_KeepAlive(int sock)
{
    JB_NET_PACK_HEAD cmd_head = {0};

    cmd_head.nFlag      = 9000;
    cmd_head.nCommand   = JBCMD_KEEP_ALIVE;
    cmd_head.nChannel   = 0;
    cmd_head.nErrorCode = 0;
    cmd_head.nBufSize   = 0;

    GkSockSend(sock, (char *)&cmd_head, sizeof(JB_NET_PACK_HEAD), 100);
    return 0;
}

/*************************************************************
* �������ܣ��ر�
* ���������
* ���������
* ����ֵ  ��
* ��ע    :
*************************************************************/
int Gk_MediaClose(int sock)
{
    CHANNEL_OPTION cmd = {0};
    int ret = GkSockRecv(sock, (char *)&cmd, sizeof(CHANNEL_OPTION));
    if (ret != sizeof(CHANNEL_OPTION)) {
        PRINT_ERR();
        return -1;
    }

    if (cmd.nStreamType == 0) {
        GkCloseLvSock(cmd.nID, GK_MAIN_STREAM);
    } else if (cmd.nStreamType == 1) {
        GkCloseLvSock(cmd.nID, GK_SUB_STREAM);
    }

    GkClosePbSock(cmd.nID);

    return 0;
}

void PrintDmsTime(DMS_SYSTEMTIME *ptime)
{
    printf("%u-%u-%u  %u  %u:%u:%u-%u\n", ptime->wYear, ptime->wMonth, ptime->wDay,
           ptime->wDayOfWeek, ptime->wHour, ptime->wMinute,
           ptime->wSecond, ptime->wMilliseconds);
}



int Gk_SetIFrame(int sock, int ext_len)
{
	int ret = 0;
    JB_CHANNEL_FRAMEREQ set_i;
    memset(&set_i, 0, sizeof(JB_CHANNEL_FRAMEREQ));
    ret = GkSockRecv(sock, (char *)&set_i, sizeof(JB_CHANNEL_FRAMEREQ));
    if(ret != sizeof(JB_CHANNEL_FRAMEREQ))
    {
        PRINT_ERR();
        return -1;
    }

    if(set_i.dwStreamType == 0 || set_i.dwStreamType == 1)
    {
        PRINT_INFO("cmd sock %d stream1 request I frame.\n", sock);
		netcam_video_force_i_frame(set_i.dwStreamType);
        g_session_reques_i_freame[set_i.dwStreamType] = 1;
    }
    return 0;
}

int GkGetIFrameRequest(int sessionNum)
{
    int ret = 0;
    if(g_session_reques_i_freame[sessionNum])
    {
        ret = g_session_reques_i_freame[sessionNum];
        g_session_reques_i_freame[sessionNum] = 0;
    }

    return ret;
}


void Gk_Reboot()
{
	netcam_sys_operation(NULL,(void *)SYSTEM_OPERATION_REBOOT); //force REBOOT
    //system("/sbin/reboot");
}

void Gk_Shundown()
{
    new_system_call("/sbin/poweroff");
}

int GkDealCmdDispatch(int sock)
{
    int ret = 0;
    fd_set fdr;
    struct timeval time = {0, 0};
    g_session_pthread_run = 1;
    JB_NET_PACK_HEAD cmd_head = {0};
    while (1) {
        if (0 == g_session_pthread_run) {
            break;
        }

        FD_ZERO(&fdr);
        FD_SET(sock, &fdr);
        time.tv_sec = 2;
        time.tv_usec = 0;
        select(sock + 1, &fdr, NULL, NULL, &time);
        if (!FD_ISSET(sock, &fdr)) {
            usleep(100000);
            continue;
        }

		memset(&cmd_head, 0x0, sizeof(JB_NET_PACK_HEAD));
		ret = GkSockRecv(sock, (char *)&cmd_head, sizeof(JB_NET_PACK_HEAD));
        if (ret == -1){
            PRINT_ERR("recv err.\n");
			break;
		}
        else if (ret != sizeof(JB_NET_PACK_HEAD)) {
            PRINT_ERR("recv %d not equal %d\n", ret, sizeof(JB_NET_PACK_HEAD));
            PrintStringToUInt((char *)&cmd_head, ret);
			//break;
            continue;
		}

        //printf("sock:%d : ", sock);

        switch (cmd_head.nCommand) {
            /* �����豸 */
            case GK_NET_CMD_REBOOT: // �����豸
                {
                    PRINT_INFO("cmd = GK_NET_CMD_REBOOT \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_Reboot();
                }
            	break;
            /* �ָ�����ֵ���� */
            case GK_NET_CMD_RESTORE: //�ָ�����ֵ����
            	{
                    PRINT_INFO("cmd = GK_NET_CMD_RESTORE \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_CmsRestore();
            	}
            	break;
            /* �����û����õ�flash�� */
            case GK_NET_CMD_SAVE:
                {
                    PRINT_INFO("cmd = GK_NET_CMD_SAVE \n");
                    //printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_CmsSave();
                }
            	break;
            /* ץ�� */
            case GK_NET_CMD_SNAP:
                {
                    PRINT_INFO("cmd = GK_NET_CMD_SNAP \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_CmsDoSnapshot(sock);
                }
            	break;
            /* �������������������� */
            case GK_NET_CMD_CONNECT_CMS:
                {
                    PRINT_INFO("cmd = GK_NET_CMD_CONNECT_CMS \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    //Gk_CmsConnectCms(sock);
                }
            	break;



            /* ���� */
            case JBCMD_KEEP_ALIVE:
                //PRINT_INFO("cmd = JBCMD_KEEP_ALIVE \n");
                ret = Gk_KeepAlive(sock);
                break;
            /* �ر�����ͨ�� */
            case JBCMD_PLAY_CLOSE:
                PRINT_INFO("cmd = JBCMD_PLAY_CLOSE \n");
                ret = Gk_MediaClose(sock);
                break;

            /* ϵͳʱ�� */
            case CMD_SETSYSTIME:
                PRINT_INFO("cmd = CMD_SETSYSTIME \n");
                ret = Gk_SetSystime(sock);
                break;
            case CMD_GETSYSTIME:
                PRINT_INFO("cmd = CMD_GETSYSTIME \n");
                ret = Gk_GetSystime(sock);
                break;
            case DMS_NET_CMD_GET_SYSTIME: //��ȡϵͳʱ��
            	{
                    PRINT_INFO("cmd = DMS_NET_CMD_GET_SYSTIME \n");
                    Gk_CmsGetSystime(sock);
            	}
            	break;
            case DMS_NET_CMD_SET_SYSTIME: //����ϵͳʱ��
            	{
                    PRINT_INFO("cmd = DMS_NET_CMD_SET_SYSTIME \n");
                    Gk_CmsSetSystime(sock);
            	}
            	break;

            /* ǿ��I֡���� */
            case CMD_REQUEST_IFRAME:
                PRINT_INFO("cmd sock = %d, cmd = CMD_REQUEST_IFRAME \n", sock);
                ret = Gk_SetIFrame(sock, cmd_head.nBufSize);
                break;

            /* �����豸 */
            case DMS_NET_CMD_REBOOT: // �����豸
                PRINT_INFO("cmd = DMS_NET_CMD_REBOOT \n");
                Gk_Reboot();
            	break;

            /* �豸�ػ� */
            case DMS_NET_CMD_SHUTDOWN: // �豸�ػ�
                PRINT_INFO("cmd = DMS_NET_CMD_SHUTDOWN \n");
                Gk_Shundown();
            	break;

            /* ���� */
            case DMS_NET_CMD_UPGRADE_DATA: //�ļ�ϵͳ��������
                PRINT_INFO("cmd = DMS_NET_CMD_UPGRADE_DATA \n");
                ret = Gk_CmdUpgradeData(sock, cmd_head.nBufSize);
                break;
            case DMS_NET_CMD_UPDATE_FLASH: //����Flash
                PRINT_INFO("cmd = DMS_NET_CMD_UPDATE_FLASH \n");
                break;
            case DMS_NET_CMD_UPGRADE_REQ: //�ļ�ϵͳ��������
                PRINT_INFO("cmd = DMS_NET_CMD_UPGRADE_REQ \n");
                //ret = Gk_CmdUpgradeReq(sock, cmd_head.nBufSize);
                break;
            case DMS_NET_CMD_UPGRADE_RESP: //�ļ�ϵͳ��������
                PRINT_INFO("cmd = DMS_NET_CMD_UPGRADE_RESP \n");
                //ret = Gk_CmdUpgradeResp(sock, cmd_head.nBufSize);
                break;


            /* �쳣���� */
            case DMS_NET_GET_EXCEPTIONCFG:  //��ȡ�쳣��������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_EXCEPTIONCFG \n");
            		Gk_CmsGetException(sock);
            	}
            	break;
            case DMS_NET_SET_EXCEPTIONCFG: //�����쳣��������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_EXCEPTIONCFG \n");
                    Gk_CmsSetException(sock);
            	}
            	break;

            /* ����ֵ���� */
            case DMS_NET_GET_RESTORECFG: //�ָ�����ֵ����
            	{
            		PRINT_INFO("cmd = DMS_NET_GET_RESTORECFG \n");
            		Gk_CmsGetRestore(sock);
    	        }
            	break;
            case DMS_NET_SET_RESTORECFG:  //�ָ�����ֵ����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RESTORECFG \n");
                    Gk_CmsSetRestore(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_RESTORECFG: //��ȡĬ�ϻָ�����ֵ����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_RESTORECFG \n");
            		Gk_CmsGetRestore(sock);
            	}
            	break;
            case DMS_NET_SET_SAVECFG: //��������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_SAVECFG \n");
                    Gk_CmsSetSave(sock);

            	}
            	break;


            /* ������� */
            case DMS_NET_GET_ALARMOUTCFG: //��ȡ�����������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ALARMOUTCFG \n");

            		Gk_CmsGetAlarmout(sock);
            	}
            	break;
            case DMS_NET_SET_ALARMOUTCFG: //���ñ����������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_ALARMOUTCFG \n");
                    Gk_CmsSetAlarmout(sock);

            	}
            	break;
            case DMS_NET_GET_ALARMOUT_STATE: // ��ȡ�������״̬
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ALARMOUT_STATE \n");
                    Gk_CmsGetAlarmoutState(sock);

            	}
            	break;
            case DMS_NET_CMD_ALARMOUT_CONTROL: //�����������
            	{
                    PRINT_INFO("cmd = DMS_NET_CMD_ALARMOUT_CONTROL \n");
                    Gk_CmsAlarmoutControl(sock);
            		 //?
            	}
            	break;


            /* �������� */
            case DMS_NET_GET_ALARMINCFG: //��ȡ�����������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ALARMINCFG \n");
            		Gk_CmsGetAlarmin(sock);
            	}
            	break;
            case DMS_NET_SET_ALARMINCFG:  //���ñ����������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_ALARMINCFG \n");
                    Gk_CmsSetAlarmin(sock);

            	}
            	break;

            /* PTZ����Э�� */
            case DMS_NET_GET_DECODERCFG: //��ȡ���������õ�Ԥ�õ����Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DECODERCFG \n");

                    Gk_CmsGetPtzDecode(sock);
            	}
            	break;
            case DMS_NET_SET_DECODERCFG: //��ȡ���������õ�Ԥ�õ����Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DECODERCFG \n");

                    Gk_CmsSetPtzDecode(sock);
            	}
            	break;
            case DMS_NET_GET_DEF_DECODERCFG: //��ȡ���������õ�Ԥ�õ����Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DECODERCFG \n");

                    Gk_CmsGetDefPtzDecode(sock);
            	}
            	break;

            /* PTZ */
            case DMS_NET_GET_ALL_PRESET: //��ȡ���������õ�Ԥ�õ����Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ALL_PRESET \n");

            		Gk_CmsGetAllPreset(sock);
            	}
            	break;
            case DMS_NET_GET_CRUISE_CFG: //��ȡͨ���������Ѿ����õ�Ѳ����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_CRUISE_CFG \n");
            		Gk_CmsGetCruise(sock);
            	}
            	break;
            case DMS_NET_SET_CRUISE_INFO: //����һ��Ѳ��
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_CRUISE_INFO \n");
                    Gk_CmsSetCruise(sock);
            	}
            	break;
            case DMS_NET_CMD_PTZ_CONTROL: //��̨����
            	{
                    PRINT_INFO("cmd = DMS_NET_CMD_PTZ_CONTROL \n");
                    Gk_CmsPtzControl(sock);
            	}
            	break;

            /* ͼ���ڵ� */
            case DMS_NET_GET_SHELTERCFG: //��ȡͼ���ڵ�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_SHELTERCFG \n");
            		Gk_CmsGetShelter(sock, 0);
            	}
            	break;
            case DMS_NET_SET_SHELTERCFG: //ͼ���ڵ�
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_SHELTERCFG \n");
                    Gk_CmsSetShelter(sock, 0);

            	}
            	break;

            /* �豸���� */
            case DMS_NET_GET_DEVICECFG: //��ȡ�豸����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEVICECFG \n");
                    Gk_CmsGetDevice(sock);
            	}
            	break;
            case DMS_NET_SET_DEVICECFG: //�豸��Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_DEVICECFG \n");
                    Gk_CmsSetDevice(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_DEVICECFG: //����Ĭ���豸����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_DEVICECFG \n");
                    Gk_CmsGetDefDevice(sock);
            	}
            	break;

            /* ���������� */
            case DMS_NET_GET_WANWIRELESSCFG_V2: //��ȡ���������߲���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_WANWIRELESSCFG_V2 \n");
                    // DMS_NET_WANWIRELESS_CONFIG_V2 no
            	}
            	break;
            case DMS_NET_SET_WANWIRELESSCFG_V2: //����������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_WANWIRELESSCFG_V2 \n");
            		//DMS_NET_WANWIRELESS_CONFIG_V2  //no
            	}
            	break;

            /* ͼ��ѹ������ */
            case DMS_NET_GET_SUPPORT_STREAM_FMT: // //��ȡϵͳ֧�ֵ�ͼ������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_SUPPORT_STREAM_FMT \n");
                    Gk_CmsGetStreamSupport(sock);
            	}
            	break;
            case DMS_NET_GET_PICCFG: //��ȡͼ��ѹ������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_PICCFG \n");
                    Gk_CmsGetVideo(sock);

            	}
            	break;
            case DMS_NET_SET_PICCFG: //����ͼ��ѹ������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_PICCFG \n");
                    Gk_CmsSetVideo(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_PICCFG: //��ȡĬ��ͼ��ѹ������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_PICCFG \n");
                    Gk_CmsGetDefVideo(sock);
            	}
            	break;

            /* ͼ��ɫ�ʲ��� */
            case DMS_NET_GET_COLOR_SUPPORT: //��ȡͼ��ɫ�ʲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_COLOR_SUPPORT \n");
                    Gk_CmsGetColorSupport(sock);
            	}
            	break;
            case DMS_NET_GET_COLORCFG: //��ȡͼ��ɫ�ʲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_COLORCFG \n");
            		Gk_CmsGetColor(sock);

            	}
            	break;
            case DMS_NET_SET_COLORCFG: //����ͼ��ɫ�ʲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_COLORCFG \n");
            		Gk_CmsSetColor(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_COLORCFG: //��ȡͼ��ɫ�ʲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_COLORCFG \n");
                    Gk_CmsGetDefColor(sock);

                        // ???
            	}
            	break;
            case DMS_NET_SET_COLORCFG_SINGLE: //��������ĳ��ͼ��ɫ�ʲ���,
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_COLORCFG_SINGLE \n");
                    Gk_CmsSetColorSingle(sock);
            		 //?
            	}
            	break;


            /* ͼ��ɫ��֧�ָ߼����� */
            case DMS_NET_GET_ENHANCED_COLOR_SUPPORT: //��ȡͼ��ɫ��֧�ָ߼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ENHANCED_COLOR_SUPPORT \n");

                    Gk_CmsGetEnhancedColorSupport(sock);
                    //brcue to check
            	}
            	break;
            case DMS_NET_GET_ENHANCED_COLOR: //��ȡͼ��ɫ�ʸ߼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ENHANCED_COLOR \n");
                    Gk_CmsGetEnhancedColor(sock);
                    //brcue  to check
            	}
            	break;

            /* ��ת�ڼ����� */
            case DMS_NET_GET_COLOR_BLACK_DETECTION: //��ȡ��ת�ڼ�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_COLOR_BLACK_DETECTION \n");
                    Gk_CmsGetDayNightDetection(sock);
                    //brcue  to check
            	}
            	break;
            case DMS_NET_SET_COLOR_BLACK_DETECTION: //���ò�ת�ڼ�����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_COLOR_BLACK_DETECTION \n");
                    Gk_CmsSetDayNightDetection(sock);

            	}
            	break;

            /* UPNP */
            case DMS_UPNP_GET_CONFIG: //��ȡUPNP����
            	{
                    PRINT_INFO("cmd = DMS_UPNP_GET_CONFIG \n");
                    //DMS_UPNP_CONFIG
            	}
            	break;
            case DMS_UPNP_SET_CONFIG: //����UPNP����
            	{
                    PRINT_INFO("cmd = DMS_UPNP_SET_CONFIG \n");
            		//DMS_UPNP_CONFIG
            	}
            	break;

            /* FTP */
            case DMS_NET_GET_FTPCFG: //��ȡFTP����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_FTPCFG \n");
                    Gk_CmsGetFtp(sock);
            	}
            	break;
            case DMS_NET_SET_FTPCFG: //����FTP����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_FTPCFG \n");
                    Gk_CmsSetFtp(sock);

            	}
            	break;
            case DMS_NET_TEST_FTP_CONNECT: //���� FTP ����
            	{
                    PRINT_INFO("cmd = DMS_NET_TEST_FTP_CONNECT \n");
                    Gk_CmsTestFtp(sock);
            	}
            	break;

            /* osd */
            case DMS_NET_GET_OSDCFG: //��ȡͼ���ַ����Ӳ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_OSDCFG \n");
                    Gk_CmsGetOsd(sock, 0);
            	}
            	break;
            case DMS_NET_SET_OSDCFG: //����ͼ���ַ����Ӳ���
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_OSDCFG \n");
                    Gk_CmsSetOsd(sock, 0);
            	}
            	break;

            /* �ֻ�ƽ̨���� */
            case DMS_GET_MOBILE_CENTER_INFO: //��ȡ�ֻ�ƽ̨����
            	{
                    PRINT_INFO("cmd = DMS_GET_MOBILE_CENTER_INFO \n");
            		//DMS_MOBILE_CENTER_INFO //no
            		Gk_CmsGetMobileCenterInfo(sock);
            	}
            	break;
            case DMS_SET_MOBILE_CENTER_INFO: //�����ֻ�ƽ̨����
            	{
                    PRINT_INFO("cmd = DMS_SET_MOBILE_CENTER_INFO \n");
            		//DMS_MOBILE_CENTER_INFO //no
            		Gk_CmsSetMobileCenterInfo(sock);
            	}
            	break;

            /* ������� */
            case DMS_NET_GET_NETCFG: //��ȡ�������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_NETCFG \n");
                    Gk_CmsGetLan(sock);
            	}
            	break;
            case DMS_NET_SET_NETCFG: //�����������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_NETCFG \n");
                    Gk_CmsSetLan(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_NETCFG: //��ȡĬ���������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_NETCFG \n");
                    Gk_CmsGetDefLan(sock);
            	}
                break;

            /* RTSP */
            case DMS_NET_GET_RTSPCFG: //��ȡRTSP����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_RTSPCFG \n");
                    Gk_CmsGetRtsp(sock);

            	}
            	break;
            case DMS_NET_SET_RTSPCFG: //rtsp����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RTSPCFG \n");
                    Gk_CmsSetRtsp(sock);
            	}
            	break;

            /* PPPOE */
            case DMS_NET_GET_PPPOECFG: //��ȡPPPOE����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_PPPOECFG \n");
            		//DMS_NET_PPPOECFG //no
            		Gk_CmsGetPPPOECFG(sock);
            	}
            	break;
            case DMS_NET_SET_PPPOECFG: //����PPPOE����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_PPPOECFG \n");
            		//DMS_NET_PPPOECFG
            		Gk_CmsSetPPPOECFG(sock);
            	}
            	break;

            /* P2P */
            case DMS_NET_GET_P2PCFG: //��ȡP2P����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_P2PCFG \n");
            		//DMS_NET_P2P_CFG //no
            		Gk_CmsGetP2PCFG(sock);
            	}
            	break;
            case DMS_NET_SET_P2PCFG: //����P2P����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_P2PCFG \n");
            		//DMS_NET_P2P_CFG
            		#if 0
            		if (0 == Gk_CmsSetP2PCFG(sock))
                    {
                        gadi_sys_thread_sleep(500);
	                    netcam_sys_operation(NULL,(void *)SYSTEM_OPERATION_REBOOT); //force REBOOT
                    }
                    #else
                    Gk_CmsSetP2PCFG(sock);
                    #endif
            	}
            	break;

            /* �豸ά�� */
            case DMS_DEV_GET_DEVICEMAINTAINCFG: //��ȡ�豸ά��
            	{
                    PRINT_INFO("cmd = DMS_DEV_GET_DEVICEMAINTAINCFG \n");
                    Gk_CmsGetMaintain(sock);
            		 //ok
            	}
            	break;
            case DMS_DEV_SET_DEVICEMAINTAINCFG: //�豸ά��
            	{
                    PRINT_INFO("cmd = DMS_DEV_SET_DEVICEMAINTAINCFG \n");
                    Gk_CmsSetMaintain(sock);

            	}
            	break;





            /* LOG��Ϣ */
            case DMS_NET_GET_LOGINFO: //��ȡLOG��Ϣ
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_LOGINFO \n");
                    Gk_CmsGetLog(sock);

            		 //ok
            	}
            	break;

            /* EMAIL */
            case DMS_NET_GET_EMAILCFG: //��ȡEMAIL����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_EMAILCFG \n");
                    Gk_CmsGetEmail(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_EMAILCFG:  //����EMAIL����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_EMAILCFG \n");
                    Gk_CmsSetEmail(sock);

            	}
            	break;
            case DMS_NET_TEST_SEND_EMAIL:  //����Email ����
            	{
                    PRINT_INFO("cmd = DMS_NET_TEST_SEND_EMAIL \n");
                    Gk_CmsTestEmail(sock);
            	}
            	break;

            /* SNMP */
            case DMS_NET_GET_SNMPCFG: //��ȡsnmp����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_SNMPCFG \n");
            		//DMS_SNMP_CFG //no
            	}
            	break;
            case DMS_NET_SET_SNMPCFG: //����SNMP����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_SNMPCFG \n");
            		//DMS_SNMP_CFG
            	}
            	break;

            /* ʱ������ʱ�Ʋ��� */
            case DMS_NET_GET_ZONEANDDSTCFG: //��ȡʱ������ʱ�Ʋ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_ZONEANDDSTCFG \n");
                    Gk_CmsGetZoneAndDst(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_ZONEANDDSTCFG: //ʱ������ʱ��
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_ZONEANDDSTCFG \n");
                    Gk_CmsSetZoneAndDst(sock);

            	}
            	break;

            /* NTP  */
            case DMS_NET_GET_NTPCFG: ////��ȡNTP ����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_NTPCFG \n");
                    Gk_CmsGetNtp(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_NTPCFG: //����NTP ����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_NTPCFG \n");
                    Gk_CmsSetNtp(sock);

            	}
            	break;
            case DMS_NET_DEF_NTPCFG: //Ĭ��NTP ����
            	{
                    PRINT_INFO("cmd = DMS_NET_DEF_NTPCFG \n");
            		Gk_CmsDefNtp(sock);
            	}
            	break;

            /* ͼ��ʱץ�� */
            case DMS_NET_GET_SNAPTIMERCFG://��ȡͼ��ʱץ�Ĳ���
            	{
            	    PRINT_INFO("cmd = DMS_NET_GET_SNAPTIMERCFG \n");
                    Gk_CmsGetSnap(sock);
            	}
            	break;
            case DMS_NET_SET_SNAPTIMERCFG: //����ͼ��ʱץ�Ĳ���
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_SNAPTIMERCFG \n");
                    Gk_CmsSetSnap(sock);
            	}
            	break;

            /* ͼ���¼�ץ�� */
            case DMS_NET_GET_SNAPEVENTCFG: //��ȡͼ���¼�ץ�Ĳ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_SNAPEVENTCFG \n");
                    Gk_CmsGetSnapEvent(sock);
            	}
            	break;
            case DMS_NET_SET_SNAPEVENTCFG: //����ͼ���¼�ץ�Ĳ���
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_SNAPEVENTCFG \n");
                    Gk_CmsSetSnapEvent(sock);

            	}
            	break;

            /* ���� */
            case DMS_NET_GET_RS232CFG: //��ȡ232���ڲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_RS232CFG \n");
                    Gk_CmsGetRs232(sock);

            		 //ok
            	}
            	break;
            case DMS_NET_SET_RS232CFG: //����232���ڲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RS232CFG \n");
                    Gk_CmsSetRs232(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_SERIAL: //Ĭ�ϴ��ڲ���
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_SERIAL \n");
                    Gk_CmsGetDefRs232(sock);

            	}
            	break;

            /* �û� */
            case DMS_NET_GET_USERCFG: //��ȡ�û�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_USERCFG \n");
                    Gk_CmsGetUser(sock);
                    //lx to check
            	}
            	break;
            case DMS_NET_SET_USERCFG: //�����û�����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_USERCFG \n");
                    Gk_CmsSetUser(sock);

            	}
            	break;

            /* ͼ���ƶ���� */
            case DMS_NET_GET_MOTIONCFG: //��ȡͼ���ƶ�������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_MOTIONCFG \n");
                    Gk_CmsGetMd(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_MOTIONCFG: //����ͼ���ƶ�������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_MOTIONCFG \n");
                    Gk_CmsSetMd(sock);

            	}
            	break;
            case DMS_NET_DEF_MOTIONCFG: //Ĭ��ͼ���ƶ�������
            	{
                    PRINT_INFO("cmd = DMS_NET_DEF_MOTIONCFG \n");
            		//DMS_NET_CHANNEL_MOTION_DETECT
            	}
            	break;

            /* ͼ����Ƶ��ʧ���� */
            case DMS_NET_GET_VLOSTCFG: //��ȡͼ����Ƶ��ʧ����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_VLOSTCFG \n");
                    Gk_CmsGetVlost(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_VLOSTCFG: //����ͼ����Ƶ��ʧ����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_VLOSTCFG \n");
                    Gk_CmsSetVlost(sock);

            	}
            	break;
            case DMS_NET_DEF_VLOSTCFG: //Ĭ��ͼ����Ƶ��ʧ����
            	{
                    PRINT_INFO("cmd = DMS_NET_DEF_VLOSTCFG \n");
            		//DMS_NET_CHANNEL_VILOST
            	}
            	break;

            /* ͼ���ڵ��������� */
            case DMS_NET_GET_HIDEALARMCFG: //��ȡͼ���ڵ���������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_HIDEALARMCFG \n");
                    Gk_CmsGetHideAlarm(sock);
            		 //ok
            	}
            	break;
            case DMS_NET_SET_HIDEALARMCFG: //����ͼ���ڵ���������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_HIDEALARMCFG \n");
                    Gk_CmsSetHideAlarm(sock);

            	}
            	break;
            case DMS_NET_DEF_HIDEALARMCFG: //Ĭ��ͼ���ڵ���������
            	{
                    PRINT_INFO("cmd = DMS_NET_DEF_HIDEALARMCFG \n");
            		//DMS_NET_CHANNEL_HIDEALARM
            	}
            	break;

            /* WIFI */
            case DMS_NET_GET_WIFICFG: //��ȡWIFI����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_WIFICFG \n");
                    Gk_CmsGetWifi(sock);
            	}
            	break;
            case DMS_NET_SET_WIFICFG: //����WIFI����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_WIFICFG \n");
                    Gk_CmsSetWifi(sock);
            	}
            	break;

            /* WIFI Connet Testing*/
            case DMS_NET_TEST_WIFI_CONNECT:
                {
                    PRINT_INFO("cmd = DMS_NET_TEST_WIFI_CONNECT \n");
                    Gk_CmsTestWifiConnect(sock);
                }
                break;

            /* WIFI վ���б� */
            case DMS_NET_GET_WIFI_SITE_LIST: //��ȡWIFI վ���б�
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_WIFI_SITE_LIST \n");
                    Gk_CmsGetWifiSiteList(sock);
            		//??

            	}
            	break;
            case DMS_NET_SET_WIFI_WPS_START: //����WPS��ʼ
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_WIFI_WPS_START \n");
                    Gk_CmsSetWifiWpsStart(sock);
            	}
            	break;

            /* DDNS���� */
            case DMS_NET_GET_DDNSCFG: //��ȡDDNS����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DDNSCFG \n");
            		//DMS_NET_DDNSCFG //no
            		Gk_CmsGetDDSCFG(sock);
            	}
            	break;
            case DMS_NET_SET_DDNSCFG: //����DDNS����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_DDNSCFG \n");
            		//DMS_NET_DDNSCFG
            		Gk_CmsSetDDSCFG(sock);
            	}
            	break;

            #ifdef MODULE_SUPPORT_LOCAL_REC
            /* ��ѯ¼���ļ� */
            case DMS_NET_GET_REC_DAY_IN_MONTH:
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_REC_DAY_IN_MONTH \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_CmsGetRecDayInMonth(sock);
            		 //ok
            	}
            	break;
            case CMD_GET_FILELIST:
                {
                    PRINT_INFO("cmd = CMD_GET_FILELIST \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    Gk_CmsGetFilelist(sock);
                }
            	break;
            /* ¼����� */
            case DMS_NET_GET_RECORDCFG: //��ȡͼ��¼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_RECORDCFG \n");
                    Gk_CmsGetRecord(sock);
            	}
            	break;
            case DMS_NET_SET_RECORDCFG: //����ͼ��¼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RECORDCFG \n");
                    Gk_CmsSetRecord(sock);

            	}
            	break;
            case DMS_NET_GET_DEF_RECORDCFG: //��ȡĬ�ϵ�¼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_DEF_RECORDCFG \n");
                    Gk_CmsGetDefRecord(sock);
            	}
            	break;
            /* ¼�������� */
            case DMS_NET_GET_RECORDSTREAMMODE://��ȡ¼��������
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_RECORDSTREAMMODE \n");
                    Gk_CmsGetRecordStreamMode(sock);
            		//ok
            	}
            	break;
            case DMS_NET_SET_RECORDSTREAMMODE: //����¼��������
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RECORDSTREAMMODE \n");
                    Gk_CmsSetRecordStreamMode(sock);

            	}
            	break;
            /* ͼ���ֶ�¼����� */
            case DMS_NET_GET_RECORDMODECFG: //��ȡͼ���ֶ�¼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_RECORDMODECFG \n");
                    Gk_CmsGetRecordMode(sock);
                    // ??
            	}
            	break;
            case DMS_NET_SET_RECORDMODECFG: //����ͼ���ֶ�¼�����
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_RECORDMODECFG \n");
                    Gk_CmsSetRecordMode(sock);

            	}
            	break;
            /* ¼��������� */
            case DMS_NET_CMD_REC_CONTROL:
                PRINT_INFO("cmd = DMS_NET_CMD_REC_CONTROL \n");
                ret = Gk_CmdRecControl(sock);
                break;

            /* NAS */
            case DMS_NET_GET_NASCFG:
                {
                    PRINT_INFO("cmd = DMS_NET_GET_NASCFG \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);

                    //�ƴ洢��չ
                    ret = Gk_GetNas(sock);
                    break;
                }
            	break;
            /* NAS */
            case DMS_NET_SET_NASCFG:
                {
                    PRINT_INFO("cmd = DMS_NET_SET_NASCFG \n");
                    printf("channel:%lu errcode:%lu extlen:%lu \n", cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);
                    //�ƴ洢��չ
                    ret = Gk_SetNas(sock);
                    break;
                }
            	break;
            /* ���̣�SD����Ϣ */
            case DMS_NET_GET_HDCFG:
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_HDCFG \n");
            		//DMS_NET_DDNSCFG //no
            		Gk_CmsGetHd(sock);
            	}
            	break;
            case DMS_NET_SET_HDCFG:
            	{
                    PRINT_INFO("cmd = DMS_NET_SET_HDCFG \n");
            		//DMS_NET_DDNSCFG //no
            		Gk_CmsSetHd(sock);
            	}
            	break;
            case DMS_NET_HD_FORMAT:
            	{
                    PRINT_INFO("cmd = DMS_NET_HD_FORMAT \n");
                    Gk_CmsHdFormat(sock);
            	}
            	break;
            case DMS_NET_GET_HD_FORMAT_STATUS:
            	{
                    PRINT_INFO("cmd = DMS_NET_GET_HD_FORMAT_STATUS \n");
                    Gk_CmsHdGetFormatStaus(sock);
            	}
            	break;
            case DMS_NET_UNLOAD_DISK:
            	{
                    PRINT_INFO("cmd = DMS_NET_UNLOAD_DISK \n");
                    Gk_CmsUnloadDisk();
            	}
            	break;
            case CMD_RECORD_BEGIN:
            	{
                    PRINT_INFO("cmd = CMD_RECORD_BEGIN \n");
                    Gk_CmsRecordStart(sock);
            	}
            	break;
            case CMD_RECORD_STOP:
            	{
                    PRINT_INFO("cmd = CMD_RECORD_STOP \n");
                    Gk_CmsRecordStop(sock);
            	}
            	break;

            case JBCMD_PLAYBACK_CMD:
            	{
                    PRINT_INFO("cmd = JBCMD_PLAYBACK_CMD \n");
                    Gk_CmsPbControl(sock);
            	}
            	break;

            #endif /* MODULE_SUPPORT_LOCAL_REC */

			#ifdef MODULE_SUPPORT_RF433
			case DMS_RF433_DEV_MODIFY_NAME:
			{
				Gk_Cms_433dev_modifyname(sock);
			}
			break;
			case DMS_RF433_DEV_GET_DEV_LIST:
			{
				Gk_Cms_433dev_getlist(sock);
			}
			break;
			case DMS_RF433_DEV_LEARN_CODE:
			{
				Gk_Cms_433dev_learncode(sock);
			}
			break;
			case DMS_RF433_DEV_DELETE_DEV:
			{
				Gk_Cms_433dev_deletecode(sock);
			}
			break;
			case DMS_RF433_DEV_EXIT_LEARN_CODE:
			{
				Gk_Cms_433dev_exit_learncode(sock);
			}
			break;
			#endif
            case CMD_TEST_SPEAKER:
            {
                PRINT_INFO("cmd = CMD_TEST_SPEAKER \n");
                Gk_CmsTestSpeaker();
            }
            break;
            case CMD_TEST_IRCUT:
            {
                PRINT_INFO("cmd = CMD_TEST_IRCUT \n");
                Gk_CmsTestIRCut();
            }
            break;
            case CMD_TEST_GET_TEST_RESULT:
            {
                PRINT_INFO("cmd = CMD_TEST_GET_TEST_RESULT \n");
                Gk_CmsTestGetTestResult(sock);
            }
            break;
            case CMD_TEST_SET_TEST_RESULT:
            {
                PRINT_INFO("cmd = CMD_TEST_SET_TEST_RESULT \n");
                Gk_CmsTestSetTestResult(sock);
            }
            break;
            case CMD_GET_FAC_CFG:
            {
                PRINT_INFO("cmd = CMD_GET_FAC_CFG \n");
                Gk_CmsGetFacCFG(sock);
            }
            break;
            case CMD_SET_FAC_CFG:
            {
                PRINT_INFO("cmd = CMD_SET_FAC_CFG \n");
                Gk_CmsSetFacCFG(sock);
            }
            break;
            case CMD_GET_AF_OFFSET:
            {
                PRINT_INFO("cmd = CMD_GET_AF_OFFSET \n");
                Gk_CmsGetAFOffset(sock);
            }
            break;
            case CMD_SET_AF_OFFSET:
            {
                PRINT_INFO("cmd = CMD_SET_AF_OFFSET \n");
                Gk_CmsSetAFOffset();
            }
            break;
            case CMD_TEST_WHITE:
            {
                PRINT_INFO("cmd = CMD_TEST_WHITE \n");
                Gk_CmsTestWhiteLed();
            }			
            break;
            case CMD_TEST_CMD:
            {
                PRINT_INFO("cmd = CMD_TEST_CMD \n");
                Gk_CmsTestCmd(sock);
            }
            break;

            /* ��֧������ */
            default:
                {
                    PRINT_INFO("cmd %lu not support. nFlag:%lu nChannel:%lu nErrorCode:%lu nBufSize:%lu",
                        cmd_head.nCommand, cmd_head.nFlag, cmd_head.nChannel, cmd_head.nErrorCode, cmd_head.nBufSize);

                    char recv_buf[GK_CMS_BUFLEN];
                    RecvExtData(sock, recv_buf, GK_CMS_BUFLEN, cmd_head.nBufSize);
                    //g_session_pthread_run = 0;
                }
                break;
        }
    }

    return 0;
}



