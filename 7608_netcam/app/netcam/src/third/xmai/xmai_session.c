/************************************************************************
author:bruce
editor:
description:ʵ����nvr�Ľ��� ��������Խ����طţ���¼�����ع��ܶ���֧�֣�
д�˴��뵫����ûʵ�֣��Ժ��������չ��֧������˽��Э��
history: 2014.6.1

************************************************************************/

#include "xmai_session.h"
#include "xmai_common_api.h"
#include "cJSON.h"
#include "media_fifo.h"
#include "cfg_video.h"
#include "netcam_api.h"
#include "util_sock.h"


UtSession g_xmai_session;
int g_xmai_pthread_run = 1;
int g_xmai_data_seq_num = 1;

pthread_mutex_t g_xmai_audio_mutex = PTHREAD_MUTEX_INITIALIZER;
int g_xmai_talk_session_id = -1;
int g_xmai_talk_send_index = 1;
int g_xmai_talk_recv_running = 0;
extern int g_xmai_is_talking;

/*************************************************************
* �������ܣ� �����̣߳������óɷ�������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
int XMaiCreateThread(ThreadEntryPtrType entry, void *para, pthread_t *pid)
{
    pthread_t ThreadId;
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);//��
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);//����
    if(pthread_create(&ThreadId, &attr, entry, para) == 0)//�����߳�
    {
        pthread_attr_destroy(&attr);
		if(pid)
			*pid = ThreadId;

		return 0;
    }

    pthread_attr_destroy(&attr);
    return -1;
}

#if 0
pid_t gettid()
{
     return syscall(SYS_gettid); 
}
#endif

/*************************************************************
* �������ܣ���socket��������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
int XMaiSockSend(int sock, char *szbuf, int len)
{
	int ret = 0;
#if 0	
   	fd_set send_set, excpt_set;
	struct timeval tv;
    int send_total = 0;

    while(send_total < len)
    {
    	tv.tv_sec = 10;
    	tv.tv_usec = 0;
            
    	FD_ZERO(&send_set);
    	FD_ZERO(&excpt_set);
    	FD_SET(sock, &send_set);
    	FD_SET(sock, &excpt_set);
        
        ret = select(sock + 1, NULL, &send_set, &excpt_set, &tv);
       	if (ret < 0) 
    	{
            PRINT_ERR("select error SockSend Failed ret:%d, sock:%d, len:%d errno:%s\n", ret, sock, len, strerror(errno));
          	return -1;
    	}
        if(0 == ret)
        {
            PRINT_ERR("select timeout SockSend Failed ret:%d, sock:%d, len:%d errno:%s\n", ret, sock, len, strerror(errno));
            return -2;
        }
    	
    	ret = send(sock, szbuf + send_total, len - send_total, 0);
        if(ret <= 0)
        {
            PRINT_ERR("SockSend Failed ret:%d, sock:%d, len:%d errno:%s\n", ret, sock, len, strerror(errno));
            return -3;
        }
         
        send_total += ret;
    }
	
    //PRINT_INFO("Leave ==========> SockSend sock:%d, send_total:%d\n",sock,  send_total);     
	return send_total;	
#else
	ret = send(sock, szbuf, len, 0);
    if(ret <= 0)
    {
        PRINT_ERR("SockSend Failed ret:%d, sock:%d, len:%d errno:%s\n", ret, sock, len, strerror(errno));
        return -3;
    }	
	if(ret != len)
		PRINT_INFO("send not equil. send_len = %d, len = %d, sock = %d\n", ret, len, sock);

	return ret;
#endif    
}

int XMaiTcpSelectSend(int sock, char *szbuf, int len, int timeout)
{
    int ret = 0;
    fd_set send_set;
    struct timeval tv;
    int send_total = 0;
    int n_timeout_num = 0;
    //if (timeout == 0)
        timeout = 2;
    while(send_total < len) {
        if(2 == n_timeout_num) {//�����ʱ2����رմ˴λỰ
            PRINT_ERR("sock:%d -- select send timeout. timeout is set %d seconds and counts is set %d\n", sock, timeout, n_timeout_num);
            return -2;
        }

    	FD_ZERO(&send_set);
    	FD_SET(sock, &send_set);
    	tv.tv_sec = timeout;
    	tv.tv_usec = 0;
        ret = select(sock + 1, NULL, &send_set, NULL, &tv);
       	if (ret < 0) {
            PRINT_ERR("sock:%d select send error:%s\n", sock, strerror(errno));
          	return -1;
    	}
        if(0 == ret) {
            n_timeout_num++;
            PRINT_ERR("sock:%d select send timeout. count:%d\n", sock, n_timeout_num);
            sleep(1);
            continue;
        }

		if(FD_ISSET(sock, &send_set)) {
        	ret = send(sock, szbuf + send_total, len - send_total, 0);
            if(ret <= 0) {
                PRINT_ERR("SockSend Failed ret:%d, sock:%d, len:%d error:%s\n", ret, sock, len, strerror(errno));
                return -3;
            }

    		n_timeout_num = 0;
			//break;
		}

        send_total += ret;
    }

    //PRINT_INFO("Leave ==========> SockSend sock:%d, send_total:%d\n",sock,  send_total);
	return send_total;
}


/*************************************************************
* �������ܣ���socket��������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
static int XMaiSockRecv(int sock, char *szbuf, int len)
{
#if 0	
    int ret = 0;
	fd_set read_set, excpt_set;
	struct timeval tv;
    int n_timeout_num = 0;
 	
	while(1)
	{
        if(2 == n_timeout_num) //�����ʱ2����رմ˴λỰ
        {
            return -1;
        }
		FD_ZERO(&read_set);
		FD_ZERO(&excpt_set);

		FD_SET(sock, &read_set);
		FD_SET(sock, &excpt_set);

		tv.tv_sec = 10;
		tv.tv_usec = 0;
	   	if ((ret = select(sock + 1, &read_set, NULL, &excpt_set, &tv)) < 0) 
		{
		    PRINT_ERR("recv error:%s\n", strerror(errno));
	      	return -1; 
		}
        if(0 == ret)
        {
            PRINT_ERR("recv istimeout\n");
            n_timeout_num++;
            continue;
        }
		if(FD_ISSET(sock, &excpt_set))
		{
		    PRINT_ERR("recv error:%s\n", strerror(errno));
			return -1;
		}
		
		if(FD_ISSET(sock, &read_set))
		{
    		//PRINT_INFO("recv have data\n");
    		n_timeout_num = 0;
			break;
		}
	}
#endif	

	//int curFlags = fcntl(sock, F_GETFL, 0);
	//printf("curFlags = %d\n", curFlags);	
	//printf("curFlags = %d\n", curFlags|O_NONBLOCK);
	//printf("sock = %d\n", sock);

	memset(szbuf, 0, len);
	int recvlen = recv(sock, szbuf, len, 0);
	if (recvlen < 0) //����
	{
    	PRINT_ERR("sock %d recv error:%s\n", sock, strerror(errno));
		return -1;
    }
	if(recvlen == 0) //����ر�socket
	{
		PRINT_INFO("tcp recv 0 len, to close sock: %d\n", sock);
		//close(sock);
		return -1;
	}
	//if(len == 0)
	//	PRINT_ERR("recv 0 len??? sock = %d\n", sock);
	
	return recvlen;
}



//////////////////////////////////////////////////////////////////////

int XMaiMakeMsg(XMaiSessionCtrl *pXMaiSessionCtrl, short messageid, char *jsonMsg)
{
    int xmaiMsgLen = 0;

	memset(pXMaiSessionCtrl->fSendbuf, 0, sizeof(pXMaiSessionCtrl->fSendbuf));
    XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)(pXMaiSessionCtrl->fSendbuf);
    
    pXMaiMsgHeader->headFlag = 0xFF;
    pXMaiMsgHeader->version = 0x01;
    pXMaiMsgHeader->reserved1 = 0x00;
    pXMaiMsgHeader->reserved2 = 0x00;
    pXMaiMsgHeader->sessionId = pXMaiSessionCtrl->fSessionInt;//������
    pXMaiMsgHeader->sequenceNum = pXMaiSessionCtrl->fPeerSeqNum;
    pXMaiMsgHeader->totalPacket = 0x00;
    pXMaiMsgHeader->curPacket = 0x00;
    pXMaiMsgHeader->messageId = messageid;
    pXMaiMsgHeader->dataLen = strlen(jsonMsg);

	if(pXMaiMsgHeader->dataLen + 20 > sizeof(pXMaiSessionCtrl->fSendbuf))
	{
		PRINT_ERR("send buff too small. send jsonMsg: %s\n", jsonMsg);	
		PRINT_ERR("send buf size = %d, send msg len = %d \n", sizeof(pXMaiSessionCtrl->fSendbuf), (pXMaiMsgHeader->dataLen + 20));
		return -1;
	}

    sprintf(pXMaiSessionCtrl->fSendbuf + 20, "%s", jsonMsg);
    xmaiMsgLen = 20 + strlen(jsonMsg);

    return xmaiMsgLen;
}

int XMaiMakeMsgUseBuf(char *fSendbuf, int buf_size, short messageid, char *jsonMsg, int fSessionInt, int fPeerSeqNum)
{
    int xmaiMsgLen = 0;
	memset(fSendbuf, 0, buf_size);
    XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)fSendbuf;
    
    pXMaiMsgHeader->headFlag = 0xFF;
    pXMaiMsgHeader->version = 0x01;
    pXMaiMsgHeader->reserved1 = 0x00;
    pXMaiMsgHeader->reserved2 = 0x01; //�°汾
    pXMaiMsgHeader->sessionId = fSessionInt;
    pXMaiMsgHeader->sequenceNum = fPeerSeqNum;
    pXMaiMsgHeader->totalPacket = 0x00;
    pXMaiMsgHeader->curPacket = 0x00;
    pXMaiMsgHeader->messageId = messageid;
    pXMaiMsgHeader->dataLen = strlen(jsonMsg);

	if(pXMaiMsgHeader->dataLen + 20 > buf_size)
	{
		//PRINT_ERR("send buff too small. send jsonMsg: %s\n", jsonMsg);			
		//PRINT_ERR("send buf size = %d, send msg len = %d \n", buf_size, (pXMaiMsgHeader->dataLen + 20));
		return -1;
	}

    sprintf(fSendbuf + 20, "%s", jsonMsg);
    xmaiMsgLen = 20 + strlen(jsonMsg);

    return xmaiMsgLen;
}

void XMaiPrintMsgHeader(char *buf)
{
	XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)buf;
	printf("headFlag = %d \n", pXMaiMsgHeader->headFlag);
	printf("version = %d \n", pXMaiMsgHeader->version);
	printf("reserved1 = %d \n", pXMaiMsgHeader->reserved1);
	printf("reserved2 = %d \n", pXMaiMsgHeader->reserved2);
	printf("sessionId = %d \n", pXMaiMsgHeader->sessionId);
	printf("sequenceNum = %d \n", pXMaiMsgHeader->sequenceNum);
	printf("totalPacket = %d \n", pXMaiMsgHeader->totalPacket);
	printf("curPacket = %d \n", pXMaiMsgHeader->curPacket);
	printf("messageId = %d \n", pXMaiMsgHeader->messageId);
	printf("dataLen = %d \n", pXMaiMsgHeader->dataLen);
}

int XMaiMakeMsgNoJsonMsg(XMaiSessionCtrl *pXMaiSessionCtrl, short messageid)
{
    int xmaiMsgLen = 0;

	memset(pXMaiSessionCtrl->fSendbuf, 0, sizeof(pXMaiSessionCtrl->fSendbuf));
    XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)(pXMaiSessionCtrl->fSendbuf);
    
    pXMaiMsgHeader->headFlag = 0xFF;
    pXMaiMsgHeader->version = 0x01;
    pXMaiMsgHeader->reserved1 = 0x00;
    pXMaiMsgHeader->reserved2 = 0x00;
    pXMaiMsgHeader->sessionId = pXMaiSessionCtrl->fSessionInt;//������
    pXMaiMsgHeader->sequenceNum = pXMaiSessionCtrl->fSequenceNum++;
    pXMaiMsgHeader->totalPacket = 0x00;
    pXMaiMsgHeader->curPacket = 0x00;
    pXMaiMsgHeader->messageId = messageid;
    pXMaiMsgHeader->dataLen = 0;

    xmaiMsgLen = 20;
    return xmaiMsgLen;
}


////////////////////////////////////////////////////////////////

int XMaiAddMediaMsgHeader(char *buf, short messageid, int dataLen, int session_id, int chnnel, int seq)
{
    XMaiMediaStreamMsgHeader *pXMaiMediaStreamMsgHeader = (XMaiMediaStreamMsgHeader *)buf;

    pXMaiMediaStreamMsgHeader->headFlag = 0xFF;
    pXMaiMediaStreamMsgHeader->version = 0x01;
    pXMaiMediaStreamMsgHeader->reserved1 = 0x00;
    pXMaiMediaStreamMsgHeader->reserved2 = 0x00;
    pXMaiMediaStreamMsgHeader->sessionId = session_id;
    pXMaiMediaStreamMsgHeader->sequenceNum = seq;
    pXMaiMediaStreamMsgHeader->channel = chnnel;
    pXMaiMediaStreamMsgHeader->endflag = 0x00;
    pXMaiMediaStreamMsgHeader->messageId = messageid;
    pXMaiMediaStreamMsgHeader->dataLen = dataLen;
    
    return 0;
}

int XMaiAddTalkMsgHeader(char *buf, short messageid, int dataLen, int session_id, int chnnel)
{
    XMaiMediaStreamMsgHeader *pXMaiMediaStreamMsgHeader = (XMaiMediaStreamMsgHeader *)buf;

    pXMaiMediaStreamMsgHeader->headFlag = 0xFF;
    pXMaiMediaStreamMsgHeader->version = 0x01;
    pXMaiMediaStreamMsgHeader->reserved1 = 0x00;
    pXMaiMediaStreamMsgHeader->reserved2 = 0x00;
    pXMaiMediaStreamMsgHeader->sessionId = session_id;
    pXMaiMediaStreamMsgHeader->sequenceNum = g_xmai_talk_send_index++;
    pXMaiMediaStreamMsgHeader->channel = chnnel;
    pXMaiMediaStreamMsgHeader->endflag = 0x00;
    pXMaiMediaStreamMsgHeader->messageId = messageid;
    pXMaiMediaStreamMsgHeader->dataLen = dataLen;
    
    return 0;
}
int XMaiAddPFrameHeader(char *dataBuf, int dataLen)
{
    VideoPFrameHeader *pPFrameH = (VideoPFrameHeader *)(dataBuf);
    //int dataLen = pstReadptr->stFrameHeader.dwVideoSize;
	
    pPFrameH->flag1 = 0x00;
    pPFrameH->flag2 = 0x00;
    pPFrameH->flag3 = 0x01;
    pPFrameH->flag4 = 0xFD;
    pPFrameH->len = dataLen;

    return 0;	
}


static int XMaiGetTimetick(int *timetick)
{
	if(timetick == NULL)
		return -1;

	XMaiTimetick *pXMaiTimetick = (XMaiTimetick *)timetick;

	long ts = time(NULL);
	struct tm tt = {0};
	struct tm *pTm = localtime_r(&ts, &tt);

	pXMaiTimetick->year = pTm->tm_year+1900-2000;
	pXMaiTimetick->month = pTm->tm_mon+1;
	pXMaiTimetick->day = pTm->tm_mday;
	pXMaiTimetick->hour = pTm->tm_hour;
	pXMaiTimetick->minute = pTm->tm_min;
	pXMaiTimetick->second = pTm->tm_sec;

#if 0
	char strTime[128] = {0};
	sprintf(strTime,"stTime: %d-%02d-%02d %02d:%02d:%02d", pXMaiTimetick->year, pXMaiTimetick->month, pXMaiTimetick->day, pXMaiTimetick->hour, pXMaiTimetick->minute, pXMaiTimetick->second);
	//printf("strTime:%s\n", strTime);
	//printf("timetick:%d\n", *timetick);
#endif	

	return 0;
}

int XMaiAddIFrameHeader(char *dataBuf, int dataLen, int width, int height, int videoFps)
{
    VideoIFrameHeader *pIFrameH = (VideoIFrameHeader *)(dataBuf);

	//int dataLen = pstReadptr->stFrameHeader.dwVideoSize;
	//int width = pstReadptr->stFrameHeader.wVideoWidth;
	//int height = pstReadptr->stFrameHeader.wVideoHeight;
	#if 1
	//int timetick = pstReadptr->stFrameHeader.dwTimeTick;
	int timetick;
	XMaiGetTimetick(&timetick);
	#else
	int timetick = 0;
	#endif
	
    pIFrameH->flag1 = 0x00;
    pIFrameH->flag2 = 0x00;
    pIFrameH->flag3 = 0x01;
    pIFrameH->flag4 = 0xFC;
    pIFrameH->flagT = 0x02;
    pIFrameH->framebit = videoFps;
    pIFrameH->width = width/8;
    pIFrameH->height = height/8;
    pIFrameH->time = timetick;
    pIFrameH->len = dataLen;

    return 0;
}

int XMaiAddAudioHeader(char *dataBuf, int dataLen)
{
    AudioDataHeader *pAudio = (AudioDataHeader *)(dataBuf);
   
    pAudio->flag1 = 0x00;
    pAudio->flag2 = 0x00;
    pAudio->flag3 = 0x01;
    pAudio->flag4 = 0xFA;
    pAudio->flagT = 0x0E;
    pAudio->sampleRate = 0x02;
    pAudio->len = dataLen;
    
    return 0; 	
}

int XMaiPrintAudioHeader(char *dataBuf)
{
    AudioDataHeader *pAudio = (AudioDataHeader *)(dataBuf);

	printf("Audio Frame Info:\n");
   	printf("flag1 = %d\n", pAudio->flag1);
	printf("flag2 = %d\n", pAudio->flag2);
	printf("flag3 = %d\n", pAudio->flag3);
	printf("flag4 = %d\n", pAudio->flag4);
	printf("flagT = %d\n", pAudio->flagT);
	printf("sampleRate = %d\n", pAudio->sampleRate);
	printf("len = %d\n", pAudio->len);
    
    return 0; 	
}

#if 1
static int XMaiCalSeq( GK_NET_FRAME_TYPE f_type, int frame_len)
{
    int send_len_once = XMAI_PACKET_SEND_LEN_ONCE; //�ְ����ͣ�һ�η���8K����

    int frame_head_len = 0;
    if (GK_NET_FRAME_TYPE_I == f_type)
        frame_head_len = 16; // sizeof(VideoIFrameHeader);
    else if (GK_NET_FRAME_TYPE_P == f_type)
        frame_head_len = 8;  // sizeof(VideoPFrameHeader);
    else if (GK_NET_FRAME_TYPE_A == f_type)
        frame_head_len = 8;  // sizeof(AudioDataHeader);

    int pack_num = 0;
	if(frame_len + frame_head_len <= send_len_once) //һ��������
	{
        pack_num ++;
		return pack_num; // 1�������꣬������ֱ���˳�
	}
	else //һ���������꣬�ȷ���һ��������һ������ MediaMsgHeader + FrameHeader, ����İ� ֻ�� MediaMsgHeader ���� seq ����
	{
        pack_num ++;
	}

    /* һ���������꣬�ȷ���һ������
     * ��һ������ MediaMsgHeader + FrameHeader + frame_data, 
     *  ����İ� ֻ�� MediaMsgHeader + frame_data, ���� seq ����
     */
    int all_len = frame_len - (send_len_once - frame_head_len);
    #if 0
    int send_len = 0;
    while(send_len < all_len)
	{
		//�ְ����ͣ�һ�η��� send_len_once ����
        if ( (all_len - send_len) >= send_len_once )
		{
			offset += send_len_once;
			send_len += send_len_once;			
        }
        else //���һ����
		{
			offset += (all_len - send_len);
			send_len += (all_len - send_len);	
        }
        pack_num ++;
    }
    #else
        #if 0
        int num = 0;
        if (all_len % send_len_once == 0)
            num = all_len / send_len_once;
        else
            num = all_len / send_len_once + 1;
        #else
        int num = (all_len + send_len_once - 1) / send_len_once;
        pack_num += num;
        #endif
    #endif

	return pack_num;
}



static int XMaiSendFrame(XMaiFrameInfo *pDataPacket, int sock, int sid, GK_NET_FRAME_TYPE f_type)
{
    int send_len_once = XMAI_PACKET_SEND_LEN_ONCE; //�ְ����ͣ�һ�η���8K����
    char media_head[40] = {0};
    int ret = 0;

    int seq = pDataPacket->seq;
    int channel = pDataPacket->channel;
    int width = pDataPacket->width;
    int height = pDataPacket->height;
    int fps = pDataPacket->fps;
    int frame_len = pDataPacket->frame_len;
    char *pData = pDataPacket->pData;
    
    int msg_head_len = 20; // sizeof(XMaiMediaStreamMsgHeader);
    int frame_head_len = 0;
    if (GK_NET_FRAME_TYPE_I == f_type)
        frame_head_len = 16; // sizeof(VideoIFrameHeader);
    else if (GK_NET_FRAME_TYPE_P == f_type)
        frame_head_len = 8;  // sizeof(VideoPFrameHeader);
    else if (GK_NET_FRAME_TYPE_A == f_type)
        frame_head_len = 8;  // sizeof(AudioDataHeader);

	if(frame_len + frame_head_len <= send_len_once) //һ��������
	{
		XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, frame_len + frame_head_len, sid, channel, seq);

        if (GK_NET_FRAME_TYPE_I == f_type)
            XMaiAddIFrameHeader(media_head + msg_head_len, frame_len, width, height, fps);
        else if (GK_NET_FRAME_TYPE_P == f_type)
            XMaiAddPFrameHeader(media_head + msg_head_len, frame_len);
        else if (GK_NET_FRAME_TYPE_A == f_type)
            XMaiAddAudioHeader(media_head + msg_head_len, frame_len);
        
		ret = XMaiTcpSelectSend(sock, media_head, msg_head_len + frame_head_len, 0);	
		if(ret < 1)
		{
			PRINT_ERR("data header send err, sock:%d, sid:%d", sock, sid);
			return -1;
		}

		ret = XMaiTcpSelectSend(sock, pData, frame_len, 0);	
		if(ret < 1)
		{
			PRINT_ERR("data body send err, sock:%d, sid:%d", sock, sid);
			return -1;
		}
        seq ++;
		return 0; // 1�������꣬������ֱ���˳�
	}
	else //һ���������꣬�ȷ���һ��������һ������ MediaMsgHeader + FrameHeader, ����İ� ֻ�� MediaMsgHeader ���� seq ����
	{
		XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, send_len_once, sid, channel, seq);

        if (GK_NET_FRAME_TYPE_I == f_type)
            XMaiAddIFrameHeader(media_head + msg_head_len, frame_len, width, height, fps);
        else if (GK_NET_FRAME_TYPE_P == f_type)
            XMaiAddPFrameHeader(media_head + msg_head_len, frame_len);
        else if (GK_NET_FRAME_TYPE_A == f_type)
            XMaiAddAudioHeader(media_head + msg_head_len, frame_len);
        
		ret = XMaiTcpSelectSend(sock, media_head, msg_head_len + frame_head_len, 0);
		if(ret < 1)
		{
			PRINT_ERR("data header send err, sock:%d, sid:%d", sock, sid);
			return -1;
		}

		ret = XMaiTcpSelectSend(sock, pData, send_len_once - frame_head_len, 0);	
		if(ret < 1)
		{
			PRINT_ERR("data body send err, sock:%d, sid:%d", sock, sid);
			return -1;
		}
        seq ++;
	}

    /* һ���������꣬�ȷ���һ������
     * ��һ������ MediaMsgHeader + FrameHeader + frame_data, 
     *  ����İ� ֻ�� MediaMsgHeader + frame_data, ���� seq ����
     */
    int offset = send_len_once - frame_head_len;
    int all_len = frame_len - offset;
    int send_len = 0;
    while(send_len < all_len)
	{
		//�ְ����ͣ�һ�η��� send_len_once ����
        if ( (all_len - send_len) >= send_len_once )
		{
			memset(media_head, 0, sizeof(media_head));
            XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, send_len_once, sid, channel, seq);
			ret = XMaiTcpSelectSend(sock, media_head, msg_head_len, 0);	
			if(ret < 1)
			{
				PRINT_ERR("data header send err, sock:%d, sid:%d", sock, sid);
				return -1;
			}
			ret = XMaiTcpSelectSend(sock, pData + offset, send_len_once, 0);	
			if(ret < 1)
			{
				PRINT_ERR("data body send err, sock:%d, sid:%d", sock, sid);
				return -1;
			}	
			
			offset += send_len_once;
			send_len += send_len_once;			
        }
        else //���һ����
		{
			memset(media_head, 0, sizeof(media_head));
            XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, all_len - send_len, sid, channel, seq);
			ret = XMaiTcpSelectSend(sock, media_head, msg_head_len, 0);	
			if(ret < 1)
			{
				PRINT_ERR("data header send err, sock:%d, sid:%d", sock, sid);
				return -1;
			}
			ret = XMaiTcpSelectSend(sock, pData + offset, all_len - send_len, 0);	
			if(ret < 1)
			{
				PRINT_ERR("data body send err, sock:%d, sid:%d", sock, sid);
				return -1;
			}	
			
			offset += (all_len - send_len);
			send_len += (all_len - send_len);	
        }
        seq ++;
    }

	return 0;
}

static int XMaiArraySendStreamPacket(char *frame_data, int frame_len, GK_NET_FRAME_HEADER *pFrameHead, int stream_type)	
{
    int ret = 0;

    XMaiFrameInfo frame_info = {0};
    frame_info.seq = g_xmai_data_seq_num;
    frame_info.channel = 0;
    frame_info.width = (pFrameHead->video_reso >> 16) & 0xFFFF;
    frame_info.height = pFrameHead->video_reso & 0xFFFF;
    frame_info.fps = pFrameHead->frame_rate;
    frame_info.frame_len = frame_len;
    frame_info.pData = frame_data;

    int is_wait_i;
    int sock = -1;
    int sid = 0;
    int i = 0;
    for (i = 0; i < UT_MAX_LINK_NUM; i++) {
		sock = UtGetLvSockByIndex(i, stream_type, &g_xmai_session);
        if (sock > 0) {
            //�ж��Ƿ�ȴ�I֡
            is_wait_i = UtGetLvSockWaitIByIndex(i, stream_type, &g_xmai_session);		
            if (is_wait_i) {
                if (pFrameHead->frame_type != GK_NET_FRAME_TYPE_I)
                    continue;
                else
                    UtSetLvSockWaitIByIndex(i, stream_type, 0, &g_xmai_session);
            }
            //��ȡsid
            sid = UtGetSidByIndex(i, &g_xmai_session);

            ret = XMaiSendFrame(&frame_info, sock, sid, pFrameHead->frame_type);
            //�������ʧ�ܣ���رն�Ӧ��data sock
            if (ret <0) {
                UtCloseLvSockByIndex(i, stream_type, &g_xmai_session);
                continue;
            }
        }
    }

    int pack_num = XMaiCalSeq(pFrameHead->frame_type, frame_len);
    g_xmai_data_seq_num += pack_num;

    return 0;
}


#if 0
static int XMaiArraySendStream(char *frame_data, int frame_len, GK_NET_FRAME_HEADER *pFrameHead, int stream_type)	
{
    int i = 0;
    int sock = 0;
    int ret = 0;
    int is_wait_i;
    char media_head[40] = {0};
    int session_id = 0;

    int seq = g_xmai_data_seq_num ++;
    
    for (i = 0; i < UT_MAX_LINK_NUM; i++) {
		sock = UtGetLvSockByIndex(i, stream_type, &g_xmai_session);
        if (sock > 0) {
            //�ж��Ƿ�ȴ�I֡
            is_wait_i = UtGetLvSockWaitIByIndex(i, stream_type, &g_xmai_session);		
            if (is_wait_i) {
                if (pFrameHead->frame_type != GK_NET_FRAME_TYPE_I)
                    continue;
                else
                    UtSetLvSockWaitIByIndex(i, stream_type, 0, &g_xmai_session);
            }
            //��ȡsid
            session_id = UtGetSidByIndex(i, &g_xmai_session);

            /* ����ÿһ֡���� */
            if (pFrameHead->frame_type == GK_NET_FRAME_TYPE_I)
            {   
                memset(media_head, 0, sizeof(media_head));
                XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, frame_len + 16, session_id, 0, seq);
                int width = (pFrameHead->video_reso >> 16) & 0xFFFF;
                int height = pFrameHead->video_reso & 0xFFFF;
                PRINT_INFO("stream%d -- [%dX%d]  fps:%d seq:%d", stream_type, width, height, pFrameHead->frame_rate, seq);
                XMaiAddIFrameHeader(media_head + 20, frame_len, width, height, pFrameHead->frame_rate);
            
                ret = XMaiTcpSelectSend(sock, media_head, 20+16, 0);
                if (ret > 0) {
                    ret = XMaiTcpSelectSend(sock, frame_data, frame_len, 0);
                }
            }
            else if (pFrameHead->frame_type == GK_NET_FRAME_TYPE_P)
            {
                memset(media_head, 0, sizeof(media_head));
                XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, frame_len + 8, session_id, 0, seq);
                XMaiAddPFrameHeader(media_head + 20, frame_len, pFrameHead->frame_rate);
            
                ret = XMaiTcpSelectSend(sock, media_head, 20+8, 0);
                if (ret > 0) {
                    ret = XMaiTcpSelectSend(sock, frame_data, frame_len, 0);
                }
                //printf("P frame, size = %lu, ret = %d\n", frame_len, ret);
            }
            else if (pFrameHead->frame_type == GK_NET_FRAME_TYPE_A)//��Ƶ֡
            {           
                memset(media_head, 0, sizeof(media_head));
                XMaiAddMediaMsgHeader(media_head, MONITOR_DATA, frame_len + 8, session_id, 0, seq);
                XMaiAddAudioHeader(media_head + 20, frame_len);
            
                ret = XMaiTcpSelectSend(sock, media_head, 20+8, 0);
                if (ret > 0) {
                    ret = XMaiTcpSelectSend(sock, frame_data, frame_len, 0);
                }
            }

            //�������ʧ�ܣ���رն�Ӧ��data sock
            if (ret <0) {
                UtCloseLvSockByIndex(i, stream_type, &g_xmai_session);
                continue;
            }
        }
    }

    return 0;
}
#endif

static int XMaiReadSendStream(MEDIABUF_HANDLE reader, int stream_type)
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
		PRINT_ERR("cmd read stream%d : mediabuf_read_frame fail.\n", stream_type);
		return -1;
    } else if (ret == 0) {
        PRINT_INFO("mediabuf_read_frame = 0\n");
        return 0;
    } else {
        //PRINT_INFO("cmd read stream%d : mediabuf_read_frame OK.\n", tBST);
    }

    XMaiArraySendStreamPacket(frame_data, frame_len, &frame_header, stream_type);

    return 0;
}


static int XMaiSendStreamLoop(int stream_id)
{
    MEDIABUF_HANDLE reader = mediabuf_add_reader(stream_id);
    if (reader == NULL) {
        PRINT_ERR("Add reader to stream%d error.\n", stream_id);
        return -1;
    }

    int ret = 0;

    while (1) {
        if (g_xmai_pthread_run == 0)
            break;

        //����Ƿ��пͻ������ӵ�cms�����û����ȴ�
		if(UtGetStreamOpenNum(stream_id, &g_xmai_session) == 0) {
            //PRINT_INFO("no online open, so not to send stream%d\n", stream_id);
            usleep(300*1000);
            continue;
        }

        /* 1 ��ȡ���� 2 ���ʹ��� */
        //if (send_data)
            ret = XMaiReadSendStream(reader, stream_id);
    }

    mediabuf_del_reader(reader);

    return 0;
}


/*************************************************************
* �������ܣ����͸����������߳�
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
void * XMaiSendMainStreamPthread(void *para)
{
    XmaiSetThreadName("XMaiSendMainStreamPthread");

    XMaiSendStreamLoop(UT_MAIN_STREAM);

    PRINT_INFO("XMaiSendMainStreamPthread end !\n");
    return NULL;
}

/*************************************************************
* �������ܣ����������������߳�
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
void * XMaiSendSubStreamPthread(void *para)
{
    XmaiSetThreadName("XMaiSendSubStreamPthread");

    XMaiSendStreamLoop(UT_SUB_STREAM);

    PRINT_INFO("XMaiSendSubStreamPthread end !\n");	
    return NULL;    
}

#endif

//////////////////////////////////////////////////
/*************************************************************
* �������ܣ���socket��������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
static int XMaiRecvMsg(XMaiSessionCtrl *pXMaiSessionCtrl, char *msg, int msg_size)
{
	int recv_len = 0;
	memset(msg, 0, msg_size);

	/* ����msg ͷ */
	int head_len = sizeof(XMaiMsgHeader);
	recv_len = XMaiSockRecv(pXMaiSessionCtrl->accept_sock, msg, head_len);
	if(recv_len < 0) //���ճ�����߶���ر���socket����ȥ�ر�sock
	{
		//PRINT_ERR();
		return -1;
	}

	if(recv_len != head_len)
	{
		PRINT_ERR();
		return -1;		
	}

	/* ��msg ͷ�еõ������Ϣ */
	if(msg[0] != 0xFF) //ȱ��ǿ׳�ľ���
	{
		PRINT_ERR();
		return -1;		
	}	

	/* ����msg ���� */
	XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)msg;
	int msg_len = pXMaiMsgHeader->dataLen;
	//PRINT_INFO("sock:%d CMD: %d\n", pXMaiSessionCtrl->accept_sock, pXMaiMsgHeader->messageId);
	if (msg_len + 20 > msg_size) //����buff̫С��msg��Ϣ̫��������
	{
		PRINT_ERR();
		return -1;		
	}	

	if(pXMaiMsgHeader->dataLen > 0)
	{

		recv_len = XMaiSockRecv(pXMaiSessionCtrl->accept_sock, msg + 20, msg_len);
		if(recv_len < 0) //���ճ�����߶���ر���socket����ȥ�ر�sock
		{
			//PRINT_ERR();
			return -1;
		}		

		if(recv_len != msg_len)
		{
			PRINT_ERR();
			return -1;		
		}	
	}

	return 0;		
}


/*************************************************************
* �������ܣ���socket��������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
static int XMaiHandleMsg(XMaiSessionCtrl *pXMaiSessionCtrl, char *msg, int msg_size)
{
	int ret = 0;
	/* �ӷ������İ��еõ� fPeerSeqNum �� sessionId */
	XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)msg;
	pXMaiSessionCtrl->fPeerSeqNum = pXMaiMsgHeader->sequenceNum;
	if(pXMaiMsgHeader->sessionId != 0) //��socket��һ�����ӣ���Ϊ0
	{
		pXMaiSessionCtrl->fSessionInt = pXMaiMsgHeader->sessionId;
		sprintf(pXMaiSessionCtrl->fSessionId, "0x%08x", pXMaiMsgHeader->sessionId);
	}

	//PRINT_INFO("[sock:%d] [sid:%d] [cmd:%d] ", 
    //    pXMaiSessionCtrl->accept_sock, pXMaiMsgHeader->sessionId, pXMaiMsgHeader->messageId);

#if 1
	//CONFIG_CHANNELTILE_DOT_SET ������
	if (pXMaiMsgHeader->messageId != CONFIG_CHANNELTILE_DOT_SET)
	{
		/* ��ӡMSGͷ��Ϣ(20���ֽ�) */
		//XMaiPrintMsgHeader(msg);

		/* ��ӡ���յ���Ϣ */
		if(pXMaiMsgHeader->dataLen > 0)
		{		
		    cJSON *json = NULL;    
		    char *out = NULL;
			json = cJSON_Parse(msg + 20);
		    if (!json){
		        PRINT_ERR("Error before: [%s]\n", cJSON_GetErrorPtr());
		        return -1;
		    }    
		    out = cJSON_Print(json);

		    PRINT_INFO("[sock:%d] [sid:%d] [cmd:%d] recv -- %s\n", 
                pXMaiSessionCtrl->accept_sock, pXMaiMsgHeader->sessionId, pXMaiMsgHeader->messageId, out);

            cJSON_Delete(json);
		    free(out);	
		}
		else
		{
		    if (pXMaiMsgHeader->messageId != KEEPALIVE_REQ)
            {      
			    PRINT_INFO("[sock:%d] [sid:%d] [cmd:%d] recv -- msg_len is 0. \n", 
                    pXMaiSessionCtrl->accept_sock, pXMaiMsgHeader->sessionId, pXMaiMsgHeader->messageId);
            }
		}
	}
    else
    {
		PRINT_ERR("[sock:%d] [sid:%d] cmd:%d is CONFIG_CHANNELTILE_DOT_SET. \n", 
            pXMaiSessionCtrl->accept_sock, pXMaiMsgHeader->sessionId, pXMaiMsgHeader->messageId);
    }
#endif

	/* ���� msg ���� */
	switch(pXMaiMsgHeader->messageId)
	{
		case LOGIN_REQ:
			ret = ReqRegisterHandle(pXMaiSessionCtrl, msg);
			break;

        case KEEPALIVE_REQ:
            ret = ReqkeepAliveHandle(pXMaiSessionCtrl, msg);
            break;

        case MONITOR_CLAIM:
            ret = ReqMonitorClaimHandle(pXMaiSessionCtrl, msg);
            break;
			
        case MONITOR_REQ:
            ret = ReqMonitorReqHandle(pXMaiSessionCtrl, msg);
            break;

		//��¼��NVR�����豸½������������
        case SYSINFO_REQ:
            ret = ReqSysinfoHandle(pXMaiSessionCtrl, msg);
            break;
		case GUARD_REQ:
			ret = ReqGuardHandle(pXMaiSessionCtrl, msg);
			break;
        case CONFIG_CHANNELTILE_GET:
            ret = ReqChannelTileGetHandle(pXMaiSessionCtrl, msg);
            break;

		case ABILITY_GET:
			ret = ReqAbilityGetHandle(pXMaiSessionCtrl, msg);
			break;

		case CONFIG_GET:
			ret = ReqConfigGetHandle(pXMaiSessionCtrl, msg);
			break;

		case CONFIG_SET:
			ret = ReqConfigSetHandle(pXMaiSessionCtrl, msg);
			break;		

        case CONFIG_CHANNELTILE_SET:
            ret = ReqChannelTileSetHandle(pXMaiSessionCtrl, msg);
            break;

        case CONFIG_CHANNELTILE_DOT_SET:
            ret = ReqChannelTileDotSetHandle(pXMaiSessionCtrl, msg);
            break;			

        case TIMEQUERY_REQ:
            ret = ReqTimeQueryHandle(pXMaiSessionCtrl, msg);
            break;

        case SYNC_TIME_REQ:
            ret = ReqSyncTimeHandle(pXMaiSessionCtrl, msg);
            break;

		case SET_IFRAME_REQ:
			ret = ReqSetIframeHandle(pXMaiSessionCtrl, msg);
			break;
			
		case SYSMANAGER_REQ:
			ret = ReqSysManagerHandle(pXMaiSessionCtrl, msg);
			break;

		case USERS_GET:
			ret = ReqUsersGetHandle(pXMaiSessionCtrl, msg);
			break;

		case GROUPS_GET:
			ret = ReqGroupsGetHandle(pXMaiSessionCtrl, msg);
			break;

		case FULLAUTHORITYLIST_GET:
			ret = ReqFullAuthorityListHandle(pXMaiSessionCtrl, msg);
			break;			
            
        case PTZ_REQ:
            ret = ReqPtzHandle(pXMaiSessionCtrl, msg);
            break;

        case TALK_CLAIM:
            ret = ReqTalkClaimHandle(pXMaiSessionCtrl, msg);
            break;	

        case TALK_REQ:
            ret = ReqTalkReqHandle(pXMaiSessionCtrl, msg);
            break;			
                                               
        default:
            PRINT_ERR("unknown msg id:%d, sock:%d, sessionType:%d ",
                pXMaiMsgHeader->messageId, pXMaiSessionCtrl->accept_sock, pXMaiSessionCtrl->fSessionType);
			XMaiPrintMsg(msg);
            break;
    }



	if(ret < 0)
		return -1;
	
    return 0;		
}



/*************************************************************
* �������ܣ��Ự�̴߳���
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
static int XMaiSessionProcess(int n_socket)
{
	//printf("XMaiSessionProcess, sock = %d\n", n_socket);
    int ret = 0;	
	char msg[XMAI_MSG_RECV_BUF_SIZE] = {0};

	XMaiSessionCtrl stXMaiSessionCtrl;
	memset(&stXMaiSessionCtrl, 0, sizeof(stXMaiSessionCtrl));
	stXMaiSessionCtrl.accept_sock = n_socket;
	stXMaiSessionCtrl.fSessionType = UNINIT_SESSION;

    //�������Ӻ�һֱѭ����������
    while(1)
    {
        if(0 == g_xmai_pthread_run)
        {
            break;
        }

		//����MSG��ͷ������	
		//PRINT_DELIMITER();
		//PRINT_INFO("recv msg, sock:%d", n_socket);
		ret = XMaiRecvMsg(&stXMaiSessionCtrl, msg, sizeof(msg));
		if(ret < 0) //������߶Է��ر���socket
		{			
			break;
		}

	
		//�����Ӧ msg ��
		//PRINT_INFO("handle msg, sock:%d", n_socket);
		ret = XMaiHandleMsg(&stXMaiSessionCtrl, msg, sizeof(msg));
		#if 1
		/* ���µ�Э�������¼�����������ʱ������ */
		if(ret < 0)
		{
			PRINT_ERR("XMaiHandleMsg error.\n");
			//break;
		}
		#endif
		
		//�������������������̴߳�����Ϳ��Թر���
		if(stXMaiSessionCtrl.fSessionType == CONNECT_SESSION)
		{
			//ֱ�ӷ��أ����ر� socket
			return 0;
		}
    }  

    ret = UtCloseCmdSock(n_socket, &g_xmai_session);
    if ((ret < 0) && (n_socket > 0)) {
        close(n_socket);        
    }

    return 0;	
	
}

/*************************************************************
* �������ܣ��Ự�߳�
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
void *XMaiSessionThread(void *param)
{
    XmaiSetThreadName("XMaiSessionThread");
    
	XMaiSessionProcess((int)param);

	PRINT_INFO("XMai_session_Thread end, sock:%d !\n", (int)param);
	return NULL;
}




/*************************************************************
* �������ܣ���¼nvr��Ȼ����ܴ�nvr���͹���������
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
void * XMaiMainThread(void *param)
{ 
	DEGLINE();

    XmaiSetThreadName("XMaiMainThread");
	
    //��ʼ���Ự����socket����
    UtSessionGlobalInit(&g_xmai_session);
	
//    pthread_mutex_init(&xmai_main_second_stream_mutex,NULL);
    pthread_mutex_init(&g_xmai_audio_mutex,NULL);  

    //���������Ự��socket
    int fListenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (fListenfd < 0) 
    {
        PRINT_ERR("socket error%s\n", strerror(errno));
        goto end;
    }

	struct sockaddr_in t_serv_addr, t_clnt_addr;
	
    /*  ���ü����Ự��socket */
	bzero(&t_serv_addr, sizeof(t_serv_addr));
    t_serv_addr.sin_family = AF_INET;
    t_serv_addr.sin_port = htons(XMAI_TCP_LISTEN_PORT);
    t_serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	
    /* �󶨼����Ự��socket */
    if (bind(fListenfd, (struct sockaddr *)&t_serv_addr, sizeof(struct sockaddr)) < 0) 
    {
        PRINT_ERR("bind err%s\n", strerror(errno));
        goto end;
    }
	
    /* ����socketתΪ��������socket */
	if (listen(fListenfd, XMAI_MAX_LINK_NUM) < 0) 
    {
        PRINT_ERR("listen error%s\n", strerror(errno));
        goto end;
    }

	DEGLINE();
	
    /* ѭ���������ȴ�nvr�������� */
    int n_len = sizeof(t_clnt_addr);
    while (1) 
    {
		//DEGLINE();
        int n_peer;
        //PRINT_INFO("begin to accept\n");
        n_peer = accept(fListenfd, (struct sockaddr *)&t_clnt_addr, (socklen_t *)&n_len);/* �������� */
        if(0 == g_xmai_pthread_run)
        {
            break;
        }
        if (n_peer < 0) 
        {
            PRINT_ERR("Server: accept failed%s\n", strerror(errno));
            break;
        }

		PRINT_DELIMITER();
        PRINT_INFO("XMaiMainThread,accept sockfd :%d, remote ip:%s, remote port:%d\n", 
                    n_peer, inet_ntoa(t_clnt_addr.sin_addr), ntohs(t_clnt_addr.sin_port));
		
		//DEGLINE();
        //�����Ự�̣߳�ÿ��һ��nvr���Ӿʹ���һ���Ự�߳�
        XMaiCreateThread(XMaiSessionThread, (void *)n_peer, NULL);

    }

end: 
    PRINT_INFO("XMaiMainThread end !\n");
      
	if(fListenfd > 0)
	{
    	close(fListenfd); 
		fListenfd = -1;
	}
	UtCloseAllSock(&g_xmai_session);
    
    //pthread_mutex_destroy(&xmai_main_second_stream_mutex); 
    pthread_mutex_destroy(&g_xmai_audio_mutex); 
    return NULL;
}



////////////////////////////////////////////////////////



int XMaiPrintMsg(char *msg)
{
	// ��ӡMSGͷ��Ϣ(20���ֽ�)
	XMaiPrintMsgHeader(msg);	

	XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)msg;
    cJSON *json = NULL;
    char *out = NULL;

    if(pXMaiMsgHeader->dataLen > 0)
	{	
	    json = cJSON_Parse(msg + 20);
	    if (!json)
		{
	        PRINT_ERR("Error before: [%s]\n", cJSON_GetErrorPtr());
	        return -1;
	    }
	    
	    out = cJSON_PrintUnformatted(json);
		printf("%s\n",out);
		
	    cJSON_Delete(json);
	    free(out);
    }	
	else
	{
		printf("msg data len is 0\n");
	}
	return 0;
}

#if 1

static int XMaiCreateBroadcastSock()
{
    int ret = 0;    
	int opt;

	//�õ����׽���
    int sockSvr = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockSvr < 0)
	{
        PRINT_ERR("socket fail, error:%s\n", strerror(errno));
        goto cleanup;
    }    

#if 0
	//���ø��׽���
    opt = 1;
    ret = setsockopt(sockSvr, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if(ret < 0){
        PRINT_ERR("setsockopt  SO_REUSEADDR fail, error:%s\n", strerror(errno));
        goto cleanup;
    }
#endif

#if 0
	//���ø��׽��ֽ��ճ�ʱ�˳�(UDP����û�г�ʱ)
	struct timeval timeout;
	timeout.tv_sec = 10; // 10��
	timeout.tv_usec = 0;
	if(setsockopt(sockSvr, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval)) < 0)
	{
        PRINT_ERR("setsockopt SO_RCVTIMEO fail, error:%s\n", strerror(errno));
        goto cleanup;
	}
#endif	

	//���ø��׽���Ϊ�㲥���� 
    opt = 1;
    ret = setsockopt(sockSvr, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    if(ret < 0)
	{
        PRINT_ERR("setsockopt SO_BROADCAST fail, error:%s\n", strerror(errno));
        goto cleanup;
    }

	struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(XMAI_BROADCAST_RCV_PORT); 
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

	//�󶨸��׽���
    ret = bind(sockSvr, (struct sockaddr *)&addr, sizeof(addr));
    if(ret < 0)
	{
        PRINT_ERR("bind  fail, error:%s\n", strerror(errno));
        goto cleanup;
    }

#if 0
	//���ø��׽���Ϊ������
    curFlags = fcntl(sockSvr, F_GETFL, 0);
    curFlags = fcntl(sockSvr, F_SETFL, curFlags |O_NONBLOCK);	
#endif

    return sockSvr;

cleanup:
    if(sockSvr >= 0)
        close(sockSvr);
    return -1;	
}

int XMaiBoardCastSendTo(int fSockSearchfd, const char* inData, const int inLength)
{
	int ret = 0;
	int sendlen = 0;
    struct sockaddr_in addr;

    memset(&addr, 0,sizeof(&addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    addr.sin_port = htons(XMAI_BROADCAST_SEND_PORT);

	//��ΪUDP ����ʱ��û�л�����������Ҫ�Է��ͺ�ķ���ֵ�����жϺ󣬶�η���
	while(sendlen < inLength)
	{
	    ret = sendto(fSockSearchfd, inData + sendlen, inLength - sendlen, 0, (struct sockaddr*)&addr,  sizeof(addr));
		if(ret < 0)
		{
			//perror("Send error");
			//���������� EAGAIN
			if (errno != EINTR && errno != EAGAIN )
			{
				PRINT_ERR("Send() socket %d error :%s\n", fSockSearchfd, strerror(errno));
				return -1;
			}
			else
				continue;
		}

		sendlen += ret;
	}

	PRINT_INFO("ip search SendLen: %d\n", sendlen);
	
	return sendlen;
}


static int XMaiSearchMsgParse(int fSockSearchfd, char *msg)
{
	//XMaiPrintMsgHeader(msg);
    XMaiMsgHeader *pXMaiMsgHeader = (XMaiMsgHeader *)msg;

    PRINT_INFO("=====msg  id:%d=======\n", pXMaiMsgHeader->messageId);
    PRINT_INFO("=====dataLen:%d=======\n", pXMaiMsgHeader->dataLen);
	switch(pXMaiMsgHeader->messageId)
	{
		case IPSEARCH_REQ:
			ReqIpsearchHandle(fSockSearchfd, msg);
			break;

		case IPSEARCH_RSP:  // �㲥��õ�������
		    PRINT_INFO("IPSEARCH_RSP\n");
			//ReqIpsearchRspHandle(fSockSearchfd, msg);
			break;

		case IP_SET_REQ:  
			ReqIpSetReqHandle(fSockSearchfd, msg);
			break;			

		case IP_SET_RSP:  			
			break;	
			 				  
        default:
            PRINT_ERR("*** udp, unknown msg id:%d ***\n", pXMaiMsgHeader->messageId);
			// ��ӡMSG��Ϣ
			XMaiPrintMsg(msg);
            break;
    }
    
    return 0;
}

static void XMaiSearchHandleLoop(int fSockSearchfd)
{
    int recvLen = 0;
    int packetLen = 0;    
    XMaiMsgHeader *pXMaiMsgHeader = NULL;
	char bufRecv[4096] = {0};
    // �㲥��ַ  
    struct sockaddr_in from;
	
	char *p = NULL;
	char buf[4096] = {0};
	
	int len = 0;
    while(1)  
    {  
        if(0 == g_xmai_pthread_run)
        {
            break;
        }
#if 0
		// ��ʼ����0
		memset(bufRecv, 0, sizeof(bufRecv));
		bzero(&from, sizeof(struct sockaddr_in));
		
		//���շ�������Ϣͷ, ֮ǰ�����˸�UDP�׽��ֽ���10�볬ʱ�˳�(UDP����û�г�ʱ)
		len = sizeof(struct sockaddr_in); 
		recvLen = recvfrom(fSockSearchfd, bufRecv, sizeof(XMaiMsgHeader), 0, (struct sockaddr *)&from, (socklen_t *)&len);
		if(recvLen <= 0)
		{
		    PRINT_ERR("recvfrom msg header error:%s\n", strerror(errno));
		    continue;
		}
		//printf("\n");
		XMaiPrintMsgHeader(bufRecv);
		//��ӡ���յ��ĵ�ַ��Ϣ
		PRINT_INFO("search from ip:%s, port:%d\n", (char *)inet_ntoa(from.sin_addr), ntohs(from.sin_port));	
		
		if(recvLen != sizeof(XMaiMsgHeader))
		{
		    PRINT_ERR("recvfrom len = %d, msg header len = %d\n", recvLen, sizeof(XMaiMsgHeader));
		    continue;			
		}
		// �ж��Ƿ������İ�ͷ
		pXMaiMsgHeader = (XMaiMsgHeader *)bufRecv;
		if(pXMaiMsgHeader->headFlag != 0xFF)
		{
		    PRINT_ERR("get msg is not xmai msg.\n");
		    continue;			
		}
 
		// �жϽ���buf ��С�Ƿ��㹻
		packetLen = 20 + pXMaiMsgHeader->dataLen;
		if(packetLen > sizeof(bufRecv))
		{
		    PRINT_ERR("receive buf is too small.\n");
		    continue;			
		}

		if(pXMaiMsgHeader->dataLen != 0)
		{
			//printf("dataLen = %d \n", pXMaiMsgHeader->dataLen);
			//���շ�������Ϣ����
			recvLen = recvfrom(fSockSearchfd, bufRecv + sizeof(XMaiMsgHeader), pXMaiMsgHeader->dataLen, 0, (struct sockaddr *)&from, (socklen_t *)&len);
			if(recvLen <= 0)
			{
			    PRINT_ERR("recvfrom msg header error:%s\n", strerror(errno));
			    continue;
			}	
			if(recvLen != pXMaiMsgHeader->dataLen)
			{
			    PRINT_ERR("recvfrom len = %d, msg data len = %d\n", recvLen, pXMaiMsgHeader->dataLen);
			    //continue;			
			}
			//printf("get datelen = %d\n", recvLen);
		}

		//��ӡ���յ��ĵ�ַ��Ϣ
		PRINT_INFO("search from ip:%s, port:%d\n", (char *)inet_ntoa(from.sin_addr), ntohs(from.sin_port));	

		//�������յ������ݰ������������ͺͽ����õ�ͬһ��fSockSearchfd
		XMaiSearchMsgParse(fSockSearchfd, bufRecv);		
#else	
		// ��ʼ����0
		memset(bufRecv, 0, sizeof(bufRecv));
		bzero(&from, sizeof(struct sockaddr_in));
		 
		//���շ�������Ϣ
		len = sizeof(struct sockaddr_in);

		recvLen = recvfrom(fSockSearchfd, bufRecv, sizeof(bufRecv)-1, 0, (struct sockaddr *)&from, (socklen_t *)&len);
		if(recvLen < 0)
		{
		    PRINT_ERR("recvfrom error:%s\n", strerror(errno));
		    break;
		}

		//��ӡ���յ��ĵ�ַ��Ϣ
		PRINT_DELIMITER();
		PRINT_INFO("search from ip:%s, port:%d\n", (char *)inet_ntoa(from.sin_addr), ntohs(from.sin_port));	

		p = bufRecv;
		while(p != (bufRecv + recvLen))
		{
		    if(p[0] == 0xFF)//ȱ��ǿ׳�ľ���
		    {
		        pXMaiMsgHeader = (XMaiMsgHeader *)p;
		        packetLen = 20 + pXMaiMsgHeader->dataLen;
		        memset(buf, 0, sizeof(buf));
		        memcpy(buf, p, packetLen);
				//������Ϣ��
		        XMaiSearchMsgParse(fSockSearchfd, buf);
		        p += packetLen;
		    }
		    else
		    {
		        p++;
		    }

		}
#endif		
    }
	
    return ;
}


/*************************************************************
* �������ܣ�����nvr�ķ������������豸�㲥��
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
void * XMaiBroadcastThread(void *param)
{
    XmaiSetThreadName("XMaiBroadcastThread");

	int fSockSearchfd = XMaiCreateBroadcastSock();
	if(fSockSearchfd < 0)
	{
        PRINT_ERR("Create Broadcast Sock error.\n");
        return NULL; 		
	}
	PRINT_INFO("Broadcast Socket:%d \n", fSockSearchfd);

	XMaiSearchHandleLoop(fSockSearchfd);

	if(fSockSearchfd > 0)
	{
		// ���� shutdown
		char send_msg[] = "shutdown";
    	int send_len = strlen(send_msg);
		XMaiBoardCastSendTo(fSockSearchfd, send_msg, send_len);

		//�ر�socket
		close(fSockSearchfd);		
	}	
     
    PRINT_INFO("XMaiBroadcastThread end !\n");
    return NULL;  
}  

#endif

/*************************************************************
* �������ܣ������ص�����
* ���������
* ���������
* ����ֵ  ��
* ��ע    ��1:$�ⲿ$����2:$��̬���$����3:$��Ƶ��ʧ$����4:$�ڵ����$����
*************************************************************/
void fXmaiNetAlarmCallback(int nChannel, int nAlarmType, int nAction, void* pParam)
{
    PRINT_INFO("nAlarmType:%d, nAction:%d\n", nAlarmType, nAction);

	if(nAction == 0)
	{
		PRINT_INFO("\n");
		return;
	}

    char eventType[20] = {0};
    char startTime[128] = {0};
	char strAction[20] = {0};
	
    long ts = time(NULL); 
    struct tm tt = {0}; 
    struct tm *pTm = localtime_r(&ts, &tt);

    sprintf(startTime,"%d-%02d-%02d %02d:%02d:%02d",pTm->tm_year+1900,pTm->tm_mon+1,pTm->tm_mday,pTm->tm_hour,pTm->tm_min,pTm->tm_sec);
	
	#if 0
    switch(nAlarmType)
    {
        DMS_ALARM_TYPE_ALARMIN = 0,     //0:�ź���������ʼ
        DMS_ALARM_TYPE_DISK_FULL,       //1:Ӳ����
        DMS_ALARM_TYPE_VLOST,           //2:�źŶ�ʧ
        DMS_ALARM_TYPE_VMOTION,         //3:�ƶ����
        DMS_ALARM_TYPE_DISK_UNFORMAT,   //4:Ӳ��δ��ʽ��
        DMS_ALARM_TYPE_DISK_RWERR,      //5:��дӲ�̳���,
        DMS_ALARM_TYPE_VSHELTER,        //6:�ڵ�����
        DMS_ALARM_TYPE_VFORMAT_UNMATCH, //7:��ʽ��ƥ�� 
        DMS_ALARM_TYPE_ILLEGAL_ACCESS,  //8:�Ƿ�����
        DMS_ALARM_TYPE_ALARMIN_RESUME,  //9:�ź��������ָ�
        DMS_ALARM_TYPE_VLOST_RESUME,    //10:��Ƶ��ʧ�����ָ�
        DMS_ALARM_TYPE_VMOTION_RESUME,  //11:��Ƶ�ƶ���ⱨ���ָ�
        DMS_ALARM_TYPE_NET_BROKEN,      //12:����Ͽ�
        DMS_ALARM_TYPE_IP_CONFLICT,     //13:IP��ͻ
    }
	#else
    switch(nAlarmType)
    {
        case DMS_ALARM_TYPE_ALARMIN:     //0:�ź���������ʼ
        	sprintf(eventType ,"%s", "LocalIO");
			sprintf(strAction ,"%s", "Start");
			break;
			
        case DMS_ALARM_TYPE_DISK_FULL:       //1:Ӳ����
			sprintf(eventType ,"%s", "StorrageLowSpace");  
			break;    
			
        case DMS_ALARM_TYPE_VLOST:            //2:�źŶ�ʧ
        	sprintf(eventType ,"%s", "VideoLoss");
			sprintf(strAction ,"%s", "Start");
			break;      
			
        case DMS_ALARM_TYPE_VMOTION:          //3:�ƶ����
			sprintf(eventType ,"%s", "VideoMotion");  
			sprintf(strAction ,"%s", "Start");
			break;
			
        case DMS_ALARM_TYPE_DISK_UNFORMAT:    //4:Ӳ��δ��ʽ��
			sprintf(eventType ,"%s", "StorageNotExist");  
			break;     
			
        case DMS_ALARM_TYPE_DISK_RWERR:       //5:��дӲ�̳���,
			sprintf(eventType ,"%s", "StorageFailure");  
			break;        
			
        case DMS_ALARM_TYPE_VSHELTER:         //6:�ڵ�����
			sprintf(eventType ,"%s", "VideoBlind");  
			break;     

        case DMS_ALARM_TYPE_ALARMIN_RESUME:         //9:�ź��������ָ�
			sprintf(eventType ,"%s", "LocalIO"); 
			sprintf(strAction ,"%s", "Stop");
			break; 

        case DMS_ALARM_TYPE_VLOST_RESUME:         //10:��Ƶ��ʧ�����ָ�
			sprintf(eventType ,"%s", "VideoLoss"); 
			sprintf(strAction ,"%s", "Stop");
			break; 

        case DMS_ALARM_TYPE_VMOTION_RESUME:         //11:��Ƶ�ƶ���ⱨ���ָ�
			sprintf(eventType ,"%s", "VideoMotion");  
			sprintf(strAction ,"%s", "Stop");
			break; 			

        case DMS_ALARM_TYPE_NET_BROKEN:       //12:����Ͽ�
			sprintf(eventType ,"%s", "NetAbort");  
			break;  
			
        case DMS_ALARM_TYPE_IP_CONFLICT:      //13:IP��ͻ
			sprintf(eventType ,"%s", "IPConfict");  
			break;   
			
        default:
			PRINT_INFO("Alarm: Type = %d", nAlarmType);
        	break;
    }	
	#endif

    int sid = 0;
    int sock = 0;
	char  fSessionId[18] = {0};
	int i;
	for(i = 0; i < XMAI_MAX_LINK_NUM; i++)
    {
        sock = UtGetCmdSockByIndex(i, &g_xmai_session);
        if (sock > 0) {
            sid = UtGetSidByIndex(i, &g_xmai_session);

			memset(fSessionId, 0, sizeof(fSessionId));
			sprintf(fSessionId, "0x%08x", sid);

			char *out;
	    	cJSON *root, *fmt;
		    root = cJSON_CreateObject();//������Ŀ
		    cJSON_AddStringToObject(root, "Name", "AlarmInfo");
		    cJSON_AddNumberToObject(root, "Ret", 100);
		    cJSON_AddStringToObject(root, "SessionID", fSessionId);
		    cJSON_AddItemToObject(root, "AlarmInfo", fmt=cJSON_CreateObject());//����Ŀ�������Ŀ

		    cJSON_AddNumberToObject(fmt, "Channel", 0);
		    cJSON_AddStringToObject(fmt, "Event", eventType);
		    cJSON_AddStringToObject(fmt, "StartTime", startTime);
			cJSON_AddStringToObject(fmt, "Status", strAction);
		    
		    out=cJSON_PrintUnformatted(root);
			PRINT_INFO("%s\n", out);
			
		  	//�Ӱ�ͷ
		    char buf[XMAI_MSG_SEND_BUF_SIZE] = {0};
			//���һ������Ϊ fPeerSeqNum����Ϊ0 ? ����Ҫ���ó�ȫ�ֱ���?
		    int xmaiMsgLen = XMaiMakeMsgUseBuf(buf, sizeof(buf), ALARM_REQ, out, sid, 0);	  	
		    free(out);
		    cJSON_Delete(root);	
			if(xmaiMsgLen < 0)
			{
		        PRINT_ERR();
				break;		
			}

			//�ӷ���
		    int ret = XMaiSockSend(sock, buf, xmaiMsgLen);
		    if(ret < 0)
			{
		        PRINT_ERR();
				break;	
		    }
        }
	}

	
}


/////////////////////////////////
/*************************************************************
* �������ܣ����ܲ�����Խ�����
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
int XMaiTalkRecv(int sock)
{
	PRINT_INFO("start XMaiTalkRecv, sock:%d\n", sock);

	int recvLen = 0;
    char bufRecv[512] = {0};
	XMaiMsgHeader *pXMaiMsgHeader = NULL;
	int dwSize = 0;
	char *lpBuffer = NULL;

	while(g_xmai_talk_recv_running)
	{		
		memset(bufRecv, 0, sizeof(bufRecv));
		recvLen = recv(sock, bufRecv, sizeof(bufRecv), 0);
		if(recvLen <= 0)
		{
		    PRINT_ERR("recv error:%s\n", strerror(errno));
		    break;
		}

		//������Ϣ��
		if(bufRecv[0] == 0xFF)//ȱ��ǿ׳�ľ���
	    {
	    	//XMaiPrintMsgHeader(bufRecv);
			//XMaiPrintAudioHeader(bufRecv+20);
			//printf("\n");
			
	        pXMaiMsgHeader = (XMaiMsgHeader *)bufRecv;	
			//PRINT_INFO("talk recv len:%d\n", pXMaiMsgHeader->dataLen);
			if(pXMaiMsgHeader->messageId == TALK_CU_PU_DATA)
			{
				dwSize = pXMaiMsgHeader->dataLen-8;
				//PRINT_INFO("talk dwSize:%d\n", dwSize);
				lpBuffer = bufRecv+20+8;

				netcam_audio_output(lpBuffer, dwSize, NETCAM_AUDIO_ENC_A_LAW, SDK_TRUE);
#if 0				
				if(dwSize<=80){
					dms_sysnetapi_WriteAudioData(g_xmai_handle, lpBuffer, dwSize, 0);
				}else{
					iPos=0;
					while(iPos+80<=dwSize){
						dms_sysnetapi_WriteAudioData(g_xmai_handle, lpBuffer+iPos, 80, 0);
						iPos+=80;
					}
				}	
#endif
			}							
	    }	
	}
	
    g_xmai_talk_recv_running = 0;
	PRINT_INFO("finish XMaiTalkRecv, sock:%d\n", sock);
	return 0;

}


/*************************************************************
* �������ܣ�������Ƶ���
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
int XMaiTalkMsgSend(int sock, char *data, int data_len, int session_id)
{
	int ret = 0;
	char media_head[40] = {0};
				
	int audio_size = data_len - 4; //��Ƶȥ��ͷ4���ֽ�
	
    XMaiAddTalkMsgHeader(media_head, TALK_PU_CU_DATA, audio_size + 8, session_id, 0);
	XMaiAddAudioHeader(media_head + 20, audio_size);

    ret = XMaiSockSend(sock, media_head, 20+8);
	if(ret >= 0)
	{
    	ret = XMaiSockSend(sock, data+4, audio_size); //��Ƶȥ��ͷ4���ֽ�
	}	
   
    return 0;

}

/*************************************************************
* �������ܣ��豸�ĶԽ����ݻص�
* ���������
* ���������
* ����ֵ  ��
*************************************************************/
int XMaiTalkCallBack(int* pData, unsigned int dwSize, int nType, DWORD dwUserData)
{
    int ret = 0;
    int buf_len = dwSize;
    char *buf = (char*)pData;

    int talk_sock = UtGetTalkSock(g_xmai_talk_session_id, &g_xmai_session);
	//PRINT_INFO("talk size:%d data:0x%08x\n", buf_len, *pData);	
	
    ret = XMaiTalkMsgSend(talk_sock, buf, buf_len, g_xmai_talk_session_id);   
    if(ret < 0)
    {   
    	PRINT_ERR();
        return -1;
    }
    return 0;
}

int XMaiTalkProcess(int session_id)
{
    int talk_sock = UtGetTalkSock(session_id, &g_xmai_session);
	g_xmai_talk_session_id = session_id;
	PRINT_INFO("talk sock:%d sessionid:%d\n", talk_sock, g_xmai_talk_session_id);
#if 0
    ret = dms_sysnetapi_ioctrl(g_xmai_handle, DMS_NET_REG_AUDIOCALLBACK, 0, XMaiTalkCallBack, sizeof(XMaiTalkCallBack));
 	if(0 != ret)
    {
        close(talk_sock);
        talk_sock = -1;
		return -1;
	}   

    memset(&stAudioTalk, 0, sizeof(DMS_NET_AUDIOTALK));
    stAudioTalk.dwSize = sizeof(DMS_NET_AUDIOTALK);
    stAudioTalk.dwUser = 0;
    stAudioTalk.nNumPerFrm = 80;
    stAudioTalk.enFormatTag = DMS_PT_G711A;
    ret = dms_sysnetapi_ioctrl(g_xmai_handle, DMS_NET_CMD_START_TALKAUDIO, 0, &stAudioTalk, sizeof(stAudioTalk));
 	if(0 != ret)
    {
        close(talk_sock);
        talk_sock = -1;
		return -1;
	}    
#endif

    g_xmai_talk_recv_running = 1;
    XMaiTalkRecv(talk_sock);	

#if 0
    dms_sysnetapi_ioctrl(g_xmai_handle, DMS_NET_CMD_STOP_TALKAUDIO, 0, 0, 0);
#endif

	PRINT_INFO("close talk sock:%d\n", talk_sock);
    close(talk_sock);
    talk_sock = -1;  	
    g_xmai_is_talking = 0; //Ϊ0������Խ�����һ�ζԽ���
	
    return 0;
	
}

void * XMaiTalkPthread(void * para)
{
    XmaiSetThreadName("XMaiTalkPthread");

	int session_id = (int)para;
    XMaiTalkProcess(session_id);
	
    PRINT_INFO("XMaiTalkPthread end !\n");
    return NULL;
}


int lib_xmai_start()
{
    //g_xmai_handle = dms_sysnetapi_open(DMS_NETPT_XMAI);
      
    printf("========================= Entry lib_xmai_start \n");

#if 0
	//ע�ᱨ���ص�����
    int ret = dms_sysnetapi_ioctrl (g_xmai_handle, DMS_NET_REG_ALARMCALLBACK, 0, fXmaiNetAlarmCallback, sizeof(fXmaiNetAlarmCallback)); 
    if(0 != ret)
    {
        PRINT_ERR();
        return -1;      
    }

	DEGLINE();
#endif

    g_xmai_pthread_run = 1;

    //�������߳�
    XMaiCreateThread(XMaiMainThread, NULL, NULL);

    //�����豸���������̣߳�����㲥��
    XMaiCreateThread(XMaiBroadcastThread, NULL, NULL);

    XMaiCreateThread(XMaiSendMainStreamPthread, NULL, NULL);
    XMaiCreateThread(XMaiSendSubStreamPthread, NULL, NULL);


	DEGLINE();
    return 0;
}

int lib_xmai_stop()
{
    //int ret = 0;
    
    g_xmai_pthread_run = 0; 
       
#if 0
    //ע�������ص�����
    ret = dms_sysnetapi_ioctrl (g_xmai_handle, DMS_NET_REG_ALARMCALLBACK, 0, NULL, 0); 
    if(0 != ret)
    {
        PRINT_ERR("DMS_NET_REG_ALARMCALLBACK error\n");        
    }      

    usleep(200000);	
	dms_sysnetapi_close(g_xmai_handle);
#endif
    PRINT_ERR("lib_xmai_close bye bye bye bye bye11111 \n");
    return 0;
}


