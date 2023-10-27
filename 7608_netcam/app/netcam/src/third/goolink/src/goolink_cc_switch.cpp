#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <memory.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <csl_sem.h>
#include <cJSON.h>
#include <goolink_sesapi.h>
#include <goolink_apiv2.h>
#include "goolink_cc_switch.h"
#include "transparent.h"
#include "debug.h"
#include <HieServer_Cfg.h>

#include "netcam_api.h"


#define NET_LED_GPIO 13
int nTimeOutMS = TIMEOUT_IDLE; //̫���˴�������; ̫���˳�cpu����Ҳ����ȡ
STRU_CONN_INFO s_hOldConnID[MAX_CONN_NUM] = {0};
static int led_ctl_enable = 1;

void sysUsecTime(struct tm * ptm);
/*
1 ͨ�����ڸ�CC1310ͨ��
2 ͨ��glnk�ϱ�CC�ӿ���Ϣ
*/
CMutex s_hSensorDataMutex;

USERDEF_CMD struUserType[] = { //glnk���ַ����������Ӧ��, ��Щ������ͷ��
    {USRDEF_CC_LIST,    KEY_CC_LIST},
    {USRDEF_CC_ADD,     KEY_CC_ADD},
    {USRDEF_CC_DEL,     KEY_CC_DEL},
    {USRDEF_CC_ALARM,   KEY_CC_ALARM},
    {USRDEF_CC_WAKEUP,  KEY_CC_WAKEUP},
    {USRDEF_CC_SETWIFI, KEY_CC_SETWIFI},
    {USRDEF_PIR,        KEY_PIR},
    {USRDEF_ALARM_ACTION, KEY_ALARM_ACTION},
    {USRDEF_HUMANTRACE, KEY_HUMANTRACE},
    {USRDEF_NETLED,     KEY_NETLED},
};
static char s_SesBufRecvIn[SES_BUF_SIZE]; //����Զ��glnk�������Զ�������
char s_SesBufSendOut[SES_BUF_SIZE]; //�������ݸ�Զ��app
SENSOR_NODE struSenseorNode[MAX_SENSOR_NODE]; //�豸���ؼ�¼cc����������

int MPU_SetAutoHumanTrace(int bEnable) //�·��Զ�����׷�ٿ���
{
	runMdCfg.handle.bCheckedFigure = bEnable;
	MdCfgSave();
}

int MPU_GetAutoHumanTrace()
{
	return runMdCfg.handle.bCheckedFigure;
}

int MPU_GetNetLed()
{
	if(strstr(sdk_cfg.name, "CUS_HC_GK7202_GC2053_V10"))
	{
		int ledEnableValue =0;
		if (access("/opt/custom/cfg/blueLed.conf", R_OK) != 0)
		{
			ledEnableValue =1;
			return ledEnableValue;
		}
		FILE *ledFp = NULL;

		ledFp = fopen("/opt/custom/cfg/blueLed.conf","r");
		if (NULL == ledFp)
		{
			ledEnableValue =1;
			return ledEnableValue;
		}
		fseek(ledFp,0L,SEEK_SET);

		char buf[6]={0};
		int readCnt = fread(buf,sizeof(buf),1,ledFp);  /* ����ֵΪ0 */
		fclose(ledFp);

		if (strcmp(buf,"false") == 0)
			ledEnableValue = 0;
		else if (strcmp(buf,"true") == 0)
			ledEnableValue = 1;

		return ledEnableValue;

	}
	else
	{

		return glnk_led_control_state();

	}
}

int MPU_SetNetLed(int bshow)
{
	if (bshow)
	{
		led_ctl_enable = 1;
	}
	else
	{
		led_ctl_enable = 0;
	}

	if(strstr(sdk_cfg.name, "CUS_HC_GK7202_GC2053_V10"))
		sdk_gpio_set_output_value(NET_LED_GPIO, bshow);
}

int glnk_led_control_state(void)
{
	return led_ctl_enable;
}

int parse_json_cmdtype(char * pbuf, USERDEF_CMD * pCmd, int nTotalCmd) //���ݴ��������ַ����ҵ���Ӧ��������ʲô
{
    JS_HEAP_OBJ_MGR heapobjmgr = {0};
    //���ﲻ��ֱ�ӱȽ�ͷ, ���ǽ���һ���ٱȽ�
    int rettype = -1, i = 0;
    //ȡjson�ַ�����ͷ, ����ͷ��������
    cJSON* root  = cJSON_ParseEx(&heapobjmgr, pbuf);
    if (NULL == root){
        return -1;
    }
    printf("1.app_orignal_cmd:\n%s\n", JsPrintFmt(root));
    cJSON * jshead = cJSON_GetArrayItem(root, 0); //ȡ�±��һ��Ԫ�ص�����
    if (NULL == jshead){
        return -1;
    }
    char *pszRootKey = jshead->string;
    char *pszRootValue = jshead->valuestring;
    printf("parse_cmdtype, pszRoot=%s, pszRootValue=%d \n", pszRootKey, pszRootValue);

    for (i = 0; i< nTotalCmd; i++){
        printf("parse_cmdtype %s  %s \n", pszRootKey, pCmd[i].szName);
        if (0 == strncmp(pszRootKey, pCmd[i].szName, strlen(pCmd[i].szName))){
            rettype = pCmd[i].iType;
            printf("parse_cmdtype, rettype=%d, protocol_name=%s \n", rettype, pCmd[i].szName);
            break;
        }
    }
    JsHeapObjFree(&heapobjmgr);
    return rettype;
}

int parse_find_item_idindex(char *pRecvIn)
{
    JS_HEAP_OBJ_MGR heapobjmgr = {0};
    int i = 0, ifinditem = 0;
    cJSON * root  = cJSON_ParseEx(&heapobjmgr, pRecvIn);
    //printf("1.orignal_app_cmd:\n%s \n",  cJSON_Print(root));
    cJSON * jshead = cJSON_GetArrayItem(root, 0); //ȡ�±��һ��Ԫ�ص�key��Ӧ��value����
    cJSON * itemSensorId = cJSON_GetObjectItem(jshead, KEY_SENSORID);//�ӵ�һ��Ԫ�����ҵ�id
    //printf("find_item 1111 %s \n",  cJSON_Print(jshead));
    printf("find_item 2222 %s \n",  JsPrintFmt(itemSensorId));

    for ( i = 0; i<MAX_SENSOR_NODE; i++){ //��������sensor�ڵ�, �����õ�sensor��Ϣȫ����ӡ��sendbuf��
        if (0 == strncmp(itemSensorId->valuestring, struSenseorNode[i].id, strlen(struSenseorNode[i].id))){
            ifinditem = i;
            printf("find_item, ifinditem=%d \n", ifinditem);
            break;
        }
    }
    if (ifinditem >= MAX_SENSOR_NODE){ //���û�ҵ�
        ifinditem = -1;
        printf("glnk_delete_macid, could not find deleteid:%s \n", itemSensorId->valuestring);
    }
    //printf("finally, ifinditem=%d\n", ifinditem);
    JsHeapObjFree(&heapobjmgr);
    return ifinditem;
}

void reply_to_app_write(const char * szCmdType, int ifinditem, char * pBufOutput)
{
    JS_HEAP_OBJ_MGR heapobjmgr = {0};
    cJSON* VideoRecordroot = cJSON_CreateObjectEx(&heapobjmgr);
    if (0==strcmp(szCmdType, KEY_CC_ADD) || 0==strcmp(szCmdType, KEY_CC_DEL)){
        cJSON* JsSensorNode    = cJSON_CreateObjectEx(&heapobjmgr);
        if (ifinditem > -1){//id�����ҵ�
            cJSON_AddStringToObject(JsSensorNode, KEY_SENSORID,     struSenseorNode[ifinditem].id);
            cJSON_AddStringToObject(JsSensorNode, KEY_SENSORLOC,    struSenseorNode[ifinditem].szLocation);
            cJSON_AddNumberToObject(JsSensorNode, KEY_ACCEPT,       struSenseorNode[ifinditem].bAccepted);
            cJSON_AddNumberToObject(JsSensorNode, KEY_HANDLEBYAPP,  struSenseorNode[ifinditem].bHandledByApp);
        }
        else{ //id������, ֪ͨ��appû���ҵ�
            cJSON_AddStringToObject(JsSensorNode, KEY_SENSORID,     "null");
        }
        cJSON_AddItemReferenceToObject(VideoRecordroot, szCmdType, JsSensorNode);
    }
    else if (0==strcmp(szCmdType, KEY_CC_LIST)){
        cJSON* JsSensorArray   = cJSON_CreateArrayEx(&heapobjmgr);
        for (int i = 0; i< MAX_SENSOR_NODE; i++){
            cJSON* JsSensorNode    = cJSON_CreateObjectEx(&heapobjmgr);
            cJSON_AddStringToObject(JsSensorNode, KEY_SENSORID,     struSenseorNode[i].id);
            cJSON_AddStringToObject(JsSensorNode, KEY_SENSORLOC,    struSenseorNode[i].szLocation);
            cJSON_AddNumberToObject(JsSensorNode, KEY_ACCEPT,       struSenseorNode[i].bAccepted);
            cJSON_AddNumberToObject(JsSensorNode, KEY_HANDLEBYAPP,  struSenseorNode[i].bHandledByApp);
            cJSON_AddNumberToObject(JsSensorNode, KEY_ALARMING,     struSenseorNode[i].bAlarming);
            cJSON_AddStringToObject(JsSensorNode, KEY_TYPE,         struSenseorNode[i].sztype); //A���𶯴�����, B�������
            cJSON_AddItemReferenceToArray(JsSensorArray, JsSensorNode);
        }
        cJSON_AddItemReferenceToObject(VideoRecordroot, KEY_CC_LIST, JsSensorArray);
    }
    else if (0==strcmp(szCmdType, KEY_HUMANTRACE)){
        printf("reply_to_app_write, KEY_HUMANTRACE ===================================== \n ");
        cJSON* JsNode   = cJSON_CreateObjectEx(&heapobjmgr);
        int bhuman_switch = MPU_GetAutoHumanTrace();
        cJSON_AddNumberToObject(JsNode, KEY_HT_SWITCH, bhuman_switch); //�м���ֶ�
        cJSON_AddItemReferenceToObject(VideoRecordroot, KEY_HUMANTRACE, JsNode); //Э��ͷ
    }
    else if (0==strcmp(szCmdType, KEY_NETLED)){ //20190225
        printf("reply_to_app_write, KEY_NETSTATUSLED >>>>>>>>>>>>>>>>>>>>>>>>>>>>> \n ");
        cJSON* JsNode   = cJSON_CreateObjectEx(&heapobjmgr);
        int bnetled_switch = MPU_GetNetLed(); //��ȡ�ײ㿪��״̬, ����Ҫ����
        cJSON_AddNumberToObject(JsNode, KEY_HT_SWITCH, bnetled_switch); //�м���ֶ�
        cJSON_AddItemReferenceToObject(VideoRecordroot, KEY_NETLED, JsNode); //Э��ͷ
        printf("reply_to_app_write, KEY_NETSTATUSLED %d<<<<<<<<<<<<<<<<<<<<<<<<<<< \n ",bnetled_switch);
    }
    else {
        printf("reply_to_app_write, protocol %s is not existed \n", szCmdType);
        return;
    }

    if(pBufOutput){
        JsToBuffer(VideoRecordroot, pBufOutput);
    }

    JsHeapObjFree(&heapobjmgr);
}


int glnk_humantrace()//����ֻ��䷢�ͻ���, �������˷���
{
    reply_to_app_write(KEY_HUMANTRACE, 0, s_SesBufSendOut);
}

int glnk_netstatusled()//����ֻ��䷢�ͻ���, �������˷���
{
    reply_to_app_write(KEY_NETLED, 0, s_SesBufSendOut);
}

//=====================================alarm_action========================================
//20181015 �¿����������
#define ACTION_TIME_SEGNUM 4
typedef struct STRU_ACTIONFILTER_TIMESEG{ //�ڸ�ʱ������Ƿ�ѡ
    int StartTime;
    int EndTime;
    int ActionMask; //ͨ��mask��ָ�������Ķ���, ����չ, �����Ժ���ܶ���������
}STRU_ACTIONFILTER_TIMESEG;

typedef struct STRU_ALARM_ACTION_INFO {
    //����Դ
    int bCheckedPIR;
    int bCheckedMotion;
    int nMotionsenstive; //�ƶ����������

    int bCheckedFigure; //3 �Ƿ������μ�����
    int StayDelayTime; //�տ�ʼ������������ʱ��, �������ʱ���Ҫ��ʼ������

    //��������
    STRU_ACTIONFILTER_TIMESEG struTimeSeg[ACTION_TIME_SEGNUM]; //Ĭ�ϸ�4��ʱ��� //20181015�޸ĳɴ�ʱ��ε�

    //�Զ���ѡ�������������
    int AlarmAudioType; //1: Ĭ������1; 2: Ĭ������2; 10: �Զ���;
}STRU_ALARM_ACTION_INFO;

/////////////////////////////////////////////////////////////////////////////////
int MPU_Get_Alarm_Action(STRU_ALARM_ACTION_INFO * pStruAlarmSrcAction)
{
	printf("MPU_Get_Alarm_Action\n");

	int sensi;
	int retSensi = 0;

	sensi = runMdCfg.sensitive;
	if(sensi <= 25)
	{
		retSensi = 1;
	}
	else if(sensi <= 50)
	{
		retSensi = 2;
	}
	else if(sensi <= 75)
	{
		retSensi = 3;
	}
	else if(sensi < 100)
	{
		retSensi = 4;
	}
	else
	{
		retSensi = 4;
	}

	pStruAlarmSrcAction->bCheckedMotion = runMdCfg.enable;

	pStruAlarmSrcAction->nMotionsenstive = retSensi;
	pStruAlarmSrcAction->StayDelayTime = runMdCfg.handle.intervalTime;
	pStruAlarmSrcAction->AlarmAudioType = runP2PCfg.alarmAudioType;//runMdCfg.handle.alarmAudioType;
	pStruAlarmSrcAction->bCheckedFigure = runMdCfg.handle.bCheckedFigure;

	for (int i = 0; i<ACTION_TIME_SEGNUM; i++)
	{
#if 0
		if(	   (runMdCfg.scheduleTime[0][i].startHour != runMdCfg.scheduleTime[0][i].stopHour)
			|| (runMdCfg.scheduleTime[0][i].startMin != runMdCfg.scheduleTime[0][i].stopMin))
#endif
		{
    	    pStruAlarmSrcAction->struTimeSeg[i].StartTime = runMdCfg.scheduleTime[0][i].startHour*60 + runMdCfg.scheduleTime[0][i].startMin;
			pStruAlarmSrcAction->struTimeSeg[i].EndTime = runMdCfg.scheduleTime[0][i].stopHour*60 + runMdCfg.scheduleTime[0][i].stopMin;
			printf("pStruAlarmSrcAction->struTimeSeg[%d].StartTime = %d\n", i, pStruAlarmSrcAction->struTimeSeg[i].StartTime);
			printf("pStruAlarmSrcAction->struTimeSeg[%d].EndTime = %d\n", i, pStruAlarmSrcAction->struTimeSeg[i].EndTime);
			if ((i>0) && (pStruAlarmSrcAction->struTimeSeg[i].StartTime == pStruAlarmSrcAction->struTimeSeg[0].StartTime))
			{
				pStruAlarmSrcAction->struTimeSeg[i].StartTime = 1440;
				pStruAlarmSrcAction->struTimeSeg[i].EndTime = 1440;
			}
			//runMdCfg.handle.is_beep[i]?(pStruAlarmSrcAction->struTimeSeg[i].ActionMask |= DVR_ALARM_TRIGGER_AUDIOOUT):(pStruAlarmSrcAction->struTimeSeg[i].ActionMask &= ~DVR_ALARM_TRIGGER_AUDIOOUT);
			runP2PCfg.is_beep[i]?(pStruAlarmSrcAction->struTimeSeg[i].ActionMask |= DVR_ALARM_TRIGGER_AUDIOOUT):(pStruAlarmSrcAction->struTimeSeg[i].ActionMask &= ~DVR_ALARM_TRIGGER_AUDIOOUT);
			runMdCfg.handle.is_rec?(pStruAlarmSrcAction->struTimeSeg[i].ActionMask |= DVR_ALARM_TRIGGER_RECORD):(pStruAlarmSrcAction->struTimeSeg[i].ActionMask &= ~DVR_ALARM_TRIGGER_RECORD);
			runMdCfg.handle.is_light[i]?(pStruAlarmSrcAction->struTimeSeg[i].ActionMask |= DVR_ALARM_TRIGGER_LIGHT):(pStruAlarmSrcAction->struTimeSeg[i].ActionMask &= ~DVR_ALARM_TRIGGER_LIGHT);
			runMdCfg.handle.is_pushMsg?(pStruAlarmSrcAction->struTimeSeg[i].ActionMask |= DVR_ALARM_TRIGGER_NOTIFYAMS):(pStruAlarmSrcAction->struTimeSeg[i].ActionMask &= ~DVR_ALARM_TRIGGER_NOTIFYAMS);
		}
	}
}

int MPU_Set_Alarm_Action(STRU_ALARM_ACTION_INFO * pStruAlarmSrcAction)
{
	printf("MPU_Set_Alarm_Action:%d\n", pStruAlarmSrcAction->nMotionsenstive);

	int pirSensi = 0;

	STRU_ALARM_ACTION_INFO *p = (STRU_ALARM_ACTION_INFO *)pStruAlarmSrcAction;
	STRU_ALARM_ACTION_INFO resp = {0} ;
	memcpy(&resp,p,sizeof(STRU_ALARM_ACTION_INFO));

	runMdCfg.enable = p->bCheckedMotion;

	pirSensi = SYS_MD_SENSITIVITY_HIGHEST + 1 - p->nMotionsenstive;
	netcam_md_set_sensitivity(0,(SYS_MD_SENSITIVITY_t)pirSensi);

	runMdCfg.handle.intervalTime = p->StayDelayTime;
	runP2PCfg.alarmAudioType = p->AlarmAudioType;

	runMdCfg.handle.recStreamNo = 1;
	runMdCfg.handle.recTime = 1;

	runMdCfg.sensitive = p->nMotionsenstive*20;

	runMdCfg.handle.is_rec = (p->struTimeSeg[0].ActionMask & DVR_ALARM_TRIGGER_RECORD)? 1:0;
	runMdCfg.handle.is_pushMsg = (p->struTimeSeg[0].ActionMask & DVR_ALARM_TRIGGER_NOTIFYAMS)? 1:0;

	int i = 0,j = 0;
	//for (i = 0; i < 7; i ++)
	{
		if(p->struTimeSeg[0].StartTime > p->struTimeSeg[0].EndTime)
		{
			runMdCfg.scheduleTime[i][0].startHour = 0;
			runMdCfg.scheduleTime[i][0].startMin = 0;
			runMdCfg.scheduleTime[i][0].stopHour = p->struTimeSeg[0].EndTime/60;
			runMdCfg.scheduleTime[i][0].stopMin = p->struTimeSeg[0].EndTime%60;
			//runMdCfg.handle.is_beep[0] = (p->struTimeSeg[0].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runP2PCfg.is_beep[0] = (p->struTimeSeg[0].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runMdCfg.handle.is_light[0] = (p->struTimeSeg[0].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0;

			runMdCfg.scheduleTime[i][1].startHour = p->struTimeSeg[0].StartTime/60;
			runMdCfg.scheduleTime[i][1].startMin = p->struTimeSeg[0].StartTime%60;
			runMdCfg.scheduleTime[i][1].stopHour = 24;
			runMdCfg.scheduleTime[i][1].stopMin = 0;
			//runMdCfg.handle.is_beep[1] = (p->struTimeSeg[1].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runP2PCfg.is_beep[1] = (p->struTimeSeg[1].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runMdCfg.handle.is_light[1] = (p->struTimeSeg[1].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0;

			runMdCfg.scheduleTime[i][2].startHour = 24;
			runMdCfg.scheduleTime[i][2].startMin = 0;
			runMdCfg.scheduleTime[i][2].stopHour = 24;
			runMdCfg.scheduleTime[i][2].stopMin = 0;
			//runMdCfg.handle.is_beep[2] = (p->struTimeSeg[2].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runP2PCfg.is_beep[2] = (p->struTimeSeg[2].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runMdCfg.handle.is_light[2] = (p->struTimeSeg[2].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0;

			runMdCfg.scheduleTime[i][3].startHour = 24;
			runMdCfg.scheduleTime[i][3].startMin = 0;
			runMdCfg.scheduleTime[i][3].stopHour = 24;
			runMdCfg.scheduleTime[i][3].stopMin = 0;
			//runMdCfg.handle.is_beep[3] = (p->struTimeSeg[3].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runP2PCfg.is_beep[3]  = (p->struTimeSeg[3].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
			runMdCfg.handle.is_light[3] = (p->struTimeSeg[3].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0;
		}
		else
		{
			for (j = 0; j < 4; j ++)
			{
				runMdCfg.scheduleTime[i][j].startHour = p->struTimeSeg[j].StartTime/60;
				runMdCfg.scheduleTime[i][j].startMin = p->struTimeSeg[j].StartTime%60;
				runMdCfg.scheduleTime[i][j].stopHour = p->struTimeSeg[j].EndTime/60;
				runMdCfg.scheduleTime[i][j].stopMin = p->struTimeSeg[j].EndTime%60;

				//runMdCfg.handle.is_beep[j] = (p->struTimeSeg[j].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
				runP2PCfg.is_beep[j] = (p->struTimeSeg[j].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0;
				runMdCfg.handle.is_light[j] = (p->struTimeSeg[j].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0;
			}
		}
	}

	MdCfgSave();
	P2PCfgSave();
}


int glnk_alarm_action(char *pRecvIn)
{
    JS_HEAP_OBJ_MGR heapobjmgr = {0};

    STRU_ALARM_ACTION_INFO struAAinfo;
    memset(&struAAinfo, 0, sizeof(STRU_ALARM_ACTION_INFO));
    cJSON * root  = cJSON_ParseEx(&heapobjmgr, pRecvIn);
    if (NULL == root){
        printf("glnk_alarm_action, 1111  return \n");
        return -1;
    }
    char *pszStr = JsPrintFmt(root);
    if (NULL == pszStr){
        printf("glnk_alarm_action, 11112222  return \n");
        return -1;
    }
    GDEBUG("1.orignal_app_cmd:\n%s \n", pszStr);
    cJSON * jshead = cJSON_GetArrayItem(root, 0); //ȡ�±��һ��Ԫ�ص�key��Ӧ��value����������
    if (NULL == jshead){
        printf("glnk_alarm_action, 2222 return \n");
        return -1;
    }
    jshead = cJSON_GetArrayItem(jshead, 0); //ȡ��һ����ȫ������
    //GDEBUG("find_item 1111:\n%s \n jshead->string =%s \n",  cJSON_Print(jshead), jshead->string);
    if (0 == strncmp(jshead->string, KEY_OP_SET, strlen(KEY_OP_SET))){ //��������set
        /*
        app����:
        {"alarm_action":{"set" : { "PirEnable":1, "MotionEnable":1, "MotionSensitive": 2, "FigureFilterEnable":1, "ActionRecord":1, "ActionSound":1, "ActionPushup":1}}}
        dev����:
        {"alarm_action":  "result": 1}
        */
        //������ȡ�±�
        cJSON * param1 = NULL;
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_PIRENABLE);//�ӵ�һ��Ԫ�����ҵ�id
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
            struAAinfo.bCheckedPIR = param1->valueint;
        }

        //�ƶ����
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_MOTIONENABLE);//�ӵ�һ��Ԫ�����ҵ�id
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
            struAAinfo.bCheckedMotion  = param1->valueint;
        }

        //�ƶ����������
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_MOTIONSENSITIVE);
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
            struAAinfo.nMotionsenstive = param1->valueint;
        }

        //���μ��
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_FIGUREPILTERENABLE);
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
             struAAinfo.bCheckedFigure = param1->valueint;
        }

        //����ʱ��
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_STAYDEAYTIME);
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
            struAAinfo.StayDelayTime = param1->valueint;
        }

        //�Զ���������������
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_ALARMAUDIOTYPE);
        if (param1){
            GDEBUG("filed1: %s, value=%s, %d \n",  JsPrintFmt(param1), param1->string, param1->valueint);
            struAAinfo.AlarmAudioType = param1->valueint;
        }

        //�����������ڴ������ܽ���
        param1 = cJSON_GetObjectItem(jshead, KEY_AA_ACTION);
        for (int i = 0; i<ACTION_TIME_SEGNUM; i++){
            cJSON * iJSArrTime = cJSON_GetArrayItem(param1, i);
            if (iJSArrTime){ //������Щ���鲻��4��

                cJSON * JSitemArr;
                GDEBUG("i=%d \n", i);

                //�Ȼ�ȡʱ��
                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTION_START); //����������ĵ�һ��Ԫ����ȡ
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n", JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].StartTime = JSitemArr->valueint;
                }

                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTION_END); //����������ĵ�һ��Ԫ����ȡ
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n",  JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].EndTime = JSitemArr->valueint;
                }

                //����SDĬ��¼��, ���豩¶������
                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTIONRECORD); //����������ĵ�һ��Ԫ����ȡ
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n",  JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].ActionMask |= JSitemArr->valueint?DVR_ALARM_TRIGGER_RECORD:0;
                }

                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTIONSOUND); //��
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n",  JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].ActionMask |= JSitemArr->valueint?DVR_ALARM_TRIGGER_AUDIOOUT:0;
                }

                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTIONLIGHT); //��
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n",  JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].ActionMask |= JSitemArr->valueint?DVR_ALARM_TRIGGER_LIGHT:0;
                }

                JSitemArr = cJSON_GetObjectItem(iJSArrTime, KEY_AA_ACTIONPUSHUP); //�ϱ�app
                if (JSitemArr){
                    GDEBUG("%s, value=%s, %d \n",  JsPrintFmt(JSitemArr), JSitemArr->string, JSitemArr->valueint);
                    struAAinfo.struTimeSeg[i].ActionMask |= JSitemArr->valueint?DVR_ALARM_TRIGGER_NOTIFYGUI:0;
                }
            }
            else{
                break;
            }
        }

        MPU_Set_Alarm_Action(&struAAinfo);
		if ((runMdCfg.enable == 0) || ((runMdCfg.handle.is_light[0] == 0) && (runMdCfg.handle.is_light[1] == 0)
			                            &&(runMdCfg.handle.is_light[2] == 0) && (runMdCfg.handle.is_light[3] == 0)))
		{
			if(strstr(sdk_cfg.name, "CUS_HC_GK7202_GC2053_V10"))
				gk_isp_led_off();
		}

        //���ظ�app
        cJSON* JsSensorNode    = cJSON_CreateObjectEx(&heapobjmgr);
        cJSON_AddNumberToObject(JsSensorNode, "result",   1);

        cJSON* VideoRecordroot = cJSON_CreateObjectEx(&heapobjmgr);
        cJSON_AddItemReferenceToObject(VideoRecordroot, KEY_ALARM_ACTION, JsSensorNode);

        JsToBuffer(VideoRecordroot, s_SesBufSendOut);
    }
    else if (0 == strncmp(jshead->string, KEY_OP_GET, strlen(KEY_OP_GET))){ //��������get
        /*
        app����:
        {"alarm_action":{"get" : null}}
        dev����:
        {"alarm_action":{"get" :
            {
                "PirEnable":1,
                "MotionEnable":1,
                "MotionSensitive": 2,

                //��������μ��һ��, ���ڹ���·����:
                "FigureFilterEnable":1,  //�ڹ���Դ·����
                "StayDelayTime": 3,  //ǰ3�벻����
                "Action":[
                        {"StartTime":0,    "EndTime":1440, "ActionRecord":1, "ActionSound":1, "ActionPushup":1, "ActionLight":1, "Action12V":1},
                        {"StartTime":1440, "EndTime":1440, "ActionRecord":1, "ActionSound":0, "ActionPushup":1, "ActionLight":1, "Action12V":1},
                        {"StartTime":1440, "EndTime":1440, "ActionRecord":1, "ActionSound":0, "ActionPushup":1, "ActionLight":1, "Action12V":1}
                        {"StartTime":1440, "EndTime":1440, "ActionRecord":1, "ActionSound":0, "ActionPushup":1, "ActionLight":1, "Action12V":1}
                        ]
                "AlarmAudioType":1,  //�����Զ���������ѡ��
            }
        }}
        */
        MPU_Get_Alarm_Action(&struAAinfo);
        cJSON* JsSensorNode    = cJSON_CreateObjectEx(&heapobjmgr);

        cJSON_AddNumberToObject(JsSensorNode, KEY_AA_MOTIONENABLE,    struAAinfo.bCheckedMotion);
        cJSON_AddNumberToObject(JsSensorNode, KEY_AA_MOTIONSENSITIVE, struAAinfo.nMotionsenstive);

        //cJSON_AddNumberToObject(JsSensorNode, KEY_AA_FIGUREPILTERENABLE, struAAinfo.bCheckedFigure);// comment it in goke solution

        cJSON_AddNumberToObject(JsSensorNode, KEY_AA_STAYDEAYTIME, struAAinfo.StayDelayTime);

        cJSON* JSAction[ACTION_TIME_SEGNUM]={NULL};
        cJSON *JsArr = cJSON_CreateArrayEx(&heapobjmgr);
        for (int i = 0; i<ACTION_TIME_SEGNUM; i++){
            JSAction[i] = cJSON_CreateObjectEx(&heapobjmgr);
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTION_START,  struAAinfo.struTimeSeg[i].StartTime);
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTION_END,    struAAinfo.struTimeSeg[i].EndTime);

            //SD�洢��Ĭ��ǿ��, �ʲ��ٱ�¶����
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTIONRECORD, (struAAinfo.struTimeSeg[i].ActionMask & DVR_ALARM_TRIGGER_RECORD)? 1:0);
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTIONSOUND,  (struAAinfo.struTimeSeg[i].ActionMask & DVR_ALARM_TRIGGER_AUDIOOUT)? 1:0);
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTIONLIGHT,  (struAAinfo.struTimeSeg[i].ActionMask & DVR_ALARM_TRIGGER_LIGHT)? 1:0);
            cJSON_AddNumberToObject(JSAction[i], KEY_AA_ACTIONPUSHUP, (struAAinfo.struTimeSeg[i].ActionMask & DVR_ALARM_TRIGGER_NOTIFYAMS)? 1:0);

            //20181116 ��Ϊ���̵������س�ͻ, ��������12V������
            //cJSON_AddNumberToObject(JSAction, KEY_AA_ACTIONV12,    (struAAinfo.struTimeSeg[i].ActionMask & DVR_ALARM_TRIGGER_V12)? 1:0);

            cJSON_AddItemReferenceToObject(JsArr, KEY_AA_ACTION, JSAction[i]); //���뵽������
        }
        cJSON_AddItemReferenceToObject(JsSensorNode, KEY_AA_ACTION, JsArr);

        cJSON_AddNumberToObject(JsSensorNode, KEY_AA_ALARMAUDIOTYPE, struAAinfo.AlarmAudioType);

        ///////////////////////////////////////////////////////////////////////////////////////////////
        //2 ��װget
        cJSON* getmiddle = cJSON_CreateObjectEx(&heapobjmgr);
        cJSON_AddItemReferenceToObject(getmiddle, KEY_OP_GET, JsSensorNode);

        //3 ��װroot
        cJSON* VideoRecordroot = cJSON_CreateObjectEx(&heapobjmgr);
        cJSON_AddItemReferenceToObject(VideoRecordroot, KEY_ALARM_ACTION, getmiddle);

        //4 �γ��ַ���
        JsToBuffer(VideoRecordroot, s_SesBufSendOut);
    }

    JsHeapObjFree(&heapobjmgr);

    return 0;
}

int my_ses_recv(int item)
{
    int ret = 0, ncmdtype = -1;
    ret = glnk_ses_read(s_hOldConnID[item].hconnid, s_SesBufRecvIn, SES_BUF_SIZE, nTimeOutMS);
    if (ret > 0){ //���Զ�������
        int bHex = 0, len = 0;
        //printf("Fnx_SesProcess ret>0, s_SesBufRecvIn:\n%s \n", s_SesBufRecvIn);
        GDEBUG("Fnx_SesProcess ret>0");
        memset(s_SesBufSendOut, 0, SES_BUF_SIZE);

        if(0 == strncmp(s_SesBufRecvIn, DOWNLOADFILE,  TOKENSIZE)){ //�����ļ�, �������Զ����������
            bHex = 1;
            s_hOldConnID[item].iType = USRDEF_DOWNLOAD;
            GDEBUG("Fnx_SesProcess, enter HEX DOWNLOADFILE\n");
            //�ڴ�ֱ����͸��д�ļ�
            len = download_file(s_SesBufRecvIn); //���ط��ͳ���

            //20180930, �������ܿ���, �������˸Ͻ��ر�����
            TLV_V_UploadAlarmResponse *phead = (TLV_V_UploadAlarmResponse *)s_SesBufRecvIn;
            if (phead->type == 3){
                s_hOldConnID[item].nconnStatus = CONN_FINISH;
            }

            ret = glnk_ses_write(s_hOldConnID[item].hconnid, s_SesBufSendOut, len, nTimeOutMS);
            GDEBUG("len=%d, ret=%d \n", len, ret);
        }
        else{ //֮ǰ��json�ַ���Э��
            ncmdtype = parse_json_cmdtype(s_SesBufRecvIn, struUserType, sizeof(struUserType)/sizeof(USERDEF_CMD));
            s_hOldConnID[item].iType = ncmdtype;
            switch(ncmdtype)
            {
				case USRDEF_ALARM_ACTION:
	                glnk_alarm_action(s_SesBufRecvIn); //20180820 ����pir�Ŀ���
	                break;
	            case USRDEF_HUMANTRACE:
	                glnk_humantrace();
	                break;
	            case USRDEF_NETLED:
	                glnk_netstatusled();
	                break;
	            default:
	                break;
            }
            printf("Fnx_SesProcess, s_SesBufSendOut:\n%s \n", s_SesBufSendOut);
            len = strlen(s_SesBufSendOut); //���ַ������������������
            if (len > 0)glnk_ses_write(s_hOldConnID[item].hconnid, s_SesBufSendOut, len, nTimeOutMS);
        }
    }
    else if (0 == ret){ //���ظ������Ҫ�ر�����
        GDEBUG("Fnx_SesProcess ret=0, hConnectionID=%d \n", s_hOldConnID[item].hconnid);
    }
    else{//��ʱ, ���ر�����
        //printf("Fnx_SesProcess ret<0, timeout, hConnectionID=%d \n", hConnectionID);
    }
    //printf("ret=%d", ret);
    return ret;
}


static void *Fnx_GlnkSesProcess(void *lParam)
{
    int ret             = 0;
	int ncmdtype        = -1;
	int i               = 0;
    int hConnectionID   = 0;
    static int bPollSes = 1;

	pthread_detach(pthread_self());
	sdk_sys_thread_set_name("GlnkSesProcessThread");
    printf("Fnx_SesProcess >>>>>>>>>>>>>>>> \n");

    while(bPollSes){
        hConnectionID = glnk_ses_poll(nTimeOutMS); //�ù�һ�ξ�û����
        //printf("Fnx_SesProcess, hConnectionID=%d \n", hConnectionID);
        if(hConnectionID){//�������ӹ���
            for (i = 0; i<MAX_CONN_NUM; i++){
                if (0 == s_hOldConnID[i].hconnid){ //��û�б���ռ��, ���Ա���, ֮��Ҫ������������connid
                    s_hOldConnID[i].hconnid = hConnectionID; //�ҵ��˿յ�, ֱ��Ͷ�뱣������
                    s_hOldConnID[i].iType = 0; //��ʱ����type
                    break;
                }
            }

            if (i == MAX_CONN_NUM){ //����һȦû���ҵ��ղ�slot, �����ܾ���.
                GDEBUG("over max connect, reject, hConnectionID=%d \n", hConnectionID); //ʼ��û�н���
                glnk_ses_close(hConnectionID);
            }
        }

        int bExistConn = 0;
        nTimeOutMS = TIMEOUT_IDLE; //�ӿ��п�ʼ��
        for (i = 0; i<MAX_CONN_NUM; i++){//���������, �Ƿ������ݿɶ�, ���߿��Թر���
            if (s_hOldConnID[i].hconnid >0){
                //����������list��ʱ, ����ȷ��timeout
                if (USRDEF_DOWNLOAD == s_hOldConnID[i].iType){ //�������ϴ��ͽ�һ������
                    nTimeOutMS = (nTimeOutMS < TIMEOUT_DOWNLOAD)?nTimeOutMS:TIMEOUT_DOWNLOAD; //1 �����ӻ��ڴ�ʱҪ����cpu, �ӿ��ȡ�ٶ�
                }
                nTimeOutMS = (nTimeOutMS < TIMEOUT_JSON)?nTimeOutMS:TIMEOUT_JSON; //1 �����ӻ��ڴ�ʱҪ����cpu, �ӿ��ȡ�ٶ�

                //��ȷ����С��ʱʱ��
                //printf("check old_connid=%d, poll improve frequency, CPU overload !! nTimeOutMS=%d ", s_hOldConnID[i].hconnid , nTimeOutMS);
            }
        }

        //������ݳ�ʱʱ����ȡ
        for (i = 0; i<MAX_CONN_NUM; i++){//���������, �Ƿ������ݿɶ�, ���߿��Թر���
            if (s_hOldConnID[i].hconnid >0){
                bExistConn = 1; //list�л�������
                ret = my_ses_recv(i);
                if (0 == ret || (CONN_FINISH == s_hOldConnID[i].nconnStatus)){ //��Щ���ӿ��������ر�, ��ôҲ���Թر�
                    GDEBUG("read finish, close old_id=%d", s_hOldConnID[i].hconnid);
                    glnk_ses_close(s_hOldConnID[i].hconnid);
                    memset(& s_hOldConnID[i], 0, sizeof(STRU_CONN_INFO));
                }
            }
        }

        if (0 == bExistConn){ //2 �ղ�,����poll�ٶ�
            nTimeOutMS = TIMEOUT_IDLE;
            //printf("change back poll slowly nTimeOutMS=%d", nTimeOutMS);
        }
    }
}

void glnk_cc_switch_init()
{
    int ret = 0;
    pthread_t tThreadCC,  tThreadSes, tThreadAlarm;

    memset(struSenseorNode, 0, sizeof(SENSOR_NODE) * MAX_SENSOR_NODE);

    //glnk_cc_test();

    ret = pthread_create(&tThreadSes, NULL, Fnx_GlnkSesProcess, NULL); //͸���ӿ�
    if (ret != 0){
        printf("glnk_cc_switch_init, create Fnx_GlnkSesProcess failed !!!!! \n");
    }
    return;
}

//////////////////////////////////�����ǲ��Ժ���///////////////////////////////////////
void test_glnk()
{
    char *pbuf = NULL;
//     pbuf = "{\"pir_alarm\":{\"set\":{\"on-off\": true, \"time\":20}}}";
//     glnk_pir(pbuf);

    pbuf = (char *) "{\"alarm_action\":                       \
                   {\"set\":                         \
                         {                           \
                         \"PirEnable\": 1,           \
                         \"MotionEnable\": 0,        \
                         \"MotionSensitive\": 2,     \
                         \"FigureFilterEnable\":1,   \
                         \"Action\":[                \
                                     { \"StartTime\":0,    \"EndTime\":1440,  \"ActionSound\":  1, \"ActionRecord\": 1, \"ActionPushup\": 1}, \
                                     { \"StartTime\":0,    \"EndTime\":1440,  \"ActionSound\":  1, \"ActionRecord\": 1, \"ActionPushup\": 1}, \
                                     { \"StartTime\":0,    \"EndTime\":1440,  \"ActionSound\":  1, \"ActionRecord\": 1, \"ActionPushup\": 1}, \
                                     { \"StartTime\":0,    \"EndTime\":1440,  \"ActionSound\":  1, \"ActionRecord\": 1, \"ActionPushup\": 1} \
                                    ]\
                          } \
                    } \
            } ";
    //glnk_alarm_action(pbuf);

    pbuf = (char *) "{\"alarm_action\":{\"get\":null}}";
    glnk_alarm_action(pbuf);

    return;

    for (int i = 0; i<MAX_SENSOR_NODE; i++){ //��������sensor�ڵ�, �����õ�sensor��Ϣȫ����ӡ��sendbuf��
        sprintf(struSenseorNode[i].id, "M%d", 1000 + i);
        sprintf(struSenseorNode[i].szLocation, "location_%d", i);
        struSenseorNode[i].bAccepted = 0;
        struSenseorNode[i].bHandledByApp = 0;
    }

    printf("\n\n");

#if 0
    parse_json_cmdtype(s_SesBufSendOut, struUserType, sizeof(struUserType)/sizeof(USERDEF_CMD));
#endif
}


void glnk_cc_test()
{
    test_glnk();
}
