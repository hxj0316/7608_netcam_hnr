#ifndef CC1310MGR_H
#define CC1310MGR_H

#if defined( __cplusplus)
extern "C" {
#endif


#define MAX_OBJ 256
typedef struct JS_HEAP_OBJ_MGR{
    int num;
    cJSON * Json[MAX_OBJ];
}JS_HEAP_OBJ_MGR;

extern int nTimeOutMS; //̫���˴�������; ̫���˳�cpu����Ҳ����ȡ
#define TIMEOUT_DOWNLOAD 30 //���ٶ�
#define TIMEOUT_JSON 200 //���ٶ�
#define TIMEOUT_IDLE 800 //����poll�ٶ�
#define MAX_CONN_NUM 12 //һ���Դ������ӵĸ���; ̫���˵Ļ�, ���ڱ���ʱ��, ��Ƶ���ؾܾ���.


    //Э��ͷ:
#define KEY_CC_LIST    "cc_list"
#define KEY_CC_ALARM   "cc_alarm"
#define KEY_CC_ADD     "cc_add"
#define KEY_CC_DEL     "cc_delete"
#define KEY_CC_WAKEUP  "cc_wakeup"
#define KEY_CC_SETWIFI "cc_setwifi"
#define KEY_PIR        "pir"
#define KEY_HUMANTRACE  "humantrace"
#define KEY_NETLED      "netled"
    //Э���е��ֶ�:
#define KEY_SENSORID    "id"
#define KEY_SENSORLOC   "location"
#define KEY_ACCEPT      "accept"
#define KEY_HANDLEBYAPP "handledbyapp"
#define KEY_ALARMING    "alarming"
#define KEY_TYPE        "type"
#define KEY_SSID        "ssid"
#define KEY_PASSWD      "passwd"
#define KEY_HT_SWITCH    "isOn"
    //20180820 ����pir�ֶ��е�key
#define KEY_OP_GET "get"
#define KEY_OP_SET "set"
#define KEY_PIR_SWITCH  "on-off"
#define KEY_PIR_SPANTIME  "time"
    //20180913 ����alarm-action�ֶ��е�key
#define KEY_ALARM_ACTION "alarm_action"
#define KEY_AA_PIRENABLE "PirEnable"
#define KEY_AA_MOTIONENABLE "MotionEnable"
#define KEY_AA_MOTIONSENSITIVE "MotionSensitive"
#define KEY_AA_FIGUREPILTERENABLE "FigureFilterEnable"
#define KEY_AA_STAYDEAYTIME       "StayDelayTime"
#define KEY_AA_ALARMAUDIOTYPE     "AlarmAudioType"
#define KEY_AA_ACTION             "Action"
#define KEY_AA_ACTION_START       "StartTime"
#define KEY_AA_ACTION_END         "EndTime"
#define KEY_AA_ACTIONRECORD       "ActionRecord"
#define KEY_AA_ACTIONSOUND        "ActionSound"
#define KEY_AA_ACTIONLIGHT        "ActionLight"
#define KEY_AA_ACTIONV12          "ActionV12"
#define KEY_AA_ACTIONPUSHUP       "ActionPushup"


//���������д���Э��
    typedef enum _USERDEFINE_PROTOCOLTYPE
    {
        USRDEF_CC_LIST = 1, //
        USRDEF_CC_ADD,
        USRDEF_CC_DEL,
        USRDEF_CC_ALARM,
        USRDEF_CC_WAKEUP, //����ĳ��cc
        USRDEF_CC_SETWIFI, //��ĳ��cc����wifi
        USRDEF_PIR, //20180820 �����PIR��͸��Э��
        USRDEF_ALARM_ACTION, //20180913
        USRDEF_DOWNLOAD, //20180915
        USRDEF_HUMANTRACE, //20190124
        USRDEF_NETLED, //20190225
        USRDEF_AF_NOTIFY, //20190401
    }USERDEFINE_PROTOCOLTYPE;

    typedef enum _CONN_STATUS
    {
        CONN_FINISH = 10,
    }CONN_STATUS;

    typedef struct STRU_CONN_INFO{
        int hconnid; //����id
        int iType; //��������Ǹ�ʲô��: USERDEFINE_PROTOCOLTYPE
        int nconnStatus; //��һ��ֵ�����ӿ�ر�
    }STRU_CONN_INFO;

    typedef struct USERDEF_CMD{
        int iType;
        const char * szName;
    }USERDEF_CMD;
#define SES_BUF_SIZE 2048

    //ÿһ����������Ҫ�������Ϣ
    typedef struct SENSOR_NODE{
        char id[32]; //��������macid
        char szLocation[128]; //�ô��������ڵ�ַ����
        int bAccepted;
        int bHandledByApp; //�Ƿ��Ѿ���app�����
        int bAlarming; //�Ƿ����ڱ���
        char sztype[2]; //һ���ַ���ʾ����, һ����ʾ��β��0
    }SENSOR_NODE;
#define MAX_SENSOR_NODE 4 //���������sensor����
    extern SENSOR_NODE struSenseorNode[MAX_SENSOR_NODE];

    //=================================================
    //=================================================
    //=================================================
    //=================================================

#define KEY_NEWID   'n'
#define KEY_EXIT    'e'
#define KEY_ALERT   'a'
#define KEY_STATUS  's'
#define KEY_DELETE  'd'
#define KEY_WAKEUP  'u'
#define KEY_SETWIFI 'w'
    typedef struct CC_CMD{
        int iType;
        char cName;
    }CC_CMD;

    typedef enum _CC_PROTOCOLTYPE
    {
        //��CC���ܵ�����
        CC_NEWID, //����ID�ϱ�
        CC_EXIT, //ɾ��id
        CC_ALERT,  //�б�������
        CC_STATUS,
        //���͸�cc������
        CC_DELETE, //���������
        CC_WAKEUP, //���������
        CC_SETWIFI, //����wifi��Ϣ
    }CC_PROTOCOLTYPE;

void glnk_cc_switch_init();
void glnk_cc_test();
int glnk_led_control_state(void);

void test_cc();

int cc_get_newid(char *pbuf, int len); //cc��������
int cc_get_exit(char *pbuf, int len); //cc�����뿪
int cc_get_alarming(char *pbuf, int len); //cc��������

int cc_set_delete(char * szDelId); //dev����ɾ��ccĳid
int cc_set_wakeup(char * szDelId); //dev��������ccĳid
int cc_set_wifi(char * szDelId, char * szssid, char * szpasswd); //dev��������wifi, ���ȫ��cc
void cc_send_one_msg(char *pbuf, int len);

int MP4Sys_Set_PWR(int bEnable,int time);
int MP4Sys_Get_PWR(int *state);

int MPU_SetAutoHumanTrace(int bEnable); //�·��Զ�����׷�ٿ���
int MPU_GetAutoHumanTrace();
int MPU_GetNetLed();
int MPU_SetNetLed(int bshow);

char * JsPrintFmt(cJSON * Json);
char * JsPrintStr(cJSON * Json);
char * JsToBuffer(cJSON * Json, char * pbuf);
void JsHeapObjFree(JS_HEAP_OBJ_MGR * pJsMgr);
cJSON *cJSON_CreateObjectEx(JS_HEAP_OBJ_MGR * pJsMgr);
cJSON *cJSON_CreateArrayEx(JS_HEAP_OBJ_MGR * pJsMgr);
cJSON *cJSON_ParseEx(JS_HEAP_OBJ_MGR * pJsMgr, const char *value);

extern STRU_CONN_INFO s_hOldConnID[MAX_CONN_NUM];

#if defined( __cplusplus)
};
#endif

#endif
