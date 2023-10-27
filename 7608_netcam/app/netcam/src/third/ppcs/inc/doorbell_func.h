#ifndef DOORBELL_FUNC_H
#define DOORBELL_FUNC_H

#include "AVIOCTRLDEFs.h"


//gpio define
#define DBC_AUDIO_ENABLE	2
#define DBC_RESET			3
#define DBC_PIR				5
#define DBC_R_LED			7
#define DBC_B_LED			8
#define DBC_CALL			9
#define DBC_LOCK			16

//audio file
#define AUDIO_FILE_ALARM             "alarm.alaw"        //�ƶ����澯--
#define AUDIO_FILE_CALLING           "calling.alaw"      //����
#define AUDIO_FILE_CALLTIMEOUT       "call-timeout.alaw" //���г�ʱ
#define AUDIO_FILE_CONFIG            "config.alaw"       //--
#define AUDIO_FILE_CFGOK             "configok.alaw"     //һ������--
#define AUDIO_FILE_MUTE              "mute.alaw"         //--
#define AUDIO_FILE_OPEN              "open.alaw"         //����
#define AUDIO_FILE_REBOOT            "reboot.alaw"       //����p2p id--
#define AUDIO_FILE_RESETOK           "reset-ok.alaw"     //��λ��
#define AUDIO_FILE_STARTUP           "startup.alaw"      //ϵͳ����
#define AUDIO_FILE_TALKOVER          "talk-over.alaw"    //ͨ������
#define AUDIO_FILE_TALKTIMEOUT       "talk-timeout.alaw" //ͨ����ʱ
#define AUDIO_FILE_WELCOME           "welcome.alaw"      //���п�ʼ
#define AUDIO_FILE_WPS               "wps.alaw"          //--


void AlarmPush( int type);
int PPDevSetupGPIO(int ActiveLow, int Direction, int Pin,int Value);
int PPDevGetGPIO(int Pin);
int PPDevSetGPIO(int Pin,int Value);
int door_open(void* arg);
int loadrunparam(void);
int GetSystemParam(SMsgAVIoctrlGetSysparamResp* p);
int GetMoveAlarmParam(SMsgAVIoctrlGetMDPResp* p);
int SetPushDevParam(SMsgAVIoctrlSetPushReq* pPushReq);
int DelPushDevParam(SMsgAVIoctrlSetPushReq* pPushReq);
int SetMoveAlarmParam(SMsgAVIoctrlSetMDPReq* p);
int check_alarm_enable();




#define LED_RED_ON() ({PPDevSetGPIO(DBC_R_LED,0);PPDevSetGPIO(DBC_B_LED,1);})
#define LED_BLUE_ON() ({PPDevSetGPIO(DBC_R_LED,1);PPDevSetGPIO(DBC_B_LED,0);})


#endif //DOORBELL_FUNC_H

