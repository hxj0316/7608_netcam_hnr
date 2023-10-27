#ifndef TUTK_PUSH_FUNC_H
#define TUTK_PUSH_FUNC_H

#include "AVIOCTRLDEFs.h"


//gpio define
#define DBC_AUDIO_ENABLE	2
#define DBC_RESET			3
#define DBC_PIR				5
#define DBC_R_LED			7
#define DBC_B_LED			8
#define DBC_CALL			9
#define DBC_LOCK			16


/*�������Ͳ���Э��汾,
V2����HTTP��ʽ������ʹ�ø�Э��
V3����HTTPS��ʽ������ʹ�ø�Э��
Ĭ��ʹ��V2Э�顣ʹ��V3Э��ʱ��Ҫ����SSL�⣬����Ӧ�ñ��0.3M���ҡ�
*/
#define USE_JIGUANGPUSH_V2   0 
typedef struct
{
	SMsgAVIoctrlSetPushReq pi;
	int type;
	int number;
	int channel;
	time_t ts;
}RL_Push_MsgT;

void* RLAlarmPushThread(void *param);


void AlarmPush(SMsgAVIoctrlSetPushReq * pi, int type,int channel,time_t ts);
int PPDevSetupGPIO(int ActiveLow, int Direction, int Pin,int Value);
int PPDevGetGPIO(int Pin);
int PPDevSetGPIO(int Pin,int Value);
int door_open(void* arg);

int HandleGet(
	int 			FileHandle,
	char * 			Buffer, 
	int 			nBytesNeed
);

int HandlePut(
	int 			FileHandle,
	char * 			Buffer, 
	int 			nBytesNeed
);

#define LED_RED_ON()  // ({PPDevSetGPIO(DBC_R_LED,0);PPDevSetGPIO(DBC_B_LED,1);})
#define LED_BLUE_ON() // ({PPDevSetGPIO(DBC_R_LED,1);PPDevSetGPIO(DBC_B_LED,0);})


int tcp_connect_timedwait(int sockfd, struct sockaddr *peeraddr, socklen_t len, int seconds);
void* RLGooglePushThread(void *param);

int tutk_resolve_host(struct in_addr *sin_addr, const char *hostname);
char * StringSafeCopyEx(
	char *			To,
	const char *	From,
	int				ToBufferSize,
	const char *	Begin,
	const char *	End
);

int TCPSocketClientTimeout(
	const char * 	DeviceIP,
	int				DevicePort,
	int				Timeout
);



int SimpleMsgPush_SSL_V3(
	const char *	Akey,
	const char *	MasterSecret,
	const char *	id,
	const char *	tipMsg,
	int				timestap,
	int				eventType,
	int				channel,
	int				Timeout
)   ;

int tcp_connect_timedwait_ms(int sockfd, struct sockaddr *peeraddr, socklen_t len, int ms);
int tutk_resolve_host(struct in_addr *sin_addr, const char *hostname);

#endif

