#ifndef NTP_CLIENT_INCLUDE_H
#define NTP_CLIENT_INCLUDE_H

/**
	ʹ��ntp client
*/
void ntpc_enable();

/**
	��ʹ��ntp client
*/
void ntpc_disable();

/**
	����ntp server�ĵ�ַ
	ntpserver: ntp server��������ַ
	serLen:    ������ַ�ĳ���
	port: ntp server�Ķ˿ں�,  < 0��ʾʹ��Ĭ�϶˿ں�123
*/
void ntpc_set_addr(char *ntpserver, int serLen, int port);

/**
	����ntp client��ͬ��ʱ��
	synctime: ͬ��ʱ�䵥λs
*/
void ntpc_set_synctime(int synctime);

/**
	���� ntp client
*/
int start_ntp_client();

#endif //NTP_CLIENT_INCLUDE_H

