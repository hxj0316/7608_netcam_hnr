#ifndef __NET_ARPING_H__
#define __NET_ARPING_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
	interface : ����
	ipaddr    : ����IP
	macaddr	  : ���Ҫ��ȡ���IP��MAC��������18���ַ��Ŀռ����ڴ�ȡMAC
 	return 	  :
 				-1 : ��ȡ����������Ϣʧ��
 				0  : ����IP����ʹ��
 				1  : ����IP������ʹ��
*/
extern int check_ip(char *interface, char *ipaddr, char *macaddr);

#ifdef __cplusplus
}
#endif


#endif //__NET_ARPING_H__

