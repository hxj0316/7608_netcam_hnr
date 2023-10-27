#ifndef __LT_WIFI_CONFIGURE__H_
#define __LT_WIFI_CONFIGURE__H_
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

//��ȡ��İ汾
char *LT_GetVoiceWifiConfigVersion();

/* =============================================
		��������wifi����
		return
			1		�����ɹ�
			0		����ʧ��
rateΪ����Ƶ�ʣ���Χ��8000--44100���������˷�ΧʱΪȱʡֵ44100��
================================================*/
int32_t LT_VoiceWifiConfigStart(char *gid, int rate);

/* ==============================================================
		����wifi���ò�ѯ
		return
			1		���յ�wifi������Ϣ
			0		������

			����������Ϊ1ʱ��wifi��ssid��password��
			�ֱ�ͨ��char **pSsid, char **pPasswd������
            ע�⣺�˺���������ֱ�ӷ��ص�
================================================================*/
int32_t LT_VoiceWifiConfigPoll(char **pSsid, char **pPasswd);

/* =============================================
		�ر�����wifi����
================================================*/
void LT_VoiceWifiConfigClose();

#ifdef __cplusplus
}
#endif

#endif
