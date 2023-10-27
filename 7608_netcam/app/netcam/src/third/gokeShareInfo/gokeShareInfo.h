#ifndef GOKE_MOJING_H
#define GOKE_MOJING_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int freshNow;
    int upgradeNow;
    int cmdType;
    char extralData[512];
}GOKE_SHARE_INFO;


typedef enum
{
    GOKE_CMD_FORMAT = 1
}GOKE_CMD_TYPE;

/*********************
goke_share_udp_info
type:
1.����
2.�ָ�����
3.��ʼ����
4.��ʽ��
5.�ڴ濨����
6.�ڴ濨������
7.�δ���
8.�������
*********************/
int goke_share_udp_info(int type, char *contentStr);

#ifdef __cplusplus
    }
#endif

#endif
