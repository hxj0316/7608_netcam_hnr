#ifndef TRANSPARENT_H
#define TRANSPARENT_H

#if defined( __cplusplus)
extern "C" {
#endif
#define TOKENSIZE 16
#define DOWNLOADFILE "downloadfile" //����Ĳ��ܳ���15bytes !
    typedef struct _TLV_V_UploadPanoAlarmResponse
    {
        char    szToken[TOKENSIZE];
        char 	filename[32];					//remap.pano ���� weight.pano
        int		type;							//1:��ʼ����  2�������� 3���������  4������
        int 	size;							//�ļ��ܳ���
        int		len;							//��ǰ���ݳ���
    }TLV_V_UploadAlarmResponse;

    typedef struct _TLV_V_DownloadAlarmResponse
    {
        char szToken[TOKENSIZE];  //��������
        int result;								//1--������� 0--ʧ��
        int reserve;							//����
    }TLV_V_DownloadAlarmResponse;


    int  download_file(char * bufrecv);

    int mysystem(const char *cmdstringl);

//=========================================================================================
//������maxzoomʱ����֪ͨ�Է�
enum{
    PROTOCOL_INQURY_MAXZOOM = 1,
    PROTOCOL_INQURY_CURZOOM = 2,
    PROTOCOL_BEGIN_CALLBACK = 100,
};
#define AF_CALLBACK "af_callback"
#define AF_TOKENSIZE 16
    typedef struct _AF_NOTIFY
    {
        char  szToken[AF_TOKENSIZE];
        int   protocol; 
        int   data;
    }AF_NOTIFY;
/*
1 ��ѯmaxzoom: protocol=1: ��ѯmaxzoom, dataΪ��ѯ���Ľ��
2 ��ѯcurzoom: protocol=2: ��ѯcurzoom, dataΪ��ѯ���Ľ��
3 ���������ص�: protocol=100, data=1��ʾ����minzoom; data=2��ʾ����maxzoom
*/

extern int af_countdown;
extern int g_af_item;
void * Thread_AF_notify(void *lParam);

enum{
    INQUERY_BOARD    = 0x01,
    INQUERY_MAX_ZOOM ,
    INQUERY_CUR_ZOOM ,
};

int handle_af(int item, AF_NOTIFY * phead);
typedef int (*FUNCNOTIFY) (int item, int ntype);


#if defined( __cplusplus)
}
#endif

#endif