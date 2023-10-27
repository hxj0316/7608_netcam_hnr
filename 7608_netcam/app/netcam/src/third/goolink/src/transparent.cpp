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
#include "transparent.h"
#include "debug.h"
#include "goolink_cc_switch.h"
#include "netcam_api.h"

extern char s_SesBufSendOut[SES_BUF_SIZE];

unsigned char AF_get_maxzoom();
unsigned char AF_inquery(unsigned char u8Cmd, int timeoutms); //from hisdk
void AF_register_af_notfiy(FUNCNOTIFY pfunc, int item);

static int model_id = -1;
static char model_filename[32] = {0};
static int model_filesize = 0;
#define FILE_TMP_ALERT    "/tmp/alert.wav" //�����ص�tmpĿ¼, ���move��ȥ
#define FILE_FINAL_ALERT  "/opt/custom/audio/alert.wav"//"/etc/Wireless/alert.wav" //������Ҫ���ǵ��ļ�
#define SIZE_WAVE_FILEHEADER 0x1000
#define MAX_FILESIZE (300 * 1024) //Ŀǰ�趨���ܳ���300K

int mysystem(const char *cmdstringl)
{
	printf(" %s:%d \n",__func__,__LINE__);
	new_system_call(cmdstringl);
}

int32_t DownLoadAlertFile(char *filename,
                        int type,
                        int size, //�ļ��ܳ���
                        char *data,
                        int len)  //�ô��·��ĳ���
{
    GDEBUG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> \n");
    int32_t ret = 0;
    static FILE *fd = NULL;
    static int count_size = 0;
    GDEBUG("%s, type = %d, size = %d, len = %d\n", filename, type, size, len);
    if(type == 1) //��ʼ����, ���ļ����
    {
        GDEBUG("begin 111111111111111 size = %d, filename=%s\n", size, filename);
        count_size = 0; //���¿�ʼ����
        //check flash
        if( (fd = fopen(FILE_TMP_ALERT, "w+")) == NULL ){
            GDEBUG( "begin 111111222222, openfile failed !!! ret=0, [%s] error [%s]\n", filename,strerror(errno));
            ret = 0;
        }
        else{
            GDEBUG( "begin 111111333333333, success \n");
            ret = 1;
        }
    }
    else if(type == 2) //���ؽ���, ���ϵ�д�ļ�
    {
        GDEBUG("downloading 2222222222222 len=%d, count_size=%d, fd=%d, date=%p \n", len, count_size);
        if (count_size > MAX_FILESIZE){ //�������ļ�������С��ܾ������������
            //������������, count_size����������Ϊ0, �����ʱ�͸�λ, �����ֿ�ʼ�ۻ�
            GDEBUG("recived file = %d has been transfered over %d, skip later file segement!!", count_size, MAX_FILESIZE);
            if(fd != NULL){ //�ص���ʱ�ļ�
                fclose(fd);
                fd = 0;
                mysystem("mv "FILE_TMP_ALERT " "FILE_FINAL_ALERT); //ǰ�洫�������ļ����ǽ���
            }
            ret = 0; //ʧ�ܾܾ�
        }
        else{
            count_size += len; //�ۼ����ص��ļ���С
            if(fd != NULL && data != NULL) //20180915 ��Ϊǰ�洫���Ǵ�wavͷ����PCM�ļ�, �Ǳ߲��ô���, ����ֱ�Ӷ�����.
            {
                if(count_size > SIZE_WAVE_FILEHEADER){ //ֻ�д��ڲ�д��
                    fwrite(data, len, 1, fd); //���ļ�д�뵽flash
                    fflush(fd);
                }
                ret = 1;
            }
            else{
                GDEBUG("downloading 22222222333333 NO write file, ret=0, len=%d, count_size=%d \n", len, count_size);
                ret = 0;
            }
        }
    }
    else if(type == 3) //�������
    {
        GDEBUG("completed successfully! 333333333333333333 len=%d, count_size=%d \n", len, count_size);
        if(fd != NULL){
            fclose(fd);
            fd = 0;
            mysystem("mv "FILE_TMP_ALERT " "FILE_FINAL_ALERT); //����������
        }
        count_size = 0;
        ret = 1;
    }
    else if(type == 4)// ʧ��
    {
        GDEBUG("failed 4444444444444444444 len=%d, count_size=%d \n", len, count_size);
        if(fd != NULL){
            fclose(fd);
            fd = 0;
        }
        GDEBUG("����ʧ��\n");
        count_size = 0;
        ret = 1;

        //��ʱɾ���ļ�
        GDEBUG("failed!! remove file: %s \n", FILE_TMP_ALERT);
        mysystem("rm "FILE_TMP_ALERT);
    }

    return ret;
}

int  download_file(char * bufrecv)
{
    TLV_V_UploadAlarmResponse *phead = (TLV_V_UploadAlarmResponse *)bufrecv;
    for (int i = 0; i<sizeof(TLV_V_UploadAlarmResponse); i++){
        if (i>0 && i%32 == 0){
            printf("\n");
        }
        if (i%4 == 0){
            printf("  ");
        }
        printf("%02x ", bufrecv[i]);
    }

    char *filename = (char *)FILE_TMP_ALERT;
    int ret = 0;
    int type   = phead->type;
    int size   = phead->size;
    char *data = bufrecv + sizeof(TLV_V_UploadAlarmResponse);
    int len    = phead->len;
    ret = DownLoadAlertFile(filename, type, size, data, len);


    TLV_V_DownloadAlarmResponse * pret =  (TLV_V_DownloadAlarmResponse * )s_SesBufSendOut; //�����ڴ�
    strncpy(pret->szToken, DOWNLOADFILE, 16);
    pret->result = ret;
    pret->reserve = 0;

    GDEBUG("return : ret=%d\n", ret);
    return sizeof(TLV_V_DownloadAlarmResponse);
}
