//
// Created by ���� on 2019-09-06.
//

#include "devsdk.h"
#include "protool.h"
#include <string.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"
#include "common.h"
#include "sdk_def.h"
#include "cfg_system.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/if_packet.h>
#include <netinet/if_ether.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>

static char g_dev_sn[128];
char devVer[64] = {0};
extern char softVersion[32];
PLUG_CONFIG local_cfg = {0};

//extern PLUG_CONFIG local_cfg;

U16 get_dev_mac(U8 *mac){
    //��ȡMAC�Ĺ��ܳ����Լ�ʵ�֣������������ο�
	char device[16]="eth0"; //teh0�������豸��
    unsigned char macaddr[ETH_ALEN]; //ETH_ALEN��6����MAC��ַ����
    //AF_INET = 1;
    int i,s;
    s = socket(AF_INET,SOCK_DGRAM,0); //�����׽ӿ�
    struct ifreq req;
    int err;
    char *ret;

    ret = strcpy(req.ifr_name,device); //���豸����Ϊ�����������
    err=ioctl(s,SIOCGIFHWADDR,&req); //ִ��ȡMAC��ַ����
    close(s);
    if( err != -1 )
    {
        memcpy(macaddr,req.ifr_hwaddr.sa_data,ETH_ALEN); //ȡ�����MAC��ַ
        printf("########MAC= \n");
        for(i=0;i<ETH_ALEN;i++)
            printf("%02X:",macaddr[i]);
        memcpy(mac, macaddr, ETH_ALEN);
    }
    printf("\r\n");
    return 0;
}

U16 get_dev_sn(char * sn)
{
    //��ȡSN�ŵĹ��ܳ����Լ�ʵ�֣������������ο�

    if (sn == NULL){
        return -1;
    }
    U8 mac[6];
    memset(mac, 0, sizeof(mac));
    get_dev_mac(mac);
    sprintf(sn, "%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX|%s", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], g_dev_sn);
    return 0;
};

U16 set_dev_sn(char * sn, int len)
{
    if ((sn == NULL) || (len == 0)){
        return -1;
    }
    memset(g_dev_sn, 0, sizeof(g_dev_sn));
    strncpy(g_dev_sn, sn, sizeof(g_dev_sn)-1);
    return 0;
};

U16 get_dev_version(char * version){
    //��ȡ�̼��汾��Ϣ�Ĺ��ܳ����Լ�ʵ�֣������������ο�

    if (version == NULL){
        return -1;
    }
    //char test_version[256] = "V0.0.1";
    if (strlen(devVer) == 0)
    {
        sprintf(version, "%s", softVersion);
    }
    else
    {
        strcpy(version, devVer);
    }
    return 0;
}

//���͹̼��汾��Ϣ
int dev_version_report()
{
    int ret = 0;

    char dev_version[256] = {0};

    data_device_mt_req report_msg;

    report_msg.dataType = datatype_version_report;

    cJSON * pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        //error happend here
        return -1;
    }

    ret = get_dev_version(dev_version);

    cJSON_AddStringToObject(pJsonRoot, "version", dev_version);

    char * pJson = cJSON_Print(pJsonRoot);
    if(NULL == pJson)
    {
        cJSON_Delete(pJsonRoot);
        return -2;
    }
    report_msg.dataByte = pJson;
    report_msg.length = strlen(report_msg.dataByte);

    printf("######## %s %d: len = [%d] pJson = \n%s\n\n", __func__, __LINE__, report_msg.length, report_msg.dataByte);

    send_management_up_data((void *)&report_msg, sizeof(report_msg)+report_msg.length);

    cJSON_Delete(pJsonRoot);

    return 0;
}

/*int dev_fwversion_report()
{
    int ret = 0;

    char dev_version[256] = {0};

    data_device_mt_req report_msg;

    report_msg.dataType = 14;

    cJSON * pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        //error happend here
        return -1;
    }

//	ret = get_dev_version(dev_version);
    memset(dev_version,0,sizeof(dev_version));
    memcpy(dev_version,"FW-V0.0.2",9);

    cJSON_AddStringToObject(pJsonRoot, "version", dev_version);

    char * pJson = cJSON_Print(pJsonRoot);
    if(NULL == pJson)
    {
        cJSON_Delete(pJsonRoot);
        return -2;
    }
    report_msg.dataByte = pJson;
    report_msg.length = strlen(report_msg.dataByte);

    printf("######## %s %d: len = [%d] pJson = \n%s\n\n", __func__, __LINE__, report_msg.length, report_msg.dataByte);

    send_management_up_data((void *)&report_msg, sizeof(report_msg)+report_msg.length);

    cJSON_Delete(pJsonRoot);

    return 0;
}*/

/// -------------�豸---------------
// *�豸��������*

// data����������json�ַ���
// data_len�����ݳ���
///-- ��ע���ڲ���ҪӦ�������£�responseֻ��Ҫ�ظ����������е�sequence���ݼ��� --

// ������
//     ��������data����Ϊ��{"sequence":65535,"SongControlS":2}
//     Ӧ������response����Ϊ��{"sequence":65535,"xxx1":"",......}������Ӧ�����ݳ���sequence�⣬���ݾ����龰���԰����������
//     ---����Ӧ�������е�sequenceΪ���������е�sequence��ֵ
int dev_data_down(const char *data, int data_len){
    printf("######## data=%s len=%d \n", data, data_len);
    printf("######## %s OK. \n", __func__);
    int ret = -1;

    if(NULL == data)
    {

        return ret;
    }
    /// ����һ�·����������ݣ�Ҳ�����д������ݣ����ο�
    /*cJSON * pJsonData = NULL;
    cJSON * pJsonRes = NULL;
    pJsonData = cJSON_Parse(data);
    pJsonRes = cJSON_CreateObject(); //cJSON_Parse("{\"sequence\":0}");

    for(int i=0; i<cJSON_GetArraySize(pJsonData); i++)   //���������json��ֵ��
    {
        cJSON * item = cJSON_GetArrayItem(pJsonData, i);

        *//* -- ��ӡjson��ֵ�� --
        if(cJSON_String == item->type)
        {
            printf("%s->", item->string);
            printf("%s\n", item->valuestring);
        }
        else if(cJSON_Number == item->type)
        {
            printf("%s->", item->string);
            printf("%s\n", item->valueint);
        }*//*

        if(!strcasecmp("sequence",item->string))
        {
            cJSON_AddNumberToObject(pJsonRes, item->string, item->valueint);
            ret = 0;
        }
        else if(!strcasecmp("xxx",item->string)) // example
        {
            //...

        }
        //...
    }

    /// Ӧ�����ݵ���Ӿ���
    // ֵΪ�ַ����Ĵ���ʽ
    //    cJSON_AddStringToObject(pJsonRes, string, string);
    // ֵΪ��ֵ�Ĵ���ʽ
    //    cJSON_AddNumberToObject(pJsonRes, string, int);


    if(-1 == ret)
    {
        cJSON_AddStringToObject(pJsonRes, "msg", "parser error");
    }

    char * pJson = cJSON_PrintUnformatted(pJsonRes);
    *response_len = strlen(pJson);
    memcpy(response, pJson, *response_len);

    cJSON_Delete(pJsonRes);
    cJSON_Delete(pJsonData);*/

    /*char response[1024*4] = "";
    int  responselen = 0;
    memcpy(response, data, data_len);
    responselen = data_len;
    send_business_ack_data(0, (unsigned char*) &response, responselen);
    send_business_up_data((unsigned char*) &response, responselen);*/

    return ret;
}
