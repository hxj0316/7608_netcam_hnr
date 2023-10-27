#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"


int main (int argc, const char * argv[])
{
	cJSON *json;
	double *test;
	char *out;

	json=cJSON_ReadFile("./audio.json");//���ļ�ϵͳ�ж�ȡ JSON ��Ŀ
	
	out=cJSON_GetItemValue( json, "audioEncode.audioEncodeChannel[0].codecTypeProperty.opt[1]" );//��ȡ��Ŀֵ
	printf("%s\n",out);

	test=cJSON_GetItemValue( cJSON_GetObjectItem(json, "audioInput"),"audioInputChannel[0].id" );
	printf("%d\n",(int)*test) ;

	out=cJSON_GetItemValue( cJSON_GetObjectItem(json, "audioEncode"), "audioEncodeChannel[0].codecTypeProperty.type" );
	printf("%s\n",out);

	cJSON_SetItemValue( json, "audioEncode.audioEncodeChannel[0].codecTypeProperty.opt[0]", "abcdefjsd" );//������Ŀֵ
	out=cJSON_GetItemValue( json, "audioEncode.audioEncodeChannel[0].codecTypeProperty.opt[0]" );
	printf("%s\n",out);

	cJSON_WriteFile( "./audiotest.json", json );//���� JSON ��Ŀ���ļ�ϵͳ��
	
	return 0;
}
