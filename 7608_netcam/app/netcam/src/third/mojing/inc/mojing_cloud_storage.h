#ifndef __MOJING_CLOUD_STORAGE_H__
#define __MOJING_CLOUD_STORAGE_H__

enum
{
	CLOUD_STORAGE_NONE = 0,
	CLOUD_STORAGE_ALL_DAY,
	CLOUD_STORAGE_DYNAMIC,
};

enum
{
	CLOUD_STATUS_OK = 0,
	CLOUD_STATUS_GET_CONFIG,
	CLOUD_STATUS_NO_CLOUD_STORAGE,
};


typedef int (*cloud_storage_notify_end_cb)(int storage_type,char *task_id,long long start_time,long long end_time);
typedef int (*cloud_storage_status_check_cb)(int cloud_storage_type, int *endtime);
typedef void* (*cloud_storage_tls_init_cb)(int fd,char *host, int port);
typedef int (*cloud_storage_tls_send_cb)(void *ssl_handle,char *buffer, int dataLen);
typedef int (*cloud_storage_tls_exit_cb)(void *ssl_handle);

typedef struct
{
	int cloudStorageType; 		//�ƴ�����(0:���ƴ�,1:ȫ��洢,2:��̬�洢)
	int streamType;				//��������(0:������,1:������,2:��������)
	int preStorageTimeSec;		//��̬�洢ʹ��-Ԥ��ʱ��(��) 0-10
	int keepTimeSec;			//��̬�洢ʹ��-�洢����ʱ��(��)30-300
	char taskID[64]; 			//��ͨ��ȫ��洢��������������ض�ӦTaskID
	int	transType;				//��������(0:TCP,1:HTTP,2:TCP+TLS,3:HTTPS)
	void *ssl;					// Mbedtls_HandleT
	int motionStartTime;
	int motionEndTime;
	char cloudStorageStatus;
	
	cloud_storage_notify_end_cb 	notify_end_cb;
	cloud_storage_status_check_cb 	status_check_cb;
	cloud_storage_tls_init_cb		tls_init_cb;
	cloud_storage_tls_send_cb		tls_send_cb;
	cloud_storage_tls_exit_cb		tls_exit_cb;
	
}CloudStorageControlT;

int mojing_cloud_storage_start(char *media_server_ip,int media_server_port, unsigned int trans_type, 
										int ssrc, CloudStorageControlT *cloudInfo);

#endif
