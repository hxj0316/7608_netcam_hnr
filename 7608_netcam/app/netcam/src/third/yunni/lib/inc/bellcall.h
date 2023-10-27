#ifndef _BELL_CALL_H_
#define _BELL_CALL_H_

//
// ϵͳ������ؽṹ��
//

//
// ELOG ����
//
enum{
	ELOG_MOTION_DETECT,	// �ƶ�����¼�
	ELOG_LOGIN,			// ��¼�¼�
	ELOG_OPERATION		// �����¼�
};

//
// PTZ ����ָ��
//
enum{
    CMD_PTZ_UP,
    CMD_PTZ_UP_STOP,
    CMD_PTZ_DOWN,
    CMD_PTZ_DOWN_STOP,
	CMD_PTZ_LEFT,
	CMD_PTZ_LEFT_STOP,
	CMD_PTZ_RIGHT,
	CMD_PTZ_RIGHT_STOP,
	CMD_PTZ_CENTER=25,
	CMD_PTZ_UP_DOWN=26,

	CMD_PTZ_UP_DOWN_STOP=27,
	CMD_PTZ_LEFT_RIGHT,
	CMD_PTZ_LEFT_RIGHT_STOP,

    CMD_PTZ_PREFAB_BIT_SET0,
    CMD_PTZ_PREFAB_BIT_RUN0,

    CMD_PTZ_PREFAB_BIT_SETF=60,
    CMD_PTZ_PREFAB_BIT_RUNF,
    CMD_PTZ_LEFT_UP=90,
    CMD_PTZ_RIGHT_UP,
    CMD_PTZ_LEFT_DOWN,
    CMD_PTZ_RIGHT_DOWN,
    CMD_PTZ_IO_HIGH,
    CMD_PTZ_IO_LOW,
    CMD_PTZ_MOTO_TEST=255,
};

//
// ���̿���ָ��
//
enum{
	CMD_STORAGE_NONE=0,
	CMD_STORAGE_FORMAT,
	CMD_STORAGE_UMOUNT
};

//
// ý�����ָ��
//
enum{
	CMD_MEDIA_NULL=0,	// ָ��ִ����ɺ�����
	CMD_MEDIA_PLAY,		// �������ļ�
	CMD_MEDIA_TURN,		// ������ͣ�л�
	CMD_MEDIA_DRAG,		// �϶���λ��
	CMD_MEDIA_STOP		// ֹͣ����
};

//
// ý���߳�״̬
//
enum{
	STA_MEDIA_IDLE=0,	// ����ģʽ
	STA_MEDIA_PLAY,		// ����ģʽ
	STA_MEDIA_PAUSE		// ��ͣģʽ
};

//
// ͼ����Ʋ���
//
typedef struct{
    int Brightness;		// 0-255
    int Contrast;		// 0-255
    int Sharpness;		// 0-255
    int Hue;        	// 0-255
    int Saturation; 	// 0-255
    int FilckFrequency;	// 0->50HZ,1->60HZ
    int FlipHor; 		// 0 not; 1, flip turn
    int MirrorVer; 		// 0 not; 2, mirror turn
    int IRCut; 			// 0 close, 1, enable
    int	Range;			// 0 Min Value 1, Max Value 2, Set Value
}PPCGI_IMAGE_T;

//
// ��Ƶ��������
//
typedef struct{
	int SampleRate;	// ������
	int BitRate;	// ������
	int	EncodeType; // ���뷽ʽ
	int Mute;		// ����
    int	Range;		// 0 Min Value 1, Max Value 2, Set Value
}PPCGI_AUDIO_T;

//
// �����Ʋ���
//
typedef struct{
    int StreamID;	// 0-2 //3 stream
    int FPS;		// 1-30
    int BitRate;	// 10-8000 kbps
    int	Range;		// 0 Min Value 1, Max Value 2, Set Value
}PPCGI_VIDEO_T;

//
// OSD ����
//
typedef struct{
	int 	OSDEnable;
	char 	OSDStr;
	int		FontSize;
	int 	FontType;
	int 	FontColor;
	int 	Pos[4];
}PPCGI_OSD_T;

//
// ʱ�����ò���
//
typedef struct{
    int  NTPEnable;		// 0: ����ʱ��ͬ�� 1: �ر�ʱ��ͬ��
    char NTPServer[32];	// ʱ��ͬ��������
    int  TimeZone;   	// seconds for UTC
    int  TimeUTCs; 		// senconds from 1970
}PPCGI_TIME_T;

//
// ��������
//
typedef struct{
	int	MotionEnable;			// �ƶ���⿪��
	int MotionSensitivity;		// �ƶ����������
	int	InputEnable;			// GPIO ����ʹ��
	int ILevel;					// ������Ч��ѹ
	int OLevel;					// �����Ч��ѹ
	int IOLinkage;				// GPIO �����������
	int UploadInterval;			// �����ϴ�����
	int AlarmByMail;			// �ʼ���������
	int AlarmScheduleEnable;	// �����ƻ�����
	int AlarmSnapshotEnable;	// ����ץͼ����
	int AlarmRecorderEnable;	// ����¼�񿪹�
	int AlarmMovePerset;		// �����ƶ�Ԥ��λ
	int AlarmMute;				// ��������
	int AlarmSchedules[7][3];	// �����ƻ�����
}PPCGI_ALARM_T;

//
// �ʼ�����
//
typedef struct{
	char SMTPSvr[128];			// SMTP �����ʼ�������
	int  Port;					// SMTP ���Ͷ˿�
	int  EncryptType;			// �ʼ����ͼ��ܷ�ʽ 0: �����ü��� 1: SSL 2:TLS
	char User[32];				// SMTP ��¼�û�
	char Pass[32];				// SMTP ��¼����
	char MailFrom[128];			// ���������ַ
	char MailTos[4][128];		// ���������ַ,�ܹ�4��
}PPCGI_MAIL_T;

//
// FTP �ͻ��� �ϴ�����
//
typedef struct{
	char FTPSvr[128];			// FTP ������
	int	 Port;					// �˿�
	int  Mode;					// ����ģʽ 0: Port 1: Pasv
	char User[32];				// ��¼�û�
	char Pass[32];				// ����
	char DataPath[255];			// �����ϴ�·��
	char FileName[255];			// �����ļ���
	int	 UploadInterval;		// �ϴ�ʱ����
}PPCGI_FTPC_T;

//
// CIFS �ϴ�����
//
typedef struct{
	char CIFSSvr[128];			// CIFS ������
	char User[32];				// ��¼�û�
	char Pass[32];				// ����
	char DataPath[255];			// �����ϴ�·��
	char CIFSPath[255];			// ����·��
}PPCGI_CIFS_T;

//
// ����ӿ�����
//
typedef struct{
	char Interface[32];			// ����ӿ�
	char IPv4[32];				// ��ַ
	char Mask[32];				// ����
	char Gate[32];				// ����
	int	 DHCP;					// ����DHCP
	char DNS1[32];				// DNS1
	char DNS2[32];				// DNS2
	char MAC[32];				// �����ַ
	int  Type;					// 0: ���� 1: ����
}PPCGI_NETWORK_T;

//
// WIFI����
//
typedef struct{
	char SSID[32];				// SSID
	char Pass[32];				// ��֤����
	char EncryptType;			// ���ܷ�ʽ
	int	 Mode;					// WIFIģʽ 0:AP 1:�м� 2:�ͻ���
	int	 Channel;				// �ŵ�
	int	 Quality;				// ͨ������
	PPCGI_NETWORK_T Network;	// Ӳ����Ϣ
}PPCGI_WIFI_T;

//
// WIFIɨ����
//
typedef struct{
	char SSID[32];				// SSID
	char MAC[32];
	char EncryptType;			// ���ܷ�ʽ
	int  Channel;
	int	 Quality;				// ͨ������
}PPCGI_WIFI_LIST_T;

typedef struct{
	PPCGI_WIFI_LIST_T * Lists;	// WIFI �б�����
	int	 ListSize;				// �����С
}PPCGI_WIFI_SCAN_T;

//
// ¼������
//
typedef struct{
	int	RecorderEnable;					// 0: ����¼�� 1: ֹͣ¼��
	int RecorderTime;					// ¼��ʱ��
	unsigned long long RecorderSize;	// ¼���С
	int	Overwrite;						// 0: �����Ǿ��ļ� 1: ���Ǿ��ļ�
	int RecorderEnableAudio;			// ¼����Ƶ
	int RecorderSchedules[7][3];		// ¼��ƻ�����
}PPCGI_RECORD_T;

//
// ý�����
//
typedef struct{
	int Year;
	int Month;
	int Day;
	int Hour;
	int Min;
	int Sec;
}COMMO_TIME_T;

typedef struct{
	int Size;					// �ļ���Ŀ
	int	Channel;				//0xff��ȫ��ͨ����0��1��2 ......
	int Type;					//0xff �� ȫ����0 �� ��ʱ¼��1 - �ƶ���⣬2 �� ����������3  �� �ֶ�¼��
	COMMO_TIME_T Times[2];		// ��ʼ����ʱ��
	void *	hSearch;			// �������
}PPCGI_MEDIA_SEARCH_T;

typedef struct{
	char Filename[256];			// �ļ���
	COMMO_TIME_T Times[2];		// ��ʼ����ʱ��
	int	Channel;				// ͨ��
	int Size;					// ��С
	int State;					// ת��״̬
	void *	hSearch;			// �������
}PPCGI_MEDIA_INFO_T;

//
// �洢��Ϣ
//
typedef struct{
	int					StorageType;	// 0:HDD 1:USB 2:SD 3:I-SCSI 4:E-SATA
	unsigned long long 	Size;			// ��С
	unsigned long long 	Used;			// ����
	char 				Filesystem[32];	// �ļ�ϵͳ
	char				Status;			// 0: δ���� 1: �ѹ��� 2: ������ 300-399: ��ʽ����,���ڽ��ȷ���
	char				Path[255];		// ����·��
	char				DeviceName[32];	// �豸����
	int					Command;		// ���̿���ָ�� �μ�ö�� CMD_STORAGE_*
}PPCGI_STORAGE_T;

//
// ��Ʒ��Ϣ�������ӿ�
//
typedef struct{
	char 				ServerStr[1024];	// P2P �������ִ�
	char 				ID[32];				// P2P ID
	char 				Model[128];			// ��Ʒ�ͺ�
	char 				Product[128];		// ��Ʒ����
	char 				Version[128];		// ϵͳ�汾
	char *				UpgradeData;		// ������������
	int					UpgradeDataLens;	// ����������
	int					UpgradeArea;		// ����������
	volatile int 		UpgradeStatus; 		// ��������
}PPCGI_PRODUCT_T;

//
// �û���Ϣ
//
typedef struct{
	int					UserID;				// �û���ʶ��
	char				User[32];			// �û���
	char				Pass[32];			// ����
}PPCGI_USER_T;

typedef struct{
	unsigned long long 	CPUFrequency;		// CPU ��Ƶ
	int					RamSize;			// �ڴ��С
	int					MaxVideoEncoder;	// ��Ƶ����������
	int					MaxVoiceEncoder;	// ��Ƶ����������
	int *				VideoBufferSizes;	// ��Ƶ��·�������ο�ֵ
	int *				VoiceBufferSizes;	// ��Ƶ��·�������ο�ֵ
}PPDEV_BROAD_T;

typedef struct{
	int 			MaxChannels;
	int				Handle;
}PPDEV_VIDEO_T;

typedef struct{
	int				Handle;
}PPDEV_AUDIO_T;

typedef struct{
	int 			Handle;
}PPDEV_VOICE_T;

typedef struct{
	char	 		File[256];
	COMMO_TIME_T	Time;
	int				Handle;
}PPDEV_MEDIA_T;

//
// �豸���ƽӿ�
// ��ƽ̨�������غ���,���ϲ�Ӧ���߼�����.
// û��ʵ�ֵĺ���,�뷵��:[-1].
//

//
// �豸��ʼ��
//
int PPDevInit(
	void *	Data,
	int		DataLens
);

//
// �ر��豸
//
int PPDevFree(
	void *	Data,
	int		DataLens
);

//
// �豸 GPIO ��ʼ��
//
int PPDevGPIOsInit(
	void *	Data,
	int 	DataLens
);

//
// �豸 GPIO ����, ���ϲ�ѭ������
//
int PPDevGPIOsCall(
	void *	Data,
	int 	DataLens
);

//
// �豸 GPIO �ͷ�
//
int PPDevGPIOsFree(
	void *	Data,
	int 	DataLens
);

//
// ý���ļ�����ʼ��
//
int PPDevMediaInit(
	void *	Data,
	int		DataLens
);

//
// ��Ƶ����ʼ��
//
int PPDevVideoInit(
	void *	Data,
	int		DataLens
);

//
// ��Ƶ����ʼ��
//
int PPDevAudioInit(
	void *	Data,
	int		DataLens
);

//
// ��������ʼ��
//
int PPDevVoiceInit(
	void *	Data,
	int		DataLens
);

//
// ��Ƶ���ͷ�
//
int PPDevVideoFree(
	void *	Data,
	int		DataLens
);

//
// ��Ƶ���ͷ�
//
int PPDevAudioFree(
	void *	Data,
	int		DataLens
);

//
// �������ͷ�
//
int PPDevVoiceFree(
	void *	Data,
	int		DataLens
);

//
// ý���ļ����ͷ�
//
int PPDevMediaFree(
	void *	Data,
	int		DataLens
);

//
// ͨ���ļ���������Ƶ֡
//
int PPDevGetMediaFrame(
	int		MediaHandle,		// ���
	char *	MediaBuffer,		// ������
	int		MediaBufferSize,	// ��������С
	int  *	MediaBufferType		// ����
);

//
// ͨ���豸������Ƶ֡
//
int PPDevGetVideoFrame(
	int		VideoHandle,		// ���
	char **	VideoBuffer,		// ������
	int		VideoBufferSize,	// ��������С
	int	 *	VideoBufferType,	// ͼ������
	int  	Channel,			// ͨ��
	int	 	Refresh				// ˢ�»���
);

//
// ͨ���豸������Ƶ֡
//
int PPDevGetVoiceFrame(
	int 	VoiceHandle,		// ���
	char *	VoiceBuffer,		// ������
	int		VoiceBufferSize,	// ��������С
	int	 *	VoiceBufferType,	// ��Ƶ����
	int  	Channel,			// ͨ��
	int	 	Refresh				// ˢ�»���
);

//
// ͨ���豸������Ƶ֡
//
int PPDevPutAudioFrame(
	char *	AudioBuffer,		// ������
	int		AudioBufferLens,	// ��������С
	int	 *	AudioBufferType		// ��Ƶ����
);

//
// ��ȡ�豸 GPIO ��״̬
//
int PPDevGetGPIO(
	int		Pin
);

//
// �����豸 GPIO ��״̬
//
int PPDevSetGPIO(
	int 	Pin,
	int		Val
);

//
//	���߶���
//
int PPDevSetStandby(
	void *	Data,
	int		DataLens
);

//
//	���Ѷ���
//
int PPDevGetStandby(
	void *	Data,
	int		DataLens
);

//
//	����ϵͳ
//
int PPDevReboot(
	void *	Data,
	int		DataLens
);

//
// P2P CGI ָ��ӿ�
// ��ƽ̨�������غ���,���ϲ�Ӧ���߼�����.
// û��ʵ�ֵĺ���,�뷵��:[-1].
//

// CGI ����ģ���ʼ��
int PPCgiModuleInit();

// CGI ����ģ������
int PPCgiModuleFree();

// ���������û�ȡ
int PPCgiSetAlias(const char * Alias);
int PPCgiGetAlias(char * Alias,int Lens);

// ��Ƶ����
int PPCgiSetAudio(PPCGI_AUDIO_T * AudioParamSet);
int PPCgiGetAudio(PPCGI_AUDIO_T * AudioParamGet);
int PPCgiSetAudioDefault();

// ͼ�����
int PPCgiSetImage(PPCGI_IMAGE_T * ImageParamSet);
int PPCgiGetImage(PPCGI_IMAGE_T * ImageParamGet);
int PPCgiSetImageDefault();

// ��Ƶ����
int PPCgiSetVideo(PPCGI_VIDEO_T * VideoParamSet);
int PPCgiGetVideo(PPCGI_VIDEO_T * VideoParamGet);
int PPCgiSetVideoDefault();

// OSD ����
int PPCgiSetOSD(PPCGI_OSD_T * OSDParamSet);
int PPCgiGetOSD(PPCGI_OSD_T * OSDParamGet);

// ¼�����
int PPCgiSetRecord(PPCGI_RECORD_T * RecordParamSet);
int PPCgiGetRecord(PPCGI_RECORD_T * RecordParamGet);

// ʱ������
int PPCgiSetTime(PPCGI_TIME_T * TimeParamSet);
int PPCgiGetTime(PPCGI_TIME_T * TimeParamGet);

// ��������
int PPCgiGetAlarm(PPCGI_ALARM_T * AlarmParamGet);
int PPCgiSetAlarm(PPCGI_ALARM_T * AlarmParamSet);

// �ʼ�����
int PPCgiGetMail(PPCGI_MAIL_T * MailParamGet);
int PPCgiSetMail(PPCGI_MAIL_T * MailParamSet);

// ��������
int PPCgiGetNetwork(PPCGI_NETWORK_T * NetworkParamGet);
int PPCgiSetNetwork(PPCGI_NETWORK_T * NetworkParamSet);

// ��������
int PPCgiSetWIFI(PPCGI_WIFI_T * WIFIParamSet);
int PPCgiGetWIFI(PPCGI_WIFI_T * WIFIParamGet);
int PPCgiLstWIFI(PPCGI_WIFI_SCAN_T * WIFIScanParamLst);

// FTP  �ϴ�����
int PPCgiSetFTPClient(PPCGI_FTPC_T * FTPClientParamSet);
int PPCgiGetFTPClient(PPCGI_FTPC_T * FTPClientParamGet);

// CIFS �ϴ�����
int PPCgiSetCIFS(PPCGI_CIFS_T * CIFSParamSet);
int PPCgiGetCIFS(PPCGI_CIFS_T * CIFSParamGet);

// HTTP �˿�����
int PPCgiGetHTTPPort();
int PPCgiSetHTTPPort(int HTTPPort);

// �洢����
int PPCgiSetStorage(PPCGI_STORAGE_T * StorageParamSet);
int PPCgiGetStorage(PPCGI_STORAGE_T * StorageParamGet);

// ý���ļ�����
int PPCgiLstMedia(PPCGI_MEDIA_INFO_T * MediaInfoLst);			// ��ȡ�����Ľ��
int PPCgiPutMediaSearch(PPCGI_MEDIA_SEARCH_T * MeidaSearchPut);	// �رռ�������
int PPCgiGetMediaSearch(PPCGI_MEDIA_SEARCH_T * MeidaSearchGet);	// �򿪲����ü�������

// ��Ʒ��Ϣ����
int PPCgiSetProduct(PPCGI_PRODUCT_T * ProductParamSet);
int PPCgiGetProduct(PPCGI_PRODUCT_T * ProductParamGet);

// �ָ���������
int PPCgiSetDefault();

// �û�����
int PPCgiGetUser(PPCGI_USER_T * UserListGet,int * ListSize); // ��ȡ�û�
int PPCgiAddUser(PPCGI_USER_T * UserSet); // �����û�
int PPCgiDelUser(PPCGI_USER_T * UserSet); // ɾ���û�
int PPCgiSetUser(PPCGI_USER_T * UserSet); // �����û�

int PPCgiSetAnonymousEnable(int Value); // ���������û����ÿ���
int PPCgiGetAnonymousEnable(); 			// ��ȡ�����û����ÿ���

// PTZ ����
int PPCgiSetPTZMove(int Direct,int Step);

// ��ȡ��̨Ѳ������
int PPCgiGetPTZRunTimes();

// ������̨Ѳ������
int PPCgiSetPTZRunTimes(int Value);

// ��ȡ��̨�˶��ٶ�
// ���� 0:�� 1:�� 2:�� 3:��	4:�ֶ�
int PPCgiGetPTZSpeed(int Direct );

// ������̨�˶��ٶ�
int PPCgiSetPTZSpeed(
	int Direct,			// ���� 0:�� 1:�� 2:�� 3:�� 4:�ֶ�
	int Value			// ֵ
);

//
// �������ⲿ����
//

// ��������
int PPMsgPush(
	int 	PID,		// ƽ̨ ID
	void *	Params,		// ����
	int		Lens		// ��������
	);

// ��ʼ�������߳�
int PPSvrInit(
	const char * ServerID,
	const char * DeviceID,
	int			 DeviceType
);

// ���Ժ���
int PPSvrTest(
	int			 DeviceType
);

// ���ٹ����߳�
int PPSvrDestroy();

//extern struct device_info_mtd device_info;

//macro
#define VALUE_CONVERT(value, min, max) \
	((value) > (max) ? (max) : (value) < (min) ? (min) : (value))

#define TRUE_FALSE_CONVERT(value, vfalse, vtrue) \
	((value) ? (vtrue) : (vfalse))

#define CHANNEL_ID_CONVERT(Channel) \
	((Channel) == 0 ? 1 : (Channel) == 1 ? 2 : (Channel) == 2 ? 0 : 0)

#endif

