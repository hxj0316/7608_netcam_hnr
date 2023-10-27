/********************************************************************************************
  ��Ȩ���� (C), 2001-2016, �Ϻ�����͵��ӿƼ����޹�˾
 *********************************************************************************************/
#ifndef __ANYAN_DEVICE_SDK_H__
#define __ANYAN_DEVICE_SDK_H__

#ifdef WIN32
#define AYAPI_ATTR 
#else
#define AYAPI_ATTR __attribute__((deprecated)) 
#endif

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    typedef char  		int8;
    typedef short 		int16;
    typedef int			int32;
    typedef unsigned char 	uint8;
    typedef unsigned short 	uint16;
    typedef unsigned int  	uint32;
    typedef unsigned long long 	uint64;

#define	 AY_MAX_CHANNEL_NUM            128

/* ʵʱ��֡���Ͷ��� live stream frame type macro */
#define  	CH_I_FRM    		0
#define  	CH_P_FRM    		1
#define  	CH_AUDIO_FRM		2

/* �ط���֡���Ͷ��� history stream frame type macro */
#define  	CH_HIS_I_FRM		3
#define  	CH_HIS_P_FRM		4
#define  	CH_HIS_AUDIO_FRM	5

/* �ϴ���ʹ�õ����ʵȼ�,û�ж�Ӧ���ʿ�������ӽ���һ�� */
#define  	UPLOAD_RATE_1       	384 	
#define  	UPLOAD_RATE_2       	500		
#define  	UPLOAD_RATE_3       	700		
#define  	UPLOAD_RATE_4       	1000		

    typedef struct
    {
	int 	block_nums;						/*���ٵĻ�������������λ��8k*block_nums*/
	uint8 	channel_num;						/*ͨ������*/
	uint16	max_rate;						/*�������*/
	uint16	min_rate;						/*��С����*/
	char	ptz_ctrl;						/*�Ƿ�֧��ptz���� 0 ��֧��,1 ֧���������� 2 ֧���������Һͱ佹,3 ֧���������Һ�Ԥ��λ, 4 ֧���������ҡ��佹��Ԥ��λ */
	char 	dev_type;						/*�豸����0 δ֪ 1 dvr, 2 nvr, 3 ipc */

	char    mic_flag;						/*�Ƿ���ʰ���� 0 û��, 1 ��*/
	char    can_rec_voice;						/*���Խ�����Ƶ 0 ��֧��, 1 ֧��*/
	char	hard_disk;						/*�Ƿ��б��ش洢 0 û��, 1 ��Ӳ�� */
	char	*p_rw_path;						/*�ɶ�д·��,���sn,���ļ���*/

	char	audio_type;						/*��Ƶ���� 0:aac, 1:g711_a, 2:g711_u, 3:mp3*/
	char 	audio_chnl;						/*��Ƶͨ��,��ͨ�� 1, ˫ͨ�� 2 */
	uint16  audio_smaple_rt;					/*��Ƶ������*/
	uint16  audio_bit_width;					/*λ��*/

	uint8   use_type;						/*�豸ʹ�����ͣ�0�����������豸��1�������豸��2����ʾ�豸�� Ĭ��0��*/
	uint8	has_tfcard;						/*�Ƿ���TF�� 0: û�� 1:�� */
	uint8   Reserved[18];						/*Ԥ���ֶ�*/
    }
    Dev_Attribut_Struct;

    typedef struct
    {
	int 		channelnum;					/*ͨ����*/
	uint16  	bit_rate;					/*����*/ 
	uint16  	frm_type;					/*֡���� CH_I_FRM   CH_P_FRM    CH_AUDIO_FRM	*/

	uint32  	frm_id;						/*֡����ˮ�����е� 2015/3/11 ����ֶ���Ч.���账��������*/
	uint32  	frm_av_id;					/*֡������ˮ��i ֡����P֡������Ƶ֡. 2015/3/11 ����ֶ���Ч.���账��������*/

	uint32  	frm_ts;						/*ʱ��� ms*/
	char 		*pdata;						/*����*/
	uint32  	frm_size;					/*֡��С*/
    }
    Stream_Event_Struct;

    typedef struct
    {
	int 		channel_index;					/*ͨ����*/
	uint16  	update_rate;					/*����*/
	uint16  	alarm_type;					/*�������� �μ�:ALARM_TYPE*/
	uint8		picture_type;					/*ͼƬ��ʽ,��δʹ��,ֻ֧��jpg */	
	uint32 		screenshot_ts;					/*ʱ���*/
	uint32  	pic_length;					/*��ͼ����*/
	uint8  		*pic_data;					/*��ͼ����*/
    }
    Screen_Shot_Struct;
    typedef struct
    {
	int 		channelnum;					/*ͨ����,start 1 */
	uint16  	bit_rate;					/*����*/
	uint32  	ts_ts;						/*֡���� ��ʼʱ��� */
	uint32 		offset;						/*ƫ��*/
	uint32 		length;
	uint8   	*ts_data;
	uint32 		ts_size;					/*�����ļ���С*/
	uint32 		ts_duration;					/*����ʱ�䳤��*/  	
    }
    Stream_History_Struct;

    typedef struct
    {
	uint16 	channelnum;						/*ͨ����*/
	uint16  bit_rate;						/*����*/
	uint16  frm_type;						/*֡���� I P */
	uint32  frm_ts;							/*ʱ��� */		
	uint8   *pdata;							/*֡����*/
	uint32  len;							/*֡���ݳ���*/
    }
    Stream_History_Frm_Struct;

    typedef struct 
    {
	uint32 start_time;
	uint32 end_time;
    }
    NVR_history_block;

    typedef struct 
    {
	uint8 channel_index;	
	uint16 rate; 
	uint32 start_time;  /*	��ʼʱ��    */	    
	uint32 end_time;    /*	����ʱ��    */
	uint32 list_len;    
	NVR_history_block hList[128];	
	uint32 seq;
    }
    HistoryListQuery_Struct;

    typedef enum
    {
	PTZ_CTRL_LEFT = 0x01,
	PTZ_CTRL_RIGHT,
	PTZ_CTRL_UP,
	PTZ_CTRL_DOWN,
	PTZ_CTRL_FOCUS_IN,
	PTZ_CTRL_FOCUS_OUT,
	PTZ_CTRL_PTZ_STOP,
	PTZ_CTRL_OPEN_VIDEO,
	PTZ_CTRL_CLOSE_VIDEO,
	PTZ_CTRL_SYS_RESET,
	PTZ_CTRL_SYS_REBOOT,
	PTZ_CTRL_MIRROR,
	PTZ_CTRL_FLIP,
    }PTZ_CTRL_ENUM;

    typedef  enum
    {
	/* ��Ƶ���� 
	 * args: [0] - enable flag, [1]~[2] - rate 
	 */
	VIDEO_CTRL,

	/* ��ʷ�ط�
	 * args: [0]~[1] - rate, [2]~[5] - start time(UTC time), [6]~[9] - duration seconds 
	 */
	HISTORY_CTRL,

	/* ��Ƶ����
	 * args: [0] - enable flag, [1]~[2] - rate 
	 */
	AUDIO_CTRL,

	/* �Ի�����: Not Finished */
	TALKING_CTRL,

	/* ץ�Ŀ���: Not Finished */
	CAPTURE_PICTURE_CTRL,

	/* ��̨���� 
	 * args: [0] - stop(0)/start(1), [1] - see @PTZ_CTRL_ENUM define, [2]~[5] - ptz steps
	 */
	PTZ_CTRL,

	/* ����Ԥ�õ� 
	 * args: [0]~[1] - preset position
	 */
	PTZ_SET_PRE_LOCAL,

	/* ����Ԥ�õ�
	 * args: [0]~[1] - preset position
	 */
	PTZ_CALL_PRE_LOCAL,

	/* ���ÿ���λ: Not Finished */
	PTZ_SET_WATCH_LOCAL,

	/* ����Ѳ��: Not Finished */
	PTZ_SET_CRUISE,

	/* �ֶ�,����λ,Ѳ��: Not Finished */
	PTZ_SET_MODE,

	/* �������� alarm control
	 * args: AY_Alarm_Struct data(memcpy)
	 * args_len = sizeof(AY_Alarm_Struct)
	 */
	ALARM_CTRL,

	/* ��ȡ�豸����: Not Finished */
	DEVICE_PARAM_GET,

	/* �����豸����: Not Finished */
	DEVICE_PARAM_SET,

	/* Уʱ NTP Time sync
	 * args: [0]~[3] - UTC Timestamp to reference,need to adjust time by timezone. 
	 */
	TIME_SYN,

	/* ������Ϣ
	 * args: [0]~[N] - info string 
	 */
	ERROR_INFO,

	/* ͸������
	 * args: [0]~[N] - user private data
	 */
	USER_DATA,

	/* WIFI����: Not Finished */
	WIFI_PASSWORD,

	/* ֹͣ�ط�
	 * args: None 
	 */
	HISTORY_STOP,

	/* �طŲ�ѯ
	 * args: [0]~[1] - rate, [2]~[5] - start time(UTC time), [6]~[9] - end time 
	 */
	HISTORY_LIST,

	/* SD������
	 * args: [0] - action: 0 format sd card,1 query sd status 
	 */
	SD_CARD_CTRL,

	/* ¼�����
	 * args: [0] - action: 0 set record mode, 1 stop record, 2 start record; [1] - record mode value(when set)
	 */
	RECORD_CTRL,

	/* ���Ԥ��λ
	 * args: [0]~[1] - preset position
	 */
	PTZ_CLEAR_PRESET,
 
	/* ��ѯ�ƶ�������ж� query motion detect sensitivity
	 * args: None 
	 */
	EXT_MDSENSE_QUERY,
	
	/* �����ƶ�������ж� set motion detect sensitivity
	 * args: [0] - switch flag 0 or 1, [1] - sensitivity level value 10 - 30(low/medium/high)
	 */
	EXT_MDSENSE_SET,

	/* ��ѯ����������ж� query audio detect sensivity
	 * args: None
	 */
	EXT_ADSENSE_QUERY,

	/* ��������������ж� set audio detect sensivity
	 * args: [0] - switch flag 0 or 1, [1] - sensitivity level value 10 - 30(low/medium/high)
	 */
	EXT_ADSENSE_SET,

	/* ��ѯ����¼ query alarm detect record 
	 * args: [0]~[3] - start time(UTC time), [4]~[7] - end time 
	 */
	EXT_DETECT_RECORD_QUERY,

	/* ��ѯsd��״̬ query sd card status information
	 * args: None
	 */
	EXT_SDCARD_QUERY,

	/* ����sd���� control sd card
	 * args: [0]~[1] - action cmd 1000: format sdcard, 1001: set record flag
	 *	 [2] - when cmd is 1001, record flag 0:disable 1:enable normal record 2: enable alarm record
	 */
	EXT_SDCARD_CONTROL,

	/* ��ѯ�豸Ԥ��λ query device preset position
	 * args: None
	 */
	EXT_PRESET_QUERY,

	/* ����Ԥ��λ���� set device preset position
	 * args:[0] - action subcmd,1:set 2: delete 3:execute, [1] - preset index 
	 * [2]~[5] - horizontal coordinates,[6]~[9] - vertical coordinates, [10]~[11] - lens zoom multiples
	 */
	EXT_PRESET_SET,

	/* ��ѯ��Ƶ��ת query device video flip/mirror parameters
	 * args: None.
	 */
	EXT_IMAGE_QUERY_TURN,

	/* ������Ƶ��ת set device image flip/mirror
	 * args: [0] - horizontal turn(mirror) cmd 0:close 1:turn 180 degree, [1] - vertical turn(flip) cmd,0:close 1:turn 180 degree
	 */
	EXT_IMAGE_SET_TURN,

	/* �����豸 reboot device
	 * args:[0] - subcmd, 1:reboot
	 */
	EXT_REBOOT_DEVICE,

	/* �ָ��������� restore factory settings
	 * args��[0] - subcmd, 1: reset
	 */
	EXT_RESET_DEVICE,

	/* �����豸 manage device 
	 */
	EXT_MANAGE_DEVICE,
    }CMD_ITEMS;

    typedef struct
    {
	int   		channel;					/*ͨ��*/
	CMD_ITEMS	cmd_id;						/*����id�ο�CMD_ITEMS*/
	char		cmd_args[1024];					/*����Я������������ע��*/
	int		cmd_args_len;					/*���ݳ���*/
    }
    CMD_PARAM_STRUCT;

    typedef  enum
    {
	MOVE_DETECT = 0x0004,						/*�ƶ����*/
	VIDEO_LOST = 0x0008,						/*��Ƶ��ʧ*/
	VIDEO_SHELTER = 0x0010,						/*��Ƶ�ڵ�*/		
	PRIVACY_NOTIYF = 0x0020,					/*��˽����*/
	AUDIO_DETECT = 0x0040,						/*�������*/
	//TBD
    }ALARM_TYPE;
    typedef  enum
    {
	JPG = 0x0001,					
	JPEG = 0x0002,
	BMP = 0x0003,								
	//TBD
    }PICTURE_TYPE;
    typedef  enum
    {
	ULK_NO_END = 0,
	ULK_END = 1,
	ULK_HIST_NORMAL = 2,
	ULK_HIST_CONTINUE = 3,
	//TBD
    }ULK_HISTORY_VIDEO_ENUM;

    typedef  enum
    {
	ULK_OFFLINE = 0,
	ULK_ONLINE = 1,
	//TBD
    }ULK_CHNL_STATUS_ENUM;

    typedef enum
    {
	ULK_AUDIO_TYPE_AAC = 0,
	ULK_AUDIO_TYPE_G711A,
	ULK_AUDIO_TYPE_G711U,
    }ULK_AUDIO_TYPE_ENUM;

    typedef enum 
    {
	AYE_VAL_SD_NOOP = 0,
	AYE_VAL_SD_FORMAT,
	AYE_VAL_SD_NEXSIT, 
	AYE_VAL_SD_OK, 
	AYE_VAL_SD_BAD,
    }AYE_TYPE_SD_STATUS;

    typedef enum
    {
	AYE_VAL_RECMODE_NOOP = 0,
	AYE_VAL_RECMODE_ALARM,
	AYE_VAL_RECMODE_NORMAL,
	AYE_VAL_RECMODE_VOICE,
	AYE_VAL_RECMODE_PLAN,
    }AYE_TYPE_RECORD_MODE;

    typedef struct preset_pos_info
    {
	int index; /* start from 0 */
	int x; /* horizontal coordinates */
	int y; /* vertical coordinates */
	int z; /* lens zoom multiples */
    }AY_PRESET_POS;

    typedef struct
    {
	char     MAC[17 + 1];				/*MAC ��ַ*/
	uint32   OEMID;					/*OEM ID*/
	char     SN[16+1];				/*sn���к�,�豸�Լ���֤Ψһ.*/
	char     OEM_name[2+1];				/*����OEM����,������ݲ�ͬ�����ɰ���ƽ̨ͳһ�ṩ*/
	char     Model[64 + 1];				/*�豸�ͺ�*/
	char     Factory[255 + 1];			/*��������(���磬����͡��������ӡ��󻪵�)*/
    } Dev_SN_Info;
    typedef struct 
    {
	char    audio_type;				/*��Ƶ���� ULK_AUDIO_TYPE_ENUM,Ŀǰֻ֧��aac*/
	char    is_end;					/* 0 û��, 1 ���� */
	uint32  audio_seq;				/*��ˮ��ÿ����Ϣ����һ����������� */
	char    *audio_buf;
	int	audio_len;
    }ULK_Audio_Struct;
    typedef struct
    {
	uint8		channel_index;			//ͨ��
	uint8		brightness;			//����
	uint8		saturation;			//���Ͷ�
	uint8		Contrast;			//�Աȶ�
	uint8		tone;				//ɫ��
    }ULK_Video_Param_Ack;				//ͼ������ϴ�
    typedef struct
    {
	uint8		channel_index;			//ͨ��
	uint8		rate_index;			//�������� 0 �� 1 ��
	uint16		bit_rate;			//����
	uint16		frame_rate;			//֡��
	uint16		video_quality;			//ͼ������ >0 ԽСԽ��
    }ULK_Video_Encode_Param_Ack;// ͼ���������ϴ�

    typedef struct
    {
	uint32  start; // the seconds from 0:0:0 in a day,scope: [0 ~ 86400],so the value of 0:0:0T would be 0.
	uint32  end; //  the seconds from 0:0:0 in a day,scope: [0 ~ 86400], so the value of 1:0:0T would be 3600. 
    }ULK_Alarm_Clock_struct;
    typedef struct
    {
	uint8			alarm_type;// 0 - motion alarm, 1 - audio alarm
	char   			alarm_flag;// 1 ��������  0 �رձ���
	uint8			alarm_sensitivity;// 0 - low, 1 - medium, 2 - high
	uint8			alarm_time_table_num;
	uint8			alarm_chn_mask[AY_MAX_CHANNEL_NUM]; /* 0 - enable, 1 - filter, default all enable */
	uint32 			alarm_interval_mix;//��������
	ULK_Alarm_Clock_struct  alarm_time_table[5];//ʱ���
    }ULK_Alarm_Struct;
    typedef struct
    {
	uint8			alarm_flag;// alarm push switch: 0 - disable, 1 - enable
	uint8                   alarm_time_table_num; // the count of alarm time table setttings,max is 5.
	uint32                  alarm_interval_mix;//alarm push period, unit is second.
	ULK_Alarm_Clock_struct  alarm_time_table[5];//alarm time table settings, see @ULK_Alarm_Clock_struct. 
    }AY_Alarm_Struct;

    typedef enum
    {
	Ulk802_11AuthModeOpen	   = 0x00,
	Ulk802_11AuthModeWEPOPEN  = 0x01,
	Ulk802_11AuthModeWEPSHARE = 0x02,

	Ulk802_11AuthModeWPAEAP = 0x03,
	Ulk802_11AuthModeWPA2EAP = 0x04,
	Ulk802_11AuthModeWPA1EAPWPA2EAP = 0x05,

	Ulk802_11AuthModeWPAPSKAES = 0x06,
	Ulk802_11AuthModeWPAPSKTKIP = 0x07,
	Ulk802_11AuthModeWPAPSKTKIPAES = 0x08,

	Ulk802_11AuthModeWPA2PSKAES = 0x09,
	Ulk802_11AuthModeWPA2PSKTKIP = 10,
	Ulk802_11AuthModeWPA2PSKTKIPAES = 11,

	Ulk802_11AuthModeWPA1PSKWPA2PSKAES = 12,
	Ulk802_11AuthModeWPA1PSKWPA2PSKTKIP = 13,
	Ulk802_11AuthModeWPA1PSKWPA2PSKTKIPAES = 14,

	//Ulk802_11AuthModeMax,
	Ulk802_11AuthModeAuto,
    }ULK_ENTICATION_MODE;

    typedef struct
    {
	PICTURE_TYPE  pic_type;
	uint8  *pic_data;
	uint32 pic_data_len;
    }Picture_info;

    typedef enum
    {
	SDCARD_NONE,
	SDCARD_NORMAL,
	SDCARD_FORMATTING,
	SDCARD_BAD,
    }SDcard_status;

    /*
     * @brief ��ʼ��SDK����
     * @param[in] oem_info �豸���̵�OEM��Ϣ,��������
     * @param[in] devattr  �豸��Ʒ�Ļ�����������,��������
     * @param[in] logfile  SDK��־��¼�ļ��ľ���·��,���������,����Ϊ NULL.
     * @ret 0 - success, -1 - fail
     * @note �����ӿ�,Ҫ��SDK�汾>=1.5.0.�����滻ԭ�е������ӿڵ���,�����������˳�����������:
     *   Ulu_SDK_Enable_Debug + Ulu_SDK_Set_OEM_Info + Ulu_SDK_Init
     */
    int Ulu_SDK_Init_All(Dev_SN_Info *oem_info,Dev_Attribut_Struct *devattr,const char *logfile);

    /*
     * �ͷ�֡���ٻ�����,����ʱ���ô˺���
     */
    void Ulu_SDK_DeInit(void);

    /*
     * ����ͼ���С
     */
    void 	Ulu_SDK_Set_Video_size(int width, int height);

    /*
     * ���ûص���������
     */
    typedef   	void(*Interact_CallBack)(CMD_PARAM_STRUCT *args);
    void 	Ulu_SDK_Set_Interact_CallBack(Interact_CallBack callback_fun);

    /* ������Ƶ�Խ��ص����� */
    typedef   	void(*Audio_AAC_CallBack)(ULK_Audio_Struct *args);
    void 	Ulu_SDK_Set_AudioAAC_CallBack(Audio_AAC_CallBack callback_fun);

    /*
     * �ú�����Ϊ�˱������溯��������������������ӵģ�Ϊ���ܹ�֧�ֶ��ֶԽ�����Ƶ���ݸ�ʽ���޸�
     * �������õĻص�����һ���������õ���ͬһ��,���໥����,���Իص��������ߵĺ�һ�ε�����Ч
     *
     * ����ʹ�øú������ @Ulu_SDK_Set_AudioAAC_CallBack ����.
     */
    typedef   	void(*Audio_Talk_CallBack)(ULK_Audio_Struct *args);
    void 	Ulu_SDK_Set_Talk_CallBack(Audio_Talk_CallBack callback_fun);

    /*
     * ���ûص���������
     * @ret  -1;����ʧ�� 0;���� -2;ָ��Ϊ��
     */
    typedef   	void(*WIFI_CallBack)(char  *ssid, char *password, ULK_ENTICATION_MODE encrypt_type);	
    int 	Ulu_SDK_Set_WIFI_CallBack(WIFI_CallBack callback_fun);

    /*
     * ���ûص���������
     *
     * @param request_encrypt 	0 ����Ҫ,1 ��Ҫ
     * @param wireless_lan_name �������� �� ra0 wlan0
     * @ret -1;����ʧ�� 0;���� -2;ָ��Ϊ��
     */
    int	     Ulu_SDK_Set_WIFI_CallBack_Ext(WIFI_CallBack callback_fun, char request_encrypt, char *wireless_lan_name);

#ifndef WIN32
    /*
     * ��������ģʽ
     *
     * @ret 0 ����; -1  û�����ûص��ӿ� ;-2 �޷��ҵ���Ӧ��wifi����
     */
    int      Ulu_SDK_Start_Wifi_Monitor(void);

    /*
     * ֹͣ����ģʽ
     * 
     * @note �ú���������ִ�У������120seconds��������WIFI_CallBack�ص������е��øú���!!!!!
     */
    void     Ulu_SDK_Stop_Wifi_Monitor(void);

    /*
     * ͨ������ͼƬ������ά���ȡwifi�� 
     * ע��:pushͼƬ����,ǰ������wifi�ص�,ͼƬ��ʽ֧��jpg 
     * @param ������Ҫ���н����İ�����ά���ץ��ͼƬ
     * @note  ��ά�������Ҫʹ��֧�ָù��ܵİ汾��, �����������-1��ʾ�汾�ⲻ֧�ֶ�ά�빦��(�򻯰�) 
     */
    int      Ulu_SDK_Push_ScreenShot(Picture_info *pic_info);
#endif

    /*
       �����û����� ��󳤶Ȳ��ܳ���1024byte
       ����ֵ:	-1  ���ȳ���1024 ; 0 ����
    */
    int      Ulu_SDK_Send_Userdata(uint8 *pdata, uint16 len);

    /* ��Ƶ�������ϱ�,����֡�ϱ�..I֡,P֡,��Ƶ֡��...
     *
     * @ret -4:�������쳣  -3:ͨ����Χ���� -2:ͨ�������� -1:��ʾû�н����������� 
     * -6: ֡���Ͳ���
     * -7: ��һ֡Ӧ����I֡,�ᶪ����֡
     * -8: �������ʽ���������I֡(���Ǵ���),ֻ�Ƕ�����I֡,���ȱ�֤I֡���� 
     *  > 0 ������ʾ������ʹ������
     */
    int 	Ulu_SDK_Stream_Event_Report(Stream_Event_Struct *pEvent);

    /*  ץ��ͼƬ�ϴ�	
	@ret  -5:ͨ����Χ����  -4:û������  -3:����̫�� -2:ʱ�䷶Χ������ -1:��������ֹ����.0 ����
	@note ͼƬ����ܳ���210k
    */
    int 	Ulu_SDK_Screen_Shot_Upload(Screen_Shot_Struct *ppic);

    /*	�����ϴ���չ�ӿ�
     *	@ret -5:ͨ����Χ����  -4:û������  -3:����̫�� -2:ʱ�䷶Χ������ -1:��������ֹ����.0 ����
     */
    int 	Ulu_SDK_Alarm_Upload_Ex(int channel, ALARM_TYPE  alarm_type);

    /*	�ϱ�ͨ��״̬
	-1 ͨ���Ŵ��� 0 ����
    */
    int 	Ulu_SDK_ChannelStatus_Event(int channel, ULK_CHNL_STATUS_ENUM status);

    /*
       ��ȡ����ϴ������ٶ�
       send_speed 	�������ݵ��ٶ�
       send_ok_speed  ʵ�ʷ��ͳɹ����ٶ�
       ע:��������4s�ڵ�����ͳ��byte
    */
    void	Ulu_SDK_Get_Upload_Speed(uint32 *send_speed, uint32 *send_ok_speed);	

    /*
       ��ȡ������������״̬
       1 -> ��������, -2 �������� -1 û������
    */
    int  	Ulu_SDK_Get_Connect_Status(void);

    /*
       ��ȡ�豸ID��Ϣ,����豸�ǵ�һ��ע��,��Ҫ�ȴ���ʱ5s,�ȴ��豸ע������id,
       ������ܻ᷵һ����Ч��id
    */
    char*	Ulu_SDK_Get_Device_ID(void);

    /*
       ���ͱ���¼��֡����
       ���� -4:�������쳣  -3:ͨ����Χ���� -2:ͨ�������� -1:��ʾû�н�����������.> 0 ������ʾ������ʹ������
    */
    int		Ulu_SDK_History_Frame_Send(Stream_History_Frm_Struct *pfrm_data, ULK_HISTORY_VIDEO_ENUM is_end);

    /*
     * ��ʷ��Ƶ��ѯ���Ϊ��ʱ֪ͨ���ͻ���
     */
    int 	Ulu_SDK_History_Srch_Rslt_None(void);

    /*
     * ��ȡ�豸����״̬
     *
     * @ret 1 - Device is online, 0 - offline, -1 - regist has fail
     * @note require sdk version >= 1.5.0.
     */
    int  Ulu_SDK_Get_Device_Online(void);

    /*
     * ��λ��������������
     *
     * @ret None.
     * @note require sdk version >= 1.5.0.
     */
    void Ulu_SDK_Reset_Stream_Links(void);


    /*
     * ������ʷ��Ƶʱ���б�
     *
     * @ret 0 - success
     * @note require sdk version >= 1.5.0.
     */
    int  Ulu_SDK_History_List_Send(HistoryListQuery_Struct *pEvent);

    /*
     * �����豸��Ƶ��RTSP�ۿ���ַ
     *
     * @param[in] url rtsp video url.
     * @ret None.
     * @note require sdk version >= 1.5.0.
     */
    void Ulu_SDK_Set_Rtsp_Url(const char *url);

    /*
     * ��ȡSDK�ķ����汾��
     *
     * @ret return release version id, for vA.B.C, id = (A<<16)|(B<<8)|C.
     * @note require sdk version >= 1.5.0.
     */
    int Ulu_SDK_Get_Version(void);

    /*
     *	��ȡ�豸�������Ʋ���
     *  @param[in] type 0 - motion alarm, 1 - audio alarm
     *  @param[in,out] pctrl return the alarm ctrl parameters if found the alarm type
     *  @ret return 0 when found, or return -1.
     */
    int	Ulu_SDK_Get_Alarm_Ctrl(uint8 type,ULK_Alarm_Struct *pctrl);

    /*
     * ��λ����豸 reset to unbind device
     *
     * @ret 0 - success, -1 - param error,-2 - network error,-3 - decode error, -4 - respnse error,-5 - code error
     */
    int Ulu_SDK_Reset_Device(void);

    /*
     * �ϱ��豸TF��״̬��¼��ģʽ
     *
     * @param[in] status device TF card status,see @AYE_TYPE_SD_STATUS define
     * @param[in] mode device record mode value,see @AYE_TYPE_RECORD_MODE define
     * @ret 0 - success, -1 - when stream link isnot connected.
     */
    int Ulu_SDK_Device_Record_Report(AYE_TYPE_SD_STATUS status,AYE_TYPE_RECORD_MODE mode);

    /*
     * ����Ԥ��λ����� response query preset position result
     * @param[in] preset[] all preset position result
     * @param[in] num count of preset position result
     */
    int Ulu_SDK_EXT_Response_All_Preset(AY_PRESET_POS preset[],int num);

    /*
     * ����Ԥ��λ������� response control preset result 
     * @param[in] index preset id,start from 0
     * @param[in] result control preset result,0 - fail, 1 - success
     */
    int Ulu_SDK_EXT_Response_Preset_Result(int index,int result);

    /*
     * ������Ƶ��ת��Ϣ response video turn parameters
     * @param[in] hturn horizontal turn value, 0: close 1: turn 180 degree
     * @param[in] vturn vertival turn value, 0: close 1: turn 180 degree
     * @ret 0 - success, -1 - fail
     */
    int Ulu_SDK_EXT_Response_Image_Turn(int hturn, int vturn);

    /*
     * �����ƶ�������ж� response motion detect sensitivity
     * @param[in] value  sensitivity value,10 - low 20 - medium 30 - high
     * @param[in] enable 0 - disable,1 - enable
     * @ret 0 - ok, -1 - fail 
     */
    int Ulu_SDK_EXT_Response_Mdsense(int value,int enable);

    /*
     * ��������������ж� response audio detect sensitivity
     * @param[in] value  sensitivity value,10 - low 20 - medium 30 - high
     * @param[in] enable 0 - disable,1 - enable
     * @ret 0 - ok, -1 - fail 
     */
    int Ulu_SDK_EXT_Response_Adsense(int value,int enable);

    /*
     * ������˽����֪ͨ response private alarm notification
     * @param[in] msg private alarm message content,max length < 64
     * @ret 0 - ok, -1 = fail
     */
    int Ulu_SDK_EXT_Response_Private_Alarm(const char *msg);

    /*
     * ��������¼ response alarm detect record
     * @param[in] start alarm record start time
     * @param[in] end alarm record end time
     * @param[in] bitmap alarm record bitmap, one bit for one minute
     */
    int Ulu_SDK_EXT_Response_Detect_Record(uint32 start,uint32 end,int bitmap);

    /* 
     * ����SD��״̬��� response sdcard status information
     * @param[in] status sdcard status
     * @param[in] record record enable flag, 0 - disable, 1 - enable normal record, 2 - enable alarm record
     * @param[in] total_v sdcard total volume,unit is MB
     * @param[in] free_v sdcard free volume,unit is MB
     */
    int Ulu_SDK_EXT_Response_Sdcard_Result(SDcard_status status,int record,int total_v,int free_v);

    /*
     * ���������SDK��־��¼����,����ͨ������ӿ�����ȡʵʱ��־���ݽ��д���
     * @ret the length of log content, or 0 if disable log or empty.
     */
    int Ulu_SDK_Get_Log(char log[],int size);

    /*	
     *	��ȡϵͳʱ���.���뵥λ
     */
    uint64 Ulu_SDK_GetTickCount(void);

#ifndef WIN32
    ///////////////////////////////// Deprecated APIs //////////////////////////////////////////////////////////
    /*	
     *	�����Զ�ʱ��ͬ��.Ĭ������
     *	�·��ص����TIME_SYN, �ϲ㴦��
     */
    void AYAPI_ATTR Ulu_SDK_Enable_AutoSyncTime(void);

    /*	
     *	��ֹ�Զ�ʱ��ͬ��
     *	ȡ���ص�����, ���Բ���Ҫ�ýӿ�,ֱ�Ӻ��Ե��ص�����TIME_SYN�Ϳ���.
     */
    void AYAPI_ATTR Ulu_SDK_Disable_AutoSyncTime(void);

    /*	�����ϴ�: ͨ��1�ı����ϴ�
     *	@note: ���鲻��ʹ�øýӿڣ�ʹ��Ulu_SDK_Alarm_Upload_Ex�ӿ����.
     */
    void AYAPI_ATTR Ulu_SDK_Alarm_Upload(ALARM_TYPE  alarm_type);

    /*
     * NVR��ʼ��:�ú����Ѿ�������Ҫ
     * �ú����Ѿ������κι���������Ϊ�˼��ݽӿڶ�����
     */
    int	AYAPI_ATTR Ulu_SDK_NVR_Init(int  blocknums, int  ts_mem_size);

    /*	ͼ������ϴ� ��δʵ��
    */
    void AYAPI_ATTR Ulu_SDK_Param_Vedio_Upload(ULK_Video_Param_Ack  *video_param);

    /*
       ͼ���������ϴ� ��δʵ��	
    */
    void AYAPI_ATTR Ulu_SDK_Code_Param_Vedio_Upload(ULK_Video_Encode_Param_Ack  *video_param);	

    /*
     * ���Ź���ʼ��,�����ⲿ���̼��������Ƿ�������,����Ӧ��ʵ���Լ��Ŀ��Ź�����
     *
     * @note call this API in the watchdog process,another process.
     */
    int AYAPI_ATTR Ulu_SDK_Watchdog_init(void);

    /*
     * ��ȡ�����豸SDK����״̬ get anyan device sdk work status in the watchdog process
     *
     * @ret 1 - anyan sdk works ok, 0 - anyan sdk has some exception, -1 - watchdog init fail.
     * @note 
     *	1. call this API in the watchdog process,another process.
     *	2. ��̼������ > 15s ,<15s û������.������ֱ�ӷ���1
     */
    int AYAPI_ATTR Ulu_SDK_Get_ulu_net_status(void);

    /*
     *  ��������ģʽ �����ô˺�����������ģʽ��ͬʱ������־�ļ�����������־�ļ��������ļ���������:/tmp/ulk.log
     *  log_file_full_name:��־�ļ�·��+�ļ���
     *  ����ʹ�ã�Ulu_SDK_Init_All(...) ����, 20150910
     */
    void AYAPI_ATTR Ulu_SDK_Enable_Debug(const char *log_file_full_name);

    /*
       ����OEM Info,�����ڴ������Լ��ĳ��������豸�ͺš�����ID������������Ϣ
       ����ʹ�ã�Ulu_SDK_Init_All(...) ����, 20150910
    */
    void AYAPI_ATTR Ulu_SDK_Set_OEM_Info(Dev_SN_Info  *Oem_info);

    /*
       ��ʼ������,Ϊ֡���ٻ�����.. 0�ɹ� -1ʧ��
       ����ʹ�ã�Ulu_SDK_Init_All(...) ����, 20150910
    */
    int AYAPI_ATTR  Ulu_SDK_Init(Dev_Attribut_Struct  *attr);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ANYAN_DEVICE_SDK_H__ */

