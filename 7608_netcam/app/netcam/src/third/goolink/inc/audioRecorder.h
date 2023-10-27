#ifdef VOICE_RECOG_DLL
#define VOICERECOGNIZEDLL_API __declspec(dllexport)
#else
#define VOICERECOGNIZEDLL_API
#endif

#ifndef AUDIO_RECORDER_H
#define AUDIO_RECORDER_H

#ifdef __cplusplus
extern "C" {
#endif

//_data�����ݸ�ʽ�Ǹ���initRecorder������������Ͷ��ģ�һ�����Ϊshort��������float��
//_sampleCout�Ǳ�ʾ_data�к��е�������������ָ_data�ĳ���
//�����Ѿ�������ź������������-1����¼���߳�Ӧ�˳�
typedef int (*r_pwrite)(void *_writer, const void *_data, unsigned long _sampleCout);

/************************************************************************
* ����¼����
*
* ���������
*
* _sampleRateInHzΪ44100
* _channelΪ��������1Ϊ��������2Ϊ������
* _audioFormatΪһ���źŵ�bit����������Ϊ16
*
*����������
*
* **_precorder ָ���û������ָ��
*
* ����ֵ 0 �ɹ�   -1 ʧ��
************************************************************************/
int LT_voiceInitRecorder(int _sampleRateInHz, int _channel, int _audioFormat, int _bufferSize, void **_precorder);

/************************************************************************
* ��ʼ¼��
*
* ����ֵ 0 �ɹ�   -1 ʧ��
************************************************************************/
int LT_voiceStartRecord(void *_recorder, void *_writer, r_pwrite _pwrite);

/************************************************************************
* ֹͣ¼�����ú�����ͬ�����أ����ٺ���������ֹͣ������ͷ��ڴ棩
*
*
* ����ֵ 0 �ɹ�   -1 ʧ��
************************************************************************/
int LT_voiceStopRecord(void *_recorder);

/************************************************************************
* �ͷ�¼��������Դ
*
* ����ֵ 0 �ɹ�   -1 ʧ��
************************************************************************/
int LT_voice_restore_init_status(int rateMode);


int LT_voice_input_enable(bool flag);


#ifdef __cplusplus
}
#endif


#endif
