#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "adpcm.h"

static int index_adjust[8] = {-1,-1,-1,-1,2,4,6,8};

static int step_table[89] = 
{
	7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,
	50,55,60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,
	408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,2066,
	2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,
	10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
};

void adpcm_encode(unsigned char * raw, int len, unsigned char * encoded, int * pre_sample, int * index)
{
	short * pcm = (short *)raw;
	int cur_sample;
	int i;
	int delta;
	int sb;
	int code;
	len >>= 1;
	
	for (i = 0;i < len;i ++)
	{
		cur_sample = pcm[i]; // �õ���ǰ�Ĳ�������
		delta = cur_sample - * pre_sample; // ���������һ��������
		if (delta < 0)
		{
			delta = -delta;
			sb = 8;	//	ȡ����ֵ
		}
		else 
		{
			sb = 0;
		}	// sb ������Ƿ���λ
		code = 4 * delta / step_table[* index];	// ���� steptable[]�õ�һ�� 0-7 ��ֵ
		if (code>7) 
			code=7;	// ������������ǿ�ȵı仯��
		
		delta = (step_table[* index] * code) / 4 + step_table[* index] / 8;	// ����ӵ�һ����Ϊ�˼������
		if (sb) 
			delta = -delta;
		* pre_sample += delta;	// �������ǰ�Ĳ�������
		if (* pre_sample > 32767)
			* pre_sample = 32767;
		else if (* pre_sample < -32768)
			* pre_sample = -32768;
		//* pre_sample = cur_sample;
		
		* index += index_adjust[code];	// ��������ǿ�ȵ����´�ȡsteptable �����
		if (* index < 0) 
			* index = 0;	// �����´εõ�����ȷ�ı仯��������
		else if (* index > 88) 
			* index = 88;
		
		if (i & 0x01)
			encoded[i >> 1] |= code | sb;
		else
			encoded[i >> 1] = (code | sb) << 4;	// ���Ϸ���λ��������
	}
}

void adpcm_decode(unsigned char * raw, int len, unsigned char * decoded, int * pre_sample, int * index)
{
	int i;
	int code;
	int sb;
	int delta;
	short * pcm = (short *)decoded;
	len <<= 1;
	for (i = 0;i < len;i ++)
	{
		if (i & 0x01)
			code = raw[i >> 1] & 0x0f;	// �õ���һ������
		else
			code = raw[i >> 1] >> 4;	// �õ���һ������
		if ((code & 8) != 0) 
			sb = 1;
		else 
			sb = 0;
		code &= 7;	//	�� code ����Ϊ���ݺͷ���

		delta = (step_table[* index] * code) / 4 + step_table[* index] / 8;	// ����ӵ�һ����Ϊ�˼������
		if (sb) 
			delta = -delta;
		* pre_sample += delta;	// �������ǰ�Ĳ�������
		if (* pre_sample > 32767)
			* pre_sample = 32767;
		else if (* pre_sample < -32768)
			* pre_sample = -32768;
		pcm[i] = * pre_sample;
		/*
		if (* pre_sample > 32767)
		pcm[i] = 32767;
		else if (* pre_sample < -32768)
		pcm[i] = -32768;
		else 
		pcm[i] = * pre_sample;
		*/
		* index += index_adjust[code];
		if (* index < 0) 
			* index = 0;
		if (* index > 88) 
			* index = 88;
	}
}


