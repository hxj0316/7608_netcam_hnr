/*!
*****************************************************************************
** FileName     : avi_utility.c
**
** Description  : avi utility api.
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Create       : 2015-8-11, create
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/

#include "avi_utility.h"

int write_char(FILE *file, char c)
{
    return fwrite(&c, sizeof(char), 1, file);
}
int write_int32(FILE *file, int i)
{
    return fwrite(&i, sizeof(int), 1, file);
}
int write_fourcc(FILE *file, int fourcc)
{
    return fwrite(&fourcc, sizeof(char)*4, 1, file);
}
int avi_write_data(FILE *file, char *data, int size)
{
    return fwrite(data, size, 1, file);
}

int read_char(FILE *file, char *c)
{
    return fread(c, sizeof(char), 1, file);
}
int read_int32(FILE *file, int *i)
{
    return fread(i, sizeof(int), 1, file);
}
int read_fourcc(FILE *file, int *fourcc)
{
    return fread(fourcc, sizeof(char)*4, 1, file);
}
int avi_utility_read_data(FILE *file, unsigned char *data, int size)
{
    return fread(data, size, 1, file);
}



void avi_set_32(void *pp, int w)
{
    char *p = pp;

    p[0] = ( w      )&0xff;
    p[1] = ( w >> 8 )&0xff;
    p[2] = ( w >> 16)&0xff;
    p[3] = ( w >> 24)&0xff;
}

/**********************************************************************
�������������ַ����н�ȡ�ַ���
��ڲ�����char *dest: Ŀ���ַ���
          char *src: Դ�ַ���
          int start: ��ȡ����ʼ�±�λ�ã�0��ʼ
          int end: ��ȡ�Ľ����±�λ�ã�end - start���ǽ�ȡ���ַ�������
����ֵ��  ��
**********************************************************************/
void avi_substring(char *dest, char *src, int start, int end)
{
    int i=start;
    if(start>strlen(src))
        return;
    if(end>strlen(src))
        end=strlen(src);
    while(i<end)
    {
        dest[i-start]=src[i];
        i++;
    }
    dest[i-start]='\0';
    return;
}

/**********************************************************************
�����������õ��ַ�����ĳ�ַ����±�λ��
��ڲ�����char *src: Դ�ַ���
          char c: ���ҵ��ַ�
����ֵ��  int: �ҵ��򷵻ظ��ַ����±�λ�ã����û�и��ַ����򷵻�-1
**********************************************************************/
int last_index_at(char *str, char c)
{
    int n;
    n = strlen(str);

    while(n--)
    {
        if(c == str[n-1])
            return n-1 ;
    }
    return -1;
}


/**********************************************************************
������������avi�ļ����õ�ͨ����
��ڲ�����char *file_name: avi�ļ���(����·����)
����ֵ��  unsigned long: ͨ����
**********************************************************************/
unsigned long avi_get_ch_num(char *file_name)
{
    unsigned long tmp;
    tmp = (unsigned long)(file_name[2] - 48);
    return tmp;
}

/**********************************************************************
������������avi�ļ����õ���ʼʱ��
��ڲ�����char *file_name: avi�ļ���(����·����)
����ֵ��  unsigned long long: ��ʼʱ��
**********************************************************************/
unsigned long long avi_get_start_time(char *file_name)
{
    char tmp_str[20] = {0};
    unsigned long long tmp64;
    avi_substring(tmp_str, file_name, 4, 4+14);
    tmp64 = (unsigned long long)atoll(tmp_str);
    return tmp64;
}

/**********************************************************************
������������avi�ļ����õ�����ʱ��
��ڲ�����char *file_name: avi�ļ���(����·����)
����ֵ��  unsigned long long: ����ʱ��
**********************************************************************/
unsigned long long avi_get_stop_time(char *file_name)
{
    char tmp_str[20] = {0};
    unsigned long long tmp64;
    avi_substring(tmp_str, file_name, 19, 19+14);
    tmp64 = (unsigned long long)atoll(tmp_str);
    return tmp64;
}

/**********************************************************************
�����������õ���ǰ���ڵ��ַ�����ʽ
��ڲ�����char *str: ��ǰ���ڵ��ַ�����ʽ
����ֵ��  ��
**********************************************************************/
void get_date_str(char *str)
{
    struct tm *ptm;
    long ts;

    ts = time(NULL);
    struct tm tt = {0};
    ptm = localtime_r(&ts, &tt);

	sprintf(str, "%04d%02d%02d", ptm->tm_year+1900,
								 ptm->tm_mon+1,
								 ptm->tm_mday);

    return;
}

/**********************************************************************
�����������õ���ǰʱ����ַ�����ʽ
��ڲ�����char *str: ��ǰʱ����ַ�����ʽ
����ֵ��  ��
**********************************************************************/
void get_nowtime_str(char *str)
{
    struct tm *ptm;
    long ts;

    ts = time(NULL);
    struct tm tt = {0};
    ptm = localtime_r(&ts, &tt);

	sprintf(str, "%04d%02d%02d%02d%02d%02d", ptm->tm_year+1900,
											 ptm->tm_mon+1,
											 ptm->tm_mday,
											 ptm->tm_hour,
											 ptm->tm_min,
											 ptm->tm_sec);

    return;
}

unsigned int get_nowtime_uint()
{
    struct tm *ptm;
    long ts;

    ts = time(NULL);
    struct tm tt = {0};
    ptm = localtime_r(&ts, &tt);

    unsigned int timep = (unsigned int)mktime(ptm);

    #if 0
    //timep += duration;
    timep += 0;
    memset(&tt, 0, sizeof(struct tm));
    ptm = localtime_r(&timep, &tt);
    char str[20] = {0};
	sprintf(str, "%04d%02d%02d%02d%02d%02d", ptm->tm_year+1900,
											 ptm->tm_mon+1,
											 ptm->tm_mday,
											 ptm->tm_hour,
											 ptm->tm_min,
											 ptm->tm_sec);

    

    ptm = gmtime_r(&timep, &tt);
    char str2[20] = {0};
	sprintf(str2, "%04d%02d%02d%02d%02d%02d", ptm->tm_year+1900,
											 ptm->tm_mon+1,
											 ptm->tm_mday,
											 ptm->tm_hour,
											 ptm->tm_min,
											 ptm->tm_sec);
    printf("local time str:%s, gm time str:%s \n", str, str2);
    #endif

    return timep;
}


/**********************************************************************
�����������õ���ǰʱ����ַ�����ʽ
��ڲ�����char *str: ��ǰʱ����ַ�����ʽ
����ֵ��  ��
**********************************************************************/
void get_nowhm_str(char *str)
{
    struct tm *ptm;
    long ts;

    ts = time(NULL);
    struct tm tt = {0};
    ptm = localtime_r(&ts, &tt);

	sprintf(str, "%02d%02d%02d", ptm->tm_hour,
								 ptm->tm_min,
								 ptm->tm_sec);

    return;
}


#if 0
/**********************************************************************
�������������ݲ�����ӡ��������ʾ.
��ڲ�����int ret: avi_record_open,avi_record_write,avi_record_close
                   �ķ���ֵ��Ϊ����
����ֵ����
**********************************************************************/
void avi_print_err(int ret)
{
    switch(ret)
    {
        case STS_OPEN_FAILED:       PRINT_INFO("open the file failed. \n"); break;
        case STS_INVALID_INPUT:     PRINT_INFO("invalid input. \n"); break;
        case STS_MKDIR_ERROR:       PRINT_INFO("mkdir error. \n"); break;
        case STS_INVALID_FORAMT:    PRINT_INFO("invalid format. \n"); break;
        case STS_MALLOC_FAILED:     PRINT_INFO("malloc error. \n"); break;
        case STS_FTRUNCATE_FAILED:  PRINT_INFO("ftruncate file failed. \n"); break;
        case STS_RENAME_FAILED:     PRINT_INFO("rename error. \n"); break;
        case STS_POLL_FAILED:       PRINT_INFO("poll error. \n"); break;

        case STS_RECORD_MODE_ERR:   PRINT_INFO("record mode error. \n"); break;
        case STS_WRITE_FAILED:      PRINT_INFO("write failed. \n"); break;
        case STS_READ_FAILED:       PRINT_INFO("read failed. \n"); break;
        case STS_SDCARD_NOT_MOUNT:  PRINT_INFO("sd card is not mounted. \n"); break;
        case STS_SDCARD_NO_SPACE:   PRINT_INFO("sd card don't have enough space. \n"); break;
        case STS_FISTFRAME_NOT_KEY: PRINT_INFO("the fist frame is not key frame. \n"); break;

        case STS_INDEX_COUNT_ERR:   PRINT_INFO("the avi index count malloc error. \n"); break;
        case STS_DOUBLE_CLOSE:      PRINT_INFO("double call close function. \n"); break;
        default: PRINT_INFO(" error. \n"); break;
    }
}
#endif

/**********************************************************************
����������ʱ���tm��ʽת����ʱ���AVI_DMS_TIME��ʽ
��ڲ�����struct tm *src: Դ������
          AVI_DMS_TIME *dst: Ŀ�������
����ֵ��  ��
**********************************************************************/
void tm_to_dmstime(AVI_DMS_TIME *dst, struct tm *src)
{
	dst->dwYear = src->tm_year;
	dst->dwMonth = src->tm_mon;
	dst->dwDay =  src->tm_mday;
	dst->dwHour = src->tm_hour;
	dst->dwMinute =src->tm_min;
	dst->dwSecond = src->tm_sec;
}

/**********************************************************************
����������ʱ���tm��ʽ�����������õ�ʱ���tm��ʽ
��ڲ�����struct tm *dst: Ŀ�������
          struct tm *src: ��һ��������
          int seconds: �ڶ���������
����ֵ��  ��
**********************************************************************/
void tm_add_seconds(struct tm *dst, struct tm *src, int time_seconds)
{
	time_t timep;
    struct tm t1 = {0};
	struct tm *p = &t1;

	p->tm_year = src->tm_year - 1900;
	p->tm_mon = src->tm_mon - 1;
	p->tm_mday = src->tm_mday;
	p->tm_hour = src->tm_hour;
	p->tm_min = src->tm_min;
	p->tm_sec = src->tm_sec;


	timep = mktime(p);

	timep += time_seconds;
    memset(&t1, 0, sizeof(struct tm));
	p = localtime_r(&timep, &t1);

	dst->tm_year = p->tm_year + 1900;
	dst->tm_mon = p->tm_mon + 1;
	dst->tm_mday =  p->tm_mday;
	dst->tm_hour = p->tm_hour;
	dst->tm_min =p->tm_min;
	dst->tm_sec = p->tm_sec;

}

/**********************************************************************
����������ʱ���u64t��ʽ����������������Ӻ��ֵ
��ڲ�����u64t opr1: ��һ��������
          int seconds: �ڶ���������
����ֵ��  u64t: ��Ӻ��ֵ
**********************************************************************/
u64t u64t_add_seconds(u64t opr1, int seconds)
{
    AVI_DMS_TIME src;
    u64t_to_time(&src, opr1);

    time_t timep;
	struct tm *p;

	time(&timep);

    struct tm t1 = {0};
    p = localtime_r(&timep, &t1);

	p->tm_year = src.dwYear - 1900;
	p->tm_mon = src.dwMonth - 1;
	p->tm_mday = src.dwDay;
	p->tm_hour = src.dwHour;
	p->tm_min = src.dwMinute;
	p->tm_sec = src.dwSecond;

	timep = mktime(p);
	timep += seconds;
    memset(&t1, 0, sizeof(struct tm));
	p = localtime_r(&timep, &t1);

    AVI_DMS_TIME dst;
	dst.dwYear = p->tm_year + 1900;
	dst.dwMonth = p->tm_mon + 1;
	dst.dwDay =  p->tm_mday;
	dst.dwHour = p->tm_hour;
	dst.dwMinute = p->tm_min;
	dst.dwSecond = p->tm_sec;

    u64t opr2 = time_to_u64t(&dst);

    return opr2;
}

/**********************************************************************
��������������2��ʱ��Ĳ�ֵ��
��ڲ�����u64t opr1: ��һ��������
          u64t opr2: �ڶ���������
����ֵ��  u64t: opr1 ��ȥ opr2
**********************************************************************/
int cal_time_sub(u64t opr1, u64t opr2)
{
    AVI_DMS_TIME src;
    u64t_to_time(&src, opr1);

    time_t timep1;
    time_t timep2;
	struct tm *p;

	time(&timep1);
    struct tm t1 = {0};
    p = localtime_r(&timep1, &t1);

	p->tm_year = src.dwYear - 1900;
	p->tm_mon = src.dwMonth - 1;
	p->tm_mday = src.dwDay;
	p->tm_hour = src.dwHour;
	p->tm_min = src.dwMinute;
	p->tm_sec = src.dwSecond;

	timep1 = mktime(p);

    memset(&src, 0, sizeof(AVI_DMS_TIME));
    u64t_to_time(&src, opr2);
	p->tm_year = src.dwYear - 1900;
	p->tm_mon = src.dwMonth - 1;
	p->tm_mday = src.dwDay;
	p->tm_hour = src.dwHour;
	p->tm_min = src.dwMinute;
	p->tm_sec = src.dwSecond;
    timep2 = mktime(p);

    return (timep1 - timep2);
}


/**********************************************************************
����������AVI_DMS_TIME ����ʱ��ת���� unsigned long long ����ʱ��
��ڲ�����AVI_DMS_TIME *p: AVI_DMS_TIME ����ʱ��
����ֵ��  u64t: unsigned long long ����ʱ��
**********************************************************************/
u64t time_to_u64t(AVI_DMS_TIME *p)
{
    char tmp[20];
    u64t tmp64 = 0;
    memset(tmp, 0, sizeof(tmp));
    sprintf(tmp, "%04lu%02lu%02lu%02lu%02lu%02lu", p->dwYear, p->dwMonth, p->dwDay,
                                            p->dwHour, p->dwMinute, p->dwSecond);
    tmp64 = atoll(tmp);
    return tmp64;
}

/**********************************************************************
����������unsigned long long ����ʱ��ת���� AVI_DMS_TIME ����ʱ��
��ڲ�����AVI_DMS_TIME *ptime: �õ� AVI_DMS_TIME ����ʱ��
          u64t tmp64: unsigned long long ����ʱ��
����ֵ��  ��
**********************************************************************/
void u64t_to_time(AVI_DMS_TIME *ptime, u64t tmp64)
{
    char tmp_str[20];
    char tmp[20];

    memset(tmp_str, 0, sizeof(tmp_str));
    sprintf(tmp_str, "%lld", tmp64);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 0, 0+4);
    ptime->dwYear = atoi(tmp);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 4, 4+2);
    ptime->dwMonth = atoi(tmp);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 6, 6+2);
    ptime->dwDay = atoi(tmp);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 8, 8+2);
    ptime->dwHour = atoi(tmp);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 10, 10+2);
    ptime->dwMinute = atoi(tmp);

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, tmp_str, 12, 12+2);
    ptime->dwSecond = atoi(tmp);

}

u64t get_u64t_now_time(void)
{
    char time_now[30];
    memset(time_now, 0, sizeof(time_now));
    get_nowtime_str(time_now);
    u64t time_64 = atoll(time_now);
    return time_64;
}

void time_to_str(long ts, char *str)
{
	struct tm *ptm = gmtime(&ts);
    if(ptm && str)
        sprintf(str, "%04d%02d%02d%02d%02d%02d", ptm->tm_year+1900,
    											 ptm->tm_mon+1,
    											 ptm->tm_mday,
    											 ptm->tm_hour,
    											 ptm->tm_min,
    											 ptm->tm_sec);
	return;
}

u64t get_time_to_u64t(long ts)
{
    char time_now[30];
    memset(time_now, 0, sizeof(time_now));
    time_to_str(ts, time_now);
    u64t time_64 = atoll(time_now);
    return time_64;
}

/* �뼶��ʱ�� */
void gk_cms_ssleep(unsigned seconds)
{
    struct timeval tv;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;
    int err;
    do {
       err = select(0, NULL, NULL, NULL, &tv);
    } while (err < 0 && errno == EINTR);
}

/* ���뼶��ʱ�� */
void gk_cms_msleep(unsigned long mSec)
{
    struct timeval tv;
    tv.tv_sec = mSec/1000;
    tv.tv_usec = (mSec % 1000) * 1000;
    int err;
    do {
       err = select(0, NULL, NULL, NULL, &tv);
    } while (err < 0 && errno == EINTR);
}
// type =1 ����ϵͳ�����룬type=2 ����ϵͳ����ms

unsigned long avi_get_sys_runtime(int type)
{
    struct timespec times = {0, 0};
    unsigned long time;

    clock_gettime(CLOCK_MONOTONIC, &times);
    //LOG_INFO("CLOCK_MONOTONIC: %lu, %lu\n", times.tv_sec, times.tv_nsec);

    if (1 == type)
    {
        time = times.tv_sec;
    }
    else
    {
        time = times.tv_sec * 1000 + times.tv_nsec / 1000000;
    }

    //LOG_INFO("time = %ld\n", time);
    return time;
}


