/*!
*****************************************************************************
** FileName     : avi_rec.h
**
** Description  : head file for avi record.
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Create       : 2015-8-11, create
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/

#ifndef _AVI_REC_H__
#define _AVI_REC_H__

#ifdef __cplusplus
extern "C"
{
#endif


enum{
    SD_STATUS_NOTINIT = 0, //sd ��������
    SD_STATUS_OK,          //sd ״̬����
    SD_STATUS_NOTFORMAT,   //sd δ��ʽ��
    SD_STATUS_FORMAT_OK,   //sd ��ʽ��OK
    SD_STATUS_READONLY,    //sd ֻ��
    SD_STATUS_FORMATING    //sd ���ڸ�ʽ��
};

extern void manu_rec_start_alltime(int num, int rec_time_once);
extern void manu_rec_start_fixtime(int num, int rec_time, int rec_time_once);
extern void manu_rec_stop(int num);
extern int is_thread_record_running(void);

extern void thread_record_close(void);
extern int mmc_sdcard_write_detect();
extern int mmc_get_sdcard_stauts();
extern void mmc_set_sdcard_status(int status);
extern void mmc_sdcard_status_check(void);
extern int mmc_sdcard_status_check_init(void);
extern int mmc_sdcard_status_check_exit(void);


int local_record_init();
void local_record_uninit();

#ifdef __cplusplus
}
#endif
#endif

