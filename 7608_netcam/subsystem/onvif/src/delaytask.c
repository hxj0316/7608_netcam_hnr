/*
****************************************************************************
*
** \file      delaytask.c
**
**
** \brief
**
** \attention THIS SAMPLE CODE IS PROVIDED AS IS. GOFORTUNE SEMICONDUCTOR
**            ACCEPTS NO RESPONSIBILITY OR LIABILITY FOR ANY ERRORS OR
**            OMMISSIONS.
**
** (C) Copyright 2012-2013 by GOKE MICROELECTRONICS CO.,LTD
**
****************************************************************************
*/

#include "delaytask.h"
#include "sdk_sys.h"
#include <unistd.h>

static struct Timeval timeVal;

DelayQueue fDelayQueue;
char watchVariable_flag=0;
static const int MILLION = 1000000;
sem_t DelayedTask_sem;

static void handleTimeout(struct AlarmHandler* fAlarmHandler)
{
	(*fAlarmHandler->fProc)(fAlarmHandler->fClientData);
	free(fAlarmHandler);
	//fAlarmHandler->fDelayQueueEntry.handleTimeout();
}
static void DelayInterval_init(DelayInterval * fDelayInterval, long seconds, long useconds)
{
	fDelayInterval->fTv.tv_sec = seconds;
	fDelayInterval->fTv.tv_usec = useconds;
}

static void EventTime__init__(struct EventTime* fEventTime, unsigned secondsSinceEpoch ,
	    unsigned usecondsSinceEpoch )
{
	fEventTime->fTv.tv_sec = secondsSinceEpoch;
	fEventTime->fTv.tv_usec = usecondsSinceEpoch;
}
static void EventTime_init(struct EventTime* fEventTime)
{
	fEventTime->init = EventTime__init__;
}


static bool timeGe(struct timeval *timeArg1, struct timeval *timeArg2)//arg1>=arg2
{
	return (((long) timeArg1->tv_sec > (long) (timeArg2->tv_sec))
			|| (((long) timeArg1->tv_sec == (long) timeArg2->tv_sec)
			&& ((long) timeArg1->tv_usec >= (long) timeArg2->tv_usec)));
}

static bool timeLe(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 <= arg2
{
    return timeGe(timeArg2, timeArg1);
}
static bool timeLt(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 <  arg2
{
    return (!timeGe(timeArg1, timeArg2));
}
static bool timeEq(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 == arg2
{
    return (timeGe(timeArg1, timeArg2) && timeGe(timeArg2, timeArg1));
}

static bool timeNe(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 != arg2
{
    return (!timeEq(timeArg1, timeArg2));
}
static void timeSub(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 = arg1 - arg2
{
	timeArg1->tv_sec -= timeArg2->tv_sec;
	timeArg1->tv_usec -= timeArg2->tv_usec;
	//���Ϊ������Ҫ����
}

static void timeAdd(struct timeval *timeArg1, struct timeval *timeArg2)//arg1 = arg1 + arg2
{
	timeArg1->tv_sec += timeArg2->tv_sec;
	timeArg1->tv_usec += timeArg2->tv_usec;
	//���Ϊ������Ҫ����
}
static void Timeval_init(Timeval *ftimeval)
{
    ftimeval->ge = timeGe;
    ftimeval->add = timeAdd;
    ftimeval->sub = timeSub;
    ftimeval->eq = timeEq;
    ftimeval->ne = timeNe;
    ftimeval->le = timeLe;
    ftimeval->lt = timeLt;
}

static void addEntry(struct DelayQueue* fDelayQueue, DelayQueueEntry* newEntry)
{
	fDelayQueue->synchronize(fDelayQueue);

	DelayQueueEntry* cur = fDelayQueue->head(fDelayQueue);
	while ((timeVal.ge)(&newEntry->fDeltaTimeRemaining.fTv, &cur->fDeltaTimeRemaining.fTv))
	{
		//newEntry->fDeltaTimeRemaining.fTv -= cur->fDeltaTimeRemaining.fTv;
		timeVal.sub(&newEntry->fDeltaTimeRemaining.fTv, &cur->fDeltaTimeRemaining.fTv);
		cur = cur->fNext;
	}

	//cur->fDeltaTimeRemaining.fTv -= newEntry->fDeltaTimeRemaining.fTv;
	timeVal.sub(&cur->fDeltaTimeRemaining.fTv, &newEntry->fDeltaTimeRemaining.fTv);
	//add it to queue Entry
	newEntry->fNext = cur;
	newEntry->fPrev = cur->fPrev;
	cur->fPrev = newEntry->fPrev->fNext = newEntry;
}

static void removeEntry(DelayQueueEntry* entry)
{
	if (entry == NULL || entry->fNext == NULL) return;//ĩ�ڵ�

	//entry->fNext->fDeltaTimeRemaining.fTv += entry->fDeltaTimeRemaining.fTv;
	timeVal.add(&entry->fNext->fDeltaTimeRemaining.fTv, &entry->fDeltaTimeRemaining.fTv);
	entry->fPrev->fNext = entry->fNext;
	entry->fNext->fPrev = entry->fPrev;
	entry->fNext = entry->fPrev = NULL;
}
static DelayQueueEntry* head(struct DelayQueue* fDelayQueue)
{
	return fDelayQueue->fDelayQueueEntry.fNext;
}
static void synchronize(struct DelayQueue* fDelayQueue)
{
	//��ȡ�����ڵ�ʱ�� timeNow
	EventTime timeNow;
	struct timeval tvNow;
	gettimeofday(&tvNow, NULL);
	EventTime_init(&timeNow);
	timeNow.init(&timeNow, tvNow.tv_sec, tvNow.tv_usec);

	//��ǰʱ����ϴθ��µ�ʱ����:���ǲ�������Ӧ�ûָ�(����������ʱ��ᵼ�´˷���)
	if (timeVal.lt(&timeNow.fTv, &fDelayQueue->fLastSyncTime.fTv))
	{
		fDelayQueue->fLastSyncTime.fTv = timeNow.fTv;
		return;
	}

	DelayInterval timeSinceLastSync;
	//timeSinceLastSync.fTv = timeNow.fTv - fDelayQueue->fLastSyncTime.fTv;

	timeSinceLastSync.fTv = timeNow.fTv;
	timeVal.sub(&timeSinceLastSync.fTv, &fDelayQueue->fLastSyncTime.fTv);
	//		timeSinceLastSync -= fDelayQueue->fLastSyncTime.fTv;


	fDelayQueue->fLastSyncTime.fTv = timeNow.fTv;//ͬ��ʱ��

	//������ʱ���� ����ʱ���ȷ����Щ�¼�Ӧ�ñ���0;���߼��� ׼�����е���
	DelayQueueEntry* curEntry = fDelayQueue->head(fDelayQueue);
	while (timeVal.ge(&timeSinceLastSync.fTv, &curEntry->fDeltaTimeRemaining.fTv))
	{
		timeVal.sub (&timeSinceLastSync.fTv, &curEntry->fDeltaTimeRemaining.fTv);
		memset(&curEntry->fDeltaTimeRemaining.fTv, DELAY_ZERO, sizeof(struct timeval));// = (long)DELAY_ZERO;
		curEntry =curEntry->fNext;
	}
	timeVal.sub(&curEntry->fDeltaTimeRemaining.fTv, &timeSinceLastSync.fTv);
}

static void handleAlarm(DelayQueue *fDelayQueue) {
  struct timeval fTv;
#if 0
      ONVIF_INFO("==>%s()head[0x%x]next[0x%x]pre[0x%x]\n", __func__, head(fDelayQueue), head(fDelayQueue)->fNext, head(fDelayQueue)->fPrev);
//    ONVIF_INFO("==>%s()head[0x%x]\n", __func__, head(fDelayQueue));
#endif
  memset(&fTv, DELAY_ZERO, sizeof(struct timeval));
  if (timeVal.ne(&fDelayQueue->head(fDelayQueue)->fDeltaTimeRemaining.fTv , &fTv)) fDelayQueue->synchronize(fDelayQueue);

  if (timeVal.eq(&fDelayQueue->head(fDelayQueue)->fDeltaTimeRemaining.fTv, &fTv))
  {
    // This event is due to be handled:

    DelayQueueEntry* toRemove = head(fDelayQueue);
#if 0
      ONVIF_INFO("head[0x%x]next[0x%x]pre[0x%x]\n", head(fDelayQueue), head(fDelayQueue)->fNext, head(fDelayQueue)->fPrev);
#endif

    fDelayQueue->removeEntry(toRemove); // do this first, in case handler accesses queue
    //��ȡ�������alarm_handler ��ָ��.stu.name, struct student, name
    AlarmHandler *tmp = container_of(toRemove, struct AlarmHandler, fDelayQueueEntry);//)
    tmp->handleTimeout(tmp);
//    toRemove->handleTimeout(); //delete itself
  }
}

void DelayQueue__init__(DelayQueue *fDelayQueue)
{
    //��Զ�����˵�ʱ��
    fDelayQueue->fDelayQueueEntry.fDeltaTimeRemaining.fTv.tv_sec = INT_MAX;
    fDelayQueue->fDelayQueueEntry.fDeltaTimeRemaining.fTv.tv_usec = MILLION-1;
    fDelayQueue->fDelayQueueEntry.fNext = fDelayQueue->fDelayQueueEntry.fPrev = &fDelayQueue->fDelayQueueEntry;
}
void DelayQueue_init(DelayQueue *fDelayQueue)
{
	fDelayQueue->head = head;
	fDelayQueue->handleAlarm = handleAlarm;
	fDelayQueue->synchronize = synchronize;
	fDelayQueue->removeEntry = removeEntry;
	fDelayQueue->addEntry = addEntry;
	fDelayQueue->init = DelayQueue__init__;
}

void SingleStep(unsigned maxDelayTime)
{
	fDelayQueue.handleAlarm(&fDelayQueue);
	return;
}

void doEventLoop(char* watchVariable) {
  // Repeatedly loop, handling readble sockets and timed events:
  while (1) {
    if (watchVariable != NULL && *watchVariable != 0) break;
    sleep(1);
    SingleStep(0);
  }
}

void AlarmHandler__init__(AlarmHandler* fAlarmHandler, TaskFunc* fProc, void * fClientData, DelayInterval *timeToDelay)
{
	fAlarmHandler->fProc = fProc;
	fAlarmHandler->fClientData = fClientData;
	fAlarmHandler->fDelayQueueEntry.fDeltaTimeRemaining.fTv = timeToDelay->fTv;
}
void AlarmHandler_init(AlarmHandler *fAlarmHandler)
{
	fAlarmHandler->init = AlarmHandler__init__;
	fAlarmHandler->handleTimeout  = handleTimeout;
}

int token(AlarmHandler * fAlarmHandler)
{
	return fAlarmHandler->fToken;
}

TaskToken scheduleDelayedTask(__int64 microseconds,
		TaskFunc* proc,
		void* clientData)
{
	if (microseconds < 0)microseconds = 0;
	DelayInterval timeToDelay;
	DelayInterval_init(&timeToDelay, (long)(microseconds/1000000), (long)(microseconds%1000000));
	AlarmHandler *alarmHandler = (AlarmHandler *)malloc(sizeof(AlarmHandler));
    AlarmHandler_init(alarmHandler);
	alarmHandler->init(alarmHandler, proc, clientData, &timeToDelay);

	fDelayQueue.addEntry(&fDelayQueue, &alarmHandler->fDelayQueueEntry);

	return (void*) (alarmHandler->fToken);
}

void *delay_task_func(void *data)
{
    sdk_sys_thread_set_name("delay_task_func");
    Timeval_init(&timeVal);
	DelayQueue_init(&fDelayQueue);
    fDelayQueue.init(&fDelayQueue);
    sem_post(&DelayedTask_sem);
//	scheduleDelayedTask(1000000, task_test, str);
	doEventLoop(&watchVariable_flag);
	return NULL;
}

int delay_task_init()
{
    pthread_t delay_task_id;
    pthread_attr_t attr;
    if(0 != sem_init(&DelayedTask_sem, 0, 0))
    {
        perror("Semaphore init failed\n");
        return -1;
    }

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&delay_task_id, &attr, delay_task_func, NULL);
    sem_wait(&DelayedTask_sem);

    return 0;
}
