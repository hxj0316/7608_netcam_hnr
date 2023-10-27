/*!
*****************************************************************************
** FileName     : avi_search_api.c
**
** Description  : head file for search file.
**
** Author       : Bruce <zhaoquanfeng@gokemicro.com>
** Create       : 2015-9-17, create
**
** (C) Copyright 2013-2014 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/

#include "avi_common.h"
#include "avi_utility.h"
#include "avi_search_api.h"
#include "mmc_api.h"
#include "avi_rec.h"


#define _SINGLE_DEBUG_ 0
#define _DEBUG_MSG_ON_ 0


/**********************************************************************
�����������õ��ļ��Ĵ�С
��ڲ�����FILE_NODE *node: �����ļ�����Ϣ�ڵ�
����ֵ��  u32t: �ļ��Ĵ�С,���ֽ�Ϊ��λ
**********************************************************************/
u32t get_file_size(char *path)
{
    FILE *fp;
    u32t tmp32;
    if(path == NULL)
    {
        PRINT_ERR();
        return -1;
    }
    
    fp = fopen(path, "rb");
    if(fp == NULL)
    {
        PRINT_ERR();
        return -1;    
    }        
    fseek(fp, 0, SEEK_END);
    tmp32 = ftell(fp);
    fclose(fp);

    return tmp32;    
}

static FILE_LIST *creat_empty_list(u32t type, u32t ch_num, u64t start, u64t stop)
{
    
    FILE_NODE *h = (FILE_NODE *)malloc(sizeof(FILE_NODE));
    if(h == NULL)
    {
        PRINT_ERR();
        return NULL;
    }
    h->next = NULL;

    FILE_LIST *list = (FILE_LIST *)malloc(sizeof(FILE_LIST));
    if(list == NULL)
    {
        PRINT_ERR();
        return NULL;
    }        
    list->type = type;
    list->ch_num = ch_num;
    list->start = start;
    list->stop = stop;
    list->head = h;
    list->len = 0;
    list->index = 0;
    
    return list;
}

static int insert_node(FILE_LIST *list, char *file_path)
{
    int i;
    FILE_NODE *p, *q;
    char file_name[128];
    p = NULL;
    q = NULL;
 
    memset(file_name, 0 ,sizeof(file_name));
    i = last_index_at(file_path, '/');
    strcpy(file_name, file_path + i + 1);
    
    p = (FILE_NODE *)malloc(sizeof(FILE_NODE));
    if(p == NULL)
    {
        PRINT_ERR();
        return -1;
    }    
    strcpy(p->path, file_path);
    p->start = avi_get_start_time(file_name);
    p->stop = avi_get_stop_time(file_name);    
    p->next = NULL;

#if _DEBUG_MSG_ON_
    PRINT_INFO("insert node to list: file_path = %s, start = %llu, stop = %llu\n", 
            p->path, p->start, p->stop);
#endif
    
    q = list->head;
    
    while(q->next != NULL)
    {
        if(q->next->start < p->start)
            q = q->next;         
        else if(q->next->start == p->start) //�����ظ�����
        {
            free(p);
            return 0;            
        }
        else
            break;
    }


    p->next = q->next;    
    q->next = p;

    list->len ++;

    return 0;
}

/**********************************************************************
�����������ж��Ƿ���ҵ��ļ�
��ڲ�����FILE_LIST *list: �����ļ��Ŀ��ƾ��
����ֵ��  1: �գ�û���ҵ��ļ�
          0: �ǿգ����ҵ����ļ�
**********************************************************************/
int is_empty_list(FILE_LIST *list)
{    
    return NULL == list->head->next;
}

/**********************************************************************
�����������õ����ҵ����ļ���Ŀ
��ڲ�����FILE_LIST *list: �����ļ��Ŀ��ƾ��
����ֵ��  int: ���ҵ����ļ���Ŀ
**********************************************************************/
int get_len_list(FILE_LIST *list)
{    
    int i = 0; 
    FILE_NODE *p;
    p = list->head;
    while (p->next != NULL)    
    {        
        i++;        
        p = p->next;    
    }    
    return i;
}

/**********************************************************************
������������ӡ�����ļ��е�һ��¼��������Ϣ 
��ڲ�����INDEX_NODE *n: ¼��������Ϣָ��
����ֵ��  ��
**********************************************************************/
void print_index_node(INDEX_NODE *n)
{ 
    PRINT_INFO("******** INDEX NODE *******\n");
    PRINT_INFO("ch_num = %u\n", n->ch_num);
    PRINT_INFO("start = %llu\n", n->start);
    PRINT_INFO("stop = %llu\n", n->stop);
    PRINT_INFO("******** INDEX NODE *******\n");    
}

/**********************************************************************
������������ӡ�����ļ����ƾ���еĵ����ļ���Ϣ 
��ڲ�����FILE_NODE *n: ���Һ�õ��ĵ����ļ���Ϣ 
����ֵ��  ��
**********************************************************************/
void print_node(FILE_NODE *n)
{
    //INT_INFO("******** NODE *******\n");
    PRINT_INFO("path = %s\n", n->path);
    //INT_INFO("start = %llu\n", n->start);
    //INT_INFO("stop = %llu\n", n->stop);
}

/**********************************************************************
������������ӡ�����ļ����ƾ���е������ļ���Ϣ 
��ڲ�����FILE_LIST *list: �����ļ����ƾ��
����ֵ��  ��
**********************************************************************/
void print_list(FILE_LIST *list)
{ 
    PRINT_INFO("\n-----------------------------------------------------------------\n");
    PRINT_INFO("print list:\n");
    PRINT_INFO("num = %u, type = %u\n", list->ch_num, list->type); 
    PRINT_INFO("start = %llu, stop = %llu\n", list->start, list->stop);
    PRINT_INFO("len = %u, index = %u\n", list->len, list->index);

    FILE_NODE *q; 
    q = list->head;
    while(q->next != NULL)
    {
        print_node(q->next);
        q = q->next;
    }

    PRINT_INFO("-----------------------------------------------------------------\n\n");    
}

/**********************************************************************
������������ӡ���սӿڵ� GK_NET_FINDDATA ��Ϣ 
��ڲ�����GK_NET_FINDDATA *p: ���Һ�õ��ĵ����ļ���Ϣ 
����ֵ��  ��
**********************************************************************/
void print_finddata(GK_NET_FINDDATA *p)
{ 
    PRINT_INFO("\n-----------------------------------------\n");
    PRINT_INFO("print finddata:\n");
    PRINT_INFO("dwSize = %lu\n", p->dwSize); 
    PRINT_INFO("csFileName = %s\n", p->csFileName); 
    PRINT_INFO("dwFileType = %lu\n", p->dwFileType); 

    PRINT_INFO("start = %04lu%02lu%02lu%02lu%02lu%02lu \n", p->stStartTime.dwYear,
                                                        p->stStartTime.dwMonth,
                                                        p->stStartTime.dwDay,
                                                        p->stStartTime.dwHour,
                                                        p->stStartTime.dwMinute,
                                                        p->stStartTime.dwSecond);
    PRINT_INFO("stop  = %04lu%02lu%02lu%02lu%02lu%02lu \n", p->stStopTime.dwYear,
                                                        p->stStopTime.dwMonth,
                                                        p->stStopTime.dwDay,
                                                        p->stStopTime.dwHour,
                                                        p->stStopTime.dwMinute,
                                                        p->stStopTime.dwSecond);  
    PRINT_INFO("dwFileSize = %lu\n", p->dwFileSize);     
    PRINT_INFO("-----------------------------------------\n");    
}


static void delete_node(FILE_LIST *list, int index)
{
    if(is_empty_list(list))
    {
        PRINT_ERR();
        return;
    }

    FILE_NODE *p, *q;

    p = list->head;

    if(index >= get_len_list(list) || index < 0)
    {
        PRINT_ERR();
        return;
    }
        

    while(index--)
        p = p->next; 

    q = p->next;
    p->next = q->next;

#if _DEBUG_MSG_ON_
    print_node(q);
    PRINT_INFO("len = %lu\n", list->len);
#endif

    q->next = NULL;
    free(q);

    list->len --;
}

/**********************************************************************
���������������ļ�����
��ڲ�����FILE_LIST *: �����ļ��Ŀ��ƾ��
����ֵ��  ��
**********************************************************************/
void search_close(FILE_LIST *list)
{   
    while(list->len)
        delete_node(list, 0);

    list->head->next = NULL;
    free(list->head);

    list->head = NULL;
    free(list);    
}

/**********************************************************************
��������������ʱ��β����ļ�
��ڲ�����FILE_LIST *: �����ļ��Ŀ��ƾ��
          FILE_NODE *node: �õ��ļ���Ϣָ��
����ֵ��  ���ն�������:
          GK_NET_FILE_SUCCESS: ��ȡ�ļ���Ϣ�ɹ�
          GK_NET_NOMOREFILE: û�и�����ļ������ҽ��� 
          GK_NET_FILE_NOFIND: δ���ҵ��ļ�
**********************************************************************/
int get_file_node(FILE_LIST *list, FILE_NODE *node)
{  
/*
#define GK_NET_FILE_SUCCESS 1000// ��ȡ�ļ���Ϣ�ɹ� 
#define GK_NET_FILE_NOFIND 1001// δ���ҵ��ļ� 
#define GK_NET_ISFINDING 1002// ���ڲ�����ȴ� 
#define GK_NET_NOMOREFILE  1003// û�и�����ļ������ҽ��� 
#define GK_NET_FILE_EXCEPTION  1004// �����ļ�ʱ�쳣 
*/
    if(list == NULL)
        return GK_NET_FILE_NOFIND;

    if(is_empty_list(list))
        return GK_NET_FILE_NOFIND;
    
    FILE_NODE *p;
    u32t index;
    p = list->head;
    index = list->index;

    if(index  == list->len)
       return GK_NET_NOMOREFILE; 

    while(index--)
        p = p->next; 

    strcpy(node->path, p->next->path);
    node->start = p->next->start;
    node->stop = p->next->stop;

    
    list->index ++;
    return GK_NET_FILE_SUCCESS;
}

static int add_file_to_list(FILE_LIST *list, u32t ch_num, u64t begin, u64t end, char *dir)
{
    DIR *pDir;  
    struct dirent *ent; 
    u32t num;
    int ret;

    u64t time_start_64, time_stop_64;
       
    
    char dir_path[128];
    char file_path[128];
    
    memset(dir_path, 0, sizeof(dir_path));
    sprintf(dir_path, "%s/%s", GRD_SD_MOUNT_POINT, dir);

    pDir = opendir(dir_path);
    if(!pDir)
    {
		PRINT_ERR("opendir fail:%s\n", dir_path);
		return 0;
	}
    while((ent = readdir(pDir)) != NULL)
    {   
        
        if(ent->d_type & DT_REG)  //�ж��Ƿ�Ϊ��Ŀ¼
        {    
#if _DEBUG_MSG_ON_
            PRINT_INFO("\n\nfind file: \n");
            PRINT_INFO("file_name = %s\n", ent->d_name); 
#endif         
            if (0 == is_avi_file_format(ent->d_name))
                continue;

            num = avi_get_ch_num(ent->d_name);
#if _DEBUG_MSG_ON_
            PRINT_INFO("ch_num = %lu, file_num = %lu\n", ch_num, num); 
#endif
            if(ch_num != num && ch_num != 0xFF)
                continue;

            time_start_64 = avi_get_start_time(ent->d_name);
            time_stop_64 = avi_get_stop_time(ent->d_name);
#if _DEBUG_MSG_ON_
            PRINT_INFO("time_start_64 = %llu, time_stop_64 = %llu\n", time_start_64, time_stop_64); 
#endif
                
            if(time_stop_64 <= begin || time_start_64 >= end)
                continue;

            memset(file_path, 0, sizeof(file_path));
            sprintf(file_path, "%s/%s", dir_path, ent->d_name);
#if _DEBUG_MSG_ON_
            PRINT_INFO("match file, file_path = %s\n", file_path);             
#endif            
            ret = insert_node(list, file_path);
            if(ret != 0)
            {
                PRINT_ERR("Fail to insert_node: %s\n", file_path);
				closedir(pDir);
                return -1;
            }
        }
    }

	closedir(pDir);
    return 0;
}

static void add_to_list(FILE_LIST *list, u32t ch_num, u64t begin, u64t end)
{
    DIR *pDir;  
    struct dirent *ent;   
    u32t date = 0; 

    u32t date_start_32, date_stop_32;

#if _DEBUG_MSG_ON_
    PRINT_INFO("\n\n******************** find dir ****************************\n");
    PRINT_INFO("find dir: begin = %llu, end = %llu\n", begin, end);
#endif
    date_start_32 = (u32t)(begin / 1000000);
    date_stop_32 = (u32t)(end / 1000000);
#if _DEBUG_MSG_ON_
    PRINT_INFO("find dir: dir_start = %lu, dir_stop = %lu\n", date_start_32, date_stop_32);
    PRINT_INFO("******************** find dir ****************************\n");
#endif


    pDir = opendir(GRD_SD_MOUNT_POINT);
    if(!pDir)
    {
        PRINT_ERR("opendir fail:%s\n", GRD_SD_MOUNT_POINT);
        return ;
    }

    while((ent = readdir(pDir)) != NULL)
    {
        if(ent->d_type & DT_DIR)  //�ж��Ƿ�ΪĿ¼
        {
            if(strcmp(ent->d_name,".") == 0 || strcmp(ent->d_name,"..") == 0)
                continue;
#if _DEBUG_MSG_ON_
            PRINT_INFO("\n\nfind dir: \n");
            PRINT_INFO("dir_name = %s\n", ent->d_name);
#endif

	            if(is_avi_dir_format(ent->d_name) == 0)
	                continue;

	            date = (u32t)atoi(ent->d_name);
		     #if _DEBUG_MSG_ON_
	           	 PRINT_INFO("date = %lu, start = %lu, stop = %lu\n", 
	                    date, date_start_32, date_stop_32);
		     #endif
	            if((date >= date_start_32) && (date <= date_stop_32))
	            {
			#if _DEBUG_MSG_ON_
	                PRINT_INFO("match dir, dir_name = %s\n", ent->d_name);
			#endif 
	                add_file_to_list(list, ch_num, begin, end, ent->d_name);         
	            }
	        }     
	    }   

	closedir(pDir);
}

/**********************************************************************
��������������ʱ��β����ļ�
��ڲ�����u32t type: ���ն�������
          RECORD_TYPE_SCHED   ��ʱ¼��
          RECORD_TYPE_MOTION  �Ƶ����¼��
          RECORD_TYPE_ALARM   �ֶ�����¼��
          RECORD_TYPE_MANU    �ֹ�¼��
          u32t ch_num: ͨ����
          u64t start: ��ʼʱ��
          u64t stop:  ����ʱ��
����ֵ��  FILE_LIST *: ���صĲ����ļ����ƾ��       
**********************************************************************/
FILE_LIST *search_file_by_time(u32t type, u32t ch_num, u64t start, u64t stop)
{
    #if 1
    /* ���ó��������������� */
    ch_num = 0xff;
    #endif

    FILE *fp;
    char index_file[128];
    int count, i, ret;
    INDEX_NODE node;
    u64t begin, end;
#if _DEBUG_MSG_ON_
    PRINT_INFO("\n\n in get_list_by_time: \n");  
#endif 
  	char timeStr[32];
	int year,month,day;

	memset(timeStr,0,sizeof(timeStr));

	sprintf(timeStr,"%llu",start);
	sscanf(timeStr,"%04d%02d%02d",&year,&month,&day);
	
    FILE_LIST *list = NULL;
    list = creat_empty_list(type, ch_num, start, stop);
    if(list == NULL)
    {
        PRINT_ERR();
        return NULL;
    }  
    
#if _DEBUG_MSG_ON_
    PRINT_INFO("after create list: num = %lu, type = %lu, start = %llu, stop = %llu\n", 
            list->ch_num, list->type, list->start, list->stop);
#endif 

    memset(index_file, 0, sizeof(index_file));
    switch(type)
    {
        case RECORD_TYPE_SCHED: //��ʱ¼�� 0x000001
            add_to_list(list, ch_num, start, stop);  //��ʱ����ڵ�ȫ��¼��
            return list;
            //sprintf(index_file, "%s/.sched", GRD_SD_MOUNT_POINT);
            break;  
        case RECORD_TYPE_MOTION: //�Ƶ����¼�� 0x000002
			sprintf(index_file, "%s/%04d%02d%02d/.motion", GRD_SD_MOUNT_POINT,year,month,day);
            break;
        case RECORD_TYPE_ALARM: //����¼�� 0x000004
            sprintf(index_file, "%s/.alarm", GRD_SD_MOUNT_POINT);
            break; 
        case RECORD_TYPE_CMD: //����¼�� 0x000008      
            sprintf(index_file, "%s/.cmd", GRD_SD_MOUNT_POINT);
            break;            
        case RECORD_TYPE_MANU: //�ֹ�¼�� 0x000010
            sprintf(index_file, "%s/.manu", GRD_SD_MOUNT_POINT);
            break; 
        default: 
            add_to_list(list, ch_num, start, stop);  //��ʱ����ڵ�ȫ��¼��
            return list;
            break;           
    }

#if _DEBUG_MSG_ON_
    PRINT_INFO("index_file =%s\n", index_file);
#endif 
    
    fp = fopen(index_file, "rb");
    if(fp == NULL)
    {
        PRINT_ERR();
        return NULL;
    }     
    fseek(fp, 0, SEEK_SET);
    ret = fread(&count, sizeof(int), 1, fp);
    if(ret != 1)
    {
        PRINT_ERR();
        fclose(fp);
        return NULL;
    }     

#if _DEBUG_MSG_ON_
    PRINT_INFO("count =%d\n\n", count);
#endif  

    for(i=0; i<count; i++)
    {
#if _DEBUG_MSG_ON_
        PRINT_INFO(" i = %d, get index node info:\n", i);
#endif 
    
        if(feof(fp))
        {
#if _DEBUG_MSG_ON_
            PRINT_INFO("find %s to end.\n", index_file);
#endif 
            break;
        }             
            
        memset(&node, 0, sizeof(INDEX_NODE));
        fread(&node, sizeof(INDEX_NODE), 1, fp);   

#if _DEBUG_MSG_ON_
        print_index_node(&node);
#endif

        if(ch_num != node.ch_num && ch_num != 0xFF)
            continue;

#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 1 ----\n");
#endif

        if((node.stop < start) || (stop < node.start))
            continue;   
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 2 ----\n");
#endif
        if((start <= node.start) && (node.stop <= stop))
        {
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 3 ----\n");
#endif       
            begin = node.start;
            end = node.stop;
        }        
        else if((node.start <= start) && (stop <= node.stop))
        {
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 4 ----\n");
#endif       
            begin = start;
            end = stop;
        }            
        else if((node.start <= start) && (start <= node.stop))
        {
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 5 ----\n");
#endif        
            begin = start;
            end = node.stop;
        }              
        else if((node.start <= stop) && (stop <= node.stop))
        {
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 6 ----\n");
#endif          
            begin = node.start;
            end = stop;
        } 
        
#if _DEBUG_MSG_ON_
    PRINT_INFO("---- position 7 ----\n");
    PRINT_INFO("adjust time : begin = %llu, end = %llu\n", begin, end);
#endif 

        if(begin == end)
            continue;

#if _DEBUG_MSG_ON_
        PRINT_INFO("begin != end\n");
#endif        

        add_to_list(list, ch_num, begin, end);
               
    }

    fclose(fp);
    return list;
}

/**********************************************************************
�������������ҵ���¼������
��ڲ�����u32t type: ���ն�������
          RECORD_TYPE_SCHED   ��ʱ¼��
          RECORD_TYPE_MOTION  �Ƶ����¼��
          RECORD_TYPE_ALARM   �ֶ�����¼��
          RECORD_TYPE_MANU    �ֹ�¼��
          u32t ch_num: ͨ����
          u64t start: ��ʼʱ��
          u64t stop:  ����ʱ��
����ֵ��  FILE_LIST *: ���صĲ����ļ����ƾ��       
**********************************************************************/
unsigned int search_day_by_month(u32t type, u32t ch_num, int year, int month)
{
    char tmp[10];
    u32t date_start_32, date_stop_32;
    
    memset(tmp, 0, sizeof(tmp));
    sprintf(tmp, "20%02d%02d01", year, month);
    date_start_32 = (u32t)atoi(tmp);
    memset(tmp, 0, sizeof(tmp));
    sprintf(tmp, "20%02d%02d31", year, month);
    date_stop_32 = (u32t)atoi(tmp);

    printf("start:%u stop:%u\n", date_start_32, date_stop_32);

    DIR *pDir;
    struct dirent *ent;
    u32t date = 0;
    unsigned int mask = 0;
    int day = 0;
    pDir = opendir(GRD_SD_MOUNT_POINT);    
    if(pDir==NULL)
    {
        PRINT_ERR("opendir fail:%s\n", GRD_SD_MOUNT_POINT);
		return mask;
    }
    while((ent = readdir(pDir)) != NULL)
    {     
        if(ent->d_type & DT_DIR)  //�ж��Ƿ�ΪĿ¼
        {         
            if(strcmp(ent->d_name,".") == 0 || strcmp(ent->d_name,"..") == 0) 
                continue;             
	     if(is_avi_dir_format(ent->d_name) == 0)
                continue;
            date = (u32t)atoi(ent->d_name);
            if((date >= date_start_32) && (date <= date_stop_32))
            {
                day = date % 100;
                printf("day: %d \n", day);
                mask |= (1 << day);      
            }
        }
    }
	closedir(pDir);

	printf("==============0x%x, mask:%x\n", (unsigned int)pDir, mask);
    return mask;
}


/**********************************************************************
������������ÿ��¼������¼�������ļ���
��ڲ�����u32t type: ���ն�������
          RECORD_TYPE_SCHED   ��ʱ¼��
          RECORD_TYPE_MOTION  �Ƶ����¼��
          RECORD_TYPE_ALARM   �ֶ�����¼��
          RECORD_TYPE_MANU    �ֹ�¼��
          u32t ch_num: ͨ����
          u64t start: ��ʼʱ��
          u64t stop:  ����ʱ��
����ֵ��  <0: ����
           0: ����
**********************************************************************/
int write_manage_info(u32t type, u32t ch_num, u64t start, u64t stop)
{
    char index_file[128];
    FILE *fp;
    int count, ret;
    INDEX_NODE index_node;
    memset(index_file, 0 ,sizeof(index_file));

	char timeStr[32] = {0};
    char datePath[32] = {0};
    char dirPath[64] = {0};

	memset(timeStr,0,sizeof(timeStr));

	sprintf(timeStr,"%llu",start);
	strncpy(datePath, timeStr, strlen("20190810"));
	sprintf(dirPath, "%s/%s", GRD_SD_MOUNT_POINT,datePath);

    //��������Ƴ����ֶ����¼�¼��Ҳ�ᴥ����¼��
    if(mmc_get_sdcard_stauts() != SD_STATUS_OK)
    {
        return -1;
    }
    switch(type)
    {
        case RECORD_TYPE_SCHED: //��ʱ¼�� 0x000001
            sprintf(index_file, "%s/.sched", GRD_SD_MOUNT_POINT);
            break;  
        case RECORD_TYPE_MOTION: //�Ƶ����¼�� 0x000002
            sprintf(index_file, "%s/%s/.motion", GRD_SD_MOUNT_POINT, datePath);
            if(access(dirPath, F_OK) != 0 ) //����Ŀ¼�����ڣ��򴴽�
            {
                ret = mkdir(dirPath, 0777);
                if(ret < 0)
                {

                    PRINT_INFO("mkdir %s error.\n", dirPath);

                    return -1;
                }
            }
            break;
        case RECORD_TYPE_ALARM: //����¼�� 0x000004
            sprintf(index_file, "%s/.alarm", GRD_SD_MOUNT_POINT);
            break; 
        case RECORD_TYPE_CMD: //����¼�� 0x000008
            sprintf(index_file, "%s/.cmd", GRD_SD_MOUNT_POINT);
            break;            
        case RECORD_TYPE_MANU: //�ֹ�¼�� 0x000010
            sprintf(index_file, "%s/.manu", GRD_SD_MOUNT_POINT);
            break;        
        case RECORD_TYPE_SCHED_2: //�ֹ�¼�� 0x000011
            sprintf(index_file, "%s/.sched2", GRD_SD_MOUNT_POINT);
            break; 				
        default:  break;           
    }

    if(access(index_file, F_OK) != 0)
    {
        fp = fopen(index_file, "wb+");
        if(NULL == fp)
        {
            PRINT_ERR();
            goto sd_error;
        }
        count = 0;
        fseek(fp, 0, SEEK_SET);
        ret = fwrite(&count, sizeof(int), 1, fp);   
        if(ret != 1)
        {
            PRINT_ERR();
            goto sd_error;
        }
#if _DEBUG_MSG_ON_
        PRINT_INFO("first write index.\n");
#endif
    }
    else
    {
        fp = fopen(index_file, "rb+");
        if(NULL == fp)
        {
            PRINT_ERR();
            goto sd_error;
        }        
        fseek(fp, 0, SEEK_SET);
        ret = fread(&count, sizeof(int), 1, fp);
        if(ret != 1)
        {
            PRINT_ERR();
            goto sd_error;
        }
#if _DEBUG_MSG_ON_
        PRINT_INFO("not first write index, get count = %d\n", count);
#endif        
    }

    memset(&index_node, 0 ,sizeof(INDEX_NODE));
    index_node.ch_num = ch_num;
    index_node.start  = start;
    index_node.stop   = stop;

  
    fseek(fp, 0, SEEK_END);
    ret = fwrite(&index_node, sizeof(INDEX_NODE), 1, fp); 
    if(ret != 1)
    {
        PRINT_ERR();
        goto sd_error;
    }    

    PRINT_INFO("write to %s : ch_num = %u, start = %llu, stop = %llu.\n", 
        index_file, index_node.ch_num, index_node.start, index_node.stop);
    
    count++;    
    fseek(fp, 0, SEEK_SET);
    ret = fwrite(&count, sizeof(int), 1, fp);  
    if(ret != 1)
    {
        PRINT_ERR();
        goto sd_error;
    }
#if _DEBUG_MSG_ON_
    PRINT_INFO("after write, count = %d\n", count);
#endif  
    
    fseek(fp, 0, SEEK_SET);    
    fclose(fp);
    return 0;
sd_error:
    mmc_sdcard_status_check();
	PRINT_ERR("Wart manager  info failed\n");
	if(fp)
		fclose(fp);
    return -1;
}




#if 0

int dms_sysnetapi_FindFile(int Handle, LPGK_NET_FILECOND pFindCond)
{
    FILE_LIST *list = NULL;

    u32t type = (u32t)pFindCond->dwFileType;
    u32t ch_num = (u32t)pFindCond->dwChannel;
    u64t start = convert_to_u64t(&(pFindCond->stStartTime));
    u64t stop = convert_to_u64t(&(pFindCond->stStopTime));

    list = get_list_by_time(type, ch_num, start, stop); 

    if(list == NULL)
    {
        PRINT_ERR();
        return -1;    
    }      
    return (int)list; 

}

/*********************************************************************************
* ����˵����
	�����ȡ���ҵ����ļ���Ϣ��
* ���������
	Handle��ģ����
	FindHandle: �ļ����Ҿ����NET_DVR_FindFile_V30()�ķ���ֵ 
	lpFindData:�����ļ���Ϣ��ָ��
* �����������
* ����ֵ  ��
   -1��ʾʧ�ܣ�����ֵ��ʾ��ǰ�Ļ�ȡ״̬����Ϣ
**********************************************************************************/
int dms_sysnetapi_FindNextFile(int Handle, int FindHandle, LPGK_NET_FINDDATA lpFindData)
{
    FILE_LIST *list = (FILE_LIST *)FindHandle;
    FILE_NODE node;
    DMS_TIME t;

    int ret;
    memset(&node, 0, sizeof(FILE_NODE));
    ret = get_node(list, &node);

    if(ret != GK_NET_FILE_SUCCESS)
    {
        PRINT_INFO("ret = %d\n", ret);
        return ret;
    }       
    
    lpFindData->dwSize = sizeof(GK_NET_FINDDATA);
    strcpy(lpFindData->csFileName, node.path);
    lpFindData->dwFileType = list->type;
    convert_to_time(&t, node.start);
    lpFindData->stStartTime = t;
    convert_to_time(&t, node.stop);
    lpFindData->stStopTime= t;  
    lpFindData->dwFileSize = get_file_size(node.path);   

    return GK_NET_FILE_SUCCESS;

}


int dms_sysnetapi_FindClose(int Handle, int FindHandle)
{
    FILE_LIST *list = (FILE_LIST *)FindHandle;

    free_list(list);

    return 0;
}

int main(int argc, char *argv[])    
{
/*
#define	RECORD_TYPE_SCHED   0x000001  //��ʱ¼�� //��Ӧȫ��
#define	RECORD_TYPE_MOTION  0x000002  //�Ƶ����¼��
#define	RECORD_TYPE_ALARM   0x000004  //����¼��
#define	RECORD_TYPE_CMD       0x000008  //����¼�� //��Ӧ��
#define	RECORD_TYPE_MANU    0x000010  //�ֹ�¼��
*/

    write_manage_info(RECORD_TYPE_MANU, 0, 20121008124000ull, 20121008124500ull);
    write_manage_info(RECORD_TYPE_CMD, 0, 20121008125000ull, 20121008125500ull);
    write_manage_info(RECORD_TYPE_ALARM, 0, 20121008130000ull, 20121008130500ull);
    write_manage_info(RECORD_TYPE_MOTION, 0, 20121008131000ull, 20121008131500ull);
    write_manage_info(RECORD_TYPE_SCHED, 0, 20121008134000ull, 20121008134500ull);

    GK_NET_FILECOND findCond;
    memset(&findCond, 0, sizeof(GK_NET_FILECOND));
    findCond.dwChannel = 0;
  
    //findCond.dwFileType = RECORD_TYPE_MANU;
    findCond.dwFileType = 0;
    convert_to_time(&(findCond.stStartTime), 20121008124000ull);
    convert_to_time(&(findCond.stStopTime), 20121008134500ull);        
    


    int handle = dms_sysnetapi_FindFile(0, &findCond);

    print_list((FILE_LIST *)handle);
    
    GK_NET_FINDDATA findData;
    memset(&findData, 0, sizeof(GK_NET_FINDDATA));    
    int ret;

/*    
    while(1)
    {
        ret = dms_sysnetapi_FindNextFile(0, handle, &findData);
        if(ret == GK_NET_FILE_NOFIND)
        {
            PRINT_INFO("no file.\n");
            break;
        }
        if(ret == GK_NET_NOMOREFILE)
        {
            PRINT_INFO("no more file.\n");
            break;
        }

        print_finddata(&findData);
    }
    */

    PRINT_INFO("\n\n\n\nfree list\n");
    dms_sysnetapi_FindClose(0, handle);

    PRINT_INFO("end.\n");
    return 0;
}

#endif

