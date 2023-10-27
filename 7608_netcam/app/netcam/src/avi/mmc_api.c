
#include "avi_common.h"
#include "avi_utility.h"
#include "mmc_api.h"
#include "utility_api.h"
#include "sdk_sys.h"
#include "avi_rec.h"

#include "cfg_record.h"

#define STAT_FTYPE_FAT  (0x00004d44)
#define STAT_FTYPE_EXFAT  (0x2011bab0)
#define STAT_FTYPE__EXT4 (0xEF53)
#define SD_FTYPE_USE_FAT	(1)


static char glb_sd_status_cheack_run = 0;
static int glb_sd_status = SD_STATUS_NOTINIT;//sd card  ״̬

static char grd_sd_pathname[128] = {0};/*SD�����豸���·��*/
static char grd_sd_partition_pathname[128]= {0};/*SD����·��*/
static int set_is_format = 0;/*�Ƿ���Ҫ��ʽ����־1:��ʽ��0:����ʽ��*/
static pthread_mutex_t sd_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t sd_check_mutex = PTHREAD_MUTEX_INITIALIZER;



int grd_sd_is_formated = 0;
int format_process = 0;

static int lastFreeSize = 0;
static int lastAllSize = 0;


static int grd_sd_is_partition_exist(void);


/**********************************************************************
�����������õ�sd����ʣ������
��ڲ�������
����ֵ��  int: ʣ��Ĵ�С����MBλ��λ
**********************************************************************/
int grd_sd_get_free_size(void)
{
    #if 0
    int ret = grd_sd_is_mount();
    if (ret != 1)
        return 0;
    #endif

    struct statfs statFS; //ϵͳstat�Ľṹ��
    u64t freeBytes = 0;

    if (statfs(GRD_SD_MOUNT_POINT, &statFS) == -1) {              //��ȡ������״̬
        //PRINT_INFO("statfs failed, path -> [%s]\n", GRD_SD_MOUNT_POINT);
        return 0;
    }    
    freeBytes = (u64t)statFS.f_bfree * (u64t)statFS.f_frsize;       //��ϸ��ʣ��ռ����������ֽ�Ϊ��λ    
    lastFreeSize = (int)(freeBytes >> 20);
#if SD_FTYPE_USE_FAT
    if ((statFS.f_type == STAT_FTYPE_FAT || statFS.f_type == STAT_FTYPE_EXFAT) == 0)
        return 0;

#else
    if (statFS.f_type != STAT_FTYPE__EXT4)
        return 0;
    
	if(lastFreeSize <= 0 || statFS.f_ffree <= 10)//return RECORD_RESERVER_SIZE - 100 then inode deplete
	{
		PRINT_ERR("freeMB:%d, free_inodes:%d\n", lastFreeSize, statFS.f_ffree);
		lastFreeSize = 200;
	}
#endif

    return lastFreeSize;  //����ʣ��ռ���������MBΪ��λ
}

/**********************************************************************
�����������õ��ϴ�sd����ʣ������
��ڲ�������
����ֵ��  int: ʣ��Ĵ�С����MBλ��λ
**********************************************************************/
int grd_sd_get_free_size_last(void)
{
    return lastFreeSize;
}

/**********************************************************************
�����������õ�sd����ȫ������
��ڲ�������
����ֵ��  int: ʣ��Ĵ�С����MBλ��λ
**********************************************************************/
int grd_sd_get_all_size_ioctl(void)
{
    int fd;
    unsigned long long totalsize;
    int r;

    if((fd = open(GRD_SD_PATHNAME_0, O_RDONLY)) < 0)
    {
        PRINT_INFO("open error, path -> [%s]\n",GRD_SD_PATHNAME_0);
        return 0;
    }
    if((r = ioctl(fd, BLKGETSIZE64, &totalsize)) < 0)
    {
        PRINT_INFO("ioctl error \n");
        close(fd);
        return 0;
    }
    close(fd);
    return (int)(totalsize >> 20) ;  //����ʣ��ռ���������MBΪ��λ
}


/**********************************************************************
�����������õ�sd����ȫ������
��ڲ�������
����ֵ��  int: ʣ��Ĵ�С����MBλ��λ
**********************************************************************/
int grd_sd_get_all_size(void)
{
    #if 0
    int ret = grd_sd_is_mount();
    if (ret != 1)
        return 0;
    #endif
#if 1
    struct statfs statFS; //ϵͳstat�Ľṹ��

    if (statfs(GRD_SD_MOUNT_POINT, &statFS) == -1) {           //��ȡ������״̬
        //PRINT_INFO("statfs failed, path -> [%s]\n", GRD_SD_MOUNT_POINT);
        return 0;
    }
    
#if SD_FTYPE_USE_FAT
        if ((statFS.f_type == STAT_FTYPE_FAT || statFS.f_type == STAT_FTYPE_EXFAT) == 0)
            return 0;
#else
        if (statFS.f_type != STAT_FTYPE__EXT4)
            return 0;
#endif

    unsigned long long blocksize = statFS.f_bsize;    //ÿ��block��������ֽ���
    unsigned long long totalsize = blocksize * statFS.f_blocks;   //�ܵ��ֽ�����f_blocksΪblock����Ŀ

    //printf("Total_size = %lluB, = %lluKB, = %lluMB, = %lluGB,\n",
    //totalsize, totalsize >> 10, totalsize >> 20, totalsize >> 30);
#else
	unsigned long long totalsize = 0;

	char sd_sysfs_size[256] = {0};
	char sd_size[64] = {0};

	if(grd_sd_pathname[0] == '\0'){
		grd_sd_is_device_exist();
		PRINT_INFO("SDCARD DEVICE:[%s]\n",grd_sd_tempname);
		memset(grd_sd_pathname,0,sizeof(grd_sd_pathname));
		memcpy(grd_sd_pathname,grd_sd_tempname,strlen(grd_sd_tempname));
	}

	sprintf(sd_sysfs_size,"/sys/class/block/%s/size",basename(grd_sd_pathname));
	FILE * fp = fopen(sd_sysfs_size,"r");
	if(fp == NULL) return 0;

	int nbytes = fread(sd_size,60,1,fp);
	totalsize = (unsigned long long)atoi(sd_size)*512;

	fclose(fp);

	PRINT_INFO("==========================================================\n");
	PRINT_INFO("SDCARD DEVICE SIZE:[%s]\n",sd_size);
	PRINT_INFO("SDCARD DEVICE:[%s]\n",grd_sd_pathname);
	PRINT_INFO("SDCARD DEVICE:[%s]\n",basename(grd_sd_pathname));
	PRINT_INFO("Total_size = %lluB, = %lluKB, = %lluMB, = %lluGB\n",
		totalsize, totalsize >> 10, totalsize >> 20, totalsize >> 30
		);
	PRINT_INFO("==========================================================\n");

#endif

    lastAllSize = (int)(totalsize >> 20);
    return lastAllSize;  //����ʣ��ռ���������MBΪ��λ
}

/**********************************************************************
�����������õ��ϴ�sd����ȫ������
��ڲ�������
����ֵ��  int: ʣ��Ĵ�С����MBλ��λ
**********************************************************************/
int grd_sd_get_all_size_last(void)
{
    return lastAllSize;
}

/**********************************************************************
����������ͨ���ļ�ϵͳ�����ж�sd���Ƿ�mount��
��ڲ�������
����ֵ��  1: ��mount��
          0: û�б�mount��
**********************************************************************/
int grd_sd_is_mount(void)
{
    struct statfs statFS; //ϵͳstat�Ľṹ��

    if (statfs(GRD_SD_MOUNT_POINT, &statFS) == -1) {           //��ȡ������״̬
        //PRINT_INFO("statfs failed, path -> [%s]\n", GRD_SD_MOUNT_POINT);
        return 0;
    }
	//PRINT_INFO("statFS.f_type %x is mounted\n", statFS.f_type);

    //��ֹ����SD�Ѿ��γ���/dev/mmcblk0�����ڣ���û���Զ�ж�ص����
    #if SD_FTYPE_USE_FAT
    if ((statFS.f_type == STAT_FTYPE_FAT || statFS.f_type == STAT_FTYPE_EXFAT)&& (grd_sd_is_partition_exist() == 1)) {
        //PRINT_INFO("%s is mounted\n", GRD_SD_MOUNT_POINT);
        return 1;
	#else
    if ((statFS.f_type == STAT_FTYPE__EXT4) && (grd_sd_is_partition_exist() == 1)) {
        //PRINT_INFO("%s is mounted\n", GRD_SD_MOUNT_POINT);
        return 1;
	#endif
    } else {
        //PRINT_INFO("%s is not mounted\n", GRD_SD_MOUNT_POINT);
        return 0;
    }

    #if 0
    int ret;
    ret = (grd_sd_get_free_size() == 0)? 0: 1;
    //PRINT_INFO("ret = %d, free size = %d\n", ret, grd_sd_get_free_size());
    return ret;
    #endif
    #if 0
    int flag = 0;
    int ret = 0;
    char line[256];

    memset(line, 0, sizeof(line));

    system("cat /proc/mounts > /tmp/mounts");

    FILE *fp = NULL;
    fp = fopen("/tmp/mounts", "r");
    if(fp == NULL)
    {
        PRINT_ERR();
        system("rm -rf /tmp/mounts"); //ɾ����ʱ�ļ�
        return -1;
    }

    while(fgets(line, sizeof(line), fp))
    {
        if(strstr(line, GRD_SD_MOUNT_POINT) != NULL)
        {
            flag = 1;
            break;
        }

        memset(line, 0, sizeof(line));
    }

    fclose(fp);
    system("rm -rf /tmp/partitions"); //ɾ����ʱ�ļ�
    return flag;
    #endif
}

/**********************************************************************
����������get the filesystem type of partion
��ڲ�����devPath, dev path
���ڲ�����fsType, filesystem type. eg. vfat, exfat
����ֵ��  <0: ����
          0 : ����
**********************************************************************/
static int sd_getFSType(char *devPath, char *fsType)
{
	int fd = 0, ret;
    char data[256] = {0};

    #if SD_FTYPE_USE_FAT
	if(!devPath)
    {
        PRINT_ERR("device is NULL!\n");
        return -1;
    }

    //read the tag data for exfat/fat32
	fd = open(devPath, O_RDONLY);
    if(!fd)
    {
        PRINT_ERR("Open device failed!\n");
        return -1;
    }
    ret = read(fd, data, sizeof(data));
	close(fd);

    if(!ret)
    {
        PRINT_ERR("read device file failed!\n");
        return -1;
    }

    if(!strncmp((data+0x52), "FAT32", strlen("FAT32"))) //th offset with 0x52 is fat32 tag;
    {
        PRINT_INFO("dev:%s, filesystem:fat32!\n", devPath);
        if(fsType)
            strcpy(fsType, "vfat");
        ret = 0;
    }
    else if(!strncmp((data+0x3), "EXFAT", strlen("EXFAT"))) //the offset with 0x3 is exfat tag;
    {
        PRINT_INFO("dev:%s, filesystem:exfat!\n", devPath);
        if(fsType)
            strcpy(fsType, "exfat");
        ret = 0;
    }
    else
    {
        PRINT_ERR("dev:%s, Unknown filesystem type!\n", devPath);
        ret = -1;
    }
	#else
		strcpy(fsType, "ext4");
	return 0;
	#endif

    return ret;
}

/**********************************************************************
����������mount sd��
��ڲ�������
����ֵ��  <0: ����
          0 : ����
**********************************************************************/
int grd_sd_mount(void)
{
    int ret = 0;
#if 1
    char fsType[16] = {0};

    if(access(GRD_SD_MOUNT_POINT, F_OK) != 0 )
    {
        if((mkdir(GRD_SD_MOUNT_POINT, 0777)) < 0)
        {
            PRINT_ERR("mkdir %s failed\n", GRD_SD_MOUNT_POINT);
            return -1;
        }
    }

    ret = sd_getFSType(grd_sd_partition_pathname, fsType);
    if(ret < 0)
    {
        PRINT_ERR("sd_getFSType %s failed\n", grd_sd_partition_pathname);
        return -1;
    }

	ret = mount(grd_sd_partition_pathname, GRD_SD_MOUNT_POINT, fsType,
						0/* ȥ��MS_SYNCHRONOUSѡ����޸�SD����д������... */, NULL);
	if(ret != 0)
        PRINT_ERR("mount failed: %s, mount %s to %s\n",strerror(errno),grd_sd_partition_pathname,GRD_SD_MOUNT_POINT);
    return ret;
#else
    char cmd[128];
    memset(cmd, 0 ,sizeof(cmd));
    sprintf(cmd, "mount %s %s", GRD_SD_PARTITION_PATHNAME, GRD_SD_MOUNT_POINT);
    new_system_call(cmd);
    return ret;
#endif
}
#if 0
int grd_sd_mount(void)
{
	int ret = 0;
    char cmd[128];
/*
    memset(cmd, 0 ,sizeof(cmd));
    sprintf(cmd, "mkdir %s", GRD_SD_MOUNT_POINT);
    system(cmd);
*/
#if 0

	ret = mount(grd_sd_partition_pathname, GRD_SD_MOUNT_POINT, "vfat", MS_SYNCHRONOUS, NULL);
	/*if(ret != 0)
        PRINT_ERR();*/
    return ret;
#endif

#if 1
    memset(cmd, 0 ,sizeof(cmd));
    sprintf(cmd, "mount %s %s", GRD_SD_PARTITION_PATHNAME, GRD_SD_MOUNT_POINT);
    new_system_call(cmd);
#endif
	return ret;
}
#endif
/**********************************************************************
����������reumount sd��
��ڲ�������
����ֵ��  <0: ����
          0 : ����
**********************************************************************/
int grd_sd_remount(void)
{
	int ret = 0;
    char fsType[16] = {0};
    if(access(GRD_SD_MOUNT_POINT, F_OK) != 0 )
    {
        if((mkdir(GRD_SD_MOUNT_POINT, 0777)) < 0)
        {
            PRINT_ERR("mkdir %s failed\n", GRD_SD_MOUNT_POINT);
            return -1;
        }
    }
    ret = sd_getFSType(grd_sd_partition_pathname, fsType);
    if(ret < 0)
    {
        PRINT_ERR("sd_getFSType %s failed\n", grd_sd_partition_pathname);
        return -1;
    }

	ret = mount(grd_sd_partition_pathname, GRD_SD_MOUNT_POINT, fsType, MS_REMOUNT, NULL);
	if(ret != 0)
        PRINT_ERR("remount sd card failed: %s\n",strerror(errno));


	return ret;
}

/**********************************************************************
����������umount sd��
��ڲ�������
����ֵ��  <0: ����
          0 : ����
**********************************************************************/
int grd_sd_umount(void)
{
	int ret = 0;

	if(0 == grd_sd_is_mount())
		return 0;
	ret = umount(GRD_SD_MOUNT_POINT);
	if(ret != 0)
	{
		PRINT_ERR("unmount sd card failed: %s\n",strerror(errno));
        new_system_call("lsof | grep sd_card");
	}
	else
		unlink(GRD_SD_MOUNT_POINT);
/*
	char cmd[128];
	memset(cmd, 0 ,sizeof(cmd));
	sprintf(cmd, "rm -rf %s", GRD_SD_MOUNT_POINT);
	system(cmd);
*/
	return ret;
}


/**********************************************************************
����������������ʽ��MMC/SD��ΪFAT32��ʽ
��ڲ�����	��
����ֵ��	-1��ʧ��
			0���ɹ�
**********************************************************************/
static int grd_sd_fdisk_and_mkdosfs(void)
{
#if 1
	FILE *fp = NULL;
	char cmd[128];
	memset(cmd, 0, sizeof(cmd));

	/*partition and format disc*/
	snprintf(cmd, sizeof(cmd), "fdisk %s > /tmp/devisc 2>&1", grd_sd_pathname);

	fp = popen(cmd, "w");
	if(fp == NULL)
	{
		PRINT_INFO("MMC popen fail\n");
		return -1;
	}

	/*del all parmary partitions and extension partitons*/
	fprintf(fp, "d\n");
	fprintf(fp, "1\n");

	fprintf(fp, "d\n");
	fprintf(fp, "2\n");

	fprintf(fp, "d\n");
	fprintf(fp, "3\n");

	fprintf(fp, "d\n");
	fprintf(fp, "4\n");

	/*one parimary partiton*/
	fprintf(fp, "n\n");
	fprintf(fp, "p\n");
	fprintf(fp, "1\n");
	fprintf(fp, "\n");
	fprintf(fp, "\n");

	/*changed fs ID for FAT32*/
	fprintf(fp, "t\n");
	#if SD_FTYPE_USE_FAT
	fprintf(fp, "c\n");
	#else
    fprintf(fp, "83\n");//fat32 b;linux 83
    #endif

	/*save*/
	fprintf(fp, "w\n");

	pclose(fp);
    delete_path("/tmp/devisc"); //ɾ����ʱ�ļ�

	sleep(1); // �ȴ��豸�ļ�����
	new_system_call("mdev -s"); // ˢ���豸�б�

#endif
	return 0;

}


/**********************************************************************
�����������ж��ַ����Ƿ�������
��ڲ�����char *str: ��Ҫ�жϵ��ַ���
����ֵ��  1: ����
          0: ������
**********************************************************************/
static int is_digit(char *str)
{
    int n, i;
    n = strlen(str);
    //int ret = 1;

    for(i = 0; i < n; i++)
    {
          if( str[i] < '0' || str[i] > '9')
            return 0;
    }
    return 1;
}

int is_fix_format_file(char *str)
{
    int n;

    char tmp[20];

    n = strlen(str);
    if(n != 18)
        return 0;

    if( str[0] != 'c' || str[1] != 'h')
        return 0;

    if( str[2] < '0' || str[2] > '9')
        return 0;

    if(str[3] != '_')
        return 0;

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, str, 4, 4+14);

    if (0 == is_digit(tmp))
        return 0;

    return 1;
}

/**********************************************************************
�����������ж���Ҫ�޸����ļ�·�����Ƿ�����Զ���ĸ�ʽ
��ڲ�����char *file_path: ��Ҫ�޸����ļ�·����
����ֵ��  1: ����
          0: ������
**********************************************************************/
int is_fix_format_path(char *file_path)
{
    int n, i;
    char str[128];
    char tmp[128];


    i = last_index_at(file_path, '/');
    if(i < 0)
        return 0;
    memset(str, 0 ,sizeof(str));
    strcpy(str, file_path + i + 1);
//    PRINT_INFO("check %s is fix format or not.\n", str);

    n = strlen(str);
    if(n != 18)
        return 0;

    if( str[0] != 'c' || str[1] != 'h')
        return 0;

    if( str[2] < '0' || str[2] > '9')
        return 0;

    if(str[3] != '_')
        return 0;

    memset(tmp, 0 ,sizeof(tmp));
    avi_substring(tmp, str, 4, 4+14);

    if (0 == is_digit(tmp))
        return 0;

    return 1;
}

/**********************************************************************
�����������ж�·�����Ƿ�����Զ���ĸ�ʽ
��ڲ�����char *str: ��Ҫ�жϵ�·����
����ֵ��  1: ����
          0: ������
**********************************************************************/
int is_avi_dir_format(char *str)
{
    int i, n;
    char tmp[30];
    int tmp32;

    n = strlen(str);

    if(n != 8)
        return 0;

    for(i = 0; i < n; i++)
    {
          if( str[i] < '0' || str[i] > '9')
            return 0;
    }

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, str, 4, 4+2);
    tmp32 = atoi(tmp);
    if(tmp32 <=0 || tmp32 > 12)
        return 0;

    memset(tmp, 0, sizeof(tmp));
    avi_substring(tmp, str, 6, 6+2);
    tmp32 = atoi(tmp);
    if(tmp32 <=0 || tmp32 > 31)
        return 0;

    return 1;
}

/**********************************************************************
�����������ж��ļ����Ƿ�����Զ���ĸ�ʽ
��ڲ�����char *str: ��Ҫ�жϵ��ļ���
����ֵ��  1: ����
          0: ������
**********************************************************************/
int is_avi_file_format(char *str)
{
    int n;
    char tmp[30];

    n = strlen(str);

    if(n != 37)
        return 0;

    if( str[0] != 'c' || str[1] != 'h')
        return 0;

    if( str[2] < '0' || str[2] > '9')
        return 0;

    if(str[3] != '_' || str[18] != '_')
        return 0;

    if(str[33] != '.' || str[34] != 'a' || str[35] != 'v' || str[36] != 'i' )
        return 0;

    memset(tmp, 0 ,sizeof(tmp));
    avi_substring(tmp, str, 4, 4+14);

    if (0 == is_digit(tmp))
        return 0;

    memset(tmp, 0 ,sizeof(tmp));
    avi_substring(tmp, str, 19, 19+14);

    if (0 == is_digit(tmp))
        return 0;

    return 1;
}

/**********************************************************************
�����������ж��ļ�·�����Ƿ�����Զ���ĸ�ʽ
��ڲ�����char *file_path: ��Ҫ�жϵ��ļ�·����
����ֵ��  1: ����
          0: ������
**********************************************************************/
int is_avi_file_path_format(char *file_path)
{
    char str[128];
    int i, ret;


    i = last_index_at(file_path, '/');
    if(i < 0)
        return 0;
    //PRINT_INFO("i : %d\n", i);
    memset(str, 0 ,sizeof(str));
    strcpy(str, file_path + i + 1);

    ret = is_avi_file_format(str);
    return ret;
}


/**********************************************************************
������������SDĿ¼�����ϵ�avi�ļ����У��������ϵ�avi�ļ�
��ڲ����� file_name: ��������Ŀ¼�е��ļ���
           min: ���ڱȽ�
           min_filename: ���ҳɹ��󣬼�¼���ϵ�avi�ļ���
����ֵ��  1: �ɹ�,�ҵ������ϵ��ļ���
          0: û���ҵ����ϵ��ļ���
**********************************************************************/
static void find_oldest_file(char *file_name, long long *min, char *min_filename)
{
    char tmp[128];
    long long tmp32;

    memset(tmp, 0 ,sizeof(tmp));
    avi_substring(tmp, file_name, 4, 4+14);
    tmp32 = atoll(tmp);
    if(tmp32 < *min)
    {
        *min = tmp32;
        strcpy(min_filename, file_name);
    }
}

/**********************************************************************
������������SDĿ¼�²������ϵ�avi�ļ���
��ڲ�����min_dir: ������ҳɹ���������ɵ��ļ�����
����ֵ��  1: �ɹ�,�ҵ������ϵ��ļ���
          0: û���ҵ����ϵ��ļ���
**********************************************************************/
static int find_oldest_dir(char *min_dir)
{
    DIR *pDir;
    struct dirent *ent;
    int min = 0;
    int tmp = 0;

    min = 99999999;

    pDir = opendir(GRD_SD_MOUNT_POINT);
	if(!pDir){
		PRINT_ERR("GRD_SD_MOUNT_POINT open fail\n");
		return 0;
	}
    while((ent = readdir(pDir)) != NULL)
    {
        if(ent->d_type & DT_DIR)  //�ж��Ƿ�ΪĿ¼
        {
            if(strcmp(ent->d_name,".") == 0 || strcmp(ent->d_name,"..") == 0)
                continue;

            if (0 == is_avi_dir_format(ent->d_name))
                continue;

            tmp = atoi(ent->d_name);
            if(tmp < min)
                min = tmp;
        }
    }
	closedir(pDir);
    if(99999999 == min)
        return 0;  //can't find dir
    sprintf(min_dir, "%s/%d", GRD_SD_MOUNT_POINT, min);
    return 1;  //find dir
}

/**********************************************************************
������������SDĿ¼�²������ϵ�avi�ļ���
��ڲ�����min_dir: ��ɵ��ļ�����
����ֵ��  0: �ɹ�,ɾ������ɵ��ļ��л����ļ�
         -1: ����
**********************************************************************/
static int del_oldest_file(char *min_dir)
{
    DIR *pDir;
    struct dirent *ent;
    long long min;
    char min_filename[128];
    char del_file[128];
    int ret = 0;

    min = 99999999999999ull;
    //PRINT_INFO("before min = %lld\n", min);
    memset(min_filename, 0, sizeof(min_filename));

    pDir = opendir(min_dir);
    if(NULL == pDir)
    {
        PRINT_ERR("opendir %s fail\n", min_dir);
        return -1;
    }
    while((ent = readdir(pDir)) != NULL)
    {

        if(ent->d_type & DT_REG)  //�ж��Ƿ�Ϊ��Ŀ¼
        {
            if (0 == is_avi_file_format(ent->d_name))
				continue;
            //PRINT_INFO("dir = %s, filename = %s\n", min_dir, ent->d_name);
            find_oldest_file(ent->d_name, &min, min_filename);
        }
    }
	closedir(pDir);
    //PRINT_INFO("find result: min = %lld, file name = %s\n", min, min_filename);
    if(min != 99999999999999ull) //�ҵ�����ɵ��ļ�
    {
        //PRINT_INFO("find !!!!!!!!!!!\n");
        memset(del_file, 0, sizeof(del_file));
        sprintf(del_file, "%s/%s", min_dir, min_filename);
        PRINT_INFO("sd card doesn't have enough space, delete file: %s\n", del_file);
        ret = remove(del_file);
        if(ret != 0)
        {
            perror("remove error : ");
            return -1;
        }
    }
    else //����С�����ڵ�Ŀ¼��û���ҵ����ļ�����ɾ������Ŀ¼
    {
        PRINT_INFO("not find !!!!!!\n");
        PRINT_INFO("del dir = %s\n", min_dir);

        if(delete_path(min_dir) != 0)
        {
            return -1;
        }
    }
    return 0;
}

/**********************************************************************
�����������ҵ����ϵ�avi�ļ��������ϵ�avi�ļ���Ȼ��ɾ�����ļ���
          ��������ϵ�avi�ļ�����û���ҵ�avi�ļ�����ɾ�����ļ��С�
          ����Ҳ������ϵ�avi�ļ��У��򷵻�0
��ڲ�������
����ֵ��  1: �ɹ�,ɾ�������ϵ��ļ��������ϵ��ļ��У���������
          0: ʧ�ܣ��Ҳ������ϵ��ļ��У�û�н���ɾ������
**********************************************************************/
int grd_sd_del_oldest_avi_file(void)
{
    char min_dir[128];
    int ret;
    memset(min_dir, 0, sizeof(min_dir));
    ret = find_oldest_dir(min_dir);
    if(ret == 0)
    {
        PRINT_INFO("no dir to del.\n");
        return 0;
    }
    if(del_oldest_file(min_dir) != 0)
    {
       PRINT_ERR("del dir %s fail.\n",min_dir);
       mmc_sdcard_status_check();
       return 0;
    }

    return 1;
}


/**********************************************************************
������������buf�н�����һ���ַ���
��ڲ�����char *hLine������������һ���ַ���
		  char **hStr��Ҫ�������ַ����ĵ�ַ
����ֵ��-1��ʧ��
		 0���ɹ�
**********************************************************************/
static int grd_sd_fileopt_getline(char *hLine, char **hStr)
{
	assert(hLine != NULL);
	assert(hStr != NULL);

	char *p = NULL;
	p = *hStr;

	while(*p == ' ' || *p == '\t' || *p == '\n')
	{
		p++;
	}

	if(*p == '\0')
	{
		return -1;
	}
	while(((*p) != '\0') && ((*p) != '\n') && ((*p) != '\r'))
	{
		*hLine++ = *p++;
	}

	*hLine = '\0';
	*hStr = p;

	return 0;
}



/**********************************************************************
�����������õ������ĸ�ʽ������
��ڲ�����char *hPartitionPathName������·����ָ��
		int diskType���豸���ͣ�	MMC/SD�� SDA...
����ֵ��-1��ʧ��
		>0�������ĸ�ʽ������
**********************************************************************/
static int grd_sd_disk_get_partition_format_type(char *hPartitionPathName, int diskType)
{
    struct statfs statFS; //ϵͳstat�Ľṹ��

    if (statfs(GRD_SD_MOUNT_POINT, &statFS) == -1) {           //��ȡ������״̬
        //PRINT_INFO("statfs failed, path -> [%s]\n", GRD_SD_MOUNT_POINT);
        return GRD_SD_FSTYPE_UNKNOWN;
    }
	//PRINT_INFO("statFS.f_type %x is mounted\n", statFS.f_type);

    //��ֹ����SD�Ѿ��γ���/dev/mmcblk0�����ڣ���û���Զ�ж�ص����
    if ((statFS.f_type == 0x00004d44 || statFS.f_type == 0x2011bab0) && (grd_sd_is_partition_exist() == 1)) {
        //PRINT_INFO("%s is mounted\n", GRD_SD_MOUNT_POINT);
        return GRD_SD_FSTYPE_FAT32;
    } else {
        //PRINT_INFO("%s is not mounted\n", GRD_SD_MOUNT_POINT);
        return GRD_SD_FSTYPE_UNKNOWN;
    }



#ifdef USE_FDSK_L
	assert(hPartitionPathName != NULL);

	int ret = 0;
	char cmd[128];
	char message[128];
	char diskPathName[32];

	memset(cmd, 0, sizeof(cmd));
	memset(message, 0, sizeof(message));
	memset(diskPathName, 0, sizeof(diskPathName));

	if(diskType == GRD_SD_DISKTYPE_MMC)		//����豸��MMC/SD��
	{
		strncpy(diskPathName, hPartitionPathName, strlen(grd_sd_pathname));
        diskPathName[strlen(grd_sd_pathname)] = '\0';
	}
	else	//���ΪSDAӲ��
	{
		strncpy(diskPathName, hPartitionPathName, strlen("/dev/sda"));
        diskPathName[strlen("/dev/sda")] = '\0';
	}

	//PRINT_INFO("pathName: %s, devPathName: %s\n", hPartitionPathName, diskPathName);

	snprintf(cmd, sizeof(cmd), "fdisk -l %s > /tmp/partitionType", diskPathName);
	new_system_call(cmd);

	int fd = 0;
	fd = open("/tmp/partitionType", O_RDONLY);
	if(fd < 0)
	{
		PRINT_INFO("open </tmp/partitionType> fail\n");
		delete_path("/tmp/partitionType");
		return -1;
	}

	char *buffer = NULL;
	buffer = (char *)malloc(1024 * 8);
	if(buffer == NULL)
	{
		PRINT_INFO("malloc buffer fail\n");
		close(fd);
		delete_path("/tmp/partitionType");
		return -1;
	}

	memset(buffer, 0, 1024 * 8);

	ret = read(fd, buffer, 1024 * 8);
	if(ret <= 0)
	{
		PRINT_INFO("read fail\n");
		free(buffer);
		buffer = NULL;
		close(fd);
		delete_path("/tmp/partitionType");
		return -1;
	}

	close(fd);
	delete_path("/tmp/partitionType");


	/*parse*/
	char line[128];
	char *str = NULL;

	memset(line, 0, sizeof(line));

	str = buffer;

	while(grd_sd_fileopt_getline(line, &str) == 0)
	{
		if((strstr(line, hPartitionPathName) != NULL) && (strstr(line, "FAT32") != NULL))
		{
			//PRINT_INFO("partition format type is FAT32, need not to be formated.\n");
			free(buffer);
			return GRD_SD_FSTYPE_FAT32;
		}

		if((strstr(line, hPartitionPathName) != NULL) && (strstr(line, "NTFS") != NULL))
		{
			PRINT_INFO("partition format type is NTFS.\n");
			free(buffer);
			return GRD_SD_FSTYPE_NTFS;
		}

		if((strstr(line, hPartitionPathName) != NULL) && (strstr(line, "PartitionMagic") != NULL))
		{
			PRINT_INFO("partition format type is PartitionMagic.\n");
			free(buffer);
			return GRD_SD_FSTYPE_REISERFS;
		}

		if((strstr(line, hPartitionPathName) != NULL) && (strstr(line, "Extended") != NULL))
		{
			PRINT_INFO("partition format type is Extended.\n");
			free(buffer);
			return GRD_SD_FSTYPE_EXTENDED;
		}
	}

	PRINT_INFO("partition format type unknown\n");
	free(buffer);
	buffer = NULL;

	return GRD_SD_FSTYPE_UNKNOWN;
	#endif
	return GRD_SD_FSTYPE_UNKNOWN;
}

/**********************************************************************
�����������ж��ļ��Ƿ����
��ڲ�����char *file_path_name: �ļ�·����ָ��
����ֵ��-1������
        	 0-9���ļ���׺����
**********************************************************************/

static int grd_sd_is_file_exist_ext(char *file_path_name)
{

    int i = 0;
    char cmd[256];
	memset(cmd, 0, sizeof(cmd));

    for(i = 0; i < 10;i++)
    {
        sprintf(cmd,"%s%d",file_path_name,i);
        if ( access(cmd,F_OK) == 0)
        {
            return i;
        }
    }
    return -1;
}





/**********************************************************************
�����������ж��ļ��Ƿ����
��ڲ�����char *file_path_name: �ļ�·����ָ��
����ֵ��-1������
		 0��������
		 1������
**********************************************************************/
static int grd_sd_is_file_exist(char *file_path_name)
{
    // ˢ���豸�ļ�ϵͳ
    //system("mdev -s");
    //sleep(1);

    char cmd[256];
	memset(cmd, 0, sizeof(cmd));

	snprintf(cmd, sizeof(cmd), "ls -l %s 2>/dev/null | wc -l > /tmp/ls_count", file_path_name);

	new_system_call(cmd);

    FILE *fp = NULL;
    char line[10];
    fp = fopen("/tmp/ls_count", "r");
    if(fp == NULL)
    {
        PRINT_INFO("open mmc file fail\n");
        delete_path("/tmp/ls_count"); //ɾ����ʱ�ļ�
        return -1;
    }

    memset(line, 0, sizeof(line));
    if(fgets(line, sizeof(line), fp) == NULL)
    {
        PRINT_INFO("fgets mmc file fail\n");
        fclose(fp);
        delete_path("/tmp/ls_count"); //ɾ����ʱ�ļ�
    }

    fclose(fp);
    delete_path("/tmp/ls_count"); //ɾ����ʱ�ļ�

    if(0 == strncmp(line, "0", 1))
    {
        //PRINT_ERR("%s not exist\n", file_path_name);
        return 0;
    }
    else if(0 == strncmp(line, "1", 1))
    {
        //PRINT_INFO("%s exist\n", file_path_name);
        return 1;
    }
    else
    {
        PRINT_INFO("check %s exist error\n", file_path_name);
        return -1;
    }
}

/**********************************************************************
�����������ж�sd���豸�Ƿ����
��ڲ�������
����ֵ��-1������
		 0��������
		 1������
**********************************************************************/
int grd_sd_is_device_exist(void)
{
    int ret = 1;
    if( grd_sd_is_file_exist_ext(GRD_SD_PATHNAME_HEADER) == -1)
    {
        ret = 0;
		if(1 == grd_sd_is_mount())//�����岦�������û��mmcblk0������mmcplk0p1 ���ҹ��سɹ�
			new_system_call("mdev -s");
    }
    return ret;
}

/**********************************************************************
�����������ж�sd���豸�����Ƿ����
��ڲ�����char *partition_name������·����ָ��
����ֵ�� 0��������
		 1������
**********************************************************************/
static int grd_sd_is_partition_exist(void)
{
    int ret = 0;
    if(access(grd_sd_partition_pathname,F_OK) == 0)
    {
        ret = 1;
    }
    return ret;
}

/**********************************************************************
����������ȷ��/dev/mmcblk[0-9] �� /dev/mmcblk0 ���� /dev/mmcblk1
��ڲ�������
����ֵ��-1����mmcblk[0-9]
		 -2����mmcblk[0-9]p[0-9]

		 0������
**********************************************************************/
int grd_sd_check_device_node(void)
{
    // ˢ���豸�ļ�ϵͳ
    //system("mdev -s");
    //sleep(1);
	int ret = 0;
    char cmd[256];
	memset(cmd, 0, sizeof(cmd));
	int blki = 0;
    int blkp = 0;

	/*����豸���/dev/mmcblk[0-9] */
    blki = grd_sd_is_file_exist_ext(GRD_SD_PATHNAME_HEADER);
    if(blki == -1)
    {
        return -1;
    }
	/*�����������*/
    snprintf(cmd, sizeof(cmd), "%s%dp", GRD_SD_PATHNAME_HEADER,blki);
    blkp = grd_sd_is_file_exist_ext(cmd);
    if(blkp == -1)
    {
        blkp = 1; // ��������������ڣ���ǿ�����÷�����Ϊ1
        ret = -2;
    }

    snprintf(grd_sd_pathname,sizeof(grd_sd_pathname),"%s%d",GRD_SD_PATHNAME_HEADER,blki);
    snprintf(grd_sd_partition_pathname,sizeof(grd_sd_partition_pathname),"%s%dp%d",GRD_SD_PATHNAME_HEADER,blki,blkp);


    return ret;
}

/**********************************************************************
�������������sd�����豸���ͷ������������ļ����͸�ʽΪFAT32��ʽ
��ڲ�������
����ֵ��
	 -4:���ڸ�ʽ��
	  -3: ��Ҫ��ʽ��
	  -2:�޿�
          -1��ʧ��(��ʽ�������ʧ��)
		  0���ɹ�
**********************************************************************/
int grd_sd_init(void)
{
	int ret = 0;
	//GRD_SD_DISKTYPE diskType = GRD_SD_DISKTYPE_MMC;
	//int formatType = GRD_SD_FSTYPE_FAT32;

    if(mmc_get_sdcard_stauts() == SD_STATUS_FORMATING)
    {
        return -4;
    }

	/*���/dev/mmcblk[0-9]�Ƿ����*/
	if(grd_sd_is_device_exist() == 0)
	{
        //PRINT_ERR("system has not a MMC device\n");
		return -2;
	}

	/*ȷ��/dev/mmcblk[0-9] �� /dev/mmcblk0 ���� /dev/mmcblk1 */
	ret = grd_sd_check_device_node();
    if(ret < 0)
    {
		if(ret == -1)
		{
			return -2;
		}
		else
		{
			return -3;
		}
    }

	/*���/dev/mmcblk0p1��/dev/mmcblk1p1�Ƿ����*/
	if( grd_sd_is_partition_exist() == 0)	//�������(grd_sd_partition_pathname)/dev/mmcblk0p1������
	{
		PRINT_ERR("system no < %s >, goto partiton first\n", grd_sd_partition_pathname);
		//grd_sd_format();
		return  -3;
	}

#if 0
	/*���/dev/mmcblk0p1��������*/
	ret = grd_sd_disk_get_partition_format_type(grd_sd_partition_pathname, GRD_SD_DISKTYPE_MMC);
	if(ret != GRD_SD_FSTYPE_FAT32)
	{
		PRINT_ERR("Check Primary </dev/mmcblk0p1> not FAT32, format partition first\n");
		//ret = grd_sd_format();
		return  -3;
	}
#endif

//	PRINT_INFO("MMC device is valid, mount\n");
    #if 0
	ret = grd_sd_umount();	//ж��MMC/SD�豸
	ret = grd_sd_mount();		//����MMC/SD�豸
	if(ret != 0)
	{
		PRINT_INFO("mount sd card failed.\n");
		return -1;
	}
    #endif
	//PRINT_INFO("init sd card success.\n");
	grd_sd_is_formated = 1;

    //fix_all_avi_file();

	return 0;
}

static void GkSdFormatThread(void)
{
	int ret;
	int umount_count = 0;
	
    sdk_sys_thread_set_name("GkSdFormatThread");

	thread_record_close();
	mmc_set_sdcard_status(SD_STATUS_FORMATING);
    format_process = 5;    
	usleep(200000);//1000ms
	/*ȷ��/dev/mmcblk[0-9] �� /dev/mmcblk0 ���� /dev/mmcblk1 */
	ret = grd_sd_check_device_node();
    if(ret == -1)
    {
    	mmc_set_sdcard_status(SD_STATUS_NOTINIT);
        format_process = 0;
		set_is_format = 0;
		PRINT_ERR("grd_sd_check_device_node fail, exit format\n");
        return;
    }
    
    new_system_call("echo  > /proc/sys/kernel/hotplug");
    new_system_call("sync");
	sleep(1);

	//ǿ��ж��
	do
	{
		sleep(1);
		ret = grd_sd_umount();
	}while((umount_count ++ < 3) && (ret < 0));
    
	if(ret < 0)
	{
		PRINT_ERR("umount fail, exit format\n");
		set_is_format = 0;
    	mmc_set_sdcard_status(SD_STATUS_OK);
		return;
	}
    /*����*/
    grd_sd_fdisk_and_mkdosfs();
    format_process = 10;

    /*��ʽ��*/
    int is_sd_mount = grd_sd_is_mount();
    if (is_sd_mount) {
        umount_count = 0;
        do
        {
            sleep(1);
            ret = grd_sd_umount();
    		if(ret < 0)
    		{
    			PRINT_ERR("umount fail\n");
    		}
        }while((umount_count ++ < 3) && (ret < 0));
    }


    format_process = 12;

    char cmd[64];
    memset(cmd, 0, sizeof(cmd));
    //mkdosfs -v -F32 -s32 /dev/mmcblk0p1
    #if SD_FTYPE_USE_FAT
    snprintf(cmd, sizeof(cmd), "/sbin/mkfs.fat -F 32 -s 128 %s", grd_sd_partition_pathname);
	#else
    snprintf(cmd, sizeof(cmd), "/usr/sbin/mkfs.ext4 %s", grd_sd_partition_pathname);//mkfs.ext4 -i 8388608 /dev/mmcblk0p1 
	#endif
    new_system_call(cmd);
    format_process = 85;

    //new_system_call("mdev -s");
    //sys_sleep(5 * 1000);
    sleep(2);
    new_system_call("sync");
    new_system_call("echo /sbin/mdev > /proc/sys/kernel/hotplug");
    grd_sd_mount();
    format_process = 95;
    new_system_call("df -h");

    format_process = 100;

    mmc_set_sdcard_status(SD_STATUS_NOTINIT);
    format_process = 0;
	set_is_format = 0;
    return;
}


/**********************************************************************
������������ʽ�� sd��
��ڲ�������
����ֵ��  <0: ����
          0 : ����
**********************************************************************/
int grd_sd_format(void)
{
	int ret;
	if(!set_is_format && mmc_get_sdcard_stauts() != SD_STATUS_FORMATING)
    {
#ifdef MODULE_SUPPORT_GOKE_UPGRADE
        goke_upgrade_udp_info(4, NULL);
#endif
		set_is_format = 1;
		//���ӳ�һ��ʱ�䣬��ֹ��ΪSD���γ����µ����ж�
		usleep(500000);//500ms
		pthread_mutex_lock(&sd_check_mutex);
		mmc_set_sdcard_status(SD_STATUS_FORMATING);
        CreateDetachThread(GkSdFormatThread, NULL, NULL);
		pthread_mutex_unlock(&sd_check_mutex);
    }
    else
    {
		PRINT_INFO("grd_sd_format error:%d,%d\n", set_is_format, mmc_get_sdcard_stauts());
    }
    return 0;

}


static void TF_audio(int _ismount,int _unmount)
{
	#if 0
	if((_ismount==_unmount)&&_unmount==0)
	{
		//PRINT_ERR("TF card is mount\n");
		netcam_audio_hint(SYSTEM_AUDIO_HINT_SD_MOUNT);
	}
	else if((_ismount==_unmount)&&_unmount==1)
	{
		//PRINT_ERR("TF card is not mount\n");
		netcam_audio_hint(SYSTEM_AUDIO_HINT_SD_NOT_MOUNT);
	}
	#endif
}


int mmc_sdcard_write_detect()
{

	int ret = -2;
	#if 0
    char detect_file[128];
	memset(detect_file,0,sizeof(detect_file));


    sprintf(detect_file, "%s/detect", GRD_SD_MOUNT_POINT);

	int fd;
    fd = open(detect_file, O_RDWR | O_CREAT);
    if(fd < 0)
    {
        //PRINT_ERR("******* sd is readonly ********\n");
        ret = -2;
    }
	else
	{
    	close(fd);
		ret = 0;
	}
	#else
	if(access(GRD_SD_MOUNT_POINT,W_OK) == 0)
	{
		ret = 0; // 0�ɶ�д���������ɶ�д
	}
	#endif
    return ret;
}


int grd_sd_get_format_process()
{
    return format_process;
}


int mmc_get_sdcard_stauts()
{
    return glb_sd_status;
}


/*sdcard error status used to osd show
*/
void mmc_set_sdcard_status(int status)
{
	pthread_mutex_lock(&sd_mutex);
	if(glb_sd_status != status){
		PRINT_INFO("sd card change:form %d to %d\n",glb_sd_status,status);

        glb_sd_status = status;
        netcam_osd_update_id();
        if (glb_sd_status == SD_STATUS_NOTINIT)
        {
            lastAllSize = 0;
            lastFreeSize = 0;
        }
    }
	pthread_mutex_unlock(&sd_mutex);

    return ;
}

void mmc_checkFsckReset(void)
{
    char fsckFile[] = "/opt/custom/cfg/fsck";
    char cms[64] = {0};
    if (access(fsckFile, F_OK) == 0)
    {
#ifdef MODULE_SUPPORT_GOKE_UPGRADE
        goke_upgrade_udp_info(6, NULL);
#endif
        sprintf(cms, "rm -f %s ", fsckFile);
        PRINT_INFO("reset sd status\n");
        new_system_call(cms);
    }
}

void mmc_sdcard_status_check(void)
{
    static int ismount=0, unmount = 1, format_err_count=0;
    static int formatCnt = 0;
    int ret, sd_status;
	int sd_is_mount;//mount is 1,umount is 0
	//PRINT_ERR("mount status is :%d=====\n",grd_sd_is_mount());

	sd_is_mount = grd_sd_is_mount();
	pthread_mutex_lock(&sd_check_mutex);
	/*���и�ʽ����������ִ�и�ʽ��*/
	//if(set_is_format)
	//	_sd_format();
	sd_status = mmc_get_sdcard_stauts();

    if (sd_status == SD_STATUS_OK || sd_status == SD_STATUS_READONLY)
    {
        if (lastAllSize == 0)
        {
            lastAllSize = grd_sd_get_all_size();
            lastFreeSize = grd_sd_get_free_size();
        }
    }

    if (sd_status == SD_STATUS_FORMATING)
    {
        PRINT_INFO("mmc_sdcard_status_check status:%d\n", sd_status);
        pthread_mutex_unlock(&sd_check_mutex);
        return;
    }

    //PRINT_ERR("SD_STATUS:%d\n", sd_status);
	switch(sd_status)
	{
	    case SD_STATUS_OK:         //sd ״̬����,���³�ʼ������״̬
	    	format_err_count = 0;
            mmc_checkFsckReset();
	    case SD_STATUS_NOTINIT:    //sd �����������³�ʼ������״̬
	    case SD_STATUS_NOTFORMAT:   //sd δ��ʽ��,���³�ʼ������״̬
	        ret = grd_sd_init();
	        if (ret < 0)
			{/*û���豸����־��sd��*/
	            if( -2 == ret ){
	               mmc_set_sdcard_status(SD_STATUS_NOTINIT);
                }
				if(-3 == ret)
				{/*���豸��㵫û�з�������Ҫ��ʽ��*/
					PRINT_INFO("New sd card, no partition,format it\n");
					//grd_sd_format();
					mmc_set_sdcard_status(SD_STATUS_NOTFORMAT);
					format_err_count++;
				}
	            break;
	        }
	        else
			{ /*��������ΪFAT32��ʽ��ȷ*/
				if(SD_STATUS_OK == sd_status && sd_is_mount && mmc_sdcard_write_detect() == 0)
				{
                    formatCnt = 0;
					break;
				}
				else{/*���������������ִ�п�д���*/
                    if(!grd_sd_is_mount())
                        grd_sd_mount();
                    int mount_ok, write_ok;
                    mount_ok = grd_sd_is_mount();
                    if(0 == mmc_sdcard_write_detect()) write_ok = 1; else write_ok = 0;
                    if(mount_ok && write_ok)
                        mmc_set_sdcard_status(SD_STATUS_FORMAT_OK);//���سɹ��ҿ�д
                    else if(mount_ok && !write_ok){//���سɹ�����д
                        mmc_set_sdcard_status(SD_STATUS_READONLY);
                    }else{//δ���سɹ�
					    mmc_set_sdcard_status(SD_STATUS_NOTFORMAT);
						format_err_count++;
                    }
                }
	        }
	    	break;
	    case SD_STATUS_FORMAT_OK:   //sd ��ʽ��OK,δ���������
	    	if(sd_is_mount)
	    	{
	    		;//�Ѿ�����ֻ����д���
	    		/*��д���*/
    			ret = mmc_sdcard_write_detect();
    			if(ret == 0)
    			{
    				mmc_set_sdcard_status(SD_STATUS_OK);
    			}
    			else
    			{
    				mmc_set_sdcard_status(SD_STATUS_READONLY);
    			}
			}
			else
			{
                mmc_set_sdcard_status(SD_STATUS_NOTFORMAT);
			}

	    	break;
	    case SD_STATUS_READONLY:    //sd ֻ��,ͣ¼�����¹���
	    	if(is_thread_record_running())
				break;
	    	//thread_record_close();//wati record thread exit
	    	//grd_sd_umount();
			//grd_sd_mount();
			if(!grd_sd_is_mount()){
				mmc_set_sdcard_status(SD_STATUS_NOTFORMAT);
                break;
            }
            /*���سɹ���д���*/
	    	ret = mmc_sdcard_write_detect();
	    	if(ret == 0){
	    	    mmc_set_sdcard_status(SD_STATUS_OK);
	    	}
	    	else
	    	{
	    	    mmc_set_sdcard_status(SD_STATUS_READONLY);
	    	    format_err_count++;
	    	}
	    	break;
	    case SD_STATUS_FORMATING:    //sd ���ڸ�ʽ��,�˳��ȴ���ʽ�����
	        if (set_is_format == 0)
	        {
	    	    mmc_set_sdcard_status(SD_STATUS_OK);
	        }
	    	break;
	}
	pthread_mutex_unlock(&sd_check_mutex);

    if(format_err_count > 2)
    {
        if(runRecordCfg.sdCardFormatFlag == 1 && formatCnt < 5)      //�Զ���ʽ��
		{
#ifdef MODULE_SUPPORT_GOKE_UPGRADE
            goke_upgrade_udp_info(5, NULL);
#endif
			grd_sd_format();  // ��μ��SD���쳣�󣬾͸�ʽ��SD��
			formatCnt++;
		}
        else if (formatCnt >= 5)
        {
            //��θ�ʽ��ʧ�ܺ�ֹͣ��ʽ��
            mmc_set_sdcard_status(SD_STATUS_FORMAT_OK);
        }
        format_err_count = 0;
    }

	if(sd_is_mount) //����Ѿ�����
	{
        if(!ismount){
            PRINT_INFO("sd card status is mount ok.\n");
            unmount =0;
            TF_audio(ismount,unmount);
            ismount=1;
        }
	}
	else
	{
		if(!unmount)
		{
			unmount = 1;
			TF_audio(ismount,unmount);
			ismount=0;
		}
	}

	return ;
}
void mmc_set_sdcard_status_error()
{
	mmc_set_sdcard_status(SD_STATUS_NOTFORMAT);
	return;
}


static void *mmc_sdcard_status_check_loop(void *arg)
{
	pthread_detach(pthread_self());
	sdk_sys_thread_set_name("sd_check_loop");

	glb_sd_status_cheack_run = 1;
	while(glb_sd_status_cheack_run)
	{
		mmc_sdcard_status_check();
		sleep(2);
	}
}

int mmc_sdcard_status_check_init(void)
{
	pthread_t sdCheckId;
	if (pthread_create(&sdCheckId, NULL, &mmc_sdcard_status_check_loop, NULL))
	{
		PRINT_ERR("create sdCheckId fail!\n");
		return -1;
	}

	return 0;
}

int mmc_sdcard_status_check_exit(void)
{
	glb_sd_status_cheack_run = 0;

	return 0;
}

