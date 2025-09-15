/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */
#include "sample_common_ive.h"
#include "ot_ivs_md.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "SDL.h"
#include "SDL_ttf.h"
#include <time.h>
#include "sample_comm.h"
#include <stdatomic.h>
#include "ot_common_dis.h"
#include "sample_audio.h"
#define FONT_PATH "./hisi_osd.ttf"

#define OT_SAMPLE_IVE_MD_IMAGE_NUM 2
#define OT_SAMPLE_IVE_MD_MILLIC_SEC 200
#define OT_SAMPLE_IVE_MD_ADD_X_VAL 32768
#define OT_SAMPLE_IVE_MD_ADD_Y_VAL 32768
#define OT_SAMPLE_IVE_MD_THREAD_NAME_LEN 16
#define OT_SAMPLE_IVE_MD_AREA_THR_STEP 8
#define OT_SAMPLE_IVE_MD_VPSS_CHN 2
#define OT_SAMPLE_IVE_MD_NUM_TWO 2
#define OT_SAMPLE_IVE_SAD_THRESHOLD 100
#define OVERLAYEX_MIN_HANDLE 20
#include "rtsp_demo.h"
typedef struct
{
    ot_svp_src_img img[OT_SAMPLE_IVE_MD_IMAGE_NUM];
    ot_svp_dst_mem_info blob;
    ot_md_attr md_attr;
    ot_sample_svp_rect_info region;
} ot_sample_ivs_md_info;

// typedef struct
// {
//     int arr[121];
//     int labelN[301];
// } Total_result;

typedef struct
{
    int arr[241];
    int labelN[301];
} Total_result;

typedef struct
{
    ot_md_chn md_chn;
    ot_vo_layer vo_layer;
    ot_vo_chn vo_chn;
    td_s32 vpss_grp;
} ot_sample_md_vo_vpss_hld;

static td_bool g_stop_signal = TD_FALSE;
static pthread_t g_md_thread;
static ot_sample_ivs_md_info g_md_info;
// static ot_sample_svp_switch g_md_switch = {TD_FALSE, TD_TRUE};
static ot_sample_svp_switch g_md_switch = {TD_TRUE, TD_FALSE};
static sample_vi_cfg g_vi_config;
static ot_sample_src_dst_size g_src_dst;

typedef struct
{
    rtsp_demo_handle g_rtsplive;
    rtsp_session_handle session;
    int channel_num;
} rtsp_handle_struct;

static pthread_t VencPid1;
static pthread_t VencPid2;
static int EXIT_MODE_X = 1;
static int End_Rtsp = 1;
rtsp_handle_struct rtsp_handle[2];
atomic_uint random_int;
// char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黄白黑绿未单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
// char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黑蓝未绿白黄单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黑蓝未绿白黄单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
static td_void sample_ivs_md_uninit(ot_sample_ivs_md_info *md_info_ptr)
{
    td_s32 i;
    td_s32 ret;

    sample_svp_check_exps_return_void(md_info_ptr == TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++)
    {
        sample_svp_mmz_free(md_info_ptr->img[i].phys_addr[0], md_info_ptr->img[i].virt_addr[0]);
    }

    sample_svp_mmz_free(md_info_ptr->blob.phys_addr, md_info_ptr->blob.virt_addr);

    ret = ot_ivs_md_exit();
    if (ret != TD_SUCCESS)
    {
        sample_svp_trace_err("ot_ivs_md_exit fail,Error(%#x)\n", ret);
        return;
    }
}

static td_s32 sample_ivs_md_init(ot_sample_ivs_md_info *md_inf_ptr, td_u32 width, td_u32 height)
{
    td_s32 ret = OT_ERR_IVE_NULL_PTR;
    td_s32 i;
    td_u32 size, sad_mode;
    td_u8 wnd_size;

    sample_svp_check_exps_return(md_inf_ptr == TD_NULL, ret, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++)
    {
        ret = sample_common_ive_create_image(&md_inf_ptr->img[i], OT_SVP_IMG_TYPE_U8C1, width, height);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                   "Error(%#x),Create img[%d] image failed!\n", ret, i);
    }
    size = sizeof(ot_ive_ccblob);
    ret = sample_common_ive_create_mem_info(&md_inf_ptr->blob, size);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),Create blob mem info failed!\n", ret);

    /* Set attr info */
    md_inf_ptr->md_attr.alg_mode = OT_MD_ALG_MODE_BG;
    md_inf_ptr->md_attr.sad_mode = OT_IVE_SAD_MODE_MB_4X4;
    md_inf_ptr->md_attr.sad_out_ctrl = OT_IVE_SAD_OUT_CTRL_THRESHOLD;
    md_inf_ptr->md_attr.sad_threshold = OT_SAMPLE_IVE_SAD_THRESHOLD * (1 << 1);
    md_inf_ptr->md_attr.width = width;
    md_inf_ptr->md_attr.height = height;
    md_inf_ptr->md_attr.add_ctrl.x = OT_SAMPLE_IVE_MD_ADD_X_VAL;
    md_inf_ptr->md_attr.add_ctrl.y = OT_SAMPLE_IVE_MD_ADD_Y_VAL;
    md_inf_ptr->md_attr.ccl_ctrl.mode = OT_IVE_CCL_MODE_4C;
    sad_mode = (td_u32)md_inf_ptr->md_attr.sad_mode;
    wnd_size = (1 << (OT_SAMPLE_IVE_MD_NUM_TWO + sad_mode));
    md_inf_ptr->md_attr.ccl_ctrl.init_area_threshold = wnd_size * wnd_size;
    md_inf_ptr->md_attr.ccl_ctrl.step = wnd_size;

    ret = ot_ivs_md_init();
    sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),ot_ivs_md_init failed!\n", ret);

md_init_fail:
    if (ret != TD_SUCCESS)
    {
        sample_ivs_md_uninit(md_inf_ptr);
    }
    return ret;
}

static td_void sample_ivs_set_src_dst_size(ot_sample_src_dst_size *src_dst, td_u32 src_width,
                                           td_u32 src_height, td_u32 dst_width, td_u32 dst_height)
{
    src_dst->src.width = src_width;
    src_dst->src.height = src_height;
    src_dst->dst.width = dst_width;
    src_dst->dst.height = dst_height;
}

/* first frame just init reference frame, if not, change the frame idx */
static td_s32 sample_ivs_md_dma_data(td_u32 cur_idx, ot_video_frame_info *frm,
                                     ot_sample_ivs_md_info *md_ptr, td_bool *is_first_frm)
{
    td_s32 ret;
    td_bool is_instant = TD_TRUE;
    if (*is_first_frm != TD_TRUE)
    {
        ret = sample_common_ive_dma_image(frm, &md_ptr->img[cur_idx], is_instant);
        sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "sample_ive_dma_image fail,Err(%#x)\n", ret);
    }
    else
    {
        ret = sample_common_ive_dma_image(frm, &md_ptr->img[1 - cur_idx], is_instant);
        sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "sample_ive_dma_image fail,Err(%#x)\n", ret);

        *is_first_frm = TD_FALSE;
    }
    return TD_SUCCESS;
}

int shmid_tx;
unsigned char *user_addr;
unsigned char *ptr_tx;
int shmid_flag;
unsigned char *ptr_flag;
unsigned int size;
char shared_memory[16] = {0xAA,0x55,0xAA,0x55};
Total_result *p;

ot_bmp stBitmap;

ot_sample_svp_rect_info region_tmp;
ot_sample_svp_rect_info region_tmp_old;
int bmp_w, bmp_h;

#if 0
static int get_stream_from_one_channl(int s_LivevencChn, rtsp_demo_handle g_rtsplive,
                                      rtsp_session_handle session)
{
    static int s_LivevencFd = 0;
    static int s_maxFd = 0;
    td_s32 ret = 0;
    fd_set read_fds;
    int nSize;
    int i;
    ot_venc_stream stVStream;

    ot_venc_chn_status stStat;
    struct timeval TimeoutVal;
    TimeoutVal.tv_sec = 2;
    TimeoutVal.tv_usec = 0;

    s_LivevencFd = ss_mpi_venc_get_fd(s_LivevencChn);
    s_maxFd = s_maxFd > s_LivevencFd ? s_maxFd : s_LivevencFd;
    s_maxFd = s_maxFd + 1;

    pthread_detach(pthread_self());

    FD_ZERO(&read_fds);
    FD_SET(s_LivevencFd, &read_fds);

    ret = select(s_maxFd, &read_fds, NULL, NULL, &TimeoutVal);
    if (ret <= 0)
    {
        printf("%s select failed!\n", __FUNCTION__);
        // sleep(1);
        // continue;
        return -1;
    }

    // Live stream
    if (FD_ISSET(s_LivevencFd, &read_fds))
    {
        ret = ss_mpi_venc_query_status(s_LivevencChn, &stStat);
        if (TD_SUCCESS != ret)
        {
            printf("ss_mpi_venc_query_status chn[%d] failed with %#x!\n", s_LivevencChn, ret);
            // continue;
            return -1;
        }
        stVStream.pack = (ot_venc_pack *)malloc(sizeof(ot_venc_pack) * stStat.cur_packs);
        stVStream.pack_cnt = stStat.cur_packs;
        ret = ss_mpi_venc_get_stream(s_LivevencChn, &stVStream, TD_TRUE);
        if (TD_SUCCESS != ret)
        {
            printf("ss_mpi_venc_get_stream .. failed with %#x!\n", ret);
            // continue;
            return -1;
        }
        unsigned char *pStremData;
        for (i = 0; i < stVStream.pack_cnt; i++)
        {
            pStremData = (unsigned char *)stVStream.pack[i].addr + stVStream.pack[i].offset;
            nSize = stVStream.pack[i].len - stVStream.pack[i].offset;

            if (g_rtsplive)
            {
                rtsp_sever_tx_video(g_rtsplive, session, pStremData, nSize, stVStream.pack[i].pts);
                // usleep(50 *1000);
            }
        }

        ret = ss_mpi_venc_release_stream(s_LivevencChn, &stVStream);
        if (TD_SUCCESS != ret)
        {
            sample_print("ss_mpi_venc_release_stream chn[%d] .. failed with %#x!\n", s_LivevencChn, ret);
            free(stVStream.pack);
            stVStream.pack = NULL;
            // continue;
            return -1;
        }

        free(stVStream.pack);
        stVStream.pack = NULL;
    }

    // printf("=================venc end\n===============");
}

/******************************************************************************
 * funciton : get stream from each channels and save them
 ******************************************************************************/
td_void *VENC_GetVencStreamProc(td_void *p)
{
    td_s32 ret = 0;
    int i;

    printf("=========chn = %d\n", rtsp_handle[0].channel_num);

    while (End_Rtsp)
    {
        // for (i = 0; i < CHN_NUM_MAX; i++)
        // if(random_int == 1) {
        //     usleep(20);
        //     continue;
        // }

        for (i = 0; i < 1; i++)
        {
            ret = get_stream_from_one_channl(rtsp_handle[i].channel_num, rtsp_handle[i].g_rtsplive,
                                             rtsp_handle[i].session);
            // if (ret < 0)
            //     End_Rtsp = 0;
        }

    }
    free(p);
  
    return NULL;
}

#endif
int string_to_bmp(char *pu8Str)
{
    SDL_PixelFormat *fmt;
    TTF_Font *font;
    SDL_Surface *text, *temp;
    if (TTF_Init() < 0)
    {
        fprintf(stderr, "Couldn't initialize TTF: %s\n", SDL_GetError());
        SDL_Quit();
    }

    font = TTF_OpenFont(FONT_PATH, 40); // change size
    if (font == NULL)
    {
        fprintf(stderr, "Couldn't load %d pt font from %s: %s\n", 18, "ptsize", SDL_GetError());
    }

    SDL_Color forecol = {0xff, 0x00, 0x00, 0xff};
    text = TTF_RenderUTF8_Solid(font, pu8Str, forecol);

    fmt = (SDL_PixelFormat *)malloc(sizeof(SDL_PixelFormat));
    memset(fmt, 0, sizeof(SDL_PixelFormat));
    fmt->BitsPerPixel = 16;
    fmt->BytesPerPixel = 2;
    //    fmt->colorkey = 0xffffffff;
    //    fmt->alpha = 0xff;

    temp = SDL_ConvertSurface(text, fmt, 0);
    // stBitmap.data = malloc(2 * (temp->w) * (temp->h));
    // if (stBitmap.data == NULL)
    // {
    //     printf("stBitmap.data faided\r\n");
    // }
    // 奇数会导致内容变成斜体
    if(temp->w%2 != 0)
	    bmp_w = temp->w + 1;
    else
	    bmp_w = temp->w;
    if(temp->h % 2 != 0)
	    bmp_h = temp->h+1;
    else
	    bmp_h = temp->h;
   // bmp_w = temp->w;
   // bmp_h = temp->h;
   // memset(stBitmap.data, 0, (2 * (temp->w) * (temp->h)));
   // memcpy(stBitmap.data, temp->pixels, (2 * (temp->w) * (temp->h)));

   // stBitmap.width = temp->w;
   // stBitmap.height = temp->h;
      memset(stBitmap.data, 0, (2 * bmp_w * bmp_h));
      memcpy(stBitmap.data, temp->pixels, (2 * bmp_w * bmp_h));
      stBitmap.width = bmp_w;
      stBitmap.height = bmp_h;

    //char savename[20] = {0};

    //snprintf(savename, 20, "./osd/now_time.bmp");
    // printf("savename = %s\n",savename);
    //SDL_SaveBMP(temp, savename);
    free(fmt);
    SDL_FreeSurface(text);
    SDL_FreeSurface(temp);
    TTF_CloseFont(font);
    TTF_Quit();

    return 0;
}

/* 
 *描述  ：用于osd 字体bmp图像生成
 *参数  ：NULL
 *返回值：无
 *注意  ：需要加载字体ttf才能使用，否则会报段错误
 */
void *bitmap_update(void )
{
    ot_rgn_handle OverlayHandle = 0;
    td_s32 s32Ret;
    int z = 0;
    // time_t now;
    // struct tm *ptm;
    // char timestr[OSD_LENGTH] = {0};
    sdk_sys_thread_set_name("bitmap_update");
    while(1)
    {
        sleep(1);
    //     z++;
    //     if(z == 10)
    //     {
    //         z = 0;
            
    //     ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
    //     s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);//s32Ret 为RGN_HANDLE OverlayHandle
    //     if(s32Ret != TD_SUCCESS)
    //     {
    //         printf("HI_MPI_RGN_SetBitMap update failed with %#x!\n", s32Ret);
    //         // return -1;
    //         continue;
    //     }
    //     }

    //     if(p->labelN[0]==0 )
    //     {
    //         continue;
    //     }
            
    //     ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
    //     s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);//s32Ret 为RGN_HANDLE OverlayHandle
    //     if(s32Ret != TD_SUCCESS)
    //     {
    //         printf("HI_MPI_RGN_SetBitMap update failed with %#x!\n", s32Ret);
    //         // return -1;
    //         continue;
    //     }
       
    //    //memset(stBitmap.data, 0, (2 * (bmp_w) * (bmp_h)));
    //    memset(stBitmap.data, 0, (2 * 3840 * 48));
       if(p->labelN[0]==0 )
        {
	    z++;
	    if(z == 10)
        {
            z = 0;
        //printf("clear bitmap\n");
	memset(stBitmap.data, 0, (2 * (bmp_w) * (bmp_h)));
        ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
        s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);
	}
            continue;
        }
            
        ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
        s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);//s32Ret 为RGN_HANDLE OverlayHandle
        if(s32Ret != TD_SUCCESS)
        {
            printf("HI_MPI_RGN_SetBitMap update failed with %#x!\n", s32Ret);
            return -1;
        }
       
       //memset(stBitmap.data, 0, (2 * (bmp_w) * (bmp_h)));
       memset(stBitmap.data, 0, (2 * 3840 * 56));
       z=0;
       
       
    }
    return 0;
  
}

#include "cfg_system.h"
#include "sdk_sys.h"

void *osd_ttf_task(void)
{
    ot_rgn_handle OverlayHandle = 0;
    int s32Ret;
    time_t now;
    struct tm *ptm;
    char timestr[720] = {0};
    int i,j;
    char b[3];
    stBitmap.data = malloc(2 * 3840 * 48);
    sdk_sys_thread_set_name("osd_ttf_task");
    if (stBitmap.data == NULL)
    {
        printf("stBitmap.data faided\r\n");
    }
    while (1)
    {
         usleep(1000000);
        // time(&now);
        // ptm = localtime(&now);
        // snprintf(timestr, 100, "时间:%d-%02d-%02d %02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
        // printf("timstr = %s\n", timestr);
        // if(p->labelN[0]==0)
        // {
        //     ;
        // }
        if(p->labelN[0]==0)
        {
            timestr[0] = ' ';
            // printf("=====no license=======\n");
        }
        else
        {
            for(i = 0;i < p->labelN[0];i++)
            {
        //         for(j=1;j<=15;j++)
        //         {
        //             if(p->labelN[(i*15)+j] < 154 && p->labelN[(i*15)+j] != 0)
        //             {
        //                 b[0] = platename[p->labelN[(i*15)+j]];
        //                 b[1] = platename[p->labelN[(i*15)+j]+1];
        //                 b[2] = platename[p->labelN[(i*15)+j]+2];
        //             }
        //             else if(p->labelN[(i*15)+j] != 0)
        //             {
        //                 b[0] = platename[p->labelN[(i*15)+j]];
        //             }
        //             strcat(timestr,b);
        //             memset(b,0,3);
        //         }    
		// strcat(timestr,"  ");
        for(j=1;j<=15;j++)
                {
                    if(p->labelN[(i*15)+j] < 154 && p->labelN[(i*15)+j] != 0)
                    {
                        b[0] = platename[p->labelN[(i*15)+j]];
                        b[1] = platename[p->labelN[(i*15)+j]+1];
                        b[2] = platename[p->labelN[(i*15)+j]+2];
			strcat(timestr,b);
                    }
                    else if(p->labelN[(i*15)+j] > 153)
                    {

                        b[0] = platename[p->labelN[(i*15)+j]];

			strcat(timestr,b);
		     
                    }
                    
                    memset(b,0,3);
                }    
		strcat(timestr,"   ");
            }
        }
//        strcat(timestr," ");
//        string_to_bmp(timestr);
//	printf("str is %s\n", timestr);
//	printf("str AI is %s\n", runSystemCfg.ntpCfg.AI_str);
//	strcpy(runSystemCfg.ntpCfg.AI_str, timestr);
        memset(timestr, 0, 720);
    }
    return 0;
}

/*
 *描述  ：用于将视频文件添加时间水印
 *参数  ：无
 *返回值：OverlayHandle
 *注意  ：参数在HI_MPI_RGN_Create并不做检查，只有在HI_MPI_RGN_AttachToChn的时候才会报出相应的错
 */
td_s32 RGN_AddOsdToVenc(void)
{
    td_s32 s32Ret;
    ot_rgn_attr stRgnAttr;
    ot_rgn_chn_attr stChnAttr;
    ot_mpp_chn stChn;
    ot_rgn_handle OverlayHandle;
    int handle_num = 1;
    int i = 0;
    int ret;
    // RGN_CANVAS_INFO_S stCanvasInfo;
    OverlayHandle = 0;
    stChn.mod_id = OT_ID_VPSS; /**模块号**/ // HI_ID_VPSS  HI_ID_VENC
    stChn.dev_id = 0;                       /**设备号**/
    stChn.chn_id = 0;                       /**通道号**/
    /**创建区域**/
    sleep(2); // 等待位图生成
    stRgnAttr.attr.overlay.canvas_num = 2;
    stRgnAttr.type = OT_RGN_OVERLAYEX;                                                /**区域类型:叠加**/
    stRgnAttr.attr.overlay.pixel_format = OT_PIXEL_FORMAT_ARGB_1555; /**像素格式**/ // PIXEL_FORMAT_BGR_565 PIXEL_FORMAT_ARGB_1555
    if (stBitmap.width % 2 != 0)
    {
        stBitmap.width += 1;
    }

    if (stBitmap.height % 2 != 0)
    {
        stBitmap.height += 1;
    }
    printf("stBitmap.width is %d ,stBitmap.height is %d\n", stBitmap.width, stBitmap.height);
    if(stBitmap.width == 0 || stBitmap.height == 0)
    {
        stBitmap.height = 4;
        stBitmap.width = 4;
    }
    stRgnAttr.attr.overlay.size.width = 1920;   // 240;        /**区域宽**/
    stRgnAttr.attr.overlay.size.height = 56; // 192; 48       /**区域高**/
    stRgnAttr.attr.overlay.bg_color = 0x0000;         // 0x00007c00; /**区域背景颜色**/

    for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
        ret = ss_mpi_rgn_create(i, &stRgnAttr);
        if (ret != TD_SUCCESS) {
            sample_print("ss_mpi_rgn_create failed with %#x!\n", ret);
            return TD_FAILURE;
        }
    }
    

    // s32Ret = ss_mpi_rgn_create(OverlayHandle, &stRgnAttr);
    // if (s32Ret != TD_SUCCESS)
    // {
    //     printf("RGN create failed: %#x\n", s32Ret);
	// return -1;
    // }
    /**将区域叠加到通道**/
    /**设置叠加区域的通道显示属性**/
    stChnAttr.is_show = TD_TRUE;
    stChnAttr.type = OT_RGN_OVERLAYEX;
    // stChnAttr.attr.overlay_chn.point.x = 640; // 240;
    // stChnAttr.attr.overlay_chn.point.y = 320; // 192;
    stChnAttr.attr.overlay_chn.point.x = 0; // 240;
    stChnAttr.attr.overlay_chn.point.y = 0; // 192;
    stChnAttr.attr.overlay_chn.bg_alpha = 128;
    stChnAttr.attr.overlay_chn.fg_alpha = 128;
    stChnAttr.attr.overlay_chn.layer = OverlayHandle;

    /**设置QP属性**/
    stChnAttr.attr.overlay_chn.qp_info.is_abs_qp = TD_TRUE;
    stChnAttr.attr.overlay_chn.qp_info.qp_val = 0;
    stChnAttr.attr.overlay_chn.qp_info.enable = TD_TRUE;

    /**定义 OSD 反色相关属性**/
    /**单元反色区域，反色处理的基本单元,[16, 64]，需 16 对齐**/
#if 0
        stChnAttr.attr.overlay_chn.stInvertColor.stInvColArea.height = 16;
        stChnAttr.attr.overlay_chn.stInvertColor.stInvColArea.width  = 16;

        /**亮度阈值,取值范围：[0, 255]**/
        stChnAttr.attr.overlay_chn.stInvertColor.u32LumThresh = 128;//128

        /**OSD 反色触发模式**/
        stChnAttr.attr.overlay_chn.stInvertColor.enChgMod     = LESSTHAN_LUM_THRESH;

        /**OSD 反色开关。overlay不支持反色**/
        stChnAttr.attr.overlay_chn.stInvertColor.bInvColEn    = TD_FALSE;
#endif
    stChnAttr.attr.overlay_chn.dst = OT_RGN_ATTACH_JPEG_MAIN;
    // OverlayHandle = 0;
    for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
        // sample_region_get_overlayex_chn_attr(i, &chn_attr->attr.overlayex_chn);
        ret = sample_region_attach_to_chn(i, &stChn, &stChnAttr);
        if (ret != TD_SUCCESS) {
            sample_print("sample_region_attach_to_chn failed!\n");
            sample_comm_region_detach_frm_chn(i - OVERLAYEX_MIN_HANDLE + 1, OT_RGN_OVERLAYEX, &stChn);
            return ret;
        }
    }

    stBitmap.pixel_format = OT_PIXEL_FORMAT_ARGB_1555;


for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
    s32Ret = ss_mpi_rgn_set_bmp(i, &stBitmap);
}
    if (s32Ret != TD_SUCCESS)
    {
        printf("HI_MPI_RGN_SetBitMap failed with %#x!\n", s32Ret);
	return -1;
    }

    return 0;
}


int sockfd;
struct sockaddr_in serverAddr;
char *buf;
#define HOST_IP "192.168.1.66"
const int PORT = 1777;
int boxcolor[10] = {0};
void *udp_recv_thread()
{

    //	void* ptr_recv;
    // ptr_recv = malloc(256);
    // int ptr_recv[512] = {0};
    //	printf("ptr size if %d\n",sizeof(ptr_recv));
    struct sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    char ptr_recv[2480] = {0};
    int recv_num = 0;
    int i, j;
    sdk_sys_thread_set_name("udp_recv_thread");
    printf("===========total result ========================== %d\n", sizeof(Total_result));
    while (1)
    {
        //  socklen_t len = sizeof(serverAddr);
        // memset(ptr_recv,0,256);
        //		printf("recv begin\n");
        recv_num = recvfrom(sockfd, ptr_recv, sizeof(ptr_recv), 0, (struct sockaddr *)&caddr, &clen);
        //	recv(sockfd,ptr_recv,256,0);
        	
        // memcpy(p, ptr_recv, 256);
        //		printf("recv num = %d\n", recv_num);
        //		printf("recv = %d, %d, %d\n", ptr_recv[0], ptr_recv[3], ptr_recv[4]);
        //		printf("p[0] = %d\n", p[0]);
        p = (Total_result *)ptr_recv;
        
        // printf("===========================================\n");
        // printf("====arr 0 = %d\n", p->arr[0]);
        // printf("===========================================\n");
        region_tmp_old.num = p->arr[0] > 8 ? 8 : p->arr[0];
        
        if (region_tmp_old.num != 0)
        {
            for (i = 0; i < region_tmp_old.num; i++)
            {
                boxcolor[i] = (td_s32)p->arr[6 * i + 1];
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].x = (td_s32)p->arr[6 * i + 3] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].y = (td_s32)p->arr[6 * i + 4] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].x = (td_s32)p->arr[6 * i + 3] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].y = (td_s32)p->arr[6 * i + 6] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].x = (td_s32)p->arr[6 * i + 5] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].y = (td_s32)p->arr[6 * i + 6] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].x = (td_s32)p->arr[6 * i + 5] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].y = (td_s32)p->arr[6 * i + 4] / 2 * 2;
            }
        }
    }
    
}

void start_udp_server()
{
    socklen_t addr_size;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    printf("start connect udp server...\n");
    memset(&serverAddr, '\0', sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    // serverAddr.sin_addr.s_addr = inet_addr(HOST_IP);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    pthread_t recv_thread = 0;

    pthread_create(&recv_thread, NULL, udp_recv_thread, NULL);
    pthread_detach(recv_thread);
}


hi_void WriteBGRPackFile(ot_svp_img *pstImg, FILE *pFp)
{
    hi_u16 y;
    hi_u8 *pU8;
    hi_u16 height;
    hi_u16 width;

    width = pstImg->width;
    height = pstImg->height*3;
    pU8 = (hi_u8*)pstImg->virt_addr[0];
    for (y = 0; y < height; y++)
    {
        if ( 1 != fwrite(pU8,width,1,pFp))
        {
            printf("write file error, y = %d\n", y);
            return ;
        }

        pU8 += pstImg->stride[0];
    }
}

void memory_tmp()
{
    size = 24883200;
    ptr_tx = malloc(size);
    ptr_flag = malloc(16);
    user_addr = malloc(size);

    shmid_flag = shmget(87, 16, IPC_CREAT|0664);
    ptr_flag = shmat(shmid_flag, NULL, 0);
    shmid_tx = shmget(100, size, IPC_CREAT | 0664);
    ptr_tx = shmat(shmid_tx, NULL, 0);
}

void code_flag()
{
            printf("shared_memory is :%x,%x,%x,%x,%x,%x,%x\n",shared_memory[0],shared_memory[1],shared_memory[2],shared_memory[3],shared_memory[4],shared_memory[5],shared_memory[6]);

}

static td_void sample_ivs_md_proc(td_void *args)
{
    td_s32 ret;
    ot_sample_ivs_md_info *md_ptr = (ot_sample_ivs_md_info *)(args);
    ot_video_frame_info frm[OT_SAMPLE_IVE_MD_VPSS_CHN]; /* 0:base_frm, 1:ext_frm */
    ot_sample_md_vo_vpss_hld hld = {0};
    td_s32 vpss_chn[] = {OT_VPSS_CHN0, OT_VPSS_CHN1};
    td_s32 cur_idx = 0;
    td_bool is_first_frm = TD_TRUE;
    atomic_init(&random_int, 1);
    
//    sdk_sys_thread_set_name("sample_ivs_md_proc");
    sample_svp_check_exps_return(md_ptr == TD_NULL, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    /* Create chn */
    ret = ot_ivs_md_create_chn(hld.md_chn, &(md_ptr->md_attr));
    sample_svp_check_exps_return(ret != TD_SUCCESS, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "ot_ivs_md_create_chn fail\n");
    // udp server

  //  start_udp_server();

  
    int count;
    size = 24883200;

    struct timeval tv;
    struct timezone tz;
    struct tm *t;

    hi_s32 s32Ret;
    ot_svp_src_img stSrc ;
    ot_svp_dst_img stDst ;
    ot_ive_handle IveHandle ;
    ot_ive_csc_ctrl stCscControl ;
    hi_bool bInstant = HI_TRUE;

    memset(&stSrc,0,sizeof(ot_svp_src_img));
    memset(&stDst,0,sizeof(ot_svp_dst_img));
    memset(&stCscControl,0,sizeof(ot_ive_csc_ctrl));
    stCscControl.mode = OT_IVE_CSC_MODE_VIDEO_BT601_YUV_TO_RGB;

    #define RGB_SAVE
    #ifdef RGB_SAVE
    FILE *fOut;
    FILE *fSrc;
    hi_char *pchDstFileName = "/sharefs/RGB_test.bgr";
    #endif

//    while (g_stop_signal == TD_FALSE)
//    {
        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1], OT_SAMPLE_IVE_MD_MILLIC_SEC);
//	printf("==============get frame\n");
//	gettimeofday(&tv, &tz);
//	t = localtime(&tv.tv_sec);
//	printf("%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);

        sample_svp_check_exps_continue(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                       "Err(%#x),vpss_get_chn_frame failed, vpss_grp(%d), vpss_chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);

        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        sample_svp_check_failed_goto(ret, ext_free, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "Error(%#x),vpss_get_chn_frame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);
       
       	/*初始化YUV输入数据结构体stSrc*/
         stSrc.type = OT_SVP_IMG_TYPE_YUV420SP;
         stSrc.phys_addr[0] = frm[0].video_frame.phys_addr[0];
         stSrc.phys_addr[1] = frm[0].video_frame.phys_addr[1];
         stSrc.phys_addr[2] = frm[0].video_frame.phys_addr[2];
         stSrc.virt_addr[0] = frm[0].video_frame.virt_addr[0];
         stSrc.virt_addr[1] = frm[0].video_frame.virt_addr[1];
         stSrc.virt_addr[2] = frm[0].video_frame.virt_addr[2];
         stSrc.stride[0] = frm[0].video_frame.stride[0];
         stSrc.stride[1] = frm[0].video_frame.stride[1];
         stSrc.stride[2] = frm[0].video_frame.stride[2];
         stSrc.width = frm[0].video_frame.width;
         stSrc.height = frm[0].video_frame.height;
         /*初始化输出RPG数据结构体并在内存中为图像数据分配空间*/

          ret = ss_mpi_sys_mmz_alloc_cached(&stDst.phys_addr[0], (hi_void *)&stDst.virt_addr[0], "DstImg",
                                          HI_NULL, stSrc.width * stSrc.height * 3);
       
       	 if(HI_SUCCESS != ret)
         {
             printf("Error(%#x),HI_MPI_SYS_MmzAlloc_Cached failed!\n",ret) ;
             ss_mpi_sys_mmz_free(stDst.phys_addr[0],(hi_void*)stDst.phys_addr) ;
             return ret;
         }
         // memset(stDst.au64VirAddr[0],0,stSrc.u32Height * stSrc.au32Stride[0] * 3) ;
         stDst.type = OT_SVP_IMG_TYPE_U8C3_PACKAGE;
         stDst.height = stSrc.height;
         stDst.width = stSrc.width;
         stDst.stride[0] = (((frm[0].video_frame.width + 15) >> 4) << 4);
        /*将YUV数据转换到RGB planar存储，地址保存在stDst结构体中*/
         ret = ss_mpi_ive_csc(&IveHandle,&stSrc,&stDst,&stCscControl,bInstant) ;
         if(HI_SUCCESS != ret)
         {
             printf("Error(%#x),HI_MPI_IVE_CSC failed!\n",ret) ;
             // return ;
         }
	  unsigned char *user_addr_vir = (unsigned char *)ss_mpi_sys_mmap_cached(stDst.phys_addr[0], size);
	    ss_mpi_sys_flush_cache(stDst.phys_addr[0],user_addr_vir, size);
	    usleep(10*1000);
            memcpy(ptr_tx, user_addr_vir, size);
            ss_mpi_sys_munmap(user_addr_vir, size);
//	    printf("shared_memory is :%x,%x,%x,%x\n",shared_memory[0],shared_memory[1],shared_memory[2],shared_memory[3]);
	    memcpy(ptr_flag,shared_memory,7);
//	    printf("ptr_flag is :%x,%x,%x,%x\n",ptr_flag[0],ptr_flag[1],ptr_flag[2],ptr_flag[3]);
//        }

         #ifdef RGB_SAVE
     //    printf("yuv2bgr success\r\n");
         fflush(stdout);
         fOut = fopen(pchDstFileName,"wb+");
         if(HI_NULL == fOut)
         {
             printf("Open out file %s fail\n",pchDstFileName);
             fclose(fOut);
             // return;
         }

         WriteBGRPackFile(&stDst, fOut);
         fclose(fOut);
        // printf("file\r\n");
         #endif
         ret = ss_mpi_sys_mmz_free(stDst.phys_addr[0],(hi_void*)stDst.virt_addr);
         if(HI_SUCCESS != ret){
             printf("Error(%#x),HI_MPI_SYS_MmzFree failed!\n",ret);
             ss_mpi_sys_mmz_free(stDst.phys_addr[0],(hi_void*)stDst.virt_addr);
             // return ;
         }
        ret = sample_ivs_md_dma_data(cur_idx, &frm[1], md_ptr, &is_first_frm);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "dma data failed, Err(%#x)\n", ret);

        /* change idx */
        if (is_first_frm == TD_TRUE)
        {
            goto change_idx;
        }

        ret = ot_ivs_md_proc(hld.md_chn, &md_ptr->img[cur_idx], &md_ptr->img[1 - cur_idx], TD_NULL, &md_ptr->blob);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "ivs_md_proc fail,Err(%#x)\n", ret);

        sample_ivs_set_src_dst_size(&g_src_dst, md_ptr->md_attr.width, md_ptr->md_attr.height,
                                    frm[0].video_frame.width, frm[0].video_frame.height);
        ret = sample_common_ive_blob_to_rect(sample_svp_convert_addr_to_ptr(ot_ive_ccblob, md_ptr->blob.virt_addr),
                                             &(md_ptr->region), OT_SVP_RECT_NUM, OT_SAMPLE_IVE_MD_AREA_THR_STEP, g_src_dst);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "blob to rect failed!\n");
//        count++;
#if 1
        // if(strlen(ptr_tx) == 0) {

//        if (count % 5 == 0)
//        {
//            user_addr = (unsigned char *)ss_mpi_sys_mmap(frm[0].video_frame.phys_addr[0], size);
//            memcpy(ptr_tx, user_addr, size);
//            ss_mpi_sys_munmap(user_addr, size);
//        }

        // if(strlen(ptr_rx) != 0) {
        //	if(1) {
        //	    region_tmp = function_draw_region();
        //	}

        //		memcpy(p, ptr_rx, 244);
        //		memset(ptr_rx, 0, 244);
        //		memset(ptr_tx, 0, 12441600);
        //
        //	for (int i = 0; i < 6; i++)
        //	{
        //	      printf("p[%d] is %d,",i, p[i]);
        //	}
        //	printf("\n");

#endif

        /* Draw rect */
        // ret = sample_common_svp_vgs_f
        // if (region_tmp_old.num != region_tmp.num)
        // {
            region_tmp = region_tmp_old;
        // }

        // if(random_int == 1) {
        if(region_tmp.num != 0)
        {
        ret = sample_common_svp_vgs_fill_rect_changecolor(&frm[0], &region_tmp, boxcolor);
        //sample_svp_check_failed_err_level_goto(ret, base_free, "sample_svp_vgs_fill_rect fail,Err(%#x)\n", ret); 
        }
        ret = ss_mpi_venc_send_frame(3, &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        if(ret != TD_SUCCESS)
            goto base_free;

	usleep(22 * 1000);
        // sample_svp_check_failed_err_level_goto(ret, base_free, "ss_mpi_venc_send_frame fail,Error(%#x)\n", ret);

        // free(stBitmap.data);
    change_idx:
        /* Change reference and current frame index */
        cur_idx = 1 - cur_idx;
    base_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);

    ext_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);
//    }

    /* destroy */
    ret = ot_ivs_md_destroy_chn(hld.md_chn);
    sample_svp_check_failed_trace(ret, SAMPLE_SVP_ERR_LEVEL_ERROR, "ot_ivs_md_destroy_chn fail,Err(%#x)\n", ret);
//    free(p);
    // free(stBitmap.data);
    return TD_NULL;
}

static td_s32 sample_ive_md_pause(td_void)
{
    printf("---------------press Enter key to exit!---------------\n");
    if (g_stop_signal == TD_TRUE)
    {
        if (g_md_thread != 0)
        {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_ivs_md_uninit(&(g_md_info));
        (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }

    (void)getchar();

    if (g_stop_signal == TD_TRUE)
    {
        if (g_md_thread != 0)
        {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }
    return TD_FALSE;
}
//***灯板开关***//
#define GPIO_BASE_ADDR 0x010230030
#define PAGE_SIZE 4096

#define GPIO_ON_CMD  "bspmm 0x010230030 0x1102"  // GPIO5_5 拉高
#define GPIO_OFF_CMD "bspmm 0x010230030 0x1100"  // GPIO5_5 拉低

volatile uint32_t *gpio_reg = NULL;

// 初始化 GPIO mmap
int init_gpio()
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem failed");
        return -1;
    }

    void *gpio_map = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE_ADDR & ~(PAGE_SIZE - 1));
    if (gpio_map == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return -1;
    }

    gpio_reg = (volatile uint32_t *)((char *)gpio_map + (GPIO_BASE_ADDR & (PAGE_SIZE - 1)));
    close(fd);
    return 0;
}

// 设置 GPIO 拉高
void set_gpio_high()
{
    if (gpio_reg) {
        *gpio_reg = 0x1102;
    }
}

// 设置 GPIO 拉低
void set_gpio_low()
{
    if (gpio_reg) {
        *gpio_reg = 0x1100;
    }
}

// 释放 GPIO mmap
void release_gpio()
{
    if (gpio_reg) {
        munmap((void *)((uintptr_t)gpio_reg & ~(PAGE_SIZE - 1)), PAGE_SIZE);
        gpio_reg = NULL;
    }
}

//**************//

//***自动对焦***//
extern td_u32 g_fv1;
int direction,change_time;
int last_fv, current_fv, next_fv, max_fv_value;
int g_focusReadyToStop = 0;
// 全局变量，用于保存当前的焦点百分比
float focus_percent = 0.0f;
// 线程停止标志
volatile int g_focusAutoStop = 0;
// focus_auto 线程函数
void *focus_auto_thread(void *arg)
{
    printf("[FocusAuto] Thread started.\n");

    while (!g_focusAutoStop)
    {
	 printf("max_fv_value is:%d , g_fv1 is :%d\n",max_fv_value,g_fv1);   
       float percent = (float)abs((int)(max_fv_value - g_fv1)) / max_fv_value * 100;
       // 将局部的 percent 赋值给全局变量 focus_percent
       focus_percent = percent;	
       printf("Percentage: %.2f%%\n", percent);
        if (percent > 7.0f || g_fv1 < 1000)
        {
	    max_fv_value = 0;
            fv_climb();
        }
        else
        {
            focus_auto_10();
	    g_focusReadyToStop = 1;
        }

        usleep(10 * 1000); // 100ms 避免过度占用 CPU
    }

    printf("[FocusAuto] Thread stopped.\n");
    return NULL;
}

pthread_t focus_thread;

// 启动线程
void start_focus_thread()
{
    g_focusAutoStop = 0;
    if (pthread_create(&focus_thread, NULL, focus_auto_thread, NULL) != 0)
    {
        perror("Failed to create focus_auto_thread");
    }
}

// 停止线程
void stop_focus_thread()
{
    g_focusAutoStop = 1;
    pthread_join(focus_thread, NULL);
}

void fv_climb(void)
{
    int step_index = 0;
    int stable_count = 0;
    int low_fv_count = 0;
    int delay_time_ms = 75;

    const int flat_threshold = 5;
    const int flat_limit = 10;
    const int low_fv_limit = 50;
    const int max_stage = 8;

    int max_fv_step_index = 0;

    int last_fv = 0;
    int current_fv = 0;
    int next_fv = 0;

    int direction_change_count = 0;

    direction = 1;  // 初始方向

    while (direction_change_count < max_stage) {

        if (g_focusAutoStop) {
            printf("[fv_climb] Received stop signal, exiting early.\n");
            break;
        }

        if (step_index > 0 && (last_fv - next_fv) > 10) {
            direction_change_count++;
            if (direction_change_count > 1) {
                focus_auto_1(); // 换向
            }
        }

        // 连续低FV值处理
        if (last_fv < 10 && next_fv < 10) {
            low_fv_count++;
            if (low_fv_count >= low_fv_limit) {
                printf("[fv_climb] FV too low, exiting.\n");
                break;
            }
        }

        // FV稳定检测（滞后区检测）
        if (step_index > 0 && abs(next_fv - last_fv) < flat_threshold) {
            stable_count++;
            printf("FV变化平稳，计数: %d（%d vs %d）\n", stable_count, next_fv, last_fv);

            if (stable_count >= flat_limit) {
                printf("[fv_climb] Detected plateau, reversing direction.\n");
                focus_auto_1();
		direction_change_count++;
                sleep_ms(500);
                stable_count = 0;
            }
        } else {
            stable_count = 0;
        }

        // 控制焦点位置推进
        switch (direction_change_count) {
            case 0: focus_auto_2(); break;
            case 1: focus_auto_3(); break;
            case 2: focus_auto_4(); break;
            case 3: focus_auto_5(); break;
            case 4: focus_auto_6(); break;
            case 5: focus_auto_7(); break;
            case 6: focus_auto_8(); break;
            case 7: focus_auto_9(); break;
	    case 8: focus_auto_10(); break;
        }

        // 模拟步进间隔
        sleep_ms(delay_time_ms);
        // 更新FV值
        next_fv = g_fv1;
        last_fv = current_fv;
        current_fv = next_fv;

        // 记录最大FV和其步数
        if (current_fv > max_fv_value) {
            max_fv_value = current_fv;
            max_fv_step_index = step_index;
        }

        printf("[fv_climb] step: %d, change_time: %d, FV: %d, maxFV: %d at step %d\n",
               step_index, direction_change_count, current_fv, max_fv_value, max_fv_step_index);

        step_index++;
    }

    //  自动对焦完成后的处理
    printf("[fv_climb] Auto focus completed. Best FV: %d at step %d\n",
           max_fv_value, max_fv_step_index);

    // printf("[fv_climb] Auto focus finished.\n");
}

//**************//



//************************UDP heartbeat server******************************//

//#define SEND_PORT 1789
//#define RECV_PORT 1790
//#define CLIENT_IP "192.168.2.103"
#define PACKET_SIZE 8
//#define TIMEOUT 5  // 超时时间（秒）

// 添加全局变量（在 config.c 中定义，在 main 文件中 extern 声明也可）
int SEND_PORT;
int RECV_PORT;
char CLIENT_IP[64];
char SERVER_IP[64];
int TIMEOUT;

void trim(char* str) {
    char *p = str;
    int l = strlen(p);
    while (l > 0 && (p[l - 1] == '\n' || p[l - 1] == '\r' || p[l - 1] == ' '))
        p[--l] = 0;
}

void load_config(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open config file");
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), file)) {
        char key[64], value[64];
        if (sscanf(line, "%[^=]=%s", key, value) == 2) {
            trim(value);
            if (strcmp(key, "SEND_PORT") == 0)
                SEND_PORT = atoi(value);
            else if (strcmp(key, "RECV_PORT") == 0)
                RECV_PORT = atoi(value);
            else if (strcmp(key, "CLIENT_IP") == 0)
                strncpy(CLIENT_IP, value, sizeof(CLIENT_IP));
            else if (strcmp(key, "SERVER_IP") == 0)
                strncpy(SERVER_IP, value, sizeof(SERVER_IP));
            else if (strcmp(key, "TIMEOUT") == 0)
                TIMEOUT = atoi(value);
        }
    }

    fclose(file);
}

unsigned char heartbeat_data[PACKET_SIZE] = {0xBF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFB};
time_t last_recv_time;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void print_packet(const char* prefix, const unsigned char* data) {
    printf("%s [ ", prefix);
    for (int i = 0; i < PACKET_SIZE; i++) {
        printf("%02X ", data[i]);
    }
    printf("]\n");
}

int is_valid_packet(const unsigned char* buf, ssize_t len) {
    return (len == PACKET_SIZE) &&
           (buf[0] == 0xBF) &&
           (buf[1] == 0xFF) &&
           (buf[6] == 0xFF) &&
           (buf[7] == 0xFB) &&
           (buf[3] == 0x00) &&
           (buf[4] == 0x00) &&
           (buf[5] == 0x00);
}

void* send_thread(void* arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SEND_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP)
    };
    bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

    struct sockaddr_in client_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SEND_PORT),
        .sin_addr.s_addr = inet_addr(CLIENT_IP)
    };

    while(1) {
        pthread_mutex_lock(&mutex);
        unsigned char packet[PACKET_SIZE];
        memcpy(packet, heartbeat_data, PACKET_SIZE);
        pthread_mutex_unlock(&mutex);

        sendto(sock, packet, PACKET_SIZE, 0,
              (struct sockaddr*)&client_addr, sizeof(client_addr));
     // print_packet("[Server] Sent:", packet);
        sleep(1);
    }
    close(sock);
    return NULL;
}

void* recv_thread(void* arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RECV_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP)
    };
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));

    while(1) {
        unsigned char buf[PACKET_SIZE];
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);

        ssize_t recv_len = recvfrom(sock, buf, PACKET_SIZE, 0,
                                   (struct sockaddr*)&client_addr, &len);

        if (is_valid_packet(buf, recv_len)) {
            pthread_mutex_lock(&mutex);
            heartbeat_data[2] = 0x01;
            last_recv_time = time(NULL);  // 更新最后接收时间
            pthread_mutex_unlock(&mutex);
        //  print_packet("[Server] Received:", buf);
        }
    }
    close(sock);
    return NULL;
}

void* check_thread(void* arg) {
    while(1) {
        sleep(1);  // 每秒检查一次
        pthread_mutex_lock(&mutex);
        time_t now = time(NULL);

        if (now - last_recv_time > TIMEOUT) {
            heartbeat_data[2] = 0x00;
      //      printf("[Server] Timeout!\n");
        }
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}

int udp_heartbeat_server() {
    load_config("/sharefs/config.ini");  // 加载配置文件
    printf("UDP Heartbeat Server\n");
    printf("SEND_PORT: %d\n", SEND_PORT);
    printf("RECV_PORT: %d\n", RECV_PORT);
    printf("CLIENT_IP: %s\n", CLIENT_IP);
    printf("SERVER_IP: %s\n", SERVER_IP);
    printf("TIMEOUT: %d\n", TIMEOUT);   
    pthread_t t1, t2, t3;
    last_recv_time = time(NULL);  // 初始化时间戳

    pthread_create(&t1, NULL, send_thread, NULL);
    pthread_create(&t2, NULL, recv_thread, NULL);
    pthread_create(&t3, NULL, check_thread, NULL);

    pthread_detach(t1);
    pthread_detach(t2);
    pthread_detach(t3);
    return 0;
} 

//************************UDP heartbeat server******************************//


/*TCP server */
//#define SERVER_IP "192.168.2.99"
#define SERVER_PORT 5477
#define BUFFER_SIZE (4 * 1024)
int uart_mcu_send;

//int server_fd, new_socket;
void send_file(int socket, const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (file == NULL) {
        perror("Failed to open file");
        return;
    }

    char buffer[BUFFER_SIZE];
    size_t bytes_read;
    char *msg =" \0";
    // 读取文件并通过socket发送
    while ((bytes_read = fread(buffer, 1, BUFFER_SIZE, file)) > 0) {
        if (send(socket, buffer, bytes_read, 0) < 0) {
            perror("Failed to send file");
            break;
        }
    }
    send(socket,msg,strlen(msg),0);
    fclose(file);
    printf("File sent successfully.\n");
}

void handle_snap(int socket_fd){

        sample_ivs_md_proc(&g_md_info);
        printf("Starting snap!!!\n");
        printf("RGB_file sending\n");
        send_file(socket_fd, "/sharefs/RGB_test.bgr");
}

//*****************//
// 形码检测类型位
#define DETECT_QRCODE   (1 << 0)
#define DETECT_BARCODE  (1 << 1)
#define DETECT_TEXT     (1 << 2)

// IPC 动作类型
typedef enum {
    IPC_ACTION_NONE = 0x00,
    IPC_ACTION_SNAPSHOT = 0x01,
    IPC_ACTION_CALIBRATE = 0x02,
    IPC_ACTION_FINETUNE = 0x04
} IPCAction;

// 放大/缩小方向
typedef enum {
    ZOOM_NONE = 0x00,
    ZOOM_IN   = 0x01,
    ZOOM_OUT  = 0x02
} ZoomDirection;

// 放大/缩小方向
typedef enum {
    FOCUS_NONE = 0x00,
    FOCUS_IN   = 0x01,
    FOCUS_OUT  = 0x02
} FocusDirection;

// 抓图指令解析结构
typedef struct {
    bool valid;

    uint8_t detect_type;
    bool detect_qrcode;
    bool detect_barcode;
    bool detect_text;

    IPCAction action;

    ZoomDirection zoom_dir;
    uint32_t zoom_factor;

    FocusDirection focus_dir;
    uint32_t focus_factor;
} OCRCommand;

// 辅助函数：从高字节序4字节转uint32_t
uint32_t parse_uint32_be(uint8_t* bytes) {
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

// 主解析函数
OCRCommand parse_ocr_command(const uint8_t cmd[16]) {
    OCRCommand result = {0};

    // 校验头尾
    if (cmd[0] != 0xcf || cmd[1] != 0xff || cmd[14] != 0xff || cmd[15] != 0xfc) {
        result.valid = false;
        return result;
    }

    result.valid = true;

    // 检测类型
    result.detect_type = cmd[2];
    result.detect_qrcode  = (cmd[2] & DETECT_QRCODE) != 0;
    result.detect_barcode = (cmd[2] & DETECT_BARCODE) != 0;
    result.detect_text    = (cmd[2] & DETECT_TEXT) != 0;

    // IPC动作
    result.action = (IPCAction)cmd[3];

    // 变倍
    result.zoom_dir = (ZoomDirection)cmd[4];
    result.zoom_factor = parse_uint32_be(&cmd[5]);

    // 变焦
    result.focus_dir = (FocusDirection)cmd[9];
    result.focus_factor = parse_uint32_be(&cmd[10]);

    return result;
}

// 示例调用
void print_ocr_command(const OCRCommand* cmd) {
    if (!cmd->valid) {
        printf("无效指令！校验失败。\n");
        return;
    }

    printf("检测类型：\n");
    if (cmd->detect_qrcode)  printf("  - 二维码\n");
    if (cmd->detect_barcode) printf("  - 一维码\n");
    if (cmd->detect_text)    printf("  - 字符\n");

    printf("IPC动作：");
    switch (cmd->action) {
        case IPC_ACTION_NONE: printf("无操作\n"); break;
        case IPC_ACTION_SNAPSHOT: printf("抓拍图像\n"); break;
        case IPC_ACTION_CALIBRATE: printf("对焦标定\n"); break;
    }

    printf("变倍方向：%d，系数：%u\n", cmd->zoom_dir, cmd->zoom_factor);
    printf("变焦方向：%d，系数：%u\n", cmd->focus_dir, cmd->focus_factor);
}

// 毫秒级 sleep 函数
void sleep_ms(int milliseconds) {
    struct timespec ts;

    // 秒部分
    ts.tv_sec = milliseconds / 1000;

    // 纳秒部分
    ts.tv_nsec = (milliseconds % 1000) * 1000000;

    // 调用 nanosleep
    nanosleep(&ts, NULL);
}

//封装变倍和变焦函数
void handle_zoom_action(ZoomDirection zoom_dir, uint32_t zoom_factor) {
	 printf("变倍方向：%d，系数：%u\n", zoom_dir,zoom_factor);
	switch (zoom_dir) {
        case ZOOM_IN:
            zoom_test_1();
         // usleep(zoom_factor);  // 延迟 zoom_factor 微秒
	    sleep_ms(zoom_factor);
	    zoom_test_stop();
	    printf("zoom in!!!\n");
            break;

        case ZOOM_OUT:
            zoom_test_2();
         // usleep(zoom_factor);  // 延迟 zoom_factor 微秒
	    sleep_ms(zoom_factor);
	    zoom_test_stop();
	    printf("zoom out!!!\n");
            break;

        default:
            // 无动作
            break;
    }
}

void handle_focus_action(FocusDirection focus_dir, uint32_t focus_factor) {
	    printf("变焦方向：%d，系数：%u\n", focus_dir, focus_factor);
	switch (focus_dir) {
        case FOCUS_IN:
            focus_test_1();
           // usleep(focus_factor);  // 延迟 focus_factor 微秒
	   sleep_ms(focus_factor);
	    focus_test_stop();
	    printf("focus in!!!\n");
            break;

        case FOCUS_OUT:
            focus_test_2();
          //  usleep(focus_factor);  // 延迟 focus_factor 微秒
	    sleep_ms(focus_factor);
	    focus_test_stop();
	    printf("focus out!!!\n");
            break;

        default:
            // 无动作
            break;
    }
}
//封装OCR镜头复位函数
void handle_ocr_reset()
{
  zoom_test_2();
//  usleep(1591680);
  sleep_ms(16000);
  zoom_test_stop();
  sleep(2);
  focus_test_2();
//  usleep(462608);
  sleep_ms(4000);
  focus_test_stop();
  sleep(1);

}

// 指令解析及调用
void handle_ocr_command(int socket_fd, const OCRCommand* cmd) {
    static int consecutive_small_focus = 0;  // 用于追踪连续小焦点值的计数器
    struct timeval start_time;  // 记录对焦开始的时间
    struct timeval current_time;  // 当前时间，用于检查超时
    long elapsed_time;  // 计算已经过去的时间（秒）
    if (!cmd->valid) {
        printf("无效指令！校验失败。\n");
        return;
    }

    printf("检测类型：\n");
    if (cmd->detect_qrcode)  printf("  - 二维码\n");
    if (cmd->detect_barcode) printf("  - 一维码\n");
    if (cmd->detect_text)    printf("  - 字符\n");

    printf("IPC动作：");
    switch (cmd->action) {
        case IPC_ACTION_NONE: printf("无操作\n"); break;
        case IPC_ACTION_SNAPSHOT: 
		printf("抓拍图像\n");
		handle_snap(socket_fd);
	       	break;
        case IPC_ACTION_CALIBRATE: 
		printf("对焦标定\n");
		//复位到原点
		handle_ocr_reset();
		// 执行变倍与变焦动作
            	handle_zoom_action(cmd->zoom_dir, cmd->zoom_factor);
//		sleep(2);
//            	handle_focus_action(cmd->focus_dir, cmd->focus_factor);
//		sleep(1);
		sdk_af_lens_init();
		printf("g_focusReadyToStop_1 is: %d\n",g_focusReadyToStop);
		start_focus_thread();
		printf("g_focusReadyToStop_2 is: %d\n",g_focusReadyToStop);
	            // 记录对焦开始的时间
            	gettimeofday(&start_time, NULL);
            	// 这里开始检查 `focus_percent` 值并计数
            	while (!g_focusAutoStop) {
                // 获取当前时间
                gettimeofday(&current_time, NULL);

                // 计算已经过去的时间（单位：秒）
                elapsed_time = (current_time.tv_sec - start_time.tv_sec);

                printf("当前焦点百分比: %.2f%%\n", focus_percent);

                // 如果 `focus_percent` 在 0% 到 10% 之间，计数器加 1
                if (focus_percent > 0.0f && focus_percent < 10.0f) {
                    consecutive_small_focus++;
                    printf("连续小焦点次数: %d\n", consecutive_small_focus);
                } else {
                    // 否则，重置计数器
                    consecutive_small_focus = 0;
                }
                // 如果连续小焦点次数达到 20 次，停止对焦
                if (consecutive_small_focus >= 20) {
                    printf("[FocusAuto] 因为连续 20 次小焦点值，停止自动对焦。\n");
                    g_focusAutoStop = 1;
		    consecutive_small_focus = 0;
                    break;
                }
                // 如果超过 20 秒且还没有达到 20 次小焦点值，停止对焦
                if (elapsed_time >= 20) {
                    printf("[FocusAuto] 超过 20 秒没有达到连续 20 次小焦点，停止自动对焦。\n");
                    g_focusAutoStop = 1;
		    consecutive_small_focus = 0;
                    break;
                }
                usleep(10 * 1000); // 100ms，避免过度占用 CPU
            }	
		stop_focus_thread();
		sleep(4);
		handle_snap(socket_fd);
		g_focusReadyToStop = 0;
		break;
	case IPC_ACTION_FINETUNE:
		printf("对焦微调\n");
		handle_zoom_action(cmd->zoom_dir, cmd->zoom_factor);
                sleep(2);
                handle_focus_action(cmd->focus_dir, cmd->focus_factor);
                sleep(1);
                handle_snap(socket_fd);
                break;
    }
}

void handle_command_ef(int socket_fd, const unsigned char *buffer) {
    if (buffer[1] == 0x01) {
        sample_ivs_md_proc(&g_md_info);
        printf("RGB_file sending\n");
        send_file(socket_fd, "/sharefs/RGB_test.bgr");
    }

    // 对焦控制
    if (buffer[2] == 0x01) {
       sdk_af_lens_init(NULL);
       // uart_mcu_send = 0x01;
        printf("focus start!\n");
	start_focus_thread();
    } else if (buffer[2] == 0x00) {
        sdk_af_lens_exit();
       // uart_mcu_send = 0x00;
       stop_focus_thread();
        focus_auto_10();
        printf("focus stop!\n");
    }

}

void handle_command_df(int socket_fd, const unsigned char *buffer)
{
    // 变倍控制
    switch (buffer[1]) {
        case 0x01:
            zoom_test_1();
            printf("zoom in!\n");
            break;
        case 0x02:
            zoom_test_2();
            printf("zoom out!\n");
            break;
        default:
            zoom_test_stop();
            printf("zoom stop!\n");
            break;
    }
   sleep_ms(20);
    // 微调对焦
    switch (buffer[2]) {
        case 0x01:
            focus_test_1();
            sleep_ms(16);
            focus_test_stop();
            printf("focus +\n");
            break;
        case 0x02:
            focus_test_2();
            sleep_ms(16);
            focus_test_stop();
            printf("focus -\n");
            break;
    }
}

void handle_command_3f(int socket_fd, const unsigned char *buffer) {
    // 灯板控制
    init_gpio();
    if (buffer[1] == 0x01) {
	set_gpio_high();
      printf("灯板开启!\n");
    } else if (buffer[2] == 0x00) {
	set_gpio_low();
      printf("灯板关闭!\n");
    }
    release_gpio();

}

void handle_command_af(int socket_fd, const unsigned char *buffer) {
    // IRCUT控制
    if (buffer[1] == 0x01) {
        ircut_on();
    } else if (buffer[1] == 0x00) {
        ircut_off();
    }
}

//*****************//

void *tcp_server_tmp(){
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    unsigned char buffer[BUFFER_SIZE];

    // 创建 socket 文件描述符
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // 绑定 IP 和端口
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    // 监听连接请求
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }

    printf("Server listening on %s:%d\n", INADDR_ANY, SERVER_PORT);

while (1) {
        // 接受客户端连接
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
            perror("Accept failed");
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        printf("Connection accepted from %s:%d\n", inet_ntoa(address.sin_addr), ntohs(address.sin_port));

        // 处理客户端请求
   while (1) {
    ssize_t bytes_received = recv(new_socket, buffer, BUFFER_SIZE, 0);

    if (bytes_received == 0) {
        printf("Client disconnected\n");
        close(new_socket);
        break;
    } else if (bytes_received < 0) {
        perror("Receive failed");
        close(new_socket);
        break;
    }

    printf("Received %zd bytes.\n", bytes_received);
    for (int i = 0; i < bytes_received; i++) {
        printf("%02x ", (unsigned char)buffer[i]);
    }
    printf("\n");

    if (bytes_received < 1) continue;
               switch (buffer[0]) {
                case 0xef:
                    handle_command_ef(new_socket, buffer);
                    break;
                case 0xcf:
                    {
                        OCRCommand parsed = parse_ocr_command(buffer);
                        handle_ocr_command(new_socket, &parsed);
			break;
                    }
		case 0xdf:
		    handle_command_df(new_socket, buffer);
                    break;
		case 0x3f:
		    handle_command_3f(new_socket, buffer);
		    break;
		case 0xaf:
		    handle_command_af(new_socket, buffer);
		    break;
                default:
                    printf("Unknown command: 0x%02x\n", buffer[0]);
                    break;     
  	 }
      }
    }

    // 关闭服务器
    close(server_fd);
}

td_void sample_ive_md(td_void)
{
    ot_size pic_size;
    hi_pic_size pic_type = PIC_1080P;
   // hi_pic_size pic_type = PIC_2688X1520;
    td_s32 ret;
    p= malloc(sizeof(Total_result));
    (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));
    /*
     * step 1: start vi vpss venc vo
     */
    ret = sample_common_svp_start_vi_vpss_venc_vo(&g_vi_config, &g_md_switch, &pic_type);
    ot_dis_cfg dis_cfg;
    ss_mpi_vi_get_chn_dis_cfg(0, 0, &dis_cfg);
    dis_cfg.motion_level = 1;
    dis_cfg.mode = 1;

    ss_mpi_vi_set_chn_dis_cfg(0, 0, &dis_cfg);
    ot_dis_attr dis_attr;
    ss_mpi_vi_get_chn_dis_attr(0, 0, &dis_attr);
    dis_attr.enable = TD_TRUE;
    dis_attr.gdc_bypass = TD_FALSE;
    ss_mpi_vi_set_chn_dis_attr(0, 0, &dis_attr);
    // sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                            //    "Error(%#x),sample_common_svp_start_vi_vpss_venc_vo failed!\n", ret);
  

 
    pic_type = PIC_1080P;
   // pic_type = PIC_2688X1520;
    ret = sample_comm_sys_get_pic_size(pic_type, &pic_size);
    // sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                            //    "Error(%#x),sample_comm_sys_get_pic_size failed!\n", ret);
    /*
     * step 2: Init Md
     */
    ret = sample_ivs_md_init(&g_md_info, pic_size.width, pic_size.height);
    // sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                            //    " Error(%#x),sample_ivs_md_init failed!\n", ret);
    g_stop_signal = TD_FALSE;
    /*
     * step 3: Create work thread
     */

//    pthread_t osd_task ;
//    pthread_create(&osd_task, NULL, osd_ttf_task, NULL);
//    pthread_detach(osd_task);

    
    venc_audio_start();
     
    memory_tmp();
   
   // tcp_server_tmp();
    pthread_t tcp_server_task;
    pthread_create(&tcp_server_task, NULL, tcp_server_tmp, NULL);
    pthread_detach(tcp_server_task);

    udp_heartbeat_server();
    
//    RGN_AddOsdToVenc();
//    pthread_t bitmap_update_t ;
//    pthread_create(&bitmap_update_t, NULL, bitmap_update, NULL);
//    pthread_detach(bitmap_update_t);  

    // sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "set thread name failed!\n");
  //  ret = pthread_create(&g_md_thread, 0, sample_ivs_md_proc, (td_void *)&g_md_info);

    // sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "pthread_create failed!\n");
    // system("cd /root/YOLOV3_rtsp_opencv_ffmpeg/out && ./main");

    //    ret = sample_ive_md_pause();


//     while (1)
//     {
//     	usleep(500 *1000);
//     }
//     sample_svp_check_exps_return_void(ret == TD_TRUE, SAMPLE_SVP_ERR_LEVEL_ERROR, "md exist!\n");
//     g_stop_signal = TD_TRUE;
//     pthread_join(g_md_thread, TD_NULL);
//     g_md_thread = 0;

//     sample_ivs_md_uninit(&(g_md_info));
//     (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

// end_md_0:
//     g_md_thread = 0;
//     g_stop_signal = TD_TRUE;
//     sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
//     return;
}

/*
 * function : Md sample signal handle
 */
td_void sample_ive_md_handle_sig(td_void)
{
    g_stop_signal = TD_TRUE;
}
