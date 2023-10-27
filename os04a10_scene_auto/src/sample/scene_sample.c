/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <limits.h>
#include <securec.h>

#include "ot_scenecomm.h"
#include "scene_loadparam.h"
#include "ot_scene_setparam.h"

ot_scene_param g_scene_param;

static hi_bool g_scene_sample_exit = HI_FALSE;

#ifndef __LITEOS__
static hi_void sample_scene_handle_signal(hi_s32 signo)
{
    if (signo == SIGINT || signo == SIGTERM) {
        g_scene_sample_exit = HI_TRUE;
    }
}

static hi_void sample_register_handler(hi_void)
{
    struct sigaction action = { 0 };
    action.sa_flags = 0;
    action.sa_handler = sample_scene_handle_signal;
    sigaction(SIGINT, &action, HI_NULL);
    sigaction(SIGTERM, &action, HI_NULL);
}
#endif

static hi_s32 sample_prompt_and_input_para(hi_char *input, hi_u32 input_len, hi_s32 *choice)
{
    printf("1.scene start\n");
    printf("2.scene pause and reset media route\n");
    printf("3.scene resume and set a new media type\n");
    printf("4.exit the sample\n");

    *choice = -1;
    (hi_void)fgets(input, input_len, stdin);
    hi_s32 ret = sscanf_s(input, "%d", choice);
    if (ret == -1) {
        printf("choice error\n");
        return HI_FAILURE;
    }
    return HI_SUCCESS;
}

static hi_s32 sample_create_para_and_set_mode(const hi_char *dir_name, hi_s32 choice, ot_scene_video_mode *video_mode)
{
    hi_s32 ret;
    //printf("When We Start SceneAuto, we neen to set video Pipe mode\n");
    //printf("Please input videomode index, without which we couldn't work  effectively.\n");
    //hi_s32 choice = -1;

    //(hi_void)fgets(input, input_len, stdin);
    //hi_s32 ret = sscanf_s(input, "%d", &choice);
    //if (ret == -1) {
    //    printf("videomode error\n");
    //   return HI_FAILURE;
    //}
    //printf("videomode type has already been input.\n");

    ret = ot_scene_create_param(dir_name, &g_scene_param, video_mode);
    if (ret != HI_SUCCESS) {
        printf("SCENETOOL_CreateParam failed\n");
        return HI_FAILURE;
    }

    ret = ot_scene_init(&g_scene_param);
    if (ret != HI_SUCCESS) {
        printf("ot_scene_init failed\n");
        return HI_FAILURE;
    }

    ot_scenecomm_expr_true_return(choice >= SCENE_MAX_VIDEOMODE, HI_FAILURE);
    ret = ot_scene_set_scene_mode(&(video_mode->video_mode[choice]));
    if (ret != HI_SUCCESS) {
        printf("HI_SRDK_SCENEAUTO_Start failed\n");
        return HI_FAILURE;
    }
    printf("The sceneauto is started already.\n");
    return HI_SUCCESS;
}

static hi_s32 scene_sample_scene_pause(hi_void)
{
    hi_s32 ret = ot_scene_pause(HI_TRUE);
    if (ret != HI_SUCCESS) {
        printf("ot_scene_pause failed\n");
        return HI_FAILURE;
    }
    printf("The sceneauto is pause.\n");
    printf("If you want to change videomode, please enter 3.\n");
    return HI_SUCCESS;
}

static hi_s32 sample_scene_set_video_mode(hi_char *input, hi_u32 input_len, const ot_scene_video_mode *video_mode)
{
    printf("Please input videomode index, without which we couldn't work  effectively.\n");
    hi_s32 choice = -1;
    (hi_void)fgets(input, input_len, stdin);
    hi_s32 ret = sscanf_s(input, "%d", &choice);
    if (ret == -1) {
        printf("videomode error\n");
        return HI_FAILURE;
    }
    printf("videomode type has already been input.\n");

    ot_scenecomm_expr_true_return(choice >= SCENE_MAX_VIDEOMODE, HI_FAILURE);
    ret = ot_scene_set_scene_mode(&(video_mode->video_mode[choice]));
    if (ret != HI_SUCCESS) {
        printf("HI_SRDK_SCENEAUTO_Start failed\n");
        return HI_FAILURE;
    }

    ret = ot_scene_pause(HI_FALSE);
    if (ret != HI_SUCCESS) {
        printf("HI_SCENE_Resume failed\n");
        return HI_FAILURE;
    }
    printf("The sceneauto is started already.\n");

    return HI_SUCCESS;
}

static hi_void sample_scene_deinit(hi_void)
{
    hi_s32 ret = ot_scene_deinit();
    if (ret != HI_SUCCESS) {
        printf("ot_scene_deinit failed\n");
        return;
    }
    printf("The scene sample is end.\n");
}

#ifdef __LITEOS__
hi_s32 app_main(hi_s32 argc, hi_char *argv[])
#else
hi_s32 main(hi_s32 argc, hi_char *argv[])
#endif
{
    hi_s32 ret, choice, videoIndex;
    hi_char input[10]; /* 10 max input char */
    ot_scene_video_mode video_mode;

    ot_scenecomm_check_pointer_return(argv, HI_FAILURE);
    if (argc < 2 || strncmp(argv[1], "-h", 3) == 0) { /* need 2 args, compare max 3 bytes with -h */
        printf("Usage : %s <inidir>\n\t\tfor example :./sample_scene ./param/sensor_imx290\n", argv[0]);
        return HI_SUCCESS;
    }

    set_dir_name(argv[1]);

    videoIndex = 0;
    if(argc > 2) {
        videoIndex = atoi(argv[2]);
    }
    printf("videoIndex = %d\n", videoIndex);

#ifndef __LITEOS__
    sample_register_handler();
#endif

    printf("init success\n");

    ret = sample_create_para_and_set_mode(argv[1], videoIndex, &video_mode);
    while (HI_FALSE == g_scene_sample_exit){
        sleep(1);
    }

#if 0
    choice = 1;
    while (g_scene_sample_exit == HI_FALSE) {    
        //ret = sample_prompt_and_input_para(input, sizeof(input), &choice);
        //if (ret != HI_SUCCESS) {
        //    continue;
        //}
        
        switch (choice) {
            case 1: /* user input 1 */
                ret = sample_create_para_and_set_mode(argv[1], videoIndex, &video_mode);
                break;
            case 2: /* user input 2 */
                ret = scene_sample_scene_pause();
                break;
            case 3: /* user input 3 */
                ret = sample_scene_set_video_mode(input, sizeof(input), &video_mode);
                break;
            case 4: /* user input 4 */
                sample_scene_deinit();
                sleep(1);
                return HI_SUCCESS;
            default:
                ret = HI_SUCCESS;
                printf("unknown input\n");
                break;
        }
        ot_scenecomm_check_return(ret, HI_FAILURE);

        sleep(1);
    }
#endif

    sample_scene_deinit();
    g_scene_sample_exit = HI_FALSE;
    return HI_SUCCESS;
}
