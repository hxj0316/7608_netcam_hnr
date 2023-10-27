/*!
*****************************************************************************
** \file        src/algo/af_params/Foctek_D14_02812IR_distance_table.c
**
** \brief
**
** \attention   THIS SAMPLE CODE IS PROVIDED AS IS. GOKE MICROELECTRONICS
**              ACCEPTS NO RESPONSIBILITY OR LIABILITY FOR ANY ERRORS OR
**              OMMISSIONS
**
** (C) Copyright 2012-2015 by GOKE MICROELECTRONICS CO.,LTD
**
*****************************************************************************
*/
#include "basetypes.h"
#include "gk_isp.h"
#include "gk_isp3a_api.h"
#include "isp3a_lens.h"

//*****************************************************************************
//*****************************************************************************
//** Local Defines
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//** Local structures
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//** Global Data
//*****************************************************************************
//*****************************************************************************
/*    This table covers physical distance(mm) to focus pulse
*    ranges from 50000cm to 1cm
*    dist_idx = 50000/dist(cm)
*    {wide, zoom_pos1, zoom_pos2, ... , zoom_pos N, tele}
*
*    index 1        500m
*     index 5        10m
*    index 100    5m
*    index 167    3m
*    index 250    2m
*    index 333    1.5m
*    index 417    1.2m
*    index 600    1m
*    index 625    0.8m
*    index 714    0.7m
*    index 833    0.6m
*    index 1000    0.5m
*    index 1111    0.45m
*
*    pulse = (a>>20) * dist_idx^2 + (b>>15) * dist_idx + (c>>2)
*/

/*
* a: less than 512
* b:
* c:
*/
//gc2053+塑料底座+yutong_xx
zoom_t YuTong_YT30031FB_DISTANCE_MAP[ZOOM_STEP] = {
 
#if 0 //标定初始化(focus length=3300; zoom length= 2312(大于))
    {0, -32768, 0, 0, 0}
                      ,        // zp_0
    {0, -32768, 672, 136, 0}
                      ,        // zp_1
    {0, -32768, 1076, 272, 0}
                      ,        // zp_2
    {0, -32768, 1388, 408, 0}
                      ,        // zp_3
    {0, -32768, 1680, 544, 0}
                      ,        // zp_4
    {0, -32768, 1892, 680, 0}
                      ,        // zp_5
    {0, -32768, 2080, 816, 0}
                      ,        // zp_6
    {0, -32768, 2232, 952, 0}
                      ,        // zp_7
    {0, -32768, 2352, 1088, 0}
                      ,        // zp_8
    {0, -32768, 2448, 1224, 0}
                      ,        // zp_9
    {0, -32768, 2528, 1360, 0}
                      ,        // zp_10
    {0, -32768, 2592, 1496, 0}
                      ,        // zp_11
    {0, -32768, 2640, 1632, 0}
                      ,        // zp_12
    {0, -32768, 2676, 1768, 0}
                      ,        // zp_13
    {0, -32768, 2700, 1904, 0}
                      ,        // zp_14
    {0, -32768, 2716, 2040, 0}
                      ,        // zp_15
    {0, -32768, 2720, 2176, 0}
                      ,                   
#endif 

#if 0 //gc2053(50即-1580，也可用-920）
    {0, -920, 2756, 0, 0}
                      ,        // zp_0
    {0, -920, 720, 136, 0}
                      ,        // zp_1
    {0, -920, 1124, 272, 0}
                      ,        // zp_2
    {0, -920, 1448, 408, 0}
                      ,        // zp_3
    {0, -920, 1720, 544, 0}
                      ,        // zp_4
    {0, -920, 1940, 680, 0}
                      ,        // zp_5
    {0, -920, 2128, 816, 0}
                      ,        // zp_6
    {0, -920, 2276, 952, 0}
                      ,        // zp_7
    {0, -920, 2396, 1088, 0}
                      ,        // zp_8
    {0, -920, 2496, 1224, 0}
                      ,        // zp_9
    {0, -920, 2576, 1360, 0}
                      ,        // zp_10
    {0, -920, 2636, 1496, 0}
                      ,        // zp_11
    {0, -920, 2684, 1632, 0}
                      ,        // zp_12
    {0, -920, 2720, 1768, 0}
                      ,        // zp_13
    {0, -920, 2744, 1904, 0}
                      ,        // zp_14
    {0, -920, 2756, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2768,  2176,  0}
                      , 

#endif 
    #if 0 //gc2053(50即-1580，也可用-920）zoom正向，focus正向。放大清楚
    {0, -920, 876+62, 0, 0}
                      ,        // zp_0
    {0, -920, 463+40, 136, 0}
                      ,        // zp_1
    {0, -920, 876+62, 272, 0}
                      ,        // zp_2
    {0, -920, 1204+80, 408, 0}
                      ,        // zp_3
    {0, -920, 1472+85, 544, 0}
                      ,        // zp_4
    {0, -920, 1678+94, 680, 0}
                      ,        // zp_5
    {0, -920, 1870+94, 816, 0}
                      ,        // zp_6
    {0, -920, 2032+89, 952, 0}
                      ,        // zp_7
    {0, -920, 2143+94, 1088, 0}
                      ,        // zp_8
    {0, -920, 2242+94, 1224, 0}
                      ,        // zp_9
    {0, -920, 2322+94, 1360, 0}
                      ,        // zp_10
    {0, -920, 2382+94, 1496, 0}
                      ,        // zp_11
    {0, -920, 2430+89, 1632, 0}
                      ,        // zp_12
    {0, -920, 2466+89, 1768, 0}
                      ,        // zp_13
    {0, -920, 2490+89, 1904, 0}
                      ,        // zp_14
    {0, -920, 2510+85, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2768,  2176,  0}
                      , 

#endif 
    #if 0 //gc2053(50即-1580，也可用-920）zoom正向，focus反向。
    {0, -920, 876, 0, 0}
                      ,        // zp_0
    {0, -920, 463, 136, 0}
                      ,        // zp_1
    {0, -920, 876, 272, 0}
                      ,        // zp_2
    {0, -920, 1204, 408, 0}
                      ,        // zp_3
    {0, -920, 1472, 544, 0}
                      ,        // zp_4
    {0, -920, 1678, 680, 0}
                      ,        // zp_5
    {0, -920, 1870, 816, 0}
                      ,        // zp_6
    {0, -920, 2032, 952, 0}
                      ,        // zp_7
    {0, -920, 2143, 1088, 0}
                      ,        // zp_8
    {0, -920, 2242, 1224, 0}
                      ,        // zp_9
    {0, -920, 2322, 1360, 0}
                      ,        // zp_10
    {0, -920, 2382, 1496, 0}
                      ,        // zp_11
    {0, -920, 2430, 1632, 0}
                      ,        // zp_12
    {0, -920, 2466, 1768, 0}
                      ,        // zp_13
    {0, -920, 2490, 1904, 0}
                      ,        // zp_14
    {0, -920, 2510, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2768,  2176,  0}
                      , 

#endif 
#if 0 //gc2053(50即-1580，也可用-920）zoom反向，focus反向。缩小清楚
    {0, -920, 2504, 0, 0}
                      ,        // zp_0
    {0, -920, 720, 136, 0}
                      ,        // zp_1
    {0, -920, 1029, 272, 0}
                      ,        // zp_2
    {0, -920, 1313, 408, 0}
                      ,        // zp_3
    {0, -920, 1562, 544, 0}
                      ,        // zp_4
    {0, -920, 1751, 680, 0}
                      ,        // zp_5
    {0, -920, 1928, 816, 0}
                      ,        // zp_6
    {0, -920, 2063, 952, 0}
                      ,        // zp_7
    {0, -920, 2178, 1088, 0}
                      ,        // zp_8
    {0, -920, 2265, 1224, 0}
                      ,        // zp_9
    {0, -920, 2336, 1360, 0}
                      ,        // zp_10
    {0, -920, 2397, 1496, 0}
                      ,        // zp_11
    {0, -920, 2436, 1632, 0}
                      ,        // zp_12
    {0, -920, 2463, 1768, 0}
                      ,        // zp_13
    {0, -920, 2492, 1904, 0}
                      ,        // zp_14
    {0, -920, 2504, 2040, 0}   //focus先+94再-94后反向，zoom初始状态处于反向
                      ,        // zp_15
    {0,  -920, 2768,  2176,  0}
                      , 

#endif
#if 0 //gc2053(50即-1580，也可用-920）zoom反向，focus正向
    {0, -920, 2756, 0, 0}
                      ,        // zp_0
    {0, -920, 720, 136, 0}
                      ,        // zp_1
    {0, -920, 1029+80, 272, 0}
                      ,        // zp_2
    {0, -920, 1313+89, 408, 0}
                      ,        // zp_3
    {0, -920, 1562+94, 544, 0}
                      ,        // zp_4
    {0, -920, 1751+101, 680, 0}
                      ,        // zp_5
    {0, -920, 1928+98, 816, 0}
                      ,        // zp_6
    {0, -920, 2063+98, 952, 0}
                      ,        // zp_7
    {0, -920, 2178+98, 1088, 0}
                      ,        // zp_8
    {0, -920, 2265+98, 1224, 0}
                      ,        // zp_9
    {0, -920, 2336+98, 1360, 0}
                      ,        // zp_10
    {0, -920, 2397+98, 1496, 0}
                      ,        // zp_11
    {0, -920, 2436+94, 1632, 0}
                      ,        // zp_12
    {0, -920, 2463+98, 1768, 0}
                      ,        // zp_13
    {0, -920, 2492+98, 1904, 0}
                      ,        // zp_14
    {0, -920, 2504+94, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2768+94,  2176,  0}
                      , 

#endif
#if 0 //gc2053(50即-1580，也可用-920）zoom正向，focus反向。加固定偏移zoom 70，focus 90
    {0, -920, 1041, 0, 0}
                      ,        // zp_0
    {0, -920, 720, 136, 0}
                      ,        // zp_1
    {0, -920, 1041, 272, 0}
                      ,        // zp_2
    {0, -920, 1328, 408, 0}
                      ,        // zp_3
    {0, -920, 1574, 544, 0}
                      ,        // zp_4
    {0, -920, 1771, 680, 0}
                      ,        // zp_5
    {0, -920, 1945, 816, 0}
                      ,        // zp_6
    {0, -920, 2084, 952, 0}
                      ,        // zp_7
    {0, -920, 2195, 1088, 0}
                      ,        // zp_8
    {0, -920, 2280, 1224, 0}
                      ,        // zp_9
    {0, -920, 2351, 1360, 0}
                      ,        // zp_10
    {0, -920, 2406, 1496, 0}
                      ,        // zp_11
    {0, -920, 2445, 1632, 0}
                      ,        // zp_12
    {0, -920, 2476, 1768, 0}
                      ,        // zp_13
    {0, -920, 2500, 1904, 0}
                      ,        // zp_14
    {0, -920, 2512, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2768+94,  2176,  0}
                      , 

#endif

#if 1 //gc2053(50即-1580，也可用-920）zoom正向，focus正向。无偏移。先测zoom_revser = 64
    {0, -920, 0, 0, 0}
                      ,        // zp_0
    {0, -920, 720, 136, 0}
                      ,        // zp_1
    {0, -920, 861+76, 272, 0}
                      ,        // zp_2
    {0, -920, 1183+89, 408, 0}
                      ,        // zp_3
    {0, -920, 1456+89, 544, 0}
                      ,        // zp_4
    {0, -920, 1666+98, 680, 0}
                      ,        // zp_5
    {0, -920, 1854+98, 816, 0}
                      ,        // zp_6
    {0, -920, 2015+98, 952, 0}
                      ,        // zp_7
    {0, -920, 2133+101, 1088, 0}
                      ,        // zp_8
    {0, -920, 2229+101, 1224, 0}
                      ,        // zp_9
    {0, -920, 2313+98, 1360, 0}
                      ,        // zp_10
    {0, -920, 2377+98, 1496, 0}
                      ,        // zp_11
    {0, -920, 2420+98, 1632, 0}
                      ,        // zp_12
    {0, -920, 2455+98, 1768, 0}
                      ,        // zp_13
    {0, -920, 2484+94, 1904, 0}
                      ,        // zp_14
    {0, -920, 2500+94, 2040, 0}
                      ,        // zp_15
    {0,  -920, 2510+94,  2176,  0}
                      , 

#endif
                                // zp_16
    {0, 0, 0, 0, 0}
    ,                           // zp_17
    {0, 0, 0, 0, 0}
    ,                           // zp_18
    {0, 0, 0, 0, 0}
    ,                           // zp_19
    {0, 0, 0, 0, 0}
    ,                           // zp_20
    {0, 0, 0, 0, 0}
    ,                           // zp_21
    {0, 0, 0, 0, 0}
    ,                           // zp_22
    {0, 0, 0, 0, 0}
    ,                           // zp_23
    {0, 0, 0, 0, 0}
    ,                           // zp_24
    {0, 0, 0, 0, 0}
    ,                           // zp_25
    {0, 0, 0, 0, 0}
    ,                           // zp_26
    {0, 0, 0, 0, 0}
    ,                           // zp_27
    {0, 0, 0, 0, 0}
    ,                           // zp_28
    {0, 0, 0, 0, 0}
    ,                           // zp_29
    {0, 0, 0, 0, 0}
    ,                           // zp_30
    {0, 0, 0, 0, 0}
    ,                           // zp_31
};


//*****************************************************************************
//*****************************************************************************
//** Local Data
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//** Local Functions Declaration
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//** API Functions
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//** Local Functions
//*****************************************************************************
//*****************************************************************************
