/*-------------------------------------------------------------------*/
/*  Copyright(C) 2011-2017 by OMRON Corporation                      */
/*  All Rights Reserved.                                             */
/*                                                                   */
/*   This source code is the Confidential and Proprietary Property   */
/*   of OMRON Corporation.  Any unauthorized use, reproduction or    */
/*   transfer of this software is strictly prohibited.               */
/*                                                                   */
/*-------------------------------------------------------------------*/
/*
   Face Detection Library
*/
#ifndef OKAODTAPI_H__
#define OKAODTAPI_H__

#define OKAO_API
#include    "OkaoCoAPI.h"
#include    "DetectionInfo.h"


#ifndef OKAO_DEF_HDETECTION
#define OKAO_DEF_HDETECTION
    typedef VOID* HDETECTION;       /* Face Detection handle */
#endif /* OKAO_DEF_HDETECTION */

#ifndef OKAO_DEF_HDTRESULT
#define OKAO_DEF_HDTRESULT
    typedef VOID* HDTRESULT;        /* Face Detection Result handle */
#endif /* OKAO_DEF_HDTRESULT */

#define DETECTOR_TYPE_SOFT_DT_V7   (DET_SOFT|DET_DT|DET_V7)  /* Face Detection V7 */

/* Still Mode for Partical area search */
#define CUSTOM_MODE_PARTIAL_STILL    (6)

/* Threshold Information */
typedef struct tag_ThresholdInfo{
    INT32           nDirection;                  /* direction for threshold setting of each face angle               */
    INT32           nAngle;                      /* roll width setting threshold values with nDirection as a central */
    INT32           nTargetThreshold;            /* confidence threshold within selected area                        */
    INT32           nNonTargetThreshold;         /* confidence threshold outside selected area                       */
    INT32           nInnerTargetThreshold;       /* inner threshold within selected area                             */
    INT32           nInnerNonTargetThreshold;    /* inner threshold outside selected area                            */
} THRESHOLDINFO;

/* Direction for Threshold Information */
#define THRESHDIRECTION_UP           (0)
#define THRESHDIRECTION_RIGHT        (1)
#define THRESHDIRECTION_DOWN         (2)
#define THRESHDIRECTION_LEFT         (3)

/* Roll Angle for Threshold Information */
#define THRESHANGLE_15               (0)
#define THRESHANGLE_45               (1)
#define THRESHANGLE_75               (2)

#ifdef  __cplusplus
extern "C" {
#endif

/**********************************************************/
/* Version infomation                                     */
/**********************************************************/
/* Gets This Library's version */
OKAO_API INT32      OKAO_DT_GetVersion(UINT8 *pucMajor, UINT8 *pucMinor);

/**********************************************************/
/* Handle Creation/Deletion                               */
/**********************************************************/
/* Creates/Deletes the Face Detection handle */
OKAO_API HDETECTION OKAO_DT_CreateHandle(HCOMMON hCO,
                                             INT32 nDetectionMode, INT32 nMaxDetectionCount);
OKAO_API INT32      OKAO_DT_DeleteHandle(HDETECTION hDT);

/* Creates/Deletes the Face Detection result handle */
OKAO_API HDTRESULT  OKAO_DT_CreateResultHandle(HCOMMON hCO);
OKAO_API INT32      OKAO_DT_DeleteResultHandle(HDTRESULT hDtResult);

/**********************************************************/
/* Excecution of Detection/Tracking                       */
/**********************************************************/
/* Executes detection */
OKAO_API INT32      OKAO_DT_Detect_GRAY(HDETECTION hDT,
                                            RAWIMAGE* pImageGRAY,
                                            INT32 nWidth, INT32 nHeight, GRAY_ORDER ImageOrder,
                                            HDTRESULT hDtResult);
OKAO_API INT32      OKAO_DT_Detect_YUV422(HDETECTION hDT,
                                              RAWIMAGE* pImageYUV,
                                              INT32 nWidth, INT32 nHeight, YUV422_ORDER ImageOrder,
                                              HDTRESULT hDtResult);
OKAO_API INT32      OKAO_DT_Detect_YUV420SP(HDETECTION hDT,
                                                RAWIMAGE* pImageY, RAWIMAGE* pImageCx,
                                                INT32 nWidth, INT32 nHeight, YUV420SP_ORDER ImageOrder,
                                                HDTRESULT hDtResult);
OKAO_API INT32      OKAO_DT_Detect_YUV420FP(HDETECTION hDT,
                                                RAWIMAGE* pImageY, RAWIMAGE* pImageCb, RAWIMAGE* pImageCr,
                                                INT32 nWidth, INT32 nHeight, YUV420FP_ORDER ImageOrder,
                                                HDTRESULT hDtResult);

/**********************************************************/
/* Gets Detection Result                                  */
/**********************************************************/
/* Gets/Gets the result count, i.e. the number of Faces detected with Face Detection */
OKAO_API INT32      OKAO_DT_GetResultCount(HDTRESULT hDtResult, INT32 *pnCount);
/* Gets the Face position info */
OKAO_API INT32      OKAO_DT_GetResultInfo(HDTRESULT hDtResult,
                                          INT32 nIndex, DETECTION_INFO *psDetectionInfo);
/* Gets the raw result info */
OKAO_API INT32      OKAO_DT_GetRawResultInfo(HDTRESULT hDtResult,
                                          INT32 nIndex, DETECTION_INFO *psDetectionInfo);

/*************************************************************/
/* Setting functions for Still and Movie mode (COMMON)       */
/*************************************************************/
/* Sets/Gets the detection size range, i.e. the minimum and maximum Face sizes for detection */
OKAO_API INT32      OKAO_DT_SetSizeRange(HDETECTION hDT, INT32 nMinSize, INT32 nMaxSize);
OKAO_API INT32      OKAO_DT_GetSizeRange(HDETECTION hDT, INT32 *pnMinSize, INT32 *pnMaxSize);

/* Sets/Gets an edge mask to restrict the area where Face Detection will be applied */
OKAO_API INT32      OKAO_DT_SetEdgeMask(HDETECTION hDT, RECT rcEdgeMask);
OKAO_API INT32      OKAO_DT_GetEdgeMask(HDETECTION hDT, RECT *prcEdgeMask);

/* Sets/Gets the search density used by Face Detection */
OKAO_API INT32      OKAO_DT_SetSearchDensity(HDETECTION hDT, INT32 nSearchDensity);
OKAO_API INT32      OKAO_DT_GetSearchDensity(HDETECTION hDT, INT32 *pnSearchDensity);

/* Sets/Gets the pose (yaw) angle and the Face inclination (roll) angle to be used for Face Detection */
OKAO_API INT32      OKAO_DT_SetAngle(HDETECTION hDT, UINT32 nPose, UINT32 nAngle);
OKAO_API INT32      OKAO_DT_GetAngle(HDETECTION hDT, UINT32 nPose, UINT32 *pnAngle);

/* Sets/Gets the threshold value for the Face Detection results */
OKAO_API INT32      OKAO_DT_SetThreshInfo(HDETECTION hDT, INT32 nPose, THRESHOLDINFO sNonTrackingThresholdInfo);
OKAO_API INT32      OKAO_DT_GetThreshInfo(HDETECTION hDT, INT32 nPose, THRESHOLDINFO *psNonTrackingThresholdInfo);

/* Sets/Gets the threshold value for the Face Detection results */
OKAO_API INT32      OKAO_DT_SetThreshold(HDETECTION hDT, INT32 nThreshold);
OKAO_API INT32      OKAO_DT_GetThreshold(HDETECTION hDT, INT32 *pnThreshold);

/*************************************************************/
/* functions for Movie mode only (MOVIE MODE only)           */
/*************************************************************/
/* Sets/Gets an edge mask to restrict the area where tracking will be applied */
OKAO_API INT32      OKAO_DT_MV_SetTrackingEdgeMask(HDETECTION hDT, RECT rcEdgeMask);
OKAO_API INT32      OKAO_DT_MV_GetTrackingEdgeMask(HDETECTION hDT, RECT *prcEdgeMask);

/* Sets/Gets the search cycle, i.e. the frame count used during Initial Face Search */
/* and New Face Search, and the search interval for New Face Search */
OKAO_API INT32      OKAO_DT_MV_SetSearchCycle(HDETECTION hDT, INT32 nInitialFaceSearchCycle,
                                              INT32 nNewFaceSearchCycle, INT32 nNewFaceSearchInterval);
OKAO_API INT32      OKAO_DT_MV_GetSearchCycle(HDETECTION hDT, INT32 *pnInitialFaceSearchCycle,
                                              INT32 *pnNewFaceSearchCycle, INT32 *pnNewFaceSearchInterval);

/* Sets/Gets a direction mask for New Face Search */
OKAO_API INT32      OKAO_DT_MV_SetDirectionMask(HDETECTION hDT, BOOL bMask);
OKAO_API INT32      OKAO_DT_MV_GetDirectionMask(HDETECTION hDT, BOOL* pbMask);

/* Set/Get the angle extension for Face Detection during tracking */
OKAO_API INT32      OKAO_DT_MV_SetAngleExtension(HDETECTION hDT, BOOL bExtension);
OKAO_API INT32      OKAO_DT_MV_GetAngleExtension(HDETECTION hDT, BOOL* pbExtension);

/* Set/Get whether to use or not the angle extension for Face Detection during tracking */
OKAO_API INT32      OKAO_DT_MV_SetPoseExtension(HDETECTION hDT, BOOL bExtension, BOOL bUseHeadTracking);
OKAO_API INT32      OKAO_DT_MV_GetPoseExtension(HDETECTION hDT, BOOL* pbExtension, BOOL* pbUseHeadTracking);

/* Sets/Gets the Tracking threshold value for the Face Detection results */
OKAO_API INT32      OKAO_DT_MV_SetThreshInfo(HDETECTION hDT,INT32 nPose, THRESHOLDINFO sTrackingThresholdInfo);
OKAO_API INT32      OKAO_DT_MV_GetThreshInfo(HDETECTION hDT,INT32 nPose, THRESHOLDINFO *psTrackingThresholdInfo);

/* Sets/Gets the Headthreshold value for the Face Detection results */
OKAO_API INT32      OKAO_DT_MV_SetHeadThreshInfo(HDETECTION hDT, INT32 nHeadThreshold);
OKAO_API INT32      OKAO_DT_MV_GetHeadThreshInfo(HDETECTION hDT, INT32 *pnHeadThreshold);

/* Sets/Gets the tracking accuracy */
OKAO_API INT32      OKAO_DT_MV_SetAccuracy(HDETECTION hDT, INT32 nAccuracy);
OKAO_API INT32      OKAO_DT_MV_GetAccuracy(HDETECTION hDT, INT32* pnAccuracy);

/* Sets/Gets the delay count used for Initial Face Search and New Face Search */
OKAO_API INT32      OKAO_DT_MV_SetDelayCount(HDETECTION hDT, INT32 nDelayCount);
OKAO_API INT32      OKAO_DT_MV_GetDelayCount(HDETECTION hDT, INT32 *pnDelayCount);

/* Sets/Gets the parameters when a Face is lost during tracking in Movie mode */
OKAO_API INT32      OKAO_DT_MV_SetLostParam(HDETECTION hDT, INT32 nMaxRetryCount, INT32 nMaxHoldCount);
OKAO_API INT32      OKAO_DT_MV_GetLostParam(HDETECTION hDT, INT32 *pnMaxRetryCount, INT32 *pnMaxHoldCount);

/* Sets/Gets the steadiness parameters of the position and size of the Face rectangle during tracking */
OKAO_API INT32      OKAO_DT_MV_SetSteadinessParam(HDETECTION hDT,
                                     INT32 nPosSteadinessParam, INT32 nSizeSteadinessParam);
OKAO_API INT32      OKAO_DT_MV_GetSteadinessParam(HDETECTION hDT,
                                     INT32 *pnPosSteadinessParam, INT32 *pnSizeSteadinessParam);

/* Sets the option to always lock the tracking of a Face designated by its Face ID, or unlock it */
OKAO_API INT32      OKAO_DT_MV_ToggleTrackingLock(HDETECTION hDT, INT32 nID);

/* Sets/Gets the tracking swap ratio used when swapping Faces during tracking */
OKAO_API INT32      OKAO_DT_MV_SetTrackingSwapParam(HDETECTION hDT, INT32 nTrackingSwapParam);
OKAO_API INT32      OKAO_DT_MV_GetTrackingSwapParam(HDETECTION hDT, INT32 *pnTrackingSwapParam);

/* Clears the tracking info stored in the Face Detection handle */
OKAO_API INT32      OKAO_DT_MV_ResetTracking(HDETECTION hDT);

/* Sets/Gets the MajorSearch parameter */
OKAO_API INT32      OKAO_DT_MV_SetMajorSize(HDETECTION hDT, INT32 nMajorSize);
OKAO_API INT32      OKAO_DT_MV_GetMajorSize(HDETECTION hDT, INT32 *pnMajorSize);

OKAO_API INT32      OKAO_DT_MV_SetMajorSearchCycle(HDETECTION hDT, INT32 nMajorSearchCycle);
OKAO_API INT32      OKAO_DT_MV_GetMajorSearchCycle(HDETECTION hDT, INT32 *pnMajorSearchCycle);

OKAO_API INT32      OKAO_DT_MV_SetMajorAngle(HDETECTION hDT, UINT32 nPose, UINT32 nMajorAngle);
OKAO_API INT32      OKAO_DT_MV_GetMajorAngle(HDETECTION hDT, UINT32 nPose, UINT32 *pnMajorAngle);


#ifdef  __cplusplus
}
#endif

#endif /* OKAODTAPI_H__ */
