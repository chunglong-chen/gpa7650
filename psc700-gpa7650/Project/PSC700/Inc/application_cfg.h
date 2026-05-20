/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2022 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#ifndef __APPLICATION_CFG_H__
#define __APPLICATION_CFG_H__

#include "psc700_conf.h"
//=================================================================
// USBD UVC Configurations
//      - Color Format: YUY2
//      - Resoultion:	PSC700_LIVEVIEW_WIDTH x PSC700_LIVEVIEW_HEIGHT
//      - Frame Rate:   30
//=================================================================
/* Configure UVC Width in YUY2 mode */
#ifndef UVC_TAR_WIDTH_YUY2
#define UVC_TAR_WIDTH_YUY2      PSC700_LIVEVIEW_WIDTH
#endif //UVC_TAR_WIDTH_YUY2
/* Configure UVC Height in YUY2 mode */
#ifndef UVC_TAR_HEIGHT_YUY2
#define UVC_TAR_HEIGHT_YUY2     PSC700_LIVEVIEW_HEIGHT
#endif //UVC_TAR_HEIGHT_YUY2
/* Configure UVC Frame Rate */
#ifndef UVC_FR
#define UVC_FR                  30
#endif //UVC_FR
/* Configure UVC Frame Interval */
#ifndef UVC_FRM_INTVL
#define UVC_FRM_INTVL           ((20000000/UVC_FR)+1)/2
#endif //UVC_FRM_INTVL

#endif		// __APPLICATION_CFG_H__
