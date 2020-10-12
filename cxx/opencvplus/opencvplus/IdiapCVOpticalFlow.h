// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvpCalcOpticalFlowPyrLK - method to extract sparse 2D motion vectors using
//                           the KLT algorithm
//
// Authors: Remi Emonet (remi.emonet@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_IDIAPCVOPTICALFLOWPYRLK_H__
#define __IP_IDIAPCVOPTICALFLOWPYRLK_H__

#include <cv.h>

CVAPI(void)  cvpCalcOpticalFlowPyrLK( const CvArr*  prev, const CvArr*  curr,
                                      CvArr*  prev_pyr, CvArr*  curr_pyr,
                                      const CvPoint2D32f* prev_features,
                                      CvPoint2D32f* curr_features,
                                      int       count,
                                      CvSize    win_size,
                                      int       level,
                                      char*     status,
                                      float*    track_error,
                                      CvTermCriteria criteria,
                                      int       flags,
                                      float Gmin = 1,
                                      int static_window_size = 0,
                                      float static_threshold = -1,
                                      int size_factor = 1 // set to 0 to disable adaptation of size in pixel to pyramid level
                                      );


#endif // __IP_IDIAPCVOPTICALFLOWPYRLK_H__
