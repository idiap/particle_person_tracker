// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionProcessor_KLT - class to extract sparse 2D motion vectors
//                                  using the KLT algorithm
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SPARSE2DMOTIONPROCESSOR_KLT_H__
#define __IP_SPARSE2DMOTIONPROCESSOR_KLT_H__

// LOCAL INCLUDES
#include "ip_Sparse2DMotionProcessor.h"         // base interest point detector

namespace ImageProcessing {

/// @brief Class to extract sparse 2D motion vectors using the KLT algorithm
///
/// This class implements the general sparse 2D motion detection method using
/// KLT algorithm on points from two consecutive images.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_Sparse2DMotionProcessor_KLT : public ip_Sparse2DMotionProcessor {

    public:

    // LIFECYCLE

    /// Constructor to initialize sparse KLT-based 2D motion processor
    /// @param gray_img_provider Grayscale image provider
    /// @param int_point_detector Interest point detector
    ip_Sparse2DMotionProcessor_KLT(ip_ImageProvider * gray_img_provider,
        ip_InterestPointDetector * int_point_detector);

    /// Destructor, deallocates partial image providers
    virtual ~ip_Sparse2DMotionProcessor_KLT();

    // OPERATIONS

    protected:

    /// Recomputes 2D motion vectors based on current and previous image
    /// interest points
    /// @param pts2dcur Current points of interest
    /// @param imgcur   Current image
    /// @param pts2dprev Previous points of interest
    /// @param imgprev   Previous image
    /// @param result_pts 2D motion vectors and the corresponding image points

    virtual void recompute_motion_points(
        const std::vector<CvPoint2D32f>& pts2dcur, IplImage * imgcur,
        const std::vector<CvPoint2D32f>& pts2dprev, IplImage * imgprev,
        ip_MotionPoints& result_pts);


    private:

    // Size of the search window of each pyramid level
    CvSize m_PyrSearchWindowSize;
    // Specifies when the iteration process of finding the flow for each point
    // on each pyramid level should be stopped.
    CvTermCriteria m_PyrTermCriteria;
    // Buffer for the pyramid for the first frame
    IplImage * m_PyramidPrev;
    // Buffer for the pyramid for the second frame
    IplImage * m_PyramidCur;
    // Maximal pyramid level number
    int m_PyrMaxLevel;
    // Tracking threshold to compare propagated and detected points
    double m_TrackThreshold;
    // Array element is set to 1 if the flow for the corresponding feature
    // has been found, 0 otherwise.
    std::vector<char> m_FoundFeatureFlags;
    // Array of double numbers containing difference between patches around
    // the original and moved points
    std::vector<float> m_FoundFeatureErrors;
    // cache for propagated points
    std::vector<CvPoint2D32f> m_PropagatedPoints;


}; // class ip_Sparse2DMotionProcessor_KLT

} // namespace ImageProcessing

#endif // __IP_SPARSE2DMOTIONPROCESSOR_KLT_H__
