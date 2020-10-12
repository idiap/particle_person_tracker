// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionProcessor - class to extract sparse 2D motion vectors
//                              from two consecutive images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SPARSE2DMOTIONPROCESSOR_H__
#define __IP_SPARSE2DMOTIONPROCESSOR_H__

// SYSTEM INCLUDES
#include <vector>                                 // STL vector

// LOCAL INCLUDES
#include "ip_InterestPointDetector.h"             // interest point detector

namespace ImageProcessing {

/// @brief Structure to represent a motion vector at a 2D point
///
/// This structure is a convenient representation of a motion vector at a
/// 2D point that is used for sparse motion estimation.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

struct ip_MotionPoint {
    CvPoint2D32f m_Point;
    CvPoint2D32f m_Motion;
};

typedef std::vector<ip_MotionPoint> ip_MotionPoints;

/// @brief Class to extract sparse 2D motion vectors from two consecutive images
///
/// This class introduces a general method to extract sparse 2D motion
/// vectors from two consecutive images based on a sequential image provider
/// and an interest point detector.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_Sparse2DMotionProcessor {

    public:

    // LIFECYCLE

    /// Constructor to initialize sparse 2D motion processor
    /// @param gray_img_provider Grayscale image provider
    /// @param int_point_detector Interest point detector
    ip_Sparse2DMotionProcessor(ip_ImageProvider * gray_img_provider,
        ip_InterestPointDetector * int_point_detector);

    /// Destructor, deallocates partial image providers
    virtual ~ip_Sparse2DMotionProcessor();

    // OPERATIONS

    /// Makes current results invalid, forces recalculation
    void invalidate();

    /// Provides estimated 2D motion vectors and the corresponding image points
    /// @return Estimated 2D motion vectors and the corresponding image points
    const ip_MotionPoints& get2DMotionPoints();

    /// Provides estimated 2D motion vectors and the corresponding image points
    /// @param roi Region of interest to search the points of interest in
    /// @return Estimated 2D motion vectors and the corresponding image points
    const ip_MotionPoints& get2DMotionPoints(const ip_RoiWindow& roi);

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
        ip_MotionPoints& result_pts) = 0;

    private:

    // provider of source grayscale images
    ip_ImageProvider * m_GrayImageProvider;
    // interest point provider
    ip_InterestPointDetector * m_PointDetector;
    // cached motion points
    ip_MotionPoints m_MotionPoints;
    // flag indicating whether the motion points are up-to-date
    bool m_PointsValid;
    // cached previous image
    IplImage * m_PreviousImage;
    // cached previous points
    std::vector<CvPoint2D32f> m_PreviousPoints;

}; // class ip_Sparse2DMotionProcessor

} // namespace ImageProcessing

#endif // __IP_SPARSE2DMOTIONPROCESSOR_H__
