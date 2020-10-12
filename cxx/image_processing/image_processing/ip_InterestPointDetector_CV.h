// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_InterestPointDetector_CV - class to extract sparse 2D interest points
//                               using OpenCV
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_INTERESTPOINTDETECTOR_CV_H__
#define __IP_INTERESTPOINTDETECTOR_CV_H__

// LOCAL INCLUDES
#include "ip_InterestPointDetector.h"             // base class


namespace ImageProcessing {

/// @brief Class to extract sparse 2D interest points using OpenCV
///
/// This class implements a general interface to extract sparse 2D interest
/// points from an image based on an image provider using OpenCV procedures.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_InterestPointDetector_CV : public ip_InterestPointDetector {

    public:

    // LIFECYCLE

    /// Constructor to initialize sparse 2D interest point detector
    /// @param img_provider Image provider
    /// @param count Point of interest max count
    ip_InterestPointDetector_CV(ip_ImageProvider * img_provider,
            unsigned count);

    /// Destructor
    virtual ~ip_InterestPointDetector_CV();

    // OPERATIONS

    protected:

    /// Computes interest points for the input image using OpenCV procedures
    /// @param image Image for which the points of interest are computed
    /// @param pts_storage A collection of points of interest to fill
    virtual void recompute_interest_points(const IplImage * image,
            std::vector<CvPoint2D32f>& pts_storage);

    private:
    // temporary image used while detecting points
    IplImage * m_EigenImage;
    // temporary image used while detecting points
    IplImage * m_TempImage;
    // quality parameter used in point detection algorithm
    float m_PointQuality;
    // distance threshold parameter used in point detection algorithm
    float m_DistanceThreshold;
    // block size parameter used in point detection algorithm
    float m_BlockSize;
    // if nonzero, Harris operator is used instead of the default one
    int m_IsHarrisNotGFTT;

}; // class ip_InterestPointDetector_CV

} // namespace ImageProcessing

#endif // __IP_INTERESTPOINTDETECTOR_CV_H__
