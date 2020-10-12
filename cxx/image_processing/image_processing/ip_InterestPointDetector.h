// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_InterestPointDetector - abstract class to extract sparse 2D interest
//                            points
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_INTERESTPOINTDETECTOR_H__
#define __IP_INTERESTPOINTDETECTOR_H__

// SYSTEM INCLUDES
#include <cv.h>                                   // for interest points

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                     // image provider interface
#include "ip_RoiWindow.h"                         // ROI declaration


namespace ImageProcessing {

/// @brief Abstract class to extract sparse 2D interest points
///
/// This class introduces a general interface to extract sparse 2D interest
/// points from an image based on an image provider.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_InterestPointDetector {

    public:

    // LIFECYCLE

    /// Constructor to initialize sparse 2D interest point detector
    /// @param img_provider Image provider
    /// @param count Point of interest max count
    ip_InterestPointDetector(ip_ImageProvider * img_provider, unsigned count);

    /// Destructor to be defined by descendants
    virtual ~ip_InterestPointDetector();

    // OPERATIONS

    /// Computes the 2D interest points in an image in the provided
    /// region of integest (ROI)
    /// @param roi Region of interest in which the points of interest
    ///            are computed
    /// @return A collection of points of interest
    const std::vector<CvPoint2D32f>& get2DInterestPoints(
            const ip_RoiWindow& roi);

    /// Returns desired number of points of interest
    /// @return Desired number of points of interest
    unsigned interest_points_count() const {
        return m_InterestPointsCount;
    }

    protected:

    /// Computes interest points for the input image
    /// @param image Image for which the points of interest are computed
    /// @param pts_storage A collection of points of interest to fill
    virtual void recompute_interest_points(const IplImage * image,
            std::vector<CvPoint2D32f>& pts_storage) = 0;

    private:

    // provider of source images
    ip_ImageProvider * m_ImageProvider;
    // cached points count
    unsigned m_InterestPointsCount;
    // cached points of interest
    std::vector<CvPoint2D32f> m_InterestPoints;

}; // class ip_InterestPointDetector

} // namespace ImageProcessing

#endif // __IP_INTERESTPOINTDETECTOR_H__
