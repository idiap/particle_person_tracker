// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_InterestPointDetector - abstract class to extract sparse 2D interest
//                            points
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES

// LOCAL INCLUDES
#include <image_processing/ip_InterestPointDetector.h>   // declaration of this

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_InterestPointDetector::ip_InterestPointDetector(
        ip_ImageProvider * img_provider, unsigned count) :
        m_ImageProvider(img_provider), m_InterestPointsCount(count) {
    m_InterestPoints.resize(count);
} // ip_InterestPointDetector

ip_InterestPointDetector::~ip_InterestPointDetector() {
} // ~ip_InterestPointDetector

const std::vector<CvPoint2D32f>&
ip_InterestPointDetector::get2DInterestPoints(const ip_RoiWindow& roi) {
    recompute_interest_points(m_ImageProvider->image(roi), m_InterestPoints);
    return m_InterestPoints;
} // get2DMotionPoints


} // namespace ImageProcessing
