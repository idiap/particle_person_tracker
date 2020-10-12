// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionProcessor - class to extract sparse 2D motion vectors
//                              from two consecutive images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_Sparse2DMotionProcessor.h> // declaration of this

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_Sparse2DMotionProcessor::ip_Sparse2DMotionProcessor(
        ip_ImageProvider * gray_img_provider,
        ip_InterestPointDetector * int_point_detector) :
        m_GrayImageProvider(gray_img_provider),
        m_PointDetector(int_point_detector),
        m_PointsValid(false),
        m_PreviousImage(0) {
} // ip_Sparse2DMotionProcessor

ip_Sparse2DMotionProcessor::~ip_Sparse2DMotionProcessor() {
} // ~ip_Sparse2DMotionProcessor

void ip_Sparse2DMotionProcessor::invalidate() {
    m_PointsValid = false;
} // invalidate

const ip_MotionPoints&
ip_Sparse2DMotionProcessor::get2DMotionPoints() {
    if (!m_PointsValid) {
        ip_RoiWindow roi;
        roi.m_iFirstColumn = 0;
        roi.m_iFirstRow = 0;
        roi.m_iWidth = m_GrayImageProvider->image_buffer()->width;
        roi.m_iHeight = m_GrayImageProvider->image_buffer()->height;

        return get2DMotionPoints(roi);

    }
    return m_MotionPoints;
} // get2DMotionPoints

const ip_MotionPoints&
ip_Sparse2DMotionProcessor::get2DMotionPoints(const ip_RoiWindow& roi) {
    if (!m_PointsValid) {

        IplImage * current_image = m_GrayImageProvider->image();
        const std::vector<CvPoint2D32f>& current_pts =
                m_PointDetector->get2DInterestPoints(roi);

        if (m_PreviousImage) {
            recompute_motion_points(current_pts, current_image,
                m_PreviousPoints, m_PreviousImage, m_MotionPoints);
        } else {
            m_PreviousImage = cvCloneImage(current_image);
        }
        cvCopy(current_image, m_PreviousImage);
//        m_PreviousImage = current_image;
        m_PreviousPoints.resize(current_pts.size());
        copy(current_pts.begin(), current_pts.end(),
                m_PreviousPoints.begin());

        m_PointsValid = true;
    }
    return m_MotionPoints;
} // get2DMotionPoints

} // namespace ImageProcessing
