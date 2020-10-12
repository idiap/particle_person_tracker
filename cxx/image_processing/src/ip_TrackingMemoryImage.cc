/**
 * @file cxx/image_processing/src/ip_TrackingMemory.cc
 * @date 28 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image-based memory of tracker positions
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */
#include "opencv2/opencv.hpp"
// SYSTEM INCLUDES
#include <boost/foreach.hpp>                             // boost foreach

// LOCAL INCLUDES
#include <image_processing/ip_TrackingMemoryImage.h>     // declaration of this

using namespace std;

namespace ImageProcessing {

ip_TrackingMemoryImage::ip_TrackingMemoryImage(
        const ip_ImageProvider * data_provider, float update_factor) :
        m_UpdateFactor(update_factor) {

    IplImage * image_prototype = data_provider->image_buffer();
    m_pMemory = cvCreateMat(image_prototype->height, image_prototype->width,
            CV_32FC1);
    cvZero(m_pMemory);
    m_GlobalRoi.m_iFirstColumn = 0;
    m_GlobalRoi.m_iFirstRow = 0;
    m_GlobalRoi.m_iWidth = image_prototype->width;
    m_GlobalRoi.m_iHeight = image_prototype->height;
} // ip_TrackingMemoryImage

ip_TrackingMemoryImage::~ip_TrackingMemoryImage() {
    cvReleaseMat(&m_pMemory);
} // ~ip_TrackingMemoryImage

void
ip_TrackingMemoryImage::update(const std::list<ip_RoiWindow>& rois) {

    // decay the whole memory
    cvScale(m_pMemory, m_pMemory, 1.0f - m_UpdateFactor);

    // DEBUG: show map of forget memory
    // cvShowImage("memory", m_pMemory);

    bool intersect_flag;
    ip_RoiWindow intersection;
    CvMat intersection_mat;

    // for every tracker ROI
    BOOST_FOREACH(const ip_RoiWindow& roi, rois) {
        intersect_flag = intersect(roi, m_GlobalRoi, intersection);
        if (intersect_flag) {
            // update its intersection with the image
            cvInitMatHeader(&intersection_mat, intersection.m_iHeight,
                intersection.m_iWidth, CV_32FC1, m_pMemory->data.ptr +
                m_pMemory->step * intersection.m_iFirstRow +
                sizeof(float) * intersection.m_iFirstColumn,
                m_pMemory->step);
            cvAddS(&intersection_mat, cvScalarAll(m_UpdateFactor),
                    &intersection_mat);
        }
    }
} // update

float
ip_TrackingMemoryImage::value(int col, int row) {
    return CV_MAT_ELEM(*m_pMemory, float, row, col);
} // value

    //{ return m_pImage->value(y, x); };

} // namespace ImageProcessing
