// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_RoiWindow - class to represent the ROI window on an image
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <algorithm>                                  // for min

// PROJECT INCLUDES
#include <opencvplus/cvp_utils.h>                     // for intersect

// LOCAL INCLUDES
#include <image_processing/ip_RoiWindow.h>            // declaration of this
#include <image_processing/ip_Exceptions.h>           // exceptions

using namespace ImageProcessing;
using namespace std;

/////////////////////////////// GLOBAL ///////////////////////////////////////

int ImageProcessing::area(const ip_RoiWindow& roi) {
    return roi.m_iHeight * roi.m_iWidth;
} // area

inline float sign(float v) {
    return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

int ImageProcessing::intersection_area(const ip_RoiWindow& roi1,
        const ip_RoiWindow& roi2) {

    ip_RoiWindow roi_intersect;
    bool intersects_flag = intersect(roi1, roi2, roi_intersect);
    if (intersects_flag) {
        return area(roi_intersect);
    } else {
        return 0;
    }
} // intersection_area

int ImageProcessing::union_area(const ip_RoiWindow& roi1,
                                const ip_RoiWindow& roi2)
{
  return area(roi1) + area(roi2) - intersection_area(roi1, roi2);
} // union_area


bool ImageProcessing::intersect(
        const ip_RoiWindow& roi1, const ip_RoiWindow& roi2,
        ip_RoiWindow& roi_result) {

    CvRect roi1cv = ip_RoiWindow::to_CvRect(roi1);
    CvRect roi2cv = ip_RoiWindow::to_CvRect(roi2);
    CvRect roi_resultcv;

    bool result =  OpenCvPlus::intersect(roi1cv, roi2cv, roi_resultcv);
    if (result) {
        roi_result = ip_RoiWindow::from_CvRect(roi_resultcv);
    }
    return result;

} // intersect

ip_RoiWindow ImageProcessing::ip_RoiWindow::from_CvRect(const CvRect& cvrect) {
    ip_RoiWindow roi;
    roi.m_iFirstColumn = cvrect.x;
    roi.m_iFirstRow = cvrect.y;
    roi.m_iWidth = cvrect.width;
    roi.m_iHeight = cvrect.height;
    return roi;
} // from_CvRect

CvRect ImageProcessing::ip_RoiWindow::to_CvRect(const ip_RoiWindow& roi) {
    return cvRect(roi.m_iFirstColumn, roi.m_iFirstRow, roi.m_iWidth,
        roi.m_iHeight);
} // to_CvRect

bool ImageProcessing::contains(const ip_RoiWindow& roi1, int col, int row) {

    return (roi1.m_iFirstColumn <= col) &&
           (roi1.m_iFirstColumn + roi1.m_iWidth > col) &&
           (roi1.m_iFirstRow <= row) &&
           (roi1.m_iFirstRow + roi1.m_iHeight > row);

} // contains

ip_RoiWindow ImageProcessing::scale(const ip_RoiWindow& roi, float scale) {
    if (scale < 0) {
        throw ip_Exception(
                "Invalid scaling parameter provided when scaling ROI");
    }
    const float sx = static_cast<float>(roi.m_iWidth) / 2;
    const float sy = static_cast<float>(roi.m_iHeight) / 2;
    const float fx = roi.m_iFirstColumn + sx * (1.0 - scale);
    const float fy = roi.m_iFirstRow + sy * (1.0 - scale);
    ip_RoiWindow result;

    if (scale > 1) {
        result.m_iFirstColumn = floor(fx);
        result.m_iFirstRow = floor(fy);
    } else {
        result.m_iFirstColumn = ceil(fx);
        result.m_iFirstRow = ceil(fy);
    }
    result.m_iWidth = static_cast<int>(roi.m_iWidth * scale);
    result.m_iHeight = static_cast<int>(roi.m_iHeight * scale);
    return result;
} // scale

std::ostream& operator<<(std::ostream& os, const ip_RoiWindow& roi) {
    os << roi.m_iFirstRow << " " << roi.m_iFirstColumn << " "
       << roi.m_iHeight   << " " << roi.m_iWidth;
    return os;
}

std::istream& operator>>(std::istream& is, ip_RoiWindow& roi) {
    is >> roi.m_iFirstRow >> roi.m_iFirstColumn
       >> roi.m_iHeight >> roi.m_iWidth;
    return is;
}

bool operator==(const ImageProcessing::ip_RoiWindow& lhs_roi,
        const ImageProcessing::ip_RoiWindow& rhs_roi) {
    return (lhs_roi.m_iFirstColumn == rhs_roi.m_iFirstColumn) &&
           (lhs_roi.m_iFirstRow == rhs_roi.m_iFirstRow) &&
           (lhs_roi.m_iHeight == rhs_roi.m_iHeight) &&
           (lhs_roi.m_iWidth == rhs_roi.m_iWidth);
}
