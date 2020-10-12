/**
 * @file cxx/image_processing/image_processing/ip_RoiWindow.h
 * @date 18 February 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image region of interest (ROI) representation
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_ROIWINDOW_H__
#define __IP_ROIWINDOW_H__

#include <iostream>                 // IO streaming
#include <cv.h>                     // OpenCV

namespace ImageProcessing {

/// @brief Class to represent the ROI window on an image
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    18.02.2011

class ip_RoiWindow
{
public:

  /// First column of the selected region
  int m_iFirstColumn;
  /// First row of the selected region
  int m_iFirstRow;
  /// Width of the selected region
  int m_iWidth;
  /// Height of the selected region
  int m_iHeight;

  // LIFECYCLE

  /// Constructs default ip_RoiWindow structure with zero values
  ip_RoiWindow():
    m_iFirstColumn(0), m_iFirstRow(0),
    m_iWidth(0),
    m_iHeight(0)
  {
  }

  /// Constructs ip_RoiWindow structure from the provided ROI values
  /// @param fc First column of ROI
  /// @param fr First row of ROI
  /// @param w ROI width
  /// @param h ROI height
  ip_RoiWindow(int fc, int fr, int w, int h):
    m_iFirstColumn(fc),
    m_iFirstRow(fr), m_iWidth(w), m_iHeight(h)
  {
  }

  // OPERATIONS

  /// Converts CvRect to ip_RoiWindow structure
  /// @param cvrect Region of interest in OpenCV CvRect structure
  /// @return Region of interest in ip_RoiWindow structure
  static ip_RoiWindow from_CvRect(const CvRect& cvrect);

  /// Converts ip_RoiWindow to CvRect structure
  /// @param roi Region of interest in ip_RoiWindow structure
  /// @return Region of interest in OpenCV CvRect structure
  static CvRect to_CvRect(const ip_RoiWindow& roi);

};

/// Measures the area of the region of interest (ROI)
/// @param roi Region of interest in ip_RoiWindow structure
/// @return Area of the region of interest
int area(const ip_RoiWindow& roi);

/// Measures the area of intersection of two regions of interest
/// @param roi1 First region of interest
/// @param roi2 Second region of interest
/// @return Intersection area of two regions of interest
int intersection_area(const ip_RoiWindow& roi1, const ip_RoiWindow& roi2);

int union_area(const ip_RoiWindow& roi1, const ip_RoiWindow& roi2);

/// Intersects two regions of interest
/// @param roi1 First region of interest
/// @param roi2 Second region of interest
/// @param roi_result ROI to store intersection of the two regions of interest;
/// contents in not defined if the two provided ROIs do not intersect
/// @return True if the two regions of interest intersect, False otherwise
bool intersect(const ip_RoiWindow& roi1, const ip_RoiWindow& roi2,
    ip_RoiWindow& roi_result);

/// Checks whether a region of interest contains a given point
/// @param roi Region of interest
/// @param col Column (X coordinate) of an image point
/// @param row Row (Y coordinate) of an image point
/// @return True if the region of interest contains the point
bool contains(const ip_RoiWindow& roi, int col, int row);

/// Scales region of interest leaving its centre unmodified and displacing its
/// boundaries proportional to the scaling factor.
/// @param roi Region of interest to scale
/// @param scale Positive scale parameter
/// @return Scaled region of interest
ip_RoiWindow scale(const ip_RoiWindow& roi, float scale);

} // namespace ImageProcessing

std::ostream& operator<<(std::ostream& os,
        const ImageProcessing::ip_RoiWindow& roi);
std::istream& operator>>(std::istream& is,
        ImageProcessing::ip_RoiWindow& roi);
bool operator==(const ImageProcessing::ip_RoiWindow& lhs_roi,
        const ImageProcessing::ip_RoiWindow& rhs_roi);

#endif // __IP_ROIWINDOW_H__
