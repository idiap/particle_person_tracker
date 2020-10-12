/**
 * @file cxx/image_processing/image_processing/ip_TrackingMemory.h
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

#ifndef IP_TRACKINGMEMORYIMAGE_H
#define IP_TRACKINGMEMORYIMAGE_H

// SYSTEM INCLUDES
#include <list>                                           // STL list

// LOCAL INCLUDES
#include <image_processing/ip_RoiWindow.h>                // ROI window
#include <image_processing/ip_ImageProvider.h>            // image provider

namespace ImageProcessing {

/// @brief Image-based memory of tracker positions
///
/// Memory is an array of a size of an image, every cell contains rate at which
/// tracked head appeared in that cell.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 3.0
/// @date    28.11.2011
class ip_TrackingMemoryImage {

  public:

    /// Constructor.
    /// Based on data provider and update factor constructs tracker memory of
    /// appropriate size and decay rate.
    /// @param data_provider Data provider used for tracking
    /// @param update_factor Factor at which memory decay occurs
    ip_TrackingMemoryImage(const ip_ImageProvider * data_provider,
            float update_factor);

    /// Destructor.
    /// Deallocates tracker memory.
    ~ip_TrackingMemoryImage();

    /// Updates memory values based on a set of currently observed tracks.
    /// Uses decay rate specified in the constructor.
    /// @param rois Currently observed tracker bounding boxes.
    void update(const std::list<ip_RoiWindow>& rois);

    /// Get tracking memory value corresponding to the specified pixel.
    /// @param col Column of the pixel
    /// @param row Row of the pixel
    /// @return Tracking memory value
    float value(int col, int row);

    /// Get the whole of tracking memory
    /// @return Tracking memory as a matrix of float values
    const CvMat * memory() const {
        return m_pMemory;
    }

    //{ return m_pImage->value(y, x); };

  private:

    // tracking memory values storage
    CvMat * m_pMemory;
    // tracking memory update / decay rate
    const float m_UpdateFactor;
    // global ROI that corresponds to the whole of input image
    ip_RoiWindow m_GlobalRoi;

};

} // namespace ImageProcessing

#endif // IP_TRACKINGMEMORYIMAGE_H
