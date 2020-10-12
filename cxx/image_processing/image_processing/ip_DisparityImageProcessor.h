/**
 * @file cxx/image_processing/image_processing/ip_DisparityImageProcessor.h
 * @date 22 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Image processor that computes disparity and depth maps
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_DISPARITYIMAGEPROCESSOR_H__
#define __IP_DISPARITYIMAGEPROCESSOR_H__

#ifndef __STEREO_MATCHER_FOUND__
#error Stereo matching module was not found, ip_DisparityImageProcessor \
    should not be used!
#endif

// SYSTEM INCLUDES
#include <stereomatcher.h>                          // stereo matcher component

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                       // base class

namespace ImageProcessing {

/// @brief Class to compute disparity and depth images
///
/// This class compute disparity and depth images from a pair of stereoscopic
/// input images
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date 22.01.2013

class ip_DisparityImageProcessor : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor
    /// Allocates disparity and depth images of the same size as input images,
    /// 1 channel and 32 and 64 bit depth respectively.
    /// @param img_provider Input image provider
    /// @param calibration_file Calibration file path
    ip_DisparityImageProcessor(ip_ImageProvider * img_provider,
        const std::string& calibration_file);

    /// Destructor, releases disparity and depth images
    virtual ~ip_DisparityImageProcessor();

    /// Overrides base class method, returns IP_IMG_PROVIDER_DATA
    /// @return Provider's ID = IP_IMG_PROVIDER_DATA
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_DISPARITY;
    };

    /// Read/write access to disparity image (IPL_DEPTH_32F, 1 channel)
    /// The image contains disparity values for each point of the left
    /// input image. If the disparity could not be calculated, the pixel
    /// contains a NaN value.
    /// @return Disparity image
    IplImage * disparity_image() {
        return m_DisparityImage_32F1;
    }

    /// Read/write access to depth image (IPL_DEPTH_64F, 3 channels)
    /// Each point of this image contains 3 values - X, Y and Z coordinate of
    /// the
    /// @return Depth image
    IplImage * depth_image() {
        return m_DepthImage_64F3;
    }

    /// Indicates whether disparity map was correctly calculated.
    /// Disparity is calculated correctly, if the source provider
    /// provides correct stereo images.
    /// @return True if disparity map was correctly calculated
    bool has_valid_disparity() const;

    /// Evaluate scene coordinates of an object that is located inside
    /// the provided region of interest (ROI). Evaluation is based on depth
    /// data.
    /// @param roi Region of interest (ROI) for which to evaluate 3D location
    /// @return Mean 3D location of points within the ROI
    CvScalar scene_coordinates_3D_depth(const ip_RoiWindow& roi) const;

    /// Evaluate scene coordinates of an object that is located inside
    /// the provided region of interest (ROI). Evaluation is based on disparity
    /// data.
    /// @param roi Region of interest (ROI) for which to evaluate 3D location
    /// @return Mean 3D location of points within the ROI
    CvScalar scene_coordinates_3D_disparity(const ip_RoiWindow& roi) const;

    protected:

    // OPERATIONS

    /// Overrides base class method, obtains the most recent image
    /// @param image Image to write results to
    virtual void
    recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // provider of source images
    ip_ImageProvider * m_SourceImageProvider;
    // Stereo matcher component
    stereo::StereoMatcher * m_StereoMatcher;
    // resized left image
    IplImage * m_ResizedLeftImage;
    // resized right image
    IplImage * m_ResizedRightImage;
    // cached disparity image
    IplImage * m_DisparityImage_32F1;
    // cached depth image
    IplImage * m_DepthImage_64F3;
    // mask image indicating valid disparity/depth values
    IplImage * m_MaskImage_8U1;
    // indicates whether disparity map was correctly calculated.
    bool m_ValidDisparityFlag;

};

} // namespace ImageProcessing

#endif // __IP_DISPARITYIMAGEPROCESSOR_H__
