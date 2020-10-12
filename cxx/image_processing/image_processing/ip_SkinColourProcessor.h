// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SkinColourProcessor - class to extract skin colour pixels from images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SKINCOLOURPROCESSOR_H__
#define __IP_SKINCOLOURPROCESSOR_H__

// PROJECT INCLUDES
#include <opencvplus/FaceColorModel.h>                   // face color model
#include <opencvplus/cvp_SkinColourModel.h>              // skin colour model

// LOCAL INCLUDES
#include "ip_ImageProviderGroup.h"                       // base class
#include "ip_RoiWindow.h"                                // ROI window

namespace ImageProcessing {

/// @brief Class to extract skin colour pixels
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_SkinColourProcessor : public ip_ImageProvider {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param colour_img_provider Colour input image
    /// @param face_colour_model Coour model that can be used to detect skin
    /// @param skin_mask_value Skin mask value, should be in the range [1, 255]
    /// to be castable to uchar
    ip_SkinColourProcessor(ip_ImageProvider * colour_img_provider,
            FaceColorModel::FaceColorModel * face_colour_model,
            float skin_mask_value);

    /// Destructor
    virtual ~ip_SkinColourProcessor();

    // OPERATIONS

    /// Overrides base class method, returns IP_IMG_PROVIDER_GRAYSCALE
    /// @return Provider's ID = IP_IMG_PROVIDER_GRAYSCALE
    /// @see ip_ImageProviderType
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_SKIN_MASK;
    };

    void update_colour_model(const OpenCvPlus::cvp_SkinColourModel& model);
    void update_colour_model(const OpenCvPlus::cvp_SkinColourModel& model,
        float rate);
    void set_colour_model(const OpenCvPlus::cvp_SkinColourModel& model);
    OpenCvPlus::cvp_SkinColourModel get_colour_model();
    void set_threshold(float threshold);
    void reset_colour_model();
    void update_colour_model(IplImage * image, IplImage * mask,
            const ip_RoiWindow& roi, float rate);

    protected:

    // OPERATIONS

    /// Overrides base class method, obtains the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    /// Computes skin mask using face colour model for the provided image and
    /// region of interest (ROI). Pixels outside ROI are marked as non-skin
    /// by default.
    /// @param ipInputImage Input RGB image of depth IPL_DEPTH_8U
    /// @param opOutputImage Output image of the same size as input image,
    /// 1 channel, depth IPL_DEPTH_8U containing 255 for skin pixels and 0
    /// for non-skin pixels
    /// @param opTempImage1 Temporary mask image (1 channel, IPL_DEPTH_8U)
    /// of the same size as the original image
    /// @param opTempImage2 Temporary mask image (1 channel, IPL_DEPTH_8U)
    /// of the same size as the original image
    /// @param opConvertedColourBuffer Temporary image (3 channels, IPL_DEPTH_32F)
    /// of the same size as the original image
    /// @param opHistogramColourBuffer Temporary image (1 channel, IPL_DEPTH_32S)
    /// of the same size as the original image
    /// @param roi Region of interest for which to compute the mask
    void compute_skin_mask_using_face_colour_model(
            IplImage const* ipInputImage, IplImage* opOutputImage,
            IplImage* opTempImage1, IplImage* opTempImage2,
            IplImage* opConvertedColourBuffer, IplImage* opHistogramColourBuffer,
            const ImageProcessing::ip_RoiWindow& roi);

    ip_ImageProvider * mSourceImageProvider;     // source image provider
    FaceColorModel::FaceColorModel * mFaceColourModel; // face colour model
    const float mSkinMaskValue;  // value to fill skin pixels with
    float mSkinColourThreshold;  // value to use to theshold
    OpenCvPlus::cvp_SkinColourModel mSkinColourModel;
    IplImage * mTempImage1;      // temporary image for skin colour computation
    IplImage * mTempImage2;      // temporary image for skin colour computation
    // temporary image for comparison operation results
    CvMat * mCompOpRes;

    // skin colour processor buffers
    IplImage * m_HistogramColourBuffer;
    IplImage * m_ConvertedColourBuffer;

};

} // namespace ImageProcessing

#endif // __IP_SKINCOLOURPROCESSOR_H__
