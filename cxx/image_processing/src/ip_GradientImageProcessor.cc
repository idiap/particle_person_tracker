// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_GradientImageProcessor - class to extract gradient components from images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <iostream>                       // IO

// PROJECT INCLUDES
#include <opencvplus/cvp_gradient_histogram_features.h>

// LOCAL INCLUDES
#include <image_processing/ip_GradientImageProcessor.h> // declaration of this
#include <image_processing/ip_PartialImageProvider.h>   // for colour components

// threshold for square of gradient magnitude
static const int GRADIENT_MAGNITUDE_THRESHOLD = 200 * 200;

using namespace std;

namespace ImageProcessing {

///////////////////////////// LOCAL DEFINITIONS //////////////////////////////

class ip_GradientXProvider : public ip_PartialImageProvider {
    public:
    ip_GradientXProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GRADIENT_X;
    };

}; // class ip_GradientXProvider

class ip_GradientYProvider : public ip_PartialImageProvider {
    public:
    ip_GradientYProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GRADIENT_Y;
    };

}; // class ip_GradientYProvider

class ip_GradientMagnitudeProvider : public ip_PartialImageProvider {
    public:
    ip_GradientMagnitudeProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GRADIENT_MAGNITUDE;
    };

}; // class ip_GradientMagnitudeProvider

class ip_GradientAngleProvider : public ip_PartialImageProvider {
    public:
    ip_GradientAngleProvider(ip_ImageProvider * master_provider,
            IplImage * buffer_image) : ip_PartialImageProvider(master_provider,
            buffer_image) {
    }
    virtual ip_ImageProviderType id() const {
        return IP_IMG_PROVIDER_GRADIENT_ANGLE;
    };

}; // class ip_GradientAngleProvider

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_GradientImageProcessor::ip_GradientImageProcessor(
        ip_ImageProvider * gray_img_provider)
        : mSourceImageProvider(gray_img_provider) {

    int width = gray_img_provider->image_buffer()->width;
    int height = gray_img_provider->image_buffer()->height;
    //int depth = gray_img_provider->image_buffer()->depth;
    int depth = IPL_DEPTH_32F;

    IplImage * gradient_x_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mGradientXProvider = new ip_GradientXProvider(this, gradient_x_image);
    add_provider(mGradientXProvider);

    IplImage * gradient_y_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mGradientYProvider = new ip_GradientYProvider(this, gradient_y_image);
    add_provider(mGradientYProvider);

    IplImage * gradient_magnitude_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mGradientMagnitudeProvider = new ip_GradientMagnitudeProvider(this,
            gradient_magnitude_image);
    add_provider(mGradientMagnitudeProvider);

    IplImage * gradient_angle_image = cvCreateImage(cvSize(width, height),
            depth, 1);
    mGradientAngleProvider = new ip_GradientAngleProvider(this,
            gradient_angle_image);
    add_provider(mGradientAngleProvider);

} // ip_RgbImageProcessor

/* virtual */ ip_GradientImageProcessor::~ip_GradientImageProcessor() {

    remove_provider(mGradientXProvider);
    remove_provider(mGradientYProvider);
    remove_provider(mGradientMagnitudeProvider);
    remove_provider(mGradientAngleProvider);

    IplImage * gradient_x_image = mGradientXProvider->image_buffer();
    cvReleaseImage(&gradient_x_image);
    IplImage * gradient_y_image = mGradientYProvider->image_buffer();
    cvReleaseImage(&gradient_y_image);
    IplImage * gradient_magnitude_image =
            mGradientMagnitudeProvider->image_buffer();
    cvReleaseImage(&gradient_magnitude_image);
    IplImage * gradient_angle_image = mGradientAngleProvider->image_buffer();
    cvReleaseImage(&gradient_angle_image);

    delete mGradientXProvider;
    delete mGradientYProvider;
    delete mGradientMagnitudeProvider;
    delete mGradientAngleProvider;
} // ~ip_GradientImageProcessor

ip_ImageProvider* ip_GradientImageProcessor::getGradientXProvider() {
    return mGradientXProvider;
} // getGradientXProvider

ip_ImageProvider* ip_GradientImageProcessor::getGradientYProvider() {
    return mGradientYProvider;
} // getGradientYProvider

ip_ImageProvider* ip_GradientImageProcessor::getGradientMagnitudeProvider() {
    return mGradientMagnitudeProvider;
} // getGradientMagnitudeProvider

ip_ImageProvider* ip_GradientImageProcessor::getGradientAngleProvider() {
    return mGradientAngleProvider;
} // getGradientAngleProvider

/* static */ void
ip_GradientImageProcessor::compute_gradients(const IplImage * ipImage_32F1C,
    IplImage* opGradientX_32F1C, IplImage* opGradientY_32F1C,
    IplImage* opGradientMagnitude_32F1C, IplImage* opGradientAngle_32F1C,
    float iMagnitudeCap) {

    OpenCvPlus::cvp_ComputeGradients(ipImage_32F1C,
        opGradientX_32F1C, opGradientY_32F1C,
        opGradientMagnitude_32F1C, opGradientAngle_32F1C,
        GRADIENT_MAGNITUDE_THRESHOLD);

}


void
ip_GradientImageProcessor::
recompute_image(IplImage* image,
                const ip_RoiWindow& roi,
                boost::posix_time::ptime& time)
{
  // obtain the most recent image from the supplier and convert it
  IplImage* source_grayscale_image = mSourceImageProvider->image(roi);
  time = mSourceImageProvider->time();

  CvRect cvroi = cvRect(roi.m_iFirstColumn, roi.m_iFirstRow, roi.m_iWidth,
                        roi.m_iHeight);
  cvSetImageROI(source_grayscale_image, cvroi);
  OpenCvPlus::cvp_ComputeGradients(source_grayscale_image,
                                   mGradientXProvider->image_buffer(),
                                   mGradientYProvider->image_buffer(),
                                   mGradientMagnitudeProvider->image_buffer(),
                                   mGradientAngleProvider->image_buffer(),
                                   GRADIENT_MAGNITUDE_THRESHOLD);
  cvResetImageROI(source_grayscale_image);
}

} // namespace ImageProcessing
