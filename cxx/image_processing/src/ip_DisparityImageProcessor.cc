/**
 * @file cxx/image_processing/src/ip_DisparityImageProcessor.cc
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

// SYSTEM INCLUDES
#include <cmath>

// PROJECT INCLUDES
#include <opencvplus/cvp_IplDepthTraits.h>

// LOCAL INCLUDES
#include <image_processing/ip_DisparityImageProcessor.h> // declaration of this
#include <image_processing/ip_StereoImageProvider.h>     // stereo provider

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_DisparityImageProcessor::ip_DisparityImageProcessor(
        ip_ImageProvider * img_provider,
        const string& calibration_file) :
        m_SourceImageProvider(img_provider),
        m_StereoMatcher(0),
        m_ValidDisparityFlag(false) {

    int width = 320;
    int height = 240;

    try {
        m_StereoMatcher = new stereo::StereoMatcher(calibration_file);
    } catch (...) {
        cerr << "Error instantiating stereo matcher (calibration file="
             << calibration_file << "). Proceeding without disparity/depth..."
             << endl;
        m_StereoMatcher = 0;
    }

    if (m_StereoMatcher) {
        width = m_StereoMatcher->width();
        height = m_StereoMatcher->height();
        try {
            StereoParameters sp;
            sp.save_float = 1;
            sp.save_raw = 1;
            sp.maxd = 255;
            sp.mind = -10;
            sp.fillholes = true;
            sp.thr = 0.5;

            m_StereoMatcher->setStereoParams(sp);
        } catch (...) {
            cerr << "Error setting stereo matcher parameters. "
                 << "Proceeding without disparity/depth..."
                 << endl;
            delete m_StereoMatcher;
            m_StereoMatcher = 0;
        }
    }

    // create the flipped image using the source image parameters

    m_DisparityImage_32F1 =
            cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    m_DepthImage_64F3 = cvCreateImage(cvSize(width, height), IPL_DEPTH_64F, 3);
    m_MaskImage_8U1 =
            cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    m_ResizedLeftImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    m_ResizedRightImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

    image(m_DisparityImage_32F1);

}

/* virtual */ ip_DisparityImageProcessor::~ip_DisparityImageProcessor() {
    // don't release disparity image - should be released by parent class!
    cvReleaseImage(&m_ResizedLeftImage);
    m_ResizedLeftImage = 0;
    cvReleaseImage(&m_ResizedRightImage);
    m_ResizedRightImage = 0;

    cvReleaseImage(&m_DepthImage_64F3);
    m_DepthImage_64F3 = 0;
    cvReleaseImage(&m_MaskImage_8U1);
    m_MaskImage_8U1 = 0;
    if (m_StereoMatcher) {
        delete m_StereoMatcher;
    }
} // ~ip_DisparityImageProcessor

bool ip_DisparityImageProcessor::has_valid_disparity() const {
    return m_ValidDisparityFlag;
} // has_valid_disparity

CvScalar ip_DisparityImageProcessor::scene_coordinates_3D_depth(
        const ip_RoiWindow& roi) const {
    CvScalar result = cvScalarAll(0);

    const int start_row = roi.m_iFirstRow;
    const int end_row = roi.m_iFirstRow + roi.m_iHeight;
    const int start_col = roi.m_iFirstColumn;
    const int end_col = roi.m_iFirstColumn + roi.m_iWidth;

    typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_64F>::type double64;

    const unsigned char * pInputCurrentStart =
            (const unsigned char *) m_DepthImage_64F3->imageData +
            roi.m_iFirstRow * m_DepthImage_64F3->widthStep +
            roi.m_iFirstColumn * 3 * sizeof(double64);

    const unsigned char * pMaskCurrentStart =
            (const unsigned char *) m_MaskImage_8U1->imageData +
            roi.m_iFirstRow * m_MaskImage_8U1->widthStep +
            roi.m_iFirstColumn;

    int pixel_count = 0;
    double mean_x = 0;
    double mean_y = 0;
    double mean_z = 0;

    // compute mean location on valid ROI pixels
    for(int row = start_row; row < end_row; ++row) {
        const double64 * pInputCurrent = (const double64 *) pInputCurrentStart;
        const unsigned char * pMaskCurrent = pMaskCurrentStart;
        for (int col = start_col; col < end_col; ++col) {
            if (*pMaskCurrent++) {
                ++pixel_count;
                mean_x += *pInputCurrent++;
                mean_y += *pInputCurrent++;
                mean_z += *pInputCurrent++;
            }
        }
        // move to the next row in the input and output images
        pInputCurrentStart += m_DepthImage_64F3->widthStep;
        pMaskCurrentStart += m_MaskImage_8U1->widthStep;
    }

    if (pixel_count > 1) {
        mean_x /= pixel_count;
        mean_y /= pixel_count;
        mean_z /= pixel_count;
    }
    result.val[0] = mean_x;
    result.val[1] = mean_y;
    result.val[2] = mean_z;

    return result;
} // scene_coordinates_3D_depth

CvScalar ip_DisparityImageProcessor::scene_coordinates_3D_disparity(
        const ip_RoiWindow& roi) const {

    const int start_row = roi.m_iFirstRow;
    const int end_row = roi.m_iFirstRow + roi.m_iHeight;
    const int start_col = roi.m_iFirstColumn;
    const int end_col = roi.m_iFirstColumn + roi.m_iWidth;

    typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;

    const unsigned char * pInputCurrentStart =
            (const unsigned char *) m_DisparityImage_32F1->imageData +
            roi.m_iFirstRow * m_DisparityImage_32F1->widthStep +
            roi.m_iFirstColumn * sizeof(float32);

    const unsigned char * pMaskCurrentStart =
            (const unsigned char *) m_MaskImage_8U1->imageData +
            roi.m_iFirstRow * m_MaskImage_8U1->widthStep +
            roi.m_iFirstColumn;

    int pixel_count = 0;
    CvScalar mean_uvd;
    mean_uvd.val[0] = roi.m_iFirstColumn + roi.m_iWidth / 2;
    mean_uvd.val[1] = roi.m_iFirstRow + roi.m_iHeight / 2;
    double mean_d = 0;

    float32 disp;

    // compute mean location on valid ROI pixels
    for(int row = start_row; row < end_row; ++row) {
        const float32 * pInputCurrent = (const float32 *) pInputCurrentStart;
        const unsigned char * pMaskCurrent = pMaskCurrentStart;
        for (int col = start_col; col < end_col; ++col) {
            disp = *pInputCurrent++;
            if (*pMaskCurrent++) {
                ++pixel_count;
                mean_d += disp;
            }
        }
        // move to the next row in the input and output images
        pInputCurrentStart += m_DisparityImage_32F1->widthStep;
        pMaskCurrentStart += m_MaskImage_8U1->widthStep;
    }

    if (pixel_count > 1) {
        mean_d /= pixel_count;
    }
    mean_uvd.val[2] = mean_d;

    CvScalar mean_xyz = m_StereoMatcher->computeXYZ(mean_uvd);

    return mean_xyz;
} // scene_coordinates_3D_disparity

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void
ip_DisparityImageProcessor::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

//    cout << "Disparity Processor: recompute image" << endl;

    /* IplImage* source_image = */ m_SourceImageProvider->image();
    time = m_SourceImageProvider->time();

    // if there was an error initializing stereo matcher, return
    if (!m_StereoMatcher) {
//        cerr << "Stereo matcher is null" << endl;
        return;
    }

    // determine if the source image provider is stereo provider
    IplImage * left_image;
    IplImage * right_image;
    if (m_SourceImageProvider->id() == IP_IMG_PROVIDER_STEREO) {
        ip_StereoImageProvider * stereo_provider =
                dynamic_cast<ip_StereoImageProvider*>(m_SourceImageProvider);
        left_image = stereo_provider->left_image();
        right_image = stereo_provider->right_image();
        m_ValidDisparityFlag = true;
    } else {
        // cerr << m_SourceImageProvider->id() << endl;
        m_ValidDisparityFlag = false;
    }

    if (m_ValidDisparityFlag) {
        IplImage * left_image_in = left_image;
        IplImage * right_image_in = right_image;
        if (m_ResizedLeftImage->width != left_image->width) {
            cvResize(left_image, m_ResizedLeftImage);
            cvResize(right_image, m_ResizedRightImage);
            left_image_in = m_ResizedLeftImage;
            right_image_in = m_ResizedRightImage;
        }
        cvZero(m_MaskImage_8U1);
        cvZero(m_DepthImage_64F3);
        cvZero(m_DisparityImage_32F1);

        cv::Mat disparity_mat(m_DisparityImage_32F1);
        cv::Mat mask_mat(m_MaskImage_8U1);
        m_StereoMatcher->computeDisparityMap(left_image_in, right_image_in,
            disparity_mat, mask_mat);
//        cv::Mat depth_mat(m_DepthImage_64F3);
//        cv::Mat mask_mat(m_MaskImage_8U1);
//        m_StereoMatcher->computeXYZMap(disparity_mat, depth_mat, mask_mat);
    }

} // recompute_image

} // namespace ImageProcessing
