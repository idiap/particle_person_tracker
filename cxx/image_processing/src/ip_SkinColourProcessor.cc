// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SkinColourProcessor - class to extract skin colour pixels from images
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <cstdio>

// PROJECT INCLUDES
#include <opencvplus/cvp_skin_color.h>                // skin colour estimation

// LOCAL INCLUDES
#include <image_processing/ip_SkinColourProcessor.h>  // declaration of this

using namespace std;
using namespace OpenCvPlus;

namespace ImageProcessing {

// threshold of skin for training set
//#define SKIN_THRESH_TRAIN 4

// parameter of gaussian skin color model
//#define ALPHA_COLOR 0.7

// parameter of gaussian skin color model
#define USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE

//////////////////////////// LOCAL CONSTANTS /////////////////////////////////

// threshold of skin for generic videos
static const float SKIN_COLOUR_THRESHOLD = 8;
static const float SKIN_COLOUR_THRESHOLD_UPDATED = 5;
static const float ALPHA_COLOUR = 0.7;

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_SkinColourProcessor::ip_SkinColourProcessor(
        ip_ImageProvider * img_provider,
        FaceColorModel::FaceColorModel * face_colour_model,
        float skin_mask_value) :
        mSourceImageProvider(img_provider),
        mFaceColourModel(face_colour_model),
        mSkinMaskValue(skin_mask_value),
        mSkinColourThreshold(SKIN_COLOUR_THRESHOLD),
        mSkinColourModel(cvp_SkinColourModel::DEFAULT) {

    const int width =  img_provider->image_buffer()->width;
    const int height = img_provider->image_buffer()->height;

    IplImage * skin_colour_mask_image = cvCreateImage(cvSize(width, height),
            IPL_DEPTH_8U, 1);
    image(skin_colour_mask_image);
    mTempImage1 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    mTempImage2 = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    mCompOpRes = cvCreateMat(height, width, CV_8UC1);

    // skin colour processor buffers
    m_HistogramColourBuffer = cvCreateImage(cvSize(width, height),
            IPL_DEPTH_32S, 1);
    m_ConvertedColourBuffer = cvCreateImage(cvSize(width, height),
            IPL_DEPTH_32F, 3);

}

/* virtual */
ip_SkinColourProcessor::~ip_SkinColourProcessor() {
    cvReleaseImage(&m_ConvertedColourBuffer);
    cvReleaseImage(&m_HistogramColourBuffer);

    cvReleaseMat(&mCompOpRes);

    cvReleaseImage(&mTempImage2);
    cvReleaseImage(&mTempImage1);
}

/* static */ void
ip_SkinColourProcessor::set_colour_model(const cvp_SkinColourModel& model) {
//    cout << "skin parameters before set: " << mSkinColourModel << endl;
    mSkinColourModel = model;
//    cout << "skin parameters after set: " << mSkinColourModel << endl;
    set_threshold(SKIN_COLOUR_THRESHOLD_UPDATED);
} // set_colour_model

/* static */ cvp_SkinColourModel
ip_SkinColourProcessor::get_colour_model() {
    return mSkinColourModel;
} // get_colour_model

/* static */ void
ip_SkinColourProcessor::reset_colour_model() {
    mSkinColourThreshold = SKIN_COLOUR_THRESHOLD;
    mSkinColourModel = cvp_SkinColourModel::DEFAULT;
} // reset_colour_model

/* static */ void
ip_SkinColourProcessor::set_threshold(float threshold) {
//    cout << "skin threshold before set: " << mSkinColourThreshold << endl;
    mSkinColourThreshold = threshold;
//    cout << "skin threshold after set: " << mSkinColourThreshold << endl;
} // set_threshold

/* static */ void
ip_SkinColourProcessor::update_colour_model(const cvp_SkinColourModel& model) {
//    cout << "skin parameters update with model " << model << endl;
//    cout << "skin parameters before update: " << mSkinColourModel << endl;
    mSkinColourModel = ALPHA_COLOUR * model +
            (1 - ALPHA_COLOUR) * mSkinColourModel;
//    cout << "skin parameters after update: " << mSkinColourModel << endl;
    set_threshold(SKIN_COLOUR_THRESHOLD_UPDATED);
} // update_colour_model

/* static */ void
ip_SkinColourProcessor::update_colour_model(const cvp_SkinColourModel& model,
        float rate) {
//    cout << "skin parameters update with rate " << rate << endl;
//    cout << "skin parameters before update: " << mSkinColourModel << endl;
    mSkinColourModel = rate * model + (1 - rate) * mSkinColourModel;
//    cout << "skin parameters after update: " << mSkinColourModel << endl;
    set_threshold(SKIN_COLOUR_THRESHOLD_UPDATED);
} // update_colour_model

/* static */ void
ip_SkinColourProcessor::update_colour_model(IplImage * image, IplImage * mask,
        const ip_RoiWindow& roi, float rate) {

    unsigned counter = 0;
    float sum_green = 0;
    float sum_green_sqr = 0;
    float sum_red = 0;
    float sum_red_sqr = 0;
    float sum_green_red = 0;

    float green = 0;
    float red = 0;
    float sum_pixel = 0;

//    cout << "skin parameters update with image " << endl;
//    cout << "skin parameters before update: " << mSkinColourModel << endl;

    for (int row = roi.m_iFirstRow; (row < image->height) &&
        (row < roi.m_iFirstRow + roi.m_iHeight); ++row) {
        for (int col = roi.m_iFirstColumn; (col < image->width) &&
            (col < roi.m_iFirstColumn + roi.m_iWidth); ++col) {
            if (CV_IMAGE_ELEM(mask, uchar, row, col) > 0) {
                sum_pixel = CV_IMAGE_ELEM(image, uchar, row, col * 3);
                sum_pixel += (green = CV_IMAGE_ELEM(image, uchar, row, col * 3 + 1));
                sum_pixel += (red   = CV_IMAGE_ELEM(image, uchar, row, col * 3 + 2));
                green /= sum_pixel;
                red   /= sum_pixel;
                sum_green += green;
                sum_green_sqr += green * green;
                sum_red += red;
                sum_red_sqr += red * red;
                sum_green_red += green * red;
                ++counter;
            }
        }
    }

//    cout << "counter " << counter << endl;

    if (counter > 0) {
        cvp_SkinColourModel model;
        model.mGreenMean = sum_green / counter;
        model.mRedMean = sum_red / counter;
        model.mGreenVar = sum_green_sqr / counter - model.mGreenMean *
                model.mGreenMean;
        model.mRedVar = sum_red_sqr / counter - model.mRedMean * model.mRedMean;
        model.mGreenRedCovar = sum_green_red / counter - model.mGreenMean *
                model.mRedMean;
        mSkinColourModel = 0.7 * (rate * model + (1 - rate) * mSkinColourModel) +
                0.3 * cvp_SkinColourModel::DEFAULT;
    }

//    cout << "skin parameters after update: " << mSkinColourModel << endl;
    set_threshold(SKIN_COLOUR_THRESHOLD_UPDATED);
}

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_SkinColourProcessor::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {
    IplImage* source_image = mSourceImageProvider->image();
    time = mSourceImageProvider->time();
    // assume the source image is read by opencv
    // the source colour space is then BGR

    #ifdef USE_FACE_COLOUR_MODEL_NOT_NORMALIZED_RG_SPACE
    compute_skin_mask_using_face_colour_model(source_image, buffer_image,
            mTempImage1, mTempImage2,
            m_ConvertedColourBuffer, m_HistogramColourBuffer, roi);
    #else
    OpenCvPlus::cvp_ComputeSkinMask(source_image, buffer_image,
        mTempImage1, mTempImage2, mSkinColourModel, mSkinColourThreshold,
        mSkinMaskValue, roi);
    #endif

//    static unsigned count = 0;
//    ostringstream oss;
//    oss << "skin_mask_image" << count << ".jpg";
//    cvSaveImage(oss.str().c_str(), buffer_image);
//    cout << oss.str() << endl << flush;
//    oss.str("");
//    oss << "source_image" << count << ".jpg";
//    cvSaveImage(oss.str().c_str(), source_image);
//    cout << oss.str() << endl << flush;
//    count++;
} // recompute_image

/////////////////////////////// PRIVATE ////////////////////////////////////

void ip_SkinColourProcessor::compute_skin_mask_using_face_colour_model(
        IplImage const* ipInputImage, IplImage* opOutputImage,
        IplImage* opTempImage1, IplImage* opTempImage2,
        IplImage* opConvertedColourBuffer, IplImage* opHistogramColourBuffer,
        const ImageProcessing::ip_RoiWindow& roi) {

    // DEBUG-START
    // assert the provided parameters are consistent with the internal logic
    assert((ipInputImage->nChannels == 3) &&
           (ipInputImage->depth == IPL_DEPTH_8U));
    assert((opOutputImage->nChannels == 1) &&
           (opOutputImage->depth == IPL_DEPTH_8U) &&
           (opOutputImage->width == ipInputImage->width) &&
           (opOutputImage->height == ipInputImage->height));
    assert((opTempImage1->nChannels == 1) &&
           (opTempImage1->depth == IPL_DEPTH_8U) &&
           (opTempImage1->width == ipInputImage->width) &&
           (opTempImage1->height == ipInputImage->height));
    assert((opTempImage2->nChannels == 1) &&
           (opTempImage2->depth == IPL_DEPTH_8U) &&
           (opTempImage2->width == ipInputImage->width) &&
           (opTempImage2->height == ipInputImage->height));
    assert((opConvertedColourBuffer->nChannels == 3) &&
           (opConvertedColourBuffer->depth == IPL_DEPTH_32F) &&
           (opConvertedColourBuffer->width == ipInputImage->width) &&
           (opConvertedColourBuffer->height == ipInputImage->height));
    assert((opHistogramColourBuffer->nChannels == 1) &&
           (opHistogramColourBuffer->depth == IPL_DEPTH_32S) &&
           (opHistogramColourBuffer->width == ipInputImage->width) &&
           (opHistogramColourBuffer->height == ipInputImage->height));
    assert((roi.m_iFirstColumn >= 0) && (roi.m_iFirstRow >= 0) &&
           (roi.m_iHeight >= 0) && (roi.m_iWidth >= 0));
    // DEBUG-END

    ImageProcessing::ip_RoiWindow global_roi;
    global_roi.m_iFirstColumn = 0;
    global_roi.m_iFirstRow = 0;
    global_roi.m_iWidth = ipInputImage->width;
    global_roi.m_iHeight = ipInputImage->height;

    mFaceColourModel->cache_probability_maps(ipInputImage,
            ip_RoiWindow::to_CvRect(roi));
    const FaceColorModel::FaceColorModel::PimFeatureType * probability_maps =
            mFaceColourModel->cached_probability_maps();

    // pixels = white
    cvSet(opTempImage1, cvScalarAll(255));
    // compute opTempImage1 = [hair > skin] & [clothes > skin] & [bg > skin]
    // pixels [hair >= skin] = black
    cvCmp(
        probability_maps->map(FaceColorModel::FCM_CHANNEL_SKIN),
        probability_maps->map(FaceColorModel::FCM_CHANNEL_HAIR),
        mCompOpRes, CV_CMP_GT);
    cvAnd(mCompOpRes, opTempImage1, opTempImage1);
    // pixels [clothes >= skin] = black
    cvCmp(
        probability_maps->map(FaceColorModel::FCM_CHANNEL_SKIN),
        probability_maps->map(FaceColorModel::FCM_CHANNEL_CLOTHES),
        mCompOpRes, CV_CMP_GT);
    cvAnd(mCompOpRes, opTempImage1, opTempImage1);
//    // pixels [background >= skin] = black
    cvCmp(
        probability_maps->map(FaceColorModel::FCM_CHANNEL_SKIN),
        probability_maps->map(FaceColorModel::FCM_CHANNEL_BACKGROUND),
        mCompOpRes, CV_CMP_GT);
    cvAnd(mCompOpRes, opTempImage1, opTempImage1);

    // set all pixels to black outside the ROI
    cvZero(opTempImage2);
    cvSetImageROI(opTempImage2, ip_RoiWindow::to_CvRect(roi));
    cvSet(opTempImage2, cvScalarAll(255));
    cvResetImageROI(opTempImage2);
    cvAnd(opTempImage2, opTempImage1, opTempImage1);

//     invert resulting image
//    cvNot(opTempImage1, opTempImage1);

    // TODO: Do we really need to use morphological operations on masks?
    // Why not do a simple copy ?
//    cvCopy(opTempImage1, opOutputImage);

    // Denoise the skin mask with an erode/dilate operation. This
    // firstly erodes, to remove lonely pixels (i.e. where the skin
    // mask is active); and then dilates, to add a 1-pixel border
    // around the mask.
    cvMorphologyEx(opTempImage1, opOutputImage, opTempImage2, NULL,
            CV_MOP_OPEN, 1);

} // compute_skin_mask_using_face_colour_model

} // namespace ImageProcessing
