// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseTrackerState2RoiConverter - converts ROI from/to tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h> // declaration of this

using namespace ImageProcessing;

//////////////////////////// LOCAL CONSTANTS /////////////////////////////////

// #define USE_UPPER_BODY_MODEL

static const float ROI_WIDTH_TO_SCALE_COEFF = 200;

namespace BICV {

/////////////////////////////// PUBLIC ///////////////////////////////////////

/* static */ bicv_HeadPoseParameters
HpParams2RoiConverter::roi2hpparams(const ip_RoiWindow& roi) {
    bicv_HeadPoseParameters result;
    result.m_Scale = static_cast<float>(roi.m_iWidth) /
            ROI_WIDTH_TO_SCALE_COEFF;
    result.m_Excentricity = static_cast<float>(roi.m_iHeight) / roi.m_iWidth;
    result.m_TranslationX = roi.m_iFirstColumn +
            static_cast<float>(roi.m_iWidth) / 2;
    result.m_TranslationY = roi.m_iFirstRow +
            static_cast<float>(roi.m_iHeight) / 2;
    return result;
} // roi2hpparams

/* static */ ip_RoiWindow
HpParams2RoiConverter::hpparams2roi(const bicv_HeadPoseParameters& hpparams) {
    ip_RoiWindow roi;
    roi.m_iWidth = static_cast<int>(
            hpparams.m_Scale * ROI_WIDTH_TO_SCALE_COEFF);
    roi.m_iHeight = static_cast<int>(
            hpparams.m_Excentricity * roi.m_iWidth);
    roi.m_iFirstColumn = static_cast<int>(hpparams.m_TranslationX -
            static_cast<float>(roi.m_iWidth) / 2);
    roi.m_iFirstRow = static_cast<int>(hpparams.m_TranslationY -
            static_cast<float>(roi.m_iHeight) / 2);
    return roi;
} // hpparams2roi

/* static */ bicv_TrackerParameters
HpParams2RoiConverter::roi2params(const ImageProcessing::ip_RoiWindow& roi) {
    bicv_TrackerParameters result;
    result.m_Scale = static_cast<float>(roi.m_iWidth) /
            ROI_WIDTH_TO_SCALE_COEFF;
    result.m_Excentricity = static_cast<float>(roi.m_iHeight) / roi.m_iWidth;
    result.m_TranslationX = roi.m_iFirstColumn +
            static_cast<float>(roi.m_iWidth) / 2;
    result.m_TranslationY = roi.m_iFirstRow +
            static_cast<float>(roi.m_iHeight) / 2;
    return result;
} // roi2params

/* static */ ImageProcessing::ip_RoiWindow
HpParams2RoiConverter::params2roi(const bicv_TrackerParameters& params) {
    ip_RoiWindow roi;
    roi.m_iWidth = static_cast<int>(
            params.m_Scale * ROI_WIDTH_TO_SCALE_COEFF);
    roi.m_iHeight = static_cast<int>(
            params.m_Excentricity * roi.m_iWidth);
    roi.m_iFirstColumn = static_cast<int>(params.m_TranslationX -
            static_cast<float>(roi.m_iWidth) / 2);
    roi.m_iFirstRow = static_cast<int>(params.m_TranslationY -
            static_cast<float>(roi.m_iHeight) / 2);
    return roi;
} // params2roi


///* static */ ImageProcessing::ip_RoiWindow
//HpParams2RoiConverter::face_roi2fcm_roi(
//        const ImageProcessing::ip_RoiWindow& face_roi) {
//
//    ImageProcessing::ip_RoiWindow fcm_roi(face_roi);
//
//#ifdef USE_UPPER_BODY_MODEL
//    fcm_roi.m_iFirstColumn -= fcm_roi.m_iWidth / 2;
//    fcm_roi.m_iWidth *= 2;
//    fcm_roi.m_iFirstRow -= fcm_roi.m_iHeight / 2;
//    fcm_roi.m_iHeight *= 3;
//    if (fcm_roi.m_iFirstColumn < 0) {
//        fcm_roi.m_iFirstColumn = 0;
//    }
//    if (fcm_roi.m_iFirstRow < 0) {
//        fcm_roi.m_iFirstRow = 0;
//    }
//#else
//    fcm_roi.m_iFirstColumn -= fcm_roi.m_iWidth / 2;
//    fcm_roi.m_iWidth *= 2;
//    fcm_roi.m_iFirstRow -= fcm_roi.m_iHeight / 2;
//    fcm_roi.m_iHeight *= 2;
//    if (fcm_roi.m_iFirstColumn < 0) {
//        fcm_roi.m_iFirstColumn = 0;
//    }
//    if (fcm_roi.m_iFirstRow < 0) {
//        fcm_roi.m_iFirstRow = 0;
//    }
//#endif
//
//    return fcm_roi;
//
//
//} // face_roi2fcm_roi
//
///* static */ ImageProcessing::ip_RoiWindow
//HpParams2RoiConverter::fcm_roi2face_roi(
//        const ImageProcessing::ip_RoiWindow& fcm_roi) {
//
//    ImageProcessing::ip_RoiWindow face_roi(fcm_roi);
//
//#ifdef USE_UPPER_BODY_MODEL
//    face_roi.m_iFirstColumn += face_roi.m_iWidth / 4;
//    face_roi.m_iWidth /= 2;
//    face_roi.m_iFirstRow += face_roi.m_iHeight / 6;
//    face_roi.m_iHeight /= 3;
//#else
//    face_roi.m_iFirstColumn += face_roi.m_iWidth / 4;
//    face_roi.m_iWidth /= 2;
//    face_roi.m_iFirstRow += face_roi.m_iHeight / 4;
//    face_roi.m_iHeight /= 2;
//#endif
//
//    return face_roi;
//
//} // face_roi2fcm_roi

} // namespace BICV
