/**
 * @file cxx/bayes_image/src/bicv_StateImageLikelihood_FCM.cc
 * @date 01 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face colour model-based likelihood
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <bayes_image/bicv_Exceptions.h>                 // exceptions
#include <cv.h>
#include <highgui.h>
#include <sstream>

// LOCAL INCLUDES
#include <bayes_image/bicv_StateImageLikelihood_FCM.h>   // image likelihood
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>

using namespace ImageProcessing;
using namespace std;

namespace BICV {

/////////////////////////////// PUBLIC ///////////////////////////////////////

bicv_StateImageLikelihood_FCM::bicv_StateImageLikelihood_FCM(
    FaceColorModel::FaceColorModel * face_colour_model) :
    m_FaceColourModel(face_colour_model), m_FcmFeature(0) {

    m_FcmFeature = new FaceColorModel::FaceColorModel::PimFeatureType(
        face_colour_model->current_pim()->data());

} // bicv_StateImageLikelihood_FCM

bicv_StateImageLikelihood_FCM::~bicv_StateImageLikelihood_FCM() {
} // ~bicv_StateImageLikelihood_FCM

/* virtual */ bicv_StateImageLikelihood_FCM::observation_type
bicv_StateImageLikelihood_FCM::sample(rng_engine_type& rng,
    const state_type& state) const {
    throw bicv_Exception("Not implemented!");
} // sample

/* virtual */ bicv_StateImageLikelihood_FCM::value_type
bicv_StateImageLikelihood_FCM::evaluate(const state_type& state,
    const observation_type& obs) const {

    ip_RoiWindow roi = HpParams2RoiConverter::params2roi(state.m_ParamsCur);
    unsigned idx = state.m_ParamsCur.m_PoseIndex;
    CvRect cvRoi =
            m_FaceColourModel->face_roi2fcm_roi(ip_RoiWindow::to_CvRect(roi),
            idx);

    m_FaceColourModel->compute_feature_on_cached_probability_maps(cvRoi,
        m_FcmFeature);
    m_FcmFeature->normalise_channels();

    return OpenCvPlus::distance_correlation_global(*m_FcmFeature,
        m_FaceColourModel->current_pim(idx)->data());

}

} // namespace BICV
