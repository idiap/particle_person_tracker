// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ColourSegmentationLikelihoodModel - likelihood model for colour
//                                          segmentation tracker
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <numeric>
#include <boost/math/special_functions/fpclassify.hpp>    // for isfinite()

// LOCAL INCLUDES
#include <bayes_image/bicv_ColourSegmentationLikelihoodModel.h>       // declaration of this
#include <bayes_image/bicv_Exceptions.h>                              // exceptions
#include <bayes_image/bicv_HeadPoseTrackerState2RoiConverter.h>       // roi2state converter

using namespace ImageProcessing;
using namespace std;

namespace BICV {

bicv_ColourSegmentationLikelihoodModel::bicv_ColourSegmentationLikelihoodModel(
        FaceColorModel::FaceColorModel * face_colour_model) :
        m_FaceColourModel(face_colour_model) {
    m_BufferImage = cvCreateImage(cvSize(
            face_colour_model->current_pim()->data().width(),
            face_colour_model->current_pim()->data().height()),
            IPL_DEPTH_8U, 3);
} // bicv_ColourSegmentationLikelihoodModel

/* virtual */
bicv_ColourSegmentationLikelihoodModel::
~bicv_ColourSegmentationLikelihoodModel() {
    cvReleaseImage(&m_BufferImage);
} // ~bicv_ColourSegmentationLikelihoodModel

/* virtual */ bicv_ColourSegmentationLikelihoodModel::observation_type
bicv_ColourSegmentationLikelihoodModel::sample(rng_engine_type& rng,
        const state_type& state) const {
    throw bicv_Exception("Not implemented!");
}

/* virtual */ bicv_ColourSegmentationLikelihoodModel::value_type
bicv_ColourSegmentationLikelihoodModel::evaluate(const state_type& state,
        const observation_type& obs) const {

    if (!((obs.m_Roi.m_iFirstColumn >= 0) && (obs.m_Roi.m_iFirstRow >= 0))) {
        cout << obs.m_Roi << endl;
        assert((obs.m_Roi.m_iFirstColumn >= 0) && (obs.m_Roi.m_iFirstRow >= 0));
    }

    CvRect cvRoi = cvRect(obs.m_Roi.m_iFirstColumn, obs.m_Roi.m_iFirstRow,
            obs.m_Roi.m_iWidth, obs.m_Roi.m_iHeight);

    float lhood_score = 0;
//    float lhood_score =  1.0f / (1.0f + exp(
//            -0.001 * m_FaceColourModel->score(obs.m_Image, cvRoi)));
    return lhood_score;

//    try {
//        cvSetImageROI(obs.m_Image, cvRoi);
//        cvResize(obs.m_Image, m_BufferImage, CV_INTER_LINEAR);
//        cvResetImageROI(obs.m_Image);
//    } catch (...) {
//        cout << "Exception caught in likelihood model " << obs.m_Roi
//             << endl << flush;
//    }
//
//    const int LOGLHOOD_MAP_STEP = FaceColorModel::FCM_SCALED_SIZE *
//        FaceColorModel::FCM_SCALED_SIZE;
//
//    // Storage for the log-likelihoods indexed as [channel][x][y]
//    vector<FaceColorModel::fcm_real> loglhood_map(
//            FaceColorModel::FCM_NUM_CHANNELS * LOGLHOOD_MAP_STEP, 0);
//
//    const IplImage * pBufferImage = m_BufferImage;
//
//    m_FaceColourModel->channel_log_likelihood(pBufferImage, &loglhood_map[0]);
//
//    // Pointer to the posterior belief over the channel of each pixel.
//    // The array is indexed as [channel][x][y] and each entry is a
//    // probability such that the sum over channels equals 1 for each (x,y).
//    const FaceColorModel::fcm_real* pim_posterior =
//            m_FaceColourModel->get_class_posterior();
//
//    // compute probabilities instead of log-likelihoods
//    // TODO: use Intel MKL library for efficient vector computations
//    vector<FaceColorModel::fcm_real> lhood_map(
//            FaceColorModel::FCM_NUM_CHANNELS * LOGLHOOD_MAP_STEP, 0);
//
//    vector<FaceColorModel::fcm_real>::const_iterator ii_llhood_begin =
//            loglhood_map.begin();
//    vector<FaceColorModel::fcm_real>::const_iterator ii_llhood_end =
//            loglhood_map.end();
//    vector<FaceColorModel::fcm_real>::const_iterator ii_llhood =
//            ii_llhood_begin;
//    vector<FaceColorModel::fcm_real>::const_iterator ii_lhood_begin =
//            lhood_map.begin();
//    vector<FaceColorModel::fcm_real>::const_iterator ii_lhood_end =
//            lhood_map.end();
//    vector<FaceColorModel::fcm_real>::iterator ii_lhood =
//            lhood_map.begin();
//
//    FaceColorModel::fcm_real llhood_val;
//    FaceColorModel::fcm_real lhood_val;
//    const FaceColorModel::fcm_real * pPrior = pim_posterior;
//    while (ii_llhood != ii_llhood_end) {
//        llhood_val = *ii_llhood++;
//        lhood_val = exp(llhood_val) * (*pPrior++);
////        if (!boost::math::isfinite(lhood_val)) {
////            int offset = ii_llhood - ii_llhood_begin - 1;
////            cout << "Infinite likelihood value, channel: " <<
////                offset / LOGLHOOD_MAP_STEP << ", coords: (" <<
////                (offset % LOGLHOOD_MAP_STEP) / FaceColorModel::FCM_NUM_CHANNELS
////                << "," <<
////                (offset % LOGLHOOD_MAP_STEP) % FaceColorModel::FCM_NUM_CHANNELS
////                << "), loglhood=" << llhood_val  <<
////                ", lhood=" << lhood_val << endl << flush;
////            assert(boost::math::isfinite(lhood_val));
////        }
//
////        cout << val << " ";
//        *ii_lhood++ = lhood_val;
//    }
//
//    // compute result as marginal log-likelihood
//    float marg_lhood = 1;
//    float v = 0;
//    ii_lhood_end = ii_lhood_begin + LOGLHOOD_MAP_STEP;
//    for (ii_lhood = lhood_map.begin(); ii_lhood != ii_lhood_end; ++ii_lhood) {
//        v = 0;
//        for (int jj = 0; jj < FaceColorModel::FCM_NUM_CHANNELS; ++jj) {
//            v += *(ii_lhood + (jj * LOGLHOOD_MAP_STEP));
//        }
//        v = log(v);
//        marg_lhood += v;
//    }
//
//    assert(boost::math::isfinite(marg_lhood));
//
//    return marg_lhood;
}

}
