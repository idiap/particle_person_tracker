/**
 * @file cxx/bayes_image/bayes_image/bicv_StateImageLikelihood_FCM.h
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

#ifndef __BICV_STATEIMAGELIKELIHOOD_FCM_H__
#define __BICV_STATEIMAGELIKELIHOOD_FCM_H__

// PROJECT INCLUDES
#include <opencvplus/FaceColorModel.h>                    // face color model

// LOCAL INCLUDES
#include "bicv_StateImageLikelihood.h"                    // image likelihood

namespace BICV {

/**
 * Likelihood model based on trained face colour pattern
 */
class bicv_StateImageLikelihood_FCM : public bicv_StateImageLikelihood {

public:

    // LIFECYCLE

    /**
     * Constructor
     * @param face_colour_model Face colour model with trained color patterns
     */
    bicv_StateImageLikelihood_FCM(
            FaceColorModel::FaceColorModel * face_colour_model);

    /**
     * Destructor
     */
    ~bicv_StateImageLikelihood_FCM();

    /**
     * Overridden base class method to sample observations based on state
     */
    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    /**
     * Overridden base class method to measure observation likelihood given state
     */
    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

private:

    FaceColorModel::FaceColorModel * m_FaceColourModel;
    FaceColorModel::FaceColorModel::PimFeatureType * m_FcmFeature;

};

} // namespace BICV

#endif // __BICV_STATEIMAGELIKELIHOOD_FCM_H__
