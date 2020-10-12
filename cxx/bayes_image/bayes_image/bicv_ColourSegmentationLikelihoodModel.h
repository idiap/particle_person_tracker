// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_ColourSegmentationLikelihoodModel - likelihood model for tracker
//                                          based on segmentation
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_COLOURSEGMENTATIONLIKELIHOODMODEL_H__
#define __BICV_COLOURSEGMENTATIONLIKELIHOODMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include <cv.h>                                           // for IplImage

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <opencvplus/FaceColorModel.h>                    // face color model

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralTrackerState.h>         // state
#include <bayes_image/bicv_PatchObservation.h>            // patch observation

namespace BICV {

/// @brief Parameters of likelihood model for general tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011
struct bicv_ColourSegmentationLikelihoodModelParameters {
};

/// @brief Likelihood model for segmentation-based tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_ColourSegmentationLikelihoodModel :
    public BayesFilter::bf_ConditionalDistr
    <bicv_TrackerState, bicv_PatchObservation, boost::mt19937> {

public:

    bicv_ColourSegmentationLikelihoodModel(
            FaceColorModel::FaceColorModel * face_colour_model);

    virtual ~bicv_ColourSegmentationLikelihoodModel();

    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

private:

    FaceColorModel::FaceColorModel * m_FaceColourModel; // face colour model
    IplImage * m_BufferImage;

};

}

#endif // __BICV_COLOURSEGMENTATIONLIKELIHOODMODEL_H__
