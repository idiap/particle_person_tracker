// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_GeneralDynamicModel - dynamic model for general tracker state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_GENERALDYNAMICMODEL_H__
#define __BICV_GENERALDYNAMICMODEL_H__

// SYSTEM INCLUDES
#include <boost/random/mersenne_twister.hpp>              // RNG
#include <vector>                                         // STL vector

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>             // likelihood
#include <bayes_filter/bf_DiscreteDistribution.h>         // discrete distr

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralTrackerState.h>         // state

namespace BICV {

/// @brief Parameters of dynamic model for general tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011
struct bicv_GeneralDynamicModelParameters {
    bicv_TrackerParameters m_StdDev;   // independent std devs for all parameters
    bicv_TrackerParameters m_ArCur;    // AR for all parameters, for cur value
    bicv_TrackerParameters m_ArPrev;   // AR for all parameters, for prev value
    bicv_TrackerParameters m_MotConst; // constant motion part
};

/// @brief Dynamic model for general tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

class bicv_GeneralDynamicModel : public BayesFilter::bf_ConditionalDistr
    <bicv_TrackerState, bicv_TrackerState, boost::mt19937> {

public:

    typedef BayesFilter::bf_ConditionalDistr<
        bicv_TrackerState, bicv_TrackerState, boost::mt19937>
        distribution_type;

    /// @param num_poses Number of possible different poses for the tracker
    bicv_GeneralDynamicModel(const bicv_GeneralDynamicModelParameters& params,
        unsigned num_poses);

    const bicv_GeneralDynamicModelParameters& parameters();

    void parameters(const bicv_GeneralDynamicModelParameters& params);

    virtual observation_type sample(rng_engine_type& rng,
        const state_type& state) const;

    virtual value_type evaluate(const state_type& state,
        const observation_type& obs) const;

private:

    bicv_GeneralDynamicModelParameters m_Params;
    // number of different poses for tracker
    unsigned m_NumPoses;
    std::vector<std::pair<unsigned, float> > m_PoseDistributionElements;
    const int m_Label;
    static int m_LabelCount;

};

}

#endif // __BICV_GENERALDYNAMICMODEL_H__
