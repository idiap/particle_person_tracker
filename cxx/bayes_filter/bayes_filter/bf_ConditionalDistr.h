/**
 * @file cxx/bayes_filter/bayes_filter/bf_ConditionalDistr.h
 * @date 04 March 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief General conditional distribution template class
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BF_CONDITIONALDISTR_H__
#define __BF_CONDITIONALDISTR_H__

// SYSTEM INCLUDES
#include <boost/math/policies/policy.hpp>        // boost policies

namespace BayesFilter {

/// @brief Class to represent a general conditional distribution
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    04.03.2011

template<typename StateType, typename ObservationType, typename RngEngine,
    typename RealType = float,
    typename Policy = boost::math::policies::policy<> >
class bf_ConditionalDistr {

public:

    typedef StateType state_type;
    typedef ObservationType observation_type;
    typedef RngEngine rng_engine_type;
    typedef RealType value_type;
    typedef Policy policy_type;

    virtual ~bf_ConditionalDistr() {};

    virtual ObservationType sample(RngEngine& rng,
        const StateType& state) const = 0;

    virtual RealType evaluate(const StateType& state,
        const ObservationType& obs) const = 0;

};

} // namespace BayesFilter

#endif // __BF_CONDITIONALDISTR_H__
