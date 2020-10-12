// Copyright (c) 2010-2020 Idiap Research Institute
//
// bf_ConditionalDistrMixture - class to represent a mixture of distributions
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)

#ifndef __BF_CONDITIONALDISTRIBUTIONMIXTURE_H__
#define __BF_CONDITIONALDISTRIBUTIONMIXTURE_H__

// SYSTEM INCLUDES
#include <utility>                               // for std::pair
#include <vector>                                // for std::vector
#include <numeric>                               // for accumulate
#include <boost/lambda/lambda.hpp>               // boost lambda exprs
#include <boost/foreach.hpp>                     // boost foreach loop

// LOCAL INCLUDES
#include "bf_ConditionalDistr.h"

namespace BayesFilter {

/// @brief Class to represent a mixture of distributions
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    06.09.2011

template<typename StateType, typename ObservationType, typename RngEngine,
    typename RealType = float,
    typename Policy = boost::math::policies::policy<> >
class bf_ConditionalDistrMixture : public virtual bf_ConditionalDistr<
    StateType, ObservationType, RngEngine, RealType, Policy> {

public:

    typedef bf_ConditionalDistr<
        StateType, ObservationType, RngEngine, RealType, Policy>
        distribution_type;
    typedef StateType state_type;
    typedef ObservationType observation_type;
    typedef RngEngine rng_engine_type;
    typedef RealType value_type;

    typedef std::pair<distribution_type*, value_type> element_type;

    // LIFECYCLE

    /// Constructor, initializes the mixture distribution with a set of
    /// weighted conditional distributions.
    /// @param weighted_distributions Weighted conditional distributions
    bf_ConditionalDistrMixture(
            const std::vector<element_type>& weighted_distributions) :
        mWeightedDistributions(weighted_distributions) {
    }

    virtual ~bf_ConditionalDistrMixture() {
        mWeightedDistributions.clear();
    }

    // OPERATIONS

    /// Returns conditional distributions and the associated weights
    /// as mixture elements in the domain of the discrete distribution for
    /// read-only access.
    /// @return Weighted conditional distributions
    const std::vector<element_type>& elements() const {
        return mWeightedDistributions;
    }

    /// Returns conditional distributions and the associated weights
    /// as mixture elements in the domain of the discrete distribution for
    /// read/write access.
    /// @return Weighted conditional distributions
    std::vector<element_type>& elements() {
        return mWeightedDistributions;
    }

    virtual ObservationType sample(RngEngine& eng,
        const StateType& state) const {

        // sample a random value in [0; 1]
        RealType val = RealType(eng() - (eng.min)()) /
                RealType((eng.max)() - (eng.min)());

        // normalize distribution weights
        typename std::vector<element_type>::const_iterator first_ii =
                mWeightedDistributions.begin();
        const typename std::vector<element_type>::const_iterator last_ii =
                mWeightedDistributions.end();

        std::vector<RealType> norm_weights(mWeightedDistributions.size());
        typename std::vector<RealType>::iterator result_ii =
                norm_weights.begin();

        // TODO: what if first_ii == last_ii ???

        // partial_sum
        RealType cum_weight;
        *result_ii++ = cum_weight = first_ii->second;
        ++first_ii;

        for (; first_ii != last_ii; ++first_ii) {
            *result_ii++ = cum_weight = cum_weight + first_ii->second;
        }

        using namespace boost::lambda;

        transform(norm_weights.begin(), norm_weights.end(), norm_weights.begin(),
            boost::lambda::_1 / cum_weight);

        // sample a distribution from a mixture
        result_ii = lower_bound(norm_weights.begin(), norm_weights.end(), val);
        distribution_type* sample_distr = (mWeightedDistributions.begin() +
                int(result_ii - norm_weights.begin()))->first;

        return sample_distr->sample(eng, state);

    }

    virtual RealType evaluate(const StateType& state,
        const ObservationType& obs) const {

        std::vector<RealType> lhoods(mWeightedDistributions.size());

        typename std::vector<RealType>::iterator first_ii = lhoods.begin();

        BOOST_FOREACH(const element_type& element, mWeightedDistributions) {
            *first_ii++ = element.first->evaluate(state, obs) * element.second;
        }

        return accumulate(lhoods.begin(), lhoods.end(), 0);

    }

private:

    // a set of distributions that comprise the mixture
    std::vector<element_type> mWeightedDistributions;
};

} // namespace BayesFilter

#endif // __BF_CONDITIONALDISTRIBUTIONMIXTURE_H__
