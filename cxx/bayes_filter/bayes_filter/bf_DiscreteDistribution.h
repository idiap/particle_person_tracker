// Copyright (c) 2010-2020 Idiap Research Institute
//
// bf_DiscreteDistribution - class to represent a general discrete distribution
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)

#ifndef __BF_DISCRETEDISTRIBUTION_H__
#define __BF_DISCRETEDISTRIBUTION_H__

// SYSTEM INCLUDES
#include <utility>                               // for std::pair
#include <vector>                                // for std::vector
#include <boost/math/policies/policy.hpp>        // boost policies
#include <boost/lambda/lambda.hpp>               // boost lambda exprs
#include <boost/foreach.hpp>                     // boost foreach loop

namespace BayesFilter {

/// @brief Class to represent a general discrete distribution
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    04.03.2011

template<typename T, typename RealType = float,
        typename Policy = boost::math::policies::policy<> >
class bf_DiscreteDistribution {

public:

    typedef T value_type;
    typedef Policy policy_type;
    typedef std::pair<T, RealType> element_type;

    // LIFECYCLE

    /// Constructor, initializes the distribution with the initial element
    /// @param elements discrete domain elements
    bf_DiscreteDistribution(const value_type& value, unsigned num_elements) :
        mElements(num_elements, std::make_pair(value, 1)) {
    }

    /// Constructor, initializes the distribution with the probability space
    /// elements domain (values and the associated weights).
    /// Weights should be positive, but can be unnormalized
    /// @param elements discrete domain elements
    bf_DiscreteDistribution(const std::vector<element_type>& elements) :
        mElements(elements) {
    }

    /// Returns elements in the domain of the discrete distribution for
    /// read-only access
    /// @return Discrete domain elements
    const std::vector<element_type>& elements() const {
        return mElements;
    }

    /// Returns elements in the domain of the discrete distribution for
    /// read/write access. User is responsible to keep elements and weights
    /// @return Discrete domain elements
    std::vector<element_type>& elements() {
        return mElements;
    }

private:

    // probability space elements (values and associated weights)
    std::vector<element_type> mElements;
};

template<typename T, typename RealType, typename Policy>
std::ostream& operator<<( std::ostream& os,
        const bf_DiscreteDistribution<T, RealType, Policy>& dist) {

    typedef typename bf_DiscreteDistribution<T, RealType, Policy>::
        element_type ElementType;
    const std::vector<ElementType>& elements = dist.elements();
    unsigned count = 0;
    BOOST_FOREACH(const ElementType& element, elements) {
        ++count;
        os << count << ": ELT: ";
        os << element.first;
        os << ", W: " << element.second << std::endl;
    }
    return os;
}

template<typename T, typename RealType, typename Policy>
inline T mode(const bf_DiscreteDistribution<T, RealType, Policy>& dist) {

    const std::vector<typename bf_DiscreteDistribution
        <T, RealType, Policy>::element_type>& elements = dist.elements();

    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator first_ii = elements.begin();
    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator largest_ii = first_ii;
    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator last_ii = elements.end();

    // TODO: what if first_ii == last_ii ???

    while (++first_ii != last_ii) {
        if (largest_ii->second < first_ii->second) {
            largest_ii = first_ii;
        }
    }
    return largest_ii->first;

} // mode

template<typename T, typename RealType, typename Policy>
inline T mean(const bf_DiscreteDistribution<T, RealType, Policy>& dist) {

    const std::vector<typename bf_DiscreteDistribution
        <T, RealType, Policy>::element_type>& elements = dist.elements();

    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator first_ii = elements.begin();
    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator last_ii = elements.end();

    RealType accumulator_weight = first_ii->second;
    T accumulator_value = first_ii->first * first_ii->second;

    // TODO: what if first_ii == last_ii ???

    while (++first_ii != last_ii) {
        accumulator_weight += first_ii->second;
        accumulator_value += first_ii->first * first_ii->second;
    }

    return (accumulator_weight > 0) ? accumulator_value / accumulator_weight :
        accumulator_value;

} // mean

template<typename T, typename RealType, typename Policy>
inline T variance(const bf_DiscreteDistribution<T, RealType, Policy>& dist) {

    const T mean_value = mean(dist);

    const std::vector<typename bf_DiscreteDistribution
        <T, RealType, Policy>::element_type>& elements = dist.elements();

    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator first_ii = elements.begin();
    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator last_ii = elements.end();

    RealType accumulator_weight = first_ii->second;
    T accumulator_value = first_ii->first * first_ii->first * first_ii->second;

    // TODO: what if first_ii == last_ii ???

    while (++first_ii != last_ii) {
        accumulator_weight += first_ii->second;
        accumulator_value += first_ii->first * first_ii->first * first_ii->second;
    }

    return (accumulator_weight > 0) ?
           accumulator_value / accumulator_weight - mean_value * mean_value :
           accumulator_value * 0;

} // variance

template<typename Engine, typename T, typename RealType, typename Policy>
inline T sample(Engine& eng,
        const bf_DiscreteDistribution<T, RealType, Policy>& dist) {

    const std::vector<typename bf_DiscreteDistribution
        <T, RealType, Policy>::element_type>& elements = dist.elements();

    RealType val = RealType(eng() - (eng.min)()) /
            RealType((eng.max)() - (eng.min)());

    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator first_ii = elements.begin();
    typename std::vector<typename bf_DiscreteDistribution<T, RealType, Policy>
        ::element_type>::const_iterator last_ii = elements.end();

    std::vector<RealType> norm_weights(elements.size());
    typename std::vector<RealType>::iterator result_ii = norm_weights.begin();

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

    result_ii = lower_bound(norm_weights.begin(), norm_weights.end(), val);
    return (elements.begin() + int(result_ii - norm_weights.begin()))->first;
}

} // namespace BayesFilter

#endif // __BF_DISCRETEDISTRIBUTION_H__
