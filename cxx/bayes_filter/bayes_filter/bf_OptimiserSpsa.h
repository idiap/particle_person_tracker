/**
 * @file cxx/bayes_filter/bayes_filter/bf_OptimiserSpsa.h
 * @date 28 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief General SPSA optimisation algorithm
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BF_OPTIMISERSPSA_H__
#define __BF_OPTIMISERSPSA_H__

// SYSTEM INCLUDES
#include <boost/math/policies/policy.hpp>        // boost policies
#include <list>                                  // STL list

namespace BayesFilter {

/// General SPSA optimisation algorithm

template<typename StateType, typename FunctionType, typename PredicateType,
    typename ValidatorType, typename RandomStateGeneratorType,
    typename RealType = float,
    typename Policy = boost::math::policies::policy<> >
class bf_OptimiserSpsa {

public:

    typedef StateType state_type;
    typedef FunctionType function_type;
    typedef PredicateType predicate_type;
    typedef ValidatorType validator_type;
    typedef RandomStateGeneratorType random_state_generator_type;
    typedef RealType value_type;
    typedef Policy policy_type;

    typedef std::pair<StateType, RealType> optpoint_type;

    bf_OptimiserSpsa(FunctionType fun, PredicateType predicate) :
        m_Function(fun), m_Predicate(predicate) {

    } // bf_OptimiserSpsa

    ~bf_OptimiserSpsa() {

    } // ~bf_OptimiserSpsa

    optpoint_type optimise(const StateType& init_val,
        int num_iter, const RealType& ca, const RealType& cb,
        RandomStateGeneratorType& rng, ValidatorType validator) {

        m_OptimisationHistory.clear();
        optpoint_type data;
        data.first = init_val;
        data.second = m_Function(init_val);
        m_OptimisationHistory.push_back(data);

        for (int i = 0; i < num_iter; ++i) {
            state_type proposal = step(i, ca, cb, data.first, rng);
            value_type val = m_Function(proposal);
            if (validator(proposal) &&
                m_Predicate(data.second, val)) {
                data.first = proposal;
                data.second = val;
                m_OptimisationHistory.push_back(data);
            }
        }
        return data;

    } // optimise

    const std::list<optpoint_type>& history() {
        return m_OptimisationHistory;
    } // history

private:

    state_type step(int n, const RealType& ca, const RealType& cb,
        const StateType& val, RandomStateGeneratorType& rng) {
        RealType an = ca / n;
        RealType bn = cb / pow(n, 1.0 / 3);
        StateType dv = rng();
        StateType bv = bn * dv;
        StateType val_p = val + bv;
        StateType val_m = val - bv;
        StateType val_t = val -
                (m_Function(val_p) - m_Function(val_m)) * an / 2 / bn * dv;
        return val_t;
    } // step

    function_type            m_Function;
    predicate_type           m_Predicate;
    std::list<optpoint_type> m_OptimisationHistory;

};

} // namespace BayesFilter

#endif // __BF_OPTIMISERSPSA_H__
