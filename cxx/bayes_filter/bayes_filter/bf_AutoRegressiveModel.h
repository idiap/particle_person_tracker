/**
 * @file cxx/bayes_filter/bayes_filter/bf_AutoRegressiveModel.h
 * @date 14 February 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Template of an autoregressive model of order p
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BF_AUTOREGRESSIVEMODEL_H__
#define __BF_AUTOREGRESSIVEMODEL_H__

// LOCAL INCLUDES
#include "bf_ConditionalDistr.h"                 // conditional distribution

namespace BayesFilter {

/// @brief Template of an autoregressive model of order p
///
/// This class defines a common template for an autoregressive model
/// of order p: AR(p). This model is a discrete random process given by
/// X_t = C + \sum\limits_{i=1}^{p} A_i X_{t-i} + W_t,
/// where A_1, ..., A_p are the parameters of the model, C is a constant and
/// W_t is white noise.
/// This model can be sampled and likelihood of the next state given
/// the history of states can be evaluated.
/// @param StateType State type, additive and multiplicative operators should
///        be defined for this type, it should be assignable and
///        copy-constructible
/// @param StateSupportPredicate Predicate that defines state space support,
///        should define method bool validate(const StateType& state) const;
/// @param StateSupportProjector Projects elements into state space support,
///        should define method StateType project(const StateType& state) const;
/// @param StateHistory Gives access to previous states X_{t-1}, ..., X_{t-p}
/// @param ParametersType Type for AR model parameters A_1, ..., A_p
/// @param Order Order p of the autoregressive model
/// @param NoiseGenerator Generator of white noise W_t on state space,
///        should define operator()
/// @param RngEngine Generator of uniform random numbers,
///        should define operator()
/// @param RealType Abstract real type, defaults to float
/// @param Policy Various policy specifications, not used currently
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 3.0
/// @date    14.02.2013

template<typename StateType, typename StateSupportPredicate,
    typename StateSupportProjector, typename StateHistory,
    typename ParametersType, unsigned Order,
    typename NoiseGenerator, typename RngEngine,
    typename RealType = float,
    typename Policy = boost::math::policies::policy<> >
class bf_AutoRegressiveModel : public bf_ConditionalDistr<
    StateHistory, StateType, RngEngine, RealType, Policy> {

public:

    typedef StateSupportPredicate predicate_type;
    typedef StateSupportProjector projector_type;

    /// Constructs autoregressive model AR(p) using state support predicate and
    /// projector.
    /// @param predicate State support predicate
    /// @param projector State support projector
    /// @param parameters Parameters A_1, ..., A_p of the AR(p) model
    /// @param state_const Constant C of the AR(p) model
    bf_AutoRegressiveModel(const StateSupportPredicate& predicate,
        const StateSupportProjector& projector,
        const ParametersType& parameters,
        const StateType& state_const,
        const NoiseGenerator& noise_generator) : m_Predicate(predicate),
            m_Projector(projector), m_Parameters(parameters),
            m_StateConst(state_const),
            m_NoiseGenerator(noise_generator) {
    } // bf_AutoRegressiveModel

    /// Destroys autoregressive model
    virtual ~bf_AutoRegressiveModel() {
    } // ~bf_AutoRegressiveModel

    /// Samples autoregressive model using state history X_{t-1}, ... X_{t-p}
    /// and white noise generator W_t
    /// @param rng White noise generator W_t
    /// @param state_history State history to be used for sampling
    ///        X_{t-1}, ... X_{t-p}
    virtual StateType sample(RngEngine& rng,
            const StateHistory& state_history) const {

        StateType next_state = m_NoiseGenerator.sample(rng);

        next_state += m_StateConst;
        for (unsigned i = 0; i < Order; ++i) {
            next_state += m_Parameters[i] * state_history[i];
        }

        if (!m_Predicate(next_state)) {
            next_state = m_Projector.project(next_state);
        }
        return next_state;

    } // sample

    /// Evaluates autoregressive model probability on state X_t
    /// using state history X_{t-1}, ... X_{t-p} and white noise generator W_t
    /// @param rng White noise generator W_t
    /// @param state_history State history to be used for sampling
    ///        X_{t-1}, ... X_{t-p}
    virtual RealType evaluate(const StateHistory& state_history,
            const StateType& state) const {
        StateType temp_state(state);
        temp_state -= m_StateConst;
        for (unsigned i = 0; i < Order; ++i) {
            temp_state -= m_Parameters[i] * state_history[i];
        }
        return m_NoiseGenerator.evaluate(temp_state);
    } // evaluate

private:

    // state space predicate that defines AR model support
    StateSupportPredicate m_Predicate;
    // state space projector that projects states to AR model support
    StateSupportProjector m_Projector;
    // AR model parameters
    ParametersType m_Parameters;
    // AR model constant
    StateType m_StateConst;
    // AR model white noise generator
    NoiseGenerator m_NoiseGenerator;

};

} // namespace BayesFilter

#endif // __BF_AUTOREGRESSIVEMODEL_H__
