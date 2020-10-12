/**
 * @file cxx/bayes_image/bayes_image/bicv_StateImageLikelihood.h
 * @date 01 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Likelihood of image observation given tracker state
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __BICV_STATEIMAGELIKELIHOOD_H__
#define __BICV_STATEIMAGELIKELIHOOD_H__

// SYSTEM INCLUDES
#include <cv.h>                                     // for IplImage from OpenCV
#include <boost/random/mersenne_twister.hpp>        // RNG

// PROJECT INCLUDES
#include <bayes_filter/bf_ConditionalDistr.h>       // condition distribution

// LOCAL INCLUDES
#include <bayes_image/bicv_GeneralTrackerState.h>   // state

namespace BICV {

typedef BayesFilter::bf_ConditionalDistr<
    bicv_TrackerState, IplImage*, boost::mt19937> bicv_StateImageLikelihood;

} // namespace BICV

#endif // __BICV_STATEIMAGELIKELIHOOD_H__
