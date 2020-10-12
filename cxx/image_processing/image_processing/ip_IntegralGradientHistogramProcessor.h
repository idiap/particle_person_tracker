// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_IntegralGradientHistogramProcessor - computes integral gradient histogram
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_INTEGRALGRADIENTHISTOGRAMPROCESSOR_H__
#define __IP_INTEGRALGRADIENTHISTOGRAMPROCESSOR_H__

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>      // for real typedef

// LOCAL INCLUDES
#include "ip_ImageProviderGroup.h"  // base class

namespace ImageProcessing {

/// @brief Class to compute integral gradient histogram
///
/// This class represents a discrete distribution over gradient angles.
/// A number of discrete angular values $\theta_i, i=1..N$
/// are chosen in the interval [0..314], that is approximately [0..100 * Pi].
/// For each integer value of an angle $\alpha\in\{0,\ldots,314\}$ a
/// discrete distirubution $P(\alpha(s) | z_s = i)$ is computed
/// to belong to a certain bin.
/// Observed images of gradient magnitudes $d$ and angles $\alpha$
/// are represented as pairs of functions $(d(s),alpha(s))$ of image
/// location $s$. The integral gradient histogram is a collection $N$ cumulative
/// representations of "votes". Each pixel $s$ contributes to the bin $i$
/// as $d(s) * P(\alpha(s) | z_s = i)$.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_IntegralGradientHistogramProcessor : public ip_ImageProviderGroup {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param gradient_angle_provider Provider of gradient angles
    /// @param gradient_magnitude_provider Provider of gradient magnitude
    /// @param nbins Number of histogram bins
    ip_IntegralGradientHistogramProcessor(
            ip_ImageProvider * gradient_angle_provider,
            ip_ImageProvider * gradient_magnitude_provider,
            int nbins);

    /// Destructor, deallocates partial image providers
    virtual ~ip_IntegralGradientHistogramProcessor();

    protected:

    // OPERATIONS

    /// Obtain the most recent image
    /// @param image Image to write results to
    virtual void recompute_image(IplImage* image, const ip_RoiWindow& roi,
            boost::posix_time::ptime& time);

    private:

    // initialize weights to be used to compute HOGs
    std::vector<OpenCvPlus::real> initialize_weight_table(int min_angle,
            int step_angle, int max_angle, OpenCvPlus::real mean_angle);

    // normalize weights in the tables
    void normalize_weight_tables();


    // provider of gradient angle images
    ip_ImageProvider * mGradientAngleProvider;
    // provider of gradient magnitude images
    ip_ImageProvider * mGradientMagnitudeProvider;
    // number of histogram bins
    int mNumBins;

};

} // namespace ImageProcessing

#endif // __IP_INTEGRALGRADIENTHISTOGRAMPROCESSOR_H__
