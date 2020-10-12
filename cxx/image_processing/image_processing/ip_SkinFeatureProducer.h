// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SkinFeatureProducer - skin feature feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SKINFEATUREPRODUCER_H__
#define __IP_SKINFEATUREPRODUCER_H__

// SYSTEM INCLUDES
#include <valarray>                                // for skin template

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>                         // for real typedef
#include <image_processing/ip_SkinFeatureParameters.h> // parameters

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                      // for data provider
#include "ip_RoiWindow.h"                          // ROI declaration

namespace ImageProcessing {

typedef std::valarray<OpenCvPlus::real> ip_SkinTemplate;

/// @brief Class that computes skin-based features for rectangular image regions
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    01.03.2011

class ip_SkinFeatureProducer {

    public:

    /// Constructor
    /// @param ism_processor Integral skin mask provider
    /// @param params skin feature parameters
    ip_SkinFeatureProducer(ip_ImageProvider* ism_processor,
        const ip_SkinFeatureParameters& params);

    /// Computes the skin feature by the provided region of integest (ROI)
    /// of the integral image
    /// @param roi Region of interest for the feature producer
    /// @return Feature as a collection of values
    const ip_SkinTemplate& compute_feature(const ip_RoiWindow& roi);

    /// Returns the last computed result
    /// @return Feature as a collection of values
    const ip_SkinTemplate& get_cached_feature() {
        return m_SkinTemplate;
    }

    /// Returns the prototype feature for given parameters
    /// @param params Skin feature parameters
    /// @return Prototype feature for given parameters
    static ip_SkinTemplate feature_prototype(
            const ip_SkinFeatureParameters& params);

    private:

    /// Measures a feature cell using the skin mask integral image
    /// @param image Skin mask integral image
    /// @param roi Region of interest to measure in
    /// @return measured feature value
    inline OpenCvPlus::real measure_feature_value(IplImage* image,
            const ip_RoiWindow& roi);

    const ip_SkinFeatureParameters mParams; // skin feature parameters
    const float    mCellMaskThreshold;  // threshold for skin/non-skin cells
    const unsigned mImageWidth;         // image width
    const unsigned mImageHeight;        // image height

    // integral skin mask provider
    ip_ImageProvider* mIsmProcessor;

    // cached ip_HistogramTemplate
    ip_SkinTemplate m_SkinTemplate;

};

/// Empirical distance between two skin features.
/// Equals to 1 if L2 distance between the two features is less than N/2,
/// where N is a number of elements in a feature.
/// Otherwise equals to 1e-4
/// @param st1 First skin feature
/// @param st1 Second skin feature
/// @return Measured empirical distance between the two features
OpenCvPlus::real distance_emp(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2);

/// L1 distance between two skin features.
/// Equals to sum of absolute values of elementwise differences of the two
/// features.
/// @param st1 First skin feature
/// @param st1 Second skin feature
/// @return Measured L1 distance between the two features
OpenCvPlus::real distance_L1(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2);

/// Squared L2 distance between two skin features.
/// Equals to sum of squared elementwise differences of the two features.
/// @param st1 First skin feature
/// @param st1 Second skin feature
/// @return Measured squared L2 distance between the two features
OpenCvPlus::real distance_L2_squared(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2);

/// Squared Mahalanobis distance between two skin features given the
/// corresponding standard deviations.
/// Equals to sum of squared elementwise differences of the two features,
/// divided by the corresponding variance (square of standard deviation) values.
/// @param st1 First skin feature
/// @param st1 Second skin feature
/// @param stddevs Standard deviation values
/// @return Measured squared Mahalanobis distance between the two features
OpenCvPlus::real distance_mahalanobis_squared(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2, const ip_SkinTemplate& stddevs);

} // namespace ImageProcessing

#endif // __IP_SKINFEATUREPRODUCER_H__
