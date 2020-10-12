// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_HogFeatureProducer - HoG feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_HOGFEATUREPRODUCER_H__
#define __IP_HOGFEATUREPRODUCER_H__

// SYSTEM INCLUDES
#include <vector>                                  // STL vector
#include <boost/serialization/access.hpp>          // access for serialisation

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>                     // for real typedef

// LOCAL INCLUDES
#include "ip_HogFeatureParameters.h"               // HOG parameters
#include "ip_HistogramTemplate.h"                  // histogram template
#include "ip_IntegralGradientHistogramProcessor.h" // provides data
#include "ip_RoiWindow.h"                          // ROI declaration

namespace ImageProcessing {

/// @brief Class that computes histogram features for rectangular image regions
///
/// The class is designed to compute histogram template features for
/// rectangular image regions based on gradient histogram integral images.
/// The class returns a reference to the resulting feature, which means that
/// in order to save the result, one must make a copy of the feature.
/// However, the copy operation can occur to be rather consuming.
///
/// The histogram template features are computed as follows.
/// The rectangular image region is divided into NxM blocks.
/// Each block in its turn is divided into KxL cells.
/// Sum of HOGs is computed for each cell,
/// cell values are normalized within a block.
/// Block values are then normalized.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    23.02.2011

class ip_HogFeatureProducer {

    public:

    /// Constructor
    /// @param igh_processor Integral gradient histogram provider
    /// @param params HoG feature parameters
    ip_HogFeatureProducer(
            ip_ImageProviderGroup* igh_processor,
            const ip_HogFeatureParameters& params);

    /// Computes the gradient histogram feature from the provided
    /// region of integest (ROI) of the integral image
    /// @param roi Region of interest for the feature producer
    /// @return Feature as a collection of histograms
    const ip_HistogramTemplate& compute_feature(const ip_RoiWindow& roi);

    /// Returns the last computed result
    /// @return Feature as a collection of histograms
    const ip_HistogramTemplate& get_cached_feature() const {
        return m_HistogramTemplate;
    }

    /// Returns the prototype feature for given parameters
    /// @param params HoG feature parameters
    /// @return Prototype feature for given parameters
    static ip_HistogramTemplate feature_prototype(
            const ip_HogFeatureParameters& params);

    private:

    /// Computes a subset of features, which is defined by the number of cells
    /// @param offset Offset at which to write to the resulting template
    /// @param roi Region of interest for the feature producer
    /// @param params HoG feature parameters
    /// @return Feature as a collection of histograms
    inline void compute_features_internal(unsigned offset,
        const ip_RoiWindow& roi, const ip_HogFeatureParameters& params);

    /// Normalizes a subset of features by blocks and in total
    /// @param offset Offset at which to write to the resulting template
    /// @param params HoG feature parameters
    inline void normalize_features_internal(unsigned offset,
        const ip_HogFeatureParameters& params);

    /// Normalizes a blocks
    /// @param block_row_idx block index in a row
    /// @param block_col_idx block index in a column
    /// @param offset Offset at which to write to the resulting template
    /// @param params HoG feature parameters
    inline void normalize_block_l2(
        unsigned block_row_idx, unsigned block_col_idx, unsigned offset,
        const ip_HogFeatureParameters& params);

    /// Normalizes all cells in total
    /// @param offset Offset at which to write to the resulting template
    /// @param params HoG feature parameters
    inline void normalize_all_l2(unsigned offset,
        const ip_HogFeatureParameters& params);

    /// Measures a feature cell using the gradient integral image
    /// @param image Gradient integral image
    /// @param roi Region of interest to measure in
    /// @return measured feature value
    inline OpenCvPlus::real measure_feature_value(IplImage* image,
            const ip_RoiWindow& roi);

    const ip_HogFeatureParameters mParamsScale1; // feature parameters, scale 1
    const ip_HogFeatureParameters mParamsScale2; // feature parameters, scale 2
    const unsigned mNumCellsScale1;              // number of cells at scale 1
    const unsigned mNumCellsScale2;              // number of cells at scale 2
    const unsigned mImageWidth;            // image width
    const unsigned mImageHeight;           // image height

    // integral gradient histogram provider
    const ip_ImageProviderGroup* mIghProcessor;

    // cached ip_HistogramTemplate
    ip_HistogramTemplate m_HistogramTemplate;

};

} // namespace ImageProcessing

#endif // __IP_HOGFEATUREPRODUCER_H__
