// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ColourSkinFeatureProducer - colour skin feature feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_COLOURSKINFEATUREPRODUCER_H__
#define __IP_COLOURSKINFEATUREPRODUCER_H__

// PROJECT INCLUDES
#include <opencvplus/cvplus.h>                     // for real typedef
#include <valarray>                                // for skin template

// LOCAL INCLUDES
#include "ip_ImageProvider.h"                      // for data provider
#include "ip_RoiWindow.h"                          // ROI declaration

namespace ImageProcessing {

/// @brief Struct to store and pass skin feature parameters
struct ip_ColourSkinFeatureParameters {
    unsigned mNumBlocksRow;       // number of blocks in a row of the feature
    unsigned mNumBlocksCol;       // number of blocks in a column of the feature
    unsigned mNumCellsInBlockRow; // number of cells in a row of a block
    unsigned mNumCellsInBlockCol; // number of cells in a column of a block

    ip_ColourSkinFeatureParameters() : mNumBlocksRow(0), mNumBlocksCol(0),
            mNumCellsInBlockRow(0), mNumCellsInBlockCol(0) {}
    ip_ColourSkinFeatureParameters(unsigned nbr, unsigned nbc, unsigned ncbr,
            unsigned ncbc) :
                mNumBlocksRow(nbr), mNumBlocksCol(nbc),
                mNumCellsInBlockRow(ncbr), mNumCellsInBlockCol(ncbc) {}
};

struct ip_ColourSkinTemplate {
    std::valarray<float> mRedColourTemplate;
    std::valarray<float> mBlueColourTemplate;
    std::valarray<float> mGreenColourTemplate;
};

/// @brief Class that computes colour skin-based features for rectangular image
/// regions
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    01.03.2011

class ip_ColourSkinFeatureProducer {

    public:

    /// Constructor
    /// @param ism_processor Integral skin mask provider
    /// @param params skin feature parameters
    ip_ColourSkinFeatureProducer(ip_ImageProvider* ism_processor,
        const ip_ColourSkinFeatureParameters& params);

    /// Computes the skin feature by the provided region of integest (ROI)
    /// of the integral image
    /// @param roi Region of interest for the feature producer
    /// @return Feature as a collection of values
    const ip_ColourSkinTemplate& compute_feature(const ip_RoiWindow& roi);

    /// Returns the last computed result
    /// @return Feature as a collection of values
    const ip_ColourSkinTemplate& get_cached_feature() {
        return m_ColourSkinTemplate;
    }

    /// Returns the prototype feature for given parameters
    /// @param params Skin feature parameters
    /// @return Prototype feature for given parameters
    static ip_ColourSkinTemplate feature_prototype(
            const ip_ColourSkinFeatureParameters& params);

    private:

    inline OpenCvPlus::real measure_red_feature_value(IplImage* image,
            const ip_RoiWindow& roi);
    inline OpenCvPlus::real measure_blue_feature_value(IplImage* image,
            const ip_RoiWindow& roi);
    inline OpenCvPlus::real measure_green_feature_value(IplImage* image,
            const ip_RoiWindow& roi);

    const ip_ColourSkinFeatureParameters mParams; // skin feature parameters
    const unsigned mImageWidth;         // image width
    const unsigned mImageHeight;        // image height

    // integral skin mask provider
    ip_ImageProvider* mImageProvider;

    // cached ip_HistogramTemplate
    ip_ColourSkinTemplate m_ColourSkinTemplate;

};

} // namespace ImageProcessing

#endif // __IP_COLOURSKINFEATUREPRODUCER_H__
