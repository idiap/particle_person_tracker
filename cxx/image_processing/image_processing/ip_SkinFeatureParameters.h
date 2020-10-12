/**
 * @file cxx/bayes_image/bayes_image/ip_SkinFeatureParameters.h
 * @date 01 March 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Stores skin colour models for various head orientations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_SKINFEATUREPARAMETERS_H__
#define __IP_SKINFEATUREPARAMETERS_H__

// SYSTEM INCLUDES
#include <boost/serialization/access.hpp>          // access for serialisation

namespace ImageProcessing {

/// @brief Structure to store and pass skin feature parameters
struct ip_SkinFeatureParameters {
    unsigned mNumBlocksRow;       // number of blocks in a row of the feature
    unsigned mNumBlocksCol;       // number of blocks in a column of the feature
    unsigned mNumCellsInBlockRow; // number of cells in a row of a block
    unsigned mNumCellsInBlockCol; // number of cells in a column of a block
    float    mSkinMaskValue;      // value indicating skin pixels in the mask

    ip_SkinFeatureParameters() : mNumBlocksRow(0), mNumBlocksCol(0),
            mNumCellsInBlockRow(0), mNumCellsInBlockCol(0),
            mSkinMaskValue(255.0f) {}
    ip_SkinFeatureParameters(unsigned nbr, unsigned nbc, unsigned ncbr,
            unsigned ncbc, float skin_mask_value) :
                mNumBlocksRow(nbr), mNumBlocksCol(nbc),
                mNumCellsInBlockRow(ncbr), mNumCellsInBlockCol(ncbc),
                mSkinMaskValue(skin_mask_value) {}

private:
    // give access to private fields to serialize properly
    friend class boost::serialization::access;

    // hide the serialization functionality
    template<class Archive>
    void serialize(Archive & ar, const unsigned int ver) {
        ar & boost::serialization::make_nvp("num_blocks_row", mNumBlocksRow)
           & boost::serialization::make_nvp("num_blocks_col", mNumBlocksCol)
           & boost::serialization::make_nvp("num_cells_block_row",
               mNumCellsInBlockRow)
           & boost::serialization::make_nvp("num_cells_block_col",
               mNumCellsInBlockCol)
           & boost::serialization::make_nvp("skin_mask_value", mSkinMaskValue);
    }
};

} // namespace ImageProcessing

#endif // __IP_SKINFEATUREPARAMETERS_H__
