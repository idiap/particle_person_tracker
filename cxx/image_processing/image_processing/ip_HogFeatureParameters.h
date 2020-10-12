/**
 * @file cxx/image_processing/image_processing/ip_HogFeatureParameters.h
 * @date 23 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief HOG feature parameters
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __IP_HOGFEATUREPARAMETERS_H__
#define __IP_HOGFEATUREPARAMETERS_H__

// SYSTEM INCLUDES
#include <boost/serialization/nvp.hpp>             // for make_nvp
#include <boost/serialization/access.hpp>          // access for serialisation

namespace ImageProcessing {

/// @brief Struct to store HOG feature parameters
///
/// This class stores various HOG feature parameters, such as information
/// on the number of blocks, number of cells per block in which to compute
/// HOGs.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    23.03.2011

struct ip_HogFeatureParameters {

    // FIELDS

    /// Number of blocks in a feature row
    unsigned mNumBlocksRow;
    /// Number of blocks in a feature column
    unsigned mNumBlocksCol;
    /// Number of cells in a block row
    unsigned mNumCellsInBlockRow;
    /// Number of cells in a block column
    unsigned mNumCellsInBlockCol;
    /// Number of histogram bins in a feature
    unsigned mNumHistBins;

    // LIFECYCLE

    /// Default constructor
    /// @param head_pose_domain Head pose discrete domain
    /// @param params HOG feature parameters

    ip_HogFeatureParameters() : mNumBlocksRow(0), mNumBlocksCol(0),
            mNumCellsInBlockRow(0), mNumCellsInBlockCol(0),
            mNumHistBins(0) {}

    /// Constructor
    /// @param nbr Number of blocks in a feature row
    /// @param nbc Number of blocks in a feature column
    /// @param ncbr Number of cells in a block row
    /// @param ncbc Number of cells in a block column
    /// @param nhb Number of histogram bins in a feature

    ip_HogFeatureParameters(unsigned nbr, unsigned nbc, unsigned ncbr,
            unsigned ncbc, unsigned nhb) :
                mNumBlocksRow(nbr), mNumBlocksCol(nbc),
                mNumCellsInBlockRow(ncbr), mNumCellsInBlockCol(ncbc),
                mNumHistBins(nhb) {}

    private:

    // OPERATIONS

    // give access to private fields for BOOST serialization
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
           & boost::serialization::make_nvp("num_hist_bins",
                   mNumHistBins);
    }

};

} // namespace ImageProcessing

#endif // __IP_HOGFEATUREPARAMETERS_H__
