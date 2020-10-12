// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_HogFeatureProducerRounded - HoG feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <numeric>                                  // for sum of squares
#include <limits>                                   // for epsilon
#include <boost/lambda/lambda.hpp>                  // for elementwise division
#include <boost/foreach.hpp>                        // boost foreach loop

// LOCAL INCLUDES
#include <image_processing/ip_HogFeatureProducerRounded.h> // declaration of this

using namespace std;
using namespace boost::lambda;

#define ROUND_POSITIVE(X) int(X+0.5)

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_HogFeatureProducerRounded::ip_HogFeatureProducerRounded(
        ip_ImageProviderGroup* igh_processor,
        const ip_HogFeatureParameters& params) :
        mParamsScale1(params),
        mParamsScale2(
                params.mNumBlocksRow, params.mNumBlocksCol,
                params.mNumCellsInBlockRow - 1, params.mNumCellsInBlockCol - 1,
                params.mNumHistBins),
        mNumCellsScale1(
                params.mNumBlocksRow * params.mNumCellsInBlockRow *
                params.mNumBlocksCol * params.mNumCellsInBlockCol),
        mNumCellsScale2(
                params.mNumBlocksRow * (params.mNumCellsInBlockRow - 1) *
                params.mNumBlocksCol * (params.mNumCellsInBlockCol - 1)),
        // assumes that the provider list in this group is not empty!
        mImageWidth(igh_processor->providers()[0]->image_buffer()->width),
        mImageHeight(igh_processor->providers()[0]->image_buffer()->height),
        mIghProcessor(igh_processor),
        m_HistogramTemplate(mNumCellsScale1 + mNumCellsScale2,
                params.mNumHistBins) {

    assert(params.mNumHistBins == igh_processor->providers().size());
} // ip_GradientHistogramFeatureProducer

const ip_HistogramTemplate&
ip_HogFeatureProducerRounded::compute_feature(const ip_RoiWindow& roi) {

    m_HistogramTemplate.clear();

    // compute features at scale 1
    // offset equals to 0
    compute_features_internal(0, roi, mParamsScale1);

    // compute features at scale 2
    // offset equals to the number of features on the previous scale
    compute_features_internal(mNumCellsScale1, roi, mParamsScale2);

    // normalize results at scale 1
    normalize_features_internal(0, mParamsScale1);

    // normalize features at scale 2
    normalize_features_internal(mNumCellsScale1, mParamsScale2);

    return m_HistogramTemplate;

} // ip_GradientHistogramFeatureProducer

ip_HistogramTemplate
ip_HogFeatureProducerRounded::feature_prototype(
        const ip_HogFeatureParameters& params) {
    return ip_HistogramTemplate(
        params.mNumBlocksRow * params.mNumCellsInBlockRow *
        params.mNumBlocksCol * params.mNumCellsInBlockCol +
        params.mNumBlocksRow * (params.mNumCellsInBlockRow - 1) *
        params.mNumBlocksCol * (params.mNumCellsInBlockCol - 1),
        params.mNumHistBins);
} // feature_prototype

/////////////////////////////// PRIVATE //////////////////////////////////////

inline void
ip_HogFeatureProducerRounded::compute_features_internal(
        unsigned offset, const ip_RoiWindow& roi,
        const ip_HogFeatureParameters& params) {

    // total number of cells in a row
    const unsigned num_cells_row_total = params.mNumBlocksRow *
            params.mNumCellsInBlockRow;
    // total number of cells in a column
    const unsigned num_cells_col_total = params.mNumBlocksCol *
            params.mNumCellsInBlockCol;

    // step size for columns in a row
    float step_col = static_cast<float>(roi.m_iWidth) / num_cells_row_total;
    // step size for rows in a column
    float step_row = static_cast<float>(roi.m_iHeight) / num_cells_col_total;

    const vector<ip_ImageProvider*>& providers = mIghProcessor->providers();
    const unsigned num_bins = providers.size();
    unsigned hist_idx;                 // index of a histogram feature to fill

    IplImage* image;
    ip_RoiWindow local_roi;       // ROI of the integral image

    for (unsigned k = 0; k < num_bins; ++k) {

        image = providers[k]->image();
        hist_idx = offset;

//        for (unsigned block_row = 0; block_row < params.mNumBlocksCol; block_row+=step_row) {
//            static float rc = block_row * params.mNumCellsInBlockCol;
//            for (unsigned block_col = 0; block_col < params.mNumBlocksRow; block_col+=step_col) {
//                static float  cr = block_col * params.mNumCellsInBlockRow;
//                for (unsigned row = 0; row < params.mNumCellsInBlockCol; row+=step_row) {
//                    for (unsigned col = 0; col < params.mNumCellsInBlockRow; col+=step_col) {
        for (unsigned block_row = 0; block_row < params.mNumBlocksCol; ++block_row) {
            for (unsigned block_col = 0; block_col < params.mNumBlocksRow; ++block_col) {
                for (unsigned row = 0; row < params.mNumCellsInBlockCol; ++row) {
                    for (unsigned col = 0; col < params.mNumCellsInBlockRow; ++col) {
                        local_roi.m_iFirstColumn =
                                roi.m_iFirstColumn +
                                ROUND_POSITIVE(step_col * (block_col * params.mNumCellsInBlockRow + col));
//                                ROUND_POSITIVE(cr + col);
                        local_roi.m_iFirstRow =
                                roi.m_iFirstRow +
                                ROUND_POSITIVE(step_row * (block_row * params.mNumCellsInBlockCol + row));
//                                ROUND_POSITIVE(rc + row);
                        local_roi.m_iWidth = roi.m_iFirstColumn +
                                ROUND_POSITIVE(step_col * (block_col * params.mNumCellsInBlockRow + col + 1)) - local_roi.m_iFirstColumn + 1;
//                                ROUND_POSITIVE(cr + col + step_col) - local_roi.m_iFirstColumn + 1;
                        local_roi.m_iHeight = roi.m_iFirstRow +
                                ROUND_POSITIVE(step_row * (block_row * params.mNumCellsInBlockRow + row + 1)) - local_roi.m_iFirstRow + 1;
//                                ROUND_POSITIVE(rc + row + step_row) - local_roi.m_iFirstRow + 1;
                        m_HistogramTemplate.get_histogram(hist_idx)[k] =
                            measure_feature_value(image, local_roi);
                        ++hist_idx;
                    }
                }
            }
        }
    }

} // compute_features_internal

inline void
ip_HogFeatureProducerRounded::normalize_features_internal(
    unsigned offset, const ip_HogFeatureParameters& params) {

    // normalize all blocks
    for (unsigned i = 0; i < params.mNumBlocksCol; ++i) {
        for (unsigned j = 0; j < params.mNumBlocksRow; ++j) {
            normalize_block_l2(i, j, offset, params);
        }
    }

    // normalize all cells
    normalize_all_l2(offset, params);

} // normalize_features_internal

inline void
ip_HogFeatureProducerRounded::normalize_block_l2(
    unsigned block_row_idx, unsigned block_col_idx,
    unsigned offset, const ip_HogFeatureParameters& params) {

    typedef ip_HistogramTemplate::Histogram::value_type HistVal;

    const unsigned num_cells_row = params.mNumBlocksRow *
            params.mNumCellsInBlockRow;
    const unsigned offset_pre_col = params.mNumCellsInBlockRow * block_col_idx;
    const unsigned offset_post_col = num_cells_row -
            params.mNumCellsInBlockRow * (block_col_idx + 1);
    unsigned cur_offset = offset + block_row_idx * params.mNumCellsInBlockCol *
            num_cells_row;

    HistVal * pHist;
    HistVal * pHistEnd;

    // compute normalization value for the block as sum of squares
    float norm_value = 0.0f;
    for (unsigned i = 0; i < params.mNumCellsInBlockCol; ++i) {
        cur_offset += offset_pre_col;
        for (unsigned j = 0; j < params.mNumCellsInBlockRow; ++j) {
            // sum of squares for all histogram values
            ip_HistogramTemplate::Histogram& histogram =
                    m_HistogramTemplate.get_histogram(cur_offset);
            pHist = &histogram[0];
            pHistEnd = pHist + histogram.size();
            for (; pHist != pHistEnd; ++pHist) {
                norm_value += (*pHist) * (*pHist);
            }
            ++cur_offset;
        }
        cur_offset += offset_post_col;
    }

    if (norm_value > 0) {
        // take square root
        norm_value = sqrt(norm_value);
        // and normalize all the values in the block
        cur_offset = offset + block_row_idx * params.mNumCellsInBlockCol *
                    num_cells_row;
        for (unsigned i = 0; i < params.mNumCellsInBlockCol; ++i) {
            cur_offset += offset_pre_col;
            for (unsigned j = 0; j < params.mNumCellsInBlockRow; ++j) {

                // divide all values of the histogram by norm_value
                ip_HistogramTemplate::Histogram& histogram =
                    m_HistogramTemplate.get_histogram(cur_offset);
                pHist = &histogram[0];
                pHistEnd = pHist + histogram.size();
                for (; pHist != pHistEnd; ++pHist) {
                    *pHist /= norm_value;
                }
                ++cur_offset;
            }
            cur_offset += offset_post_col;
        }
    }

} // normalize_block_l2

inline void
ip_HogFeatureProducerRounded::normalize_all_l2(unsigned offset,
        const ip_HogFeatureParameters& params) {

    typedef ip_HistogramTemplate::Histogram::value_type HistVal;

    const unsigned num_cells =
            params.mNumBlocksRow * params.mNumCellsInBlockRow *
            params.mNumBlocksCol * params.mNumCellsInBlockCol;
    unsigned cur_offset = offset;
    HistVal * pHist;
    HistVal * pHistEnd;

    // compute the overall normalization value as sum of squares
    float norm_value = 0.0f;
    for (unsigned i = 0; i < num_cells; ++i) {
        // sum of squares for all histogram values
        ip_HistogramTemplate::Histogram& histogram =
                m_HistogramTemplate.get_histogram(cur_offset);
        pHist = &histogram[0];
        pHistEnd = pHist + histogram.size();
        for (; pHist != pHistEnd; ++pHist) {
            norm_value += (*pHist) * (*pHist);
        }
        ++cur_offset;
    }

    if (norm_value > 0) {
        // take square root
        norm_value = sqrt(norm_value);
        // and normalize all the values in the template
        cur_offset = offset;
        for (unsigned i = 0; i < num_cells; ++i) {
            // divide all values of the histogram by norm_value
            ip_HistogramTemplate::Histogram& histogram =
                m_HistogramTemplate.get_histogram(cur_offset);
            pHist = &histogram[0];
            pHistEnd = pHist + histogram.size();
            for (; pHist != pHistEnd; ++pHist) {
                *pHist /= norm_value;
            }
            ++cur_offset;
        }
    }
} // normalize_all_l2

inline OpenCvPlus::real
ip_HogFeatureProducerRounded::measure_feature_value(IplImage* img,
        const ip_RoiWindow& roi) {

    int first_column = (roi.m_iFirstColumn > 0) ? roi.m_iFirstColumn : 0;
    int first_row    = (roi.m_iFirstRow > 0) ? roi.m_iFirstRow : 0;
    int last_column  = roi.m_iFirstColumn + roi.m_iWidth;
    if (last_column > mImageWidth) {
        last_column = mImageWidth;
    }
    --last_column;
    int last_row  = roi.m_iFirstRow + roi.m_iHeight;
    if (last_row > mImageHeight) {
        last_row = mImageHeight;
    }
    --last_row;

    if ((first_column > last_column) || (first_row > last_row)) {
        return 0;
    }

    const char *const imgdata_ptr = img->imageData;
    const int width_step = img->widthStep;

    // get integration result for rectangle (0, 0, lastrow, lastcol)

    const float * bottom_row = (float*)(imgdata_ptr + width_step * last_row);
    const bool first_row_positive = (first_row > 0);
    --first_row;
    const bool first_col_positive = (first_column > 0);
    --first_column;

    OpenCvPlus::real result = bottom_row[last_column];

    if (first_row_positive) {
        const float * top_row = (float*)(imgdata_ptr + width_step * (first_row));
        // substract upper rectangle (0, 0, firstrow - 1, lastcol)
        result -= top_row[last_column];
        if (first_col_positive) {
            // add upper-left rectangle (0, 0, firstrow - 1, firstcol - 1)
            result += top_row[first_column];
        }
    }

    if (first_col_positive) {
        // substract left rectangle (0, 0, lastrow, firstcol - 1)
        result -= bottom_row[first_column];
    }
    return result;

} // measure_feature_value

} // namespace ImageProcessing
