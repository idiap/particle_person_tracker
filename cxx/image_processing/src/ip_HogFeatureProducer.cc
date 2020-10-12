// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_HogFeatureProducer - histogram feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <numeric>                                  // for sum of squares
#include <limits>                                   // for epsilon
#include <boost/lambda/lambda.hpp>                  // for elementwise division
#include <iostream>
#include <fstream>
#include <sstream>

// LOCAL INCLUDES
#include <image_processing/ip_HogFeatureProducer.h>  // declaration of this

using namespace std;
using namespace boost::lambda;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_HogFeatureProducer::ip_HogFeatureProducer(
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
ip_HogFeatureProducer::compute_feature(const ip_RoiWindow& roi) {

    m_HistogramTemplate.clear();

    // compute features at scale 1
    // offset equals to 0
    compute_features_internal(0, roi, mParamsScale1);

    ostringstream oss1;
    oss1 << "vk_hog_feature_"
            << mParamsScale1.mNumBlocksCol << mParamsScale1.mNumBlocksRow
            << mParamsScale1.mNumCellsInBlockCol << mParamsScale1.mNumCellsInBlockRow;
    ofstream ofs1((oss1.str() + "_unnormalized.txt").c_str());
    for (unsigned i = 0; i < mNumCellsScale1; ++i) {
        const ip_HistogramTemplate::Histogram& histogram =
                m_HistogramTemplate.get_histogram(i);
        for (unsigned k = 0; k < histogram.size(); ++k) {
            ofs1 << histogram[k] << " ";
        }
        ofs1 << endl;
    }

    // compute features at scale 2
    // offset equals to the number of features on the previous scale
    compute_features_internal(mNumCellsScale1, roi, mParamsScale2);

    ostringstream oss2;
    oss2 << "vk_hog_feature_"
            << mParamsScale2.mNumBlocksCol << mParamsScale2.mNumBlocksRow
            << mParamsScale2.mNumCellsInBlockCol << mParamsScale2.mNumCellsInBlockRow;
    ofstream ofs2((oss2.str() + "_unnormalized.txt").c_str());
    for (unsigned i = 0; i < mNumCellsScale2; ++i) {
        const ip_HistogramTemplate::Histogram& histogram =
                m_HistogramTemplate.get_histogram(i + mNumCellsScale1);
        for (unsigned k = 0; k < histogram.size(); ++k) {
            ofs2 << histogram[k] << " ";
        }
        ofs2 << endl;
    }

    // normalize results at scale 1
    normalize_features_internal(0, mParamsScale1);

    // normalize features at scale 2
    normalize_features_internal(mNumCellsScale1, mParamsScale2);

    return m_HistogramTemplate;

} // ip_GradientHistogramFeatureProducer

ip_HistogramTemplate
ip_HogFeatureProducer::feature_prototype(
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
ip_HogFeatureProducer::compute_features_internal(
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

    // compute template cell indices of the cells that intersect with the image:
    // they form a rectangle defined by first and last column and row
    // Derivation for column index: we look for all indices n such that
    // 0 <= roi.m_iFirstColumn + (n + 0.5) * step_row <= img_width
    // and 0 <= n <= max_n
    // Derivation for row index: the same

    int first_col_index = static_cast<int>(
        ceil(max(0.0, -static_cast<float>(roi.m_iFirstColumn) /
                step_col - 0.5)));
    int last_col_index = static_cast<int>(
        floor(min(static_cast<float>(mImageWidth - roi.m_iFirstColumn) /
                      step_col - 0.5f,
                static_cast<float>(num_cells_row_total - 1))));

    int first_row_index = static_cast<int>(
        ceil(max(0.0, -static_cast<float>(roi.m_iFirstRow) / step_row - 0.5)));
    int last_row_index = static_cast<int>(
        floor(min(static_cast<float>(mImageHeight - roi.m_iFirstRow) /
                      step_row - 0.5f,
                  static_cast<float>(num_cells_col_total - 1))));

    // check if the ROI intersects with the image
    if ((first_row_index <= last_row_index) &&
            (first_col_index <= last_col_index)) {

        // compute template cell values using the integral image for gradients
        // result is represented as an array of concatenated rows

        const vector<ip_ImageProvider*>& providers = mIghProcessor->providers();
        unsigned num_bins = providers.size();
        IplImage* image;
        ip_RoiWindow local_roi;       // ROI of the integral image
        int hist_idx;                 // index of a histogram feature to fill
        float row_cached_lower_limit;   // current first line of the ROI
        float col_cached_lower_limit;   // current first column of the ROI
        const int step_correction = num_cells_row_total - last_col_index - 1;
        const int base_offset = offset + first_row_index * num_cells_row_total;
        static const float epsilon = 1e-3;

        // compute image cell indices of the cells that intersect with the
        // template cell at (i, j). They form a rectangle defined by local_roi
        // Derivation for column index: we look for all columns c such that
        // c + 1  > roi.m_iFirstColumn + i * step_col;
        // c < roi.m_iFirstColumn + (i + 1) * step_col;
        // 0 <= c <= c_max
        // Derivation for row index: the same

        for (unsigned k = 0; k < num_bins; ++k) {

            image = providers[k]->image();
            hist_idx = base_offset;

            row_cached_lower_limit = roi.m_iFirstRow +
                    first_row_index * step_row;
            for (int i = first_row_index; i <= last_row_index; ++i) {
                hist_idx += first_col_index;

                local_roi.m_iFirstRow = static_cast<int>(
                    max(0.0f, floor(row_cached_lower_limit)));
                row_cached_lower_limit += step_row;
                local_roi.m_iHeight = static_cast<int>(
                    // min of the last row in the image and in the ROI
                    min(static_cast<float>(mImageHeight - 1),
                        floor(row_cached_lower_limit - epsilon)))
                    - local_roi.m_iFirstRow + 1;
                col_cached_lower_limit = roi.m_iFirstColumn +
                        first_col_index * step_col;
                for (int j = first_col_index; j <= last_col_index; ++j) {

                    local_roi.m_iFirstColumn = static_cast<int>(
                        max(0.0f, floor(col_cached_lower_limit)));
                    col_cached_lower_limit += step_col;
                    local_roi.m_iWidth = static_cast<int>(
                       // min of the last column in the image and in the ROI
                       min(static_cast<float>(mImageWidth - 1),
                           floor(col_cached_lower_limit - epsilon)))
                       - local_roi.m_iFirstColumn + 1;
                    m_HistogramTemplate.get_histogram(hist_idx)[k] =
                        measure_feature_value(image, local_roi);
                    ++hist_idx;
                }
                hist_idx += step_correction;
            }
        }

    }

} // compute_features_internal

inline void
ip_HogFeatureProducer::normalize_features_internal(
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
ip_HogFeatureProducer::normalize_block_l2(
    unsigned block_row_idx, unsigned block_col_idx,
    unsigned offset, const ip_HogFeatureParameters& params) {

    const unsigned num_cells_row = params.mNumBlocksRow *
            params.mNumCellsInBlockRow;
    const unsigned offset_pre_col = params.mNumCellsInBlockRow * block_col_idx;
    const unsigned offset_post_col = num_cells_row -
            params.mNumCellsInBlockRow * (block_col_idx + 1);
    unsigned cur_offset = offset + block_row_idx * params.mNumCellsInBlockCol *
            num_cells_row;

    // compute normalization value for the block as sum of squares
    float norm_value = 0.0f;
    for (unsigned i = 0; i < params.mNumCellsInBlockCol; ++i) {
        cur_offset += offset_pre_col;
        for (unsigned j = 0; j < params.mNumCellsInBlockRow; ++j) {
            // sum of squares for all histogram values
            const ip_HistogramTemplate::Histogram& histogram =
                    m_HistogramTemplate.get_histogram(cur_offset);
            norm_value = inner_product(histogram.begin(), histogram.end(),
                    histogram.begin(), norm_value);
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
                transform(histogram.begin(), histogram.end(),
                    histogram.begin(), boost::lambda::_1 / norm_value);
                ++cur_offset;
            }
            cur_offset += offset_post_col;
        }
    }

} // normalize_block_l2

inline void
ip_HogFeatureProducer::normalize_all_l2(unsigned offset,
        const ip_HogFeatureParameters& params) {

    const unsigned num_cells =
            params.mNumBlocksRow * params.mNumCellsInBlockRow *
            params.mNumBlocksCol * params.mNumCellsInBlockCol;
    unsigned cur_offset = offset;

    // compute the overall normalization value as sum of squares
    float norm_value = 0.0f;
    for (unsigned i = 0; i < num_cells; ++i) {
        // sum of squares for all histogram values
        const ip_HistogramTemplate::Histogram& histogram =
                m_HistogramTemplate.get_histogram(cur_offset);
        norm_value = inner_product(histogram.begin(), histogram.end(),
                histogram.begin(), norm_value);
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
            transform(histogram.begin(), histogram.end(), histogram.begin(),
                    boost::lambda::_1 / norm_value);
            ++cur_offset;
        }
    }
} // normalize_all_l2

inline OpenCvPlus::real
ip_HogFeatureProducer::measure_feature_value(IplImage* img,
        const ip_RoiWindow& roi) {

    int last_column = roi.m_iFirstColumn + roi.m_iWidth - 1;
    int last_row = roi.m_iFirstRow + roi.m_iHeight - 1;

    // get integration result for rectangle (0, 0, lastrow, lastcol)
    OpenCvPlus::real result = CV_IMAGE_ELEM(img, float, last_row, last_column);

    // substract upper rectangle (0, 0, firstrow - 1, lastcol)
    result -= ((roi.m_iFirstRow > 0) ?
        CV_IMAGE_ELEM(img, float, roi.m_iFirstRow - 1, last_column) : 0);

    // substract left rectangle (0, 0, lastrow, firstcol - 1)
    result -= ((roi.m_iFirstColumn > 0) ?
        CV_IMAGE_ELEM(img, float, last_row, roi.m_iFirstColumn - 1) : 0);

    // add upper-left rectangle (0, 0, firstrow - 1, firstcol - 1)
    result += ((roi.m_iFirstColumn > 0) && (roi.m_iFirstRow > 0) ?
        CV_IMAGE_ELEM(img, float, roi.m_iFirstRow - 1,
                roi.m_iFirstColumn - 1) : 0);
    return result;

} // measure_feature_value

} // namespace ImageProcessing
