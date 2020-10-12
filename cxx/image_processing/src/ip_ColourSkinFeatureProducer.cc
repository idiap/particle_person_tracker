// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_ColourSkinFeatureProducer - colour skin feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/lambda/lambda.hpp>   // for transformation in operators
#include <numeric>                   // for inner product

// LOCAL INCLUDES
#include <image_processing/ip_ColourSkinFeatureProducer.h> // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////
ip_ColourSkinFeatureProducer::ip_ColourSkinFeatureProducer(
        ip_ImageProvider* image_provider,
        const ip_ColourSkinFeatureParameters& params) :
        mParams(params),
        mImageWidth(image_provider->image_buffer()->width),
        mImageHeight(image_provider->image_buffer()->height),
        mImageProvider(image_provider){

    unsigned feature_size = params.mNumBlocksRow * params.mNumCellsInBlockRow *
            params.mNumBlocksCol * params.mNumCellsInBlockCol;

    m_ColourSkinTemplate.mBlueColourTemplate.resize(feature_size);
    m_ColourSkinTemplate.mGreenColourTemplate.resize(feature_size);
    m_ColourSkinTemplate.mRedColourTemplate.resize(feature_size);

} // ip_SkinFeatureProducer

const ip_ColourSkinTemplate&
ip_ColourSkinFeatureProducer::compute_feature(const ip_RoiWindow& roi) {

    m_ColourSkinTemplate.mBlueColourTemplate = 0.0f;
    m_ColourSkinTemplate.mRedColourTemplate = 0.0f;
    m_ColourSkinTemplate.mGreenColourTemplate = 0.0f;

    // total number of cells in a row
    const unsigned num_cells_row_total = mParams.mNumBlocksRow *
            mParams.mNumCellsInBlockRow;
    // total number of cells in a column
    const unsigned num_cells_col_total = mParams.mNumBlocksCol *
            mParams.mNumCellsInBlockCol;

    // step size for columns in a row
    float step_col = static_cast<float>(roi.m_iWidth) / num_cells_row_total;
    // step size for rows in a column
    float step_row = static_cast<float>(roi.m_iHeight) / num_cells_col_total;

    unsigned hist_idx = 0;                 // index of a histogram feature to fill

    IplImage* image = mImageProvider->image();
    ip_RoiWindow local_roi;       // ROI of the integral image

    for (unsigned block_row = 0; block_row < mParams.mNumBlocksCol; ++block_row) {
        for (unsigned block_col = 0; block_col < mParams.mNumBlocksRow; ++block_col) {
            for (unsigned row = 0; row < mParams.mNumCellsInBlockCol; ++row) {
                for (unsigned col = 0; col < mParams.mNumCellsInBlockRow; ++col) {
                    local_roi.m_iFirstColumn = roi.m_iFirstColumn +
                            round(step_col * (block_col * mParams.mNumCellsInBlockRow + col));
                    local_roi.m_iFirstRow = roi.m_iFirstRow +
                            round(step_row * (block_row * mParams.mNumCellsInBlockCol + row));
                    local_roi.m_iWidth = roi.m_iFirstColumn +
                            round(step_col * (block_col * mParams.mNumCellsInBlockRow + col + 1)) - local_roi.m_iFirstColumn + 1;
                    local_roi.m_iHeight = roi.m_iFirstRow +
                            round(step_row * (block_row * mParams.mNumCellsInBlockRow + row + 1)) - local_roi.m_iFirstRow + 1;
                    m_ColourSkinTemplate.mBlueColourTemplate[hist_idx] =
                            measure_blue_feature_value(image, local_roi);
                    m_ColourSkinTemplate.mRedColourTemplate[hist_idx] =
                            measure_red_feature_value(image, local_roi);
                    m_ColourSkinTemplate.mGreenColourTemplate[hist_idx] =
                            measure_green_feature_value(image, local_roi);
                    ++hist_idx;
                }
            }
        }
    }

    return m_ColourSkinTemplate;

}

ip_ColourSkinTemplate
ip_ColourSkinFeatureProducer::feature_prototype(
        const ip_ColourSkinFeatureParameters& params) {
    ip_ColourSkinTemplate prototype;

    prototype.mBlueColourTemplate = valarray<float>(0.0f,
            params.mNumBlocksRow * params.mNumCellsInBlockRow *
            params.mNumBlocksCol * params.mNumCellsInBlockCol);
    prototype.mRedColourTemplate = prototype.mBlueColourTemplate;
    prototype.mGreenColourTemplate = prototype.mBlueColourTemplate;
    return prototype;
} // feature_prototype

/////////////////////////////// PRIVATE ///////////////////////////////////////

inline OpenCvPlus::real
ip_ColourSkinFeatureProducer::measure_blue_feature_value(IplImage* img,
        const ip_RoiWindow& roi) {

    const int last_column = roi.m_iFirstColumn + roi.m_iWidth - 1;
    const int last_row = roi.m_iFirstRow + roi.m_iHeight - 1;

    float result = 0;
//    cout << "CHANNELS " << img->nChannels << ", ROI " << roi << endl;
//    cout << "DEPTH " << img->depth << ", DATA " << img->imageData << endl;

    for (int row = roi.m_iFirstRow; row <= last_row; ++row) {
        for (int col = roi.m_iFirstColumn; col <= last_column; ++col) {
            result += CV_IMAGE_ELEM(img, uchar, row, col * 3);
        }
    }

    return result;

} // measure_feature_value

inline OpenCvPlus::real
ip_ColourSkinFeatureProducer::measure_green_feature_value(IplImage* img,
        const ip_RoiWindow& roi) {

    const int last_column = roi.m_iFirstColumn + roi.m_iWidth - 1;
    const int last_row = roi.m_iFirstRow + roi.m_iHeight - 1;

    float result = 0;

    for (int row = roi.m_iFirstRow; row <= last_row; ++row) {
        for (int col = roi.m_iFirstColumn; col <= last_column; ++col) {
            result += CV_IMAGE_ELEM(img, uchar, row, col * 3 + 1);
        }
    }

    return result;

} // measure_feature_value

inline OpenCvPlus::real
ip_ColourSkinFeatureProducer::measure_red_feature_value(IplImage* img,
        const ip_RoiWindow& roi) {

    const int last_column = roi.m_iFirstColumn + roi.m_iWidth - 1;
    const int last_row = roi.m_iFirstRow + roi.m_iHeight - 1;

    float result = 0;

    for (int row = roi.m_iFirstRow; row <= last_row; ++row) {
        for (int col = roi.m_iFirstColumn; col <= last_column; ++col) {
            result += CV_IMAGE_ELEM(img, uchar, row, col * 3 + 2);
        }
    }

    return result;

} // measure_feature_value

} // namespace ImageProcessing
