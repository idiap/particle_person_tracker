// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_SkinFeatureProducer - skin feature feature computation class
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <boost/lambda/lambda.hpp>   // for transformation in operators
#include <boost/foreach.hpp>         // for fabs of a vector
#include <numeric>                   // for inner product

// LOCAL INCLUDES
#include <image_processing/ip_SkinFeatureProducer.h>   // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////
ip_SkinFeatureProducer::ip_SkinFeatureProducer(
        ip_ImageProvider* ism_processor,
        const ip_SkinFeatureParameters& params) :
        mParams(params),
        mCellMaskThreshold(params.mSkinMaskValue / 2),
        mImageWidth(ism_processor->image_buffer()->width),
        mImageHeight(ism_processor->image_buffer()->height),
        mIsmProcessor(ism_processor),
        m_SkinTemplate(0.0f,
                mParams.mNumBlocksRow * mParams.mNumCellsInBlockRow *
                mParams.mNumBlocksCol * mParams.mNumCellsInBlockCol) {

} // ip_SkinFeatureProducer

const ip_SkinTemplate&
ip_SkinFeatureProducer::compute_feature(const ip_RoiWindow& roi) {

    m_SkinTemplate = 0.0f;

    // total number of cells in a row
    const unsigned num_cells_row_total =
            mParams.mNumBlocksRow * mParams.mNumCellsInBlockRow;
    // total number of cells in a column
    const unsigned num_cells_col_total =
            mParams.mNumBlocksCol * mParams.mNumCellsInBlockCol;

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

        // compute template cell values using the integral image for skin mask
        // result is represented as an array of concatenated rows

        ip_RoiWindow local_roi;       // ROI of the integral image
        int hist_idx;                 // index of a histogram feature to fill
        float row_cached_lower_limit;   // current first line of the ROI
        float col_cached_lower_limit;   // current first column of the ROI
        const int step_correction = num_cells_row_total - last_col_index - 1;
        const int base_offset = first_row_index * num_cells_row_total;
        static const float epsilon = 1e-3;

        // compute image cell indices of the cells that intersect with the
        // template cell at (i, j). They form a rectangle defined by local_roi
        // Derivation for column index: we look for all columns c such that
        // c + 1  > roi.m_iFirstColumn + i * step_col;
        // c < roi.m_iFirstColumn + (i + 1) * step_col;
        // 0 <= c <= c_max
        // Derivation for row index: the same

        IplImage * image = mIsmProcessor->image();
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

                m_SkinTemplate[hist_idx] =
                        measure_feature_value(image, local_roi);
                ++hist_idx;
            }
            hist_idx += step_correction;
        }

    }

    return m_SkinTemplate;

}

ip_SkinTemplate
ip_SkinFeatureProducer::feature_prototype(
        const ip_SkinFeatureParameters& params) {
    return ip_SkinTemplate(0.0f,
            params.mNumBlocksRow * params.mNumCellsInBlockRow *
            params.mNumBlocksCol * params.mNumCellsInBlockCol);
} // feature_prototype

/////////////////////////////// PRIVATE ///////////////////////////////////////

// half of the value that is written to the mask for skin pixels
static const float MAGIC_CONST = 1.0 / 2;

inline OpenCvPlus::real
ip_SkinFeatureProducer::measure_feature_value(IplImage* img,
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

    return (result >= mCellMaskThreshold * roi.m_iWidth * roi.m_iHeight) ?
            1.0 : 0.0;

} // measure_feature_value

OpenCvPlus::real distance_emp(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2) {

    using namespace boost::lambda;

    ip_SkinTemplate tst(st1);
    const unsigned last = st1.size();
    transform(&st1[0], &st1[last], &st2[0], &tst[0],
            boost::lambda::_1 - boost::lambda::_2);
    OpenCvPlus::real sum_sqr = inner_product(
            &tst[0], &tst[last], &tst[0], 0.0f);

    return (sum_sqr <= 0.5 * last) ? 1 : 0.0001;
} // distance_emp

//float fabs_local(float v) { return v; }

OpenCvPlus::real distance_L1(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2) {
    ip_SkinTemplate tst(st1);
    const unsigned last = st1.size();
    transform(&st1[0], &st1[last], &st2[0], &tst[0],
            boost::lambda::_1 - boost::lambda::_2);

    // cannot use transform with std::fabs : it is an inline function!
    // Two solutions: redeclare fabs locally or write a loop mylelf
    // Opted for second solution:
    OpenCvPlus::real *const pArrEnd = &tst[last];
    for (OpenCvPlus::real * pArr = &tst[0]; pArr != pArrEnd; ++pArr) {
        *pArr = fabs(*pArr);
    }

    OpenCvPlus::real sum_abs = accumulate(&tst[0], &tst[last], 0.0f);
    return sum_abs;
} // distance_L1

OpenCvPlus::real distance_L2_squared(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2) {
    ip_SkinTemplate tst(st1);
    const unsigned last = st1.size();
    transform(&st1[0], &st1[last], &st2[0], &tst[0],
            boost::lambda::_1 - boost::lambda::_2);
    OpenCvPlus::real sum_sqr = inner_product(
            &tst[0], &tst[last], &tst[0], 0.0f);
    return sum_sqr;
} // distance_L2

OpenCvPlus::real distance_mahalanobis_squared(const ip_SkinTemplate& st1,
        const ip_SkinTemplate& st2, const ip_SkinTemplate& stddevs) {

    using namespace boost::lambda;

    const unsigned last = st1.size();

//    ostream_iterator<OpenCvPlus::real> out_it(cout, " ");
//    cout << "skin template 1: ";
//    copy(&st1[0], &st1[last], out_it);
//    cout << endl;
//    cout << "skin template 2: ";
//    copy(&st2[0], &st2[last], out_it);
//    cout << endl;
//    cout << "stddevs: ";
//    copy(&stddevs[0], &stddevs[last], out_it);
//    cout << endl;

    ip_SkinTemplate tst(st1);
    transform(&st1[0], &st1[last], &st2[0], &tst[0],
            boost::lambda::_1 - boost::lambda::_2);
//    cout << "difference: ";
//    copy(&tst[0], &tst[last], out_it);
//    cout << endl;

    transform(&tst[0], &tst[last], &stddevs[0], &tst[0],
            boost::lambda::_1 / boost::lambda::_2);
//    cout << "weighted: ";
//    copy(&tst[0], &tst[last], out_it);
//    cout << endl;
    OpenCvPlus::real sum_sqr = inner_product(
            &tst[0], &tst[last], &tst[0], 0.0f);

//    cout << "distance between skin patterns: sum_sqr=" << sum_sqr << endl;
    return sum_sqr;
}

} // namespace ImageProcessing
