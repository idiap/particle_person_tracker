/**
 * @file cxx/opencvplus/src/cvp_FaceColorModel.cc
 * @date Thu May 10 17:57:08 2012 +0200
 * @author Vasil Khalidov <vasil.khalidov@idiap.ch>
 * @author Carl Scheffler <carl.scheffler@idiap.ch>
 *
 * @brief Implementation of the face colour model class.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <stdio.h>                                        // ??
#include <iostream>                                       // STL I/O
#include <fstream>                                        // STL file I/O
#include <stdexcept>                                      //
#include <algorithm>                                      // STL algorithms
#include <numeric>
#include <iomanip>
#include <cmath>

#include <boost/foreach.hpp>                              // BOOST_FOREACH loop
#include <boost/math/special_functions/fpclassify.hpp>    // for isfinite()
#include <boost/math/distributions/students_t.hpp>        // for Student t
#include <boost/lambda/lambda.hpp>                        // boost lambda exprs
#include <boost/integer/integer_log2.hpp>
#include <boost/static_assert.hpp>                        // for 32-bit int

//#include <highgui.h>

// PROJECT INCLUDES
#include <bayes_filter/bf_DiscreteDistribution.h>

// LOCAL INCLUDES
#include <opencvplus/cvp_utils.h>                         // for intersect
#include <opencvplus/FaceColorModel.h>
#include <opencvplus/cvp_helper_functions.h>              // for bgr2ypbpr

using namespace OpenCvPlus;
using namespace std;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

// number of bytes per integer value in binary data
#define NUM_BYTES_INT 4
// number of bytes per floating point value in binary data
#define NUM_BYTES_DOUBLE 8
// define 32bit integer type depending on target architecture
typedef int int_32;
// define 64bit float type depending on target architecture
typedef double double_64;

// check what sizes of int and double
// 32-bit and 64-bit are used respectively to store binary data
BOOST_STATIC_ASSERT(sizeof(int_32) == NUM_BYTES_INT);
BOOST_STATIC_ASSERT(sizeof(double_64) == NUM_BYTES_DOUBLE);

// precision for fast logarithm computation (0<=n<=23), i.e.
// number of bits used from the mantissa
static const int LOG_COMPUTATION_PRECISION = 14;
// number of adaptation results stored in a buffer for filtering
static const unsigned ADAPTATION_FEATURES_NUM = 10;
// weight of prior PIM model (0 <= w <= 1) in the adaptation process
static float ADAPTATION_PRIOR_WEIGHT = 0.5;
// adaptation rate of colour model in the adaptation process
static float ADAPTATION_COLOUR_RATE = 100;
// default number of variational updates during colour model adaptation
static const unsigned VARIATIONAL_ITERATIONS_NUM_DEFAULT = 10;
// constant used for continuous palette precomputations: -1.5*log(2*pi)
static const float CONTINUOUS_PALETTE_PRECOMPUTE_CONSTANT = -2.756815599614018;
// log |A| of transformation matrix A used in BGR to YPbPr convertion
static const float LOG_DETERMINANT_BGR2YBR = -1.4427388438401436;

//#define USE_INTEGRAL_PROBA_IMAGES
#define SQR_FAST(X) ((X) * (X))
#define ROUND_POSITIVE(X) int((X) + 0.5)

// should include prior PIM as prior term for PIM adaptation?
//#define USE_PRIOR_FOR_ADAPTATION
// should normalize colour model adaptation to PIM size?
// related to ADAPTATION_COLOUR_RATE
#define COMPENSATE_ADAPTATION_FOR_MODEL_SIZE

namespace FaceColorModel {

/////////////////////////////// PUBLIC ///////////////////////////////////////

    FaceColorModel::FaceColorModel(const FaceColorModelConfig& config,
        fcm_real iKMrf/*=2*/) :
            mConfig(config),
            mVariationalIterations(VARIATIONAL_ITERATIONS_NUM_DEFAULT),
            mKMrf(iKMrf),
            m_PriorPaletteBundle(0),
            m_CurrentPaletteBundle(0),
            m_ProbabilityMapsCacheAllocated(false),
            m_ConvertedImagesCacheAllocated(false),
            m_UsesYPbPrImage(false) {

        // load prior PIM model from file
        initialize_pim_models(mConfig.m_PimModel);
        // load prior colour models from files and perform precomputations
        try {
            initialize_colour_models();
            // allocate various buffers used on adaptation stage
            try {
                initialize_adaptation_buffers();
                // initialise current probability buffers
                // with prior buffers content
                try {
                    reset_to_prior();
                } catch (...) {
                    deinitialize_adaptation_buffers();
                    throw;
                }
            } catch (...) {
                deinitialize_colour_models();
                throw;
            }
        } catch (...) {
            deinitialize_pim_models();
            throw;
        }
    } // FaceColorModel

    FaceColorModel::~FaceColorModel() {
        if (m_ConvertedImagesCacheAllocated) {
            deallocate_converted_images_cache();
        }
        if (m_ProbabilityMapsCacheAllocated) {
            deallocate_probability_maps_cache();
        }
        deinitialize_adaptation_buffers();
        deinitialize_colour_models();
        deinitialize_pim_models();
    } // ~FaceColorModel

    CvRect FaceColorModel::face_roi2fcm_roi(const CvRect & face_roi,
            unsigned idx /* = 0 */ ) const {
        cvp_PimModel * pim_model = m_CurrentPimModels[idx];
        const CvRect& pim_face_rect = pim_model->face_rect();
        const unsigned pim_model_width = pim_model->data().width();
        const unsigned pim_model_height = pim_model->data().height();
        CvRect result;
        result.x = face_roi.x - static_cast<int>(round(static_cast<float>(
            face_roi.width * pim_face_rect.x) / pim_face_rect.width));
        result.y = face_roi.y - static_cast<int>(round(static_cast<float>(
            face_roi.height * pim_face_rect.y) / pim_face_rect.height));
        result.width = static_cast<int>(round(static_cast<float>(
            pim_model_width * face_roi.width) / pim_face_rect.width));
        result.height = static_cast<int>(round(static_cast<float>(
            pim_model_height * face_roi.height) / pim_face_rect.height));
        return result;
    } // face_roi2fcm_roi

    CvRect FaceColorModel::fcm_roi2face_roi(const CvRect & fcm_roi,
            unsigned idx /* =0 */ ) const {
        cvp_PimModel * pim_model = m_CurrentPimModels[idx];
        const CvRect& pim_face_rect = pim_model->face_rect();
        const unsigned pim_model_width = pim_model->data().width();
        const unsigned pim_model_height = pim_model->data().height();
        CvRect result;
        result.x = fcm_roi.x + static_cast<int>(round(static_cast<float>(
            fcm_roi.width * pim_face_rect.x) / pim_model_width));
        result.y = fcm_roi.y + static_cast<int>(round(static_cast<float>(
            fcm_roi.height * pim_face_rect.y) / pim_model_height));
        result.width = static_cast<int>(round(static_cast<float>(
            pim_face_rect.width * fcm_roi.width) / pim_model_width));
        result.height = static_cast<int>(round(static_cast<float>(
            pim_face_rect.height * fcm_roi.height) / pim_model_height));
        return result;
    } // fcm_roi2face_roi

    void FaceColorModel::prepare(const IplImage * ipImage) {
        // DEBUG-START
        // verify input image format
        assert((ipImage->nChannels == 3) && (ipImage->depth == IPL_DEPTH_8U));
        // DEBUG-END

        const unsigned image_width = static_cast<unsigned>(ipImage->width);
        const unsigned image_height = static_cast<unsigned>(ipImage->height);

        // allocate / reallocate caches for probability maps, if necessary
        if (!m_ProbabilityMapsCacheAllocated) {
            allocate_probability_maps_cache(FCM_NUM_CHANNELS, image_width,
                    image_height);
        } else if ((m_ProbabilityMapsCache.m_NumChannels != FCM_NUM_CHANNELS) ||
            (m_ProbabilityMapsCache.m_Width != image_width) ||
            (m_ProbabilityMapsCache.m_Height != image_height)) {
            deallocate_probability_maps_cache();
            allocate_probability_maps_cache(FCM_NUM_CHANNELS, image_width,
                    image_height);
        }

        // allocate / reallocate caches for converted images, if necessary
        if (!m_ConvertedImagesCacheAllocated) {
            allocate_converted_images_cache(image_width, image_height);
        } else if ((m_ConvertedImagesCache.m_Width != image_width) ||
                   (m_ConvertedImagesCache.m_Height != image_height)) {
            deallocate_converted_images_cache();
            allocate_converted_images_cache(image_width, image_height);
        }

} // prepare

    void FaceColorModel::cache_probability_maps(const IplImage * ipImage) {
        const CvRect roi = cvRect(0, 0, ipImage->width, ipImage->height);
        cache_probability_maps(ipImage, roi);
    } // cache_probability_maps

    void FaceColorModel::cache_probability_maps(const IplImage * ipImage,
            const CvRect& roi) {

        // DEBUG-START
        // verify input image format and cache parameters
        assert((ipImage->nChannels == 3) && (ipImage->depth == IPL_DEPTH_8U));
        assert(m_ProbabilityMapsCacheAllocated);
        assert(m_ConvertedImagesCacheAllocated);
        assert(m_ProbabilityMapsCache.m_ProbabilityMaps);
        assert(m_ConvertedImagesCache.m_CachedYPbPrImage);
        assert((m_ProbabilityMapsCache.m_ProbabilityMaps->width() ==
                    static_cast<unsigned>(ipImage->width)) &&
               (m_ProbabilityMapsCache.m_ProbabilityMaps->height() ==
                    static_cast<unsigned>(ipImage->height)));
        assert(m_ProbabilityMapsCache.m_ProbabilityMaps->channels() ==
                FCM_NUM_CHANNELS);
        // DEBUG-END

        // convert input image into histogram colours and YPrPb space
        if (m_ConvertedImagesCache.m_CachedYPbPrImage) {
            bgr2ypbpr(ipImage, m_ConvertedImagesCache.m_CachedYPbPrImage, roi);
        }
        typedef std::pair<const unsigned, IplImage*> CacheEntry;
        BOOST_FOREACH(CacheEntry& mapped_val,
                m_ConvertedImagesCache.m_CachedHistogramBinImages) {
            image_to_histogram(ipImage, mapped_val.first, mapped_val.second,
                    roi);
        }

        // compute probability maps on cached images
        compute_color_marginal_log_likelihood(
                m_ProbabilityMapsCache.m_LogProbabilityMaps, roi);

        for (unsigned i = 0; i < FCM_NUM_CHANNELS; ++i) {
            cvExp(m_ProbabilityMapsCache.m_LogProbabilityMaps->map(i),
                  m_ProbabilityMapsCache.m_ProbabilityMaps->map(i));
        }

    #ifdef USE_INTEGRAL_PROBA_IMAGES
        for (unsigned i = 0; i < FCM_NUM_CHANNELS; ++i) {
            cvIntegral(m_ProbabilityMapsCache.m_ProbabilityMaps->map(i),
                m_ProbabilityMapsCache.m_IntegralProbabilityMaps->map(i));
        }
    #endif

    } // cache_probability_maps

    CvScalar FaceColorModel::colour_likelihoods(CvScalar rgb_colour) {

        CvScalar result;

        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
          cvp_Palette * palette = m_CurrentPaletteBundle->palette(channel);

          if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
              // get precomputed means and normalization constant
              ContinuousColorPrecompute* pPrecompute =
                  (ContinuousColorPrecompute*)palette->precomputed_data();
              CvScalar ypbpr = bgr2ypbpr(rgb_colour);
              result.val[channel] =
                  exp(compute_color_marginal_log_likelihood_continuous(
                      ypbpr, pPrecompute));
          } else if (palette->type() == cvp_Palette::FCM_DISCRETE) {
              DiscreteColorPrecompute* pPrecompute =
                  (DiscreteColorPrecompute*)palette->precomputed_data();
              int histogram_index = bgr_to_histogram(rgb_colour,
                      pPrecompute->binsNumPerDimension);
              result.val[channel] =
                  exp(compute_color_marginal_log_likelihood_discrete(
                      histogram_index, palette, pPrecompute));
          }
        }
        return result;
    } // colour_likelihoods

    void FaceColorModel::compute_feature_on_cached_probability_maps(
            const CvRect & roi, PimFeatureType * feature) {

        // DEBUG-START
        // verify caches
        assert(m_ProbabilityMapsCacheAllocated);
        assert(m_ProbabilityMapsCache.m_ProbabilityMaps);
        // DEBUG-END

        // Check the feature is of the appropriate size
        const unsigned feature_numchannels = feature->channels();
        if (feature_numchannels != FCM_NUM_CHANNELS) {
            throw std::runtime_error("FaceColorModel: Output feature should have"
                    "the same number of channels as the model");
        }

        const unsigned feature_height = feature->height();
        const unsigned feature_width = feature->width();

        // step size for columns in a row
        const float step_col = static_cast<float>(roi.width) / feature_width;
        // step size for rows in a column
        const float step_row = static_cast<float>(roi.height) / feature_height;

        CvRect local_roi;
        local_roi.width = ROUND_POSITIVE(step_col);
        local_roi.height = ROUND_POSITIVE(step_row);

        PimFeatureType::value_type result;
        PimFeatureType::value_type * result_data;
        unsigned char * proba_data;
        PimFeatureType::value_type * current_row_proba_data = 0;

        for (unsigned channel = 0; channel < feature_numchannels; ++channel) {

            result = (channel == FCM_CHANNEL_BACKGROUND) ? 1 : 0;

    #ifdef USE_INTEGRAL_PROBA_IMAGES
            CvMat * integral_map = m_ProbabilityMapsCache.m_IntegralProbabilityMaps->map(channel);
    #endif

            CvMat * proba_map = m_ProbabilityMapsCache.m_ProbabilityMaps->map(channel);
            CvMat * result_map = feature->map(channel);

            const int result_step = result_map->step /
                    sizeof(PimFeatureType::value_type) - result_map->width;

            proba_data = proba_map->data.ptr;
            result_data = reinterpret_cast<PimFeatureType::value_type*>(
                    result_map->data.ptr);

            // y coordinate of the row centre
            float cur_proba_row_offset = step_row / 2;
            for (unsigned row = 0; row < feature_height; ++row) {
                local_roi.y = ROUND_POSITIVE(cur_proba_row_offset);
                // fill row parts outside the ROI wit trivial values
                if ((local_roi.y < 0) ||
                    (local_roi.y + local_roi.height > proba_map->height - 1)) {
                    for (unsigned col = 0; col < feature_width; ++col) {
                        *result_data++ = result;
                    }
                } else {
                    // go to the beginning of the row defined by local_roi.y
                    current_row_proba_data =
                            reinterpret_cast<PimFeatureType::value_type*>(
                                    proba_data + proba_map->step * local_roi.y);
                    float cur_proba_col_offset = step_col / 2;
                    for (unsigned col = 0; col < feature_width; ++col) {
                        local_roi.x = ROUND_POSITIVE(cur_proba_col_offset);
                        if ((local_roi.x < 0) ||
                            (local_roi.x + local_roi.width > proba_map->width - 1)) {
                            result = 1;
                        } else {
    #ifdef USE_INTEGRAL_PROBA_IMAGES
                            result = cvGetReal2D(integral_map,
                                        local_roi.y + local_roi.height,
                                        local_roi.x + local_roi.width) +
                                cvGetReal2D(integral_map, local_roi.y, local_roi.x) -
                                cvGetReal2D(integral_map, local_roi.y,
                                        local_roi.x + local_roi.width) -
                                cvGetReal2D(integral_map, local_roi.y + local_roi.height,
                                        local_roi.x);
    #else
                            result = *(current_row_proba_data + local_roi.x);
    #endif
                        }
                        *result_data++ = result;
                        cur_proba_col_offset += step_col;
                    }
                } // else if (local_roi.y < 0) || ...

                cur_proba_row_offset += step_row;
                result_data += result_step;

            } // end for each row
        } // end for each channel

        feature->normalise_channels();

    } // compute_feature_on_cached_probability_maps


    void FaceColorModel::adapt_to(IplImage * ipImage, const CvRect& roi,
        unsigned idx /* = 0 */, fcm_real rate_pim /* = 1*/,
        fcm_real rate_colour /* = 1*/) {

        // DEBUG-START
        // verify input image format
        assert((ipImage->nChannels == 3) && (ipImage->depth == IPL_DEPTH_8U));
        // DEBUG-END

        if ((rate_pim < numeric_limits<fcm_real>::epsilon()) &&
            (rate_colour < numeric_limits<fcm_real>::epsilon())) {
            return;
        }

        // convert detected object ROI to colour model ROI
        CvRect fcm_roi = face_roi2fcm_roi(roi);

        // extract image ROI of the right size, associated validity mask
        // and convert it to the required colour spaces
        extract_adaptation_images(ipImage, fcm_roi);

        // initialize adapted palette bundle and PIM feature
        *m_AdaptationCache.m_AdaptedPaletteBundle = *m_PriorPaletteBundle;
        *m_AdaptationCache.m_AdaptedPimFeature = m_PriorPimModels[idx]->data();

        // perform MRF iteration
        for(unsigned variationalIteration = 0;
                variationalIteration < mVariationalIterations;
                variationalIteration++) {
            // update belief over palettes
            update_adapted_palettes();
            // update belief over probability maps
            update_adapted_pim(idx);
        }

        // save adapted palettes and pims into circular buffers
        *(m_AdaptationCache.m_PaletteBundles.front()) =
                *m_AdaptationCache.m_AdaptedPaletteBundle;
        *(m_AdaptationCache.m_PimFeaturesBuffers[idx].front()) =
                *m_AdaptationCache.m_AdaptedPimFeature;
        // rotate circular buffers so that the most recent entry is at the end
        m_AdaptationCache.m_PaletteBundles.rotate(
                m_AdaptationCache.m_PaletteBundles.begin() + 1);
        m_AdaptationCache.m_PimFeaturesBuffers[idx].rotate(
                m_AdaptationCache.m_PimFeaturesBuffers[idx].begin() + 1);

        // recompute current palettes and pims from the adapted ones
        compute_current_from_adapted(idx);

    } // adapt_to

    void FaceColorModel::reset_to_prior() {
        const unsigned num_models = m_CurrentPimModels.size();
        for (unsigned idx = 0; idx < num_models; ++idx) {
            *(m_CurrentPimModels[idx]) = *(m_PriorPimModels[idx]);
        }
        *m_CurrentPaletteBundle = *m_PriorPaletteBundle;

        for (unsigned idx = 0; idx < num_models; ++idx) {
            for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
              *(m_AdaptationCache.m_PimFeaturesBuffers[idx][i]) =
                      m_PriorPimModels[idx]->data();
            }
        }

        for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
          *(m_AdaptationCache.m_PaletteBundles[i]) = *m_PriorPaletteBundle;
        }
    } // reset_to_prior

    /* static */ int FaceColorModel::bgr_to_histogram(const CvScalar& rgb_val,
            unsigned num_histogram_bins) {
        const double * pInputCurrent = rgb_val.val;
        const int bBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
        const int gBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
        const int rBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
        return bBin + num_histogram_bins * (gBin + rBin * num_histogram_bins);
    } // bgr_to_histogram

    /* static */ CvScalar FaceColorModel::histogram_to_bgr(int idx,
            unsigned num_histogram_bins) {
        CvScalar bgr_colour;
        int rBin = idx / (num_histogram_bins * num_histogram_bins);
        idx = idx % (num_histogram_bins * num_histogram_bins);
        int gBin = idx / num_histogram_bins;
        int bBin = idx % num_histogram_bins;
        bgr_colour.val[2] = double(255 * (rBin + 0.5)) / num_histogram_bins;
        bgr_colour.val[1] = double(255 * (gBin + 0.5)) / num_histogram_bins;
        bgr_colour.val[0] = double(255 * (bBin + 0.5)) / num_histogram_bins;
        return bgr_colour;
    } // bgr_to_histogram

    /* static */ void
    FaceColorModel::image_to_histogram(const IplImage * ipImage,
        unsigned num_histogram_bins,
        IplImage* opHistogram, const CvRect& roi) {

        const int start_row = roi.y;
        const int end_row = roi.y + roi.height;
        const int start_col= roi.x;
        const int end_col = roi.x + roi.width;

        // verify the provided data is appropriate
        assert((ipImage->depth == IPL_DEPTH_8U) && (ipImage->nChannels == 3));
        assert((opHistogram->depth == IPL_DEPTH_32S) &&
               (opHistogram->nChannels == 1));
        assert(ipImage->height == opHistogram->height);
        assert(ipImage->width == opHistogram->width);
        assert((roi.y >= 0) && (roi.x >= 0) &&
               (roi.height >= 0) && (roi.width >= 0) &&
               (end_row <= ipImage->height) &&
               (end_col <= ipImage->width));

        typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32s;

        const unsigned char * pImageData =
                (unsigned char *)ipImage->imageData;
        unsigned char * pHistogramData =
                (unsigned char *)opHistogram->imageData;

        const unsigned char * pInputCurrentStart = pImageData +
                roi.y * ipImage->widthStep + roi.x * 3;
        unsigned char * pOutputCurrentStart = pHistogramData +
                roi.y * opHistogram->widthStep + roi.x * 4;

        // compute histogram indices for ROI pixels only
        for(int row = start_row; row < end_row; ++row) {
            const unsigned char * pInputCurrent = pInputCurrentStart;
            int32s * pOutputCurrent = (int32s *) pOutputCurrentStart;
            for(int col = start_col; col < end_col; ++col) {
                const int bBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                        >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
                const int gBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                        >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
                const int rBin = ((int)(*pInputCurrent++) * num_histogram_bins)
                        >> 8; // bin index value in [0..NUM_HISTOGRAM_BINS-1]
                *pOutputCurrent++ =
                    bBin + num_histogram_bins *
                        (gBin + rBin * num_histogram_bins);
            }
            // move to the next row in the input and output images
            pInputCurrentStart += ipImage->widthStep;
            pOutputCurrentStart += opHistogram->widthStep;
        }
    } // image_to_histogram

/////////////////////////////// PRIVATE //////////////////////////////////////

    void FaceColorModel::allocate_probability_maps_cache(unsigned num_channels,
            unsigned width, unsigned height) {

        m_ProbabilityMapsCache.m_NumChannels = num_channels;
        m_ProbabilityMapsCache.m_Width = width;
        m_ProbabilityMapsCache.m_Height = height;
        m_ProbabilityMapsCache.m_LogProbabilityMaps =
                new PimFeatureType(num_channels, width, height);
        m_ProbabilityMapsCache.m_ProbabilityMaps =
                new PimFeatureType(num_channels, width, height);
        m_ProbabilityMapsCache.m_IntegralProbabilityMaps =
                new PimIntegralFeatureType(num_channels, width + 1, height + 1);
        m_ProbabilityMapsCacheAllocated = true;

    } // allocate_probability_maps_cache

    void FaceColorModel::deallocate_probability_maps_cache() {
        m_ProbabilityMapsCacheAllocated = false;
        m_ProbabilityMapsCache.m_NumChannels = 0;
        m_ProbabilityMapsCache.m_Width = 0;
        m_ProbabilityMapsCache.m_Height = 0;
        delete m_ProbabilityMapsCache.m_LogProbabilityMaps;
        m_ProbabilityMapsCache.m_LogProbabilityMaps = 0;
        delete m_ProbabilityMapsCache.m_ProbabilityMaps;
        m_ProbabilityMapsCache.m_ProbabilityMaps = 0;
        delete m_ProbabilityMapsCache.m_IntegralProbabilityMaps;
        m_ProbabilityMapsCache.m_IntegralProbabilityMaps = 0;
    } // deallocate_probability_maps_cache

    void FaceColorModel::allocate_converted_images_cache(
            unsigned width, unsigned height) {
        m_ConvertedImagesCache.m_Width = width;
        m_ConvertedImagesCache.m_Height = height;

        const unsigned palettes_num = m_CurrentPaletteBundle->size();
        for (unsigned palette_idx = 0; palette_idx < palettes_num; ++palette_idx) {
            cvp_Palette * palette = m_CurrentPaletteBundle->palette(palette_idx);
            if (palette->type() == cvp_Palette::FCM_DISCRETE) {
                DiscreteColorPrecompute * precompute =
                    static_cast<DiscreteColorPrecompute*>(
                        palette->precomputed_data());
                const unsigned num_bins = precompute->binsNumPerDimension;
                if (m_ConvertedImagesCache.m_CachedHistogramBinImages.find(num_bins) ==
                        m_ConvertedImagesCache.m_CachedHistogramBinImages.end()) {
                    IplImage * image = cvCreateImage(
                            cvSize(width, height), IPL_DEPTH_32S, 1);
                    m_ConvertedImagesCache.m_CachedHistogramBinImages[num_bins] = image;
                }
            }
        }

        if (m_UsesYPbPrImage) {
            m_ConvertedImagesCache.m_CachedYPbPrImage =
                cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
        }

        m_ConvertedImagesCacheAllocated = true;
    } // allocate_converted_images_cache

    void FaceColorModel::deallocate_converted_images_cache() {
        m_ConvertedImagesCacheAllocated = false;
        m_ConvertedImagesCache.m_Width = 0;
        m_ConvertedImagesCache.m_Height = 0;
        if (m_ConvertedImagesCache.m_CachedYPbPrImage) {
            cvReleaseImage(&m_ConvertedImagesCache.m_CachedYPbPrImage);
            m_ConvertedImagesCache.m_CachedYPbPrImage = 0;
        }
        typedef pair<const unsigned, IplImage*> CacheEntry;
        BOOST_FOREACH(CacheEntry& mapped_val,
                m_ConvertedImagesCache.m_CachedHistogramBinImages) {
            cvReleaseImage(&mapped_val.second);
        }
        m_ConvertedImagesCache.m_CachedHistogramBinImages.clear();
    } // deallocate_converted_images_cache

    void FaceColorModel::initialize_pim_models(const std::string& fnames_str) {
        list<string> fnames;
        parse_pim_files_string(fnames_str, fnames);
        m_PriorPimModels.resize(fnames.size());
        m_CurrentPimModels.resize(fnames.size());

        unsigned idx = 0;
        BOOST_FOREACH(const string& fname, fnames) {
            m_PriorPimModels[idx] = cvp_PimModel::load_from_file(fname);
            m_CurrentPimModels[idx] = new cvp_PimModel(*(m_PriorPimModels[idx]));
            ++idx;
        }
    } // initialize_pim_model

    void FaceColorModel::deinitialize_pim_models() {
        unsigned num_models = m_PriorPimModels.size();
        for (unsigned idx = 0; idx < num_models; ++idx) {
            delete m_PriorPimModels[idx];
            delete m_CurrentPimModels[idx];
        }
    } // deinitialize_pim_model

    void FaceColorModel::parse_pim_files_string(const string& fnames_str,
        list<string>& fnames) {

        string fnames_remaining = fnames_str;
        size_t pos_colon = fnames_remaining.find(':');
        while ((pos_colon != string::npos)) {
            string file_left = fnames_remaining.substr(0, pos_colon);
            fnames.push_back(file_left);
            fnames_remaining = fnames_remaining.substr(pos_colon + 1);
            pos_colon = fnames_remaining.find(':');
        }
        fnames.push_back(fnames_remaining);
    } // parse_pim_files_string

    void FaceColorModel::initialize_colour_models() {
        // fill palette infos
        vector<string> palette_infos(FCM_NUM_CHANNELS);
        palette_infos[FCM_CHANNEL_BACKGROUND] = mConfig.m_BackgroundColourModel;
        palette_infos[FCM_CHANNEL_CLOTHES] = mConfig.m_ClothesColourModel;
        palette_infos[FCM_CHANNEL_SKIN] = mConfig.m_SkinColourModel;
        palette_infos[FCM_CHANNEL_HAIR] = mConfig.m_HairColourModel;

        // load bundles using infos
        m_PriorPaletteBundle = cvp_PaletteBundle::load_from_files(palette_infos);
        m_CurrentPaletteBundle = new cvp_PaletteBundle(*m_PriorPaletteBundle);

        // set m_UsesYPbPrImage if YPbPr image should be computed
        const unsigned palettes_num = m_CurrentPaletteBundle->size();
        for (unsigned palette_idx = 0; palette_idx < palettes_num; ++palette_idx) {
          cvp_Palette * palette = m_CurrentPaletteBundle->palette(palette_idx);
          if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
              m_UsesYPbPrImage = true;
              break;
          }
        }

    } // initialize_colour_models

    void FaceColorModel::deinitialize_colour_models() {
        // delete bundles
        delete m_PriorPaletteBundle;
        delete m_CurrentPaletteBundle;
    } // deinitialize_colour_models

    void FaceColorModel::initialize_adaptation_buffers() {

        const unsigned pim_model_width = m_CurrentPimModels[0]->data().width();
        const unsigned pim_model_height = m_CurrentPimModels[0]->data().height();
        const unsigned num_models = m_CurrentPimModels.size();

        // allocate and initialize adaptation PIM model collections
        m_AdaptationCache.m_PimFeaturesBuffers.resize(num_models);
        for (unsigned idx = 0; idx < num_models; ++idx) {
            m_AdaptationCache.m_PimFeaturesBuffers[idx].resize(
                    ADAPTATION_FEATURES_NUM);
            for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
                m_AdaptationCache.m_PimFeaturesBuffers[idx].push_back(
                      new PimFeatureType(m_PriorPimModels[idx]->data()));
            }
        }

        // allocate and initialize adaptation palette collections
        m_AdaptationCache.m_PaletteBundles.resize(ADAPTATION_FEATURES_NUM);
        for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
          m_AdaptationCache.m_PaletteBundles.push_back(
                  new cvp_PaletteBundle(*m_PriorPaletteBundle));
        }

        // initialize adaptation palette and PIM buffers
        m_AdaptationCache.m_AdaptedPaletteBundle =
              new cvp_PaletteBundle(*m_PriorPaletteBundle);
        m_AdaptationCache.m_AdaptedPimFeature =
              new PimFeatureType(m_PriorPimModels[0]->data());
        m_AdaptationCache.m_AdaptedLogPimFeature =
              new PimFeatureType(m_PriorPimModels[0]->data());
        m_AdaptationCache.m_PriorLogPimFeature =
              new PimFeatureType(m_PriorPimModels[0]->data());

        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            cvLog(m_PriorPimModels[0]->data().map(channel),
                  m_AdaptationCache.m_PriorLogPimFeature->map(channel));
        }

        // adaptation image and adaptation warp matrix
        m_AdaptationCache.mImageAdaptationBuffer = cvCreateImage(
              cvSize(pim_model_width, pim_model_height), IPL_DEPTH_8U, 3);
        m_AdaptationCache.mImageAdaptationMask = cvCreateImage(
              cvSize(pim_model_width, pim_model_height), IPL_DEPTH_8U, 1);
        m_AdaptationCache.mImageAdaptationWarpMatrix =
              cvCreateMat(2, 3, CV_32FC1);

        // initialize YPbPr image cache for adaptation
        m_AdaptationCache.m_YPbPrImage = (m_UsesYPbPrImage ?
          cvCreateImage(
              cvSize(pim_model_width, pim_model_height), IPL_DEPTH_32F, 3) :
          0);

        // initialize histogram image caches for adaptation
        const unsigned palettes_num = m_CurrentPaletteBundle->size();
        for (unsigned palette_idx = 0; palette_idx < palettes_num; ++palette_idx) {
            cvp_Palette * palette = m_CurrentPaletteBundle->palette(palette_idx);
            if (palette->type() == cvp_Palette::FCM_DISCRETE) {
                DiscreteColorPrecompute * precompute =
                  static_cast<DiscreteColorPrecompute*>(
                      palette->precomputed_data());
                const unsigned num_bins = precompute->binsNumPerDimension;
                if (m_AdaptationCache.m_HistogramBinImages.find(num_bins) ==
                      m_AdaptationCache.m_HistogramBinImages.end()) {
                  IplImage * image = cvCreateImage(
                      cvSize(pim_model_width, pim_model_height),
                      IPL_DEPTH_32S, 1);
                  m_AdaptationCache.m_HistogramBinImages[num_bins] = image;
                }
            }
        }
    } // initialize_adaptation_buffers

    void FaceColorModel::deinitialize_adaptation_buffers() {

        // delete histogram image caches for adaptation
        typedef pair<const unsigned, IplImage*> CacheEntry;
        BOOST_FOREACH(CacheEntry& mapped_val,
              m_AdaptationCache.m_HistogramBinImages) {
          cvReleaseImage(&mapped_val.second);
        }
        m_AdaptationCache.m_HistogramBinImages.clear();

        if (m_UsesYPbPrImage) {
          // delete YPbPr image cache for adaptation
          cvReleaseImage(&m_AdaptationCache.m_YPbPrImage);
          m_AdaptationCache.m_YPbPrImage = 0;
        }

        // adaptation image and adaptation warp matrix
        cvReleaseMat(&m_AdaptationCache.mImageAdaptationWarpMatrix);
        cvReleaseImage(&m_AdaptationCache.mImageAdaptationMask);
        cvReleaseImage(&m_AdaptationCache.mImageAdaptationBuffer);

        // delete adaptation palette and PIM buffers
        delete m_AdaptationCache.m_PriorLogPimFeature;
        delete m_AdaptationCache.m_AdaptedPimFeature;
        delete m_AdaptationCache.m_AdaptedLogPimFeature;
        delete m_AdaptationCache.m_AdaptedPaletteBundle;

        const unsigned num_models = m_CurrentPimModels.size();
        for (unsigned idx = 0; idx < num_models; ++idx) {
            // delete PIM model collections
            for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
              delete m_AdaptationCache.m_PimFeaturesBuffers[idx][i];
            }
            m_AdaptationCache.m_PimFeaturesBuffers[idx].resize(0);
        }
        m_AdaptationCache.m_PimFeaturesBuffers.resize(0);

        // delete adaptation palette collections
        for (unsigned i = 0; i < ADAPTATION_FEATURES_NUM; ++i) {
          delete m_AdaptationCache.m_PaletteBundles[i];
        }
        m_AdaptationCache.m_PaletteBundles.resize(0);

    } // deinitialize_adaptation_buffers

    void FaceColorModel::extract_adaptation_images(IplImage * image,
          const CvRect& roi) {

        // warp image ROI (x0, y0, w, h) into RGB buffer (0, 0, W, H)
        CvMat * warp_mat = m_AdaptationCache.mImageAdaptationWarpMatrix;
        const float alpha = static_cast<float>(
              m_AdaptationCache.mImageAdaptationBuffer->width) / roi.width;
        const float beta = static_cast<float>(
              m_AdaptationCache.mImageAdaptationBuffer->height) / roi.height;
        CV_MAT_ELEM(*warp_mat, float, 0, 0) = alpha;
        CV_MAT_ELEM(*warp_mat, float, 0, 1) = 0;
        CV_MAT_ELEM(*warp_mat, float, 0, 2) = -alpha * roi.x;
        CV_MAT_ELEM(*warp_mat, float, 1, 0) = 0;
        CV_MAT_ELEM(*warp_mat, float, 1, 1) = beta;
        CV_MAT_ELEM(*warp_mat, float, 1, 2) = -beta * roi.y;

        try {
            cvWarpAffine(image, m_AdaptationCache.mImageAdaptationBuffer,
                    warp_mat);
        } catch (...) {
            stringstream oss;
            oss << "Face colour model adaptation: error while extracting"
                    "image patch! (" << roi.x << ", " << roi.y << ", " <<
                    roi.width << ", " << roi.height << ")";
            throw cvp_Exception(oss.str());
        }

        // compute mask for the RGB buffer
        cvZero(m_AdaptationCache.mImageAdaptationMask);
        CvRect roi_image_src;
        roi_image_src.x = static_cast<int>(-alpha * roi.x);
        roi_image_src.y = static_cast<int>(-beta * roi.y);
        roi_image_src.width = static_cast<int>(alpha * (image->width - 1) + 1);
        roi_image_src.height = static_cast<int>(beta * (image->height - 1) + 1);
        CvRect roi_image_adapt = cvRect(0, 0,
                m_AdaptationCache.mImageAdaptationBuffer->width,
                m_AdaptationCache.mImageAdaptationBuffer->height);
        CvRect roi_mask;
        if (intersect(roi_image_src, roi_image_adapt, roi_mask)) {
            cvSetImageROI(m_AdaptationCache.mImageAdaptationMask, roi_mask);
            cvSet(m_AdaptationCache.mImageAdaptationMask, cvScalarAll(255));
            cvResetImageROI(m_AdaptationCache.mImageAdaptationMask);
        } else {
            return;
        }

        typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;
        typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32s;

        // convert to YPbPr image if necessary
        if (m_UsesYPbPrImage) {
            bgr2ypbpr(m_AdaptationCache.mImageAdaptationBuffer,
                m_AdaptationCache.m_YPbPrImage, roi_mask);
        }

        // convert to histogram images if necessary
        typedef std::pair<const unsigned, IplImage*> CacheEntry;
        BOOST_FOREACH(CacheEntry& mapped_val,
                m_AdaptationCache.m_HistogramBinImages) {
            image_to_histogram(m_AdaptationCache.mImageAdaptationBuffer,
                    mapped_val.first, mapped_val.second, roi_mask);
        }

    } // extract_adaptation_images

    void FaceColorModel::update_adapted_palettes() {
        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            CvMat * pim = m_AdaptationCache.m_AdaptedPimFeature->map(channel);
            cvp_Palette * palette =
                    m_AdaptationCache.m_AdaptedPaletteBundle->palette(channel);
            if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
//                ContinuousColorPrecompute* pPrecompute =
//                    (ContinuousColorPrecompute*)palette->precomputed_data();
                update_adapted_palette_continuous(
                    m_AdaptationCache.m_YPbPrImage,
                    m_AdaptationCache.mImageAdaptationMask, pim, palette);
            } else {
                DiscreteColorPrecompute* pPrecompute =
                    (DiscreteColorPrecompute*)palette->precomputed_data();
                update_adapted_palette_discrete(
                    m_AdaptationCache.m_HistogramBinImages[
                        pPrecompute->binsNumPerDimension],
                    m_AdaptationCache.mImageAdaptationMask, pim, palette);
            }
        }
    } // update_adapted_palettes

    void FaceColorModel::update_adapted_palette_continuous(
        IplImage * imageYPbPr, IplImage * imageMask, CvMat * pim_layer,
        OpenCvPlus::cvp_Palette * palette) {

        // 0, 1 and 2 order moments of YPbPr colour components
        fcm_real moment_0 = 0;
        fcm_real moment_1[3] = {0,0,0};
        fcm_real moment_2[3] = {0,0,0};

        const unsigned char * pCurrentYPbPrRow =
                (const unsigned char *)imageYPbPr->imageData;
        const unsigned char * pCurrentMaskRow =
                (const unsigned char *)imageMask->imageData;
        const unsigned char * pCurrentPimRow =
                (const unsigned char *)pim_layer->data.ptr;

        typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;

#ifdef COMPENSATE_ADAPTATION_FOR_MODEL_SIZE
        int model_size = 0;
#endif

        // compute colour moments
        for (int row = 0; row < imageYPbPr->height; ++row) {
            float32 * pYPbPrVal = (float32*) pCurrentYPbPrRow;
            unsigned char * pMaskVal = (unsigned char*) pCurrentMaskRow;
            PimFeatureType::value_type * pPimVal =
                    (PimFeatureType::value_type *) pCurrentPimRow;
            for (int col = 0; col < imageYPbPr->width; ++col) {
                const PimFeatureType::value_type pim_val = *pPimVal++;
                float32 colour_comp;
                fcm_real x;
                if (*pMaskVal++) {
                    moment_0 += pim_val;
                    for (int i = 0; i < 3; ++i) {
                        colour_comp = *pYPbPrVal++;
                        x = colour_comp * pim_val;
                        moment_1[i] += x;
                        moment_2[i] += x * colour_comp;
                    }
#ifdef COMPENSATE_ADAPTATION_FOR_MODEL_SIZE
                    ++model_size;
#endif
                } else {
                    pYPbPrVal++;
                    pYPbPrVal++;
                    pYPbPrVal++;
                }
            }
            pCurrentYPbPrRow += imageYPbPr->widthStep;
            pCurrentMaskRow += imageMask->widthStep;
            pCurrentPimRow += pim_layer->step;
        }

#ifdef COMPENSATE_ADAPTATION_FOR_MODEL_SIZE
        moment_0 *= ADAPTATION_COLOUR_RATE / model_size;
        for (int i = 0; i < 3; ++i) {
            moment_1[i] *= ADAPTATION_COLOUR_RATE / model_size;
            moment_2[i] *= ADAPTATION_COLOUR_RATE / model_size;
        }
#endif

        // compute distribution parameters based on moments
        cvp_Palette::Data& palette_data = palette->data();
        fcm_real sqr1, sqr2;

        // eta   = m_PaletteData[0+4*i];
        // tau   = m_PaletteData[1+4*i];
        // alpha = m_PaletteData[2+4*i];
        // beta  = m_PaletteData[3+4*i];
        for(int i = 0; i < 3; i++) {
            const fcm_real eta_old = palette_data[4 * i];
            const fcm_real tau_old = palette_data[4 * i + 1];
            palette_data[4 * i + 1] = tau_old + moment_0;
            palette_data[4 * i] = (eta_old * tau_old + moment_1[i])
                    / palette_data[4 * i + 1];
            palette_data[4 * i + 2] = palette_data[4 * i + 2] + moment_0 / 2;
            sqr1 = eta_old;
            sqr2 = palette_data[4 * i];
            palette_data[4 * i + 3] =
                    palette_data[4 * i + 3] + (moment_2[i] +
                    SQR_FAST(sqr1) * tau_old -
                    SQR_FAST(sqr2) * palette_data[4 * i + 1]) / 2;
        }

        // precompute palette constants based on this data
        palette->precompute();
    } // update_adapted_palette_continuous

    void FaceColorModel::update_adapted_palette_discrete(IplImage * imageHist,
        IplImage * imageMask, CvMat * pim_layer,
        OpenCvPlus::cvp_Palette * palette) {

        const unsigned char * pCurrentImgRow =
                (const unsigned char *)imageHist->imageData;
        const unsigned char * pCurrentMaskRow =
                (const unsigned char *)imageMask->imageData;
        const unsigned char * pCurrentPimRow =
                (const unsigned char *)pim_layer->data.ptr;

        cvp_Palette::Data& palette_data = palette->data();

        typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32;

        for (int row = 0; row < imageHist->height; ++row) {
            int32 * pHistVal = (int32*) pCurrentImgRow;
            unsigned char * pMaskVal = (unsigned char*) pCurrentMaskRow;
            PimFeatureType::value_type * pPimVal =
                    (PimFeatureType::value_type *) pCurrentPimRow;
            for (int col = 0; col < imageHist->width; ++col) {
                if (*pMaskVal++) {
                    palette_data[*pHistVal++] += *pPimVal++;
                } else {
                    pHistVal++;
                    pPimVal++;
                }
            }
            pCurrentImgRow += imageHist->widthStep;
            pCurrentMaskRow += imageMask->widthStep;
            pCurrentPimRow += pim_layer->step;
        }
        palette->precompute();
    } // update_adapted_palette_discrete

    void FaceColorModel::update_adapted_pim(unsigned idx) {

        // compute log PIMs
        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            cvLog(m_AdaptationCache.m_AdaptedPimFeature->map(channel),
                  m_AdaptationCache.m_AdaptedLogPimFeature->map(channel));
        }

//        fcm_real kMrf = (mKMrf * variationalIteration) / mVariationalIterations;
        fcm_real kMrf = mKMrf;

        unsigned char * pPimPrevRow;
        unsigned char * pPimCurRow;
        unsigned char * pPimNextRow;
        unsigned char * pLogPimCurRow;
        unsigned char * pPriorPimCurRow;
        unsigned char * pLogPriorPimCurRow;

        typedef PimFeatureType::value_type Value;
        Value * cur_logpim;
        Value * cur_priorpim;
        Value * cur_logpriorpim;
        Value * prev_row;
        Value * next_row;
        Value * cur_row;

        // Add contribution from Markov random field
        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            CvMat * log_pim =
                    m_AdaptationCache.m_AdaptedLogPimFeature->map(channel);
            CvMat * pim =
                    m_AdaptationCache.m_AdaptedPimFeature->map(channel);
            CvMat * prior_pim = m_PriorPimModels[idx]->data().map(channel);
            CvMat * log_prior_pim = m_AdaptationCache.m_PriorLogPimFeature->
                    map(channel);

            // First row
            pLogPimCurRow = log_pim->data.ptr;
            pPriorPimCurRow = prior_pim->data.ptr;
            pLogPriorPimCurRow = log_prior_pim->data.ptr;
            pPimCurRow = pim->data.ptr;
            pPimNextRow = pPimCurRow + pim->step;

            cur_logpim = (Value *) pLogPimCurRow;
            cur_priorpim = (Value *) pPriorPimCurRow;
            cur_logpriorpim = (Value *) pLogPriorPimCurRow;
            cur_row = (Value *) pPimCurRow;
            next_row = (Value *) pPimNextRow;

#ifdef USE_PRIOR_FOR_ADAPTATION
#define LOG_PIM (*cur_logpriorpim);
#else
#define LOG_PIM (*cur_logpim);
#endif

            *cur_logpim = kMrf * (*next_row++ + *(cur_row + 1)) + LOG_PIM;
            cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
            for (int col = 1; col < pim->width - 1; ++col) {
                *cur_logpim =
                    kMrf * (*(cur_row - 1) + *next_row++ + *(cur_row + 1)) +
                    LOG_PIM;
                cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
            }
            *cur_logpim = kMrf * (*(cur_row - 1) + *next_row) + LOG_PIM;

            pPimPrevRow = pPimCurRow;
            pPimCurRow = pPimNextRow;
            pPimNextRow += pim->step;
            pLogPimCurRow += log_pim->step;
            pPriorPimCurRow += prior_pim->step;
            pLogPriorPimCurRow += log_prior_pim->step;

            // Second row till the one before last
            for (int row = 1; row < pim->height - 1; ++row) {
                cur_logpim = (Value *) pLogPimCurRow;
                cur_priorpim = (Value *) pPriorPimCurRow;
                cur_logpriorpim = (Value *) pLogPriorPimCurRow;
                prev_row = (Value *) pPimPrevRow;
                cur_row = (Value *) pPimCurRow;
                next_row = (Value *) pPimNextRow;

                *cur_logpim = kMrf * (*prev_row++ + *next_row++ + *(cur_row + 1)) +
                        LOG_PIM;
                cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
                for (int col = 1; col < pim->width - 1; ++col) {
                    *cur_logpim = kMrf *
                        (*(cur_row - 1) + *prev_row++ + *next_row++ +
                         *(cur_row + 1)) + LOG_PIM;
                    cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
                }
                *cur_logpim = kMrf *
                        (*(cur_row - 1) + *prev_row++ + *next_row++) + LOG_PIM;

                pPimPrevRow = pPimCurRow;
                pPimCurRow = pPimNextRow;
                pPimNextRow += pim->step;
                pLogPimCurRow += log_pim->step;
                pPriorPimCurRow += prior_pim->step;
                pLogPriorPimCurRow += log_prior_pim->step;
            }

            // Last row
            cur_logpim = (Value *) pLogPimCurRow;
            cur_priorpim = (Value *) pPriorPimCurRow;
            cur_logpriorpim = (Value *) pLogPriorPimCurRow;
            prev_row = (Value *) pPimPrevRow;
            cur_row = (Value *) pPimCurRow;

            *cur_logpim = kMrf * (*prev_row++ + *(cur_row + 1)) + LOG_PIM;
            cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
            for (int col = 1; col < pim->width - 1; ++col) {
                *cur_logpim = kMrf *
                    (*(cur_row - 1) + *prev_row++ + *(cur_row + 1)) + LOG_PIM;
                cur_logpim++; cur_logpriorpim++; cur_priorpim++; cur_row++;
            }
            *cur_logpim = kMrf * (*(cur_row - 1) + *prev_row) + LOG_PIM;
        }

        // Add contributions from color likelihoods
        update_adapted_pim_color_contribution();

        // convert log PIM to PIM, normalize PIM
        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            cvExp(m_AdaptationCache.m_AdaptedLogPimFeature->map(channel),
                  m_AdaptationCache.m_AdaptedPimFeature->map(channel));
        }
        m_AdaptationCache.m_AdaptedPimFeature->normalise_channels();
    } // update_adapted_pim

    void FaceColorModel::update_adapted_pim_color_contribution() {

        IplImage * mask_image = m_AdaptationCache.mImageAdaptationMask;

        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            CvMat * log_pim =
                    m_AdaptationCache.m_AdaptedLogPimFeature->map(channel);
            cvp_Palette * palette =
                m_AdaptationCache.m_AdaptedPaletteBundle->palette(channel);

            if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
                ContinuousColorPrecompute* pPrecompute =
                    (ContinuousColorPrecompute*)palette->precomputed_data();
                const float log_constant = pPrecompute->logConstantUpdate;
                const float ao2b0 = pPrecompute->alphaOver2Beta[0];
                const float ao2b1 = pPrecompute->alphaOver2Beta[1];
                const float ao2b2 = pPrecompute->alphaOver2Beta[2];
                const float mu0 = pPrecompute->mu[0];
                const float mu1 = pPrecompute->mu[1];
                const float mu2 = pPrecompute->mu[2];

                typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;
                IplImage * image = m_AdaptationCache.m_YPbPrImage;

                unsigned char * pCurrentImgRow = (unsigned char *) image->imageData;
                unsigned char * pCurrentMaskRow = (unsigned char *) mask_image->imageData;
                unsigned char * pCurrentPimRow = log_pim->data.ptr;

                for (int row = 0; row < image->height; ++row) {
                    float32 * pImgVal = (float32*) pCurrentImgRow;
                    unsigned char * pMaskVal = (unsigned char*) pCurrentMaskRow;
                    PimFeatureType::value_type * pPimVal =
                            (PimFeatureType::value_type *) pCurrentPimRow;
                    for (int col = 0; col < image->width; ++col) {
                        if (*pMaskVal++) {
                            float accumulator = 0;
                            float sqr_buf = *pImgVal++ - mu0;
                            accumulator += ao2b0 * sqr_buf * sqr_buf;
                            sqr_buf = *pImgVal++ - mu1;
                            accumulator += ao2b1 * sqr_buf * sqr_buf;
                            sqr_buf = *pImgVal++ - mu2;
                            accumulator += ao2b2 * sqr_buf * sqr_buf;
                            *pPimVal++ += log_constant - accumulator;
                        } else {
                            pImgVal++;
                            pImgVal++;
                            pImgVal++;
                            pPimVal++;
                        }
                    }
                    pCurrentImgRow += image->widthStep;
                    pCurrentMaskRow += mask_image->widthStep;
                    pCurrentPimRow += log_pim->step;
                }

            } else {

                DiscreteColorPrecompute* pPrecompute =
                    (DiscreteColorPrecompute*)palette->precomputed_data();
                const float log_constant = pPrecompute->logConstantUpdate;

                typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32;
                IplImage * image = m_AdaptationCache.m_HistogramBinImages[
                    pPrecompute->binsNumPerDimension];

                unsigned char * pCurrentImgRow = (unsigned char *) image->imageData;
                unsigned char * pCurrentMaskRow = (unsigned char *) mask_image->imageData;
                unsigned char * pCurrentPimRow = log_pim->data.ptr;

                for (int row = 0; row < image->height; ++row) {
                    int32 * pImgVal = (int32*) pCurrentImgRow;
                    unsigned char * pMaskVal = (unsigned char*) pCurrentMaskRow;
                    PimFeatureType::value_type * pPimVal =
                            (PimFeatureType::value_type *) pCurrentPimRow;
                    for (int col = 0; col < image->width; ++col) {
                        if (*pMaskVal++) {
                            *pPimVal++ += pPrecompute->digammaDistribution[
                                          *pImgVal++] - log_constant;
                        } else {
                            pImgVal++;
                            pPimVal++;
                        }
                    }
                    pCurrentImgRow += image->widthStep;
                    pCurrentMaskRow += mask_image->widthStep;
                    pCurrentPimRow += log_pim->step;
                }
            }
        }
    } // update_adapted_pim_color_contribution

    void FaceColorModel::compute_current_from_adapted(unsigned idx) {
        // update current palettes
//        *m_CurrentPaletteBundle = *m_AdaptationCache.m_AdaptedPaletteBundle;
        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
            cvp_Palette * palette = m_CurrentPaletteBundle->palette(channel);
            if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
                compute_current_from_adapted_palette_continuous(
                        channel, palette);
            } else if (palette->type() == cvp_Palette::FCM_DISCRETE) {
                compute_current_from_adapted_palette_discrete(
                        channel, palette);
            }
            palette->precompute();
        }
        // update current pim:
        // 1) set to prior
        PimFeatureType & current_pim = m_CurrentPimModels[idx]->data();
        current_pim = m_PriorPimModels[idx]->data();
        // 2) multiply by ADAPTATION_PRIOR_WEIGHT
        const unsigned num_channels = current_pim.channels();
        for (unsigned channel = 0; channel < num_channels; ++channel) {
            cvConvertScale(current_pim.map(channel), current_pim.map(channel),
                ADAPTATION_PRIOR_WEIGHT);
        }
        // 3) add adapted PIMs multiplied by (1 - ADAPTATION_PRIOR_WEIGHT)
        typedef boost::circular_buffer<PimFeatureType*> PimBuffer;
        const PimBuffer::const_iterator first =
                m_AdaptationCache.m_PimFeaturesBuffers[idx].begin();
        const PimBuffer::const_iterator last =
                m_AdaptationCache.m_PimFeaturesBuffers[idx].end();
        const unsigned buffer_len = m_AdaptationCache.m_PimFeaturesBuffers[idx].size();
        for (PimBuffer::const_iterator i = first; i != last; ++i) {
            for (unsigned channel = 0; channel < num_channels; ++channel) {
                cvConvertScale((*i)->map(channel),
                    m_AdaptationCache.m_AdaptedPimFeature->map(channel),
                    (1.0 - ADAPTATION_PRIOR_WEIGHT) / buffer_len);
                cvAdd(current_pim.map(channel),
                      m_AdaptationCache.m_AdaptedPimFeature->map(channel),
                      current_pim.map(channel));
            }
        }
        // 4) normalize PIMs
        current_pim.normalise_channels();
    } // compute_current_from_adapted

    void FaceColorModel::compute_current_from_adapted_palette_continuous(
        unsigned channel, cvp_Palette * palette) {

        #define COLOUR_SPACE_DIMS 3

        typedef boost::circular_buffer<cvp_PaletteBundle*> PaletteBuffer;
        typedef cvp_Palette::DataType Real;
        const PaletteBuffer::const_iterator first =
                m_AdaptationCache.m_PaletteBundles.begin();
        const PaletteBuffer::const_iterator last =
                m_AdaptationCache.m_PaletteBundles.end();
        cvp_Palette * prior_palette = m_PriorPaletteBundle->palette(channel);
        const unsigned buffer_len = m_AdaptationCache.m_PaletteBundles.size();
        Real sum_tau[] = {0, 0, 0};
        Real sum_alpha[] = {0, 0, 0};
        Real sum_etatau[] = {0, 0, 0};
        Real sum_Ec2[] = {0, 0, 0};
        for (PaletteBuffer::const_iterator i = first; i != last; ++i) {
            cvp_Palette * adapted_palette = (*i)->palette(channel);
            Real * data_ptr = &adapted_palette->data()[0];
            Real * prior_data_ptr = &prior_palette->data()[0];
            for (int j = 0; j < COLOUR_SPACE_DIMS; ++j) {
                Real eta_ij = *data_ptr++;
                Real tau_ij = *data_ptr++;
                Real alpha_ij = *data_ptr++;
                Real beta_ij = *data_ptr++;
                Real eta_pj = *prior_data_ptr++;
                Real tau_pj = *prior_data_ptr++;
                /* Real alpha_pj = * */ prior_data_ptr++;
                Real beta_pj = *prior_data_ptr++;

                sum_tau[j] += tau_ij;
                sum_alpha[j] += alpha_ij;
                sum_etatau[j] += eta_ij * tau_ij;
                sum_Ec2[j] += (beta_ij - beta_pj) * 2 -
                    (eta_pj * eta_pj * tau_pj -
                     eta_ij * eta_ij * tau_ij);
            }
        }
        Real * result_ptr = &palette->data()[0];
        Real * prior_data_ptr = &prior_palette->data()[0];
        for (int j = 0; j < COLOUR_SPACE_DIMS; ++j) {
            Real eta_pj = *prior_data_ptr++;
            Real tau_pj = *prior_data_ptr++;
            /*Real alpha_pj = * */prior_data_ptr++;
            Real beta_pj = *prior_data_ptr++;
            Real tau_j = sum_tau[j] / buffer_len;
            Real alpha_j = sum_alpha[j] / buffer_len;
            Real eta_j = sum_etatau[j] / (tau_j * buffer_len);
            Real beta_j = beta_pj + (sum_Ec2[j] / buffer_len +
                    eta_pj * eta_pj * tau_pj - eta_j * eta_j * tau_j) / 2;
            *result_ptr++ = eta_j;
            *result_ptr++ = tau_j;
            *result_ptr++ = alpha_j;
            *result_ptr++ = beta_j;
        }
    } // compute_current_from_adapted_palette_continuous

    void FaceColorModel::compute_current_from_adapted_palette_discrete(
        unsigned channel, OpenCvPlus::cvp_Palette * palette) {
        typedef boost::circular_buffer<cvp_PaletteBundle*> PaletteBuffer;
        typedef cvp_Palette::DataType Real;
        const PaletteBuffer::const_iterator first =
                m_AdaptationCache.m_PaletteBundles.begin();
        const PaletteBuffer::const_iterator last =
                m_AdaptationCache.m_PaletteBundles.end();
        const unsigned buffer_len = m_AdaptationCache.m_PaletteBundles.size();
        // set all intensity values to 0 initially
        fill(palette->data().begin(), palette->data().end(), 0.0f);
        for (PaletteBuffer::const_iterator i = first; i != last; ++i) {
            cvp_Palette * adapted_palette = (*i)->palette(channel);
            // add up intensities from every adaptation result
            transform(
                adapted_palette->data().begin(), adapted_palette->data().end(),
                palette->data().begin(), palette->data().begin(),
                boost::lambda::_1 + boost::lambda::_2);
        }
        // divide by number of adaptation results
        transform(
            palette->data().begin(), palette->data().end(),
            palette->data().begin(), boost::lambda::_1 / buffer_len);

    } // compute_current_from_adapted_palette_discrete

    void FaceColorModel::compute_color_marginal_log_likelihood(
        PimFeatureType * proba_maps, const CvRect& roi) {

        for(unsigned channel = 0; channel < FCM_NUM_CHANNELS; channel++) {
          cvp_Palette * palette = m_CurrentPaletteBundle->palette(channel);
          CvMat * proba_map = proba_maps->map(channel);

          if (palette->type() == cvp_Palette::FCM_CONTINUOUS) {
              // get precomputed means and normalization constant
              ContinuousColorPrecompute* pPrecompute =
                  (ContinuousColorPrecompute*)palette->precomputed_data();
              IplImage * image = m_ConvertedImagesCache.m_CachedYPbPrImage;
              compute_color_marginal_log_likelihood_continuous(
                  image, proba_map, pPrecompute, roi);
          } else if (palette->type() == cvp_Palette::FCM_DISCRETE) {
              DiscreteColorPrecompute* pPrecompute =
                  (DiscreteColorPrecompute*)palette->precomputed_data();
              IplImage * image =
                  m_ConvertedImagesCache.m_CachedHistogramBinImages[
                        pPrecompute->binsNumPerDimension];
              assert(image);
              compute_color_marginal_log_likelihood_discrete(
                  image, proba_map, palette, pPrecompute, roi);
          }
        }
    } // compute_color_marginal_log_likelihood

  void FaceColorModel::compute_color_marginal_log_likelihood_continuous(
      const IplImage * image, CvMat * proba_map,
      ContinuousColorPrecompute * pPrecompute, const CvRect& roi) {

      typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32F>::type float_32;

      // prepare fast logarithm computation, ROI values
      // FastLog * fast_log = FastLog::instance();

      const int start_row = roi.y;
      const int end_row = roi.y + roi.height;
      const int start_col= roi.x;
      const int end_col = roi.x + roi.width;

      const float log_constant = pPrecompute->logConstantMarginalLikelihood;
      const float mu0 = pPrecompute->mu[0];
      const float mu1 = pPrecompute->mu[1];
      const float mu2 = pPrecompute->mu[2];
      const float power0 = pPrecompute->studentPower[0];
      const float power1 = pPrecompute->studentPower[1];
      const float power2 = pPrecompute->studentPower[2];
      const float scale0 = pPrecompute->studentScale[0];
      const float scale1 = pPrecompute->studentScale[1];
      const float scale2 = pPrecompute->studentScale[2];

      // init pointers to image and map data
      const unsigned char * pInput =
              (const unsigned char *)image->imageData;
      unsigned char * pOutput = proba_map->data.ptr;

      // init pointers to the first pixel of ROI
      const unsigned char * pInputCurrentStart = pInput +
              start_row * image->widthStep + start_col * 3 *
              sizeof(float_32);
      unsigned char * pOutputCurrentStart = pOutput +
              start_row * proba_map->step + start_col *
              sizeof(PimFeatureType::value_type);

      float accumulator;
      float sqr_buf;

      // evaluate log likelihoods for the given ROI
      for(int row = start_row; row < end_row; row++) {
          const float_32 * pInputCurrent = (float_32 *)pInputCurrentStart;
          PimFeatureType::value_type * pOutputCurrent =
                  (PimFeatureType::value_type *) pOutputCurrentStart;
          for(int col = start_col; col < end_col; col++) {
              accumulator = 0;
              sqr_buf = *pInputCurrent++ - mu0;
              // accumulator += power0 * fast_log->log(1 + sqr_buf * sqr_buf / scale0 / ((-2) * power0 - 1));
              accumulator += power0 * std::log(1 + sqr_buf * sqr_buf / scale0 / ((-2) * power0 - 1));
              sqr_buf = *pInputCurrent++ - mu1;
              // accumulator += power1 * fast_log->log(1 + sqr_buf * sqr_buf / scale1 / ((-2) * power1 - 1));
              accumulator += power1 * std::log(1 + sqr_buf * sqr_buf / scale1 / ((-2) * power1 - 1));
              sqr_buf = *pInputCurrent++ - mu2;
              // accumulator += power2 * fast_log->log(1 + sqr_buf * sqr_buf / scale2 / ((-2) * power2 - 1));
              accumulator += power2 * std::log(1 + sqr_buf * sqr_buf / scale2 / ((-2) * power2 - 1));
              *pOutputCurrent++ = log_constant + accumulator;
          }
          // move to the next row in the input and output data
          pInputCurrentStart += image->widthStep;
          pOutputCurrentStart += proba_map->step;
      }
  } // compute_color_marginal_log_likelihood_continuous

  void FaceColorModel::compute_color_marginal_log_likelihood_discrete(
      const IplImage * image, CvMat * proba_map, cvp_Palette * palette,
      DiscreteColorPrecompute * pPrecompute, const CvRect& roi) {

      typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32S>::type int_32;

      // prepare fast logarithm computation, ROI values
      // FastLog * fast_log = FastLog::instance();

      const int start_row = roi.y;
      const int end_row = roi.y + roi.height;
      const int start_col= roi.x;
      const int end_col = roi.x + roi.width;

      const cvp_Palette::Data& data = palette->data();
      const float log_constant = pPrecompute->logConstantMarginalLikelihood;

      // init pointers to image and map data
      const unsigned char * pInput =
              (const unsigned char *)image->imageData;
      unsigned char * pOutput = proba_map->data.ptr;

      // init pointers to the first pixel of ROI
      const unsigned char * pInputCurrentStart = pInput +
              start_row * image->widthStep + start_col *
              sizeof(int_32);
      unsigned char * pOutputCurrentStart = pOutput +
              start_row * proba_map->step + start_col *
              sizeof(PimFeatureType::value_type);

      // evaluate log likelihoods for the given ROI
      for(int row = start_row; row < end_row; row++) {
          const int_32 * pInputCurrent = (int_32 *)pInputCurrentStart;
          PimFeatureType::value_type * pOutputCurrent =
                  (PimFeatureType::value_type *) pOutputCurrentStart;
          for(int col = start_col; col < end_col; col++) {
            // *pOutputCurrent++ = fast_log->log(data[*pInputCurrent++]) + log_constant;
            *pOutputCurrent++ = std::log(data[*pInputCurrent++]) + log_constant;
          }
          // move to the next row in the input and output data
          pInputCurrentStart += image->widthStep;
          pOutputCurrentStart += proba_map->step;
      }
  } // compute_color_marginal_log_likelihood_discrete

  double FaceColorModel::compute_color_marginal_log_likelihood_continuous(
          const CvScalar& yprpb, ContinuousColorPrecompute * pPrecompute) {

      // FastLog * fast_log = FastLog::instance();

      double result = 0;
      double sqr_buf;
      for (int i = 0; i < 3; ++i) {
          sqr_buf = yprpb.val[i] - pPrecompute->mu[i];
          result += pPrecompute->studentPower[i] *
              // fast_log->log(1 + sqr_buf * sqr_buf /
              std::log(1 + sqr_buf * sqr_buf /
                      pPrecompute->studentScale[i] /
                      ((-2) * pPrecompute->studentPower[i] - 1));
      }
      return result + pPrecompute->logConstantMarginalLikelihood;

  } // compute_color_marginal_log_likelihood_continuous

  double FaceColorModel::compute_color_marginal_log_likelihood_discrete(
          int histogram_index, cvp_Palette * palette,
          DiscreteColorPrecompute * pPrecompute) {

      const cvp_Palette::Data& data = palette->data();

      // FastLog * fast_log = FastLog::instance();
      // return fast_log->log(data[histogram_index]) +
      return std::log(data[histogram_index]) +
              pPrecompute->logConstantMarginalLikelihood;

  } // compute_color_marginal_log_likelihood_discrete

} // namespace FaceColorModel

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////
