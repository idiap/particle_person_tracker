/**
 * @file cxx/opencvplus/opencvplus/FaceColorModel.h
 * @date 01 June 2012
   @author Carl Scheffler (Carl.Scheffler@idiap.ch)
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Class for face color modelling and adaptation. Explicitly models
 *        and adapts color distributions for skin, hair, clothing and
 *        background around a face. Color models can be used to segmented a
 *        target image into face, hair, clothing and background regions.
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __FACECOLORMODEL_H__
#define __FACECOLORMODEL_H__

// SYSTEM INCLUDES
#include <cv.h>
#include <list>
#include <map>
#include <boost/circular_buffer.hpp>

// LOCAL INCLUDES
#include <opencvplus/cvp_PimModel.h>
#include <opencvplus/cvp_PaletteBundle.h>

namespace FaceColorModel {

    typedef double fcm_real;

    /// Indices for the different color models
    const unsigned FCM_NUM_CHANNELS       = 4;
    const unsigned FCM_CHANNEL_SKIN       = 0;
    const unsigned FCM_CHANNEL_HAIR       = 1;
    const unsigned FCM_CHANNEL_CLOTHES    = 2;
    const unsigned FCM_CHANNEL_BACKGROUND = 3;

    const int FCM_NUM_EVALUATION_SCHEMES = 10;

    struct FaceColorModelConfig {
        /// Path to the prior background colour model
        std::string m_BackgroundColourModel;
        /// Path to the prior clothes colour model
        std::string m_ClothesColourModel;
        /// Path to the prior hair colour model
        std::string m_HairColourModel;
        /// Path to the prior skin colour model
        std::string m_SkinColourModel;
        /// Path to the PIM model
        std::string m_PimModel;
    };

    class FaceColorModel {

    public:

        /// Type for PIM feature
        typedef OpenCvPlus::cvp_PimModel::PimFeatureType PimFeatureType;

        // LIFECYCLE

        /// Constructor. Allocates PIM model, colour models and adaptation
        /// buffers based on provided configuration.
        /// @param config Configuration file that contains paths to trained models.
        /// @param iKMrf The strength of the Markov random field between
        ///   adjacent pixels, used during color model adaptation.
        FaceColorModel(const FaceColorModelConfig& config, fcm_real iKMrf = 2.0f);

        /// Destructor. Deallocates PIM model, colour models and adaptation
        /// buffers.
        ~FaceColorModel();

        // OPERATIONS

        /// Return FCM_DISCRETE if the channel has a discrete color
        /// distribution and FCM_CONTINUOUS otherwise.
        /// @param iIndex The color channel for which to return the type.
        OpenCvPlus::cvp_Palette::Type get_channel_type(int iChannelIndex) const {
            return m_CurrentPaletteBundle->palette(iChannelIndex)->type();
        }

        /// Read access to configuration options for face colour model
        /// @return Configuration options for face colour model
        const FaceColorModelConfig& get_config() const {
            return mConfig;
        }

        /// Read access to prior palette bundle. Bundle contains palettes that
        /// were loaded using face colour model configuration options.
        /// In total there are FCM_NUM_CHANNELS palettes in the bundle.
        /// Palettes for different classes can be accessed using class indices
        /// (FCM_CHANNEL_SKIN, FCM_CHANNEL_HAIR, FCM_CHANNEL_CLOTHES,
        /// FCM_CHANNEL_BACKGROUND).
        /// @return Prior palette bundle
        const OpenCvPlus::cvp_PaletteBundle * prior_palette_bundle() const {
            return m_PriorPaletteBundle;
        }

        /// Read access to the current palette bundle.
        /// Bundle contains palettes that were loaded and probably updated
        /// using adaptation procedure.
        /// In total there are FCM_NUM_CHANNELS palettes in the bundle.
        /// Palettes for different classes can be accessed using class indices
        /// (FCM_CHANNEL_SKIN, FCM_CHANNEL_HAIR, FCM_CHANNEL_CLOTHES,
        /// FCM_CHANNEL_BACKGROUND).
        /// @return Current palette bundle
        const OpenCvPlus::cvp_PaletteBundle * current_palette_bundle() const {
            return m_CurrentPaletteBundle;
        }

        /// Read access to prior probability index maps (PIM).
        /// PIM consists of several layers, one layer per class.
        /// Layer elements contain probability (normalized or unnormalized)
        /// of a location to belong to a class.
        /// @param idx Index of the PIM model to return, defaults to 0
        /// @return Prior probability index maps
        const OpenCvPlus::cvp_PimModel * prior_pim(unsigned idx = 0) const {
            return m_PriorPimModels[idx];
        }

        /// Read access to current probability index maps (PIM).
        /// PIM consists of several layers, one layer per class.
        /// Layer elements contain probability (normalized or unnormalized)
        /// of a location to belong to a class.
        /// @param idx Index of the PIM model to return, defaults to 0
        /// @return Current probability index maps
        const OpenCvPlus::cvp_PimModel * current_pim(unsigned idx = 0) const {
            return m_CurrentPimModels[idx];
        }

        /// Number of PIM models (corresponding to different poses of the same
        /// object)
        /// @return Number of PIM models
        unsigned pim_models_count() const {
            return m_PriorPimModels.size();
        }

        /// Converts region of interest (ROI) associated with a detected face
        /// into a ROI associated with the face colour model.
        /// @param face_roi ROI associated with a detected face
        /// @param index Index of the PIM model to use for conversion,
        ///        defaults to 0
        /// @return ROI associated with face colour model
        CvRect face_roi2fcm_roi(const CvRect & face_roi,
            unsigned index = 0) const;

        /// Converts region of interest (ROI) associated with face colour model
        /// into a ROI associated with the detected face.
        /// @param fcm_roi ROI associated with face colour model
        /// @param index Index of the PIM model to use for conversion,
        ///        defaults to 0
        /// @return ROI associated with a detected face
        CvRect fcm_roi2face_roi(const CvRect & fcm_roi,
            unsigned index = 0) const;

        /// Prepare to work with images of certain size.
        /// One should call this method before using the model to generate
        /// probability maps. Allocates probability map caches,
        /// deallocates previous ones if necessary.
        /// The input image should have 3 channels of IPL_DEPTH_8U.
        /// @param ipImage Input image (3 channels, IPL_DEPTH_8U)
        void prepare(const IplImage * ipImage);

        /// Cache probabilities for every channel for all image pixels.
        /// One should call prepare method before to be sure caches are
        /// properly allocated
        /// @param ipImage Input image.
        void cache_probability_maps(const IplImage * ipImage);

        /// Cache probabilities for every channel for all the pixels within the
        /// indicated region of interest (ROI). For all pixels outside the ROI
        /// sets background probability to 1 and probabilities of other classes
        /// to 0.
        /// One should call prepare method before to be sure caches are
        /// properly allocated
        /// @param ipImage Input image.
        /// @param roi Region of interest for which to cache the data.
        void cache_probability_maps(const IplImage * ipImage, const CvRect& roi);

        /// Compute likelihood values for a given colour using current palettes
        /// @param bgr_colour BGR colour to compute likelihood values on
        /// @return Likelihood values for the four classes
        CvScalar colour_likelihoods(CvScalar bgr_colour);

        /// Read access to probability maps previously calculated on some
        /// image and cached.
        /// Probability maps consist of several layers, one layer per class.
        /// Layer elements contain probabilities (normalized or unnormalized)
        /// of a location to belong to a class.
        /// @return Cached probability maps
        const PimFeatureType * cached_probability_maps() const {
            return m_ProbabilityMapsCache.m_ProbabilityMaps;
        }

        /// Compute probability index maps (PIM) feature on the provided
        /// region of interest (ROI). Cached PIMs have the size of the input
        /// image, which is different from PIM feature size. This method
        /// provides "resampled" probability maps that correspond to the
        /// selected ROI and have the size of PIM feature. Output feature
        /// should have as many channels, as the prior model.
        /// Probability maps consist of several layers, one layer per class.
        /// Layer elements contain probabilities (normalized or unnormalized)
        /// of a location to belong to a class.
        /// @param roi Region of interest for which to calculate the feature.
        /// @param feature Input image.
        /// @return PIM feature for the provided ROI
        void compute_feature_on_cached_probability_maps(const CvRect & roi,
                PimFeatureType * feature);

        /// Adapt current PIM and colour models to the given data.
        /// This method assumes the provided region of interest (ROI) comes
        /// from a detector that was used to train colour models, or at least
        /// the cropping is similar. Then this method performs
        /// simultaneous adaptation of the current PIM model and
        /// colour palettes based on the provided image region.
        /// @param ipImage The source image to use for adaptation.
        ///   The image is assumed to contain a detection provided by the same
        ///   detector that was used to train PIM model and palettes.
        ///   It should have 3 channels (BGR) of depth 8U (1 byte).
        /// @param roi Region of interest of the image reported by a detector,
        ///   to which to adapt the model
        /// @param idx Index of the PIM model to adapt, defaults to 0
        /// @param rate_pim Adaptation rate (0 <= rate <= 1) for probability maps.
        ///   Rate 0 means that probability maps stay the same,
        ///   rate 1 forces prior maps to be rewritten with
        ///   new values regardless of old values.
        /// @param rate_colour Adaptation rate (0 <= rate <= 1) for palettes.
        ///   Rate 0 means that palettes stay the same,
        ///   rate 1 forces palettes to be rewritten with
        ///   new values regardless of old values.
        void adapt_to(IplImage * ipImage, const CvRect& roi, unsigned idx = 0,
                fcm_real rate_pim = 1.0, fcm_real rate_colour = 1.0);

        /// Undo any adaptation, reset current models to prior models.
        void reset_to_prior();

        /// Convert a BGR value with components in range [0, 255]
        /// to colour histogram index (an integer value).
        /// @param bgr_val Input BGR pixel
        /// @return Histogram index of a given colour
        static int bgr_to_histogram(const CvScalar& bgr_val,
                unsigned num_histogram_bins);

        /// Convert histogram index (an integer value) to a BGR value with
        /// components in range [0, 255]
        /// @param idx Histogram index of a colour
        /// @return BGR colour
        static CvScalar histogram_to_bgr(int idx, unsigned num_histogram_bins);

        /// Convert image to colour histogram indices. Every pixel colour
        /// (RGB - 3 values per pixel, 1 byte per channel) is mapped
        /// to bin index (1 value, 4 bytes). Only pixels within the specified
        /// region of interest (ROI) are considered. ROI should refer to a valid
        /// region within the image. Specifying ROI that lies outside the image
        /// would result in unpredictable behaviour.
        /// @param ipImage Input RGB image (3 channels, IPL_DEPTH_8U)
        /// @param num_histogram_bins Number of bins in a discrete palette
        /// @param opHistogram Output histogram image (1 channel, IPL_DEPTH_32S)
        /// @param roi Region of interest for which to perform the convertion
        static void image_to_histogram(const IplImage * ipImage,
                unsigned num_histogram_bins,
                IplImage* opHistogram, const CvRect& roi);

    private:

        typedef OpenCvPlus::cvp_PimFeature<
            OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_64F>::type>
            PimIntegralFeatureType;

        /// Cache for various probability maps
        struct ProbabilityMapCache {
            PimFeatureType * m_LogProbabilityMaps;
            PimFeatureType * m_ProbabilityMaps;
            PimIntegralFeatureType * m_IntegralProbabilityMaps;
            unsigned m_NumChannels;
            unsigned m_Width;
            unsigned m_Height;
        };

        /// Cache for converted images
        struct ConvertedImagesCache {
            // Cache for colour histogram images (each pixel is a bin number)
            std::map<unsigned, IplImage*> m_CachedHistogramBinImages;
            // Cache for YPbPr image
            IplImage* m_CachedYPbPrImage;
            unsigned m_Width;
            unsigned m_Height;
        };

        /// Cache for various adaptation data
        struct AdaptationCache {
            std::vector<boost::circular_buffer<PimFeatureType*> >
                m_PimFeaturesBuffers;
            boost::circular_buffer<OpenCvPlus::cvp_PaletteBundle*>
                m_PaletteBundles;
            IplImage * mImageAdaptationBuffer;
            IplImage * mImageAdaptationMask;
            CvMat * mImageAdaptationWarpMatrix;

            OpenCvPlus::cvp_PaletteBundle* m_AdaptedPaletteBundle;
            PimFeatureType* m_AdaptedPimFeature;
            PimFeatureType* m_AdaptedLogPimFeature;
            PimFeatureType* m_PriorLogPimFeature;

            // Cache for colour histogram images (each pixel is a bin number)
            std::map<unsigned, IplImage*> m_HistogramBinImages;
            // Cache for YPbPr image
            IplImage* m_YPbPrImage;
        };

        /// Allocates probability map buffers.
        /// @param num_channels Number of probability map layers
        /// @param width Probability map width
        /// @param height Probability map height
        void allocate_probability_maps_cache(unsigned num_channels,
                unsigned width, unsigned height);

        /// Dellocates probability map buffers.
        void deallocate_probability_maps_cache();

        /// Allocates caches for converted images.
        /// @param width Image width
        /// @param height Image height
        void allocate_converted_images_cache(unsigned width, unsigned height);

        /// Dellocates caches for converted images.
        void deallocate_converted_images_cache();

        /// Loads PIM model from a file and initializes all related arrays
        /// @param fname Model file name
        /// @throw cvp_Exception if file does not exist or has invalid format
        void initialize_pim_models(const std::string& fname);

        /// Deallocates all arrays created for PIM model
        void deinitialize_pim_models();

        /// Parses input string to check if one or several PIM models were
        /// specified. Fill PIM model file names into the provided list.
        /// @param fnames_str String containing PIM model file name or a list
        ///        of colon separated files (file1:file2:file3)
        /// @param fnames List of strings to fill individual file names in
        void parse_pim_files_string(const std::string& fnames_str,
            std::list<std::string>& fnames);

        /// Loads colour models from files and initializes all related arrays
        /// @throw cvp_Exception if any of the files does not exist or
        /// has invalid format
        void initialize_colour_models();

        /// Deallocates all arrays created for colour models
        void deinitialize_colour_models();

        /// Allocates various buffers used on adaptation stage
        void initialize_adaptation_buffers();

        /// Deallocates various buffers used on adaptation stage
        void deinitialize_adaptation_buffers();

        /// Extract adaptation buffer, the corresponding mask and
        /// YPbPr and histogram representations from the input image and ROI
        /// using warp transform
        void extract_adaptation_images(IplImage * image, const CvRect& roi);

        /// Update adaptation palettes based on colour data from
        /// adaptation image and current probability index map (PIM).
        void update_adapted_palettes();

        /// Update continuous adaptation palette based on colour data from
        /// YPbPr image and current probability index map (PIM).
        /// @param imageYPbPr YPbPr image (3 channels, IPL_DEPTH_32F)
        /// @param imageMask Mask image (1 channel, IPL_DEPTH_8U)
        /// @param pim_layer Probability index map for the given palette
        /// @param palette Palette data
        void update_adapted_palette_continuous(IplImage * imageYPbPr,
            IplImage * imageMask, CvMat * pim_layer,
            OpenCvPlus::cvp_Palette * palette);

        /// Update discrete adaptation palette based on colour data from
        /// histogram bin image and current probability index map (PIM).
        /// @param imageHist Histogram bin image (1 channel, IPL_DEPTH_32S)
        /// @param imageMask Mask image (1 channel, IPL_DEPTH_8U)
        /// @param pim_layer Probability index map for the given palette
        /// @param palette Palette data
        void update_adapted_palette_discrete(IplImage * imageHist,
            IplImage * imageMask, CvMat * pim_layer,
            OpenCvPlus::cvp_Palette * palette);

        /// Update adaptation pim based on colour data from
        /// adaptation image and current palettes
        /// @param idx Index of the PIM model to adapt
        void update_adapted_pim(unsigned idx);

        /// Adds colour data log likelihoods
        void update_adapted_pim_color_contribution();

        /// Computes current PIM and palettes based on adaptation results stored
        /// in circular buffers
        /// @param idx Index of the PIM model to compute current PIM
        void compute_current_from_adapted(unsigned idx);

        /// Compute current continuous palette parameters from
        /// cached adapted models
        /// @param channel Channel number
        /// @param palette Current palette to store parameters to
        void compute_current_from_adapted_palette_continuous(
            unsigned channel, OpenCvPlus::cvp_Palette * palette);

        /// Compute current discrete palette parameters from
        /// cached adapted models
        /// @param channel Channel number
        /// @param palette Current palette to store parameters to
        void compute_current_from_adapted_palette_discrete(
            unsigned channel, OpenCvPlus::cvp_Palette * palette);

        /// Computes log-likelihoods for every pixel of the original image
        /// and every channel (palette) using corresponding cached images
        /// @param proba_map Probability maps to write results to
        /// @param roi Region of interest to compute probabilities
        void compute_color_marginal_log_likelihood(PimFeatureType * proba_map,
                const CvRect& roi);

        /// Computes log-likelihoods of continuous colour distribution
        /// for pixels in the ROI of provided YPrPb image
        /// and stores them into the probability map.
        /// @param image Input YPrPb image (3 channels, IPL_DEPTH_32F)
        /// @param proba_map Probability map to write results to
        /// @param precomputed_data Continuous distribution data
        /// @param roi Region of interest in which to compute probabilities
        void compute_color_marginal_log_likelihood_continuous(
                const IplImage * image, CvMat * proba_map,
                OpenCvPlus::ContinuousColorPrecompute * precomputed_data,
                const CvRect& roi);

        /// Computes log-likelihoods of discrete colour distribution
        /// for pixels in the ROI of provided histogram bin image
        /// and stores them into the probability map.
        /// @param image Histogram bin image (3 channels, IPL_DEPTH_32S)
        /// @param proba_map Probability map to write results to
        /// @param palette Discrete palette data
        /// @param precomputed_data Discrete palette precomputed data
        /// @param roi Region of interest in which to compute probabilities
        void compute_color_marginal_log_likelihood_discrete(
                const IplImage * image, CvMat * proba_map,
                OpenCvPlus::cvp_Palette * palette,
                OpenCvPlus::DiscreteColorPrecompute * precomputed_data,
                const CvRect& roi);

        /// Computes log-likelihood of continuous colour distribution
        /// for an YPrPb value
        /// @param yprpb YPrPb colour value
        /// @param precomputed_data Precomputed palette data
        /// @return Log-likelihood of the colour
        double compute_color_marginal_log_likelihood_continuous(
                const CvScalar& yprpb,
                OpenCvPlus::ContinuousColorPrecompute * precomputed_data);

        /// Computes log-likelihood of discrete colour distribution
        /// for an histogram index value
        /// @param histogram_index Histogram index value
        /// @param palette Discrete palette data
        /// @param precomputed_data Discrete palette precomputed data
        /// @return Log-likelihood of the colour
        double compute_color_marginal_log_likelihood_discrete(
            int histogram_index, OpenCvPlus::cvp_Palette * palette,
            OpenCvPlus::DiscreteColorPrecompute * precomputed_data);

        // Attributes

        FaceColorModelConfig mConfig;

        /// Number of variational updates to perform when adapting color models.
        unsigned mVariationalIterations;
        /// The strength of the Markov random field between adjacent pixels.
        fcm_real mKMrf;

        /// Prior PIM models
        std::vector<OpenCvPlus::cvp_PimModel*> m_PriorPimModels;
        /// Current PIM modesl (could be prior / posterior / filtered etc.)
        std::vector<OpenCvPlus::cvp_PimModel*> m_CurrentPimModels;

        /// Prior palettes
        OpenCvPlus::cvp_PaletteBundle * m_PriorPaletteBundle;
        /// Current palettes (could be prior / posterior / filtered etc.)
        OpenCvPlus::cvp_PaletteBundle * m_CurrentPaletteBundle;

        // Probability maps cache
        bool m_ProbabilityMapsCacheAllocated;
        ProbabilityMapCache m_ProbabilityMapsCache;

        // Converted images cache
        bool m_ConvertedImagesCacheAllocated;
        ConvertedImagesCache m_ConvertedImagesCache;

        // Adaptation PIM and palette bundle caches
        AdaptationCache m_AdaptationCache;

        // flag indicating whether there are palettes that use
        // YPbPr converted image
        bool m_UsesYPbPrImage;

  }; // class FaceColorModel

} // namespace FaceColorModel

#endif
