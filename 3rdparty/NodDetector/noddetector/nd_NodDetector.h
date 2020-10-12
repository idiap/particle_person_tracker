/**
 * @file noddetector/nd_NodDetector.h
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Detector of head nod gestures based on point pattern motion
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __ND_NODDETECTOR_H__
#define __ND_NODDETECTOR_H__

// SYSTEM INCLUDES
#include <fftw3.h>                                     // FFT

// LOCAL INCLUDES
#include "nd_PointPattern.h"                           // point pattern
#include "nd_PointPatternMotionData.h"                 // motion data

namespace NodDetector {

struct nd_NodDetectorCache;

/// @brief Detector of head nod gestures based on point pattern motion.
///
/// Detects head nods using motion data of a predefined point pattern.
/// Point motion is assumed to be sampled regularly at certain frequency.
/// X and Y coordinate motion data is converted into Fourier coefficients
/// and given to a support vector machine (SVM) to classify motion as a nod or
/// a non-nod.
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Laurent Nguyen <lnguyen@idiap.ch>
/// @version 1.0
/// @date   04.03.2013

class nd_NodDetector {

    public:

    // LIFECYCLE

    /// Deallocates all caches (FFT, weights etc.)
    ~nd_NodDetector();

    // OPERATIONS

    /// Loads nod detector data from a file and initializes all cache
    /// structures.
    /// @param data_file_path Path to detector data file
    /// @return Nod detector instance
    static nd_NodDetector * load(const std::string& data_file_path);

    /// Creates detector computation cache
    /// @return Nod detector computation cache
    nd_NodDetectorCache * create_cache() const;

    /// Detects head nod based on point pattern motion data
    /// @param motion_data Point pattern motion data
    /// @param cache Cache to keep temporary values
    /// @return True, if nod was detected
    bool detect_nod(const nd_PointPatternMotionData& motion_data,
        nd_NodDetectorCache& cache) const;

    /// Point pattern used by this nod detector
    /// @return Point pattern used by this nod detector
    const nd_PointPattern& point_pattern() const {
        return m_PointPattern;
    } // point_pattern

    /// Expected frequency of motion data measurements to classify
    double motion_data_frequency() const {
        return m_MotionDataFrequency;
    } // motion_data_frequency

    /// Expected size of motion data measurements
    size_t motion_data_size() const {
        return m_MotionDataSize;
    } // motion_data_frequency

    /// Model feature normalisation mean values
    const nd_PointPatternMotionData& feature_normalisation_means() const {
        return m_FeatureNormBias;
    } // feature_normalisation_means

    /// Model feature normalization scale values
    const nd_PointPatternMotionData& feature_normalisation_scales() const {
        return m_FeatureNormScale;
    } // feature_normalisation_means

    /// Linear SVM weights
    const nd_PointPatternMotionData& svm_weights() const {
        return m_SvmWeights;
    } // svm_weights

    /// Linear SVM bias
    double svm_bias() const {
        return m_SvmBias;
    } // svm_bias

    /// Get threshold used by linear support vector machine (SVM) for
    /// classification
    /// @return SVM classification threshold
    double threshold() const {
        return m_SvmThreshold;
    }

    /// Set threshold used by linear support vector machine (SVM) for
    /// classification
    /// @param New SVM classification threshold
    void threshold(double value) {
        m_SvmThreshold = value;
    }

    private:

    // LIFECYCLE

    /// Constructs nod detector based on the specified point pattern and
    /// working at certain data sampling frequency and with certain data length.
    /// @param point_pattern Point pattern to be used to detect nods
    /// @param feature_norm_bias Bias used for feature normalization
    /// @param feature_norm_scale Scale used for feature normalization
    /// @param svm_weights Weights used by linear support vector machine (SVM)
    ///        for classification
    /// @param svm_bias Bias used by linear support vector machine (SVM)
    ///        for classification
    /// @param motion_data_frequency Frequency at which motion data is expected
    ///        to be sampled
    nd_NodDetector(const nd_PointPattern& point_pattern,
        const nd_PointPatternMotionData& feature_norm_bias,
        const nd_PointPatternMotionData& feature_norm_scale,
        const nd_PointPatternMotionData& svm_weights,
        double svm_bias, double motion_data_frequency);

    // OPERATIONS

    /// Initializes Gaussian window function for certain window size.
    /// @param window Vector to save window data into
    /// @param alpha Scale parameter for Gaussian window
    void initialize_gaussian_window(std::vector<double>& window,
        double alpha) const;

    /// Applies Gaussian window function to the pattern motion data.
    /// @param pattern_motion_data Data to apply Gaussian window to
    /// @param pattern_motion_data_windowed Data to write results to
    void apply_gaussian_window_to_motion_data(
            const nd_PointPatternMotionData& pattern_motion_data,
            nd_PointPatternMotionData& pattern_motion_data_windowed) const;

    /// Computes motion features on FFT data and stores them in the provided
    /// structure.
    /// @param fft_data Fourier coefficients as arrays of complex numbers
    /// @param features Feature vectors to fill
    void compute_motion_features(const std::vector<fftw_complex*>& fft_data,
            nd_PointPatternMotionData& features) const;

    /// Computes motion feature on FFT data and stores it in the provided
    /// array.
    /// @param fft_data Input Fourier coefficients as an array of complex numbers
    /// @param mean_data Input array of feature normalisation bias values
    /// @param scale_data Input array of normalisation scale values
    /// @param feature_data Output array of feature values
    /// @param size Size of both arrays
    void compute_motion_feature(const fftw_complex * fft_data,
            const double * mean_data, const double * scale_data,
            double * feature_data, unsigned size) const;

    /// Produces classification of observed point motion into nod or non-nod
    /// using provided support vector machine (SVM) parameters
    double svm_classify(const nd_PointPatternMotionData& features,
            const nd_PointPatternMotionData& svm_weights,
            double svm_bias) const;

    /// Assert whether pattern motion data provided for nod detection
    /// has valid contents. Available in debug mode only, triggers SIG_ABORT
    /// on failure.
    /// @param motion_data Point pattern motion data
    /// @param size Expected motion data size
    void assert_pattern_motion_data_valid(
            const nd_PointPatternMotionData& motion_data,
            unsigned size) const;

    /// Assert that point pattern and support vector machine (SVM) parameters
    /// have consistent structure
    void assert_constructor_data_valid() const;

    // FIELDS

    // point pattern used by this nod detector
    nd_PointPattern m_PointPattern;
    // frequency of motion data measurements
    double m_MotionDataFrequency;
    // size of motion data measurements
    size_t m_FeatureSize;
    // size of motion data measurements
    size_t m_MotionDataSize;
    // FFT plan, contains precomputed data for FFT
    fftw_plan m_FftPlan;
    // FFT results storage for all point motion datas
    std::vector<fftw_complex*> m_FftData;
    // Feature normalization bias values
    nd_PointPatternMotionData m_FeatureNormBias;
    // Feature normalization scale values
    nd_PointPatternMotionData m_FeatureNormScale;
    // Linear SVM weights
    nd_PointPatternMotionData m_SvmWeights;
    // Linear SVM bias
    double m_SvmBias;
    // Linear SVM threshold
    double m_SvmThreshold;
    // Gaussian window function samples
    std::vector<double> m_GaussianWindow;

}; // class nd_NodDetector

class nd_NodDetectorCache {

    private:

    /// Cached pattern motion data after Gaussian window application
    nd_PointPatternMotionData m_PatternMotionDataWindowed;
    /// Cached pattern motion features computed from FFT
    nd_PointPatternMotionData m_PatternMotionDataFeatures;

    // hidden default constructor
    nd_NodDetectorCache();
    // hidden copy constructor
    nd_NodDetectorCache(const nd_NodDetectorCache& other);
    // hidden constructor
    nd_NodDetectorCache(size_t pattern_size, size_t motion_data_size) :
        m_PatternMotionDataWindowed(pattern_size, motion_data_size),
        m_PatternMotionDataFeatures(pattern_size, motion_data_size) {
    }

    friend class nd_NodDetector;
};

} // namespace NodDetector

#endif // __ND_NODDETECTOR_H__
