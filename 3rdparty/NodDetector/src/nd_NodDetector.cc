/**
 * @file src/nd_NodDetector.cc
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

// SYSTEM INCLUDES
#include <iostream>                            // STL I/O
#include <fstream>                             // STL file I/O
#include <sstream>                             // STL string I/O
#include <numeric>                             // STL inner_product
#include <stdexcept>                           // STL runtime_error
#include <cmath>                               // for exp
#include <boost/foreach.hpp>                   // boost foreach loop
#include <boost/lambda/lambda.hpp>             // boost lambda exprs
#include <boost/static_assert.hpp>             // boost type sizeof assertions

// LOCAL INCLUDES
#include <noddetector/nd_NodDetector.h>        // declaration of this

using namespace std;

namespace NodDetector {

/////////////////////////////// CONSTANTS ////////////////////////////////////

static const double DEFAULT_SVM_THRESHOLD = 0.0;
static const double GAUSSIAN_WINDOW_SCALE = 2.5;
#define NUM_BYTES_INT 4
#define NUM_BYTES_DOUBLE 8

// define 32bit integer type depending on target architecture
typedef int int_32;
// define 64bit float type depending on target architecture
typedef double double_64;

// check the size of int, 32-bit is used to read binary data from file
BOOST_STATIC_ASSERT(sizeof(int_32) == NUM_BYTES_INT);
// check the size of double, 64-bit is used to read binary data from file
BOOST_STATIC_ASSERT(sizeof(double_64) == NUM_BYTES_DOUBLE);

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

// reads a 32-bit integer from the provided binary stream
static int_32 read_int32(istream& in, char * buffer, const string& descr,
        bool unsigned_flag);
// reads a 64-bit floating point number from the provided binary stream
static double_64 read_double64(istream& in, char * buffer, const string& descr);
// reads a point pattern from the provided binary stream
static void read_point_coordinates(istream& in, vector<nd_Point>& points);
// reads point pattern motion data from the provided binary stream
static void read_point_pattern_motion_data(istream& in,
        nd_PointPatternMotionData& data);
// reads point motion data from the provided binary stream
static void read_point_motion_data(istream& in, nd_PointMotionData& data);

/////////////////////////////// PUBLIC ///////////////////////////////////////

nd_NodDetector::~nd_NodDetector() {
    // deallocate FFT plan
    fftw_destroy_plan(m_FftPlan);
    // deallocate FFT data
    BOOST_FOREACH(fftw_complex * fft_data, m_FftData) {
        fftw_free(fft_data);
    }
} // ~nd_NodDetector

/* static */ nd_NodDetector *
nd_NodDetector::load(const std::string& data_file_path) {
    ifstream data_file_istream;
    data_file_istream.open(data_file_path.c_str(), ios::in | ios::binary);
    if (!data_file_istream.is_open()) {
        throw runtime_error("Nod detector model file " + data_file_path +
                " could not be opened!");
    }
    char buffer[NUM_BYTES_DOUBLE];

    try {
        // read the number of points in a point pattern (=N)
        int_32 num_points = read_int32(data_file_istream, buffer,
                "number of points", true);
        if (num_points <= 0) {
            ostringstream oss;
            oss << "Expected positive number of points in a point pattern "
                << "in nod detector model file " << data_file_path
                << ", " << num_points << " encountered";
            throw runtime_error(oss.str());
        }
        // read point coordinates in a point pattern (2*N)
        vector<nd_Point> points(num_points);
        read_point_coordinates(data_file_istream, points);
        nd_PointPattern point_pattern(points);
        // read the length of motion measurements buffer (=M)
        int_32 mot_buffer_len = read_int32(data_file_istream, buffer,
                "motion buffer length", true);
        if (mot_buffer_len <= 0) {
            ostringstream oss;
            oss << "Expected positive motion buffer length "
                << "in nod detector model file " << data_file_path
                << ", " << mot_buffer_len << " encountered";
            throw runtime_error(oss.str());
        }
        // read feature normalisation means (2*N*M)
        nd_PointPatternMotionData feature_norm_means(num_points, mot_buffer_len);
        read_point_pattern_motion_data(data_file_istream, feature_norm_means);
        // read feature normalisation scales (2*N*M)
        nd_PointPatternMotionData feature_norm_scales(num_points, mot_buffer_len);
        read_point_pattern_motion_data(data_file_istream, feature_norm_scales);
        // read support vector machine (SVM) weights (2*N*M)
        nd_PointPatternMotionData svm_weights(num_points, mot_buffer_len);
        read_point_pattern_motion_data(data_file_istream, svm_weights);
        // read support vector machine (SVM) bias
        double_64 svm_bias = read_double64(data_file_istream, buffer,
                "SVM bias");
        // read expected frequency of motion measurements
        double_64 frequency_Hz = read_double64(data_file_istream, buffer,
                "frequency");
        if (frequency_Hz <= 0) {
            ostringstream oss;
            oss << "Expected positive motion measurements frequency parameter "
                << "in nod detector model file " << data_file_path
                << ", " << frequency_Hz << " encountered";
            throw runtime_error(oss.str());
        }

        data_file_istream.close();

        return new nd_NodDetector(point_pattern, feature_norm_means,
            feature_norm_scales, svm_weights, svm_bias, frequency_Hz);
    } catch (...) {
        data_file_istream.close();
        throw;
    }
} // load

nd_NodDetectorCache *
nd_NodDetector::create_cache() const {
    return new nd_NodDetectorCache(m_PointPattern.m_Points.size(),
        m_MotionDataSize);
} // create_cache

// #define DETECT_NOD_TRACE

bool
nd_NodDetector::detect_nod(
const nd_PointPatternMotionData& pattern_motion_data,
nd_NodDetectorCache& cache) const {

    // check whether data is valid
    assert_pattern_motion_data_valid(pattern_motion_data, m_MotionDataSize);

#ifdef DETECT_NOD_TRACE
//    ofstream ofs("detect_nod_dump.txt");
    ostream& ofs = cout;
    ofs << "Input data:" << endl;
    ofs << pattern_motion_data;
#endif

    const unsigned point_pattern_size = m_PointPattern.m_Points.size();

    // apply Gaussian window to all motion data
    apply_gaussian_window_to_motion_data(pattern_motion_data,
            cache.m_PatternMotionDataWindowed);

#ifdef DETECT_NOD_TRACE
    ofs << "Windowed data:" << endl;
    ofs << cache.m_PatternMotionDataWindowed;
#endif

    // apply FFT to motion data of all points
    fftw_complex *const * fft_data = &m_FftData[0];
    nd_PointMotionData * pt_motion_data =
            &cache.m_PatternMotionDataWindowed.m_PointMotionDatas[0];
    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        // apply FFT to X motion data of a point
        fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_XMotionData[0],
                *fft_data++);
        // apply FFT to Y motion data of a point
        fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_YMotionData[0],
                *fft_data++);
        pt_motion_data++;
    }

    // compute motion features on FFT data
    compute_motion_features(m_FftData, cache.m_PatternMotionDataFeatures);

#ifdef DETECT_NOD_TRACE
    ofs << "Features:" << endl;
    ofs << cache.m_PatternMotionDataFeatures;
    ofs << "SVM Weights:" << endl;
    ofs << m_SvmWeights << endl;
    ofs << "SVM bias:" << endl;
    ofs << m_SvmBias << endl;
#endif

    // apply SVM classifier to motion features
    double result = svm_classify(cache.m_PatternMotionDataFeatures,
        m_SvmWeights, m_SvmBias);

#ifdef DETECT_NOD_TRACE
    ofs << "Result:" << endl;
    ofs << result << endl;
    ofs << "Result (" << result << ") "
        << ((result > m_SvmThreshold) ? ">" : "<")
        << " Threshold (" << m_SvmThreshold << "), "
        << ((result > m_SvmThreshold) ? "nod detected" : "nod not detected")
        << endl;
#endif

    return (result > m_SvmThreshold);

} // detect_nod

/////////////////////////////// PRIVATE //////////////////////////////////////

nd_NodDetector::nd_NodDetector(const nd_PointPattern& point_pattern,
    const nd_PointPatternMotionData& feature_norm_bias,
    const nd_PointPatternMotionData& feature_norm_scale,
    const nd_PointPatternMotionData& svm_weights,
    double svm_bias, double motion_data_frequency) :
    m_PointPattern(point_pattern), m_MotionDataFrequency(motion_data_frequency),
    m_FeatureSize(svm_weights.m_PointMotionDatas[0].m_XMotionData.size()),
    m_MotionDataSize(2 * m_FeatureSize - 1),
    m_FftData(m_PointPattern.m_Points.size() * 2),
    m_FeatureNormBias(feature_norm_bias),
    m_FeatureNormScale(feature_norm_scale),
    m_SvmWeights(svm_weights),
    m_SvmBias(svm_bias),
    m_SvmThreshold(DEFAULT_SVM_THRESHOLD),
    m_GaussianWindow(m_MotionDataSize) {

    assert_constructor_data_valid();

    // allocate FFT data
    BOOST_FOREACH(fftw_complex *& fft_data, m_FftData) {
        fft_data = (fftw_complex*) fftw_malloc(
                sizeof(fftw_complex) * m_MotionDataSize);
    }
    // allocate and initialize FFT plan
    m_FftPlan = fftw_plan_dft_r2c_1d(m_MotionDataSize, 0, 0, FFTW_ESTIMATE);
    // initialize Gaussian window

    const double gaussian_window_scale =
            static_cast<double>(4.0 * m_MotionDataSize * m_MotionDataSize) /
            ((m_MotionDataSize - 1) * (m_MotionDataSize - 1));
    initialize_gaussian_window(m_GaussianWindow, gaussian_window_scale);

} // nd_NodDetector

void
nd_NodDetector::initialize_gaussian_window(
        std::vector<double>& window, double alpha) const {

    const unsigned wsize = window.size();
    assert(wsize > 1);

    double * wptr = &window[0];
    const double start = -static_cast<double>(wsize) / 2;
    const double step = static_cast<double>(wsize) / (wsize - 1);
    double pt = start;
    double val;

    for (unsigned idx = 0; idx < wsize; ++idx) {
        val = alpha * pt / start;
        *wptr++ = exp(-0.5 * val * val);
        pt += step;
    }
} // initialize_gaussian_window

void
nd_NodDetector::apply_gaussian_window_to_motion_data(
        const nd_PointPatternMotionData& pattern_motion_data,
        nd_PointPatternMotionData& pattern_motion_data_windowed) const {

    const unsigned point_pattern_size = m_PointPattern.m_Points.size();

    const nd_PointMotionData * pt_motion_data_in =
            &pattern_motion_data.m_PointMotionDatas[0];
    nd_PointMotionData * pt_motion_data_out =
            &pattern_motion_data_windowed.m_PointMotionDatas[0];

    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        // apply Gaussian window to X motion data of a point
        transform(pt_motion_data_in->m_XMotionData.begin(),
                  pt_motion_data_in->m_XMotionData.end(),
                  m_GaussianWindow.begin(),
                  pt_motion_data_out->m_XMotionData.begin(),
                  boost::lambda::_1 * boost::lambda::_2);
        // apply Gaussian window to Y motion data of a point
        transform(pt_motion_data_in->m_YMotionData.begin(),
                  pt_motion_data_in->m_YMotionData.end(),
                  m_GaussianWindow.begin(),
                  pt_motion_data_out->m_YMotionData.begin(),
                  boost::lambda::_1 * boost::lambda::_2);
        pt_motion_data_in++;
        pt_motion_data_out++;
    }

} // apply_gaussian_window_to_motion_data

void
nd_NodDetector::compute_motion_features(
        const std::vector<fftw_complex*>& fft_data,
        nd_PointPatternMotionData& features) const {

    const unsigned point_pattern_size = m_PointPattern.m_Points.size();
    const fftw_complex *const * fft_data_in = &fft_data[0];
    const nd_PointMotionData * norm_mean_in =
            &m_FeatureNormBias.m_PointMotionDatas[0];
    const nd_PointMotionData * norm_scale_in =
            &m_FeatureNormScale.m_PointMotionDatas[0];
    nd_PointMotionData * features_data_out = &features.m_PointMotionDatas[0];

    const fftw_complex * cur_fft_buffer;
    const double * cur_norm_bias_buffer;
    const double * cur_norm_scale_buffer;
    double * cur_feature_buffer;

    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        // feature for X motion data
        cur_fft_buffer = *fft_data_in++;
        cur_feature_buffer = &features_data_out->m_XMotionData[0];
        cur_norm_bias_buffer = &norm_mean_in->m_XMotionData[0];
        cur_norm_scale_buffer = &norm_scale_in->m_XMotionData[0];
        compute_motion_feature(cur_fft_buffer, cur_norm_bias_buffer,
                cur_norm_scale_buffer, cur_feature_buffer, m_FeatureSize);
        // feature for Y motion data
        cur_fft_buffer = *fft_data_in++;
        cur_feature_buffer = &features_data_out->m_YMotionData[0];
        cur_norm_bias_buffer = &norm_mean_in->m_YMotionData[0];
        cur_norm_scale_buffer = &norm_scale_in->m_YMotionData[0];
        compute_motion_feature(cur_fft_buffer, cur_norm_bias_buffer,
                cur_norm_scale_buffer, cur_feature_buffer, m_FeatureSize);
        norm_mean_in++;
        norm_scale_in++;
        features_data_out++;
    }

} // compute_motion_features

void nd_NodDetector::compute_motion_feature(const fftw_complex * fft_data,
        const double * mean_data, const double * scale_data,
        double * feature_data, unsigned size) const {
    double val_re, val_im, val_abs;
    for (unsigned j = 0; j < size; ++j) {
        val_re = (*fft_data)[0];
        val_im = (*fft_data)[1];
        val_abs = 2 * sqrt(val_re * val_re + val_im * val_im);
        *feature_data++ = (val_abs - *mean_data++) / (*scale_data++);
        fft_data++;
    }
} // compute_motion_feature

double nd_NodDetector::svm_classify(const nd_PointPatternMotionData& features,
        const nd_PointPatternMotionData& svm_weights,
        double svm_bias) const {

    const unsigned point_pattern_size = m_PointPattern.m_Points.size();
    const nd_PointMotionData * features_in = &features.m_PointMotionDatas[0];
    const nd_PointMotionData * weights_in = &svm_weights.m_PointMotionDatas[0];

    // compute inner products of motion features with SVM weight vectors
    double result = 0;
    double inner_product_result = 0;
    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        inner_product_result = inner_product(
                weights_in->m_XMotionData.begin(),
                weights_in->m_XMotionData.end(),
                features_in->m_XMotionData.begin(),
                0.0);
        result += inner_product_result;
        inner_product_result = inner_product(
                weights_in->m_YMotionData.begin(),
                weights_in->m_YMotionData.end(),
                features_in->m_YMotionData.begin(),
                0.0);
        result += inner_product_result;
        features_in++;
        weights_in++;
    }
    result += svm_bias;
    return result;
} // svm_classify

void
nd_NodDetector::assert_pattern_motion_data_valid(
        const nd_PointPatternMotionData& motion_data,
        unsigned size) const {
    assert(motion_data.m_PointMotionDatas.size() ==
            m_PointPattern.m_Points.size());
    BOOST_FOREACH(const nd_PointMotionData& pt_motion,
            motion_data.m_PointMotionDatas) {
        assert(pt_motion.m_XMotionData.size() == size);
        assert(pt_motion.m_YMotionData.size() == size);
    }
} // assert_pattern_motion_data_valid

void
nd_NodDetector::assert_constructor_data_valid() const {

    assert_pattern_motion_data_valid(m_SvmWeights, m_FeatureSize);
    assert_pattern_motion_data_valid(m_FeatureNormBias, m_FeatureSize);
    assert_pattern_motion_data_valid(m_FeatureNormScale, m_FeatureSize);

} // assert_constructor_data_valid

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

int_32 read_int32(istream& in, char * buffer, const string& descr,
        bool unsigned_flag) {
    const unsigned num_chars_to_read = NUM_BYTES_INT;
    in.read(buffer, num_chars_to_read);
    const unsigned num_chars_read = in.gcount();
    if (num_chars_read != num_chars_to_read) {
        ostringstream oss;
        oss << "Error reading integer value (" << descr
            << ") from nod detector model file, expected "
            << num_chars_to_read << ", actually read " << num_chars_read;
        throw runtime_error(oss.str());
    }
    int_32 result = *(int_32*)buffer;
    if (unsigned_flag && (result < 0)) {
        ostringstream oss;
        oss << "Error reading unsigned integer value (" << descr
            << ") from nod detector model file, negative value encountered: "
            << result;
        throw runtime_error(oss.str());
    }
    return result;
} // read_int32

double_64 read_double64(istream& in, char * buffer, const string& descr) {
    const unsigned num_chars_to_read = NUM_BYTES_DOUBLE;
    in.read(buffer, num_chars_to_read);
    const unsigned num_chars_read = in.gcount();
    if (num_chars_read != num_chars_to_read) {
        ostringstream oss;
        oss << "Error reading double value (" << descr
            << ") from nod detector model file, expected "
            << num_chars_to_read << ", actually read " << num_chars_read;
        throw runtime_error(oss.str());
    }
    double_64 result = *(double_64*)buffer;
    return result;
} // read_double64

void read_point_coordinates(istream& in, vector<nd_Point>& points) {
    char buffer[NUM_BYTES_DOUBLE];
    const unsigned points_num = points.size();
    nd_Point * pt_ptr = &points[0];
    for (unsigned idx = 0; idx < points_num; ++idx) {
        pt_ptr->m_X = read_double64(in, buffer, "point X");
        pt_ptr->m_Y = read_double64(in, buffer, "point Y");
        pt_ptr++;
    }
} // read_point_coordinates

void read_point_pattern_motion_data(istream& in,
        nd_PointPatternMotionData& data) {
    const unsigned points_num = data.m_PointMotionDatas.size();
    nd_PointMotionData * ptmot_ptr = &data.m_PointMotionDatas[0];
    for (unsigned idx = 0; idx < points_num; ++idx) {
        read_point_motion_data(in, *ptmot_ptr++);
    }
} // read_point_pattern_motion_data

void read_point_motion_data(istream& in, nd_PointMotionData& data) {
    char buffer[NUM_BYTES_DOUBLE];
    const unsigned mot_data_len = data.m_XMotionData.size();
    nd_PointMotionData::RealType * mot_data_ptr;
    // read X motion
    mot_data_ptr = &data.m_XMotionData[0];
    for (unsigned idx = 0; idx < mot_data_len; ++idx) {
        *mot_data_ptr++ = read_double64(in, buffer, "motion X");
    }
    // read Y motion
    mot_data_ptr = &data.m_YMotionData[0];
    for (unsigned idx = 0; idx < mot_data_len; ++idx) {
        *mot_data_ptr++ = read_double64(in, buffer, "motion Y");
    }
} // read_point_motion_data

} // namespace NodDetector
