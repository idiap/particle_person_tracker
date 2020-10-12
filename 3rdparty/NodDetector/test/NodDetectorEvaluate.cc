/**
 * @file test/NodDetectorEvaluate.cc
 * @date 13 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Test nod detector computations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE NodDetectorEvaluateTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <iostream>                            // STL I/O
#include <fstream>                             // STL file I/O
#include <stdexcept>                           // STL runtime_error
#include <cmath>                               // STL exp
#include <boost/test/included/unit_test.hpp>   // boost unit testing
#include <boost/foreach.hpp>                   // boost foreach loop
#include <boost/lambda/lambda.hpp>             // boost lambda exprs

// PROJECT INCLUDES
#include <noddetector/nd_NodDetector.h>        // nod detector

using namespace std;
using namespace NodDetector;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace NodDetector {
    struct nd_Logger {
        nd_Logger()   {
            const char* report_file_path = getenv("ND_TESTREPORT_FILE");
              if (!report_file_path) {
                  std::cerr <<
                  "Environment variable $ND_TESTREPORT_FILE is not set."
                  " Have you setup your working environment correctly?" <<
                  std::endl;
                  throw runtime_error("");
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~nd_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(nd_Logger);

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

// variable name that contains a path to test data directory
static const string DATA_DIR_ENVIRONMENT_VAR = "ND_TESTDATA_DIR";

static void initialize_gaussian_window(vector<double>& window, double alpha);
static void apply_gaussian_window_to_motion_data(
    const vector<double>& gaussian_window, const nd_PointPattern& point_pattern,
    const nd_PointPatternMotionData& pattern_motion_data,
    nd_PointPatternMotionData& pattern_motion_data_windowed);
static void compute_motion_features_uncorrected(
        const nd_PointPattern& point_pattern,
        const std::vector<fftw_complex*>& fft_data,
        nd_PointPatternMotionData& features);
static void compute_motion_feature_uncorrected(const fftw_complex * fft_data,
        double * feature_data, unsigned size);
static void compute_motion_features(
        const nd_PointPattern& point_pattern,
        const nd_PointPatternMotionData& norm_biases,
        const nd_PointPatternMotionData& norm_scales,
        const std::vector<fftw_complex*>& fft_data,
        nd_PointPatternMotionData& features);
static void compute_motion_feature(const fftw_complex * fft_data,
        const double * mean_data, const double * scale_data,
        double * feature_data, unsigned size);

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_noddetector_evaluate )

BOOST_AUTO_TEST_CASE( evaluate_gaussian_window ) {
    static const string FILENAME_STR = "noddetector1.data";
    static const unsigned GWIN_5_LEN = 21;
    static const double GWIN_5[] = {
        0.000060, 0.000380, 0.001983, 0.008525, 0.030178, 0.087948, 0.211010,
        0.416793, 0.677760, 0.907338, 1.000000, 0.907338, 0.677760, 0.416793,
        0.211010, 0.087948, 0.030178, 0.008525, 0.001983, 0.000380, 0.000060
    };
    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");
    data_path.append(FILENAME_STR);

    nd_NodDetector * nod_detector = nd_NodDetector::load(data_path);
    unsigned win_len = nod_detector->motion_data_size();
    double alpha = static_cast<double>(4.0 * win_len * win_len) /
        ((win_len - 1) * (win_len - 1));

    vector<double> gwin(win_len);
    initialize_gaussian_window(gwin, alpha);
    for (unsigned idx = 0; idx < GWIN_5_LEN; ++idx) {
        if (GWIN_5[idx] < 1e-2) {
            BOOST_CHECK_CLOSE(gwin[idx], GWIN_5[idx], 1);
        } else {
            BOOST_CHECK_CLOSE(gwin[idx], GWIN_5[idx], 1e-2);
        }
    }
}

BOOST_AUTO_TEST_CASE( apply_gaussian_window ) {
    static const string FILENAME_STR = "noddetector1.data";
    static const double SWIN[] = {
        0.000060, 0.000759, 0.005948, 0.034099, 0.150888, 0.527686, 1.477073,
        3.334348, 6.099840, 9.073378, 11.000000, 10.888053, 8.810880, 5.835108,
        3.165156, 1.407164, 0.513018, 0.153445, 0.037668, 0.007591, 0.001256
    };
     // get data directory
     char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
     if (!data_dir) {
         BOOST_FAIL("Data directory environment variable is not set up "
                 "correctly - verify project configuration!");
     }
     string data_path;
     data_path.append(data_dir);
     data_path.append("/");
     data_path.append(FILENAME_STR);

     nd_NodDetector * nod_detector = nd_NodDetector::load(data_path);
     const unsigned win_len = nod_detector->motion_data_size();
     double alpha = static_cast<double>(4.0 * win_len * win_len) /
         ((win_len - 1) * (win_len - 1));

     vector<double> gwin(win_len);
     initialize_gaussian_window(gwin, alpha);

     nd_PointPatternMotionData motion_data(
             nod_detector->point_pattern().m_Points.size(),
             win_len);
     BOOST_FOREACH(nd_PointMotionData& data, motion_data.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             data.m_XMotionData[idx] = idx + 1;
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             data.m_YMotionData[idx] = idx + 1;
         }
     }

     nd_PointPatternMotionData motion_data_windowed(motion_data);
     apply_gaussian_window_to_motion_data(gwin, nod_detector->point_pattern(),
         motion_data, motion_data_windowed);

     BOOST_FOREACH(const nd_PointMotionData& data, motion_data_windowed.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             if (SWIN[idx] < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], SWIN[idx], 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], SWIN[idx], 1e-2);
             }
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             if (SWIN[idx] < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], SWIN[idx], 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], SWIN[idx], 1e-2);
             }
         }
     }
}

BOOST_AUTO_TEST_CASE( compute_fft_features_uncorrected ) {
    static const string FILENAME_STR = "noddetector1.data";
    static const double FFTWIN[] = {
        125.046834, 100.306394, 51.714796, 17.088863, 3.604974, 0.484155,
        0.040845, 0.002519, 0.000248, 0.000310, 0.000306
    };
     // get data directory
     char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
     if (!data_dir) {
         BOOST_FAIL("Data directory environment variable is not set up "
                 "correctly - verify project configuration!");
     }
     string data_path;
     data_path.append(data_dir);
     data_path.append("/");
     data_path.append(FILENAME_STR);

     nd_NodDetector * nod_detector = nd_NodDetector::load(data_path);
     const unsigned point_pattern_size =
             nod_detector->point_pattern().m_Points.size();
     const unsigned win_len = nod_detector->motion_data_size();
     const unsigned feature_len = (win_len + 1) / 2;
     double alpha = static_cast<double>(4.0 * win_len * win_len) /
         ((win_len - 1) * (win_len - 1));

     vector<double> gwin(win_len);
     initialize_gaussian_window(gwin, alpha);

     nd_PointPatternMotionData motion_data(point_pattern_size, win_len);
     BOOST_FOREACH(nd_PointMotionData& data, motion_data.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             data.m_XMotionData[idx] = idx + 1;
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             data.m_YMotionData[idx] = idx + 1;
         }
     }

     nd_PointPatternMotionData motion_data_windowed(motion_data);
     apply_gaussian_window_to_motion_data(gwin, nod_detector->point_pattern(),
         motion_data, motion_data_windowed);

     fftw_plan m_FftPlan = fftw_plan_dft_r2c_1d(win_len, 0, 0, FFTW_ESTIMATE);
     std::vector<fftw_complex*> m_FftData(point_pattern_size * 2);
     BOOST_FOREACH(fftw_complex *& fft_data, m_FftData) {
         fft_data = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * win_len);
     }

     // apply FFT to motion data of all points
     fftw_complex *const * fft_data = &m_FftData[0];
     nd_PointMotionData * pt_motion_data =
             &motion_data_windowed.m_PointMotionDatas[0];
     for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
         // apply FFT to X motion data of a point
         fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_XMotionData[0],
                 *fft_data++);
         // apply FFT to Y motion data of a point
         fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_YMotionData[0],
                 *fft_data++);
         pt_motion_data++;
     }

     nd_PointPatternMotionData feature_data(point_pattern_size, feature_len);

     // compute motion features on FFT data
     compute_motion_features_uncorrected(nod_detector->point_pattern(),
             m_FftData, feature_data);


     BOOST_FOREACH(const nd_PointMotionData& data, feature_data.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             if (FFTWIN[idx] < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], FFTWIN[idx], 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], FFTWIN[idx], 1e-2);
             }
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             if (FFTWIN[idx] < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], FFTWIN[idx], 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], FFTWIN[idx], 1e-2);
             }
         }
     }
}

BOOST_AUTO_TEST_CASE( compute_fft_features_corrected ) {
    static const string FILENAME_STR = "noddetector1.data";
    static const double FFTWIN[] = {
        6.286162, 5.228587, 2.945727, 0.925213, -0.222195, -0.606483, -0.670779, -0.691236, -0.682562, -0.616641, -0.522713,
        6.257049, 5.244383, 3.022678, 1.021820, -0.133508, -0.541107, -0.604201, -0.606976, -0.593007, -0.530853, -0.460335,
        7.386597, 6.054440, 3.428807, 1.249669, -0.028043, -0.503097, -0.596523, -0.598641, -0.578503, -0.547627, -0.497402,
        7.712452, 6.442348, 3.542249, 0.857436, -0.478363, -0.775990, -0.845179, -0.878317, -0.806634, -0.714332, -0.596357,
        7.821168, 6.721564, 4.193575, 1.404711, -0.242346, -0.597794, -0.702074, -0.750650, -0.699281, -0.631019, -0.549933,
        8.468720, 7.094426, 3.903649, 0.969554, -0.437366, -0.735369, -0.794036, -0.825666, -0.774303, -0.696987, -0.597467
    };
     // get data directory
     char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
     if (!data_dir) {
         BOOST_FAIL("Data directory environment variable is not set up "
                 "correctly - verify project configuration!");
     }
     string data_path;
     data_path.append(data_dir);
     data_path.append("/");
     data_path.append(FILENAME_STR);

     nd_NodDetector * nod_detector = nd_NodDetector::load(data_path);
     const unsigned point_pattern_size =
             nod_detector->point_pattern().m_Points.size();
     const unsigned win_len = nod_detector->motion_data_size();
     const unsigned feature_len = (win_len + 1) / 2;
     double alpha = static_cast<double>(4.0 * win_len * win_len) /
         ((win_len - 1) * (win_len - 1));

     vector<double> gwin(win_len);
     initialize_gaussian_window(gwin, alpha);

     nd_PointPatternMotionData motion_data(point_pattern_size, win_len);
     BOOST_FOREACH(nd_PointMotionData& data, motion_data.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             data.m_XMotionData[idx] = idx + 1;
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             data.m_YMotionData[idx] = idx + 1;
         }
     }

     nd_PointPatternMotionData motion_data_windowed(motion_data);
     apply_gaussian_window_to_motion_data(gwin, nod_detector->point_pattern(),
         motion_data, motion_data_windowed);

     fftw_plan m_FftPlan = fftw_plan_dft_r2c_1d(win_len, 0, 0, FFTW_ESTIMATE);
     std::vector<fftw_complex*> m_FftData(point_pattern_size * 2);
     BOOST_FOREACH(fftw_complex *& fft_data, m_FftData) {
         fft_data = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * win_len);
     }

     // apply FFT to motion data of all points
     fftw_complex *const * fft_data = &m_FftData[0];
     nd_PointMotionData * pt_motion_data =
             &motion_data_windowed.m_PointMotionDatas[0];
     for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
         // apply FFT to X motion data of a point
         fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_XMotionData[0],
                 *fft_data++);
         // apply FFT to Y motion data of a point
         fftw_execute_dft_r2c(m_FftPlan, &pt_motion_data->m_YMotionData[0],
                 *fft_data++);
         pt_motion_data++;
     }

     nd_PointPatternMotionData feature_data(point_pattern_size, feature_len);

     // compute motion features on FFT data
     compute_motion_features(nod_detector->point_pattern(),
             nod_detector->feature_normalisation_means(),
             nod_detector->feature_normalisation_scales(),
             m_FftData, feature_data);

     const double * fft_gt_ptr = FFTWIN;
     double fft_gt_val;
     BOOST_FOREACH(const nd_PointMotionData& data, feature_data.m_PointMotionDatas) {
         for (unsigned idx = 0; idx < data.m_XMotionData.size(); ++idx) {
             fft_gt_val = *fft_gt_ptr++;
             if (fft_gt_val < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], fft_gt_val, 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_XMotionData[idx], fft_gt_val, 1e-2);
             }
         }
         for (unsigned idx = 0; idx < data.m_YMotionData.size(); ++idx) {
             fft_gt_val = *fft_gt_ptr++;
             if (fft_gt_val < 1e-2) {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], fft_gt_val, 1);
             } else {
                 BOOST_CHECK_CLOSE(data.m_YMotionData[idx], fft_gt_val, 1e-2);
             }
         }
     }
}


BOOST_AUTO_TEST_SUITE_END()

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

void initialize_gaussian_window(vector<double>& window, double alpha) {
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

void apply_gaussian_window_to_motion_data(
        const vector<double>& gaussian_window,
        const nd_PointPattern& point_pattern,
        const nd_PointPatternMotionData& pattern_motion_data,
        nd_PointPatternMotionData& pattern_motion_data_windowed) {

    const unsigned point_pattern_size = point_pattern.m_Points.size();

    const nd_PointMotionData * pt_motion_data_in =
            &pattern_motion_data.m_PointMotionDatas[0];
    nd_PointMotionData * pt_motion_data_out =
            &pattern_motion_data_windowed.m_PointMotionDatas[0];

    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        // apply Gaussian window to X motion data of a point
        transform(pt_motion_data_in->m_XMotionData.begin(),
                  pt_motion_data_in->m_XMotionData.end(),
                  gaussian_window.begin(),
                  pt_motion_data_out->m_XMotionData.begin(),
                  boost::lambda::_1 * boost::lambda::_2);
        // apply Gaussian window to Y motion data of a point
        transform(pt_motion_data_in->m_YMotionData.begin(),
                  pt_motion_data_in->m_YMotionData.end(),
                  gaussian_window.begin(),
                  pt_motion_data_out->m_YMotionData.begin(),
                  boost::lambda::_1 * boost::lambda::_2);
        pt_motion_data_in++;
        pt_motion_data_out++;
    }

} // apply_gaussian_window_to_motion_data

void compute_motion_features_uncorrected(
        const nd_PointPattern& point_pattern,
        const std::vector<fftw_complex*>& fft_data,
        nd_PointPatternMotionData& features) {

    const unsigned feature_size =
            features.m_PointMotionDatas[0].m_XMotionData.size();
    const unsigned point_pattern_size = point_pattern.m_Points.size();
    const fftw_complex *const * fft_data_in = &fft_data[0];
    nd_PointMotionData * features_data_out = &features.m_PointMotionDatas[0];

    const fftw_complex * cur_fft_buffer;
    double * cur_feature_buffer;

    for (unsigned idx = 0; idx < point_pattern_size; ++idx) {
        // feature for X motion data
        cur_fft_buffer = *fft_data_in++;
        cur_feature_buffer = &features_data_out->m_XMotionData[0];
        compute_motion_feature_uncorrected(cur_fft_buffer, cur_feature_buffer, feature_size);
        // feature for Y motion data
        cur_fft_buffer = *fft_data_in++;
        cur_feature_buffer = &features_data_out->m_YMotionData[0];
        compute_motion_feature_uncorrected(cur_fft_buffer, cur_feature_buffer, feature_size);
        features_data_out++;
    }
} // compute_motion_features

void compute_motion_feature_uncorrected(const fftw_complex * fft_data,
        double * feature_data, unsigned size) {
    double val_re, val_im, val_abs;
    for (unsigned j = 0; j < size; ++j) {
        val_re = (*fft_data)[0];
        val_im = (*fft_data)[1];
        val_abs = 2 * sqrt(val_re * val_re + val_im * val_im);
        *feature_data++ = val_abs;
        fft_data++;
    }
} // compute_motion_feature

void compute_motion_features(
        const nd_PointPattern& point_pattern,
        const nd_PointPatternMotionData& norm_biases,
        const nd_PointPatternMotionData& norm_scales,
        const std::vector<fftw_complex*>& fft_data,
        nd_PointPatternMotionData& features) {

    const unsigned feature_size =
            features.m_PointMotionDatas[0].m_XMotionData.size();
    const unsigned point_pattern_size = point_pattern.m_Points.size();
    const fftw_complex *const * fft_data_in = &fft_data[0];
    const nd_PointMotionData * norm_mean_in =
            &norm_biases.m_PointMotionDatas[0];
    const nd_PointMotionData * norm_scale_in =
            &norm_scales.m_PointMotionDatas[0];
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
                cur_norm_scale_buffer, cur_feature_buffer, feature_size);
        // feature for Y motion data
        cur_fft_buffer = *fft_data_in++;
        cur_feature_buffer = &features_data_out->m_YMotionData[0];
        cur_norm_bias_buffer = &norm_mean_in->m_YMotionData[0];
        cur_norm_scale_buffer = &norm_scale_in->m_YMotionData[0];
        compute_motion_feature(cur_fft_buffer, cur_norm_bias_buffer,
                cur_norm_scale_buffer, cur_feature_buffer, feature_size);
        norm_mean_in++;
        norm_scale_in++;
        features_data_out++;
    }

} // compute_motion_features

void compute_motion_feature(const fftw_complex * fft_data,
        const double * mean_data, const double * scale_data,
        double * feature_data, unsigned size) {
    double val_re, val_im, val_abs;
    for (unsigned j = 0; j < size; ++j) {
        val_re = (*fft_data)[0];
        val_im = (*fft_data)[1];
        val_abs = 2 * sqrt(val_re * val_re + val_im * val_im);
        *feature_data++ = (val_abs - *mean_data++) / (*scale_data++);
        fft_data++;
    }
} // compute_motion_feature
