/**
 * @file cxx/image_processing/test/HogFeatureDistance.cc
 * @date 19 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief HogFeatureDistance unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE SkinFeatureDistanceTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <image_processing/ip_HistogramTemplate.h>

using namespace ImageProcessing;
using namespace OpenCvPlus;
using namespace std;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace OpenCvPlus {
    struct cvp_Logger {
        cvp_Logger()   {
            const char* report_file_path = getenv("TTRACK_TESTREPORT_FILE");
              if (!report_file_path) {
                  std::cerr <<
                  "Environment variable $TTRACK_TESTREPORT_FILE is not set."
                  " Have you setup your working environment correctly?" <<
                  std::endl;
                  throw cvp_Exception();
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~cvp_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(cvp_Logger)

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_hog_feature_distance )

BOOST_AUTO_TEST_CASE( test_hog_feature_distance_01 ) {

    static const unsigned FEATURE_SIZE = 4;
    static const unsigned NUM_BINS = 8;

    ip_HistogramTemplate hog_feature_1(FEATURE_SIZE, NUM_BINS);
    for (unsigned i = 0; i < FEATURE_SIZE; ++i) {
        ip_HistogramTemplate::Histogram& hist = hog_feature_1.get_histogram(i);
        fill_n(&hist[0], NUM_BINS, 1.0);
    }


    ip_HistogramTemplate hog_feature_2(FEATURE_SIZE, NUM_BINS);
    for (unsigned i = 0; i < FEATURE_SIZE; ++i) {
        ip_HistogramTemplate::Histogram& hist = hog_feature_2.get_histogram(i);
        fill_n(&hist[0], NUM_BINS, 0.1 * i);
    }

    OpenCvPlus::real d_L1 = distance_L1(hog_feature_1, hog_feature_2);
    BOOST_CHECK_CLOSE(d_L1, NUM_BINS * (1.0 + 0.9 + 0.8 + 0.7), 1e-5);
    OpenCvPlus::real d_L1_thr = distance_L1(hog_feature_1, hog_feature_2, 1.0);
    BOOST_CHECK_CLOSE(d_L1_thr, 4.0, 1e-5);

    OpenCvPlus::real d_L2 = distance_L2_squared(hog_feature_1, hog_feature_2);
    BOOST_CHECK_CLOSE(d_L2,
            NUM_BINS * (1.0 + 0.9 * 0.9 + 0.8 * 0.8 + 0.7 * 0.7), 1e-5);
    OpenCvPlus::real d_L2_thr =
            distance_L2_squared(hog_feature_1, hog_feature_2, 1.0);
    BOOST_CHECK_CLOSE(d_L2_thr, 4.0, 1e-5);

    OpenCvPlus::real d_C2 = distance_Chi2(hog_feature_1, hog_feature_2);
    BOOST_CHECK_CLOSE(d_C2, NUM_BINS *
            (1.0 + 0.9 * 0.9 / 1.1 + 0.8 * 0.8 / 1.2 + 0.7 * 0.7 / 1.3), 1e-5);
    OpenCvPlus::real d_C2_thr =
            distance_Chi2(hog_feature_1, hog_feature_2, 1.0);
    BOOST_CHECK_CLOSE(d_C2_thr, 4.0, 1e-5);


    ip_HistogramTemplate stddev_1(FEATURE_SIZE, NUM_BINS);
    for (unsigned i = 0; i < FEATURE_SIZE; ++i) {
        ip_HistogramTemplate::Histogram& hist = stddev_1.get_histogram(i);
        fill_n(&hist[0], NUM_BINS, 1.0);
    }
    OpenCvPlus::real d_Mah_1 = distance_Mahalanobis_squared(
            hog_feature_1, hog_feature_2, stddev_1);
    BOOST_CHECK_CLOSE(d_Mah_1,
            NUM_BINS * (1.0 + 0.9 * 0.9 + 0.8 * 0.8 + 0.7 * 0.7), 1e-5);
    OpenCvPlus::real d_Mah_thr_1 =
            distance_Mahalanobis_squared(hog_feature_1, hog_feature_2,
                    stddev_1, 1.0);
    BOOST_CHECK_CLOSE(d_Mah_thr_1, 4.0, 1e-5);

    ip_HistogramTemplate stddev_2(FEATURE_SIZE, NUM_BINS);
    for (unsigned i = 0; i < FEATURE_SIZE; ++i) {
        ip_HistogramTemplate::Histogram& hist = stddev_2.get_histogram(i);
        fill_n(&hist[0], NUM_BINS, 2.0);
    }
    OpenCvPlus::real d_Mah_2 = distance_Mahalanobis_squared(
            hog_feature_1, hog_feature_2, stddev_2);
    BOOST_CHECK_CLOSE(d_Mah_2,
            NUM_BINS * (1.0 + 0.9 * 0.9 + 0.8 * 0.8 + 0.7 * 0.7) / 4.0, 1e-5);
    OpenCvPlus::real d_Mah_thr_2 =
            distance_Mahalanobis_squared(hog_feature_1, hog_feature_2,
                    stddev_2, 1.0);
    BOOST_CHECK_CLOSE(d_Mah_thr_2, 3.0 + 0.98, 1e-5);
}

BOOST_AUTO_TEST_SUITE_END()
