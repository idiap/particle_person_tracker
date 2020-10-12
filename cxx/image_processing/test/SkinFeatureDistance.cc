/**
 * @file cxx/image_processing/test/SkinFeatureDistance.cc
 * @date 19 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief SkinFeatureDistance unit tests
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
#include <image_processing/ip_SkinFeatureProducer.h>

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

BOOST_AUTO_TEST_SUITE( test_skin_feature_distance )

BOOST_AUTO_TEST_CASE( test_skin_feature_distance_01 ) {

    static const unsigned FEATURE_SIZE = 10;
    ip_SkinTemplate skin_feature_1(FEATURE_SIZE);
    fill_n(&skin_feature_1[0], FEATURE_SIZE, 1.0);

    ip_SkinTemplate skin_feature_2(FEATURE_SIZE);
    fill_n(&skin_feature_2[0], FEATURE_SIZE, 0.5);

    OpenCvPlus::real d_emp = distance_emp(skin_feature_1, skin_feature_2);
    BOOST_CHECK_CLOSE(d_emp, 1.0, 1e-5);
    OpenCvPlus::real d_L1 = distance_L1(skin_feature_1, skin_feature_2);
    BOOST_CHECK_CLOSE(d_L1, 5.0, 1e-5);
    OpenCvPlus::real d_L2 = distance_L2_squared(skin_feature_1, skin_feature_2);
    BOOST_CHECK_CLOSE(d_L2, 2.5, 1e-5);

    ip_SkinTemplate skin_feature_stddev_1(FEATURE_SIZE);
    fill_n(&skin_feature_stddev_1[0], FEATURE_SIZE, 1.0);
    OpenCvPlus::real d_Mah_1 = distance_mahalanobis_squared(
            skin_feature_1, skin_feature_2, skin_feature_stddev_1);
    BOOST_CHECK_CLOSE(d_Mah_1, 2.5, 1e-5);

    ip_SkinTemplate skin_feature_stddev_2(FEATURE_SIZE);
    fill_n(&skin_feature_stddev_2[0], FEATURE_SIZE, 2.0);
    OpenCvPlus::real d_Mah_2 = distance_mahalanobis_squared(
            skin_feature_1, skin_feature_2, skin_feature_stddev_2);
    BOOST_CHECK_CLOSE(d_Mah_2, 2.5 / 4.0, 1e-5);

}

BOOST_AUTO_TEST_SUITE_END()
