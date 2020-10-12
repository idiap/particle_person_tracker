/**
 * @file test/NodDetectorLoad.cc
 * @date 13 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Test loading of nod detector model
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE NodDetectorLoadTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <iostream>                            // STL I/O
#include <fstream>                             // STL file I/O
#include <stdexcept>                           // STL runtime_error
#include <boost/test/included/unit_test.hpp>   // boost unit testing

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

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_noddetector_load )

BOOST_AUTO_TEST_CASE( load_nonexisting ) {
    // get data directory
     char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
     if (!data_dir) {
         BOOST_FAIL("Data directory environment variable is not set up "
                 "correctly - verify project configuration!");
     }
     string data_path;
     data_path.append(data_dir);
     data_path.append("/");
     data_path.append("noddetector_nonexisting.data");

     nd_NodDetector * nod_detector;
     BOOST_CHECK_THROW(nod_detector = nd_NodDetector::load(data_path),
         runtime_error);
}

BOOST_AUTO_TEST_CASE( load_existing_1 ) {

    static const string FILENAME_STR = "noddetector1.data";
    static const size_t MOTION_DATA_SIZE = 11;
    static const double MOTION_DATA_FREQUENCY = 9.0;
    static const size_t POINTS_NUM = 3;
    static const double POINTS_COORDS[] = {0.25, 0.33, 0.75, 0.33, 0.5, 0.66};
    static const double NORM_MEANS[] = {
        8.059490, 8.524548, 8.040117, 6.753971, 5.416837, 4.438959, 3.835061, 3.341224, 2.817767, 2.296808, 1.883041,
        7.453480, 7.787560, 7.208441, 5.916066, 4.642866, 3.716233, 3.129247, 2.667064, 2.214137, 1.825951, 1.556971,
        5.887305, 6.194188, 5.746960, 4.748249, 3.796942, 3.115375, 2.703962, 2.397086, 2.073717, 1.753306, 1.495408,
        6.005479, 6.965380, 7.780704, 8.144300, 8.359372, 8.503267, 8.303669, 7.381568, 5.820093, 4.305316, 3.142067,
        4.882948, 5.340156, 5.520085, 5.507129, 5.634810, 5.837362, 5.644589, 4.906720, 3.923667, 3.038617, 2.355617,
        5.643541, 6.509132, 7.252624, 7.619879, 7.811554, 7.967792, 7.718038, 6.786442, 5.310961, 3.888379, 2.837377
    };
    static const double NORM_SCALES[] = {
        18.610298, 17.553854, 14.826450, 11.170278, 8.154400, 6.520877, 5.656430, 4.830052, 4.127861, 3.724207, 3.601853,
        18.793740, 17.641510, 14.724149, 10.934217, 7.773991, 5.973087, 5.111549, 4.389872, 3.733325, 3.439070, 3.381591,
        16.131857, 15.544330, 13.406364, 9.875102, 6.845517, 5.230046, 4.464400, 4.000001, 3.584197, 3.201078, 3.005821,
        15.434957, 14.488663, 12.402879, 10.431753, 9.938895, 10.334036, 9.776415, 8.401352, 7.214979, 6.026619, 5.268260,
        15.363932, 14.128593, 11.015592, 8.244921, 8.375782, 8.954943, 7.981699, 6.533270, 5.610647, 4.814923, 4.282908,
        14.099331, 13.221262, 11.389900, 9.766328, 9.617977, 10.176706, 9.668577, 8.216303, 6.858704, 5.578396, 4.748502
    };
    static const double SVM_BIAS = -0.011229;
    static const double SVM_WEIGHTS[] = {
        -0.063315, -0.050275, -0.014649,  0.018806,  0.013188, -0.018096, -0.008729,  0.018297,  0.029025,  0.013491, -0.003860,
        -0.068175, -0.064702, -0.045830, -0.025741, -0.025622, -0.036188, -0.027941, -0.016533, -0.018894, -0.034071, -0.035133,
        -0.077121, -0.072441, -0.054366, -0.030480, -0.026375, -0.044389, -0.038606, -0.027732, -0.019452, -0.027378, -0.033935,
        -0.058951, -0.033398,  0.031589,  0.102845,  0.104451,  0.076519,  0.093883,  0.133933,  0.127959,  0.099885,  0.045928,
        -0.080445, -0.091550, -0.094674, -0.069359, -0.035090, -0.022039, -0.014882, -0.003474, -0.004279, -0.009622, -0.022054,
        -0.033857, -0.005876,  0.057999,  0.118490,  0.107787,  0.064643,  0.064128,  0.087058,  0.076488,  0.044212, -0.002785,
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

     // check point coordinates
     const nd_PointPattern& point_pattern = nod_detector->point_pattern();
     BOOST_CHECK_EQUAL(point_pattern.m_Points.size(), POINTS_NUM);
     const double * pt_ptr = POINTS_COORDS;
     double pt_coord_val;
     for (unsigned idx = 0; idx < POINTS_NUM; ++idx) {
         pt_coord_val = *pt_ptr++;
         BOOST_CHECK_CLOSE(point_pattern.m_Points[idx].m_X, pt_coord_val, 1e-5);
         pt_coord_val = *pt_ptr++;
         BOOST_CHECK_CLOSE(point_pattern.m_Points[idx].m_Y, pt_coord_val, 1e-5);
     }

     // check motion data length
     BOOST_CHECK_EQUAL(nod_detector->motion_data_size(), 2 * MOTION_DATA_SIZE - 1);

     // check feature normalisation means
     const nd_PointPatternMotionData& norm_means =
             nod_detector->feature_normalisation_means();
     BOOST_CHECK_EQUAL(norm_means.m_PointMotionDatas.size(), POINTS_NUM);
     const double * norm_means_ptr = NORM_MEANS;
     double norm_means_val;
     BOOST_FOREACH(const nd_PointMotionData& pt_motion,
             norm_means.m_PointMotionDatas) {
         BOOST_CHECK_EQUAL(pt_motion.m_XMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             norm_means_val = *norm_means_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_XMotionData[idx], norm_means_val, 1e-3);
         }
         BOOST_CHECK_EQUAL(pt_motion.m_YMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             norm_means_val = *norm_means_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_YMotionData[idx], norm_means_val, 1e-3);
         }
     }

     // check feature normalisation scales
     const nd_PointPatternMotionData& norm_scales =
             nod_detector->feature_normalisation_scales();
     BOOST_CHECK_EQUAL(norm_scales.m_PointMotionDatas.size(), POINTS_NUM);
     const double * norm_scales_ptr = NORM_SCALES;
     double norm_scales_val;
     BOOST_FOREACH(const nd_PointMotionData& pt_motion,
             norm_scales.m_PointMotionDatas) {
         BOOST_CHECK_EQUAL(pt_motion.m_XMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             norm_scales_val = *norm_scales_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_XMotionData[idx], norm_scales_val, 1e-3);
         }
         BOOST_CHECK_EQUAL(pt_motion.m_YMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             norm_scales_val = *norm_scales_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_YMotionData[idx], norm_scales_val, 1e-3);
         }
     }

     // check SVM weights
     const nd_PointPatternMotionData& svm_weights = nod_detector->svm_weights();
     BOOST_CHECK_EQUAL(svm_weights.m_PointMotionDatas.size(), POINTS_NUM);
     const double * svm_weights_ptr = SVM_WEIGHTS;
     double svm_weights_val;
     BOOST_FOREACH(const nd_PointMotionData& pt_motion,
             svm_weights.m_PointMotionDatas) {
         BOOST_CHECK_EQUAL(pt_motion.m_XMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             svm_weights_val = *svm_weights_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_XMotionData[idx], svm_weights_val, 1e-3);
         }
         BOOST_CHECK_EQUAL(pt_motion.m_YMotionData.size(), MOTION_DATA_SIZE);
         for (unsigned idx = 0; idx < MOTION_DATA_SIZE; ++idx) {
             svm_weights_val = *svm_weights_ptr++;
             BOOST_CHECK_CLOSE(pt_motion.m_YMotionData[idx], svm_weights_val, 1e-3);
         }
     }

     // check SVM bias
     BOOST_CHECK_CLOSE(nod_detector->svm_bias(), SVM_BIAS, 1e-3);

     // check motion data frequency
     BOOST_CHECK_CLOSE(nod_detector->motion_data_frequency(),
             MOTION_DATA_FREQUENCY, 1e-5);
}

BOOST_AUTO_TEST_SUITE_END()
