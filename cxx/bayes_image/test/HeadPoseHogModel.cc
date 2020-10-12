/**
 * @file cxx/bayes_image/test/HeadPoseHogModel.cc
 * @date 22 October 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Unit tests for bicv_HeadPoseHogModel class
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE HeadPoseHogModelTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <bayes_image/bicv_HogModelTrainer.h>
#include <bayes_image/bicv_Exceptions.h>

using namespace BICV;
using namespace ImageProcessing;
using namespace std;
using namespace boost::filesystem;

typedef bicv_HeadPoseHogModel::HeadPose HeadPose;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace BICV {
    struct bicv_Logger {
        bicv_Logger() {
            const char* report_file_path = getenv("TTRACK_TESTREPORT_FILE");
              if (!report_file_path) {
                  throw bicv_Exception( \
                      "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
                      " Have you setup your working environment correctly?");
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~bicv_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(bicv_Logger)

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_load )

BOOST_AUTO_TEST_CASE( test_load_nonexisting ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    bicv_HeadPoseHogModel * model;
    std::string filepath = data_dir + "/hog_model_xyz.xml";

    BOOST_CHECK_THROW(model = bicv_HeadPoseHogModel::load(filepath), \
        bicv_Exception);
}

BOOST_AUTO_TEST_CASE( test_load_correct_1 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    bicv_HeadPoseHogModel * model;
    std::string filepath = data_dir + "/hog_model_1.xml";

    model = bicv_HeadPoseHogModel::load(filepath);

    static const int ETALON_HISTOGRAM_SIZE = 8;
    static const HeadPose ETALON_DESCRIPTOR = HeadPose(10, 10, 10);

    // check mean templates
    const vector<ip_HistogramTemplate>& mean_templates =
        model->mean_templates();
    BOOST_CHECK_EQUAL(mean_templates.size(), 1);
    const ip_HistogramTemplate& mean_template = mean_templates[0];
    BOOST_CHECK_EQUAL(mean_template.size(), 1);
    const ip_HistogramTemplate::Histogram& mean_histogram =
        mean_template.get_histogram(0);
    BOOST_CHECK_EQUAL(mean_histogram.size(), ETALON_HISTOGRAM_SIZE);
    for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
        BOOST_CHECK_EQUAL(mean_histogram[i], i + 1);
    }

    // check mean template descriptors
    const std::vector<HeadPose>& mean_template_descriptors =
        model->mean_template_descriptors();
    BOOST_CHECK_EQUAL(mean_template_descriptors.size(), 1);
    const HeadPose& mean_template_descriptor = mean_template_descriptors[0];
    BOOST_CHECK(mean_template_descriptor == ETALON_DESCRIPTOR);

    // check stddev templates
    const vector<ip_HistogramTemplate>& stddev_templates =
        model->stddev_templates();
    BOOST_CHECK_EQUAL(stddev_templates.size(), 1);
    const ip_HistogramTemplate& stddev_template = stddev_templates[0];
    BOOST_CHECK_EQUAL(stddev_template.size(), 1);
    const ip_HistogramTemplate::Histogram& stddev_histogram =
            stddev_template.get_histogram(0);
    BOOST_CHECK_EQUAL(stddev_histogram.size(), ETALON_HISTOGRAM_SIZE);
    for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
        BOOST_CHECK_CLOSE(stddev_histogram[i], 1.1 * (i + 1), 1e-5);
    }

    // check stddev template descriptors
    const std::vector<HeadPose>& stddev_template_descriptors =
        model->stddev_template_descriptors();
    BOOST_CHECK_EQUAL(stddev_template_descriptors.size(), 1);
    const HeadPose& stddev_template_descriptor = stddev_template_descriptors[0];
    BOOST_CHECK(stddev_template_descriptor == ETALON_DESCRIPTOR);
}

BOOST_AUTO_TEST_CASE( test_load_correct_21 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    bicv_HeadPoseHogModel * model;
    std::string filepath = data_dir + "/hog_model_21.xml";

    model = bicv_HeadPoseHogModel::load(filepath);

    static const int ETALON_TEMPLATE_SIZE = 5;
    static const int ETALON_HISTOGRAM_SIZE = 8;
    static const HeadPose ETALON_DESCRIPTOR = HeadPose(10, 10, 10);

    // check mean templates
    const vector<ip_HistogramTemplate>& mean_templates =
        model->mean_templates();
    BOOST_CHECK_EQUAL(mean_templates.size(), 1);
    const ip_HistogramTemplate& mean_template = mean_templates[0];
    BOOST_CHECK_EQUAL(mean_template.size(), ETALON_TEMPLATE_SIZE);
    for (int j = 0; j < ETALON_TEMPLATE_SIZE; ++j) {
        const ip_HistogramTemplate::Histogram& mean_histogram =
            mean_template.get_histogram(j);
        BOOST_CHECK_EQUAL(mean_histogram.size(), ETALON_HISTOGRAM_SIZE);
        for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
            BOOST_CHECK_EQUAL(mean_histogram[i], i + 1);
        }
    }

    // check mean template descriptors
    const std::vector<HeadPose>& mean_template_descriptors =
        model->mean_template_descriptors();
    BOOST_CHECK_EQUAL(mean_template_descriptors.size(), 1);
    const HeadPose& mean_template_descriptor = mean_template_descriptors[0];
    BOOST_CHECK(mean_template_descriptor == ETALON_DESCRIPTOR);

    // check stddev templates
    const vector<ip_HistogramTemplate>& stddev_templates =
        model->stddev_templates();
    BOOST_CHECK_EQUAL(stddev_templates.size(), 1);
    const ip_HistogramTemplate& stddev_template = stddev_templates[0];
    BOOST_CHECK_EQUAL(stddev_template.size(), ETALON_TEMPLATE_SIZE);
    for (int j = 0; j < ETALON_TEMPLATE_SIZE; ++j) {
        const ip_HistogramTemplate::Histogram& stddev_histogram =
                stddev_template.get_histogram(j);
        BOOST_CHECK_EQUAL(stddev_histogram.size(), ETALON_HISTOGRAM_SIZE);
        for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
            BOOST_CHECK_CLOSE(stddev_histogram[i], 1.1 * (i + 1), 1e-5);
        }
    }

    // check stddev template descriptors
    const std::vector<HeadPose>& stddev_template_descriptors =
        model->stddev_template_descriptors();
    BOOST_CHECK_EQUAL(stddev_template_descriptors.size(), 1);
    const HeadPose& stddev_template_descriptor = stddev_template_descriptors[0];
    BOOST_CHECK(stddev_template_descriptor == ETALON_DESCRIPTOR);
}

BOOST_AUTO_TEST_CASE( test_load_noparams ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    bicv_HeadPoseHogModel * model;
    std::string filepath = data_dir + "/hog_model_noparams.xml";

    BOOST_CHECK_THROW(model = bicv_HeadPoseHogModel::load(filepath), \
        bicv_Exception);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( test_save )

BOOST_AUTO_TEST_CASE( test_save_1 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    bicv_HeadPoseHogModel * model;
    std::string filepath = data_dir + "/hog_model_1.xml";

    model = bicv_HeadPoseHogModel::load(filepath);

    std::string outfilepath = data_dir + "/hog_model_1_out.xml";
    path p (outfilepath);
    BOOST_REQUIRE(!exists(p));

    model->save(outfilepath);

    bicv_HeadPoseHogModel * model_reloaded;

    model_reloaded = bicv_HeadPoseHogModel::load(outfilepath);

    static const int ETALON_HISTOGRAM_SIZE = 8;
    static const HeadPose ETALON_DESCRIPTOR = HeadPose(10, 10, 10);

    // check mean templates
    const vector<ip_HistogramTemplate>& mean_templates =
        model_reloaded->mean_templates();
    BOOST_CHECK_EQUAL(mean_templates.size(), 1);
    const ip_HistogramTemplate& mean_template = mean_templates[0];
    BOOST_CHECK_EQUAL(mean_template.size(), 1);
    const ip_HistogramTemplate::Histogram& mean_histogram =
        mean_template.get_histogram(0);
    BOOST_CHECK_EQUAL(mean_histogram.size(), ETALON_HISTOGRAM_SIZE);
    for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
        BOOST_CHECK_EQUAL(mean_histogram[i], i + 1);
    }

    // check mean template descriptors
    const std::vector<HeadPose>& mean_template_descriptors =
        model_reloaded->mean_template_descriptors();
    BOOST_CHECK_EQUAL(mean_template_descriptors.size(), 1);
    const HeadPose& mean_template_descriptor = mean_template_descriptors[0];
    BOOST_CHECK(mean_template_descriptor == ETALON_DESCRIPTOR);

    // check stddev templates
    const vector<ip_HistogramTemplate>& stddev_templates =
        model_reloaded->stddev_templates();
    BOOST_CHECK_EQUAL(stddev_templates.size(), 1);
    const ip_HistogramTemplate& stddev_template = stddev_templates[0];
    BOOST_CHECK_EQUAL(stddev_template.size(), 1);
    const ip_HistogramTemplate::Histogram& stddev_histogram =
            stddev_template.get_histogram(0);
    BOOST_CHECK_EQUAL(stddev_histogram.size(), ETALON_HISTOGRAM_SIZE);
    for (int i = 0; i < ETALON_HISTOGRAM_SIZE; ++i) {
        BOOST_CHECK_CLOSE(stddev_histogram[i], 1.1 * (i + 1), 1e-5);
    }

    // check stddev template descriptors
    const std::vector<HeadPose>& stddev_template_descriptors =
        model_reloaded->stddev_template_descriptors();
    BOOST_CHECK_EQUAL(stddev_template_descriptors.size(), 1);
    const HeadPose& stddev_template_descriptor = stddev_template_descriptors[0];
    BOOST_CHECK(stddev_template_descriptor == ETALON_DESCRIPTOR);

    remove(outfilepath);
}

BOOST_AUTO_TEST_SUITE_END()
