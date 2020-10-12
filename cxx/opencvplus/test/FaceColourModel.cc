/**
 * @file cxx/opencvplus/test/FaceColourModel.cc
 * @date 01 June 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief FaceColourModel unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE FaceColourModelTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
//#include <boost/filesystem/path.hpp>
#include <boost/math/distributions/students_t.hpp>       // for Student t
#include <boost/filesystem.hpp>
#include <set>
#include <iostream>
#include <iomanip>
#include <fstream>

// PROJECT INCLUDES
#include <opencvplus/cvp_Exceptions.h>
#include <opencvplus/cvp_helper_functions.h>
#include <opencvplus/FaceColorModel.h>

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

BOOST_GLOBAL_FIXTURE(cvp_Logger);

////////////////////////// LOCAL DECLARATIONS ////////////////////////////////

static FaceColorModel::FaceColorModelConfig
default_fcm_config(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd16d16(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd8d8(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd4d4(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd4d4Carl32(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd8d8Carl32(const string& data_dir);

static FaceColorModel::FaceColorModelConfig
fcm_config_ccd16d16Carl32(const string& data_dir);

void fill_image_row(IplImage * image, unsigned row,
        const CvScalar& start_colour, const CvScalar& end_colour);


/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_face_colour_model_utilities )

BOOST_AUTO_TEST_CASE( test_image_to_histogram_1 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    const cvp_Palette * palette = face_colour_model->prior_palette_bundle()->
            palette(FaceColorModel::FCM_CHANNEL_CLOTHES);
    DiscreteColorPrecompute * precompute =
            (DiscreteColorPrecompute*)palette->precomputed_data();
    // prepare input image
    IplImage * image = cvCreateImage(cvSize(100, 100), IPL_DEPTH_8U, 3);
    cvSet(image, cvScalarAll(143)); // indices (8, 8, 8)
    // prepare output image
    IplImage * hist_image = cvCreateImage(cvSize(100, 100), IPL_DEPTH_32S, 1);
    cvZero(hist_image);
    // define ROI
    CvRect roi = cvRect(10, 10, 10, 10);

    typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32s;

    const unsigned num_bins = precompute->binsNumPerDimension;
    const int bin_gt = 4;
    FaceColorModel::FaceColorModel::image_to_histogram(
            image, num_bins, hist_image, roi);
    for (int row = 0; row < 100; ++row) {
        const bool row_in_roi_flag =
                (row >= roi.y) && (row < roi.y + roi.height);
        for (int col = 0; col < 100; ++col) {
            const bool col_in_roi_flag =
                    (col >= roi.x) && (col < roi.x + roi.width);
            if (row_in_roi_flag && col_in_roi_flag) {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    bin_gt + num_bins * (bin_gt + num_bins * bin_gt));
            } else {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    0);
            }
        }
    }

    const cvp_Palette * palette2 = face_colour_model->prior_palette_bundle()->
             palette(FaceColorModel::FCM_CHANNEL_BACKGROUND);
    DiscreteColorPrecompute * precompute2 =
            (DiscreteColorPrecompute*)palette2->precomputed_data();
    const unsigned num_bins2 = precompute2->binsNumPerDimension;
    const int bin_gt2 = 8;
//    const int bin_gt2 = 4;
    FaceColorModel::FaceColorModel::image_to_histogram(
            image, num_bins2, hist_image, roi);
    for (int row = 0; row < 100; ++row) {
        const bool row_in_roi_flag =
                (row >= roi.y) && (row < roi.y + roi.height);
        for (int col = 0; col < 100; ++col) {
            const bool col_in_roi_flag =
                    (col >= roi.x) && (col < roi.x + roi.width);
            if (row_in_roi_flag && col_in_roi_flag) {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    bin_gt2 + num_bins2 * (bin_gt2 + num_bins2 * bin_gt2));
            } else {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    0);
            }
        }
    }

    cvReleaseImage(&hist_image);
    cvReleaseImage(&image);
    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_image_to_histogram_2 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    const cvp_Palette * palette = face_colour_model->prior_palette_bundle()->
            palette(FaceColorModel::FCM_CHANNEL_CLOTHES);

    DiscreteColorPrecompute * precompute1 =
            (DiscreteColorPrecompute*)palette->precomputed_data();
    const unsigned num_bins1 = precompute1->binsNumPerDimension;
    const int bin_gt1 = 7;
    // prepare input image
    IplImage * image = cvCreateImage(cvSize(100, 100), IPL_DEPTH_8U, 3);
    cvSet(image, cvScalarAll(255)); // indices (8, 8, 8)
    // prepare output image
    IplImage * hist_image = cvCreateImage(cvSize(100, 100), IPL_DEPTH_32S, 1);
    cvZero(hist_image);
    // define ROI
    CvRect roi = cvRect(10, 10, 10, 10);

    typedef cvp_IplTypeTraits<IPL_DEPTH_32S>::type int32s;

    FaceColorModel::FaceColorModel::image_to_histogram(
            image, num_bins1, hist_image, roi);
    for (int row = 0; row < 100; ++row) {
        const bool row_in_roi_flag =
                (row >= roi.y) && (row < roi.y + roi.height);
        for (int col = 0; col < 100; ++col) {
            const bool col_in_roi_flag =
                    (col >= roi.x) && (col < roi.x + roi.width);
            if (row_in_roi_flag && col_in_roi_flag) {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    bin_gt1 + num_bins1 * (bin_gt1 + num_bins1 * bin_gt1));
            } else {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    0);
            }
        }
    }

    const cvp_Palette * palette2 = face_colour_model->prior_palette_bundle()->
             palette(FaceColorModel::FCM_CHANNEL_BACKGROUND);
    DiscreteColorPrecompute * precompute2 =
            (DiscreteColorPrecompute*)palette2->precomputed_data();
    const unsigned num_bins2 = precompute2->binsNumPerDimension;
    const int bin_gt2 = 15;
//    const int bin_gt2 = 7;
    FaceColorModel::FaceColorModel::image_to_histogram(
            image, num_bins2, hist_image, roi);
    for (int row = 0; row < 100; ++row) {
        const bool row_in_roi_flag =
                (row >= roi.y) && (row < roi.y + roi.height);
        for (int col = 0; col < 100; ++col) {
            const bool col_in_roi_flag =
                    (col >= roi.x) && (col < roi.x + roi.width);
            if (row_in_roi_flag && col_in_roi_flag) {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    bin_gt2 + num_bins2 * (bin_gt2 + num_bins2 * bin_gt2));
            } else {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(hist_image, int32s, row, col),
                    0);
            }
        }
    }

    cvReleaseImage(&hist_image);
    cvReleaseImage(&image);
    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_bgr2ypbpr_01 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    const unsigned num_cols = 16;
    const unsigned num_rows = 16;

    IplImage * image = cvCreateImage(cvSize(num_cols, num_rows),
            IPL_DEPTH_8U, 3);

    fill_image_row(image, 0, CV_RGB(0, 0, 0), CV_RGB(255, 0, 0));
    fill_image_row(image, 1, CV_RGB(255, 0, 255), CV_RGB(255, 0, 0));
    fill_image_row(image, 2, CV_RGB(255, 0, 255), CV_RGB(255, 255, 255));
    fill_image_row(image, 3, CV_RGB(255, 255, 0), CV_RGB(255, 255, 255));
    fill_image_row(image, 4, CV_RGB(255, 255, 0), CV_RGB(0, 255, 0));
    fill_image_row(image, 5, CV_RGB(0, 255, 255), CV_RGB(0, 255, 0));
    fill_image_row(image, 6, CV_RGB(0, 255, 255), CV_RGB(0, 0, 255));
    fill_image_row(image, 7, CV_RGB(0, 0, 0), CV_RGB(0, 0, 255));
    fill_image_row(image, 8, CV_RGB(0, 0, 0), CV_RGB(0, 255, 0));
    fill_image_row(image, 9, CV_RGB(255, 0, 255), CV_RGB(0, 255, 0));
    fill_image_row(image, 10, CV_RGB(255, 0, 255), CV_RGB(0, 255, 255));
    fill_image_row(image, 11, CV_RGB(0, 0, 0), CV_RGB(0, 255, 255));
    fill_image_row(image, 12, CV_RGB(0, 0, 0), CV_RGB(255, 255, 0));
    fill_image_row(image, 13, CV_RGB(0, 255, 255), CV_RGB(255, 255, 0));
    fill_image_row(image, 14, CV_RGB(0, 255, 255), CV_RGB(255, 0, 0));
    fill_image_row(image, 15, CV_RGB(0, 0, 255), CV_RGB(255, 0, 0));

    IplImage * dest_image1 = cvCreateImage(cvSize(num_cols, num_rows),
            IPL_DEPTH_32F, 3);
    cvZero(dest_image1);
    IplImage * dest_image2 = cvCreateImage(cvSize(num_cols, num_rows),
            IPL_DEPTH_32F, 3);
    cvZero(dest_image2);

    bgr2ypbpr(image, dest_image1, cvRect(0, 0, num_cols, num_rows));
    bgr2ypbpr(image, dest_image2, cvRect(num_cols / 4, num_rows / 4,
            num_cols / 2, num_rows / 2));

    typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32F>::type float_32;

    for (unsigned row = num_rows / 4; row < num_rows / 4 + num_rows / 2; ++row) {
        for (unsigned col = num_cols / 4; col < num_cols / 4 + num_cols / 2; ++col) {
            for (unsigned idx = 0; idx < 3; ++idx) {
                BOOST_CHECK_EQUAL(
                    CV_IMAGE_ELEM(dest_image1, float_32, row, col * 3 + idx),
                    CV_IMAGE_ELEM(dest_image2, float_32, row, col * 3 + idx));
            }
        }
    }

    fs::path image_path = report_file_dir / "RGBImage.png";
    fs::path dest_image1_path = report_file_dir / "YPbPrImage1.png";
    fs::path dest_image2_path = report_file_dir / "YPbPrImage2.png";

    fs::path image_yml_path = report_file_dir / "RGBImage.yml";
    fs::path dest_image1_yml_path = report_file_dir / "YPbPrImage1.yml";
    fs::path dest_image2_yml_path = report_file_dir / "YPbPrImage2.yml";

    IplImage * dest_image2save1 = cvCreateImage(cvSize(num_cols, num_rows),
            IPL_DEPTH_8U, 3);
    IplImage * dest_image2save2 = cvCreateImage(cvSize(num_cols, num_rows),
            IPL_DEPTH_8U, 3);

    cvConvertScale(dest_image1, dest_image2save1, 255);
    cvConvertScale(dest_image2, dest_image2save2, 255);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    cvSaveImage(image_path.string().c_str(), image, &compression_params[0]);
    cvSaveImage(dest_image1_path.string().c_str(), dest_image2save1, &compression_params[0]);
    cvSaveImage(dest_image2_path.string().c_str(), dest_image2save2, &compression_params[0]);

    cvSave(image_yml_path.string().c_str(), image);
    cvSave(dest_image1_yml_path.string().c_str(), dest_image1);
    cvSave(dest_image2_yml_path.string().c_str(), dest_image2);

    cvReleaseImage(&dest_image2);
    cvReleaseImage(&dest_image1);
    cvReleaseImage(&image);

}

BOOST_AUTO_TEST_CASE( test_bgr2ypbpr_02 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    const unsigned num_cols = 4;

    IplImage * image = cvCreateImage(cvSize(num_cols, 1), IPL_DEPTH_8U, 3);

    unsigned char r[] = {245, 224, 219, 231};
    unsigned char g[] = {172, 154, 144, 147};
    unsigned char b[] = {131, 118, 121, 119};

    for (unsigned col = 0; col < num_cols; ++col) {
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3) = b[col];
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3 + 1) = g[col];
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3 + 2) = r[col];
    }

    IplImage * dest_image1 = cvCreateImage(cvSize(num_cols, 1), IPL_DEPTH_32F, 3);
    cvZero(dest_image1);

    typedef OpenCvPlus::cvp_IplTypeTraits<IPL_DEPTH_32F>::type float_32;

    bgr2ypbpr(image, dest_image1, cvRect(0, 0, num_cols, 1));

    float_32 y[]  = {245, 224, 219, 231};
    float_32 pb[] = {172, 154, 144, 147};
    float_32 pr[] = {131, 118, 121, 119};

    for (unsigned col = 0; col < num_cols; ++col) {
        y[col]  = (r[col] * 0.299 + g[col] * 0.587 + b[col] * 0.114) / 255;
        pb[col] = (r[col] * (-0.168736) + g[col] * (-0.331264) + b[col] * 0.5) / 255 + 0.5;
        pr[col] = (r[col] * 0.5 + g[col] * (-0.418688) + b[col] * (-0.081312)) / 255 + 0.5;
    }

    for (unsigned col = 0; col < num_cols; ++col) {
        BOOST_CHECK_CLOSE(CV_IMAGE_ELEM(dest_image1, float_32, 0, col * 3),
                y[col], 1e-5);
        BOOST_CHECK_CLOSE(CV_IMAGE_ELEM(dest_image1, float_32, 0, col * 3 + 1),
                pb[col], 1e-5);
        BOOST_CHECK_CLOSE(CV_IMAGE_ELEM(dest_image1, float_32, 0, col * 3 + 2),
                pr[col], 1e-5);
    }

    cvReleaseImage(&dest_image1);
    cvReleaseImage(&image);

}


BOOST_AUTO_TEST_CASE( test_pim_model_load_1 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    const FaceColorModel::FaceColorModel::PimFeatureType& prior_pim =
            face_colour_model->prior_pim()->data();
    const FaceColorModel::FaceColorModel::PimFeatureType& current_pim =
            face_colour_model->current_pim()->data();

    BOOST_REQUIRE(prior_pim == current_pim);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_pim_model_load_save_1 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    const FaceColorModel::FaceColorModel::PimFeatureType& prior_pim =
            face_colour_model->prior_pim()->data();
    const FaceColorModel::FaceColorModel::PimFeatureType& current_pim =
            face_colour_model->current_pim()->data();

    BOOST_REQUIRE(prior_pim == current_pim);

    fs::path outfilepath = report_file_dir / "pim_model_saved_1.data";
    fs::path p (outfilepath);
    BOOST_REQUIRE(!fs::exists(p));

    face_colour_model->current_pim()->save_to_file(outfilepath.string());

    cvp_PimModel * model = cvp_PimModel::load_from_file(outfilepath.string());
    BOOST_CHECK(model->data() == face_colour_model->prior_pim()->data());

    fs::remove(outfilepath);

    delete model;
    delete face_colour_model;

}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE( test_face_colour_model_operations )

BOOST_AUTO_TEST_CASE( test_cached_probability_maps ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);
    face_colour_model->cache_probability_maps(image);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();

    pim_feature->save_to_file(
        (report_file_dir / "fcm_proba_maps_pimfeature.data").string());

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_cached_probability_maps_roi ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);

    face_colour_model->cache_probability_maps(image, cvRect(35, 30, 185, 180));

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_proba_maps_pimfeature_roi.data").string());

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_compute_feature_1 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);

    face_colour_model->cache_probability_maps(image);

    FaceColorModel::FaceColorModel::PimFeatureType * pim_feature = new
            FaceColorModel::FaceColorModel::PimFeatureType(
                    face_colour_model->current_pim()->data());

    face_colour_model->compute_feature_on_cached_probability_maps(
        cvRect(35, 30, 185, 180), pim_feature);

    pim_feature->save_to_file(
        (report_file_dir / "fcm_pimfeature_1.data").string());

    delete pim_feature;

    cvReleaseImage(&image);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_adaptation_01 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd16d16(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);

    face_colour_model->cache_probability_maps(image);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation01_proba_maps_before_adaptation.data").string());
    face_colour_model->current_pim()->save_to_file(
        (report_file_dir / "fcm_adaptation01_pim_model_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
        (report_file_dir / "fcm_adaptation01_skin_palette_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation01_hair_palette_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
        (report_file_dir / "fcm_adaptation01_clothes_palette_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation01_bg_palette_before_adaptation.data").string());

    CvRect roi = cvRect(81, 79, 65, 65);
    face_colour_model->adapt_to(image, roi);

    face_colour_model->cache_probability_maps(image);

    pim_feature = face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation01_proba_maps_after_adaptation.data").string());

    face_colour_model->current_pim()->save_to_file(
        (report_file_dir / "fcm_adaptation01_pim_model_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
        (report_file_dir / "fcm_adaptation01_skin_palette_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation01_hair_palette_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
        (report_file_dir / "fcm_adaptation01_clothes_palette_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation01_bg_palette_after_adaptation.data").string());

    cvReleaseImage(&image);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_adaptation_01_2 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd4d4Carl32(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image3.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);

    face_colour_model->cache_probability_maps(image);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_proba_maps_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_palette_hair_before_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_palette_bg_before_adaptation.data").string());

    CvRect roi = cvRect(200, 69, 47, 47);
    face_colour_model->adapt_to(image, roi);
    face_colour_model->cache_probability_maps(image);

    pim_feature = face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_proba_maps_after_adaptation.data").string());

    face_colour_model->current_pim()->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_pim_model_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_palette_hair_after_adaptation.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation01_2_palette_bg_after_adaptation.data").string());

    cvReleaseImage(&image);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_adaptation_02 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    const unsigned num_cols = 4;

    IplImage * image = cvCreateImage(cvSize(num_cols, 1), IPL_DEPTH_8U, 3);

    unsigned char r[] = {245, 224, 219, 231};
    unsigned char g[] = {172, 154, 144, 147};
    unsigned char b[] = {131, 118, 121, 119};

    for (unsigned col = 0; col < num_cols; ++col) {
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3) = b[col];
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3 + 1) = g[col];
        CV_IMAGE_ELEM(image, unsigned char, 0, col * 3 + 2) = r[col];
    }

    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd4d4(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image_loaded = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image_loaded);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature;

    face_colour_model->prepare(image);
    face_colour_model->cache_probability_maps(image);
    pim_feature = face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation02_pixel_probas_before_adaptation.data").string());
    CvRect roi = cvRect(77, 89, 60, 60);
    face_colour_model->adapt_to(image_loaded, roi);
    face_colour_model->cache_probability_maps(image);
    pim_feature = face_colour_model->cached_probability_maps();
    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation02_pixel_probas_after_adaptation.data").string());

    cvReleaseImage(&image);

}

BOOST_AUTO_TEST_CASE( test_adaptation_03 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd16d16(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image1.jpg").string().c_str());
    BOOST_REQUIRE(image);

    face_colour_model->prepare(image);

    face_colour_model->cache_probability_maps(image);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();

    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation03_proba_maps_00.data").string());
    face_colour_model->current_pim()->save_to_file(
        (report_file_dir / "fcm_adaptation03_pim_model_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
        (report_file_dir / "fcm_adaptation03_skin_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation03_hair_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
        (report_file_dir / "fcm_adaptation03_clothes_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation03_bg_palette_00.data").string());

    CvRect roi = cvRect(81, 79, 65, 65);
    ostringstream oss;

    for (int iter_num = 0; iter_num < 10; ++iter_num) {
        face_colour_model->adapt_to(image, roi);

        face_colour_model->cache_probability_maps(image);
        pim_feature = face_colour_model->cached_probability_maps();
        oss.str("");
        oss << "fcm_adaptation03_proba_maps_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        pim_feature->save_to_file((report_file_dir / oss.str()).string());

        oss.str("");
        oss << "fcm_adaptation03_pim_model_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_pim()->save_to_file((report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation03_skin_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation03_hair_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation03_clothes_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation03_bg_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
            (report_file_dir / oss.str()).string());
    }

    cvReleaseImage(&image);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_adaptation_04 ) {

    namespace fs = boost::filesystem;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd8d8(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvLoadImage(
        (data_dir_path / "face_colour_model_image2.jpg").string().c_str());
    BOOST_REQUIRE(image);

    CvRect roi = cvRect(43, 31, 62, 62);

    CvMat * warp_mat = cvCreateMat(2, 3, CV_32F);
    cvReleaseMat(&warp_mat);

    face_colour_model->prepare(image);
    face_colour_model->cache_probability_maps(image);

    const FaceColorModel::FaceColorModel::PimFeatureType * pim_feature =
            face_colour_model->cached_probability_maps();

    pim_feature->save_to_file(
        (report_file_dir / "fcm_adaptation04_proba_maps_00.data").string());
    face_colour_model->current_pim()->save_to_file(
        (report_file_dir / "fcm_adaptation04_pim_model_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
        (report_file_dir / "fcm_adaptation04_skin_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
        (report_file_dir / "fcm_adaptation04_hair_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
        (report_file_dir / "fcm_adaptation04_clothes_palette_00.data").string());
    face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
        (report_file_dir / "fcm_adaptation04_bg_palette_00.data").string());

    ostringstream oss;

    for (int iter_num = 0; iter_num < 10; ++iter_num) {
        face_colour_model->adapt_to(image, roi);

        face_colour_model->cache_probability_maps(image);
        pim_feature = face_colour_model->cached_probability_maps();
        oss.str("");
        oss << "fcm_adaptation04_proba_maps_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        pim_feature->save_to_file((report_file_dir / oss.str()).string());

        oss.str("");
        oss << "fcm_adaptation04_pim_model_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_pim()->save_to_file((report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation04_skin_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation04_hair_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation04_clothes_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_CLOTHES)->save_to_file(
            (report_file_dir / oss.str()).string());
        oss.str("");
        oss << "fcm_adaptation04_bg_palette_" << setw(2) << setfill('0') << iter_num + 1 << ".data";
        face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_BACKGROUND)->save_to_file(
            (report_file_dir / oss.str()).string());
    }

    cvReleaseImage(&image);

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_normalization_01 ) {

    namespace fs = boost::filesystem;
    using namespace boost::math;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    int num_hist_bins = 4;
    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd4d4Carl32(data_dir);

    ostringstream oss;
    oss << "proba_sums_" << num_hist_bins << ".txt";
    ofstream out((report_file_dir / oss.str()).string().c_str());

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);
    double bin_size = 1.0 / (num_hist_bins * num_hist_bins * num_hist_bins);
    CvScalar sum_probas;
    for (int i = 0; i < 4; ++i) {
        sum_probas.val[i] = 0;
    }

    const cvp_Palette * palette_skin =
            face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN);
    const ContinuousColorPrecompute * precompute =
            (ContinuousColorPrecompute *)palette_skin->precomputed_data();
    CvScalar sum_skin;
    for (int i = 0; i < 4; ++i) {
        sum_skin.val[i] = 0;
    }
    double sum_skin_prods = 0;

    set<int> hist_indices;
    CvScalar bgr_colour;
    for (int rBin = 0; rBin < num_hist_bins; ++rBin) {
        bgr_colour.val[2] = double(255 * (rBin + 0.5)) / num_hist_bins;
        for (int gBin = 0; gBin < num_hist_bins; ++gBin) {
            bgr_colour.val[1] = double(255 * (gBin + 0.5)) / num_hist_bins;
            for (int bBin = 0; bBin < num_hist_bins; ++bBin) {
                bgr_colour.val[0] = double(255 * (bBin + 0.5)) / num_hist_bins;
                int hist_idx = face_colour_model->bgr_to_histogram(
                        bgr_colour, num_hist_bins);
                if (hist_indices.find(hist_idx) != hist_indices.end()) {
                    out << "Histogram index: " << hist_idx << endl
                        << "rBin " << rBin << ", rVal " << bgr_colour.val[2]
                        << ", gBin " << gBin << ", gVal " << bgr_colour.val[1]
                        << ", bBin " << bBin << ", bVal " << bgr_colour.val[0]
                        << endl;
                }
                BOOST_CHECK(hist_indices.find(hist_idx) == hist_indices.end());
                hist_indices.insert(hist_idx);
                CvScalar lhoods = face_colour_model->colour_likelihoods(bgr_colour);
                CvScalar ypbpr = bgr2ypbpr(bgr_colour);

                CvScalar skin_probas;
                for (int i = 0; i < 3; ++i) {
                    const double alpha = -precompute->studentPower[i] - 0.5;
                    const double sigma2 = precompute->studentScale[i];
                    const double mu = precompute->mu[i];
                    students_t stud_dist(2 * alpha);
                    skin_probas.val[i] = pdf(stud_dist, 0) / sqrt(sigma2) *
                            pow(1 + (ypbpr.val[i] - mu) * (ypbpr.val[i] - mu) / sigma2 / (2 * alpha), precompute->studentPower[i])
                            / (cdf(stud_dist, (1-mu) / sqrt(sigma2)) - cdf(stud_dist, -mu / sqrt(sigma2)));
                }
                double prod = 1;
                for (int i = 0; i < 3; ++i) {
                    sum_skin.val[i] += skin_probas.val[i] * bin_size * 0.2363;
                    prod *= skin_probas.val[i];
                }
                sum_skin_prods += prod * bin_size * 0.2363;

                for (unsigned channel = 0; channel < FaceColorModel::FCM_NUM_CHANNELS; ++channel) {
                    sum_probas.val[channel] +=
                            ((channel == FaceColorModel::FCM_CHANNEL_SKIN) ||
                            (channel == FaceColorModel::FCM_CHANNEL_HAIR)) ?
                            lhoods.val[channel] * bin_size * 0.2363 :
                            lhoods.val[channel] * bin_size;
                }
            }
        }
    }


    ostream_iterator<double> out_iter(out, ", ");
    copy(sum_skin.val, sum_skin.val + 3, out_iter);
    out << endl;
    out << sum_skin_prods << endl;
    copy(sum_probas.val, sum_probas.val + 4, out_iter);
    out << endl;

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_normalization_02 ) {

    namespace fs = boost::filesystem;
    using namespace boost::math;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    int num_hist_bins = 8;
    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd8d8Carl32(data_dir);

    ostringstream oss;
    oss << "proba_sums_" << num_hist_bins << ".txt";
    ofstream out((report_file_dir / oss.str()).string().c_str());

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);
    double bin_size = 1.0 / (num_hist_bins * num_hist_bins * num_hist_bins);
    CvScalar sum_probas;
    for (int i = 0; i < 4; ++i) {
        sum_probas.val[i] = 0;
    }

    const cvp_Palette * palette_skin =
            face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN);
    const ContinuousColorPrecompute * precompute =
            (ContinuousColorPrecompute *)palette_skin->precomputed_data();
    CvScalar sum_skin;
    for (int i = 0; i < 4; ++i) {
        sum_skin.val[i] = 0;
    }
    double sum_skin_prods = 0;

    set<int> hist_indices;
    CvScalar bgr_colour;
    for (int rBin = 0; rBin < num_hist_bins; ++rBin) {
        bgr_colour.val[2] = double(255 * (rBin + 0.5)) / num_hist_bins;
        for (int gBin = 0; gBin < num_hist_bins; ++gBin) {
            bgr_colour.val[1] = double(255 * (gBin + 0.5)) / num_hist_bins;
            for (int bBin = 0; bBin < num_hist_bins; ++bBin) {
                bgr_colour.val[0] = double(255 * (bBin + 0.5)) / num_hist_bins;
                int hist_idx = face_colour_model->bgr_to_histogram(
                        bgr_colour, num_hist_bins);
                if (hist_indices.find(hist_idx) != hist_indices.end()) {
                    out << "Histogram index: " << hist_idx << endl
                        << "rBin " << rBin << ", rVal " << bgr_colour.val[2]
                        << ", gBin " << gBin << ", gVal " << bgr_colour.val[1]
                        << ", bBin " << bBin << ", bVal " << bgr_colour.val[0]
                        << endl;
                }
                BOOST_CHECK(hist_indices.find(hist_idx) == hist_indices.end());
                hist_indices.insert(hist_idx);
                CvScalar ypbpr = bgr2ypbpr(bgr_colour);
                CvScalar skin_probas;
                for (int i = 0; i < 3; ++i) {
                    const double alpha = -precompute->studentPower[i] - 0.5;
                    const double sigma2 = precompute->studentScale[i];
                    const double mu = precompute->mu[i];
                    students_t stud_dist(2 * alpha);
                    skin_probas.val[i] = pdf(stud_dist, 0) / sqrt(sigma2) *
                            pow(1 + (ypbpr.val[i] - mu) * (ypbpr.val[i] - mu) / sigma2 / (2 * alpha), precompute->studentPower[i])
                            / (cdf(stud_dist, (1-mu) / sqrt(sigma2)) - cdf(stud_dist, -mu / sqrt(sigma2)));
                }
                double prod = 1;
                for (int i = 0; i < 3; ++i) {
                    sum_skin.val[i] += skin_probas.val[i] * bin_size * 0.2363;
                    prod *= skin_probas.val[i];
                }
                sum_skin_prods += prod * bin_size * 0.2363;

                CvScalar lhoods = face_colour_model->colour_likelihoods(bgr_colour);
                for (unsigned channel = 0; channel < FaceColorModel::FCM_NUM_CHANNELS; ++channel) {
                    sum_probas.val[channel] +=
                        ((channel == FaceColorModel::FCM_CHANNEL_SKIN) ||
                        (channel == FaceColorModel::FCM_CHANNEL_HAIR)) ?
                        lhoods.val[channel] * bin_size * 0.2363 :
                        lhoods.val[channel] * bin_size;
                }
            }
        }
    }

    ostream_iterator<double> out_iter(out, ", ");
    copy(sum_skin.val, sum_skin.val + 3, out_iter);
    out << endl;
    out << sum_skin_prods << endl;
    copy(sum_probas.val, sum_probas.val + 4, out_iter);
    out << endl;

    delete face_colour_model;

}

BOOST_AUTO_TEST_CASE( test_normalization_03 ) {

    namespace fs = boost::filesystem;
    using namespace boost::math;

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string data_dir(data);
    fs::path data_dir_path(data_dir);

    const char* report_file_var = getenv("TTRACK_TESTREPORT_FILE");
    if (!report_file_var){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTREPORT_FILE is not set." \
            " Have you setup your working environment correctly?");
    }
    const std::string report_file(report_file_var);
    fs::path report_file_path(report_file);
    fs::path report_file_dir(report_file_path.parent_path());

    int num_hist_bins = 16;
    FaceColorModel::FaceColorModelConfig config =
            fcm_config_ccd16d16Carl32(data_dir);

    ostringstream oss;
    oss << "proba_sums_" << num_hist_bins << ".txt";
    ofstream out((report_file_dir / oss.str()).string().c_str());

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);
    double bin_size = 1.0 / (num_hist_bins * num_hist_bins * num_hist_bins);
    CvScalar sum_probas;
    for (int i = 0; i < 4; ++i) {
        sum_probas.val[i] = 0;
    }

    const cvp_Palette * palette_skin =
            face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN);
    const ContinuousColorPrecompute * precompute =
            (ContinuousColorPrecompute *)palette_skin->precomputed_data();
    CvScalar sum_skin;
    for (int i = 0; i < 4; ++i) {
        sum_skin.val[i] = 0;
    }
    double sum_skin_prods = 0;

    set<int> hist_indices;
    CvScalar bgr_colour;
    for (int rBin = 0; rBin < num_hist_bins; ++rBin) {
        bgr_colour.val[2] = double(255 * (rBin + 0.5)) / num_hist_bins;
        for (int gBin = 0; gBin < num_hist_bins; ++gBin) {
            bgr_colour.val[1] = double(255 * (gBin + 0.5)) / num_hist_bins;
            for (int bBin = 0; bBin < num_hist_bins; ++bBin) {
                bgr_colour.val[0] = double(255 * (bBin + 0.5)) / num_hist_bins;
                int hist_idx = face_colour_model->bgr_to_histogram(
                        bgr_colour, num_hist_bins);
                if (hist_indices.find(hist_idx) != hist_indices.end()) {
                    out << "Histogram index: " << hist_idx << endl
                        << "rBin " << rBin << ", rVal " << bgr_colour.val[2]
                        << ", gBin " << gBin << ", gVal " << bgr_colour.val[1]
                        << ", bBin " << bBin << ", bVal " << bgr_colour.val[0]
                        << endl;
                }
                BOOST_CHECK(hist_indices.find(hist_idx) == hist_indices.end());
                hist_indices.insert(hist_idx);
                CvScalar ypbpr = bgr2ypbpr(bgr_colour);
                CvScalar skin_probas;
                for (int i = 0; i < 3; ++i) {
                    const double alpha = -precompute->studentPower[i] - 0.5;
                    const double sigma2 = precompute->studentScale[i];
                    const double mu = precompute->mu[i];
                    students_t stud_dist(2 * alpha);
                    skin_probas.val[i] = pdf(stud_dist, 0) / sqrt(sigma2) *
                            pow(1 + (ypbpr.val[i] - mu) * (ypbpr.val[i] - mu) / sigma2 / (2 * alpha), precompute->studentPower[i])
                            / (cdf(stud_dist, (1-mu) / sqrt(sigma2)) - cdf(stud_dist, -mu / sqrt(sigma2)));
                }
                double prod = 1;
                for (int i = 0; i < 3; ++i) {
                    sum_skin.val[i] += skin_probas.val[i] * bin_size * 0.2363;
                    prod *= skin_probas.val[i];
                }
                sum_skin_prods += prod * bin_size * 0.2363;

                CvScalar lhoods = face_colour_model->colour_likelihoods(bgr_colour);
                for (unsigned channel = 0; channel < FaceColorModel::FCM_NUM_CHANNELS; ++channel) {
                    sum_probas.val[channel] +=
                        ((channel == FaceColorModel::FCM_CHANNEL_SKIN) ||
                        (channel == FaceColorModel::FCM_CHANNEL_HAIR)) ?
                        lhoods.val[channel] * bin_size * 0.2363 :
                        lhoods.val[channel] * bin_size;
                }
            }
        }
    }

    ostream_iterator<double> out_iter(out, ", ");
    copy(sum_skin.val, sum_skin.val + 3, out_iter);
    out << endl;
    out << sum_skin_prods << endl;
    copy(sum_probas.val, sum_probas.val + 4, out_iter);
    out << endl;

    delete face_colour_model;

}


BOOST_AUTO_TEST_SUITE_END()

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

/* static */ FaceColorModel::FaceColorModelConfig
default_fcm_config(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_16_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_8_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_head.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd8d8(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_8_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_8_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_Carl.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd4d4(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_4_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_4_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_head.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd4d4Carl32(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_4_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_4_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_Carl_32.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd8d8Carl32(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_8_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_8_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_Carl_32.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd16d16Carl32(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_16_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_16_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_Carl_32.data";
    return config;
} // default_fcm_config

/* static */ FaceColorModel::FaceColorModelConfig
fcm_config_ccd16d16(const string& data_dir) {
    FaceColorModel::FaceColorModelConfig config;
    config.m_BackgroundColourModel = data_dir +
            "/color_prior_background_discrete_16_64bit.data";
    config.m_ClothesColourModel = data_dir +
            "/color_prior_clothes_discrete_16_64bit.data";
    config.m_HairColourModel =  data_dir +
            "/color_prior_hair_continuous_64bit.data";
    config.m_SkinColourModel = data_dir +
            "/color_prior_skin_continuous_64bit.data";
    config.m_PimModel = data_dir +
            "/pim_front_prior_Carl_32.data";
    return config;
} // default_fcm_config

void fill_image_row(IplImage * image, unsigned row,
        const CvScalar& start_colour, const CvScalar& end_colour) {

    vector<CvScalar> colours(image->width);

    for (unsigned col = 0; col < static_cast<unsigned>(image->width); ++col) {
        for (unsigned idx = 0; idx < 3; ++idx) {
            CV_IMAGE_ELEM(image, unsigned char, row, col * 3 + idx) =
                static_cast<unsigned char>(
                    round(start_colour.val[idx] + static_cast<float>(col *
                    (end_colour.val[idx] - start_colour.val[idx])) /
                    (image->width -1)));
        }
    }
} // fill_image_row
