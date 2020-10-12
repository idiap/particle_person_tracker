/**
 * @file cxx/image_processing/test/SkinColourProcessor.cc
 * @date 02 January 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief SkinColourProcessor unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE SkinColourProcessorTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <highgui.h>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <opencvplus/cvp_helper_functions.h>
#include <image_processing/ip_SingleImageProvider.h>
#include <image_processing/ip_SkinColourProcessor.h>

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

////////////////////////// LOCAL DECLARATIONS ////////////////////////////////

static FaceColorModel::FaceColorModelConfig
default_fcm_config(const string& data_dir);

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_skin_colour_processor )

BOOST_AUTO_TEST_CASE( test_skin_colour_processor_01 ) {

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

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvCreateImage(cvSize(10, 10), IPL_DEPTH_8U, 3);
    cvZero(image);
    CvRect skin_roi = cvRect(3, 3, 4, 4);
    cvSetImageROI(image, skin_roi);
    CvScalar skin_pixel = CV_RGB(205, 133, 63);
    CvScalar skin_pixel_ypbpr = bgr2ypbpr(skin_pixel);
    ofstream out((report_file_dir / "SkinColourProcessor_Test01.txt").string().c_str());

    ostream_iterator<double> out_iter(out, ", ");
    copy(skin_pixel_ypbpr.val, skin_pixel_ypbpr.val + 3, out_iter);
    out << endl;
    cvSet(image, skin_pixel);
    cvResetImageROI(image);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    fs::path image_path = report_file_dir / "SkinImage.png";
    cvSaveImage(image_path.string().c_str(), image, &compression_params[0]);

    ip_ImageProvider * image_provider = new ip_SingleImageProvider(image);
    ip_SkinColourProcessor * skin_colour_processor = new ip_SkinColourProcessor(
        image_provider, face_colour_model, 255.0f);
    face_colour_model->prepare(image);

    image_provider->invalidate();
    skin_colour_processor->invalidate();

    fs::path skin_mask_path = report_file_dir / "SkinMaskImage.png";
    IplImage * skin_mask_image = skin_colour_processor->image();
    cvSaveImage(skin_mask_path.string().c_str(), skin_mask_image,
            &compression_params[0]);
    face_colour_model->cached_probability_maps()->save_to_file(
            (report_file_dir / "SkinColourProcessor_Test01_proba_maps.data").string());
    out << *face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_SKIN) << endl;
    out << *face_colour_model->current_palette_bundle()->palette(FaceColorModel::FCM_CHANNEL_HAIR) << endl;
    CvScalar lhoods = face_colour_model->colour_likelihoods(skin_pixel);
    copy(lhoods.val, lhoods.val + 4, out_iter);
    out << endl;
    CvScalar lhoods2;
    for (unsigned channel = 0; channel < FaceColorModel::FCM_NUM_CHANNELS; ++channel) {
        lhoods2.val[channel] = CV_MAT_ELEM(
                *face_colour_model->cached_probability_maps()->map(channel),
                FaceColorModel::FaceColorModel::PimFeatureType::value_type, 4, 4);
    }
    copy(lhoods2.val, lhoods2.val + 4, out_iter);
    out << endl;
    for (int col = 0; col < skin_mask_image->width; ++col) {
        for (int row = 0; row < skin_mask_image->height; ++row) {
            const unsigned char pixel =
                CV_IMAGE_ELEM(skin_mask_image, unsigned char, row, col);
            if ((col >= 3) && (col < 7) && (row >= 3) && (row < 7)) {
                BOOST_CHECK_EQUAL(pixel, 255);
            } else {
                BOOST_CHECK_EQUAL(pixel, 0);
            }
        }
    }


    delete skin_colour_processor;
    delete image_provider;
    cvReleaseImage(&image);

}

BOOST_AUTO_TEST_CASE( test_skin_colour_processor_02 ) {

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

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvCreateImage(cvSize(10, 10), IPL_DEPTH_8U, 3);
    CvScalar bg_pixel = CV_RGB(0, 0, 255);
    cvSet(image, bg_pixel);
    CvRect skin_roi = cvRect(3, 3, 4, 4);
    cvSetImageROI(image, skin_roi);
    CvScalar skin_pixel = CV_RGB(205, 133, 63);
    cvSet(image, skin_pixel);
    cvResetImageROI(image);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

//    fs::path image_path = report_file_dir / "SkinImage.png";
//    cvSaveImage(image_path.string().c_str(), image, &compression_params[0]);

    ip_ImageProvider * image_provider = new ip_SingleImageProvider(image);
    ip_SkinColourProcessor * skin_colour_processor = new ip_SkinColourProcessor(
        image_provider, face_colour_model, 255.0f);
    face_colour_model->prepare(image);

    image_provider->invalidate();
    skin_colour_processor->invalidate();

//    fs::path skin_mask_path = report_file_dir / "SkinMaskImage.png";
    IplImage * skin_mask_image = skin_colour_processor->image();
//    cvSaveImage(skin_mask_path.string().c_str(), skin_mask_image,
//            &compression_params[0]);
    for (int col = 0; col < skin_mask_image->width; ++col) {
        for (int row = 0; row < skin_mask_image->height; ++row) {
            const unsigned char pixel =
                CV_IMAGE_ELEM(skin_mask_image, unsigned char, row, col);
            if ((col >= 3) && (col < 7) && (row >= 3) && (row < 7)) {
                BOOST_CHECK_EQUAL(pixel, 255);
            } else {
                BOOST_CHECK_EQUAL(pixel, 0);
            }
        }
    }

    delete skin_colour_processor;
    delete image_provider;
    cvReleaseImage(&image);

}

BOOST_AUTO_TEST_CASE( test_skin_colour_processor_03 ) {

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

    FaceColorModel::FaceColorModelConfig config =
            default_fcm_config(data_dir);

    FaceColorModel::FaceColorModel * face_colour_model =
        new FaceColorModel::FaceColorModel(config);

    IplImage * image = cvCreateImage(cvSize(100, 100), IPL_DEPTH_8U, 3);
    CvScalar bg_pixel = CV_RGB(0, 0, 255);
    cvSet(image, bg_pixel);
    CvRect skin_roi = cvRect(30, 30, 40, 40);
    cvSetImageROI(image, skin_roi);
    CvScalar skin_pixel = CV_RGB(205, 133, 63);
    cvSet(image, skin_pixel);
    cvResetImageROI(image);

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

//    fs::path image_path = report_file_dir / "SkinImage.png";
//    cvSaveImage(image_path.string().c_str(), image, &compression_params[0]);

    ip_ImageProvider * image_provider = new ip_SingleImageProvider(image);
    ip_SkinColourProcessor * skin_colour_processor = new ip_SkinColourProcessor(
        image_provider, face_colour_model, 255.0f);
    face_colour_model->prepare(image);

    image_provider->invalidate();
    skin_colour_processor->invalidate();

    ip_RoiWindow roi(0, 0, 50, 50);
    IplImage * skin_mask_image = skin_colour_processor->image(roi);

//    fs::path skin_mask_path = report_file_dir / "SkinMaskImage.png";
//    cvSaveImage(skin_mask_path.string().c_str(), skin_mask_image,
//            &compression_params[0]);

//    face_colour_model->cached_probability_maps()->save_to_file(
//            (report_file_dir / "SkinMaskTestProbaMaps.data").string());

    for (int col = 0; col < skin_mask_image->width; ++col) {
        for (int row = 0; row < skin_mask_image->height; ++row) {
            const unsigned char pixel =
                CV_IMAGE_ELEM(skin_mask_image, unsigned char, row, col);
            if ((col >= 30) && (col < 50) && (row >= 30) && (row < 50)) {
                BOOST_CHECK_EQUAL(pixel, 255);
            } else {
                BOOST_CHECK_EQUAL(pixel, 0);
            }
        }
    }

    delete skin_colour_processor;
    delete image_provider;
    cvReleaseImage(&image);

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
