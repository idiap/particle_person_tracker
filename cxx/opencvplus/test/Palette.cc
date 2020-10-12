/**
 * @file cxx/opencvplus/test/Palette.cc
 * @date 20 September 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Palette unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PaletteTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <cstdlib>                           // for getenv
#include <list>                              // STL list
#include <iostream>
#include <fstream>
#include <numeric>                                       // for accumulate
#include <boost/test/unit_test.hpp>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/digamma.hpp>
#include <boost/filesystem.hpp>

// LOCAL INCLUDES
#include <opencvplus/cvp_Exceptions.h>
#include <opencvplus/cvp_Palette.h>

using namespace OpenCvPlus;
using namespace std;
using namespace boost::filesystem;

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

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

// variable name that contains a path to test data directory
static const string DATA_DIR_ENVIRONMENT_VAR = "TTRACK_TESTDATA_DIR";

struct ModelInfo {
    ModelInfo(const string& file_name, cvp_Palette::Type model_type,
            unsigned data_size) :
        m_FileName(file_name), m_ModelType(model_type), m_DataSize(data_size) {
    }
    string m_FileName;
    cvp_Palette::Type m_ModelType;
    unsigned m_DataSize;
};

typedef list<ModelInfo> ModelInfoList;

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_palette )

BOOST_AUTO_TEST_CASE( test_palette_load_simple_discrete_1 ) {
    // get data directory
     char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
     if (!data_dir) {
         BOOST_FAIL("Data directory environment variable is not set up "
                 "correctly - verify project configuration!");
     }
     string data_path;
     data_path.append(data_dir);
     data_path.append("/");

     ModelInfoList model_infos;
     cvp_Palette::Type palette_type;
     unsigned palette_size;
     unsigned bins_num;

     palette_type = cvp_Palette::FCM_CONTINUOUS;
     palette_size = 12;
     model_infos.push_back(
         ModelInfo(data_path + "sample_prior_continuous_1_64bit.data",
             palette_type, palette_size));

     palette_type = cvp_Palette::FCM_DISCRETE;
     bins_num = 4;
     palette_size = bins_num * bins_num * bins_num;
     model_infos.push_back(
         ModelInfo(data_path + "sample_prior_discrete_4_64bit.data",
             palette_type, palette_size));

     BOOST_FOREACH(const ModelInfo& model_info, model_infos) {
         cvp_Palette * palette =
                 cvp_Palette::load_from_file(model_info.m_FileName);
         BOOST_CHECK_EQUAL(palette->data().size(), model_info.m_DataSize);
         BOOST_CHECK_EQUAL(palette->type(), model_info.m_ModelType);
         for (unsigned i = 0; i < palette->data().size(); ++i) {
             BOOST_CHECK_CLOSE(palette->data()[i], static_cast<float>(i + 1),
                     1.0e-5);
         }
     }

     model_infos.clear();

     palette_type = cvp_Palette::FCM_CONTINUOUS;
     palette_size = 12;
     model_infos.push_back(
         ModelInfo(data_path + "sample_prior_continuous_2_64bit.data",
             palette_type, palette_size));

     palette_type = cvp_Palette::FCM_DISCRETE;
     bins_num = 8;
     palette_size = bins_num * bins_num * bins_num;
     model_infos.push_back(
         ModelInfo(data_path + "sample_prior_discrete_8_64bit.data",
             palette_type, palette_size));

     BOOST_FOREACH(const ModelInfo& model_info, model_infos) {
         cvp_Palette * palette =
                 cvp_Palette::load_from_file(model_info.m_FileName);
         const unsigned data_size = palette->data().size();
         BOOST_CHECK_EQUAL(data_size, model_info.m_DataSize);
         BOOST_CHECK_EQUAL(palette->type(), model_info.m_ModelType);
         for (unsigned i = 0; i < data_size; ++i) {
             BOOST_CHECK_CLOSE(palette->data()[i], static_cast<float>(i + 1.2),
                     1.0e-5);
         }
         if (palette_size == cvp_Palette::FCM_DISCRETE) {
             DiscreteColorPrecompute * precompute =
                 static_cast<DiscreteColorPrecompute*>
                 (palette->precomputed_data());
             BOOST_CHECK_EQUAL(precompute->binsNumPerDimension, 8);
             BOOST_CHECK_CLOSE(precompute->logBinVolume, -log(data_size), 1e-5);
             for (unsigned i = 0; i < data_size; ++i) {
                 BOOST_CHECK_CLOSE(precompute->digammaDistribution[i],
                         boost::math::digamma(i + 1.2),
                         1.0e-5);
             }
             const cvp_Palette::DataType sum_intensities =
                 accumulate(palette->data().begin(), palette->data().end(), 0.0);
             cvp_Palette::DataType digamma_value =
                     boost::math::digamma(sum_intensities);
             BOOST_CHECK_CLOSE(digamma_value + precompute->logBinVolume,
                     precompute->logConstantUpdate, 1e-5);
             BOOST_CHECK_CLOSE(-log(sum_intensities) - precompute->logBinVolume,
                     precompute->logConstantMarginalLikelihood, 1e-5);
         }
     }
}

BOOST_AUTO_TEST_CASE( test_palette_load ) {

    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    ModelInfoList model_infos;
    cvp_Palette::Type palette_type;
    unsigned palette_size;
    unsigned bins_num;

    palette_type = cvp_Palette::FCM_DISCRETE;
    bins_num = 16;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_16_64bit.data",
                    palette_type, palette_size));

    bins_num = 8;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_8_64bit.data",
                    palette_type, palette_size));

    bins_num = 4;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_4_64bit.data",
                    palette_type, palette_size));

    palette_type = cvp_Palette::FCM_CONTINUOUS;
    palette_size = 12;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_continuous_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_continuous_64bit.data",
                    palette_type, palette_size));

    BOOST_FOREACH(const ModelInfo& model_info, model_infos) {
        cvp_Palette * palette =
                cvp_Palette::load_from_file(model_info.m_FileName);
        BOOST_CHECK_EQUAL(palette->data().size(), model_info.m_DataSize);
        BOOST_CHECK_EQUAL(palette->type(), model_info.m_ModelType);
    }

}

BOOST_AUTO_TEST_CASE( test_palette_save ) {
    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    ModelInfoList model_infos;
    cvp_Palette::Type palette_type;
    unsigned palette_size;
    unsigned bins_num;

    palette_type = cvp_Palette::FCM_DISCRETE;
    bins_num = 16;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_16_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_16_64bit.data",
                    palette_type, palette_size));

    bins_num = 8;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_8_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_8_64bit.data",
                    palette_type, palette_size));

    bins_num = 4;
    palette_size = bins_num * bins_num * bins_num;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_background_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_clothes_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_discrete_4_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_discrete_4_64bit.data",
                    palette_type, palette_size));

    palette_type = cvp_Palette::FCM_CONTINUOUS;
    palette_size = 12;
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_hair_continuous_64bit.data",
                    palette_type, palette_size));
    model_infos.push_back(
            ModelInfo(data_path + "color_prior_skin_continuous_64bit.data",
                    palette_type, palette_size));

    BOOST_FOREACH(const ModelInfo& model_info, model_infos) {
        cvp_Palette * palette =
                cvp_Palette::load_from_file(model_info.m_FileName);

        const string outfilepath = data_path + "/colour_model_saved_1.data";
        path p (outfilepath);
        BOOST_REQUIRE(!exists(p));
        palette->save_to_file(outfilepath);
        cvp_Palette * palette2 =
                cvp_Palette::load_from_file(outfilepath);
        BOOST_CHECK_EQUAL(palette->type(), palette2->type());
        BOOST_CHECK_EQUAL(palette->data().size(), palette2->data().size());
        if (palette->data().size() == palette2->data().size()) {
            for (unsigned i = 0; i < palette->data().size(); ++i) {
                BOOST_CHECK_CLOSE(palette->data()[i], palette2->data()[i], 1e-5);
            }
        }
        remove(outfilepath);
    }
}

BOOST_AUTO_TEST_CASE( test_palette_assign ) {
    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    cvp_Palette * palette_cont1 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_1_64bit.data");
    cvp_Palette * palette_cont2 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_2_64bit.data");
    cvp_Palette * palette_discr1 = cvp_Palette::load_from_file(
            data_path + "sample_prior_discrete_4_64bit.data");
    cvp_Palette * palette_discr2 = cvp_Palette::load_from_file(
            data_path + "sample_prior_discrete_8_64bit.data");

    *palette_cont1 = *palette_cont2;
    BOOST_CHECK(palettes_equal(*palette_cont1, *palette_cont2));
    BOOST_CHECK(palettes_equal(*palette_cont2, *palette_cont1));

    *palette_discr1 = *palette_discr2;
    BOOST_CHECK(palettes_equal(*palette_discr1, *palette_discr2));
    BOOST_CHECK(palettes_equal(*palette_discr2, *palette_discr1));

    *palette_cont1 = *palette_discr1;
    BOOST_CHECK(palettes_equal(*palette_cont1, *palette_discr1));
    BOOST_CHECK(palettes_equal(*palette_discr1, *palette_cont1));

    *palette_discr2 = *palette_cont2;
    BOOST_CHECK(palettes_equal(*palette_discr2, *palette_cont2));
    BOOST_CHECK(palettes_equal(*palette_cont2, *palette_discr2));
}

BOOST_AUTO_TEST_CASE( test_palette_copyconstruct ) {
    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    cvp_Palette * palette_cont1 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_1_64bit.data");
    cvp_Palette * palette_cont2 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_2_64bit.data");
    cvp_Palette * palette_discr1 = cvp_Palette::load_from_file(
            data_path + "sample_prior_discrete_4_64bit.data");
    cvp_Palette * palette_discr2 = cvp_Palette::load_from_file(
            data_path + "sample_prior_discrete_8_64bit.data");

    cvp_Palette palette_copy1(*palette_cont1);
    BOOST_CHECK(palettes_equal(*palette_cont1, palette_copy1));
    cvp_Palette palette_copy2(*palette_cont2);
    BOOST_CHECK(palettes_equal(*palette_cont2, palette_copy2));
    cvp_Palette palette_copy3(*palette_discr1);
    BOOST_CHECK(palettes_equal(*palette_discr1, palette_copy3));
    cvp_Palette palette_copy4(*palette_discr2);
    BOOST_CHECK(palettes_equal(*palette_discr2, palette_copy4));

    cvp_Palette palette_copy5(palette_copy1);
    BOOST_CHECK(palettes_equal(palette_copy1, palette_copy5));
    cvp_Palette palette_copy6(palette_copy4);
    BOOST_CHECK(palettes_equal(palette_copy4, palette_copy6));
}

BOOST_AUTO_TEST_CASE( test_palette_equality ) {
    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    cvp_Palette * palette_cont1 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_1_64bit.data");
    cvp_Palette * palette_cont2 = cvp_Palette::load_from_file(
            data_path + "sample_prior_continuous_2_64bit.data");
    cvp_Palette palette_copy1(*palette_cont1);
    BOOST_CHECK(palettes_equal(*palette_cont1, palette_copy1));
    BOOST_CHECK(palettes_header_equal(*palette_cont1, palette_copy1));

    BOOST_CHECK(!palettes_equal(*palette_cont1, *palette_cont2));
    BOOST_CHECK(!palettes_header_equal(*palette_cont1, *palette_cont2));
}

BOOST_AUTO_TEST_SUITE_END()
