/**
 * @file cxx/opencvplus/test/PaletteBundle.cc
 * @date 20 September 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief PaletteBundle unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PaletteBundleTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <cstdlib>                           // for getenv
#include <list>                              // STL list
#include <iostream>
#include <fstream>
#include <boost/test/unit_test.hpp>

// LOCAL INCLUDES
#include <opencvplus/cvp_Exceptions.h>
#include <opencvplus/cvp_PaletteBundle.h>

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

/////////////////////////// LOCAL DECLARATIONS ///////////////////////////////

// variable name that contains a path to test data directory
static const string DATA_DIR_ENVIRONMENT_VAR = "TTRACK_TESTDATA_DIR";

struct ModelInfo {
    ModelInfo(const string& file_name, cvp_Palette::Type model_type, unsigned data_size) :
        m_FileName(file_name), m_ModelType(model_type), m_DataSize(data_size) {
    }
    string m_FileName;
    cvp_Palette::Type m_ModelType;
    unsigned m_DataSize;
};

typedef list<ModelInfo> ModelInfoList;

void check_bundles_equal(const cvp_PaletteBundle& bundle1,
        const cvp_PaletteBundle& bundle2);

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_palette_undle )

BOOST_AUTO_TEST_CASE( test_palette_bundle_copy_construct ) {

    // get data directory
    char * data_dir = getenv(DATA_DIR_ENVIRONMENT_VAR.c_str());
    if (!data_dir) {
        BOOST_FAIL("Data directory environment variable is not set up "
                "correctly - verify project configuration!");
    }
    string data_path;
    data_path.append(data_dir);
    data_path.append("/");

    list<string> model_infos;
    model_infos.push_back(
            data_path + "color_prior_background_discrete_16_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_clothes_discrete_16_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_hair_discrete_16_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_skin_discrete_16_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_background_discrete_8_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_clothes_discrete_8_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_hair_discrete_8_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_skin_discrete_8_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_background_discrete_4_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_clothes_discrete_4_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_hair_discrete_4_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_skin_discrete_4_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_hair_continuous_64bit.data");
    model_infos.push_back(
            data_path + "color_prior_skin_continuous_64bit.data");

    vector<string> palette_infos(model_infos.begin(), model_infos.end());
    cvp_PaletteBundle * palette_bundle =
            cvp_PaletteBundle::load_from_files(palette_infos);

    {
        cvp_PaletteBundle palette_bundle_copy(*palette_bundle);
        check_bundles_equal(*palette_bundle, palette_bundle_copy);
    }

    cvp_PaletteBundle * palette_bundle2 =
            cvp_PaletteBundle::load_from_files(palette_infos);
    check_bundles_equal(*palette_bundle, *palette_bundle2);

    delete palette_bundle2;
    delete palette_bundle;

}

BOOST_AUTO_TEST_SUITE_END()

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

void check_bundles_equal(const cvp_PaletteBundle& bundle1,
        const cvp_PaletteBundle& bundle2) {

    BOOST_CHECK_EQUAL(bundle1.size(), bundle2.size());
    const unsigned data_size = bundle1.size();
    for (unsigned i = 0; i < data_size; ++i) {
        BOOST_CHECK(palettes_equal(*bundle1.palette(i), *bundle2.palette(i)));
    }

} // check_bundles_equal
