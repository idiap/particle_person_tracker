/**
 * @file cxx/opencvplus/test/FaceDetectorStatistics.cc
 * @date 21 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief FaceDetectorStatistics unit tests
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE FaceDetectorStatisticsTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <opencvplus/cvp_Exceptions.h>
#include <opencvplus/cvp_FaceDetectorStatistics.h>

using namespace OpenCvPlus;
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

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_serialization )

BOOST_AUTO_TEST_CASE( test_fdstats_save_load_1 ) {

    // get data directory
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    cvp_FaceDetectorStatistics fdstats;

    {
        // fill values into fdstats
        double* pointers[] = {
                fdstats.m_MeanHeadPose.val,
                fdstats.m_StddevHeadPose.val,
                fdstats.m_MeanBBoxTransform.val,
                fdstats.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats.m_MeanHeadPose.val + 3,
                fdstats.m_StddevHeadPose.val + 3,
                fdstats.m_MeanBBoxTransform.val  +4,
                fdstats.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float val = 0.0f;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                *parr++ = val;
                val += 0.1;
            }
            val += 1.0;
        }
    }

    // save fdstats
    std::string outfilepath = data_dir + "/fdstats_saved_1.data";
    path p (outfilepath);
    BOOST_REQUIRE(!exists(p));
    cvp_FaceDetectorStatistics::save(fdstats, outfilepath);

    cvp_FaceDetectorStatistics fdstats_loaded =
            cvp_FaceDetectorStatistics::load(outfilepath);

    // check values in fdstats_loaded
    {
        double* pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val,
                fdstats_loaded.m_StddevHeadPose.val,
                fdstats_loaded.m_MeanBBoxTransform.val,
                fdstats_loaded.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val + 3,
                fdstats_loaded.m_StddevHeadPose.val + 3,
                fdstats_loaded.m_MeanBBoxTransform.val  +4,
                fdstats_loaded.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float val = 0.0f;
        float val_loaded;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                val_loaded = *parr++;
                BOOST_CHECK_CLOSE(val, val_loaded, 1e-5);
                val += 0.1;
            }
            val += 1.0;
        }
    }

    remove(outfilepath);

}

BOOST_AUTO_TEST_CASE( test_load_nonexisting ) {

    // get data directory
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    std::string filepath = data_dir + "/fdstats_xyz.data";
    cvp_FaceDetectorStatistics fdstats_loaded;
    BOOST_CHECK_THROW(
            fdstats_loaded = cvp_FaceDetectorStatistics::load(filepath),
            cvp_Exception);

}

BOOST_AUTO_TEST_CASE( test_fdstats_save_load_2 ) {

    // get data directory
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    cvp_FaceDetectorStatistics fdstats;

    float vals [] = {
            1.52752294, 3.37155963, 0.0,
            31.0272398, 29.35990711, 0.0,
            3.94311927, -12.52752294, 1.14674453, 1.46323025,
            35.42243721, 17.07457132, 0.20225817, 0.21279665
    };

    {
        // fill values into fdstats
        double* pointers[] = {
                fdstats.m_MeanHeadPose.val,
                fdstats.m_StddevHeadPose.val,
                fdstats.m_MeanBBoxTransform.val,
                fdstats.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats.m_MeanHeadPose.val + 3,
                fdstats.m_StddevHeadPose.val + 3,
                fdstats.m_MeanBBoxTransform.val  +4,
                fdstats.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                *parr++ = *val++;
            }
        }
    }

    // save fdstats
    std::string outfilepath = data_dir + "/fdstats_saved_2.data";
    path p (outfilepath);
    BOOST_REQUIRE(!exists(p));
    cvp_FaceDetectorStatistics::save(fdstats, outfilepath);

    cvp_FaceDetectorStatistics fdstats_loaded =
            cvp_FaceDetectorStatistics::load(outfilepath);

    // check values in fdstats_loaded
    {
        double* pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val,
                fdstats_loaded.m_StddevHeadPose.val,
                fdstats_loaded.m_MeanBBoxTransform.val,
                fdstats_loaded.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val + 3,
                fdstats_loaded.m_StddevHeadPose.val + 3,
                fdstats_loaded.m_MeanBBoxTransform.val  +4,
                fdstats_loaded.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        float val_expected, val_loaded;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                val_expected = *val++;
                val_loaded = *parr++;
                BOOST_CHECK_CLOSE(val_expected, val_loaded, 1e-5);
            }
        }
    }

    remove(outfilepath);

}

BOOST_AUTO_TEST_CASE( test_fdstats_save_load_3 ) {

    // get data directory
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    cvp_FaceDetectorStatistics fdstats;

    float vals [] = {
            40.1431127, 7.27191413, 0.0,
            36.72806687, 30.26462505, 0.0,
            19.19677996, -8.83184258, 1.08606261, 1.27929867,
            20.98642351, 17.18884863, 0.2419421, 0.26629473
    };

    {
        // fill values into fdstats
        double* pointers[] = {
                fdstats.m_MeanHeadPose.val,
                fdstats.m_StddevHeadPose.val,
                fdstats.m_MeanBBoxTransform.val,
                fdstats.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats.m_MeanHeadPose.val + 3,
                fdstats.m_StddevHeadPose.val + 3,
                fdstats.m_MeanBBoxTransform.val  +4,
                fdstats.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                *parr++ = *val++;
            }
        }
    }

    // save fdstats
    std::string outfilepath = data_dir + "/fdstats_saved_3.data";
    path p (outfilepath);
    BOOST_REQUIRE(!exists(p));
    cvp_FaceDetectorStatistics::save(fdstats, outfilepath);

    cvp_FaceDetectorStatistics fdstats_loaded =
            cvp_FaceDetectorStatistics::load(outfilepath);

    // check values in fdstats_loaded
    {
        double* pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val,
                fdstats_loaded.m_StddevHeadPose.val,
                fdstats_loaded.m_MeanBBoxTransform.val,
                fdstats_loaded.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val + 3,
                fdstats_loaded.m_StddevHeadPose.val + 3,
                fdstats_loaded.m_MeanBBoxTransform.val  +4,
                fdstats_loaded.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        float val_expected, val_loaded;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                val_expected = *val++;
                val_loaded = *parr++;
                BOOST_CHECK_CLOSE(val_expected, val_loaded, 1e-5);
            }
        }
    }

    remove(outfilepath);

}

BOOST_AUTO_TEST_CASE( test_fdstats_save_load_4 ) {

    // get data directory
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw cvp_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    cvp_FaceDetectorStatistics fdstats;

    float vals [] = {
            -41.58158614, 5.71558797, 0.0,
            37.07860713, 28.95889117, 0.0,
            -16.99544211, -10.87967183, 1.06288141, 1.24800378,
            22.50268853, 18.02065352, 0.2770701, 0.30205692
    };

    {
        // fill values into fdstats
        double* pointers[] = {
                fdstats.m_MeanHeadPose.val,
                fdstats.m_StddevHeadPose.val,
                fdstats.m_MeanBBoxTransform.val,
                fdstats.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats.m_MeanHeadPose.val + 3,
                fdstats.m_StddevHeadPose.val + 3,
                fdstats.m_MeanBBoxTransform.val  +4,
                fdstats.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                *parr++ = *val++;
            }
        }
    }

    // save fdstats
    std::string outfilepath = data_dir + "/fdstats_saved_4.data";
    path p (outfilepath);
    BOOST_REQUIRE(!exists(p));
    cvp_FaceDetectorStatistics::save(fdstats, outfilepath);

    cvp_FaceDetectorStatistics fdstats_loaded =
            cvp_FaceDetectorStatistics::load(outfilepath);

    // check values in fdstats_loaded
    {
        double* pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val,
                fdstats_loaded.m_StddevHeadPose.val,
                fdstats_loaded.m_MeanBBoxTransform.val,
                fdstats_loaded.m_StddevBBoxTransform.val};
        double* end_pointers[] = {
                fdstats_loaded.m_MeanHeadPose.val + 3,
                fdstats_loaded.m_StddevHeadPose.val + 3,
                fdstats_loaded.m_MeanBBoxTransform.val  +4,
                fdstats_loaded.m_StddevBBoxTransform.val + 4};

        double**const pointers_begin = pointers;
        double**const pointers_end = pointers + 4;
        double**const end_pointers_begin = end_pointers;

        double** pointer = pointers_begin;
        double** end_pointer = end_pointers_begin;

        double * parr;
        double * end_parr;
        float * val = vals;
        float val_expected, val_loaded;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                val_expected = *val++;
                val_loaded = *parr++;
                BOOST_CHECK_CLOSE(val_expected, val_loaded, 1e-5);
            }
        }
    }

    remove(outfilepath);

}

BOOST_AUTO_TEST_SUITE_END()
