/**
 * @file cxx/image_processing/test/TrackingMemoryImage.cc
 * @date 28 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Unit tests for ip_TrackingMemoryIamge class and related operations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TrackingMemoryImageTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>
#include <list>

// PROJECT INCLUDES
#include <image_processing/ip_Exceptions.h>
#include <image_processing/ip_RoiWindow.h>
#include <image_processing/ip_SingleImageProvider.h>
#include <image_processing/ip_TrackingMemoryImage.h>

using namespace ImageProcessing;
using namespace std;

/////////////////////////////// FIXTURE //////////////////////////////////////

namespace ImageProcessing {
    struct ip_Logger {
        ip_Logger()   {
            const char* report_file_path = getenv("TTRACK_TESTREPORT_FILE");
              if (!report_file_path) {
                  std::cerr <<
                  "Environment variable $TTRACK_TESTREPORT_FILE is not set."
                  " Have you setup your working environment correctly?" <<
                  std::endl;
                  throw ip_Exception();
              }
              m_LogStream.open(report_file_path);
              boost::unit_test::unit_test_log.set_stream(m_LogStream);
        }
        ~ip_Logger()  {
            boost::unit_test::unit_test_log.set_stream( std::cout );
            m_LogStream << "</TestLog>";
            m_LogStream.close();
        }

        private:
        std::ofstream m_LogStream;
    };
}

BOOST_GLOBAL_FIXTURE(ip_Logger)

////////////////////////// LOCAL DECLARATIONS ////////////////////////////////

static void initialize_source_image(IplImage * image);

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_memory_image_operations )

BOOST_AUTO_TEST_CASE( test_area ) {
    const int width = 10;
    const int height = 10;
    const float factor = 0.01;
    const float precision = 1e-5;

    IplImage * image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    initialize_source_image(image);
    ip_ImageProvider * data_provider = new ip_SingleImageProvider(image);

    ip_TrackingMemoryImage * tmemory = new ip_TrackingMemoryImage(
            data_provider, factor);

    list<ip_RoiWindow> rois;

    // test first update
    ip_RoiWindow roi1 = ip_RoiWindow::from_CvRect(cvRect(2, 2, 2, 2));
    rois.push_back(roi1);

    tmemory->update(rois);

    float val;
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (((row < roi1.m_iFirstRow) ||
                 (row >= roi1.m_iFirstRow + roi1.m_iHeight)) &&
                ((col < roi1.m_iFirstColumn) ||
                 (col >= roi1.m_iFirstColumn + roi1.m_iWidth))) {
                val = tmemory->value(col, row);
                BOOST_CHECK_CLOSE(val, 0.0f, precision);
            }
        }
    }
    for (int row = roi1.m_iFirstRow; row < roi1.m_iFirstRow + roi1.m_iHeight; ++row) {
        for (int col = roi1.m_iFirstColumn; col < roi1.m_iFirstColumn + roi1.m_iWidth; ++col) {
            val = tmemory->value(col, row);
            BOOST_CHECK_CLOSE(val, factor, precision);
        }
    }

    // test second update
    rois.clear();
    ip_RoiWindow roi2 = ip_RoiWindow::from_CvRect(cvRect(3, 3, 2, 2));
    rois.push_back(roi2);

    tmemory->update(rois);

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (((row < 2) || (row > 4)) && ((col < 2) || (col > 4))) {
                val = tmemory->value(col, row);
                BOOST_CHECK_CLOSE(val, 0.0f, precision);
            }
        }
    }
    val = tmemory->value(2, 2);
    BOOST_CHECK_CLOSE(val, factor * (1.0f - factor), precision);
    val = tmemory->value(3, 2);
    BOOST_CHECK_CLOSE(val, factor * (1.0f - factor), precision);
    val = tmemory->value(2, 3);
    BOOST_CHECK_CLOSE(val, factor * (1.0f - factor), precision);
    val = tmemory->value(4, 2);
    BOOST_CHECK_CLOSE(val, 0.0f, precision);
    val = tmemory->value(2, 4);
    BOOST_CHECK_CLOSE(val, 0.0f, precision);
    val = tmemory->value(3, 4);
    BOOST_CHECK_CLOSE(val, factor, precision);
    val = tmemory->value(4, 4);
    BOOST_CHECK_CLOSE(val, factor, precision);
    val = tmemory->value(4, 3);
    BOOST_CHECK_CLOSE(val, factor, precision);
    val = tmemory->value(3, 3);
    BOOST_CHECK_CLOSE(val, factor * (2 - factor), precision);

    delete data_provider;
    cvReleaseImage(&image);
}

BOOST_AUTO_TEST_SUITE_END()

////////////////////////// LOCAL DEFINITIONS /////////////////////////////////

/* static */ void initialize_source_image(IplImage * image) {
    cvZero(image);
}
