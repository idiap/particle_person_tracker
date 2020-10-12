/**
 * @file cxx/image_processing/test/RoiWindow.cc
 * @date 28 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Unit tests for ip_RoiWindow class and related operations
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE RoiWindowTests
#define BOOST_TEST_MAIN

// SYSTEM INCLUDES
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <image_processing/ip_Exceptions.h>
#include <image_processing/ip_RoiWindow.h>

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

/////////////////////////////// SUITES ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_roi_window_operations )

BOOST_AUTO_TEST_CASE( test_area ) {

    ip_RoiWindow roi1;
    roi1.m_iFirstColumn = 0;
    roi1.m_iFirstRow = 0;
    roi1.m_iWidth = 0;
    roi1.m_iHeight = 0;
    int area_roi1 = area(roi1);
    BOOST_CHECK_EQUAL(area_roi1, roi1.m_iWidth * roi1.m_iHeight);

    ip_RoiWindow roi2;
    roi2.m_iFirstColumn = 0;
    roi2.m_iFirstRow = 0;
    roi2.m_iWidth = 10;
    roi2.m_iHeight = 10;
    int area_roi2 = area(roi2);
    BOOST_CHECK_EQUAL(area_roi2, roi2.m_iWidth * roi2.m_iHeight);

    ip_RoiWindow roi3;
    roi3.m_iFirstColumn = 10;
    roi3.m_iFirstRow = 10;
    roi3.m_iWidth = 5;
    roi3.m_iHeight = 5;
    int area_roi3 = area(roi3);
    BOOST_CHECK_EQUAL(area_roi3, roi3.m_iWidth * roi3.m_iHeight);

    ip_RoiWindow roi4;
    roi4.m_iFirstColumn = 30;
    roi4.m_iFirstRow = 10;
    roi4.m_iWidth = 1;
    roi4.m_iHeight = 1;
    int area_roi4 = area(roi4);
    BOOST_CHECK_EQUAL(area_roi4, roi4.m_iWidth * roi4.m_iHeight);
}

BOOST_AUTO_TEST_CASE( test_intersection_area ) {

    ip_RoiWindow roi_global;
    roi_global.m_iFirstColumn = 0;
    roi_global.m_iFirstRow = 0;
    roi_global.m_iWidth = 100;
    roi_global.m_iHeight = 100;

    ip_RoiWindow roi1;
    roi1.m_iFirstColumn = 0;
    roi1.m_iFirstRow = 0;
    roi1.m_iWidth = 100;
    roi1.m_iHeight = 100;
    int area_11 = intersection_area(roi_global, roi1);
    BOOST_CHECK_EQUAL(area_11, roi1.m_iWidth * roi1.m_iHeight);
    int area_12 = intersection_area(roi1, roi_global);
    BOOST_CHECK_EQUAL(area_12, roi1.m_iWidth * roi1.m_iHeight);

    ip_RoiWindow roi2;
    roi2.m_iFirstColumn = 10;
    roi2.m_iFirstRow = 10;
    roi2.m_iWidth = 10;
    roi2.m_iHeight = 10;
    int area_21 = intersection_area(roi_global, roi2);
    BOOST_CHECK_EQUAL(area_21, roi2.m_iWidth * roi2.m_iHeight);
    int area_22 = intersection_area(roi2, roi_global);
    BOOST_CHECK_EQUAL(area_22, roi2.m_iWidth * roi2.m_iHeight);

    ip_RoiWindow roi3;
    roi3.m_iFirstColumn = -5;
    roi3.m_iFirstRow = -5;
    roi3.m_iWidth = 10;
    roi3.m_iHeight = 10;
    int area_31 = intersection_area(roi_global, roi3);
    BOOST_CHECK_EQUAL(area_31, 25);
    int area_32 = intersection_area(roi3, roi_global);
    BOOST_CHECK_EQUAL(area_32, 25);

    ip_RoiWindow roi4;
    roi4.m_iFirstColumn = 10;
    roi4.m_iFirstRow = -5;
    roi4.m_iWidth = 10;
    roi4.m_iHeight = 10;
    int area_41 = intersection_area(roi_global, roi4);
    BOOST_CHECK_EQUAL(area_41, 50);
    int area_42 = intersection_area(roi4, roi_global);
    BOOST_CHECK_EQUAL(area_42, 50);

    ip_RoiWindow roi5;
    roi5.m_iFirstColumn = 95;
    roi5.m_iFirstRow = -5;
    roi5.m_iWidth = 10;
    roi5.m_iHeight = 10;
    int area_51 = intersection_area(roi_global, roi5);
    BOOST_CHECK_EQUAL(area_51, 25);
    int area_52 = intersection_area(roi5, roi_global);
    BOOST_CHECK_EQUAL(area_52, 25);

    ip_RoiWindow roi6;
    roi6.m_iFirstColumn = 95;
    roi6.m_iFirstRow = 10;
    roi6.m_iWidth = 10;
    roi6.m_iHeight = 10;
    int area_61 = intersection_area(roi_global, roi6);
    BOOST_CHECK_EQUAL(area_61, 50);
    int area_62 = intersection_area(roi6, roi_global);
    BOOST_CHECK_EQUAL(area_62, 50);

    ip_RoiWindow roi7;
    roi7.m_iFirstColumn = 95;
    roi7.m_iFirstRow = 95;
    roi7.m_iWidth = 10;
    roi7.m_iHeight = 10;
    int area_71 = intersection_area(roi_global, roi7);
    BOOST_CHECK_EQUAL(area_71, 25);
    int area_72 = intersection_area(roi7, roi_global);
    BOOST_CHECK_EQUAL(area_72, 25);

    ip_RoiWindow roi8;
    roi8.m_iFirstColumn = 10;
    roi8.m_iFirstRow = 95;
    roi8.m_iWidth = 10;
    roi8.m_iHeight = 10;
    int area_81 = intersection_area(roi_global, roi8);
    BOOST_CHECK_EQUAL(area_81, 50);
    int area_82 = intersection_area(roi8, roi_global);
    BOOST_CHECK_EQUAL(area_82, 50);

    ip_RoiWindow roi9;
    roi9.m_iFirstColumn = -5;
    roi9.m_iFirstRow = 95;
    roi9.m_iWidth = 10;
    roi9.m_iHeight = 10;
    int area_91 = intersection_area(roi_global, roi9);
    BOOST_CHECK_EQUAL(area_91, 25);
    int area_92 = intersection_area(roi9, roi_global);
    BOOST_CHECK_EQUAL(area_92, 25);

    ip_RoiWindow roi10;
    roi10.m_iFirstColumn = -5;
    roi10.m_iFirstRow = 10;
    roi10.m_iWidth = 10;
    roi10.m_iHeight = 10;
    int area_101 = intersection_area(roi_global, roi10);
    BOOST_CHECK_EQUAL(area_101, 50);
    int area_102 = intersection_area(roi10, roi_global);
    BOOST_CHECK_EQUAL(area_102, 50);

    ip_RoiWindow roi11;
    roi11.m_iFirstColumn = -10;
    roi11.m_iFirstRow = -10;
    roi11.m_iWidth = 10;
    roi11.m_iHeight = 10;
    int area_111 = intersection_area(roi_global, roi11);
    BOOST_CHECK_EQUAL(area_111, 0);
    int area_112 = intersection_area(roi11, roi_global);
    BOOST_CHECK_EQUAL(area_112, 0);

    ip_RoiWindow roi12;
    roi12.m_iFirstColumn = -10;
    roi12.m_iFirstRow = -10;
    roi12.m_iWidth = 11;
    roi12.m_iHeight = 11;
    int area_121 = intersection_area(roi_global, roi12);
    BOOST_CHECK_EQUAL(area_121, 1);
    int area_122 = intersection_area(roi12, roi_global);
    BOOST_CHECK_EQUAL(area_122, 1);
}

BOOST_AUTO_TEST_CASE( test_intersect ) {
    ip_RoiWindow roi_global;
    roi_global.m_iFirstColumn = 0;
    roi_global.m_iFirstRow = 0;
    roi_global.m_iWidth = 100;
    roi_global.m_iHeight = 100;
    ip_RoiWindow roi_intersect;
    bool roi_intersect_flag;

    ip_RoiWindow roi1;
    roi1.m_iFirstColumn = 0;
    roi1.m_iFirstRow = 0;
    roi1.m_iWidth = 100;
    roi1.m_iHeight = 100;
    roi_intersect_flag = intersect(roi_global, roi1, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 100);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 100);
    roi_intersect_flag = intersect(roi1, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 100);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 100);

    ip_RoiWindow roi2;
    roi2.m_iFirstColumn = 10;
    roi2.m_iFirstRow = 10;
    roi2.m_iWidth = 10;
    roi2.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi2, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);
    roi_intersect_flag = intersect(roi2, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);

    ip_RoiWindow roi3;
    roi3.m_iFirstColumn = -5;
    roi3.m_iFirstRow = -5;
    roi3.m_iWidth = 10;
    roi3.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi3, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi3, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi4;
    roi4.m_iFirstColumn = 10;
    roi4.m_iFirstRow = -5;
    roi4.m_iWidth = 10;
    roi4.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi4, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi4, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi5;
    roi5.m_iFirstColumn = 95;
    roi5.m_iFirstRow = -5;
    roi5.m_iWidth = 10;
    roi5.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi5, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi5, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi6;
    roi6.m_iFirstColumn = 95;
    roi6.m_iFirstRow = 10;
    roi6.m_iWidth = 10;
    roi6.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi6, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);
    roi_intersect_flag = intersect(roi6, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);

    ip_RoiWindow roi7;
    roi7.m_iFirstColumn = 95;
    roi7.m_iFirstRow = 95;
    roi7.m_iWidth = 10;
    roi7.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi7, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi7, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi8;
    roi8.m_iFirstColumn = 10;
    roi8.m_iFirstRow = 95;
    roi8.m_iWidth = 10;
    roi8.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi8, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi8, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi9;
    roi9.m_iFirstColumn = -5;
    roi9.m_iFirstRow = 95;
    roi9.m_iWidth = 10;
    roi9.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi9, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);
    roi_intersect_flag = intersect(roi9, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 95);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 5);

    ip_RoiWindow roi10;
    roi10.m_iFirstColumn = -5;
    roi10.m_iFirstRow = 10;
    roi10.m_iWidth = 10;
    roi10.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi10, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);
    roi_intersect_flag = intersect(roi10, roi_global, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 10);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 5);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 10);

    ip_RoiWindow roi11;
    roi11.m_iFirstColumn = -10;
    roi11.m_iFirstRow = -10;
    roi11.m_iWidth = 10;
    roi11.m_iHeight = 10;
    roi_intersect_flag = intersect(roi_global, roi11, roi_intersect);
    BOOST_CHECK(!roi_intersect_flag);
    roi_intersect_flag = intersect(roi11, roi_global, roi_intersect);
    BOOST_CHECK(!roi_intersect_flag);

    ip_RoiWindow roi12;
    roi12.m_iFirstColumn = -10;
    roi12.m_iFirstRow = -10;
    roi12.m_iWidth = 11;
    roi12.m_iHeight = 11;
    roi_intersect_flag = intersect(roi_global, roi12, roi_intersect);
    BOOST_CHECK(roi_intersect_flag);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstColumn, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iFirstRow, 0);
    BOOST_CHECK_EQUAL(roi_intersect.m_iWidth, 1);
    BOOST_CHECK_EQUAL(roi_intersect.m_iHeight, 1);
}

BOOST_AUTO_TEST_CASE( test_contains ) {
    ip_RoiWindow roi_global;
    roi_global.m_iFirstColumn = 0;
    roi_global.m_iFirstRow = 0;
    roi_global.m_iWidth = 100;
    roi_global.m_iHeight = 100;
    bool contains_flag;

    contains_flag = contains(roi_global, 0, 0);
    BOOST_CHECK(contains_flag);
    contains_flag = contains(roi_global, 10, 10);
    BOOST_CHECK(contains_flag);
    contains_flag = contains(roi_global, 99, 99);
    BOOST_CHECK(contains_flag);
    contains_flag = contains(roi_global, 100, 100);
    BOOST_CHECK(!contains_flag);
    contains_flag = contains(roi_global, 0, -1);
    BOOST_CHECK(!contains_flag);

}

BOOST_AUTO_TEST_CASE( test_from_CvRect ) {
    ip_RoiWindow roi;
    roi.m_iFirstColumn = 0;
    roi.m_iFirstRow = 1;
    roi.m_iWidth = 100;
    roi.m_iHeight = 50;
    CvRect rect = cvRect(0, 1, 100, 50);
    ip_RoiWindow roi2 = ip_RoiWindow::from_CvRect(rect);
    BOOST_CHECK(roi == roi2);
}

BOOST_AUTO_TEST_CASE( test_to_CvRect ) {
    ip_RoiWindow roi;
    roi.m_iFirstColumn = 0;
    roi.m_iFirstRow = 1;
    roi.m_iWidth = 100;
    roi.m_iHeight = 50;
    CvRect rect = ip_RoiWindow::to_CvRect(roi);
    CvRect rect2 = cvRect(0, 1, 100, 50);
    BOOST_CHECK_EQUAL(rect.x, rect2.x);
    BOOST_CHECK_EQUAL(rect.y, rect2.y);
    BOOST_CHECK_EQUAL(rect.width, rect2.width);
    BOOST_CHECK_EQUAL(rect.height, rect2.height);
}

BOOST_AUTO_TEST_CASE( test_scale1 ) {
    ip_RoiWindow roi;
    roi.m_iFirstColumn = 0;
    roi.m_iFirstRow = 1;
    roi.m_iWidth = 100;
    roi.m_iHeight = 50;

    ip_RoiWindow roi_scaled1 = scale(roi, 1.0f);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iFirstColumn, roi.m_iFirstColumn);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iFirstRow, roi.m_iFirstRow);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iWidth, roi.m_iWidth);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iHeight, roi.m_iHeight);

    ip_RoiWindow roi_scaled2 = scale(roi, 2.0f);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstColumn, -50);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstRow, -24);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iWidth, 200);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iHeight, 100);
    roi_scaled2 = scale(roi_scaled2, 0.5f);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstColumn, roi.m_iFirstColumn);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstRow, roi.m_iFirstRow);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iWidth, roi.m_iWidth);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iHeight, roi.m_iHeight);

    ip_RoiWindow roi_scaled3 = scale(roi, 0.5f);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iFirstColumn, 25);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iFirstRow, 14);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iWidth, 50);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iHeight, 25);
    roi_scaled3 = scale(roi_scaled3, 2.0f);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iFirstColumn, roi.m_iFirstColumn);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iFirstRow, roi.m_iFirstRow);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iWidth, roi.m_iWidth);
    BOOST_CHECK_EQUAL(roi_scaled3.m_iHeight, roi.m_iHeight);
}

BOOST_AUTO_TEST_CASE( test_scale2 ) {
    ip_RoiWindow roi;
    roi.m_iFirstColumn = 0;
    roi.m_iFirstRow = 0;
    roi.m_iWidth = 3;
    roi.m_iHeight = 3;

    ip_RoiWindow roi_scaled1 = scale(roi, 1.0f);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iFirstColumn, roi.m_iFirstColumn);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iFirstRow, roi.m_iFirstRow);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iWidth, roi.m_iWidth);
    BOOST_CHECK_EQUAL(roi_scaled1.m_iHeight, roi.m_iHeight);

    ip_RoiWindow roi_scaled2 = scale(roi, 2.0f);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstColumn, -2);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstRow, -2);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iWidth, 6);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iHeight, 6);
    roi_scaled2 = scale(roi_scaled2, 0.5f);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstColumn, roi.m_iFirstColumn);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iFirstRow, roi.m_iFirstRow);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iWidth, roi.m_iWidth);
    BOOST_CHECK_EQUAL(roi_scaled2.m_iHeight, roi.m_iHeight);
}

BOOST_AUTO_TEST_SUITE_END()
