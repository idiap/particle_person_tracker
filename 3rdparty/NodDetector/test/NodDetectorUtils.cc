/**
 * @file test/NodDetectorUtils.cc
 * @date 14 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Test nod detector interpolation utilities
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE NodDetectorDetectionTests
#define BOOST_TEST_MAIN

//#define DEBUG_TRACE

// SYSTEM INCLUDES
#include <iostream>                            // STL I/O
#include <fstream>                             // STL file I/O
#include <stdexcept>                           // STL runtime_error
#include <cmath>                               // STL exp
#include <boost/test/included/unit_test.hpp>   // boost unit testing
#include <boost/foreach.hpp>                   // boost foreach loop
#include <boost/lambda/lambda.hpp>             // boost lambda exprs

// PROJECT INCLUDES
#include <noddetector/nd_Utils.h>              // nod detector utils

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

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_noddetector_utils )

BOOST_AUTO_TEST_CASE( interpolate_1 ) {

    const double prev_ts = 0;

    double ts_raw[] =  { 2.0,  3.0,  5.0,  6.0,  8.0,
                         9.0, 11.0, 12.0, 14.0, 15.0 };
    const vector<double> ts(ts_raw, ts_raw + 10);

    double vx_raw[] =  { 2.0, -1.0,  1.0, -2.0,  2.0,
                        -1.0,  1.0, -2.0,  2.0, -1.0 };
    double vy_raw[] =  { 2.0, -1.0,  1.0, -2.0,  2.0,
                        -1.0,  1.0, -2.0,  2.0, -1.0 };
    const vector<double> vx(vx_raw, vx_raw + 10);
    const vector<double> vy(vy_raw, vy_raw + 10);

    double ts_new_raw[] =  { 1.5,  3.0,  4.5,  6.0,  7.5,
                             9.0, 10.5, 12.0, 13.5, 15.0 };
    const vector<double> ts_new(ts_new_raw, ts_new_raw + 10);

    vector<double> vx_new(vx);


    ofstream fout("interp_dump.txt");
    ostream_iterator<double> out_it(fout, " ");

    interpolate_motion(prev_ts, ts, vx, ts_new, vx_new, ND_INTERP_LINEAR);
    copy(vx_new.begin(), vx_new.end(), out_it);
    fout << endl;
    interpolate_motion(prev_ts, ts, vx, ts_new, vx_new, ND_INTERP_POLYNOMIAL);
    copy(vx_new.begin(), vx_new.end(), out_it);
    fout << endl;
    interpolate_motion(prev_ts, ts, vx, ts_new, vx_new, ND_INTERP_CSPLINE);
    copy(vx_new.begin(), vx_new.end(), out_it);
    fout << endl;
}

BOOST_AUTO_TEST_SUITE_END()
