/**
 * @file test/NodDetectorDetection.cc
 * @date 13 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Test nod detector detections
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

static nd_PointPatternMotionData load_motion_data(const string& data_fname);
static bool extract_motion_data(const nd_PointPatternMotionData& raw_data,
        nd_PointPatternMotionData& extracted_data,
        unsigned offset);

//////////////////////////////// TESTS ///////////////////////////////////////

BOOST_AUTO_TEST_SUITE( test_noddetector_detect )

BOOST_AUTO_TEST_CASE( detect_nods_1 ) {
    static const unsigned MOTION_DATA_SIZE = 9157;
    static const string FILENAME_STR = "noddetector.data";
    static const string MOTION_DATA_STR = "motion3pts.txt";


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
    nd_NodDetectorCache * cache = nod_detector->create_cache();
    nd_PointPatternMotionData pattern_motion_data(
            nod_detector->point_pattern().m_Points.size(),
            nod_detector->motion_data_size());


    string motion_data_path;
    motion_data_path.append(data_dir);
    motion_data_path.append("/");
    motion_data_path.append(MOTION_DATA_STR);
    nd_PointPatternMotionData raw_file_data =
            load_motion_data(motion_data_path);

    BOOST_REQUIRE_EQUAL(raw_file_data.m_PointMotionDatas.size(), 3);
    BOOST_FOREACH(const nd_PointMotionData& pt_motion_data,
            raw_file_data.m_PointMotionDatas) {
        BOOST_CHECK_EQUAL(pt_motion_data.m_XMotionData.size(), MOTION_DATA_SIZE);
        BOOST_CHECK_EQUAL(pt_motion_data.m_YMotionData.size(), MOTION_DATA_SIZE);
    }

    ofstream fout("nd_dump.txt");
    unsigned offset = 0;
    while (extract_motion_data(raw_file_data, pattern_motion_data, offset++)) {
        fout << nod_detector->detect_nod(pattern_motion_data, *cache) << endl;
    }
}

BOOST_AUTO_TEST_SUITE_END()

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

nd_PointPatternMotionData load_motion_data(const string& data_fname) {

#ifdef DEBUG_TRACE
    ofstream fout("ddump.txt");
    fout << "Loading motion data from " << data_fname << endl;
#endif

    ifstream data_file_istream;
    data_file_istream.open(data_fname.c_str(), ios::in);
    if (!data_file_istream.is_open()) {
#ifdef DEBUG_TRACE
        fout << "Could not open file " << data_fname << " for reading!" <<  endl;
#endif
        return nd_PointPatternMotionData(0, 0);
    }

    stringbuf buffer;
    istream in_buffer(&buffer);

    stringbuf num_buffer;
    istream in_num_buffer(&num_buffer);

    char c;
    double d;

    typedef list<double> DataList;
    vector<DataList> ptdata(6);

    // first line is headers

    data_file_istream.get(buffer);
#ifdef DEBUG_TRACE
    unsigned line_count = 1;
    fout << "Read line " << line_count++
         << ", GCount = " << data_file_istream.gcount()
         << ", buffer=\"" << buffer.str() << "\""
         << ", in_buffer_good=\"" << in_buffer.good() << "\""
         << ", in_buffer_eof=\"" << in_buffer.eof() << "\""
         << ", in_buffer_fail=\"" << in_buffer.fail() << "\""
         << ", in_buffer_bad=\"" << in_buffer.bad() << "\""
         << endl;
#endif
    buffer.str("");
    data_file_istream.get(c);
    assert(c == '\n');

    while (data_file_istream.good()) {
        // get a line into the buffer
        data_file_istream.get(buffer);
#ifdef DEBUG_TRACE
        fout << "Read line " << line_count++
             << ", GCount = " << data_file_istream.gcount()
             << ", buffer=\"" << buffer.str() << "\""
             << ", in_buffer_good=\"" << in_buffer.good() << "\""
             << ", in_buffer_eof=\"" << in_buffer.eof() << "\""
             << ", in_buffer_fail=\"" << in_buffer.fail() << "\""
             << ", in_buffer_bad=\"" << in_buffer.bad() << "\""
             << endl;
#endif
        if (data_file_istream.gcount() > 0) {
            // get 5 numbers
            for (unsigned idx = 0; idx < 5; ++idx) {
                in_buffer.get(num_buffer, ',');
#ifdef DEBUG_TRACE
                fout << "    Token=\"" << num_buffer.str() << "\""
                     << "in_good=\"" << in_num_buffer.good() << "\""
                     << "in_eof=\"" << in_num_buffer.eof() << "\""
                     << "in_fail=\"" << in_num_buffer.fail() << "\""
                     << "in_bad=\"" << in_num_buffer.bad() << "\"";
#endif
                in_num_buffer >> d;
                in_num_buffer.clear();
#ifdef DEBUG_TRACE
                fout << ", d=" << d << endl;
#endif
                ptdata[idx].push_back(d);
                num_buffer.str("");
                in_buffer.get(c);
                assert(c == ',');
            }
            // get 6th number
            in_buffer.get(num_buffer);
            in_buffer.clear();
#ifdef DEBUG_TRACE
                fout << "    Token=\"" << num_buffer.str() << "\""
                     << "in_good=\"" << in_num_buffer.good() << "\""
                     << "in_eof=\"" << in_num_buffer.eof() << "\""
                     << "in_fail=\"" << in_num_buffer.fail() << "\""
                     << "in_bad=\"" << in_num_buffer.bad() << "\"";
#endif
            in_num_buffer >> d;
            in_num_buffer.clear();
#ifdef DEBUG_TRACE
                fout << ", d=" << d << endl;
#endif
            ptdata[5].push_back(d);
            num_buffer.str("");

            buffer.str("");
        }
        // process endline character
        data_file_istream.get(c);
        assert(c == '\n');
    }

    const unsigned motion_data_size = ptdata[0].size();
    nd_PointPatternMotionData motion_data(3, motion_data_size);

    typedef vector<nd_PointMotionData::RealType> * VecPtr;

    VecPtr motdata[] = {
            &motion_data.m_PointMotionDatas[0].m_XMotionData,
            &motion_data.m_PointMotionDatas[1].m_XMotionData,
            &motion_data.m_PointMotionDatas[2].m_XMotionData,
            &motion_data.m_PointMotionDatas[0].m_YMotionData,
            &motion_data.m_PointMotionDatas[1].m_YMotionData,
            &motion_data.m_PointMotionDatas[2].m_YMotionData
    };

    for (unsigned idx = 0; idx < 6; ++idx) {
        copy(ptdata[idx].begin(), ptdata[idx].end(), motdata[idx]->begin());
    }

    return motion_data;

} // load_motion_data

bool extract_motion_data(const nd_PointPatternMotionData& raw_data,
        nd_PointPatternMotionData& extracted_data,
        unsigned offset) {
    const unsigned raw_data_length =
            raw_data.m_PointMotionDatas[0].m_XMotionData.size();
    const unsigned extracted_data_length =
            extracted_data.m_PointMotionDatas[0].m_XMotionData.size();
    const unsigned pt_num = raw_data.m_PointMotionDatas.size();
    assert(pt_num == extracted_data.m_PointMotionDatas.size());

    if (offset + extracted_data_length > raw_data_length) {
        return false;
    }

    const nd_PointMotionData::RealType * raw_data_ptr;

    for (unsigned idx = 0; idx < pt_num; ++idx) {
        raw_data_ptr = &raw_data.m_PointMotionDatas[idx].m_XMotionData[0] +
                offset;
        copy(raw_data_ptr, raw_data_ptr + extracted_data_length,
            &extracted_data.m_PointMotionDatas[idx].m_XMotionData[0]);

        raw_data_ptr = &raw_data.m_PointMotionDatas[idx].m_YMotionData[0] +
                offset;
        copy(raw_data_ptr, raw_data_ptr + extracted_data_length,
            &extracted_data.m_PointMotionDatas[idx].m_YMotionData[0]);
    }

    return true;
} // extract_motion_data
