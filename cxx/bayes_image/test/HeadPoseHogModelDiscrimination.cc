/**
 * @file cxx/bayes_image/test/HeadPoseHogModelDiscrimination.cc
 * @date 04 February 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Unit tests for HeadPoseHogModel capacity for discrimination
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
#include <boost/version.hpp>                 // boost version
#include <boost/filesystem.hpp>
#include <boost/version.hpp>                           // boost version
#include <iostream>
#include <fstream>

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPoseDiscreteDomain.h>
#include <bayes_image/bicv_HogModelTrainer.h>
#include <bayes_image/bicv_Exceptions.h>

using namespace OpenCvPlus;
using namespace BICV;
using namespace ImageProcessing;
using namespace std;
using namespace boost::filesystem;

typedef bicv_HeadPoseHogModel::HeadPose HeadPose;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static void
parse_input_annotation_file(const string& input_file,
        vector<string>& example_image_files,
        vector<bicv_TrainingData>& example_annotations);

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

BOOST_AUTO_TEST_SUITE( test_model_Elisa )

BOOST_AUTO_TEST_CASE( test_distance_L2 ) {

    // get path to the data DIR that contains model file
    char* data = getenv("TTRACK_TESTDATA_DIR");
    if (!data){
        throw bicv_Exception( \
            "Environment variable $TTRACK_TESTDATA_DIR is not set." \
            " Have you setup your working environment correctly?");
    }
    std::string data_dir(data);

    std::string filepath = data_dir + "/hog_model_Elisa.xml";
    bicv_HeadPoseHogModel * model = bicv_HeadPoseHogModel::load(filepath);

    std::string database_fpath_str =
            "/scratch/vkhalidov/data/head_pose/prima_headpose/"
            "PointingDB_face_annotation.csv";

    path database_fpath(database_fpath_str);
    if (!exists(database_fpath)) {
        return;
    }

    vector<string> example_image_files;
    vector<bicv_TrainingData> example_annotations;
    parse_input_annotation_file(database_fpath_str,
            example_image_files, example_annotations);

    cvp_HeadPoseDiscreteDomain head_pose_domain;
    list<ip_ImageProvider*> providers;

    // main training loop over examples
    for (unsigned id = 0; id < example_image_files.size(); ++id) {
        const string& data_fname = example_image_files[id];
        const bicv_TrainingData& data_annotation = example_annotations[id];

        IplImage * train_image = cvLoadImage(data_fname.c_str());
        if (!train_image) {
            ostringstream oss;
            oss << "Error loading image \"" << data_fname << "\"!";
            throw bicv_Exception(oss.str());
        }

        const int train_image_width = train_image->width;
        const int train_image_height = train_image->height;
        const CvRect train_face_rect = data_annotation.m_FaceRect;
        const HeadPose head_pose = data_annotation.m_HeadPose;
        ip_RoiWindow base_window = ip_RoiWindow::from_CvRect(train_face_rect);

        const OpenCvPlus::cvp_HeadPoseDiscreteDomain::ElementId head_pose_idx =
            head_pose_domain.id(head_pose_domain.discretize(head_pose));

        const vector<ip_HistogramTemplate>& templates = model->mean_templates();


//        ip_HogFeatureProducerRounded * hog_feature_producer =
//            create_hog_feature_provider_chain(
//                data_fname, image_width, image_height,
//                parameters, providers, false);
//        delete_hog_feature_provider_chain(providers, hog_feature_producer);

//        distance_L2_squared(templates[0]);

    }

}


BOOST_AUTO_TEST_SUITE_END()

//////////////////////// LOCAL DEFINITIONS ///////////////////////////////////

// parses annotation data in CSV format
static void
parse_input_annotation_file(const string& input_file,
        vector<string>& example_image_files,
        vector<bicv_TrainingData>& example_annotations) {

    namespace fs = boost::filesystem;

    typedef boost::tokenizer<boost::escaped_list_separator<char> > Tokenizer;
    static const unsigned CSV_FILE_PATH  = 0;
    static const unsigned CSV_PERSON_ID  = 1;
    static const unsigned CSV_SESSION_ID  = 2;
    static const unsigned CSV_PAN        = 3;
    static const unsigned CSV_TILT       = 4;
    static const unsigned CSV_HEAD_COL1  = 5;
    static const unsigned CSV_HEAD_ROW1  = 6;
    static const unsigned CSV_HEAD_COL2  = 7;
    static const unsigned CSV_HEAD_ROW2  = 8;
    static const unsigned CSV_LEYE_COL   = 9;
    static const unsigned CSV_LEYE_ROW   = 10;
    static const unsigned CSV_REYE_COL   = 11;
    static const unsigned CSV_REYE_ROW   = 12;
    static const unsigned CSV_NUM_FIELDS = 13; // number of values in CSV file

    ifstream input_stream(input_file.c_str());
    if (!input_stream) {
        return;
    }

    fs::path input_path(input_file);

#if BOOST_VERSION > 104500
    fs::path full_input_fpath = fs::absolute(input_path);
#else
    fs::path full_input_fpath = fs::absolute(input_path);
//    fs::path full_input_fpath = fs::complete(input_path);
#endif

    fs::path base_dir = full_input_fpath.parent_path();

    list<string> image_files;
    list<bicv_TrainingData> annotations;

    vector<string> tokens;
    string txt_line;
    getline(input_stream, txt_line);
    for (unsigned line_count = 1; getline(input_stream, txt_line); ++line_count) {
        Tokenizer tok(txt_line);
        tokens.assign(tok.begin(), tok.end());
        if (tokens.size() != CSV_NUM_FIELDS) {
            ostringstream oss;
            oss << "Error parsing input file \"" << input_file << "\", line "
                << line_count << ": "
                << "Expected " << CSV_NUM_FIELDS << " tokens, but found "
                << tokens.size() << ": ";
            ostream_iterator<string> out_it(oss, ", ");
            copy(tokens.begin(), tokens.end(), out_it);
            throw runtime_error(oss.str());
        }
        // push image path
        fs::path example_image_path = base_dir / tokens[CSV_FILE_PATH];
        image_files.push_back(example_image_path.string());
        // push annotation
        bicv_TrainingData data;
        // -- person and session ID
        istringstream(tokens[CSV_PERSON_ID]) >> data.m_PersonId;
        istringstream(tokens[CSV_SESSION_ID]) >> data.m_SessionId;
        // -- pan and tilt
        int pan, tilt;
        istringstream(tokens[CSV_PAN]) >> pan;
        istringstream(tokens[CSV_TILT]) >> tilt;
        data.m_HeadPose = OpenCvPlus::cvp_HeadPoseDiscreteDomain::HeadPose(
            pan, tilt, 0);
        // -- head bounding box
        int col1, row1, col2, row2;
        istringstream(tokens[CSV_HEAD_COL1]) >> col1;
        istringstream(tokens[CSV_HEAD_ROW1]) >> row1;
        istringstream(tokens[CSV_HEAD_COL2]) >> col2;
        istringstream(tokens[CSV_HEAD_ROW2]) >> row2;
        data.m_FaceRect.x = min(col1, col2);
        data.m_FaceRect.width = abs(col1 - col2) + 1;
        data.m_FaceRect.y = min(row1, row2);
        data.m_FaceRect.height = abs(row1 - row2) + 1;
        // -- left and right eye
        istringstream(tokens[CSV_LEYE_COL]) >> col1;
        istringstream(tokens[CSV_LEYE_ROW]) >> row1;
        istringstream(tokens[CSV_REYE_COL]) >> col2;
        istringstream(tokens[CSV_REYE_ROW]) >> row2;
        data.m_LeftEye.x = col1;
        data.m_LeftEye.y = row1;
        data.m_RightEye.x = col2;
        data.m_RightEye.y = row2;
        annotations.push_back(data);
    }

    example_image_files.assign(image_files.begin(), image_files.end());
    example_annotations.assign(annotations.begin(), annotations.end());
} // parse_input_annotation_file
