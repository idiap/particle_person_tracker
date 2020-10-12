/**
 * @file cxx/examples/facetrack/standalone/TrainHogModel.cpp
 * @date 24 October 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Trains the histogram of gradients (HOG) model based on annotated data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/program_options.hpp>            // boost program options
#include <boost/filesystem.hpp>                 // boost filesystem
#include <boost/tokenizer.hpp>                  // boost tokenizer
#include <boost/foreach.hpp>                    // boost foreach loop
#include <sstream>                              // STL string stream
#include <iostream>                             // STL IO streams
#include <fstream>                              // STL file stream

// PROJECT INCLUDES
#include <bayes_image/bicv_HogModelTrainer.h>   // HOG trainer

// LOCAL INCLUDES

using namespace std;
using namespace BICV;
using namespace ImageProcessing;
using namespace OpenCvPlus;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const string HELP_OPTION_STR = "help";
static const string INPUT_OPTION_STR = "input";
static const string OUTPUT_OPTION_STR = "output";

static const unsigned NUM_BLOCKS_ROW = 2;   // number of blocks per pattern row
static const unsigned NUM_BLOCKS_COL = 2;   // number of blocks per pattern col
static const unsigned NUM_CELLS_IN_BLOCK_ROW = 4; // number of cells per block row
static const unsigned NUM_CELLS_IN_BLOCK_COL = 4; // number of cells per block col
static const unsigned NUM_HIST_BINS = 8;    // number of bins in a histogram

static void
verify_options(const po::options_description& options,
               const po::variables_map& options_map);

static void
parse_input_annotation_file(const string& input_file,
        vector<string>& example_image_files,
        vector<bicv_TrainingData>& example_annotations);

/////////////////////////////// PUBLIC ///////////////////////////////////////

int main (int argc, char **argv) {

    po::variables_map options_map;
    po::options_description cmdline_options("General options");
    cmdline_options.add_options()
        (HELP_OPTION_STR.c_str(), "Produce help message")
        ((INPUT_OPTION_STR + ",i").c_str(),  po::value<string>(), "Input annotations file")
        ((OUTPUT_OPTION_STR + ",o").c_str(), po::value<string>(), "Output file")
        ;
    po::store(po::parse_command_line(argc, argv, cmdline_options),
            options_map);
    po::notify(options_map);

    try{
        verify_options(cmdline_options, options_map);
    } catch (runtime_error& e) {
        cerr << e.what() << endl;
        return 1;
    }

    // use head pose domain default constructor
    cvp_HeadPoseDiscreteDomain head_pose_domain;
    // set up hog feature parameters
    ip_HogFeatureParameters hog_feature_parameters(
        NUM_BLOCKS_ROW, NUM_BLOCKS_COL,
        NUM_CELLS_IN_BLOCK_ROW, NUM_CELLS_IN_BLOCK_COL,
        NUM_HIST_BINS);

    // fill image and annotation data, parse input data file
    vector<string> example_image_files;
    vector<bicv_TrainingData> example_annotations;
    cout << "Parsing annotations file... " << flush;
    parse_input_annotation_file(options_map[INPUT_OPTION_STR].as<string>(),
            example_image_files, example_annotations);
    cout << "Done!" << endl;
//    for (unsigned i = 0; i < example_image_files.size(); ++i) {
//        cout << example_image_files[i] << " (" <<
//                example_annotations[i].m_HeadPose << ") (" <<
//                example_annotations[i].m_FaceRect.x << ", " <<
//                example_annotations[i].m_FaceRect.y << ", " <<
//                example_annotations[i].m_FaceRect.width << ", " <<
//                example_annotations[i].m_FaceRect.height << ") (" <<
//                example_annotations[i].m_LeftEye.x << ", " <<
//                example_annotations[i].m_LeftEye.y << ") (" <<
//                example_annotations[i].m_RightEye.x << ", " <<
//                example_annotations[i].m_RightEye.y << ") " <<
//                endl;
//    }

    // train the head pose HOG model
    cout << "Training HOG model... " << flush;
    bicv_HeadPoseHogModel * head_pose_hog_model = bicv_HeadPoseHogModel::train(
        &head_pose_domain, hog_feature_parameters,
        example_image_files, example_annotations);
    head_pose_hog_model->save(options_map[OUTPUT_OPTION_STR].as<string>());
    cout << "Done!" << endl;

    return 0;

} // main

//////////////////////////// LOCAL DEFINITIONS ///////////////////////////////

static void
verify_options(const po::options_description& options,
               const po::variables_map& options_map) {
    // for "--help" option show help message:
    if (options_map.count(HELP_OPTION_STR)) {
        ostringstream oss;
        oss << options;
        throw runtime_error(oss.str());
    }

    // check "--input" is provided and refers to a regular file
    if (options_map.count(INPUT_OPTION_STR)) {
        fs::path input_path(options_map[INPUT_OPTION_STR].as<string>());
        if (exists(input_path)) {
            if (!fs::is_regular_file(input_path)) {
                ostringstream oss;
                oss << "Input path " << input_path <<
                        " does not refer to a regular file!";
                throw runtime_error(oss.str());
            }
        } else {
            ostringstream oss;
            oss << "Input path " << input_path <<
                    " does not refer to an existing file!";
            throw runtime_error(oss.str());
        }
    } else {
        ostringstream oss;
        oss << "Input annotations file must be provided!";
        throw runtime_error(oss.str());
    }

    // check "--output" is provided
    if (options_map.count(OUTPUT_OPTION_STR)) {
        fs::path output_path(options_map[OUTPUT_OPTION_STR].as<string>());
        if (exists(output_path)) {
            if (!fs::is_regular_file(output_path)) {
                ostringstream oss;
                oss << "Output path " << output_path <<
                        " exists and does not refer to a regular file!";
                throw runtime_error(oss.str());
            } else {
                string answer;
                while ((answer != "Y") && (answer != "N")) {
                    cout << "File " << output_path <<
                        " already exists. Overwrite? (Y/N):";
                    cin >> answer;
                }
                if (answer == "N") {
                    exit(0);
                }
            }
        }
    } else {
        ostringstream oss;
        oss << "Output file must be provided!";
        throw runtime_error(oss.str());
    }
} // verify_options

// parses annotation data in CSV format
static void
parse_input_annotation_file(const string& input_file,
        vector<string>& example_image_files,
        vector<bicv_TrainingData>& example_annotations) {

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
    fs::path full_input_fpath = fs::complete(input_path);
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
