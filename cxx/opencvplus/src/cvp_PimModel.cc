/**
 * @file cxx/opencvplus/src/cvp_PimModel.cc
 * @date 25 May 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Probability index map (PIM) model class
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <iostream>                            // STL I/O
#include <fstream>                             // STL file I/O

// LOCAL INCLUDES
#include <opencvplus/cvp_PimModel.h>           // declaration of this
#include <opencvplus/cvp_IplDepthTraits.h>     // maps types to OpenCV depth
#include <opencvplus/cvp_Exceptions.h>         // OpenCvPlus exceptions

using namespace std;

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

#define NUM_CHANNELS 4

#define NUM_BYTES_INT 4
#define NUM_BYTES_DOUBLE 8

// define 32bit integer type depending on target architecture
typedef int int_32;
// define 64bit float type depending on target architecture
typedef double double_64;

// check the size of int, 32-bit is used to read binary data from file
BOOST_STATIC_ASSERT(sizeof(int_32) == NUM_BYTES_INT);
// check the size of double, 64-bit is used to read binary data from file
BOOST_STATIC_ASSERT(sizeof(double_64) == NUM_BYTES_DOUBLE);

// read 32bit integer value from a binary stream
static int_32 read_int32_from_stream(istream& instream, char * buffer,
        const string& descr, bool unsigned_flag);
// fill data from 64bit floating point array into a pim feature

/////////////////////////////// PUBLIC ///////////////////////////////////////

namespace OpenCvPlus {

cvp_PimModel::cvp_PimModel(const cvp_PimModel& other) : m_Name(other.m_Name),
        m_FaceRect(other.m_FaceRect) {
    m_PimFeature = new PimFeatureType(other.data());
} // cvp_PimModel

cvp_PimModel::~cvp_PimModel() {
    delete m_PimFeature;
} // ~cvp_PimModel

/* static */ cvp_PimModel*
cvp_PimModel::load_from_file(const std::string& filename) {
    ifstream pim_model_file;
    pim_model_file.open(filename.c_str(), ios::in | ios::binary);
    if (pim_model_file.is_open()) {
        cvp_PimModel * model = 0;
        try {
            char buffer_int[NUM_BYTES_INT];
            const unsigned model_width = static_cast<unsigned>(
                read_int32_from_stream(pim_model_file, buffer_int,
                        "model_width", true));
            const unsigned model_height = static_cast<unsigned>(
                read_int32_from_stream(pim_model_file, buffer_int,
                        "model_height", true));
            const int_32 fc = read_int32_from_stream(
                    pim_model_file, buffer_int, "first_column", false);
            const int_32 fr = read_int32_from_stream(
                    pim_model_file, buffer_int, "first_row", false);
            const int_32 w = read_int32_from_stream(
                    pim_model_file, buffer_int, "width", true);
            const int_32 h = read_int32_from_stream(
                    pim_model_file, buffer_int, "height", true);
            const unsigned name_len = static_cast<unsigned>(
                read_int32_from_stream(pim_model_file, buffer_int,
                        "name_length", true));

            char * data_name = new char[name_len + 1];
            try {
                pim_model_file.read(data_name, name_len);
                const unsigned num_chars_read = pim_model_file.gcount();
                if (num_chars_read != name_len) {
                    ostringstream oss;
                    oss << "Error reading model name from PIM model file,"
                        << " expected " << name_len << " characters,"
                        << " actually read " << num_chars_read;
                    throw cvp_Exception(oss.str());
                }
            } catch (...) {
                delete [] data_name;
                throw;
            }
            string data_name_str(data_name, name_len + 1);
            delete [] data_name;

//            m_PimModelWidth = model_width;
//            m_PimModelHeight = model_height;
//            m_PimFaceRect = cvRect(fc, fr, w, h);

            // create PIM feature prototype
            PimFeatureType * pim_feature = new PimFeatureType(
                NUM_CHANNELS, model_width, model_height);

            // try reading model data into the feature prototype
            const unsigned pim_model_size = model_width * model_height;
            vector<double_64> pim_model_data(NUM_CHANNELS * pim_model_size);
            const unsigned num_chars_to_read =
                    NUM_CHANNELS * pim_model_size * NUM_BYTES_DOUBLE;
            try {
                pim_model_file.read((char*)(&pim_model_data[0]),
                        num_chars_to_read);
                const unsigned num_chars_read = pim_model_file.gcount();
                if (num_chars_read != num_chars_to_read) {
                    ostringstream oss;
                    oss << "Error reading data from PIM model file,"
                        << " expected " << num_chars_to_read << " bytes,"
                        << " actually read " << num_chars_read;
                    throw cvp_Exception(oss.str());
                }
                pim_feature->from_array(pim_model_data);
            } catch (...) {
                delete pim_feature;
                throw;
            }

            model = new cvp_PimModel(data_name_str, pim_feature,
                cvRect(fc, fr, w, h));

        } catch (...) {
            pim_model_file.close();
            throw;
        }

        pim_model_file.close();

        return model;

    } else {
        throw cvp_Exception("PIM model file " + filename +
            " could not be opened!");
    }
} // load_from_file

void cvp_PimModel::save_to_file(const std::string& filename) const {
    ofstream pim_model_file;
    pim_model_file.open(filename.c_str(), ios::out | ios::binary);
    if (pim_model_file.is_open()) {
        try {
            ostringstream oss;
            char buffer_int[NUM_BYTES_INT];

            // write PIM feature size
            *(int_32*)buffer_int = static_cast<int_32>(m_PimFeature->width());
            pim_model_file.write(buffer_int, NUM_BYTES_INT);
            *(int_32*)buffer_int = static_cast<int_32>(m_PimFeature->height());
            pim_model_file.write(buffer_int, NUM_BYTES_INT);

            // write face coordinates
            *(int_32*)buffer_int = static_cast<int_32>(m_FaceRect.x);
            pim_model_file.write(buffer_int, NUM_BYTES_INT);
            *(int_32*)buffer_int = static_cast<int_32>(m_FaceRect.y);
            pim_model_file.write(buffer_int, NUM_BYTES_INT);
            *(int_32*)buffer_int = static_cast<int_32>(m_FaceRect.width);
            pim_model_file.write(buffer_int, NUM_BYTES_INT);
            *(int_32*)buffer_int = static_cast<int_32>(m_FaceRect.height);
            pim_model_file.write(buffer_int, NUM_BYTES_INT);

            // write name and its length
            *(int_32*)buffer_int = static_cast<int_32>(m_Name.size());
            pim_model_file.write(buffer_int, NUM_BYTES_INT);
            const char * data_name = m_Name.c_str();
            pim_model_file.write(data_name, m_Name.size());

            // write model data
            const unsigned pim_model_size = m_PimFeature->width() *
                m_PimFeature->height();
            vector<double_64> pim_model_data(NUM_CHANNELS * pim_model_size);
            const unsigned num_chars_to_write =
                    NUM_CHANNELS * pim_model_size * NUM_BYTES_DOUBLE;
            m_PimFeature->to_array(pim_model_data);
            pim_model_file.write((char*)(&pim_model_data[0]),
                    num_chars_to_write);

            pim_model_file.close();
        } catch(...) {
            pim_model_file.close();
            throw;
        }
    } else {
        throw cvp_Exception("PIM model file " + filename +
            " could not be opened for write!");
    }
} // save_to_file

cvp_PimModel& cvp_PimModel::operator=(const cvp_PimModel& rhs) {
    m_Name = rhs.name();
    m_FaceRect = rhs.face_rect();
    *m_PimFeature = rhs.data();
    return *this;
} // operator=

/////////////////////////////// PRIVATE //////////////////////////////////////

cvp_PimModel::cvp_PimModel(const std::string& name,
    PimFeatureType * pim_feature, const CvRect& face_rect) :
    m_Name(name), m_PimFeature(pim_feature), m_FaceRect(face_rect) {
} // cvp_PimModel

} // namespace OpenCvPlus

/////////////////////////// LOCAL DEFINITIONS ////////////////////////////////

/* static */
int_32 read_int32_from_stream(istream& in, char * buffer, const string& descr,
        bool unsigned_flag) {

    using namespace OpenCvPlus;

    const unsigned num_chars_to_read = NUM_BYTES_INT;
    in.read(buffer, num_chars_to_read);
    const unsigned num_chars_read = in.gcount();
    if (num_chars_read != num_chars_to_read) {
        ostringstream oss;
        oss << "Error reading integer value (" << descr
            << ") from PIM model file, expected "
            << num_chars_to_read << ", actually read " << num_chars_read;
        throw cvp_Exception(oss.str());
    }
    int_32 result = *(int_32*)buffer;
    if (unsigned_flag && (result < 0)) {
        ostringstream oss;
        oss << "Error reading unsigned integer value (" << descr
            << ") from PIM model file, negative value encountered: "
            << result;
        throw cvp_Exception(oss.str());
    }
    return result;
} // read_int32_from_stream
