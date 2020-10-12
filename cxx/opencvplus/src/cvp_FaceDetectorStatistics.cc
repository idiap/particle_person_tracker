/**
 * @file cxx/opencvplus/src/cvp_FaceDetectorStatistics.cc
 * @date 21 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector head pose and bounding box coordinates statistics
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <iostream>                                       // STL I/O
#include <fstream>                                        // STL file I/O

// LOCAL INCLUDES
#include <opencvplus/cvp_Exceptions.h>                   // exception
#include <opencvplus/cvp_FaceDetectorStatistics.h>       // declaration of this

using namespace std;

namespace OpenCvPlus {

/* static */ void
cvp_FaceDetectorStatistics::save(
        const cvp_FaceDetectorStatistics& fdstats,
        const std::string& fname) {
    ofstream fdstats_file;
    fdstats_file.open(fname.c_str(), ios::out | ios::binary);
    if (fdstats_file.is_open()) {
        const double* pointers[] = {
                fdstats.m_MeanHeadPose.val,
                fdstats.m_StddevHeadPose.val,
                fdstats.m_MeanBBoxTransform.val,
                fdstats.m_StddevBBoxTransform.val};
        const double* end_pointers[] = {
                fdstats.m_MeanHeadPose.val + 3,
                fdstats.m_StddevHeadPose.val + 3,
                fdstats.m_MeanBBoxTransform.val  +4,
                fdstats.m_StddevBBoxTransform.val + 4};

        const double**const pointers_begin = pointers;
        const double**const pointers_end = pointers + 4;
        const double**const end_pointers_begin = end_pointers;

        const double** pointer = pointers_begin;
        const double** end_pointer = end_pointers_begin;

        const double * parr;
        const double * end_parr;
        float val;
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                val = static_cast<float>(*parr++);
                fdstats_file.write((char*)(&val), sizeof(float));
            }
        }

        fdstats_file.close();

    } else {
        throw cvp_Exception("Face detector statistics file \"" + fname +
            "\" could not be opened for writing!");
    }
} // save

/* static */ cvp_FaceDetectorStatistics
cvp_FaceDetectorStatistics::load(const std::string& fname) {
    ifstream fdstats_file;
    fdstats_file.open(fname.c_str(), ios::in | ios::binary);
    if (fdstats_file.is_open()) {
        cvp_FaceDetectorStatistics fdstats;
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
        char buffer[sizeof(float)];
        for (; pointer != pointers_end; ++pointer, ++end_pointer) {
            parr = *pointer;
            end_parr = *end_pointer;
            while (parr != end_parr) {
                fdstats_file.read(buffer, sizeof(float));
                *parr++ = *(float*)buffer;
            }
        }
        fdstats_file.close();
        return fdstats;
    } else {
        throw cvp_Exception("Face detector statistics file \"" + fname +
            "\" could not be opened for reading!");
    }
} // load

std::ostream& operator<<(std::ostream& out,
        const cvp_FaceDetectorStatistics& stats) {
    out << "Head pose: mean (" << stats.m_MeanHeadPose.val[0] << ", "
        << stats.m_MeanHeadPose.val[1] << ", "
        << stats.m_MeanHeadPose.val[2] << "), "
        << "stddev (" << stats.m_StddevHeadPose.val[0] << ", "
        << stats.m_StddevHeadPose.val[1] << ", "
        << stats.m_StddevHeadPose.val[2] << ") " << endl;
    out << "BBox transform: mean (" << stats.m_MeanBBoxTransform.val[0] << ", "
        << stats.m_MeanBBoxTransform.val[1] << ", "
        << stats.m_MeanBBoxTransform.val[2] << ", "
        << stats.m_MeanBBoxTransform.val[3] << "), "
        << "stddev (" << stats.m_StddevBBoxTransform.val[0] << ", "
        << stats.m_StddevBBoxTransform.val[1] << ", "
        << stats.m_StddevBBoxTransform.val[2] << ", "
        << stats.m_StddevBBoxTransform.val[3] << ") " << endl;
    return out;
} // operator<<

} // namespace OpenCvPlus
