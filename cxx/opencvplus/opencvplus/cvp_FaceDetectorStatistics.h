/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorStatistics.h
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

#ifndef __CVP_FACEDETECTORSTATISTICS_H__
#define __CVP_FACEDETECTORSTATISTICS_H__

// SYSTEM INCLUDES
#include <ostream>

// LOCAL INCLUDES
#include <opencvplus/cvp_HeadPose.h>                     // head pose
#include <cv.h>                                          // for CvScalar

namespace OpenCvPlus {

/// @brief Structure to store face detector head pose and bounding box
/// coordinates statistics
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    21.11.2012

struct cvp_FaceDetectorStatistics {
    typedef cvp_HeadPose<float> HeadPose;

    /// Mean head pose for detected faces represented by a 3D vector
    /// (pan, tilt, roll)
    CvScalar m_MeanHeadPose;
    /// Standard deviation of head pose components for detected faces
    /// represented by a 3D vector - standard deviations for (pan, tilt, roll)
    CvScalar m_StddevHeadPose;

    /// Mean bounding box transform (X translation, Y translation,
    /// width ratio, height ratio) for detected faces
    CvScalar m_MeanBBoxTransform;
    /// Standard deviation of bounding box transform components
    /// (X translation, Y translation, width ratio, height ratio)
    /// for detected faces
    CvScalar m_StddevBBoxTransform;

    /// Save face detector statistics data into a file
    /// @param fdstats Face detector statistics data
    /// @param fname File name to store data to
    static void save(const cvp_FaceDetectorStatistics& fdstats,
            const std::string& fname);

    /// Load face detector statistics data from a file
    /// @param fname File name to load data from
    /// @return Face detector statistics data
    static cvp_FaceDetectorStatistics load(const std::string& fname);
};

/// Dumps face detector statistics to output stream
/// @param out Output stream
/// @param stats Face detector statistics
std::ostream& operator<<(std::ostream& out,
        const cvp_FaceDetectorStatistics& stats);

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORSTATISTICS_H__
