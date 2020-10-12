/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorStatisticsStorage.h
 * @date 21 November 2012
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Storage for various face detector statistics
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORSTATISTICSSTORAGE_HPP__
#define __CVP_FACEDETECTORSTATISTICSSTORAGE_HPP__

// SYSTEM INCLUDES
#include <string>
#include <map>
#include <ostream>

// LOCAL INCLUDES
#include "cvp_FaceDetectorPose.h"               // face detector pose
#include "cvp_FaceDetectorStatistics.h"         // face detector statistics

namespace OpenCvPlus {

/// @brief Factory to produce face detectors
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    09.02.2011

class cvp_FaceDetectorStatisticsStorage {

    public:

    typedef std::map<cvp_FaceDetectorPose, std::string> ConfigMap;

    // LIFECYCLE

    /// Constructor
    cvp_FaceDetectorStatisticsStorage(const ConfigMap& storage_config);

    // OPERATIONS
    const cvp_FaceDetectorStatistics& face_detector_statistics(
            cvp_FaceDetectorPose pose) const;

    private:

    typedef std::map<cvp_FaceDetectorPose, cvp_FaceDetectorStatistics> StatsMap;

    StatsMap m_StatsCollection;

};

/// Dumps face detector statistics storage to output stream
/// @param out Output stream
/// @param stats Face detector statistics storage
std::ostream& operator<<(std::ostream& out,
        const cvp_FaceDetectorStatisticsStorage& stats_storage);

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORSTATISTICSSTORAGE_HPP__
