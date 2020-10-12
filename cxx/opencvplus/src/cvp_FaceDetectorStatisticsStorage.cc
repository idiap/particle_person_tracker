/**
 * @file cxx/opencvplus/src/cvp_FaceDetectorStatisticsStorage.cc
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

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                              // boost foreach

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetectorStatisticsStorage.h> // declaration of this
#include <opencvplus/cvp_Exceptions.h>                    // exceptions

using namespace std;

namespace OpenCvPlus {

/////////////////////////////// PUBLIC ///////////////////////////////////////

cvp_FaceDetectorStatisticsStorage::cvp_FaceDetectorStatisticsStorage(
        const ConfigMap& storage_config) {

    BOOST_FOREACH(const ConfigMap::value_type& entry, storage_config) {
        cvp_FaceDetectorPose pose = entry.first;
        const string& fname = entry.second;
        cvp_FaceDetectorStatistics stats =
                cvp_FaceDetectorStatistics::load(fname);
        m_StatsCollection[pose] = stats;
    }
} // cvp_FaceDetectorStatisticsStorage


const cvp_FaceDetectorStatistics&
cvp_FaceDetectorStatisticsStorage::face_detector_statistics(
        cvp_FaceDetectorPose pose) const {
    StatsMap::const_iterator i = m_StatsCollection.find(pose);
    if (m_StatsCollection.end() == i) {
        ostringstream oss;
        oss << "No statistics for pose " << pose << '!';
        throw cvp_Exception(oss.str());
    } else {
        return i->second;
    }
} // face_detector_statistics

std::ostream& operator<<(std::ostream& out,
        const cvp_FaceDetectorStatisticsStorage& stats_storage) {
    out << "Front:" << endl <<
        stats_storage.face_detector_statistics(CVP_FACEDETECTOR_FACE);
    out << "Left:" << endl <<
        stats_storage.face_detector_statistics(CVP_FACEDETECTOR_PROFILE_LEFT);
    out << "Right:" << endl <<
        stats_storage.face_detector_statistics(CVP_FACEDETECTOR_PROFILE_RIGHT);
    return out;
} // operator<<

} // namespace OpenCvPlus
