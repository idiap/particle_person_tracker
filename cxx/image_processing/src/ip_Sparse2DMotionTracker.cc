// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionTracker - class to track sparse 2D motion vectors
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <list>                                         // STL list

// LOCAL INCLUDES
#include <image_processing/ip_Sparse2DMotionTracker.h>  // declaration of this

using namespace std;

namespace ImageProcessing {

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const double TRACKING_THRESHOLD = 4;

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_Sparse2DMotionTracker::ip_Sparse2DMotionTracker(
        ip_Sparse2DMotionProcessor * motion_processor,
        unsigned history_length) : m_MotionProcessor(motion_processor),
                                   m_EtalonMotionTrack(history_length),
                                   m_TracksValid(false) {

} // ip_Sparse2DMotionTracker

/* virtual */
ip_Sparse2DMotionTracker::~ip_Sparse2DMotionTracker() {
    m_MotionProcessor = 0;
    m_MotionTracks.clear();
} // ~ip_Sparse2DMotionTracker

void ip_Sparse2DMotionTracker::invalidate() {
    m_TracksValid = false;
} // invalidate

/* virtual */ const ip_MotionTracks&
ip_Sparse2DMotionTracker::get2DMotionTracks() {
    if (!m_TracksValid) {
        const ip_MotionPoints& pts = m_MotionProcessor->get2DMotionPoints();
        assign_detected_pts_to_tracks(pts);
        m_TracksValid = true;
    }
    return m_MotionTracks;
} // get2DMotionTracks

/* virtual */ const ip_MotionTracks&
ip_Sparse2DMotionTracker::get2DMotionTracks(const ip_RoiWindow& roi) {
    if (!m_TracksValid) {
        const ip_MotionPoints& pts = m_MotionProcessor->get2DMotionPoints(roi);
        assign_detected_pts_to_tracks(pts);
        m_TracksValid = true;
    }
    return m_MotionTracks;
} // get2DMotionTracks

/////////////////////////////// PRIVATE //////////////////////////////////////

void ip_Sparse2DMotionTracker::assign_detected_pts_to_tracks(
        const ip_MotionPoints& pts) {

    ip_MotionPoints::const_iterator ii_begin = pts.begin();
    ip_MotionPoints::const_iterator ii_end = pts.end();
    ip_MotionTracks::iterator jj_begin = m_MotionTracks.begin();
    ip_MotionTracks::iterator jj_end = m_MotionTracks.end();

    CvPoint2D32f trackPt;
    vector<int> trajectory_updated(m_MotionTracks.size(), 0);
    int * ptrajflag;
    bool point_assigned;
    list<ip_MotionPoint> new_track_candidates;

    for (ip_MotionPoints::const_iterator ii = ii_begin; ii != ii_end; ++ii) {
        point_assigned = false;
        ptrajflag = &trajectory_updated[0];
        for (ip_MotionTracks::iterator jj = jj_begin; jj != jj_end; ++jj) {
            if (!(*ptrajflag)) {
                // if the trajectory has not been assigned a point yet
                trackPt = jj->back().m_Point;
                if (((trackPt.x - (ii->m_Point.x - ii->m_Motion.x)) *
                     (trackPt.x - (ii->m_Point.x - ii->m_Motion.x)) +
                     (trackPt.y - (ii->m_Point.y - ii->m_Motion.y)) *
                     (trackPt.y - (ii->m_Point.y - ii->m_Motion.y))) <
                     TRACKING_THRESHOLD) {
                    // add the point to the trajectory if it is close enough
                    jj->push_back(*ii);
                    *ptrajflag = 1;
                    point_assigned = true;
                }
            }
            ptrajflag++;
        }
        if (!point_assigned) {
            new_track_candidates.push_back(*ii);
        }
    }

    // initiate trajectories from all the unassociated points
    for (list<ip_MotionPoint>::const_iterator ii = new_track_candidates.begin();
            ii != new_track_candidates.end(); ++ii) {
        ip_MotionTrack new_track(m_EtalonMotionTrack);
        new_track.push_back(*ii);
        m_MotionTracks.push_back(new_track);
    }

    // delete trajectoried that were not updated

} // assign_detected_pts_to_tracks

} // namespace ImageProcessing
