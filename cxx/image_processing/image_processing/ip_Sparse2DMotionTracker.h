// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionTracker - class to track sparse 2D motion vectors
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_SPARSE2DMOTIONTRACKER_H__
#define __IP_SPARSE2DMOTIONTRACKER_H__

// SYSTEM INCLUDES
#include <list>                                     // STL list
#include <boost/circular_buffer.hpp>                // boost circular buffer

// LOCAL INCLUDES
#include "ip_Sparse2DMotionProcessor.h"             // sparse motion detector

namespace ImageProcessing {

/// @brief Defines point motion track of a fixed length
///
/// Defines point motion track of a fixed length.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

typedef boost::circular_buffer<ip_MotionPoint> ip_MotionTrack;

typedef std::list<ip_MotionTrack> ip_MotionTracks;


/// @brief Class to track sparse 2D motion vectors
///
/// This class introduces a method to track sparse 2D motion vectors.
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_Sparse2DMotionTracker {

    public:

    // LIFECYCLE

    /// Constructor to initialize sparse 2D motion tracker
    /// @param motion_processor Sparse motion processor providing observations
    ip_Sparse2DMotionTracker(ip_Sparse2DMotionProcessor * motion_processor,
            unsigned history_length = 10);

    /// Destructor, deallocates motion tracker history
    virtual ~ip_Sparse2DMotionTracker();

    // OPERATIONS

    /// Makes current history invalid, forces recalculation
    void invalidate();

    /// Provides estimated 2D motion vectors and the corresponding image points
    /// @return Estimated 2D motion vectors and the corresponding image points
    virtual const ip_MotionTracks& get2DMotionTracks();

    /// Provides estimated 2D motion vectors and the corresponding image points
    /// @param roi Region of interest to search the points of interest in
    /// @return Estimated 2D motion vectors and the corresponding image points
    const ip_MotionTracks& get2DMotionTracks(const ip_RoiWindow& roi);


    private:

    // assigns a set of detected motion points to a set of existing tracks,
    // points that could not be assigned initiate new tracks
    void assign_detected_pts_to_tracks(const ip_MotionPoints& pts);

    // sparse motion processor
    ip_Sparse2DMotionProcessor * m_MotionProcessor;

    // motion tracks history
    std::list<ip_MotionTrack> m_MotionTracks;

    // motion track of the etalon length
    ip_MotionTrack m_EtalonMotionTrack;

    // flag indicating whether the motion tracks are up-to-date
    bool m_TracksValid;


}; // class ip_Sparse2DMotionTracker

} // namespace ImageProcessing

#endif // __IP_SPARSE2DMOTIONTRACKER_H__
