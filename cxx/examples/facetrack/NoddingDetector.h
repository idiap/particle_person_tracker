/**
 * @file cxx/examples/facetrack/NodDetector.h
 * @date 15 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Detects nods for a tracked person
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __NODDINGDETECTOR_H__
#define __NODDINGDETECTOR_H__

// SYSTEM INCLUDES
#include <string>                                   // STL string

// PROJECT INCLUDES
#include <bayes_image/bicv_HeadPoseTracker.h>       // head pose tracker
#ifdef __ND_NOD_DETECTOR_FOUND__
#include <noddetector/nd_NodDetector.h>             // nod detector
#endif

// LOCAL INCLUDES

/// @brief Nod detector based on head motion observations
///
/// Nod detector based on head motion observations
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 3.0
/// @date    15.03.2012

class NoddingDetector {

    public:

    // LIFECYCLE

    /// Constructor
    NoddingDetector(const std::string& nod_detector_config_file);

    /// Destructor
    ~NoddingDetector();

    // OPERATIONS

    bool detect_nod(const BICV::bicv_HeadPoseTracker& tracker) const;

    private:


#ifdef __ND_NOD_DETECTOR_FOUND__
    NodDetector::nd_PointPatternMotionData create_motion_data(
        const std::list<BICV::bicv_HeadPoseTracker::TimedMotionEstim>&
            motion_history) const;

    NodDetector::nd_NodDetector * m_NodDetector;
    NodDetector::nd_NodDetectorCache * m_NodDetectorCache;
    double m_NodDetectorTimeWindow_MicroSec;
    double m_NodDetectorTimeStep_MicroSec;
#endif

};

#endif // __NODDINGDETECTOR_H__
