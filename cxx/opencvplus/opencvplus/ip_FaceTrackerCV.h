// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_FaceTrackerCV - OpenCV-based face tracker
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __IP_FACETRACKERCV_H__
#define __IP_FACETRACKERCV_H__

// SYSTEM INCLUDES
#include <cv.h>                                          // OpenCV
#include <list>                                          // STL list
#include <string>                                        // STL string

// PROJECT INCLUDES
#include <ip_RoiWindow.h>                                // ROI window
#include <ip_ImageProvider.h>                            // image provider

namespace ImageProcessing {

/// @brief Structure to represent a tracked face
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

struct ip_FaceTrack {
    boost::circular_buffer<ip_FaceDescriptor> m_FaceTrajectory;
    boost::circular_buffer<ip_FaceDescriptor> m_Timestamps;
};

/// @brief OpenCV-based face tracker
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class ip_FaceTrackerCV {

    public:

    // LIFECYCLE

    ip_FaceTrackerCV(ip_FaceDetectorCV * face_detector);
    ~ip_FaceTrackerCV();

    // OPERATIONS

    const std::list<ip_FaceTrack>& track_faces(
            const std::list<ip_FaceDescriptor>& detected_faces);

    const std::list<ip_RoiWindow>& cached_tracks() const {
        return mFaces;
    }

    void invalidate() {
        mFaces.clear();
    }

    private:

    ip_ImageProvider *  mImageProvider;    // image provider
    CvHaarClassifierCascade * mCascade;
    CvMemStorage            * mStorage;
    std::list<ip_FaceTrack> mFaces;

};

} // namespace ImageProcessing

#endif // __IP_FACETRACKERCV_H__
