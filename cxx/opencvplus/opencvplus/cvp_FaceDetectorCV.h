/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorCV.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector implementation from OpenCV
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORCV_H__
#define __CVP_FACEDETECTORCV_H__

// SYSTEM INCLUDES
#include <cv.h>                                          // OpenCV

// LOCAL INCLUDES
#include "opencvplus/cvp_FaceDetector.h"                 // face detector

namespace OpenCvPlus {

/// @brief OpenCV face detector
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class cvp_FaceDetectorCV : public cvp_FaceDetector {

    public:

    // LIFECYCLE

    /// Constructor.
    /// @param cascade_path Path to the trained OpenCV detector cascade.
    /// @param pose Face orientation that was learnt by the detector.
    cvp_FaceDetectorCV(const std::string& cascade_path,
            cvp_FaceDetectorPose pose);

    /// Destructor.
    virtual ~cvp_FaceDetectorCV();

    protected:

    // OPERATIONS

    /// Overriddes base class method: allocates storage and
    /// initializes detector.
    /// @param image Image to detect faces on.
    virtual void prepare_internal(IplImage * image);

    /// Overrides base class method. Stores a list of faces detected by the
    /// OpenCV face detector on the provided image in the provided list.
    /// @param image Image to detect faces on.
    /// @param storage A list to store detected faces to.
    virtual void detect_faces_internal(IplImage * image,
            const boost::posix_time::ptime& time,
            std::list<cvp_FaceDescriptor>& storage);

    private:

    CvHaarClassifierCascade * mCascade;
    CvMemStorage            * mStorage;

};

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTORCV_H__
