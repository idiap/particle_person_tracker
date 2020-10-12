/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorCTU.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector implementation using detector developed in
 * Czech Technical University
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORCTU_H__
#define __CVP_FACEDETECTORCTU_H__

#ifdef __CTU_FACE_DETECTOR_FOUND__

// SYSTEM INCLUDES
#include <EyeFace.h>                                     // CTU face detector
#include <list>                                          // STL list
#include <string>                                        // STL string

// LOCAL INCLUDES
#include "cvp_FaceDetector.h"                            // face detector

namespace OpenCvPlus {

/// @brief Face detector from Czech Technical University
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class cvp_FaceDetectorCTU : public cvp_FaceDetector {

    public:

    // LIFECYCLE

    /// Constructor.
    cvp_FaceDetectorCTU(const std::string& module_dir,
        const std::string& config_file);

    /// Destructor.
    virtual ~cvp_FaceDetectorCTU();

    protected:

    // OPERATIONS

    /// Overriddes base class method: allocates storage and
    /// initializes detector.
    /// @param image Image to detect faces on.
    virtual void prepare_internal(IplImage * image);

    /// Overrides base class method. Stores a list of faces detected by the
    /// CTU face detector on the provided image in the provided list.
    /// @param image Image to detect faces on.
    /// @param time Timestamp of the image to assign to face detections
    /// @param storage A list to store detected faces to.
    virtual void detect_faces_internal(IplImage * image,
            const boost::posix_time::ptime& time,
            std::list<cvp_FaceDescriptor>& storage);

    private:

    void cleanup_buffers();
    void copy_contents(IplImage * src_grayscale_img, EfMatrixUc * dst_img);

    void * m_InternalState;
    EfMatrixUc * m_InputImage;
    IplImage * m_GrayScaleImage;

};

} // namespace OpenCvPlus

#endif // __CTU_FACE_DETECTOR_FOUND__

#endif // __CVP_FACEDETECTORCTU_H__
