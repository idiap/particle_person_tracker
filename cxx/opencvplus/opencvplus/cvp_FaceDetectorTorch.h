/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetectorTorch.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector using Torch3 library
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTORTORCH_H__
#define __CVP_FACEDETECTORTORCH_H__

#ifdef __TORCH_FACE_DETECTOR_FOUND__

// SYSTEM INCLUDES
#include <TorchFaceDetector.h>                           // torch3 face detector
#include <list>                                          // STL list
#include <string>                                        // STL string
#include <boost/foreach.hpp>                             // boost foreach loop

// LOCAL INCLUDES
#include "cvp_FaceDetector.h"                            // face detector

namespace OpenCvPlus {

/// @brief Face detector from Torch3 library
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class cvp_FaceDetectorTorch : public cvp_FaceDetector {

    public:

    // LIFECYCLE

    /// Constructor
    /// @param model_dir Directory containing trained models
    /// @param config_file Configuration file name
    cvp_FaceDetectorTorch(const std::string& model_dir,
        const std::string& config_file);

    /// Destructor
    virtual ~cvp_FaceDetectorTorch();

    protected:

    // OPERATIONS

    /// Overriddes base class method: allocates storage and
    /// initializes detector.
    /// @param image Image to detect faces on.
    virtual void prepare_internal(IplImage * image);

    /// Overrides base class method. Stores a list of faces detected by the
    /// OpenCV face detector on the provided image in the provided list.
    /// @param image Image to detect faces on.
    /// @param time Timestamp of the image to assign to face detections
    /// @param storage A list to store detected faces to.
    virtual void detect_faces_internal(IplImage * image,
            const boost::posix_time::ptime& time,
            std::list<cvp_FaceDescriptor>& storage);

    private:

    void cleanup_buffers();

    Torch::TorchFaceDetector * m_TorchFaceDetector;
    IplImage * m_GrayScaleImage;

};

} // namespace OpenCvPlus

#endif // __TORCH_FACE_DETECTOR_FOUND__

#endif // __CVP_FACEDETECTORTORCH_H__
