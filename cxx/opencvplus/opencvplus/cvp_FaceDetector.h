/**
 * @file cxx/opencvplus/opencvplus/cvp_FaceDetector.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief General interface for a face detector
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __CVP_FACEDETECTOR_H__
#define __CVP_FACEDETECTOR_H__

// SYSTEM INCLUDES
#include <list>                                          // STL list
#include <string>                                        // STL string
#include <boost/date_time/posix_time/posix_time.hpp>     // boost ptime
#include <cv.h>                                          // OpenCV library
#include <boost/random/mersenne_twister.hpp>             // random numbers

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetectorPose.h>             // FD pose
#include <opencvplus/cvp_FaceDescriptor.h>               // detection info
#include <opencvplus/cvp_BodyDescriptor.h>

namespace OpenCvPlus {

/// @brief General face detector interface
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    03.02.2011

class cvp_FaceDetector {

    public:

    // LIFECYCLE

    /// Constructor.
    ///@param pose Pose describing orientation of faces detected by this face
    /// detector
    cvp_FaceDetector(cvp_FaceDetectorPose pose);

    /// Destructor.
    virtual ~cvp_FaceDetector();

    // OPERATIONS

    /// Prepares a face detector to work with images of a certain type
    /// and image fractions. Makes a call to internal preparation method
    /// overridden in descendants to perform initialisation and
    /// storage allocation.
    /// Images passed to the detect_faces method should have the same type
    /// as the one provided to this method, otherwise a runtime error
    /// could occur.
    /// @param image Example of image this detector would deal with.
    /// @param area_fraction Area fraction to be used for face detection,
    ///         defaults to 1.0
    void prepare(IplImage * image, float area_fraction = 1.0f);

    /// Detect faces on a given image. The faces are cached and can be further
    /// retrieved by calling cached_faces method.
    /// @param image Image to detect faces on
    /// @param time Timestamp of the image, to be assigned to face detections
    /// @return A list of face descriptors.
    const std::list<cvp_FaceDescriptor>& detect_faces(IplImage * image,
        const boost::posix_time::ptime& time);

    /// Return cached faces stored from the last detection.
    /// @return A list of cached face descriptors.
    const std::list<cvp_FaceDescriptor>& cached_faces() const {
        return mFaces;
    }

    /// Return cached bodies stored from the last detection.
    /// @return A list of cached body descriptors.
    const std::list<cvp_BodyDescriptor>& cached_bodies() const {
        return mBodies;
    }

    /// Clears cached faces.
    void invalidate() { mFaces.clear(); mBodies.clear(); }

    /// Returns pose describing orientation of detected faces
    cvp_FaceDetectorPose pose() const {
        return m_Pose;
    }

    protected:

    /// Pure virtual method to be overridden by descendants: should store
    /// a list of faces detected on the provided image in the provided list.
    /// @param image Image to detect faces on.
    /// @param time Timestamp of the image to assign to face detections
    /// @param storage A list to store detected faces into.
    virtual void detect_faces_internal(IplImage * image,
            const boost::posix_time::ptime& time,
            std::list<cvp_FaceDescriptor>& storage) = 0;

    /// Pure virtual method to be overridden by descendants: allocates
    /// storage and initializes detector.
    /// @param image Image to detect faces on.
    virtual void prepare_internal(IplImage * image) = 0;

    private:

    /// Generates random seed for random number generator m_RoiRng
    /// @return Random seed for random number generator
    inline unsigned generate_random_seed() const;

    /// Selects a random part of the source image that matches the size of
    /// the buffer and warps it to the buffer
    /// @param image Input image
    /// @param buffer Image buffer to extract part of the input image to
    void warp_random_image_part_to_buffer(IplImage * image, IplImage * buffer);

    /// Corrects face coordinates by current warp parameters
    /// @param image Input image
    /// @param faces List of detected faces
    void correct_coordinates_by_warp(IplImage *image,
                                     std::list<cvp_FaceDescriptor>& faces);


    // face detector pose
    cvp_FaceDetectorPose m_Pose;
    // cache - list of previously detected faces
    std::list<cvp_FaceDescriptor> mFaces;
    // buffer image for optimized face detection on parts of the original image
    IplImage * m_BufferImage;
    // warp matrix for buffer image
    CvMat * m_BufferWarpMat;
    // random number generator for ROI generator
    boost::mt19937 m_RoiRng;

protected:
    // cache - list of previously detected bodies (only for CPM)
    std::list<cvp_BodyDescriptor> mBodies;

};

} // namespace OpenCvPlus

#endif // __CVP_FACEDETECTOR_H__
