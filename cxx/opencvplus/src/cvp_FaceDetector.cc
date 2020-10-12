// Copyright (c) 2011-2020 Idiap Research Institute
//
// cvp_FaceDetector - general interface for a face detector
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <iterator>                                      // for back_inserter
#include <boost/random.hpp>                              // taus88 RNG
#include <boost/foreach.hpp>                             // foreach
#include <ctime>                                         // time
#include <sstream>                                       // string stream
#include <highgui.h>                                     // OpenCV save image

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetector.h>                 // declaration of this
#include <opencvplus/cvp_IplDepthTraits.h>               // traits

#define ROUND_POSITIVE(X) int((X) + 0.5)


using namespace std;


namespace OpenCvPlus {


cvp_FaceDetector::
cvp_FaceDetector(cvp_FaceDetectorPose pose):
  m_Pose(pose),
  m_BufferImage(0),
  m_RoiRng(generate_random_seed())
{
  m_BufferWarpMat = cvCreateMat(2, 3, CV_32FC1);

  typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;

  CV_MAT_ELEM(*m_BufferWarpMat, float32, 0, 0) = 1;
  CV_MAT_ELEM(*m_BufferWarpMat, float32, 0, 1) = 0;
  CV_MAT_ELEM(*m_BufferWarpMat, float32, 1, 0) = 0;
  CV_MAT_ELEM(*m_BufferWarpMat, float32, 1, 1) = 1;
}

cvp_FaceDetector::
~cvp_FaceDetector()
{
  cvReleaseMat(&m_BufferWarpMat);
  if (m_BufferImage) {
    cvReleaseImage(&m_BufferImage);
  }
}


void
cvp_FaceDetector::
prepare(IplImage *image, float area_fraction)
{
  assert(area_fraction <= 1);
  float buffer_image_width = image->width;
  buffer_image_width *= area_fraction;
  float buffer_image_height = image->height;
  buffer_image_height *= area_fraction;
  m_BufferImage = cvCreateImage(cvSize(ROUND_POSITIVE(buffer_image_width),
                                       ROUND_POSITIVE(buffer_image_height)),
                                image->depth, image->nChannels);

  prepare_internal(m_BufferImage);
}


const std::list<cvp_FaceDescriptor>&
cvp_FaceDetector::detect_faces(IplImage *image,
                               const boost::posix_time::ptime& time)
{
  invalidate();

  warp_random_image_part_to_buffer(image, m_BufferImage);

  list<cvp_FaceDescriptor> faces;
  detect_faces_internal(m_BufferImage, time, faces);

  correct_coordinates_by_warp(image, faces);

  copy(faces.begin(), faces.end(), back_inserter(mFaces));

  return mFaces;
}


inline unsigned
cvp_FaceDetector::generate_random_seed() const {
    static const unsigned NUM_BOOTSTRAP_ITER = 100;
    boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::ptime current_time =
            boost::posix_time::microsec_clock::local_time();
    long current_microsec = (current_time - epoch).total_microseconds();
    boost::taus88 rng(current_microsec);
    for (unsigned i = 0; i < NUM_BOOTSTRAP_ITER; ++i) {
        rng();
    }
    unsigned result = rng();
    return result;
} // generate_random_seed

void cvp_FaceDetector::warp_random_image_part_to_buffer(IplImage * image,
        IplImage * buffer) {
    const float random_val_x = float(m_RoiRng() - m_RoiRng.min()) /
                       float(m_RoiRng.max() - m_RoiRng.min());
    const int offset_x =
            ROUND_POSITIVE(random_val_x * (image->width - buffer->width));
    const float random_val_y = float(m_RoiRng() - m_RoiRng.min()) /
                       float(m_RoiRng.max() - m_RoiRng.min());
    const int offset_y =
            ROUND_POSITIVE(random_val_y * (image->height - buffer->height));

    typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;

    CV_MAT_ELEM(*m_BufferWarpMat, float32, 0, 2) = offset_x;
    CV_MAT_ELEM(*m_BufferWarpMat, float32, 1, 2) = offset_y;

    cvWarpAffine(image, buffer, m_BufferWarpMat,
            CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

} // warp_random_image_part_to_buffer

void cvp_FaceDetector::correct_coordinates_by_warp(IplImage * image,
        std::list<cvp_FaceDescriptor>& faces) {

    typedef cvp_IplTypeTraits<IPL_DEPTH_32F>::type float32;
    const int offset_x = CV_MAT_ELEM(*m_BufferWarpMat, float32, 0, 2);
    const int offset_y = CV_MAT_ELEM(*m_BufferWarpMat, float32, 1, 2);

    BOOST_FOREACH(cvp_FaceDescriptor& face, faces) {
        face.m_FaceRegion.x += offset_x;
        face.m_FaceRegion.y += offset_y;
        face.m_ImageHeight = image->height;
        face.m_ImageWidth = image->width;
    }
} // correct_coordinates_by_warp

} // namespace OpenCvPlus
