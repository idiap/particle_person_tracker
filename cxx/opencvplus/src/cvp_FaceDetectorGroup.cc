/**
 * @file cxx/opencvplus/src/cvp_FaceDetectorGroup.cc
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Face detector group to work with several face detectors simultaneously
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

// SYSTEM INCLUDES
#include <boost/foreach.hpp>                             // foreach loop
#include <omp.h>                                         // OpenMP

// LOCAL INCLUDES
#include <opencvplus/cvp_FaceDetectorGroup.h>            // declaration of this

using namespace std;

namespace OpenCvPlus {

/////////////////////////////// PUBLIC ///////////////////////////////////////

cvp_FaceDetectorGroup::
cvp_FaceDetectorGroup(const std::list<cvp_FaceDetectorType>& face_detector_types,
                      const boost::program_options::variables_map& vm):
  m_FaceDetectors(face_detector_types.size(), 0),
  m_CurrentFaceDetectorIndex(0),
  m_IsPrepared(false)
{
  const unsigned num_face_detectors = face_detector_types.size();
  try {
    const list<cvp_FaceDetectorType>::const_iterator fd_types_begin =
      face_detector_types.begin();
    const list<cvp_FaceDetectorType>::const_iterator fd_types_end =
      face_detector_types.end();
    list<cvp_FaceDetectorType>::const_iterator fd_types_it = fd_types_begin;

    for (unsigned i = 0; i < num_face_detectors; ++i, ++fd_types_it) {
      cvp_FaceDetector * face_detector =
        cvp_FaceDetectorFactory::create_face_detector(
                                                      *fd_types_it, vm);
      m_FaceDetectors[i] = face_detector;
    }
  } catch (...) {
    BOOST_FOREACH(cvp_FaceDetector * fd, m_FaceDetectors) {
      delete fd;
    }
    m_FaceDetectors.clear();
    throw;
  }
}


cvp_FaceDetectorGroup::
~cvp_FaceDetectorGroup()
{
  BOOST_FOREACH(cvp_FaceDetector *fd, m_FaceDetectors)
    {
      if(fd)
        delete fd;
    }
    m_FaceDetectors.clear();
}


void
cvp_FaceDetectorGroup::
prepare(IplImage *image,
        float area_fraction)
{
  std::cout << "[FaceDetectorGroup] Preparing with image size "
            << image->width << "x" << image->height
            << std::endl;

  BOOST_FOREACH(cvp_FaceDetector * fd, m_FaceDetectors)
    {
      fd->prepare(image, area_fraction);
    }
  m_IsPrepared = true;
}


const std::list<cvp_FaceDescriptor>&
cvp_FaceDetectorGroup::
detect_faces(IplImage *image,
             const boost::posix_time::ptime& time)
{
  if(!m_IsPrepared) prepare(image);

  invalidate();

  const unsigned num_face_detectors = m_FaceDetectors.size();

  unsigned i;

  // int n;

  // #pragma omp parallel private(i, n) num_threads(3)
  {
    // n = omp_get_thread_num();
    // #pragma omp for
    for(i = 0; i < num_face_detectors; ++i)
      {
        const std::list<cvp_FaceDescriptor>& faces =
          m_FaceDetectors[i]->detect_faces(image, time);

        const std::list<cvp_BodyDescriptor>& bodies =
          m_FaceDetectors[i]->cached_bodies();

      // cout << "Thread " << n << " added " << faces.size()
      //      << " faces from detector " << i << endl;

// #pragma omp critical
      {
        copy(faces.begin(), faces.end(), back_inserter(m_Faces));
        copy(bodies.begin(), bodies.end(), back_inserter(m_Bodies));
      }
    }
  }

  return m_Faces;

}


const std::list<cvp_FaceDescriptor>&
cvp_FaceDetectorGroup::
detect_faces_next(IplImage *image,
                  const boost::posix_time::ptime& time)
{
  if(!m_IsPrepared) prepare(image);

  invalidate();

  const std::list<cvp_FaceDescriptor>& faces =
    m_FaceDetectors[m_CurrentFaceDetectorIndex]->detect_faces(image, time);

  const std::list<cvp_BodyDescriptor>& bodies =
    m_FaceDetectors[m_CurrentFaceDetectorIndex]->cached_bodies();

  m_CurrentFaceDetectorIndex++;

  m_CurrentFaceDetectorIndex =
    m_CurrentFaceDetectorIndex % m_FaceDetectors.size();

  copy(faces.begin(), faces.end(), back_inserter(m_Faces));
  copy(bodies.begin(), bodies.end(), back_inserter(m_Bodies));

  return m_Faces;

}


void
cvp_FaceDetectorGroup::
invalidate()
{
  m_Faces.clear();
  m_Bodies.clear();
  BOOST_FOREACH(cvp_FaceDetector *fd, m_FaceDetectors)
    {
      fd->invalidate();
    }
}

} // namespace OpenCvPlus
