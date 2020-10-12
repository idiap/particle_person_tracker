/**
 * @file cxx/opencvplus/src/cvp_FaceDetectorCV.cc
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


#include "opencvplus/cvp_FaceDetectorCV.h"

namespace OpenCvPlus {


cvp_FaceDetectorCV::
cvp_FaceDetectorCV(const std::string& cascade_path,
                   cvp_FaceDetectorPose pose):
  cvp_FaceDetector(pose)
{
  mCascade = (CvHaarClassifierCascade*) cvLoad(cascade_path.c_str(), 0, 0, 0);
  mStorage = cvCreateMemStorage(0);
}


cvp_FaceDetectorCV::
~cvp_FaceDetectorCV()
{
  cvReleaseHaarClassifierCascade(&mCascade);
  cvReleaseMemStorage(&mStorage);
}


void
cvp_FaceDetectorCV::
prepare_internal(IplImage *image)
{
}


void
cvp_FaceDetectorCV::
detect_faces_internal(IplImage *image,
                      const boost::posix_time::ptime& time,
                      std::list<cvp_FaceDescriptor>& storage) {

    CvSeq *faces =
      cvHaarDetectObjects(image, mCascade, mStorage, 1.2, 5,
                          CV_HAAR_DO_CANNY_PRUNING,
                          cvSize(40,40));

    if(faces)
      {
        cvp_FaceDescriptor fd;
//        boost::posix_time::ptime detection_time =
//                boost::posix_time::microsec_clock::local_time();
        fd.m_ImageWidth = image->width;
        fd.m_ImageHeight = image->height;
        for(int i = 0; i < faces->total; i++)
          {
            CvRect * r = ( CvRect* ) cvGetSeqElem( faces, i );
            fd.m_FaceRegion = *r;
            fd.m_DetectionTime = time;
            fd.m_Pose = pose();
            storage.push_back(fd);
          }

        // // To test joints
        // cvp_BodyDescriptor bd;
        // bd.m_ImageWidth = image->width;
        // bd.m_ImageHeight = image->height;
        // for(int i = 0; i < faces->total; i++)
        //   {
        //     CvRect *r = ( CvRect* ) cvGetSeqElem( faces, i );
        //     bd.m_Nose.x = r->x + r->width/2;
        //     bd.m_Nose.y = r->y + r->height/2;
        //     bd.m_DetectionTime = time;
        //     mBodies.push_back(bd);
        //     std::cout << "bd.m_Nose.x " << bd.m_Nose.x
        //               << " width " << bd.m_ImageWidth
        //               << std::endl;
        //   }


    }
} // detect_faces

} // namespace OpenCvPlus
