// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_Sparse2DMotionProcessor_KLT - class to extract sparse 2D motion vectors
//                                  using the KLT algorithm
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// PROJECT INCLUDES
#include <opencvplus/IdiapCVOpticalFlow.h>    // modified KLT

// LOCAL INCLUDES
#include <image_processing/ip_Sparse2DMotionProcessor_KLT.h> // declaration of this

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const CvSize KLT_PYR_SEARCH_WINDOW_SIZE_INIT = cvSize(3,3);
static const int KLT_PYR_TERM_CRITERIA_MAX_ITER = 20;
static const double KLT_PYR_TERM_CRITERIA_EPSILON = .3;
static const CvTermCriteria KLT_PYR_TERM_CRITERIA_INIT = cvTermCriteria(
        CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,
        KLT_PYR_TERM_CRITERIA_MAX_ITER, KLT_PYR_TERM_CRITERIA_EPSILON);
static const int KLT_PYR_MAX_LEVEL_INIT = 5;
static const double KLT_TRACK_THRESHOLD_INIT = 4;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_Sparse2DMotionProcessor_KLT::ip_Sparse2DMotionProcessor_KLT(
        ip_ImageProvider * gray_img_provider,
        ip_InterestPointDetector * int_point_detector) :
        ip_Sparse2DMotionProcessor(gray_img_provider, int_point_detector),
        m_PyrMaxLevel(KLT_PYR_MAX_LEVEL_INIT),
        m_TrackThreshold(KLT_TRACK_THRESHOLD_INIT) {

    m_PyrSearchWindowSize = KLT_PYR_SEARCH_WINDOW_SIZE_INIT;
    m_PyrTermCriteria = KLT_PYR_TERM_CRITERIA_INIT;
    IplImage * img = gray_img_provider->image_buffer();
    m_PyramidPrev = cvCreateImage(cvSize(img->width, img->height), 8, 1 );
    m_PyramidCur  = cvCreateImage(cvSize(img->width, img->height), 8, 1 );

} // ip_Sparse2DMotionProcessor_KLT

/* virtual */
ip_Sparse2DMotionProcessor_KLT::~ip_Sparse2DMotionProcessor_KLT() {
    cvReleaseImage(&m_PyramidPrev); m_PyramidPrev = 0;
    cvReleaseImage(&m_PyramidCur); m_PyramidCur = 0;
} // ~ip_Sparse2DMotionProcessor_KLT

////////////////////////////// PROTECTED /////////////////////////////////////

/* virtual */ void
ip_Sparse2DMotionProcessor_KLT::recompute_motion_points(
        const std::vector<CvPoint2D32f>& pts2dcur, IplImage * img_cur,
        const std::vector<CvPoint2D32f>& pts2dprev, IplImage * img_prev,
        ip_MotionPoints& result_pts) {

    m_FoundFeatureFlags.resize(pts2dprev.size());
    m_FoundFeatureErrors.resize(pts2dprev.size());
    m_PropagatedPoints.resize(pts2dprev.size());
    result_pts.clear();
    result_pts.reserve(pts2dprev.size());

    cvCalcOpticalFlowPyrLK(img_prev, img_cur, m_PyramidPrev, m_PyramidCur,
        &pts2dprev[0], &m_PropagatedPoints[0], pts2dprev.size(),
        m_PyrSearchWindowSize, m_PyrMaxLevel,
        &m_FoundFeatureFlags[0], &m_FoundFeatureErrors[0],
        m_PyrTermCriteria, 0);

    // fill in tracking results
     CvPoint2D32f propPt, curPt, prevPt;
     ip_MotionPoint motPt;
     for (unsigned i = 0; i < pts2dprev.size(); ++i) {
         if (!m_FoundFeatureFlags[i]) {
             continue;
         }
         prevPt = pts2dprev[i];
         propPt = m_PropagatedPoints[i];
//         motPt.m_Point = propPt;
//         motPt.m_Motion = cvPoint2D32f(propPt.x - prevPt.x, propPt.y - prevPt.y);
//         result_pts.push_back(motPt);

         for (unsigned j = 0; j < pts2dcur.size(); ++j) {
             curPt = pts2dcur[j];
             if ((curPt.x - propPt.x) * (curPt.x - propPt.x) +
                 (curPt.y - propPt.y) * (curPt.y - propPt.y) < m_TrackThreshold) {
                 if ((curPt.x - prevPt.x) * (curPt.x - prevPt.x) +
                         (curPt.y - prevPt.y) * (curPt.y - prevPt.y) < 80) {
                     motPt.m_Point = curPt;
                     motPt.m_Motion = cvPoint2D32f(
                         curPt.x - prevPt.x, curPt.y - prevPt.y);
                     result_pts.push_back(motPt);
                     break;
                 }
             }
         }
     }

} // recompute_motion_points

} // namespace ImageProcessing
