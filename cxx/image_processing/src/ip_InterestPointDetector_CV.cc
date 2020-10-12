// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_InterestPointDetector_CV - class to extract sparse 2D interest points
//                               using OpenCV
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.


// LOCAL INCLUDES
#include <image_processing/ip_InterestPointDetector_CV.h> // declaration of this

//////////////////////////// LOCAL DECLARATIONS //////////////////////////////

static const float QUALITY_INIT = 0.0001;
static const float DISTANCE_THRESHOLD_INIT = 5;
static const int BLOCK_SIZE_INIT = 3;
static const int IS_HARRIS_INIT = 1;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_InterestPointDetector_CV::ip_InterestPointDetector_CV(
        ip_ImageProvider * img_provider, unsigned count) :
        ip_InterestPointDetector(img_provider, count),
        m_PointQuality(QUALITY_INIT),
        m_DistanceThreshold(DISTANCE_THRESHOLD_INIT),
        m_BlockSize(BLOCK_SIZE_INIT),
        m_IsHarrisNotGFTT(IS_HARRIS_INIT) {

    IplImage* src_img = img_provider->image_buffer();

    int depth = IPL_DEPTH_32F;
    m_EigenImage = cvCreateImage(cvSize(src_img->width, src_img->height),
            depth, 1);
    m_TempImage  = cvCreateImage(cvSize(src_img->width, src_img->height),
            depth, 1);
} // ip_InterestPointDetector_CV

ip_InterestPointDetector_CV::~ip_InterestPointDetector_CV() {
} // ~ip_InterestPointDetector_CV

////////////////////////////// PROTECTED /////////////////////////////////////

/* virtual */ void
ip_InterestPointDetector_CV::recompute_interest_points(
        const IplImage * image, std::vector<CvPoint2D32f>& pts_storage) {

    int count = interest_points_count();
    cvGoodFeaturesToTrack(image, m_EigenImage, m_TempImage, &pts_storage[0],
        &count, m_PointQuality, m_DistanceThreshold, 0, m_BlockSize,
        m_IsHarrisNotGFTT);
    pts_storage.resize(count);

} // recompute_interest_points

} // namespace ImageProcessing
