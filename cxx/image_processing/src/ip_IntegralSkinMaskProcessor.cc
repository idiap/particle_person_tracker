// Copyright (c) 2011-2020 Idiap Research Institute
//
// ip_IntegralSkinMaskProcessor - computes integral skin mask
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// LOCAL INCLUDES
#include <image_processing/ip_IntegralSkinMaskProcessor.h> // declaration of this

using namespace std;

namespace ImageProcessing {

/////////////////////////////// PUBLIC ///////////////////////////////////////

ip_IntegralSkinMaskProcessor::ip_IntegralSkinMaskProcessor(
        ip_ImageProvider * skin_mask_provider) :
        mSkinMaskProvider(skin_mask_provider) {

    int width =  skin_mask_provider->image_buffer()->width;
    int height = skin_mask_provider->image_buffer()->height;

    IplImage * _integral_skin_mask_image = cvCreateImage(cvSize(width, height),
        IPL_DEPTH_32F, 1);
    image(_integral_skin_mask_image);
} // ip_IntegralSkinMaskProcessor

/////////////////////////////// PROTECTED ////////////////////////////////////

/* virtual */ void ip_IntegralSkinMaskProcessor::recompute_image(
        IplImage* buffer_image, const ip_RoiWindow& roi,
        boost::posix_time::ptime& time) {

    const IplImage *const skin_mask_image = mSkinMaskProvider->image(roi);
    time = mSkinMaskProvider->time();
    cvSetZero(buffer_image);

    float integrateUp;
    float integrateUpLeft;
    float integrateLeft;

    for(int row = roi.m_iFirstRow; row < roi.m_iFirstRow + roi.m_iHeight; ++row) {
        integrateLeft = 0;
        for(int column = roi.m_iFirstColumn; column < roi.m_iFirstColumn + roi.m_iWidth; ++column) {

            integrateUp = (row ? CV_IMAGE_ELEM(
                    buffer_image, float, row - 1, column) : 0);
            integrateUpLeft = (((row) && (column)) ? CV_IMAGE_ELEM(
                    buffer_image, float, row - 1, column - 1) : 0);
            integrateLeft +=
                    CV_IMAGE_ELEM(skin_mask_image, uchar, row, column) +
                    integrateUp - integrateUpLeft;
            CV_IMAGE_ELEM(buffer_image, float, row, column) = integrateLeft;
        } // Next column
    } // Next row

} // recompute_image

} // namespace ImageProcessing
