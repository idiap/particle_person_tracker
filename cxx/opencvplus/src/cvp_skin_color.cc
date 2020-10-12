/**
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_skin_color.h"

#include <cstdio>
#include <iostream>

namespace OpenCvPlus {

  // --------------------------------------------------

void cvp_ComputeSkinMask(IplImage const* ipInputImage, IplImage* opOutputImage,
          IplImage* opTempImage1, IplImage* opTempImage2,
          const cvp_SkinColourModel& skin_colour_model,
          real iDetectionThreshold, real skin_mask_value) {

    const uchar skin_mask_value_uchar = static_cast<uchar>(skin_mask_value);

    // Parameters of the 2-dimensional Gaussian distribution over skin color
    const real meanGreen = skin_colour_model.mGreenMean;
    const real meanRed = skin_colour_model.mRedMean;
    const real varianceGreen = skin_colour_model.mGreenVar;
    const real varianceRed = skin_colour_model.mRedVar;
    const real covarianceGreenRed = skin_colour_model.mGreenRedCovar;

    // Compute inverse covariance matrix of the skin color Gaussian
    const real determinant = varianceGreen * varianceRed - covarianceGreenRed *
            covarianceGreenRed;
    const real sigmaInv11 = varianceRed / determinant;
    const real sigmaInv12 = -covarianceGreenRed / determinant;
    const real sigmaInv22 = varianceGreen / determinant;

    float pixelSum;
    float green;
    float red;

    CvRect roi = cvGetImageROI(ipInputImage);

    // Compute skin mask as a thresholded Gaussian log likelihood
    cvSetZero(opTempImage1);
    for(int r = roi.y; r < roi.y + roi.height; r++) {
        for(int c = roi.x; c < roi.x + roi.width; c++) {
            pixelSum = CV_IMAGE_ELEM(ipInputImage, uchar, r, c * 3); // blue component
            green = CV_IMAGE_ELEM(ipInputImage, uchar, r, c * 3 + 1);
            red = CV_IMAGE_ELEM(ipInputImage, uchar, r, c * 3 + 2);
            if ((green >= 30) && (red >= 70)) {
                pixelSum += (green + red);
                green /= pixelSum;
                red /= pixelSum;

                // Evaluate the unnormalized log probability of the skin color
                // Gaussian (multiplied by -0.5)
                green -= meanGreen;
                red -= meanRed;
                // evaluate the mahalanobis distance between
                // the trained skin colour and the normalized pixel colour
                pixelSum = (sigmaInv11 * green + sigmaInv12 * red) * green +
                        (sigmaInv12 * green + sigmaInv22 * red) * red;
                if (pixelSum < iDetectionThreshold) {
                    CV_IMAGE_ELEM(opTempImage1, uchar, r, c) = skin_mask_value_uchar;
                }
            }
        }
    }

    // Denoise the skin mask with an erode/dilate operation. This
    // firstly erodes, to remove lonely pixels (i.e. where the skin
    // mask is active); and then dilates, to add a 1-pixel border
    // around the mask.
    cvMorphologyEx(opTempImage1, opOutputImage, opTempImage2, NULL,
            CV_MOP_OPEN, 1);
  }

  // --------------------------------------------------

  void cvp_UpdateSkinColor(cvp_SkinColourModel& iopSkinColor,
          IplImage* ipImage, int iY, int iX, int iHeight,
			   int iWidth, real iAlphaColor, real iSkinThreshold) {

    CvRect roi = cvRect(iX, iY, iWidth, iHeight);
    // Extract patch of interest.
    // NOTE: Having to use SetImageROI makes it impossible to have
    // const ipImage. Is there another way of copying the image data
    // to pInputImagePatch... is it even necessary?
    cvSetImageROI(ipImage, roi);
    IplImage* pInputImagePatch = cvCreateImage(cvGetSize(ipImage), IPL_DEPTH_8U, 3);
    cvCopy(ipImage, pInputImagePatch, NULL);
    cvResetImageROI(ipImage);

    // Compute skin color mask
    IplImage* pSkinColorMask = cvCreateImage(cvGetSize(pInputImagePatch), IPL_DEPTH_8U, 1);
    IplImage* pTempImage1 = cvCreateImage(cvGetSize(pInputImagePatch), IPL_DEPTH_8U, 1);
    IplImage* pTempImage2 = cvCreateImage(cvGetSize(pInputImagePatch), IPL_DEPTH_8U, 1);
    cvSetImageROI(pInputImagePatch, roi);
    cvp_ComputeSkinMask(pInputImagePatch, pSkinColorMask, pTempImage1,
            pTempImage2, iopSkinColor, iSkinThreshold, 1.0);
    cvResetImageROI(pInputImagePatch);

    if(true) {
      cvSaveImage("skin_adapt_cropped_face.jpg", pInputImagePatch);
      cvSaveImage("skin_adapt_mask_before.jpg", pSkinColorMask);
    }

    int firstRow = cvRound(pInputImagePatch->height/10.);
    int lastRow = cvRound((9*pInputImagePatch->height)/10.);
    int firstColumn = cvRound(pInputImagePatch->width/10.);
    int lastColumn = cvRound((9*pInputImagePatch->width)/10.);

    // Compute mean and covariance of red and green pixels for skin model
    int counter = 0; // Number of skin pixels found
    double sumGreen1 = 0; // Sum of green pixel values
    double sumGreen2 = 0; // Sum of squares of green pixel values
    double sumRed1 = 0; // Sum of red pixel values
    double sumRed2 = 0; // Sum of squares of red pixel values
    double sumGreenRed = 0; // Sum of product of red and green pixel values
    for(int row = firstRow; row < lastRow; row++)
      for(int column = firstColumn; column < lastColumn; column++)
	if(cvGet2D(pSkinColorMask, row, column).val[0] > 0) {
	  int ind = pInputImagePatch->nChannels * column;
	  real bluePixel = (real)cvp_PixVal8U(pInputImagePatch, row, ind+0);
	  real greenPixel = (real)cvp_PixVal8U(pInputImagePatch, row, ind+1);
	  real redPixel = (real)cvp_PixVal8U(pInputImagePatch, row, ind+2);
	  real pixelSum = bluePixel + greenPixel + redPixel;

	  greenPixel /= pixelSum;
	  redPixel /= pixelSum;

	  sumGreen1 += greenPixel;
	  sumGreen2 += greenPixel*greenPixel;
	  sumRed1 += redPixel;
	  sumRed2 += redPixel*redPixel;
	  sumGreenRed += greenPixel*redPixel;
	  counter++;
	}

    if(counter > 0) {
        cvp_SkinColourModel model;
        model.mGreenMean = sumGreen1/counter;
        model.mRedMean = sumRed1/counter;
        model.mGreenVar = sumGreen2/counter - model.mGreenMean * model.mGreenMean;
        model.mRedVar = sumRed2/counter - model.mRedMean * model.mRedMean;
        model.mGreenRedCovar = sumGreenRed/counter -
            model.mGreenMean * model.mRedMean;

        // Update color model mean
        //iopSkinColor = (1-iAlphaColor)*iopSkinColor + iAlphaColor*model;
        iopSkinColor.mGreenMean = (1 - iAlphaColor) * iopSkinColor.mGreenMean +
            iAlphaColor * model.mGreenMean;
        iopSkinColor.mRedMean = (1 - iAlphaColor) * iopSkinColor.mRedMean +
            iAlphaColor * model.mRedMean;
        iopSkinColor.mGreenVar = (1 - iAlphaColor) * iopSkinColor.mGreenVar +
            iAlphaColor * model.mGreenVar;
        iopSkinColor.mRedVar = (1 - iAlphaColor) * iopSkinColor.mRedVar +
            iAlphaColor * model.mRedVar;
        iopSkinColor.mGreenRedCovar =
            (1 - iAlphaColor) * iopSkinColor.mGreenRedCovar +
            iAlphaColor * model.mGreenRedCovar;

    }

    // Release memory
    cvReleaseImage(&pInputImagePatch);
    cvReleaseImage(&pSkinColorMask);
    cvReleaseImage(&pTempImage1);
    cvReleaseImage(&pTempImage2);
  }

  // --------------------------------------------------

} // namespace OpenCvPlus
