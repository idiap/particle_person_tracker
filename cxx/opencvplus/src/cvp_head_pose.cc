/**
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_head_pose.h"

#include <cstdio>
#include <string>
#include <iostream>
#include <opencvplus/cvp_HeadPose.h>

using namespace std;

namespace OpenCvPlus {

    typedef cvp_HeadPoseDiscreteDomain::HeadPose HeadPose;

    HeadPose extract_head_pose_from_filename(const char* filename);

  // --------------------------------------------------

void cvp_trainMultiScaleGradientHistograms(
    std::string iPath, cvp_HistogramTemplateSimple* const* iopTrainedMeanHistograms,
    int const* ipPoseActive, int iMaxNumberOfPoses, real iMagnitudeCap,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock) {

    int imageSize = 64; // Training input image size

    int numberOfBins = iopTrainedMeanHistograms[0]->get_number_of_bins();

    int numberOfBlocksAtScale1 = iNumberOfColumnBlocks * iNumberOfRowBlocks *
                                 iColumnsPerBlock * iRowsPerBlock;
    int numberOfBlocksAtScale2 = iNumberOfColumnBlocks * iNumberOfRowBlocks *
                                 (iColumnsPerBlock-1) * (iRowsPerBlock-1);

    if(iopTrainedMeanHistograms[0]->get_number_of_histograms() != numberOfBlocksAtScale1+numberOfBlocksAtScale2)
      throw cvp_ExceptionValueError("iopTrainedMeanHistograms has incorrect dimensions in cvp_trainMultiScaleGradientHistograms.");

    // Build map to valid pose indices (i.e. ipPoseActive[i] is true)
    int* poseIndexMap = new int [iMaxNumberOfPoses];
    int activePoseCount = 0;
    for(int i = 0; i < iMaxNumberOfPoses; i++) {
      if(ipPoseActive[i] > 0) {
    poseIndexMap[i] = activePoseCount;
    activePoseCount++;
      } else
    poseIndexMap[i] = -1;
    }

    // Variables for loading and computing gradients on training images
    // * Image as read from a training file
    IplImage* pInputImage;
    // * Temporary storage for doing image conversions
    IplImage* pTempImage8U3  = cvCreateImage(cvSize(imageSize,imageSize), IPL_DEPTH_8U, 3);
    IplImage* pTempImage8U1  = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_8U, 1);
    IplImage* pTempImage32F1 = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_32F, 1);
    // * Storage for image gradients
    IplImage* pGradientX         = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_32F, 1);
    IplImage* pGradientY         = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_32F, 1);
    IplImage* pGradientMagnitude = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_32F, 1);
    IplImage* pGradientAngle     = cvCreateImage(cvGetSize(pTempImage8U3), IPL_DEPTH_32F, 1);

    // Initialize integral image gradient histogram data structure
    vector<CvMat*> pIntegralGradientHistogram(numberOfBins);
    for (int i = 0; i < numberOfBins; i++) {
        pIntegralGradientHistogram[i] = cvCreateMat(imageSize+1, imageSize+1, CV_32FC1);
    }

    // Variables for computing multi-scale histograms for each training image
    real* pHistogramAtScale1 = new real[numberOfBins*numberOfBlocksAtScale1];
    real* pHistogramAtScale2 = new real[numberOfBins*numberOfBlocksAtScale2];
    real* pHistogramTemp = new real[numberOfBins];

    // Variables for keeping track of cumulant statistics, for computing
    // histogram means and variances across training images
    double* pHistogramCumulant0 = new double [activePoseCount]; // Number of samples observed
    double** pHistogramCumulant1 = new double* [activePoseCount]; // Sum of samples
    double** pHistogramCumulant2 = new double* [activePoseCount]; // Sum of squares of samples
    for(int j = 0; j < activePoseCount; j++) {
      pHistogramCumulant0[j] = 0;
      pHistogramCumulant1[j] = new double [numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2)];
      pHistogramCumulant2[j] = new double [numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2)];
      for(int i = 0; i < numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2); i++)
    pHistogramCumulant1[j][i] = pHistogramCumulant2[j][i] = 0;
    }

    // Open training image index file
    FILE *pIndexFile = fopen((iPath + "fake_index.txt").c_str(), "rt");
    if(pIndexFile == NULL){
      delete [] poseIndexMap;
      delete [] pHistogramAtScale1;
      delete [] pHistogramAtScale2;
      delete [] pHistogramTemp;
      delete [] pHistogramCumulant0;
      throw cvp_ExceptionIOError("Training index file not found: " + iPath + "fake_index.txt");
    }

// This directive will cause the program to write the histogram
// features for each training image to the same directory as the
// training index file. It is mostly for debugging and analysis of
// training results and should be disabled in production versions.

//#define __OUTPUT_TRAINING_SAMPLES

#ifdef __OUTPUT_TRAINING_SAMPLES
    FILE *sampleFile = fopen((iPath + "training_samples").c_str(), "wb");
    printf("%d %d %d %d\n", sizeof(int), sizeof(real),
       numberOfBins*numberOfBlocksAtScale1,
       numberOfBins*numberOfBlocksAtScale2);
#endif

    CvRect roi = cvRect(0, 0, imageSize, imageSize);
    while(true) { // Loop over index file entries
      char pFilename[1024];
      int poseIndex;
      real weight;

      // Read a line
      if(fscanf(pIndexFile, "%s %d %f", pFilename, &poseIndex, &weight) == EOF) break;

      if(ipPoseActive[poseIndex] > 0) {
    pInputImage = cvLoadImage((iPath + pFilename).c_str(), 1);
    cout << "Loading image " << iPath << pFilename << endl;
    if(!pInputImage)
      throw cvp_ExceptionIOError("Could not open training image");

    // Rescale to training size if necessary, convert to gray scale,
    // convert to floating point
    if((pInputImage->height != imageSize) || (pInputImage->width != imageSize))
      cvResize(pInputImage, pTempImage8U3, CV_INTER_CUBIC);
    else
      cvCopy(pInputImage, pTempImage8U3, NULL);
    cvCvtColor(pTempImage8U3, pTempImage8U1, CV_BGR2GRAY);
    cvConvert(pTempImage8U1, pTempImage32F1);

    // Compute multi-scale gradient histograms for this image
    cvSetImageROI(pTempImage32F1, roi);
    cvp_ComputeGradients(pTempImage32F1, pGradientX, pGradientY,
                 pGradientMagnitude, pGradientAngle, iMagnitudeCap);
    cvResetImageROI(pTempImage32F1);

    cvSaveImage("cvp_gradient_x.jpg", pGradientX);
    cvSaveImage("cvp_gradient_y.jpg", pGradientY);
    cvSaveImage("cvp_gradient_magnitude.jpg", pGradientMagnitude);
    cvSaveImage("cvp_gradient_angle.jpg", pGradientAngle);
    cvp_ComputeIntegralGradientHistogram(
          pGradientMagnitude, pGradientAngle, &pIntegralGradientHistogram[0],
          numberOfBins);
    cvp_ComputeBlockGradientHistogram(imageSize, imageSize, pHistogramAtScale1, pHistogramTemp,
                      numberOfBins, &pIntegralGradientHistogram[0],
                      iNumberOfColumnBlocks, iNumberOfRowBlocks,
                      iColumnsPerBlock, iRowsPerBlock);
    cvp_ComputeBlockGradientHistogram(imageSize, imageSize, pHistogramAtScale2, pHistogramTemp,
                      numberOfBins, &pIntegralGradientHistogram[0],
                      iNumberOfColumnBlocks, iNumberOfRowBlocks,
                      iColumnsPerBlock-1, iRowsPerBlock-1);

#ifdef __OUTPUT_TRAINING_SAMPLES
    fwrite(&poseIndex, sizeof(int), 1, sampleFile);
    fwrite(&weight, sizeof(real), 1, sampleFile);
    fwrite(pHistogramAtScale1, sizeof(real),
           numberOfBins * numberOfBlocksAtScale1, sampleFile);
    fwrite(pHistogramAtScale2, sizeof(real),
           numberOfBins * numberOfBlocksAtScale2, sampleFile);
#endif

    // Update cumulant statistics
    for(int t = 0; t < numberOfBins * numberOfBlocksAtScale1; t++) {
      real temp = pHistogramAtScale1[t];
      pHistogramCumulant1[poseIndexMap[poseIndex]][t] += temp*weight;
      pHistogramCumulant2[poseIndexMap[poseIndex]][t] += temp*temp*weight;
    }
    for(int t = 0; t < numberOfBins * numberOfBlocksAtScale2; t++) {
      real temp = pHistogramAtScale2[t];
      pHistogramCumulant1[poseIndexMap[poseIndex]][numberOfBins*numberOfBlocksAtScale1+t] += temp*weight;
      pHistogramCumulant2[poseIndexMap[poseIndex]][numberOfBins*numberOfBlocksAtScale1+t] += temp*temp*weight;
    }
    pHistogramCumulant0[poseIndexMap[poseIndex]] += weight;

    cvReleaseImage(&pInputImage);
      }
    }
    fclose(pIndexFile);

#ifdef __OUTPUT_TRAINING_SAMPLES
    fclose(sampleFile);
#endif

    // Release memory for training images
    cvReleaseImage(&pTempImage8U1);
    cvReleaseImage(&pTempImage8U3);
    cvReleaseImage(&pTempImage32F1);
    cvReleaseImage(&pGradientX);
    cvReleaseImage(&pGradientY);
    cvReleaseImage(&pGradientMagnitude);
    cvReleaseImage(&pGradientAngle);
    delete[] pHistogramAtScale1;
    delete[] pHistogramAtScale2;
    delete[] pHistogramTemp;
    for(int i = 0; i < numberOfBins; i++)
      cvReleaseMat(&pIntegralGradientHistogram[i]);

    // Compute and store mean and standard deviation for features
    real* mean = new real[numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2)];
    real* stdev = new real[numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2)];
    for(int k = 0; k < activePoseCount; k++) {
      if(pHistogramCumulant0[k] == 0) {
    // No training samples for this pose: set to uniform with
    // large variance. The overall length of the vector is
    // sqrt(2) since it is the concatenation of two normalised
    // (histogram) vectors.
    real uniform = sqrt(2./(numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2)));
    for(int i = 0; i < numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2); i++) {
      mean[i] = uniform;
      stdev[i] = 100; // This doesn't matter for now since the
              // standard deviation is not used anywhere,
              // but it should be something broad.
    }
      } else {
    // Compute and standard deviation of histogram vector
    for(int i = 0; i < numberOfBins*(numberOfBlocksAtScale1+numberOfBlocksAtScale2); i++) {
      double mu = pHistogramCumulant1[k][i] / pHistogramCumulant0[k];
      mean[i] = (real)mu;
      stdev[i] = (real)(pHistogramCumulant2[k][i] / pHistogramCumulant0[k] - mu*mu);
    }
      }

      // Reshape histogram
      real** outputHistogramData = iopTrainedMeanHistograms[k]->get_histogram_pointer_array();
      for(int i = 0; i < numberOfBlocksAtScale1+numberOfBlocksAtScale2; i++)
    for(int j = 0; j < numberOfBins; j++)
      outputHistogramData[i][j] = mean[i*numberOfBins+j];
    }

    // Release memory for compute histogram mean and variance
    delete[] poseIndexMap;
    delete[] mean;
    delete[] stdev;
    for(int i = 0; i < activePoseCount; i++) {
      delete [] pHistogramCumulant1[i];
      delete [] pHistogramCumulant2[i];
    }
    delete [] pHistogramCumulant0;
    delete [] pHistogramCumulant1;
    delete [] pHistogramCumulant2;
  }

  // --------------------------------------------------

  /// Compute a skin pixel mask across pixel blocks in an image.
  /// @param iImageHeight The height in pixels of the input image.
  /// @param iImageWidth The width in pixels of the input image.
  /// @param ipIntegralSkinMask Integral image indicating which pixels
  ///        are skin pixels.
  /// @param opBlockedSkinMask Vector of length (iNumberOfColumnBlocks *
  ///                          iNumberOfRowBlocks * iColumnsPerBlock *
  ///			       iRowsPerBlock) into which to write the
  ///                          blocked skin mask.
  /// @param iNumberOfColumnBlocks Number of large blocks in images, along x.
  /// @param iNumberOfRowBlocks Number of large blocks in images, along y.
  /// @param iColumnsPerBlock Number of small blocks per large block, along x.
  /// @param iRowsPerBlock Number of small blocks per large block, along y.
  void _compute_blocked_skin_mask(
    int iImageHeight, int iImageWidth, CvMat const* ipIntegralSkinMask, real* opBlockedSkinMask,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock) {

    int numberOfBlocks = iNumberOfColumnBlocks * iNumberOfRowBlocks * iColumnsPerBlock * iRowsPerBlock;
    int stepRow = iImageHeight / (iRowsPerBlock*iNumberOfRowBlocks);
    int stepColumn = iImageWidth / (iColumnsPerBlock*iNumberOfColumnBlocks);

    // Compute the bounding box coordinates (top, left, bottom, right)
    // of each block
    int** pBlockCoordinates = new int* [numberOfBlocks];
    for(int i = 0; i < numberOfBlocks; i++)
      pBlockCoordinates[i] = new int [4];
    int blockIndex = 0;
    for(int blockRow = 0; blockRow < iNumberOfRowBlocks; blockRow++) {
      for(int blockColumn = 0; blockColumn < iNumberOfColumnBlocks; blockColumn++) {
	for(int rowInBlock = 0; rowInBlock < iRowsPerBlock; rowInBlock++) {
	  for(int columnInBlock = 0; columnInBlock < iColumnsPerBlock; columnInBlock++) {
	    int imageRow = blockRow * iRowsPerBlock + rowInBlock;
	    int imageColumn = blockColumn * iColumnsPerBlock + columnInBlock;
	    pBlockCoordinates[blockIndex][0] = imageRow*stepRow;
	    pBlockCoordinates[blockIndex][1] = imageColumn*stepColumn;
	    pBlockCoordinates[blockIndex][2] = (imageRow+1)*stepRow;
	    pBlockCoordinates[blockIndex][3] = (imageColumn+1)*stepColumn;
	    // To be sure that we fill the input Bounding Box
	    // Note that Bounding Box on the right side and bottom might
	    // be bigger than the other ones
	    if(imageColumn == iColumnsPerBlock * iNumberOfColumnBlocks - 1)
	      pBlockCoordinates[blockIndex][3] = iImageWidth-1;
	    if(imageRow == iRowsPerBlock * iNumberOfRowBlocks - 1)
	      pBlockCoordinates[blockIndex][2] = iImageHeight-1;
	    blockIndex++;
	  }
	}
      }
    }

    // Compute the number of skin pixels in each block, and threshold at at least half being on
    for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++) {
      cvp_ComputeSumOfGradientHistograms(pBlockCoordinates[blockIndex][0], pBlockCoordinates[blockIndex][1],
					 pBlockCoordinates[blockIndex][2], pBlockCoordinates[blockIndex][3],
					 &ipIntegralSkinMask, &opBlockedSkinMask[blockIndex], 1);
      int width = pBlockCoordinates[blockIndex][2] - pBlockCoordinates[blockIndex][0];
      int height = pBlockCoordinates[blockIndex][3] - pBlockCoordinates[blockIndex][1];
      int threshold = width * height * 255 / 2;
      opBlockedSkinMask[blockIndex] = (opBlockedSkinMask[blockIndex] >= threshold) ? 1.0 : 0.0;
    }

    for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++)
      delete [] pBlockCoordinates[blockIndex];
    delete [] pBlockCoordinates;
  }

  // --------------------------------------------------

  void cvp_trainSkinMask(
    std::string iPath, const cvp_HeadPoseDiscreteDomain& head_pose_domain,
    const cvp_SkinColourModel&  ipSkinColor, float iSkinThreshold,
    int iNumberOfColumnBlocks, int iNumberOfRowBlocks, int iColumnsPerBlock, int iRowsPerBlock,
    cvp_HistogramTemplateSimple* const* opTrainedSkinMasks) {

    int imageSize = 64;

    // Variables for loading and processing training images
    IplImage* pInputImage;
    IplImage* pNormalizedInputImage = cvCreateImage(cvSize(imageSize,imageSize), IPL_DEPTH_8U, 3);
    IplImage* pSkinMask = cvCreateImage(cvSize(imageSize,imageSize),IPL_DEPTH_8U, 1);
    IplImage *pTempImage1 = cvCreateImage(cvSize(imageSize,imageSize), IPL_DEPTH_8U, 1);
    IplImage *pTempImage2 = cvCreateImage(cvSize(imageSize,imageSize), IPL_DEPTH_8U, 1);

    // Variables for storing integral and blocked skin masks
    int numberOfBlocks = iNumberOfColumnBlocks * iNumberOfRowBlocks * iColumnsPerBlock * iRowsPerBlock;
    if(numberOfBlocks != opTrainedSkinMasks[0]->get_number_of_histograms())
      throw cvp_ExceptionAssertionFailed("In compute_all_ref_histograms_color, numberOfBlocks != opTrainedSkinMasks[0]->get_number_of_histograms()");
    real* blockedSkinMask = new real[numberOfBlocks];
    CvMat* pIntegralSkinMask;
    pIntegralSkinMask = cvCreateMat(imageSize, imageSize, CV_32FC1);

    // Variables for keeping track of cumulant statistics, for
    // computing final reference skin mask
    unsigned head_pose_domain_size = head_pose_domain.size();
    double* pHistogramCumulant0 = new double [head_pose_domain_size]; // Number of samples observed
    double** pHistogramCumulant1 = new double* [head_pose_domain_size]; // Sum of samples
    for(unsigned j = 0; j < head_pose_domain_size; j++) {
      pHistogramCumulant0[j] = 0;
      pHistogramCumulant1[j] = new double [numberOfBlocks];
      for(int i = 0; i < numberOfBlocks; i++) {
          pHistogramCumulant1[j][i] = 0;
      }
    }

    // Open training image index file
    FILE *indexFile = fopen((iPath + "index.txt").c_str(), "rt");
    if(indexFile == NULL) {
        delete [] blockedSkinMask;
        delete [] pHistogramCumulant0;
        for(unsigned j = 0; j < head_pose_domain_size; j++)
          delete [] pHistogramCumulant1[j];
        delete[] pHistogramCumulant1;
        throw cvp_ExceptionIOError("Training index file not found.");
    }

    HeadPose head_pose;
    int head_pose_idx;
    CvRect roi = cvRect(0, 0, imageSize, imageSize);

    while(true) { // Loop over index file entries
      char pFilename[1024];
      int poseIndex;
      float weight;

      // Read a line
      if(fscanf(indexFile, "%s %d %f", pFilename, &poseIndex, &weight) == EOF) break;

      head_pose = extract_head_pose_from_filename(pFilename);

      if(head_pose_domain.contains(head_pose)) {
          head_pose_idx = head_pose_domain.id(head_pose);
          // Read and normalize image
          pInputImage = cvLoadImage((iPath + pFilename).c_str(), 1);
          if(!pInputImage) {
              throw cvp_ExceptionIOError("Could not open training image");
          }
          if((pInputImage->height != imageSize) || (pInputImage->width != imageSize)) {
              cvResize(pInputImage, pNormalizedInputImage, CV_INTER_CUBIC);
          } else {
              cvCopy(pInputImage, pNormalizedInputImage, NULL);
          }

          // Compute skin mask for this image
          cvSetImageROI(pNormalizedInputImage, roi);
          cvp_ComputeSkinMask(pNormalizedInputImage, pSkinMask, pTempImage1, pTempImage2,
			    ipSkinColor, iSkinThreshold, 1.0);
          cvResetImageROI(pNormalizedInputImage);

          cvp_ComputeIntegralImage(pSkinMask, pIntegralSkinMask);
          _compute_blocked_skin_mask(imageSize, imageSize, pIntegralSkinMask, blockedSkinMask,
                  iNumberOfColumnBlocks, iNumberOfRowBlocks,
				  iColumnsPerBlock, iRowsPerBlock);

          // Update cumulant statistics
          for(int blockIndex = 0; blockIndex < numberOfBlocks; blockIndex++) {
              pHistogramCumulant1[head_pose_idx][blockIndex] +=
                      blockedSkinMask[blockIndex]*weight;
          }
          pHistogramCumulant0[head_pose_idx] += weight;

          // Release image
          cvReleaseImage(&pInputImage);
      }
    }

    // Compute and store representative mask
    for(unsigned activePoseIndex = 0; activePoseIndex < head_pose_domain_size; activePoseIndex++) {
      real threshold = pHistogramCumulant0[activePoseIndex] / 2.;
      for(int i = 0; i < numberOfBlocks; i++) {
          opTrainedSkinMasks[activePoseIndex]->get_histogram_pointer_array()[i][0] = \
                  (pHistogramCumulant1[activePoseIndex][i] >= threshold) ? 1 : 0;
      }
    }

    // Release memory
    delete [] blockedSkinMask;
    cvReleaseMat(&pIntegralSkinMask);

    cvReleaseImage(&pNormalizedInputImage);
    cvReleaseImage(&pSkinMask);
    cvReleaseImage(&pTempImage1);
    cvReleaseImage(&pTempImage2);

    for(unsigned i = 0; i < head_pose_domain_size; i++) {
        delete [] pHistogramCumulant1[i];
    }
    delete [] pHistogramCumulant0;
    delete [] pHistogramCumulant1;
    fclose(indexFile);
  }

  // --------------------------------------------------

  void cvp_LoadHeadPoseModel(
    cvp_HistogramTemplateSimple* const* opGradientHistogramTemplate,
    cvp_HistogramTemplateSimple* const* opSkinMaskTemplate,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain,
    int iNumberOfGradientBins, int iNumberSplitBlockColumns,
    int iNumberSplitBlockRows, int iNumberSplitColumns, int iNumberSplitRows) {

    // Open gradient histogram file
    std::string pGradientHistogramFilename = "training_model/reference_models_texture.txt";
    FILE *pGradientHistogramFile;
    pGradientHistogramFile = fopen(pGradientHistogramFilename.c_str(), "r");
    if(!pGradientHistogramFile) {
        throw cvp_ExceptionIOError("Could not open file for reading trained gradient histogram.");
    }

    // Open skin mask file
    std::string pSkinMaskFilename = "training_model/reference_models_colors.txt";
    FILE *pSkinMaskFile;
    pSkinMaskFile = fopen(pSkinMaskFilename.c_str(), "r");
    if(!pSkinMaskFile) {
        throw cvp_ExceptionIOError("Could not open file for reading trained skin mask.");
    }

    unsigned head_pose_domain_size = head_pose_domain.size();

    // Read models
    int numberOfBlocksAtScale1 = iNumberSplitBlockColumns * iNumberSplitBlockRows * iNumberSplitColumns * iNumberSplitRows;
    int numberOfBlocksAtScale2 = iNumberSplitBlockColumns * iNumberSplitBlockRows * (iNumberSplitColumns-1) * (iNumberSplitRows-1);
    for(unsigned poseIndex = 0; poseIndex < head_pose_domain_size; poseIndex++) {
        for(int blockIndex = 0; blockIndex < numberOfBlocksAtScale1 + numberOfBlocksAtScale2; blockIndex++) {
            for(int binIndex = 0; binIndex < iNumberOfGradientBins; binIndex++) {
                if(fscanf(pGradientHistogramFile, "%f ",
                    &opGradientHistogramTemplate[poseIndex]->
                    get_histogram_pointer(blockIndex)[binIndex])<=0)
                  fprintf(stderr, "could not read gradient histogram file\n");
            }
            if(fscanf(pGradientHistogramFile, "\n")<=0)
                  fprintf(stderr, "could not read gradient histogram file\n");
        }
        if(fscanf(pGradientHistogramFile, "\n")<=0)
                  fprintf(stderr, "could not read gradient histogram file\n");
        for(int blockIndex = 0; blockIndex < numberOfBlocksAtScale1; blockIndex++) {
            if(fscanf(pSkinMaskFile, "%f ", &opSkinMaskTemplate[poseIndex]->get_histogram_pointer(blockIndex)[0])<=0)
                  fprintf(stderr, "could not read skin mask file\n");
        }
        if(fscanf(pSkinMaskFile,"\n")<=0)
                  fprintf(stderr, "could not read skin mask file\n");
    }

    fclose(pGradientHistogramFile);
    fclose(pSkinMaskFile);
  }

  // --------------------------------------------------

  void cvp_SaveHeadPoseModel(
    cvp_HistogramTemplateSimple const* const* ipGradientHistogramTemplate,
    cvp_HistogramTemplateSimple const* const* ipSkinMaskTemplate,
    const cvp_HeadPoseDiscreteDomain& head_pose_domain, int iNumberOfGradientBins,
    int iNumberSplitBlockColumns, int iNumberSplitBlockRows, int iNumberSplitColumns, int iNumberSplitRows) {

    // Open gradient histogram file
    std::string pGradientHistogramFilename = "training_model/reference_models_texture.txt";
    FILE *pGradientHistogramFile;
    pGradientHistogramFile = fopen(pGradientHistogramFilename.c_str(), "w");
    if(!pGradientHistogramFile)
      throw cvp_ExceptionIOError("Could not open file for writing trained gradient histogram.");

    // Open skin mask file
    std::string pSkinMaskFilename = "training_model/reference_models_colors.txt";
    FILE *pSkinMaskFile;
    pSkinMaskFile = fopen(pSkinMaskFilename.c_str(), "w");
    if(!pSkinMaskFile)
      throw cvp_ExceptionIOError("Could not open file for writing trained skin mask.");

    unsigned head_pose_domain_size = head_pose_domain.size();

    // Write models
    int numberOfBlocksAtScale1 = iNumberSplitBlockColumns * iNumberSplitBlockRows * iNumberSplitColumns * iNumberSplitRows;
    int numberOfBlocksAtScale2 = iNumberSplitBlockColumns * iNumberSplitBlockRows * (iNumberSplitColumns-1) * (iNumberSplitRows-1);
    for(unsigned poseIndex = 0; poseIndex < head_pose_domain_size; poseIndex++) {
        for(int blockIndex = 0; blockIndex < numberOfBlocksAtScale1 + numberOfBlocksAtScale2; blockIndex++) {
            for(int binIndex = 0; binIndex < iNumberOfGradientBins; binIndex++) {
                fprintf(pGradientHistogramFile, "%f ", ipGradientHistogramTemplate[poseIndex]->get_histogram_pointer(blockIndex)[binIndex]);
            }
            fprintf(pGradientHistogramFile, "\n");
        }
        fprintf(pGradientHistogramFile, "\n");
        for(int blockIndex = 0; blockIndex < numberOfBlocksAtScale1; blockIndex++) {
            fprintf(pSkinMaskFile, "%f ", ipSkinMaskTemplate[poseIndex]->get_histogram_pointer(blockIndex)[0]);
        }
        fprintf(pSkinMaskFile,"\n");
    }

    fclose(pGradientHistogramFile);
    fclose(pSkinMaskFile);
  }

  // --------------------------------------------------

  HeadPose extract_head_pose_from_filename(const char* filename) {
      // file name format is fixed: Personne01/personne01107-60+0.jpg
      string fname(filename);
      size_t found;

      // cut the directory name
      found = fname.find_last_of("/");
      if (found != string::npos) {
          fname = fname.substr(found + 1);
      }

      // cut file extension
      found = fname.find_first_of(".");
      if (found != string::npos) {
          fname = fname.substr(0, found);
      }

      // cut person's ID
      fname = fname.substr(13);

      // extract tilt and pan
      istringstream iss(fname);
      int pan, tilt;
      iss >> tilt >> pan;

      return HeadPose(pan, tilt, 0);
  }

} // namespace OpenCvPlus
