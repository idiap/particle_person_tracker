/**
 * Copyright (c) 2018-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_HistogramTemplate.h"

#include <cstdio>

namespace OpenCvPlus {

  // --------------------------------------------------

  // Lifecycle methods

  // --------------------------------------------------

  cvp_HistogramTemplate::cvp_HistogramTemplate() {
    mpHistograms = NULL;
    mNumberOfDimensions = 0;
    mNumberOfHistograms = 0;
  }

  // --------------------------------------------------

  cvp_HistogramTemplate::cvp_HistogramTemplate(int iNumberOfHistograms, int iNumberOfDimensions, int *ipBinsPerDimension,
					       real **ipRanges, bool iIsUniform) {
    mpHistograms = NULL;
    create_histogram(iNumberOfHistograms, iNumberOfDimensions, ipBinsPerDimension, ipRanges, iIsUniform);
  }

  // --------------------------------------------------

  cvp_HistogramTemplate::cvp_HistogramTemplate(int iNumberOfHistograms, int iNumberOfDimensions, int iBinsPerDimension,
					       real iMinRange, real iMaxRange) {
    mpHistograms = NULL;
    create_histogram_uniform(iNumberOfHistograms, iNumberOfDimensions, iBinsPerDimension, iMinRange, iMaxRange);
  }

  // --------------------------------------------------

  cvp_HistogramTemplate::~cvp_HistogramTemplate() {
    delete_histogram();
  }

  // --------------------------------------------------

  // Lifecycle support methods

  // --------------------------------------------------

  void cvp_HistogramTemplate::create_histogram(int iNumberOfHistograms, int iNumberOfDimensions, int* ipBinsPerDimension,
					       real** ipRanges, bool iIsUniform) {
    delete_histogram();
    mNumberOfHistograms = iNumberOfHistograms;
    mNumberOfDimensions = iNumberOfDimensions;
    mpHistograms = new CvHistogram* [mNumberOfHistograms];
    for(int i = 0; i < mNumberOfHistograms; i++) {
      mpHistograms[i] = cvCreateHist(mNumberOfDimensions, ipBinsPerDimension, CV_HIST_ARRAY, 0, iIsUniform?1:0);
      cvSetHistBinRanges(mpHistograms[i], ipRanges, iIsUniform?1:0);
    }
  }

  // --------------------------------------------------

  void cvp_HistogramTemplate::create_histogram_uniform(int iNumberOfHistograms, int iNumberOfDimensions,
						       int iNumberOfBins, real iMinRange, real iMaxRange) {
    real eps = 0.0001;
    int* pDimensions;
    real** pRanges;

    pDimensions = new int [iNumberOfDimensions];
    pRanges = new real* [iNumberOfDimensions];
    for(int i = 0; i < iNumberOfDimensions; i++) {
      pDimensions[i] = iNumberOfBins;
      pRanges[i] = new real [2];
      pRanges[i][0] = iMinRange;
      pRanges[i][1] = iMaxRange + eps;
    }

    create_histogram(iNumberOfHistograms, iNumberOfDimensions, pDimensions, pRanges, true);

    for(int i = 0; i < iNumberOfDimensions; i++)
      delete [] pRanges[i];
    delete [] pRanges;
    delete [] pDimensions;
  }

  // --------------------------------------------------

  void cvp_HistogramTemplate::delete_histogram() {
    if(mpHistograms != NULL) {
      for(int i = 0; i < mNumberOfHistograms; i++)
	cvReleaseHist(&mpHistograms[i]);
      delete [] mpHistograms;
    }
  }

  // --------------------------------------------------

  // Getter and setter methods

  // --------------------------------------------------

  CvHistogram* cvp_HistogramTemplate::get_histogram_pointer(int iIndex) const {
    if((iIndex >= 0) && (iIndex < mNumberOfHistograms))
      return mpHistograms[iIndex];
    else
      throw cvp_ExceptionIndexError("Histogram index out of range.");
  }

  // --------------------------------------------------

  int cvp_HistogramTemplate::get_number_of_bins(int iDimension) const {
    int pSize[CV_MAX_DIM];

    // Get number of bins along each dimension
    cvGetDims(mpHistograms[0]->bins, pSize);

    if(iDimension == -1) {
      int total = 1;
      for(int i = 0; i < mNumberOfDimensions; i++)
	total *= pSize[i];
      return total;
    } else if((iDimension >= 0) && (iDimension < mNumberOfDimensions))
      return pSize[iDimension];
    else
      throw cvp_ExceptionIndexError("Dimension index out of range.");
  }

  // --------------------------------------------------

  // General methods

  // --------------------------------------------------

  cvp_HistogramTemplate* cvp_HistogramTemplate::create_copy() const {
    cvp_HistogramTemplate* pHistogramTemplate;
    pHistogramTemplate = new cvp_HistogramTemplate();
    pHistogramTemplate->mNumberOfHistograms = mNumberOfHistograms;
    pHistogramTemplate->mpHistograms = new CvHistogram* [pHistogramTemplate->mNumberOfHistograms];
    for(int i = 0; i < pHistogramTemplate->mNumberOfHistograms; i++) {
      pHistogramTemplate->mpHistograms[i] = NULL;
      cvCopyHist(mpHistograms[i], &pHistogramTemplate->mpHistograms[i]);
    }
    pHistogramTemplate->mNumberOfDimensions = mNumberOfDimensions;
    return pHistogramTemplate;
  }

  // --------------------------------------------------

  bool cvp_HistogramTemplate::same_template(const cvp_HistogramTemplate *ipHistogramTemplate) const {
    if((ipHistogramTemplate->mNumberOfHistograms != mNumberOfHistograms) ||
       (ipHistogramTemplate->mNumberOfDimensions != mNumberOfDimensions))
      return false;
    for(int i = 0; i < mNumberOfDimensions; i++)
      if(get_number_of_bins(i) != ipHistogramTemplate->get_number_of_bins(i))
	return false;
    return true;
  }

  // --------------------------------------------------

  /*
    NOTE: This method has been removed from the class since it is
    buggy and does not seem to be used by any other classes in the
    head pose tracking project. There is another form of the update()
    method below, which is simpler and not buggy.

  // --------------------------------------------------

  void cvp_HistogramTemplate::update(cvp_HistogramTemplate* ipHistogramTemplate, real iFactor,
				     real iSlowUpdateFactor, int iNumberOfPositionsToUpdate) {
    if(!same_template(ipHistogramTemplate))
      throw cvp_ExceptionValueError("Histogram templates do not match.");

    real* pReal1 = NULL;
    real* pReal2 = NULL;
    int totalBins = get_number_of_bins();

    int updatePos = iNumberOfPositionsToUpdate;
    if(updatePos == -1)
      updatePos = mNumberOfHistograms;

    real factor = iFactor;

    for(int i = 0; i < updatePos; i++) {
      if(i >= updatePos) // TODO: This is a bug. This condition will
			 // never be true, and hence iSlowUpdateFactor
			 // remains unused. Is the for-loop over i
			 // supposed to be from 0 to
			 // mNumberOfHistograms instead?
	factor = iSlowUpdateFactor;

      cvGetRawData(mpHistograms[i]->bins, (uchar**)&pReal1);
      cvGetRawData(ipHistogramTemplate->mpHistograms[i]->bins, (uchar**)&pReal2);

      for(int j = 0; j < totalBins; j++)
	pReal1[j] = (1.0-factor)*pReal1[j] + factor*pReal2[j];
    }
  }
  */

  // --------------------------------------------------

  void cvp_HistogramTemplate::update(const cvp_HistogramTemplate* ipHistogramTemplate,
				     real iFactor, int iHistogramIndex) {
    if(!same_template(ipHistogramTemplate))
      throw cvp_ExceptionValueError("Histogram templates do not match.");

    real* pReal1 = NULL;
    real* pReal2 = NULL;
    int totalBins = get_number_of_bins();
    cvGetRawData(mpHistograms[iHistogramIndex]->bins, (uchar**)&pReal1);
    cvGetRawData(ipHistogramTemplate->mpHistograms[iHistogramIndex]->bins, (uchar**)&pReal2);
    for(int j = 0; j < totalBins; j++)
      pReal1[j] = (1.0-iFactor)*pReal1[j] + iFactor*pReal2[j];
  }

  // --------------------------------------------------

  // Input/output methods

  // --------------------------------------------------

  void cvp_HistogramTemplate::debug_information(std::ostream iOutputStream, const std::string iComments) const {
    iOutputStream << "Histogram template";
    if(iComments != "")
      iOutputStream << ":" << iComments;
    iOutputStream << "\n";

    iOutputStream << "Histograms: " << mNumberOfHistograms << ", "
		  << "Dimensions: " << mNumberOfDimensions << "\n";

    if(mpHistograms[0]->type == CV_HIST_ARRAY) {
      for(int i = 0; i < mNumberOfHistograms; i++) {
	iOutputStream << "Histogram #" << i << "\n";
	real* pReal = (real*)mpHistograms[i]->bins;
	real* pEnd = pReal + get_number_of_bins();
	while(pReal < pEnd)
	  iOutputStream << (*pReal++) << " ";
	iOutputStream << "\n";
      }
    }
    iOutputStream << "\n";
  }

  // --------------------------------------------------

} // namespace OpenCvPlus
