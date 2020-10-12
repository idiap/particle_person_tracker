/**
 * Copyright (c) 2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#include "opencvplus/cvp_HistogramTemplateSimple.h"

#include <cstdio>

#include "opencvplus/cvp_Exceptions.h"

namespace OpenCvPlus {

  // --------------------------------------------------

  // Lifecycle methods

  // --------------------------------------------------

  cvp_HistogramTemplateSimple::cvp_HistogramTemplateSimple() {
    mpHistograms = NULL;
    mNumberOfBins = 0;
    mNumberOfHistograms = 0;
  }

  // --------------------------------------------------

  cvp_HistogramTemplateSimple::cvp_HistogramTemplateSimple(int iNumberOfHistograms, int iNumberOfBins) {
    mpHistograms = NULL;
    create_histogram(iNumberOfHistograms, iNumberOfBins);
  }

  // --------------------------------------------------

  cvp_HistogramTemplateSimple::~cvp_HistogramTemplateSimple() {
    delete_histogram();
  }

  // --------------------------------------------------

  void cvp_HistogramTemplateSimple::delete_histogram() {
    if(mpHistograms != NULL) {
       for(int i = 0; i < mNumberOfHistograms; i++)
	 delete [] mpHistograms[i];
       delete [] mpHistograms;
    }
  }

  // --------------------------------------------------

  void cvp_HistogramTemplateSimple::create_histogram(int iNumberOfHistograms, int iNumberOfBins) {
    delete_histogram();
    mNumberOfBins = iNumberOfBins;
    mNumberOfHistograms = iNumberOfHistograms;
    mpHistograms = new real* [mNumberOfHistograms];
    for(int i = 0; i < mNumberOfHistograms; i++)
      mpHistograms[i] = new real [mNumberOfBins];
  }

  // --------------------------------------------------

  // Getter and setter methods

  // --------------------------------------------------

  real* cvp_HistogramTemplateSimple::get_histogram_pointer(int iIndex) const {
    if((iIndex >= 0) && (iIndex < mNumberOfHistograms))
      return mpHistograms[iIndex];
    else
      throw cvp_ExceptionIndexError("Index out of range in cvp_HistogramTemplateSimple::get_histogram_pointer().");
  }

  // --------------------------------------------------

  // General methods

  // --------------------------------------------------

  cvp_HistogramTemplateSimple* cvp_HistogramTemplateSimple::create_copy() const {
    cvp_HistogramTemplateSimple *pNewHistogram;

    pNewHistogram = new cvp_HistogramTemplateSimple();
    pNewHistogram->mNumberOfHistograms = mNumberOfHistograms;
    pNewHistogram->mNumberOfBins = mNumberOfBins;
    pNewHistogram->mpHistograms = new real* [mNumberOfHistograms];
    for(int i = 0; i < mNumberOfHistograms; i++) {
      pNewHistogram->mpHistograms[i] = new real [mNumberOfBins];
      for(int j = 0; j < mNumberOfBins; j++)
	// @todo Make this faster using memcpy()
	pNewHistogram->mpHistograms[i][j] = mpHistograms[i][j];
    }
    return pNewHistogram;
  }

  // --------------------------------------------------

  bool cvp_HistogramTemplateSimple::same_template(const cvp_HistogramTemplateSimple* ipHistogramTemplate) const {
    return ((ipHistogramTemplate->mNumberOfHistograms == mNumberOfHistograms) &&
	    (ipHistogramTemplate->mNumberOfBins == mNumberOfBins));
  }

  // --------------------------------------------------

  // Input/output methods

  // --------------------------------------------------

  void cvp_HistogramTemplateSimple::debug_information(std::ostream& iOutputStream, const std::string iComments) const {
    iOutputStream << "Histogram template";
    if(iComments != "")
      iOutputStream << ":" << iComments;
    iOutputStream << "\n";

    iOutputStream << "Histograms: " << mNumberOfHistograms << ", "
		  << "Bins: " << mNumberOfBins << "\n";

    for(int i = 0; i < mNumberOfHistograms; i++) {
      iOutputStream << "Histogram #" << i << "\n";
      real* pReal = (real*)mpHistograms[i];
      real* pEnd = pReal + mNumberOfBins;
      while(pReal < pEnd)
	iOutputStream << (*pReal++) << " ";
      iOutputStream << "\n";
    }
    iOutputStream << "\n";
  }

  // --------------------------------------------------

} // namespace OpenCvPlus
