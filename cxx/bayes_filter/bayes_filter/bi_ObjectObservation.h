/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See file COPYING for the licence associated with this software.
 */

#ifndef BI_OBJECT_OBSERVATION_H
#define BI_OBJECT_OBSERVATION_H

#include "boost/date_time/posix_time/posix_time.hpp"

namespace BayesImage
{

class bi_ObjectObservation
{
public:
  float m_f1_BatDist;  /// (1 minus Batt. distance) of mean of current distribution
  float m_fLikelihoodMean;  /// likelihood of mean of current distribution
  bool m_bDetectionAssociated;
  bool m_bUpdateDetection;

  int m_iIterSinceLastDetection;
  boost::posix_time::ptime m_TimeLastDetection;

  float m_fTrackingMemoryValue;
  float m_fMotionMemoryValue;
  float m_fCurVariance[4];
  float m_fCurVarianceNorm[4];
  float m_fRACurVariance;   // running average of max normalised x/y variance
  float m_fRACurVariance2;  // running avg. of max norm. x/y variance squared
  float m_fRVCurVariance;   // running variance of max normalised x/y variance
  float m_fRALikelihoodMean;
  float m_fRALikelihoodMean2;
  float m_fRVLikelihoodMean;
  int m_iCumulatedCounter;
  float m_fAverageMotion;
  bool m_bFirstObservation;
  float m_fHTM_Variance;
  float m_fHTU_Variance;
  float m_fHTM_Likelihood;
  float m_fHTUmin_Variance;
  float m_fHTMmax_Variance;
  float m_fHTMmax_Likelihood;
#ifdef VISUAL_ACTIVITY
  float m_fAverageAverageMotion;
  float m_fAverageAverageMotionLongTerm;

  bi_ObjectObservation() {
    m_fAverageAverageMotion=0;
    m_fAverageAverageMotionLongTerm=0;
  };
#endif

  void reset()
  {
    m_iCumulatedCounter=0;
    m_f1_BatDist=0.0;
    m_fLikelihoodMean=0.0;
    m_bDetectionAssociated=0;
    m_bUpdateDetection=0;
    m_fTrackingMemoryValue=0.0;
    m_fAverageMotion=0;

    /*
      #ifdef VISUAL_ACTIVITY
      m_fAverageAverageMotion=0;
      m_fAverageAverageMotionLongTerm=0;
      #endif
    */
  };
  void resetHinckleyTestVariables()
  { m_fHTM_Variance=0;
    m_fHTMmax_Variance=-1e6;
    m_fHTU_Variance=0;
    m_fHTUmin_Variance=1e6;
    m_fHTM_Likelihood=0;
    m_fHTMmax_Likelihood=-1e6;
  };

};

}

#endif
