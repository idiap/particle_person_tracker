/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#ifndef BF_CONFIDENCE_PROB_H
#define BF_CONFIDENCE_PROB_H

#include "bf_Confidence.h"


namespace BayesFilter
{
class bf_ExponentialDistribution;

class bf_ConfidenceProb : public bf_Confidence
{
  private:
    int m_iDetectorFreq;
    float m_fAlpha;
    float m_fRunAvgLikelihood;
    float m_fFacePosterior;
    float m_fNotFacePosterior;
    float m_fTransitionPosterior;
    float m_fRatio;
    float m_fLikelihoodThreshold;

    int m_iCumulatedCounter;
    float m_fLikelihoodMeanPosCum, m_fLikelihoodMeanNegCum;
    float m_fTrackingMemoryValuePosCum, m_fTrackingMemoryValueNegCum;
    float m_fVarianceNormPosCum, m_fVarianceNormNegCum;
    float m_fLastFaceDetectionIterValuePosCum, m_fLastFaceDetectionIterValueNegCum;
    float m_fHTMVarPosCum, m_fHTUVarPosCum, m_fHTMLLPosCum;
    float m_fHTMVarNegCum, m_fHTUVarNegCum, m_fHTMLLNegCum;

    // threshold 2/3
    /*
    static const float p_d_v = 0.9999;
    static const float pd_v =  0.0001;
    static const float p_dv =  0.4;
    static const float pdv =   0.6;
    */
    /*
    static const float p_d_v = 0.9999;
    static const float pd_v =  0.0001;
    static const float p_dv =  0.4;
    static const float pdv =   0.6;
    */
    // threshold 4 experimental
    /*
    static const float p_d_v = 0.9999;
    static const float pd_v =  0.0001;
    static const float p_dv =  0.9;
    static const float pdv =   0.1;
    */
    float p_d_v;
    float pd_v;
    float p_dv;
    float pdv;
    /*
    // threshold 4    // precision slightly lower
    static const float p_d_v = 0.99999;
    static const float pd_v =  0.00001;
    static const float p_dv =  0.7;
    static const float pdv =   0.3;
    */
    int m_iUpdatesWithoutDetector;

  public:
    bf_ConfidenceProb(int detector_freq);
    ~bf_ConfidenceProb();

    void update(bool face_detections_reported, bi_ObjectObservation* obs);
    void reset() {
        bf_Confidence::reset();
        m_fRunAvgLikelihood=1.0;
        m_fFacePosterior=0.99;
        m_fNotFacePosterior=0.01;
        m_fRatio=m_fFacePosterior / m_fNotFacePosterior;
        m_fConfidence=0.01 * m_fRatio; };
    void resetCum() {
      m_fTrackingMemoryValuePosCum=1.0;
      m_fTrackingMemoryValueNegCum=1.0;
      m_fLikelihoodMeanPosCum=1.0;
      m_fLikelihoodMeanNegCum=1.0;
      m_fVarianceNormPosCum=1.0;
      m_fVarianceNormNegCum=1.0;
      m_fHTMVarPosCum=1.0;
      m_fHTMVarNegCum=1.0;
      m_fHTUVarPosCum=1.0;
      m_fHTUVarNegCum=1.0;
      m_fHTMLLPosCum=1.0;
      m_fHTMLLNegCum=1.0;
      m_fLastFaceDetectionIterValuePosCum=1.0;
      m_fLastFaceDetectionIterValueNegCum=1.0;
      m_iCumulatedCounter=0;
    };
};

}

#endif
