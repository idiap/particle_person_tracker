/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#include <math.h>
#include <stdio.h>
//#include <pantheios/pantheios.hpp>
//#include <pantheios/inserters.hpp>
#include "utils.h"
//#include "bf_ExponentialDistribution.h"
#include "bayes_filter/bf_ConfidenceProb.h"
#include "bayes_filter/bi_ObjectObservation.h"

//using namespace pantheios;


//static const float LHOOD_MEAN = 0.0000001; // for facetrack

#define FACETRACK_CONSTANTS
//#define FACETRACK_CS_CONSTANTS

#ifdef FACETRACK_CONSTANTS
static const float LHOOD_MEAN = 1e-7;
//static const float LHOOD_AMP = 5;
static const float LHOOD_AMP = 10;
static const float VAR_MEAN = 100;
//static const float VAR_AMP = 0.001;
static const float VAR_AMP = 0.1;
static const float TRACKMEM_MEAN = 0.58;
static const float TRACKMEM_AMP = 2.0;
static const float LHOOD_DROP_MEAN = 0.5;
static const float LHOOD_DROP_AMP = 0.91;
static const float VAR_DROP_MEAN = 150;
static const float VAR_DROP_AMP = 0.003;
static const float VAR_INC_MEAN = 100;
//static const float VAR_INC_AMP = 0.0005;
static const float VAR_INC_AMP = 0.005;
#endif

#ifdef FACETRACK_CS_CONSTANTS
static const float LHOOD_MEAN = 0.19;
static const float LHOOD_AMP = 5.0;
static const float VAR_MEAN = 2000;
static const float VAR_AMP = 0.001;
static const float TRACKMEM_MEAN = 0.1;
static const float TRACKMEM_AMP = 2.0;
static const float LHOOD_DROP_MEAN = 0.5;
static const float LHOOD_DROP_AMP = 0.91;
static const float VAR_DROP_MEAN = 150;
static const float VAR_DROP_AMP = 0.003;
static const float VAR_INC_MEAN = 100;
static const float VAR_INC_AMP = 0.0005;
#endif


namespace BayesFilter
{


bf_ConfidenceProb::bf_ConfidenceProb(int detector_freq)
{
  m_fAlpha = 0.1;
  m_iDetectorFreq = detector_freq;

  resetCum();
  m_iCumulatedCounter=0;
  m_iUpdatesWithoutDetector=0;
}

bf_ConfidenceProb::~bf_ConfidenceProb()
{
}

void
bf_ConfidenceProb::
update(bool face_detections_reported, bi_ObjectObservation* obs)
{
  /**
   * See section III.B.2 of
   *
   * [1] Duffner, Stefan, and Jean-Marc Odobez. "Track creation and
   * deletion framework for long-term online multiface tracking." IEEE
   * Transactions on image processing 22.1 (2013): 272-285.
   */

  // std::cout << "PROBABILITY " << face_detections_reported
  //           << " without " << m_iUpdatesWithoutDetector
  //           << std::endl;

  float oneoverpi = 1.0/M_PI;
  float a = 1.0*oneoverpi;
  float minvalue = 1e-6;
  float maxvalue = 1.0/minvalue;
  float vd, _vd, den;
  //float plv, pl_v;
  float like, unlike;
  if (obs->m_bUpdateDetection)
  {
    // the face detector has been called (in this iteration)
    vd = m_fFacePosterior;
    _vd = m_fNotFacePosterior;

    //////////////////////////////////////////////////////////////////////
    // Original values, independant of m_iUpdatesWithoutDetector which
    // sometimes cause tracklet not to be killed when the face
    // disappears very quickly.
    //////////////////////////////////////////////////////////////////////
    p_d_v = 0.9999;
    pd_v =  0.0001;
    p_dv =  0.88;
    pdv =   0.12;

    /*
    if (m_iUpdatesWithoutDetector>10)
    {
      p_d_v = 0.9999;
      pd_v =  0.0001;
      p_dv =  0.9;
      pdv =   0.1;
    }
    else if (m_iUpdatesWithoutDetector>2)
    {
    */
      p_d_v = 0.9999;
      pd_v =  0.0001;
      p_dv =  0.88;
      pdv =   0.12;
      /*
    }
    else
    {
      p_d_v = 0.9999;
      pd_v =  0.0001;
      p_dv =  0.85;
      pdv =   0.15;
    }
    */

    // face detection likelihood
    if (obs->m_bDetectionAssociated)
    {
      // detection has been associated with this object
      like = pdv;
      unlike = pd_v;
    }
    else
    {
      // no associated detection
      like = p_dv;
      unlike = p_d_v;
    }
    //log_DEBUG("             FD  ratio: ", pantheios::real(like/unlike));

    float plhv;
    float lhval;
    lhval = obs->m_fLikelihoodMean;
    //plhv = a*(atan(5*(lhval-0.19)))+0.5; // ORIGINAL
    plhv = a*(atan(LHOOD_AMP*(lhval-LHOOD_MEAN)))+0.5;
    plhv *= m_fLikelihoodMeanPosCum;
    plhv = powf(plhv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             LH       ratio: ", pantheios::real(plhv), "  value: ", pantheios::real(lhval));
    like *= plhv;
    unlike *= 1-plhv;

    float pvv;
    float varval;
    varval = std::max(obs->m_fCurVarianceNorm[0], obs->m_fCurVarianceNorm[1]);
    pvv = -a*(atan(VAR_AMP*(varval-VAR_MEAN)))+0.5;
    pvv *= m_fVarianceNormPosCum;
    pvv = powf(pvv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             VAR      ratio: ", pantheios::real(pvv), "  value: ", pantheios::real(varval));
    like *= pvv;
    unlike *= 1.0-pvv;

    float ptmv; //, ptm_v;
    float tmval;
    tmval = obs->m_fTrackingMemoryValue;
    ptmv = a*(atan(TRACKMEM_AMP*(tmval-TRACKMEM_MEAN)))+0.5;
    ptmv *= m_fTrackingMemoryValuePosCum;
    ptmv = powf(ptmv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             TM       ratio: ", pantheios::real(ptmv), "  value: ", pantheios::real(tmval));
    like *= ptmv;
    unlike *= 1-ptmv;

    float htmlh_val, phtmlhv;
    htmlh_val = obs->m_fHTMmax_Likelihood - obs->m_fHTM_Likelihood;
    phtmlhv = -a*(atan(LHOOD_DROP_AMP * (htmlh_val - LHOOD_DROP_MEAN)))+0.5;
    phtmlhv *= m_fHTMLLPosCum;
    phtmlhv = powf(phtmlhv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             LH  DROP ratio: ", pantheios::real(phtmlhv), "  value: ", pantheios::real(htmlh_val));
    like *= phtmlhv;
    unlike *= 1-phtmlhv;

    float htmvar_val, phtmvarv;
    htmvar_val = obs->m_fHTMmax_Variance - obs->m_fHTM_Variance;
    phtmvarv = -a*(atan(VAR_DROP_AMP*(htmvar_val-VAR_DROP_MEAN)))+0.5;
    phtmvarv *= m_fHTMVarPosCum;
    phtmvarv = powf(phtmvarv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             VAR DROP ratio: ", pantheios::real(phtmvarv), "  value: ", pantheios::real(htmvar_val));

    like *= phtmvarv;
    unlike *= 1-phtmvarv;

    float htuvar_val, phtuvarv;
    htuvar_val = obs->m_fHTU_Variance - obs->m_fHTUmin_Variance;
    phtuvarv = -a*(atan(VAR_INC_AMP*(htuvar_val-VAR_INC_MEAN)))+0.5;
    phtuvarv *= m_fHTUVarPosCum;
    phtuvarv = powf(phtuvarv, 1.0/(m_iCumulatedCounter+1));
    //log_DEBUG("             VAR INCR ratio: ", pantheios::real(phtuvarv), "  value: ", pantheios::real(htuvar_val));

    like *= phtuvarv;
    unlike *= 1-phtuvarv;

    float trans=0.99;
    float itrans = 1 - trans;

    //log_DEBUG("                 unlike: ", pantheios::real(unlike), "  like: ", pantheios::real(like), "  lh-ratio: ", pantheios::real(like/unlike));
    vd = like * (m_fFacePosterior*trans + m_fNotFacePosterior*itrans);
    _vd = unlike * (m_fNotFacePosterior*trans + m_fFacePosterior*itrans);
    den = vd + _vd;
    m_fFacePosterior = vd / den;
    m_fNotFacePosterior = _vd / den;

    m_fRatio = m_fFacePosterior / m_fNotFacePosterior;
    if (m_fRatio>maxvalue)
    {
      m_fNotFacePosterior = m_fFacePosterior/maxvalue;
      m_fRatio = m_fFacePosterior / m_fNotFacePosterior;
    }
    else if (m_fRatio<minvalue)
    {
      m_fFacePosterior = m_fNotFacePosterior*minvalue;
      m_fRatio = m_fFacePosterior / m_fNotFacePosterior;
    }
    //log_DEBUG("                 ratio: ", pantheios::real(m_fRatio));

    if (m_fRatio>100.0)
      m_fConfidence=1.0;
    else if (m_fRatio<0.0)
      m_fConfidence=0.0;
    else
      m_fConfidence=0.01*m_fRatio;

    resetCum();
  }
  else
  {
    float ptmv; //, ptm_v;
    float tmval;
    tmval = obs->m_fTrackingMemoryValue;
    ptmv = a*(atan(TRACKMEM_AMP*(tmval-TRACKMEM_MEAN)))+0.5;
    m_fTrackingMemoryValuePosCum *= ptmv;
    //m_fTrackingMemoryValueNegCum *= 1-ptmv;

    // likelihood likelihood
    float plhv, plh_v;
    float lhval = obs->m_fLikelihoodMean;
    plhv = a*(atan(LHOOD_AMP*(lhval-LHOOD_MEAN)))+0.5;
    plh_v = 1.0-plhv;
    m_fLikelihoodMeanPosCum *= plhv;
    //m_fLikelihoodMeanNegCum *= plh_v;

    // variance likelihood
    float pvv, pv_v;
    float varval = std::max(obs->m_fCurVarianceNorm[0], obs->m_fCurVarianceNorm[0]);
    pvv = -a*(atan(VAR_AMP*(varval-VAR_MEAN)))+0.5;
    pv_v = 1.0-pvv;
    m_fVarianceNormPosCum *= pvv;
    //m_fVarianceNormNegCum *= pv_v;

    float htmlh_val, phtmlhv;
    htmlh_val = obs->m_fHTMmax_Likelihood - obs->m_fHTM_Likelihood;
    phtmlhv = -a*(atan(LHOOD_DROP_AMP*(htmlh_val-LHOOD_DROP_MEAN)))+0.5;
    m_fHTMLLPosCum *= phtmlhv;
    //m_fHTMLLNegCum *= 1-phtmlhv;

    float htmvar_val, phtmvarv;
    htmvar_val = obs->m_fHTMmax_Variance - obs->m_fHTM_Variance;
    phtmvarv = -a*(atan(VAR_DROP_AMP*(htmvar_val-VAR_DROP_MEAN)))+0.5;
    m_fHTMVarPosCum *= phtmvarv;
    //m_fHTMVarNegCum *= 1-phtmvarv;

    float htuvar_val, phtuvarv;
    htuvar_val = obs->m_fHTU_Variance - obs->m_fHTUmin_Variance;
    phtuvarv = -a*(atan(VAR_INC_AMP*(htuvar_val-VAR_INC_MEAN)))+0.5;
    m_fHTUVarPosCum *= phtuvarv;
    //m_fHTUVarNegCum *= 1-phtuvarv;

    m_iCumulatedCounter++;
  }
  if (face_detections_reported)
    m_iUpdatesWithoutDetector=0;
  else
    m_iUpdatesWithoutDetector++;
}


}
