/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#ifndef BF_CONFIDENCE_H
#define BF_CONFIDENCE_H

namespace BayesImage
{
  class bi_ObjectObservation;
}


using namespace BayesImage;

namespace BayesFilter
{

class bf_Confidence
{
  protected:
    float m_fConfidence;

  public:
    bf_Confidence() { reset(); };

    virtual void reset() { m_fConfidence=1.0; };
    virtual void update(bool face_detector_reported, bi_ObjectObservation* obs)=0;
    float getValue() { return m_fConfidence; };

    virtual ~bf_Confidence() {};
};

}

#endif
