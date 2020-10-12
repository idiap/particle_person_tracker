/*
 * Copyright (c) 2010-2020 Idiap Research Institute
 *
 * See COPYING file for the complete license text.
 */

#ifndef IP_CREATE_OBJECT2_H
#define IP_CREATE_OBJECT2_H

// SYSTEM INCLUDES
#include <list>

// PROJECT INCLUDES
#include <image_processing/ip_RoiWindow.h>
#include <image_processing/ip_TrackingMemoryImage.h>    // tracking memory
#include <opencvplus/cvp_FaceDetector.h>

#include "Parameters.h"
#include "ip_Image.h"
#include "ip_ColorDef.h"

namespace ImageProcessing{


ip_BoundingBox roiWindow2BoundingBox(ip_RoiWindow rw);


class ip_TrackingMemory;

/**
 * See following article for explanation
 *
 * [1] Duffner, Stefan, and Jean-Marc Odobez. "Track creation and
 * deletion framework for long-term online multiface tracking." IEEE
 * Transactions on image processing 22.1 (2013): 272-285.
 *
 */
class ip_CreateObject2
{
  private:
    ip_Image<float>* m_pImage;
//#ifdef RESEARCH
#ifdef SIMPLE_ADD
    ip_Image<float>* m_pOldImage;
    ip_Image<float>* m_pOldOldImage;
    ip_Image<float>* m_pNewImage;
#endif

    /*
    // threshold 1
    static const float p_d_v = 0.9999;  // TODO change this
    static const float pd_v =  0.0001;
    static const float p_dv =  0.2;
    static const float pdv =   0.8;
    */
    /*
    // threshold 2
    static const float p_d_v = 0.9999;
    static const float pd_v =  0.0001;
    static const float p_dv =  0.4;
    static const float pdv =   0.6;
    */
    // threshold 3
    float p_d_v; // = 0.9999;
    float pd_v; // =  0.0001;
    float p_dv; // =  0.4;
    float pdv; // =   0.6;
    /*
    // threshold 4 experimental
    static const float p_d_v = 0.9999;
    static const float pd_v =  0.0001;
    static const float p_dv =  0.8;
    static const float pdv =   0.2;
    */
    /*
    // threshold 4    // precision slightly lower ,
    //   values for threshold 3 work better
    static const float p_d_v = 0.99999;
    static const float pd_v =  0.00001;
    static const float p_dv =  0.7;
    static const float pdv =   0.3;
    */


    ip_Image<float>* m_pDetectionsImage;
    ip_Image<float>* m_pFacePosterior;
    ip_Image<float>* m_pNotFacePosterior;
    ip_TrackingMemoryImage * m_pTrackingMemory;
    ip_BoundingBox m_ImageBB;

//#endif

    //ip_FaceDetector* m_pDetector;
    float m_rUpdateFactor;
    int m_iUpdatesWithoutDetector;

  public:
    ip_CreateObject2(ip_TrackingMemoryImage * trackmem, int width, int height, float update_factor);
    ~ip_CreateObject2();

    ip_Image<float>* getImage() { return m_pImage; };
    void init();
    //void update();
    void update(bool face_detector_reported,
        const std::list<OpenCvPlus::cvp_FaceDescriptor>& faces);
    void save(const char* filename);
    float getValue(int x, int y) { return m_pImage->value(y, x); };


};



}

#endif
